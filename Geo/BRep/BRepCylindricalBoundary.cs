using GeoCore;

namespace Geo.BRep
{
    /// <summary>
    /// Cylindrical side patches often have a UV seam, so mesh boundary extraction yields
    /// separate loops at v=0 and v=1. Merge into one rectangular outer loop for B-rep export.
    /// </summary>
    internal static class BRepCylindricalBoundary
    {
        private const double UvTolerance = 1e-4;

        public static void MergeSeamLoopsIfNeeded(BRepFace face)
        {
            if (face.SurfaceType != SurfaceType.Cylindrical || face.CylinderParams == null)
                return;
            if (face.Loops.Count < 2)
                return;

            if (face.Loops.Count == 2)
            {
                TryMergeTwoSeamLoops(face);
                return;
            }

            RemoveSeamArtifactLoops(face);
            RemoveNonNestedInnerLoops(face);
        }

        private static void RemoveNonNestedInnerLoops(BRepFace face)
        {
            if (face.Loops.Count < 2)
                return;

            BRepTrimLoop outer = null;
            foreach (var loop in face.Loops)
            {
                if (loop.IsOuter)
                    outer = loop;
            }

            if (outer == null || outer.UvPoints.Count < 4)
                return;

            var outerUv = UnwrapMeshUv(outer.UvPoints);
            for (int i = face.Loops.Count - 1; i >= 0; i--)
            {
                var loop = face.Loops[i];
                if (loop.IsOuter || loop.UvPoints.Count < 3)
                    continue;

                var innerUv = UnwrapMeshUv(loop.UvPoints);
                if (!UvCentroidInside(innerUv, outerUv))
                    face.Loops.RemoveAt(i);
            }
        }

        private static bool UvCentroidInside(IReadOnlyList<Vec2D> inner, IReadOnlyList<Vec2D> outer)
        {
            double cx = 0, cy = 0;
            int count = inner.Count - 1;
            if (count < 2)
                count = inner.Count;
            for (int i = 0; i < count; i++)
            {
                cx += inner[i].X;
                cy += inner[i].Y;
            }
            cx /= count;
            cy /= count;

            return PointInPolygon(new Vec2D(cx, cy), outer);
        }

        private static bool PointInPolygon(Vec2D point, IReadOnlyList<Vec2D> polygon)
        {
            bool inside = false;
            for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            {
                var pi = polygon[i];
                var pj = polygon[j];
                bool intersects = (pi.Y > point.Y) != (pj.Y > point.Y) &&
                    point.X < (pj.X - pi.X) * (point.Y - pi.Y) / (pj.Y - pi.Y + 1e-30) + pi.X;
                if (intersects)
                    inside = !inside;
            }

            return inside;
        }

        private static List<Vec2D> UnwrapMeshUv(IReadOnlyList<Vec2D> meshUv)
        {
            var result = new List<Vec2D>(meshUv.Count);
            double offset = 0;
            for (int i = 0; i < meshUv.Count; i++)
            {
                if (i > 0)
                {
                    double du = meshUv[i].X - meshUv[i - 1].X;
                    if (du < -0.5)
                        offset += 1.0;
                    else if (du > 0.5)
                        offset -= 1.0;
                }

                result.Add(new Vec2D(meshUv[i].X + offset, meshUv[i].Y));
            }

            return result;
        }

        private static void TryMergeTwoSeamLoops(BRepFace face)
        {
            if (face.Loops.Count != 2)
                return;
            if (!IsConstantVLoop(face.Loops[0], out double v0) ||
                !IsConstantVLoop(face.Loops[1], out double v1))
                return;
            if (Math.Abs(v1 - v0) < UvTolerance)
                return;

            BRepTrimLoop bottom = v0 < v1 ? face.Loops[0] : face.Loops[1];
            BRepTrimLoop top = v0 < v1 ? face.Loops[1] : face.Loops[0];
            if (!TryBuildRectangularLoop(bottom, top, out var merged))
                return;

            face.Loops.Clear();
            face.Loops.Add(merged);
        }

        /// <summary>
        /// Boolean-trimmed cylindrical sides keep real inner holes but still pick up v=0/v=1 seam loops.
        /// Drop seam artifacts when a non-seam loop already carries the trimmed boundary.
        /// </summary>
        private static void RemoveSeamArtifactLoops(BRepFace face)
        {
            bool hasNonSeamLoop = false;
            foreach (var loop in face.Loops)
            {
                if (!IsSeamLoop(loop))
                {
                    hasNonSeamLoop = true;
                    break;
                }
            }

            if (!hasNonSeamLoop)
                return;

            for (int i = face.Loops.Count - 1; i >= 0; i--)
            {
                if (IsSeamLoop(face.Loops[i]))
                    face.Loops.RemoveAt(i);
            }
        }

        private static bool IsSeamLoop(BRepTrimLoop loop)
        {
            if (loop.UvPoints == null || loop.UvPoints.Count < 2)
                return false;
            return IsBottomSeamLoop(loop) || IsTopSeamLoop(loop);
        }

        private static bool IsBottomSeamLoop(BRepTrimLoop loop)
        {
            return IsConstantVLoop(loop, out double v) && v <= UvTolerance;
        }

        private static bool IsTopSeamLoop(BRepTrimLoop loop)
        {
            return IsConstantVLoop(loop, out double v) && v >= 1.0 - UvTolerance;
        }

        /// <summary>
        /// Bore walls after a boolean often sit at mesh-v ≈ 0.04 / 0.96, not 0 / 1.
        /// </summary>
        private static bool IsConstantVLoop(BRepTrimLoop loop, out double v)
        {
            v = 0;
            if (loop.UvPoints == null || loop.UvPoints.Count < 2)
                return false;
            double min = loop.UvPoints[0].Y;
            double max = min;
            for (int i = 1; i < loop.UvPoints.Count; i++)
            {
                double y = loop.UvPoints[i].Y;
                if (y < min)
                    min = y;
                if (y > max)
                    max = y;
            }
            if (max - min > 0.05)
                return false;
            v = 0.5 * (min + max);
            return true;
        }

        private static bool TryBuildRectangularLoop(BRepTrimLoop bottom, BRepTrimLoop top, out BRepTrimLoop merged)
        {
            merged = null;
            if (bottom.WorldPoints.Count < 2 || top.WorldPoints.Count < 2)
                return false;

            var world = new List<Vec3D>();
            var uv = new List<Vec2D>();

            AppendPolyline(bottom, world, uv);

            int topAtU1 = IndexClosestU(top, 1.0);
            AppendPoint(top, topAtU1, world, uv);

            for (int i = bottom.UvPoints.Count - 2; i >= 1; i--)
            {
                int topIndex = IndexClosestU(top, bottom.UvPoints[i].X);
                AppendPoint(top, topIndex, world, uv);
            }

            int topAtU0 = IndexClosestU(top, 0.0);
            AppendPoint(top, topAtU0, world, uv);

            AppendPoint(bottom, 0, world, uv);

            if (world.Count < 4)
                return false;

            merged = new BRepTrimLoop
            {
                WorldPoints = world,
                UvPoints = uv,
                IsOuter = true
            };
            return true;
        }

        private static int IndexClosestU(BRepTrimLoop loop, double targetU)
        {
            int best = 0;
            double bestDist = double.MaxValue;
            for (int i = 0; i < loop.UvPoints.Count; i++)
            {
                double d = Math.Abs(loop.UvPoints[i].X - targetU);
                if (d < bestDist)
                {
                    bestDist = d;
                    best = i;
                }
            }

            return best;
        }

        private static void AppendPolyline(BRepTrimLoop loop, List<Vec3D> world, List<Vec2D> uv)
        {
            for (int i = 0; i < loop.WorldPoints.Count; i++)
                AppendPoint(loop, i, world, uv);
        }

        private static void AppendPoint(BRepTrimLoop loop, int index, List<Vec3D> world, List<Vec2D> uv)
        {
            var p = loop.WorldPoints[index];
            var t = loop.UvPoints[index];
            if (world.Count > 0)
            {
                var last = world[world.Count - 1];
                if ((p - last).Length() < 1e-9)
                    return;
            }

            world.Add(p);
            uv.Add(t);
        }
    }
}
