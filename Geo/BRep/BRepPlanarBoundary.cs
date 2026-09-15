using GeoCore;

namespace Geo.BRep
{
    /// <summary>
    /// Boolean-trimmed planar patches can yield adjacent boundary fragments classified as
    /// inner loops even though they are not geometrically nested inside the outer loop.
    /// </summary>
    internal static class BRepPlanarBoundary
    {
        private const double Tolerance = 1e-6;

        public static void RemoveNonNestedInnerLoops(BRepFace face)
        {
            if (face.SurfaceType != SurfaceType.Planar && !face.UsesMeshFallback)
                return;
            if (face.Loops.Count < 2)
                return;
            BRepTrimLoop outer = null;
            foreach (var loop in face.Loops)
            {
                if (loop.IsOuter)
                    outer = loop;
            }

            if (outer == null || outer.WorldPoints.Count < 4)
                return;
            if (!TryBuildPlaneBasis(outer.WorldPoints, face, out var origin, out var axisX, out var axisY))
                return;

            for (int i = face.Loops.Count - 1; i >= 0; i--)
            {
                var loop = face.Loops[i];
                if (loop.IsOuter)
                    continue;
                if (!IsNestedInside(loop, outer, origin, axisX, axisY))
                    face.Loops.RemoveAt(i);
            }
        }

        private static bool TryBuildPlaneBasis(
            IReadOnlyList<Vec3D> outer,
            BRepFace face,
            out Vec3D origin,
            out Vec3D axisX,
            out Vec3D axisY)
        {
            origin = default;
            axisX = default;
            axisY = default;

            if (face.PlaneParams != null)
            {
                origin = face.PlaneParams.Origin;
                axisX = face.PlaneParams.RefDir;
                var planeNormal = face.PlaneParams.Normal;
                axisY = Vec3DOps.Cross(planeNormal, axisX).Normalized();
                return axisX.Length() > Tolerance && axisY.Length() > Tolerance;
            }

            if (outer.Count < 3)
                return false;

            origin = outer[0];
            axisX = (outer[1] - outer[0]).Normalized();
            var newell = new Vec3D(0, 0, 0);
            for (int i = 0; i < outer.Count - 1; i++)
            {
                var a = outer[i] - origin;
                var b = outer[i + 1] - origin;
                newell.X += a.Y * b.Z - a.Z * b.Y;
                newell.Y += a.Z * b.X - a.X * b.Z;
                newell.Z += a.X * b.Y - a.Y * b.X;
            }

            if (newell.Length() < Tolerance)
                return false;

            axisY = Vec3DOps.Cross(newell.Normalized(), axisX).Normalized();
            return axisY.Length() > Tolerance;
        }

        private static bool IsNestedInside(
            BRepTrimLoop inner,
            BRepTrimLoop outer,
            Vec3D origin,
            Vec3D axisX,
            Vec3D axisY)
        {
            if (inner.WorldPoints.Count < 3)
                return false;

            var centroid = new Vec3D(0, 0, 0);
            int count = inner.WorldPoints.Count - 1;
            if (count < 2)
                count = inner.WorldPoints.Count;
            for (int i = 0; i < count; i++)
            {
                centroid.X += inner.WorldPoints[i].X;
                centroid.Y += inner.WorldPoints[i].Y;
                centroid.Z += inner.WorldPoints[i].Z;
            }
            centroid.X /= count;
            centroid.Y /= count;
            centroid.Z /= count;

            var outer2d = new List<Vec2D>(outer.WorldPoints.Count);
            for (int i = 0; i < outer.WorldPoints.Count; i++)
                outer2d.Add(Project(outer.WorldPoints[i], origin, axisX, axisY));

            var test = Project(centroid, origin, axisX, axisY);
            return PointInPolygon(test, outer2d);
        }

        private static Vec2D Project(Vec3D point, Vec3D origin, Vec3D axisX, Vec3D axisY)
        {
            var d = point - origin;
            return new Vec2D(Vec3DOps.Dot(d, axisX), Vec3DOps.Dot(d, axisY));
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
    }
}
