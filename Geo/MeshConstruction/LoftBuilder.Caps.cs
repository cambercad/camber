using System;
using System.Collections.Generic;
using GeoCore;

namespace Geo
{
    public static partial class LoftBuilder
    {
        /// <summary>Sketch-space coincidence for cap rim (crease duplicate columns may differ by ~1e-15 in double).</summary>
        private static bool SketchPointsNearlyEqual(Vec2D a, Vec2D b, double sketchSpan)
        {
            double eps = Math.Max(1e-10, 1e-12 * Math.Max(1.0, sketchSpan));
            return Math.Abs(a.X - b.X) <= eps && Math.Abs(a.Y - b.Y) <= eps;
        }

        /// <summary>Removes consecutive sketch-space duplicates (e.g. crease left/right columns) for ear clipping; maps clean vertex index → original rim index.</summary>
        private static void SanitizeSketchLoopForCapTriangulation(
            Vec2D[] sketchLoop, double sketchSpan, out List<Vec2D> cleanSketch, out List<int> cleanToOrig)
        {
            cleanSketch = new List<Vec2D>();
            cleanToOrig = new List<int>();
            int n = sketchLoop.Length;
            for (int i = 0; i < n; i++)
            {
                if (cleanSketch.Count == 0 ||
                    !SketchPointsNearlyEqual(cleanSketch[^1], sketchLoop[i], sketchSpan))
                {
                    cleanSketch.Add(sketchLoop[i]);
                    cleanToOrig.Add(i);
                }
            }

            while (cleanSketch.Count >= 2 &&
                   SketchPointsNearlyEqual(cleanSketch[0], cleanSketch[^1], sketchSpan))
            {
                cleanSketch.RemoveAt(cleanSketch.Count - 1);
                cleanToOrig.RemoveAt(cleanToOrig.Count - 1);
            }
        }

        /// <summary>Twice the signed area of a simple 2D polygon (CCW positive).</summary>
        private static double PolygonSignedArea2D(IReadOnlyList<Vec2D> pts)
        {
            double a = 0;
            int n = pts.Count;
            for (int i = 0; i < n; i++)
            {
                var p = pts[i];
                var q = pts[(i + 1) % n];
                a += p.X * q.Y - p.Y * q.X;
            }
            return a;
        }

        private static bool PreflightCapPolygon(List<Vec2D> cleanSketch, double sketchSpan, bool expectPositiveSignedArea)
        {
            if (cleanSketch.Count < 3)
                return false;
            double area2 = PolygonSignedArea2D(cleanSketch);
            double scale = Math.Max(1.0, sketchSpan * sketchSpan);
            if (Math.Abs(area2) < 1e-20 * scale)
                return false;
            return expectPositiveSignedArea ? area2 > 0 : area2 < 0;
        }

        /// <summary>Copy sanitized rim and, if CW, reflect sketch Y so the loop is CCW for ear clipping.</summary>
        private static List<Vec2D> CopyCapSketchLoopCcWForEarClip(IReadOnlyList<Vec2D> cleanSketch)
        {
            var pts = new List<Vec2D>(cleanSketch.Count);
            for (int i = 0; i < cleanSketch.Count; i++)
                pts.Add(cleanSketch[i]);

            if (PolygonSignedArea2D(pts) < 0)
            {
                for (int i = 0; i < pts.Count; i++)
                {
                    Vec2D p = pts[i];
                    pts[i] = new Vec2D(p.X, -p.Y);
                }
            }

            return pts;
        }

        /// <summary><see cref="LoftCapTriangulationMode.Robust"/>: only reject degenerate caps after the same CCW align as triangulation.</summary>
        private static bool PreflightCapSketchNonDegenerateAfterCcWAlign(IReadOnlyList<Vec2D> cleanSketch, double sketchSpan)
        {
            List<Vec2D> pts = CopyCapSketchLoopCcWForEarClip(cleanSketch);
            if (pts.Count < 3)
                return false;
            double area2 = PolygonSignedArea2D(pts);
            double scale = Math.Max(1.0, sketchSpan * sketchSpan);
            return area2 > 0 && Math.Abs(area2) >= 1e-20 * scale;
        }

        private static double MaxAbsVec2SketchCoords(IReadOnlyList<Vec2D> pts)
        {
            double m = 0;
            for (int i = 0; i < pts.Count; i++)
            {
                m = Math.Max(m, Math.Abs(pts[i].X));
                m = Math.Max(m, Math.Abs(pts[i].Y));
            }
            return m;
        }

        /// <summary>
        /// CCW-align in sketch space, then triangulate via exact <see cref="Int2"/> coordinates (same ear clip + Delaunay pipeline,
        /// different arithmetic). Avoids near-degenerate <see cref="Vec2D"/> orientation tests on NACA-like caps without changing triangulation sources.
        /// </summary>
        private static List<Tri> TriangulateSimpleClosedCapInSketchSpace(List<Vec2D> cleanSketch, double sketchSpan)
        {
            List<Vec2D> pts = CopyCapSketchLoopCcWForEarClip(cleanSketch);
            double spanSafe = Math.Max(sketchSpan, 1e-15);
            double q = 1_000_000.0 / spanSafe;
            double maxAbs = MaxAbsVec2SketchCoords(pts);
            if (maxAbs > 1e-15)
                q = Math.Min(q, (int.MaxValue - 8) / maxAbs);

            var ip = new List<Int2>(pts.Count);
            for (int i = 0; i < pts.Count; i++)
            {
                Vec2D p = pts[i];
                ip.Add(new Int2((int)Math.Round(p.X * q), (int)Math.Round(p.Y * q)));
            }

            return Triangulator.TriangulatePolygon(ip, delaunayPostProcess: true);
        }

        private static void EmitPlanarCapFromSketch2D(
            Vec2D[] sketchLoop,
            Vec3D[] worldRow,
            Rat3Hybrid[] preciseRim,
            Vec3D capNormalUnit,
            bool flipWinding,
            int groupId,
            LoftCapTriangulationMode capMode,
            double minSquaredCrossNorm,
            CoordinateConverter converter,
            List<Vec3D> vertices,
            List<Vec3D> normals,
            List<Vec2D> uv,
            List<Rat3Hybrid> precisePositions,
            List<Tri> triangles,
            List<int> triangleGroups)
        {
            int n = sketchLoop.Length;
            if (worldRow == null || worldRow.Length != n)
                throw new ArgumentException("worldRow must match sketchLoop length.", nameof(worldRow));
            if (n < 3)
                throw new ArgumentException("Cap requires at least three points.", nameof(sketchLoop));

            Vec2D min = new Vec2D(double.MaxValue, double.MaxValue);
            Vec2D max = new Vec2D(double.MinValue, double.MinValue);
            for (int i = 0; i < n; i++)
            {
                Vec2D p = sketchLoop[i];
                if (p.X < min.X) min.X = p.X;
                if (p.Y < min.Y) min.Y = p.Y;
                if (p.X > max.X) max.X = p.X;
                if (p.Y > max.Y) max.Y = p.Y;
            }

            double span = Math.Max(max.X - min.X, max.Y - min.Y);

            // Dedicated cap rim verts: independent UV island from minimum-area rectangle in sketch space.
            // The cap is a distinct planar face; its sharp rim retains separate normals.
            Vec2D[] rimUv = AutoUV.ComputePlanarUvFromPoints2D(sketchLoop);
            var rimIdx = new int[n];
            int capBase = vertices.Count;
            for (int i = 0; i < n; i++)
            {
                Vec3D pos = worldRow[i];
                rimIdx[i] = capBase + i;
                vertices.Add(converter.Convert(preciseRim[i]));
                normals.Add(capNormalUnit);
                uv.Add(rimUv[i]);
                precisePositions.Add(preciseRim[i]);
            }

            SanitizeSketchLoopForCapTriangulation(sketchLoop, span, out var cleanSketch, out var cleanToOrig);
            bool robust = capMode == LoftCapTriangulationMode.Robust;
            bool expectCcW = !flipWinding;
            bool preflightOk = robust
                ? PreflightCapSketchNonDegenerateAfterCcWAlign(cleanSketch, span)
                : PreflightCapPolygon(cleanSketch, span, expectCcW);

            bool useFanCap = cleanSketch.Count >= 3 &&
                IsConvexClosedPolygon2D(cleanSketch.ToArray(), cleanSketch.Count, span);

            if (useFanCap)
            {
                Vec3D centroid = new Vec3D(0, 0, 0);
                var preciseCentroid = new Rat3Hybrid(0, 0, 0);
                Vec2D sketchCentroid = new Vec2D(0, 0);
                for (int i = 0; i < n; i++)
                {
                    centroid += worldRow[i];
                    preciseCentroid += preciseRim[i];
                    preciseCentroid.Simplify();
                    sketchCentroid.X += sketchLoop[i].X;
                    sketchCentroid.Y += sketchLoop[i].Y;
                }
                centroid = centroid * (1.0 / n);
                sketchCentroid = sketchCentroid * (1.0 / n);
                int ci = vertices.Count;
                preciseCentroid /= new BigRationalHybrid(n);
                preciseCentroid.Simplify();
                vertices.Add(converter.Convert(preciseCentroid));
                normals.Add(capNormalUnit);
                uv.Add(AutoUV.ComputePlanarUvPoint(sketchCentroid, sketchLoop));
                precisePositions.Add(preciseCentroid);

                for (int i = 0; i < n; i++)
                {
                    int j = (i + 1) % n;
                    if (SketchPointsNearlyEqual(sketchLoop[i], sketchLoop[j], span))
                        continue;
                    Tri newTri = flipWinding
                        ? new Tri(rimIdx[i], rimIdx[j], ci)
                        : new Tri(rimIdx[j], rimIdx[i], ci);
                    Vec3D pa = vertices[rimIdx[i]];
                    Vec3D pb = vertices[rimIdx[j]];
                    if (MeshConstructionHelpers.IsDegenerateTriangleMesh(newTri, pa, pb, vertices[ci], minSquaredCrossNorm))
                        continue;
                    triangles.Add(newTri);
                    triangleGroups.Add(groupId);
                }
                return;
            }

            if (cleanSketch.Count < 3)
                throw new InvalidOperationException("Loft end cap: polygon collapsed to fewer than three points after removing consecutive duplicates.");

            if (!preflightOk)
                throw new InvalidOperationException(
                    robust
                        ? "Loft end cap: degenerate polygon in sketch space after sanitize."
                        : "Loft end cap: polygon winding/area preflight failed; try LoftCapTriangulationMode.Robust or fix profile orientation.");

            List<Tri> capTris = TriangulateSimpleClosedCapInSketchSpace(cleanSketch, span);

            EmitCapTrisFromClean(capTris, cleanToOrig, rimIdx, flipWinding, groupId, vertices, minSquaredCrossNorm, triangles, triangleGroups);
        }

        private static void EmitCapTrisFromClean(
            List<Tri> capTris,
            List<int> cleanToOrig,
            int[] rimIdx,
            bool flipWinding,
            int groupId,
            IReadOnlyList<Vec3D> vertices,
            double minSquaredCrossNorm,
            List<Tri> triangles,
            List<int> triangleGroups)
        {
            foreach (var tri in capTris)
            {
                int oa = cleanToOrig[tri.A];
                int ob = cleanToOrig[tri.B];
                int oc = cleanToOrig[tri.C];
                // Ear-clip returns CCW tris. Fan caps with flipWinding=false use Tri(j,i,c) (CW on the rim)
                // so the rim opposes the side strip (i→j). Match that sense here: flipWinding=false reverses CCW.
                Tri newTri = flipWinding
                    ? new Tri(rimIdx[oa], rimIdx[ob], rimIdx[oc])
                    : new Tri(rimIdx[oa], rimIdx[oc], rimIdx[ob]);

                Vec3D pa = vertices[rimIdx[oa]];
                Vec3D pb = vertices[rimIdx[ob]];
                Vec3D pc = vertices[rimIdx[oc]];
                if (MeshConstructionHelpers.IsDegenerateTriangleMesh(newTri, pa, pb, pc, minSquaredCrossNorm))
                    continue;

                triangles.Add(newTri);
                triangleGroups.Add(groupId);
            }
        }

        private static bool IsConvexClosedPolygon2D(Vec2D[] pts, int n, double sketchSpan)
        {
            if (n < 3)
                return false;
            int sign = 0;
            for (int i = 0; i < n; i++)
            {
                int i1 = (i + 1) % n;
                int i2 = (i + 2) % n;
                var p0 = pts[i];
                var p1 = pts[i1];
                var p2 = pts[i2];
                if (SketchPointsNearlyEqual(p0, p1, sketchSpan) || SketchPointsNearlyEqual(p1, p2, sketchSpan))
                    continue;
                double cross = (p1.X - p0.X) * (p2.Y - p1.Y) - (p1.Y - p0.Y) * (p2.X - p1.X);
                double scale = Math.Max(1.0, sketchSpan * sketchSpan);
                if (Math.Abs(cross) < 1e-14 * scale)
                    continue;
                int s = cross > 0 ? 1 : -1;
                if (sign == 0)
                    sign = s;
                else if (s != sign)
                    return false;
            }
            return sign != 0;
        }
    }
}
