using System;
using System.Collections.Generic;
using GeoCore;

namespace Geo
{
    public static partial class LoftBuilder
    {
        /// <summary>Removes consecutive sketch-space duplicates (e.g. crease left/right columns) for ear clipping; maps clean vertex index → original rim index.</summary>
        private static void SanitizeSketchLoopForCapTriangulation(
            Vec2D[] sketchLoop, Rat3Hybrid[] preciseRim, out List<Vec2D> cleanSketch, out List<int> cleanToOrig)
        {
            cleanSketch = new List<Vec2D>();
            cleanToOrig = new List<int>();
            int n = sketchLoop.Length;
            for (int i = 0; i < n; i++)
            {
                if (cleanSketch.Count == 0 ||
                    preciseRim[cleanToOrig[^1]] != preciseRim[i])
                {
                    cleanSketch.Add(sketchLoop[i]);
                    cleanToOrig.Add(i);
                }
            }

            while (cleanSketch.Count >= 2 &&
                   preciseRim[cleanToOrig[0]] == preciseRim[cleanToOrig[^1]])
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

        internal static void EmitPlanarCapFromSketch2D(
            Vec2D[] sketchLoop,
            Rat3Hybrid[] preciseRim,
            Vec3D capNormalUnit,
            bool flipWinding,
            int groupId,
            LoftCapTriangulationMode capMode,
            CoordinateConverter converter,
            List<Vec3D> vertices,
            List<Vec3D> normals,
            List<Vec2D> uv,
            List<Rat3Hybrid> precisePositions,
            List<Tri> triangles,
            List<int> triangleGroups)
        {
            int n = sketchLoop.Length;
            if (preciseRim == null || preciseRim.Length != n)
                throw new ArgumentException("preciseRim must match sketchLoop length.", nameof(preciseRim));
            if (n < 3)
                throw new ArgumentException("Cap requires at least three points.", nameof(sketchLoop));

            // Dedicated cap rim verts: independent UV island from minimum-area rectangle in sketch space.
            // The cap is a distinct planar face; its sharp rim retains separate normals.
            Vec2D[] rimUv = AutoUV.ComputePlanarUvFromPoints2D(sketchLoop);
            var rimIdx = new int[n];
            int capBase = vertices.Count;
            for (int i = 0; i < n; i++)
            {
                rimIdx[i] = capBase + i;
                vertices.Add(converter.Convert(preciseRim[i]));
                normals.Add(capNormalUnit);
                uv.Add(rimUv[i]);
                precisePositions.Add(preciseRim[i]);
            }

            // Duplicate removal and triangulation must agree with the side mesh.
            SanitizeSketchLoopForCapTriangulation(sketchLoop, preciseRim, out var cleanSketch, out var cleanToOrig);
            if (cleanToOrig.Count < 3)
                throw new InvalidOperationException("Loft end cap: polygon collapsed to fewer than three exact rim vertices.");

            // The side and cap share authoritative rational positions. A second
            // integer-grid rounding of sketch coordinates can turn collinear rim
            // vertices into tiny ears; dropping those ears then opens the solid.
            // Project the existing exact plane instead, without moving its rim.
            int axis = Math.Abs(capNormalUnit.X) > Math.Abs(capNormalUnit.Y)
                ? (Math.Abs(capNormalUnit.X) > Math.Abs(capNormalUnit.Z) ? 0 : 2)
                : (Math.Abs(capNormalUnit.Y) > Math.Abs(capNormalUnit.Z) ? 1 : 2);
            var projected = new List<Rat2Hybrid>(cleanToOrig.Count);
            foreach (int i in cleanToOrig)
            {
                var p = preciseRim[i];
                projected.Add(axis == 0 ? new Rat2Hybrid(p.Y, p.Z)
                    : axis == 1 ? new Rat2Hybrid(p.Z, p.X) : new Rat2Hybrid(p.X, p.Y));
            }
            var area = BigRationalHybrid.Zero;
            for (int i = 0; i < projected.Count; i++)
            {
                var a = projected[i]; var b = projected[(i + 1) % projected.Count];
                area += a.X * b.Y - a.Y * b.X;
            }
            if (area.Sign() == 0)
                throw new InvalidOperationException("Loft end cap: exact rim has zero projected area.");
            if (area.Sign() < 0)
                for (int i = 0; i < projected.Count; i++)
                    projected[i] = new Rat2Hybrid(projected[i].X, -projected[i].Y);

            bool useFanCap = IsConvexClosedPolygon2D(projected);
            if (useFanCap)
            {
                var preciseCentroid = new Rat3Hybrid(0, 0, 0);
                Vec2D sketchCentroid = new Vec2D(0, 0);
                for (int i = 0; i < n; i++)
                {
                    preciseCentroid += preciseRim[i];
                    preciseCentroid.Simplify();
                    sketchCentroid.X += sketchLoop[i].X;
                    sketchCentroid.Y += sketchLoop[i].Y;
                }
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
                    if (preciseRim[i] == preciseRim[j])
                        continue;
                    Tri newTri = flipWinding
                        ? new Tri(rimIdx[i], rimIdx[j], ci)
                        : new Tri(rimIdx[j], rimIdx[i], ci);
                    if (MeshConstructionHelpers.IsCollapsedTriangle(newTri, precisePositions))
                        continue;
                    triangles.Add(newTri);
                    triangleGroups.Add(groupId);
                }
                return;
            }

            // Strict mode preserves its authored-winding check. Robust mode
            // uses the exact projected area above, without a model-size cutoff.
            if (capMode != LoftCapTriangulationMode.Robust &&
                (PolygonSignedArea2D(cleanSketch) > 0) != !flipWinding)
                throw new InvalidOperationException("Loft end cap: polygon winding preflight failed.");
            List<Tri> capTris = Triangulator.TriangulatePolygon(projected, delaunayPostProcess: true);

            EmitCapTrisFromClean(capTris, cleanToOrig, rimIdx, flipWinding, groupId, triangles, triangleGroups);
        }

        private static void EmitCapTrisFromClean(
            List<Tri> capTris,
            List<int> cleanToOrig,
            int[] rimIdx,
            bool flipWinding,
            int groupId,
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

                // Exact ear clipping emits positive-area triangles. Their rim
                // edges are topological constraints, even when display-space
                // area is small; removing a triangle here creates a hole.

                triangles.Add(newTri);
                triangleGroups.Add(groupId);
            }
        }

        private static bool IsConvexClosedPolygon2D(IReadOnlyList<Rat2Hybrid> pts)
        {
            int sign = 0;
            for (int i = 0; i < pts.Count; i++)
            {
                var a = pts[i]; var b = pts[(i + 1) % pts.Count]; var c = pts[(i + 2) % pts.Count];
                int turn = ((b.X - a.X) * (c.Y - b.Y) - (b.Y - a.Y) * (c.X - b.X)).Sign();
                if (turn == 0) continue;
                if (sign != 0 && turn != sign) return false;
                sign = turn;
            }
            return sign != 0;
        }
    }
}
