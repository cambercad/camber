using System.Collections.Generic;
using GeoCore;

namespace Geo
{
    public static partial class LoftBuilder
    {
        /// <summary>
        /// Scale-aware tolerances from the loft lattice: minimum squared cross norm for triangle area,
        /// and minimum squared edge length for skipping collapsed u-quads (replaces raw 1e-16 with model-relative values).
        /// </summary>
        private static void ComputeLoftMeshTolerancesFromGrid(Vec3D[][] grid, int vRows, int m,
            out double minSquaredCrossNorm, out double minSquaredEdgeLen)
        {
            double minX = double.MaxValue, minY = minX, minZ = minX;
            double maxX = double.MinValue, maxY = maxX, maxZ = maxX;
            for (int v = 0; v < vRows; v++)
            {
                for (int k = 0; k < m; k++)
                {
                    Vec3D p = grid[v][k];
                    if (p.X < minX) minX = p.X;
                    if (p.Y < minY) minY = p.Y;
                    if (p.Z < minZ) minZ = p.Z;
                    if (p.X > maxX) maxX = p.X;
                    if (p.Y > maxY) maxY = p.Y;
                    if (p.Z > maxZ) maxZ = p.Z;
                }
            }
            double dx = maxX - minX;
            double dy = maxY - minY;
            double dz = maxZ - minZ;
            double L = Math.Max(dx, Math.Max(dy, dz));
            if (L < 1e-30)
                L = 1;
            minSquaredCrossNorm = MeshConstructionHelpers.LoftMinSquaredCrossNormFromMaxExtent(L);
            double edgeEps = L * 1e-14;
            minSquaredEdgeLen = Math.Max(TolSq, edgeEps * edgeEps);
        }

        private static void EmitGridVertices(
            Vec3D[][] grid,
            int m,
            int vRows,
            bool closedU,
            UColumnKind[] colKind,
            Vec2D[][] profileTu2D,
            IReadOnlyList<CoordinateSystem> systems,
            int pCount,
            int s,
            double[] rowVUniform,
            List<Vec3D> vertices,
            List<Vec3D> normals,
            List<Vec2D> uv,
            List<Rat3Hybrid> precise,
            CoordinateConverter converter)
        {
            int uVertCount = closedU ? m + 1 : m;
            for (int v = 0; v < vRows; v++)
            {
                double vv = rowVUniform != null
                    ? rowVUniform[v]
                    : (vRows == 1 ? 0 : (double)v / (vRows - 1));
                for (int u = 0; u < uVertCount; u++)
                {
                    int g = closedU && u == m ? 0 : u;
                    double uu = m == 1 ? 0 : (closedU ? (double)u / m : (double)u / (m - 1));
                    var pos = grid[v][g];
                    vertices.Add(pos);
                    uv.Add(new Vec2D(uu, vv));
                    precise.Add(MeshConstructionHelpers.ToPrecise(converter, pos));
                }
            }

            int baseN = normals.Count;
            while (normals.Count < vertices.Count)
                normals.Add(new Vec3D(0, 0, 0));

            int uQuadCount = closedU ? m : (m - 1);
            for (int v = 0; v < vRows - 1; v++)
            {
                for (int q = 0; q < uQuadCount; q++)
                {
                    int u0 = q;
                    int u1 = q + 1;
                    int i00 = baseN + v * uVertCount + u0;
                    int i01 = baseN + v * uVertCount + u1;
                    int i10 = i00 + uVertCount;
                    int i11 = i01 + uVertCount;
                    Vec3D e1 = vertices[i01] - vertices[i00];
                    Vec3D e2 = vertices[i10] - vertices[i00];
                    Vec3D n = Vec3DOps.Cross(e1, e2);
                    Vec3D n2 = Vec3DOps.Cross(vertices[i11] - vertices[i01], vertices[i10] - vertices[i01]);
                    normals[i00] += n;
                    normals[i01] += n;
                    normals[i01] += n2;
                    normals[i10] += n;
                    normals[i10] += n2;
                    normals[i11] += n2;
                }
            }

            for (int i = baseN; i < normals.Count; i++)
                normals[i] = normals[i].Normalized();

            for (int v = 0; v < vRows; v++)
            {
                int pLo, pHi;
                double wHi;
                if (rowVUniform != null)
                    GetRowProfileBlendFromVUniform(rowVUniform[v], pCount, out pLo, out pHi, out wHi);
                else
                    GetRowProfileBlend(v, pCount, s, out pLo, out pHi, out wHi);
                int vm = v > 0 ? v - 1 : 0;
                int vp = v < vRows - 1 ? v + 1 : vRows - 1;
                for (int u = 0; u < uVertCount; u++)
                {
                    int g = closedU && u == m ? 0 : u;
                    if (g >= colKind.Length || colKind[g] == UColumnKind.Uniform)
                        continue;
                    int iVert = baseN + v * uVertCount + u;
                    Vec3D tV = vertices[baseN + vp * uVertCount + u] - vertices[baseN + vm * uVertCount + u];
                    double lenV = tV.Length();
                    if (lenV < EpsLen)
                        continue;
                    tV = tV * (1.0 / lenV);
                    Vec2D tuA = profileTu2D[pLo][g];
                    Vec2D tuB = profileTu2D[pHi][g];
                    Vec3D tuW = systems[pLo].DirectionTo3D(tuA) * (1 - wHi) + systems[pHi].DirectionTo3D(tuB) * wHi;
                    double lenU = tuW.Length();
                    if (lenU < EpsLen)
                        continue;
                    tuW = tuW * (1.0 / lenU);
                    Vec3D nPrescribed = Vec3DOps.Cross(tV, tuW);
                    double lenN = nPrescribed.Length();
                    if (lenN < EpsLen)
                        continue;
                    nPrescribed = nPrescribed * (1.0 / lenN);
                    if (Vec3DOps.Dot(nPrescribed, normals[iVert]) < 0)
                        nPrescribed = -nPrescribed;
                    normals[iVert] = nPrescribed;
                }
            }
        }

        private static int LoftVertexIndexToGridG(int vertexU, bool closedU, int m)
        {
            if (closedU && vertexU == m)
                return 0;
            return vertexU;
        }

        private static void EmitSideQuads(
            Vec3D[][] grid,
            int m,
            int vRows,
            int baseVert,
            bool closedU,
            IReadOnlyList<Vec3D> vertices,
            double minSquaredCrossNorm,
            double minSquaredEdgeLen,
            List<Tri> triangles,
            List<int> groups,
            int groupId)
        {
            int uVertCount = closedU ? m + 1 : m;
            int uQuadCount = closedU ? m : (m - 1);
            for (int v = 0; v < vRows - 1; v++)
            {
                for (int q = 0; q < uQuadCount; q++)
                {
                    int u0 = q;
                    int u1 = q + 1;
                    int g0 = LoftVertexIndexToGridG(u0, closedU, m);
                    int g1 = LoftVertexIndexToGridG(u1, closedU, m);
                    if (Vec3DOps.DistanceSquared(grid[v][g0], grid[v][g1]) <= minSquaredEdgeLen)
                        continue;
                    if (Vec3DOps.DistanceSquared(grid[v + 1][g0], grid[v + 1][g1]) <= minSquaredEdgeLen)
                        continue;

                    int i00 = baseVert + v * uVertCount + u0;
                    int i01 = baseVert + v * uVertCount + u1;
                    int i10 = i00 + uVertCount;
                    int i11 = i01 + uVertCount;
                    var triA = new Tri(i00, i01, i10);
                    var triB = new Tri(i01, i11, i10);
                    Vec3D p00 = vertices[i00];
                    Vec3D p01 = vertices[i01];
                    Vec3D p10 = vertices[i10];
                    Vec3D p11 = vertices[i11];
                    if (!MeshConstructionHelpers.IsDegenerateTriangleMesh(triA, p00, p01, p10, minSquaredCrossNorm))
                    {
                        triangles.Add(triA);
                        groups.Add(groupId);
                    }
                    if (!MeshConstructionHelpers.IsDegenerateTriangleMesh(triB, p01, p11, p10, minSquaredCrossNorm))
                    {
                        triangles.Add(triB);
                        groups.Add(groupId);
                    }
                }
            }
        }

        private static void EnsurePositiveVolume(List<Vec3D> vertices, List<Tri> triangles)
        {
            double vol6 = 0;
            foreach (var t in triangles)
            {
                var a = vertices[t.A];
                var b = vertices[t.B];
                var c = vertices[t.C];
                vol6 += Vec3DOps.Dot(a, Vec3DOps.Cross(b, c));
            }
            if (vol6 >= 0)
                return;
            for (int i = 0; i < triangles.Count; i++)
            {
                var t = triangles[i];
                triangles[i] = new Tri(t.A, t.C, t.B);
            }
        }

        private static CoordinateConverter MakeConverterFromPolylines(List<List<Vec2D>> polys, List<CoordinateSystem> systems)
        {
            double ext = 1;
            for (int i = 0; i < polys.Count; i++)
            {
                foreach (var p in polys[i])
                {
                    var w = systems[i].PointTo3D(p);
                    ext = Math.Max(ext, Math.Max(Math.Abs(w.X), Math.Max(Math.Abs(w.Y), Math.Abs(w.Z))));
                }
            }
            ext = ext * 1.5 + 1;
            return new CoordinateConverter(new Box3D(new Vec3D(-ext), new Vec3D(ext)));
        }
    }
}
