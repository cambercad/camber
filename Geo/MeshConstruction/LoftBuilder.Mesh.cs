using System.Collections.Generic;
using Curves;
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

        // A matched polygon side stays in the affine plane of its authored
        // section edges. Independent XYZ rounding must not turn it into a
        // corrugated surface. Solid lofts use automatic tangents here; guided
        // surface lofts follow a separate construction path.
        private static Rat3Hybrid?[][] PreservePlanarMatchedSides(
            IReadOnlyList<LoftPreparedProfile> profiles, IReadOnlyList<double> columns,
            double[] rows, int rowCount, LoftStyle style, CoordinateConverter converter,
            Rat3Hybrid[] startRim, Rat3Hybrid[] endRim)
        {
            var authored = profiles.Select(profile =>
            {
                var frame = new PreciseFrameTransform(converter, profile.System);
                return profile.Poly.Select(p => frame.Transform(new Vec3D(p.X, p.Y, 0))).ToArray();
            }).ToArray();
            int count = authored[0].Length;
            int sides = profiles[0].Closed ? count : count - 1;
            var planar = new bool[sides];
            for (int face = 0; face < sides; face++)
            {
                var points = authored.SelectMany(p => new[] { p[face], p[(face + 1) % count] }).ToArray();
                var normal = new Rat3Hybrid(0, 0, 0);
                for (int i = 1; i < points.Length && normal.IsZero(); i++)
                    for (int j = i + 1; j < points.Length && normal.IsZero(); j++)
                        normal = Rat3Hybrid.Cross(points[i] - points[0], points[j] - points[0]);
                planar[face] = !normal.IsZero() && points.All(p => Rat3Hybrid.Dot(normal, p - points[0]).Sign() == 0);
            }
            if (!planar.Any(value => value)) return null;
            BigRationalHybrid Scalar(double value)
            {
                var rational = new BigRational(value);
                return new BigRationalHybrid(rational.Numerator, rational.Denominator);
            }
            var uKnots = Enumerable.Range(0, sides + 1).Select(i => (double)i / sides).ToArray();
            var vKnots = Enumerable.Range(0, profiles.Count).Select(i => (double)i / (profiles.Count - 1)).ToArray();
            static int Span(double[] knots, double parameter)
            {
                int index = Array.BinarySearch(knots, parameter);
                return Math.Min(knots.Length - 2, index >= 0 ? index : ~index - 1);
            }
            var result = Enumerable.Range(0, rowCount).Select(_ => new Rat3Hybrid?[columns.Count]).ToArray();
            for (int column = 0; column < columns.Count; column++)
            {
                int face = Span(uKnots, columns[column]);
                var fraction = (Scalar(columns[column]) - Scalar(uKnots[face]))
                    / (Scalar(uKnots[face + 1]) - Scalar(uKnots[face]));
                int previous = face == 0 ? sides - 1 : face - 1;
                bool atPrevious = fraction.Sign() == 0 && (face > 0 || profiles[0].Closed);
                if (!planar[face] && !(atPrevious && planar[previous])) continue;
                var points = authored.Select(p => p[face] + (p[(face + 1) % count] - p[face]) * fraction).ToArray();
                Rat3Hybrid Tangent(int index) => index == 0 ? points[1] - points[0]
                    : index == points.Length - 1 ? points[^1] - points[^2]
                    : (points[index + 1] - points[index - 1]) / new BigRationalHybrid(2);
                for (int row = 0; row < rowCount; row++)
                {
                    double v = rows != null ? rows[row] : (double)row / (rowCount - 1);
                    int span = Span(vKnots, v);
                    var t = (Scalar(v) - Scalar(vKnots[span])) / (Scalar(vKnots[span + 1]) - Scalar(vKnots[span]));
                    Rat3Hybrid point;
                    if (style == LoftStyle.Ruled)
                        point = points[span] + (points[span + 1] - points[span]) * t;
                    else
                    {
                        var t2 = t * t;
                        var t3 = t2 * t;
                        var two = new BigRationalHybrid(2);
                        var three = new BigRationalHybrid(3);
                        point = points[span] * (two * t3 - three * t2 + BigRationalHybrid.One)
                            + Tangent(span) * (t3 - two * t2 + t)
                            + points[span + 1] * (-two * t3 + three * t2)
                            + Tangent(span + 1) * (t3 - t2);
                    }
                    point.Simplify();
                    result[row][column] = point;
                }
                startRim[column] = result[0][column].Value;
                endRim[column] = result[^1][column].Value;
            }
            return result;
        }

        private static void EmitGridVertices(
            Vec3D[][] grid,
            int m,
            int vRows,
            bool closedU,
            UColumnKind[] colKind,
            IReadOnlyList<double> constructionU,
            Vec2D[][] profileTu2D,
            IReadOnlyList<CoordinateSystem> systems,
            int pCount,
            int s,
            double[] rowVUniform,
            Rat3Hybrid[] startRim,
            Rat3Hybrid[] endRim,
            Rat3Hybrid?[][] planarGrid,
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
                    if (constructionU != null)
                        uu = closedU && u == m ? 1 : constructionU[g];
                    var pos = grid[v][g];
                    var exact = planarGrid?[v][g] ?? (v == 0 ? startRim[g] : v == vRows - 1 ? endRim[g] :
                        MeshConstructionHelpers.ToPrecise(converter, pos));
                    vertices.Add(converter.Convert(exact));
                    uv.Add(new Vec2D(uu, vv));
                    precise.Add(exact);
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

            // The UV seam duplicates a geometric vertex. Share its area-weighted
            // normal unless the profile explicitly has a crease there. Otherwise
            // even a circular loft acquires a visible lighting seam.
            if (closedU && colKind[0] == UColumnKind.Uniform &&
                colKind[m - 1] == UColumnKind.Uniform)
            {
                for (int v = 0; v < vRows; v++)
                {
                    int first = baseN + v * uVertCount;
                    int last = first + m;
                    Vec3D shared = normals[first] + normals[last];
                    normals[first] = shared;
                    normals[last] = shared;
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

        private static void ApplyAnalyticLoftNormals(
            Vec3D[][] profileWorld, IReadOnlyList<CurveStrip2D> strips,
            IReadOnlyList<CoordinateSystem> systems, List<double> columns,
            List<UColumnKind> kinds, Vec2D[][] creaseTangents, double[] seams,
            bool closed, LoftStyle style, double[] rowParameters,
            int rows, int baseVertex, List<Vec3D> normals, INurbsSurface constructionSupport = null)
        {
            if (constructionSupport != null)
            {
                int width = closed ? columns.Count + 1 : columns.Count;
                for (int row = 0; row < rows; row++)
                for (int column = 0; column < width; column++)
                {
                    int g = column == columns.Count ? 0 : column;
                    double u = column == columns.Count ? 1 : columns[g];
                    if (kinds[g] == UColumnKind.CreaseLeft)
                        u = Math.BitDecrement(u == 0 && closed ? 1 : u);
                    else if (kinds[g] == UColumnKind.CreaseRight)
                        u = Math.BitIncrement(u == 1 && closed ? 0 : u);
                    u = Math.Clamp(u, 0, 1);
                    double v = rowParameters != null ? rowParameters[row] : (double)row / (rows - 1);
                    var normal = constructionSupport.EvaluateNormal(u, v).Normalized();
                    int index = baseVertex + row * width + column;
                    // U/V derivatives follow the emitted quad winding. A local
                    // comparison with finite-difference crease normals can reverse
                    // isolated vertices; closed-solid orientation is applied once
                    // to the whole mesh by EnsurePositiveVolume.
                    normals[index] = normal;
                }
                return;
            }
            if (strips.Any(strip => strip == null)) return;
            int count = profileWorld.Length, m = columns.Count, stride = closed ? m + 1 : m;
            for (int u = 0; u < stride; u++)
            {
                int g = closed && u == m ? 0 : u;
                var points = new Vec3D[count];
                var derivatives = new Vec3D[count];
                for (int p = 0; p < count; p++)
                {
                    points[p] = profileWorld[p][g];
                    double authored = ToAuthoredU(columns[g], seams[p], closed);
                    Vec2D derivative = strips[p].DerivativeAtNormalizedArcLength(authored);
                    if (kinds[g] != UColumnKind.Uniform)
                        derivative = creaseTangents[p][g] * derivative.Length();
                    derivatives[p] = systems[p].DirectionTo3D(derivative);
                }
                // Default cardinal Hermite interpolation is linear in its input
                // points, so the same operator interpolates full u derivatives.
                var positionSpline = style == LoftStyle.Ruled ? null : new CubicHermiteSpline3D(points);
                var derivativeSpline = style == LoftStyle.Ruled ? null : new CubicHermiteSpline3D(derivatives);
                for (int row = 0; row < rows; row++)
                {
                    double v = rowParameters != null ? rowParameters[row] : (double)row / (rows - 1);
                    Vec3D du, dv;
                    if (style == LoftStyle.Ruled)
                    {
                        double parameter = v * (count - 1);
                        int span = Math.Min((int)Math.Floor(parameter), count - 2);
                        double fraction = parameter - span;
                        du = derivatives[span] * (1 - fraction) + derivatives[span + 1] * fraction;
                        dv = points[span + 1] - points[span];
                    }
                    else
                    {
                        du = derivativeSpline.Evaluate(v).Origin;
                        dv = positionSpline.Evaluate(v).Tangent;
                    }
                    Vec3D normal = Vec3DOps.Cross(du, dv);
                    if (normal.LengthSquared() == 0) continue;
                    normal = normal.Normalized();
                    int index = baseVertex + row * stride + u;
                    normals[index] = normal;
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
            int groupId,
            IReadOnlyList<int> columnGroups = null)
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
                        groups.Add(columnGroups == null ? groupId : columnGroups[q]);
                    }
                    if (!MeshConstructionHelpers.IsDegenerateTriangleMesh(triB, p01, p11, p10, minSquaredCrossNorm))
                    {
                        triangles.Add(triB);
                        groups.Add(columnGroups == null ? groupId : columnGroups[q]);
                    }
                }
            }
        }

        private static void EnsurePositiveVolume(List<Vec3D> vertices, List<Tri> triangles, List<Vec3D> normals)
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
            for (int i = 0; i < normals.Count; i++)
                normals[i] = -normals[i];
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
