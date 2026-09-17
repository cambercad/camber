using System;
using System.Collections.Generic;
using Curves;
using GeoCore;

namespace Geo
{
    public static partial class LoftBuilder
    {
        private static void ResampleProfileColumns(
            List<Vec2D> poly,
            List<Vec2D> norms,
            bool closed,
            CoordinateSystem cs,
            List<double> uColumns,
            List<UColumnKind> colKinds,
            List<int> creaseIdx,
            double tolU,
            CurveStrip2D analyticStrip,
            double seamU0,
            Vec3D[] worldRow,
            Vec2D[] sketchRow,
            Vec2D[] tu2dRow,
            bool matchingVertices = false, NURBS.BSplineCurve matchedCurve = null)
        {
            CurveStrip2D strip;
            if (analyticStrip != null)
                strip = analyticStrip;
            else
                strip = CurveStrip2D.FromTessellatedPolyline(poly, norms, closed);

            double total = strip.TotalLength;
            if (total < EpsLen)
                throw new InvalidOperationException("Degenerate loft profile (zero length).");

            double[] distBuf = strip.HasPolylineSamplingData
                ? ToDoubleArray(strip.PolylineOpenChainDistances!)
                : LineStrip2D.BuildDistanceBuffer(poly is List<Vec2D> l ? l : new List<Vec2D>(poly));

            var creaseSet = new HashSet<int>(creaseIdx ?? new List<int>());

            for (int k = 0; k < uColumns.Count; k++)
            {
                if (matchedCurve != null)
                {
                    double u = uColumns[k];
                    var point = matchedCurve.EvaluateUniform(u);
                    double sideU = colKinds[k] == UColumnKind.CreaseLeft ? Math.BitDecrement(u == 0 && closed ? 1 : u) :
                        colKinds[k] == UColumnKind.CreaseRight ? Math.BitIncrement(u == 1 && closed ? 0 : u) : u;
                    var tangent = matchedCurve.EvaluateUniformDU(Math.Clamp(sideU, 0, 1));
                    var local = point - cs.Origin;
                    sketchRow[k] = new Vec2D(Vec3DOps.Dot(local, cs.X), Vec3DOps.Dot(local, cs.Y));
                    worldRow[k] = point;
                    tu2dRow[k] = new Vec2D(Vec3DOps.Dot(tangent, cs.X), Vec3DOps.Dot(tangent, cs.Y)).Normalized();
                    continue;
                }
                if (matchingVertices)
                {
                    int spans = closed ? poly.Count : poly.Count - 1;
                    double parameter = uColumns[k] * spans;
                    int segment = Math.Min((int)Math.Floor(parameter), spans - 1);
                    double fraction = parameter - segment;
                    var start = poly[segment];
                    var end = poly[(segment + 1) % poly.Count];
                    var point = start * (1 - fraction) + end * fraction;
                    sketchRow[k] = point;
                    worldRow[k] = cs.PointTo3D(point);
                    tu2dRow[k] = (end - start).Normalized();
                    continue;
                }
                double uSeam = uColumns[k];
                double uAuth = ToAuthoredU(uSeam, seamU0, closed);
                strip.EvaluatePositionAndNormalAtNormalizedArcLength(uAuth, out Vec2D p2, out _);
                sketchRow[k] = p2;
                worldRow[k] = cs.PointTo3D(p2);

                var kind = colKinds[k];
                if (!TryTangentForCreaseColumn(poly, closed, distBuf, total, creaseSet, uAuth, kind, tolU, out Vec2D tCrease))
                {
                    if (analyticStrip != null)
                        tCrease = analyticStrip.TangentUnitAtNormalizedArcLength(uAuth);
                    else
                        tCrease = TangentCentralAlongStrip(strip, uAuth * total, total);
                }
                tu2dRow[k] = tCrease;
            }
        }

        private static double[] ToDoubleArray(IReadOnlyList<double> src)
        {
            var a = new double[src.Count];
            for (int i = 0; i < src.Count; i++)
                a[i] = src[i];
            return a;
        }

        private static bool TryTangentForCreaseColumn(
            List<Vec2D> poly,
            bool closed,
            double[] distBuf,
            double total,
            HashSet<int> creaseVert,
            double uColumn,
            UColumnKind kind,
            double tolU,
            out Vec2D tangent)
        {
            tangent = default;
            if (kind == UColumnKind.Uniform)
                return false;
            int n = poly.Count;
            foreach (int vidx in creaseVert)
            {
                if (vidx < 0 || vidx >= n)
                    continue;
                double uV = distBuf[vidx] / total;
                if (Math.Abs(uV - uColumn) > tolU * 10 && Math.Abs(uV - uColumn) > 1e-8)
                    continue;
                if (kind == UColumnKind.CreaseLeft)
                {
                    int ip = closed ? (vidx - 1 + n) % n : vidx - 1;
                    if (ip < 0)
                        return false;
                    var d = poly[vidx] - poly[ip];
                    double len = Math.Sqrt(d.X * d.X + d.Y * d.Y);
                    if (len < EpsLen)
                        return false;
                    tangent = new Vec2D(d.X / len, d.Y / len);
                    return true;
                }
                int @in = closed ? (vidx + 1) % n : vidx + 1;
                if (@in >= n)
                    return false;
                var d2 = poly[@in] - poly[vidx];
                double len2 = Math.Sqrt(d2.X * d2.X + d2.Y * d2.Y);
                if (len2 < EpsLen)
                    return false;
                tangent = new Vec2D(d2.X / len2, d2.Y / len2);
                return true;
            }
            return false;
        }

        private static Vec2D TangentCentralAlongStrip(CurveStrip2D strip, double dist, double total)
        {
            double eps = Math.Max(total * 1e-10, 1e-9);
            double da = Math.Max(0, dist - eps);
            double db = Math.Min(total, dist + eps);
            if (db <= da + 1e-15)
                db = Math.Min(total, da + eps);
            strip.EvaluatePositionAndNormalAtNormalizedArcLength(da / total, out var pa, out _);
            strip.EvaluatePositionAndNormalAtNormalizedArcLength(db / total, out var pb, out _);
            var t = pb - pa;
            double len = Math.Sqrt(t.X * t.X + t.Y * t.Y);
            if (len < EpsLen)
                return new Vec2D(1, 0);
            return new Vec2D(t.X / len, t.Y / len);
        }

        private static void SnapCreaseDuplicateColumnsInGrid(
            Vec3D[][] grid,
            int vRows,
            List<double> uColumns,
            List<UColumnKind> colKinds,
            double tolU)
        {
            int m = uColumns.Count;
            if (colKinds == null || colKinds.Count != m)
                return;
            for (int k = 0; k + 1 < m; k++)
            {
                if (Math.Abs(uColumns[k] - uColumns[k + 1]) > tolU)
                    continue;
                if (colKinds[k] != UColumnKind.CreaseLeft || colKinds[k + 1] != UColumnKind.CreaseRight)
                    continue;
                for (int row = 0; row < vRows; row++)
                    grid[row][k + 1] = grid[row][k];
            }
        }

        /// <summary>
        /// Hermite loft in v: merge <see cref="CubicHermiteSpline3D.Tessellate(double)"/> parameters from every u column so each
        /// longitudinal spline respects <paramref name="maxDeviation"/>; optionally union uniform v samples from <paramref name="s"/>.
        /// </summary>
        private static void BuildHermiteLoftGridAdaptive(
            Vec3D[][] profileWorld,
            double maxDeviation,
            int s,
            LoftStyle style,
            IReadOnlyList<double> surfaceRows,
            out Vec3D[][] grid,
            out double[] rowVUniform)
        {
            int pCount = profileWorld.Length;
            int m = profileWorld[0].Length;
            var splines = new CubicHermiteSpline3D[m];
            for (int k = 0; k < m; k++)
            {
                var pts = new List<Vec3D>(pCount);
                for (int p = 0; p < pCount; p++)
                    pts.Add(profileWorld[p][k]);
                splines[k] = new CubicHermiteSpline3D(pts);
            }

            double tolMerge = surfaceRows == null ? Math.Max(1e-12, maxDeviation * 1e-9) : 0;
            var merged = surfaceRows == null ? new List<double>() : new List<double>(surfaceRows);
            for (int j = 0; j < pCount; j++)
                merged.Add(pCount > 1 ? (double)j / (pCount - 1) : 0);

            for (int k = 0; k < m && style != LoftStyle.Ruled && surfaceRows == null; k++)
            {
                foreach (var vx in splines[k].Tessellate(maxDeviation))
                    merged.Add(vx.Uniform);
            }

            if (s > 0 && pCount > 1)
            {
                for (int p = 1; p < pCount; p++)
                {
                    for (int t = 1; t <= s; t++)
                    {
                        double w = t / (double)(s + 1);
                        merged.Add((p - 1 + w) / (pCount - 1));
                    }
                }
            }

            merged.Sort();
            var dedup = new List<double>();
            foreach (double u in merged)
            {
                if (dedup.Count == 0 || Math.Abs(u - dedup[^1]) > tolMerge)
                    dedup.Add(Math.Clamp(u, 0, 1));
            }

            if (dedup.Count == 0)
            {
                dedup.Add(0);
                dedup.Add(1);
            }
            else
            {
                if (dedup[0] > tolMerge)
                    dedup.Insert(0, 0);
                else
                    dedup[0] = 0;
                if (dedup[^1] < 1 - tolMerge)
                    dedup.Add(1);
                else
                    dedup[^1] = 1;
            }

            int vRows = dedup.Count;
            grid = new Vec3D[vRows][];
            rowVUniform = dedup.ToArray();
            for (int r = 0; r < vRows; r++)
            {
                grid[r] = new Vec3D[m];
                double vu = rowVUniform[r];
                for (int k = 0; k < m; k++)
                    if (style == LoftStyle.Ruled)
                    {
                        double spanParameter = vu * (pCount - 1);
                        int span = Math.Min((int)spanParameter, pCount - 2);
                        double fraction = spanParameter - span;
                        grid[r][k] = profileWorld[span][k] * (1 - fraction) + profileWorld[span + 1][k] * fraction;
                    }
                    else
                        grid[r][k] = splines[k].Evaluate(vu).Origin;
            }
        }

        /// <summary>Maps global Hermite parameter u ∈ [0,1] to profile indices for crease tangent blending.</summary>
        private static void GetRowProfileBlendFromVUniform(double vU, int pCount, out int pLo, out int pHi, out double wHi)
        {
            if (pCount <= 1)
            {
                pLo = pHi = 0;
                wHi = 0;
                return;
            }

            if (vU <= 0)
            {
                pLo = pHi = 0;
                wHi = 0;
                return;
            }

            if (vU >= 1)
            {
                pLo = pHi = pCount - 1;
                wHi = 0;
                return;
            }

            double fu = vU * (pCount - 1);
            int seg = (int)Math.Floor(fu + 1e-14);
            if (seg >= pCount - 1)
            {
                pLo = pCount - 2;
                pHi = pCount - 1;
                wHi = fu - pLo;
            }
            else
            {
                pLo = seg;
                pHi = seg + 1;
                wHi = fu - seg;
            }
        }

        private static void BuildLoftGrid(Vec3D[][] profileWorld, LoftStyle style, int s, Vec3D[][] grid)
        {
            int pCount = profileWorld.Length;
            int m = profileWorld[0].Length;
            int vRows = grid.Length;

            if (style == LoftStyle.Hermite)
            {
                var splines = new CubicHermiteSpline3D[m];
                for (int k = 0; k < m; k++)
                {
                    var pts = new List<Vec3D>(pCount);
                    for (int p = 0; p < pCount; p++)
                        pts.Add(profileWorld[p][k]);
                    splines[k] = new CubicHermiteSpline3D(pts);
                }
                int row = 0;
                for (int p = 0; p < pCount; p++)
                {
                    if (p > 0)
                    {
                        for (int t = 1; t <= s; t++)
                        {
                            double w = t / (double)(s + 1);
                            double vParam = pCount > 1 ? (p - 1 + w) / (pCount - 1) : 0;
                            for (int k = 0; k < m; k++)
                                grid[row][k] = splines[k].Evaluate(vParam).Origin;
                            row++;
                        }
                    }
                    for (int k = 0; k < m; k++)
                        grid[row][k] = profileWorld[p][k];
                    row++;
                }
                return;
            }

            if (style == LoftStyle.Ruled)
            {
                int row = 0;
                for (int p = 0; p < pCount; p++)
                {
                    if (p > 0)
                    {
                        for (int t = 1; t <= s; t++)
                        {
                            double w = t / (double)(s + 1);
                            for (int k = 0; k < m; k++)
                                grid[row][k] = profileWorld[p - 1][k] * (1 - w) + profileWorld[p][k] * w;
                            row++;
                        }
                    }
                    for (int k = 0; k < m; k++)
                        grid[row][k] = profileWorld[p][k];
                    row++;
                }
                return;
            }

            int rowIdx = 0;
            for (int p = 0; p < pCount; p++)
            {
                if (p > 0)
                {
                    for (int t = 1; t <= s; t++)
                    {
                        double w = t / (double)(s + 1);
                        for (int k = 0; k < m; k++)
                        {
                            Vec3D pm1 = p >= 2 ? profileWorld[p - 2][k] : profileWorld[p - 1][k] * 2 - profileWorld[p][k];
                            Vec3D p0 = profileWorld[p - 1][k];
                            Vec3D p1 = profileWorld[p][k];
                            Vec3D p2 = p + 1 < pCount ? profileWorld[p + 1][k] : profileWorld[p][k] * 2 - profileWorld[p - 1][k];
                            grid[rowIdx][k] = CatmullRom(pm1, p0, p1, p2, w);
                        }
                        rowIdx++;
                    }
                }
                for (int k = 0; k < m; k++)
                    grid[rowIdx][k] = profileWorld[p][k];
                rowIdx++;
            }
        }

        private static Vec3D CatmullRom(Vec3D p0, Vec3D p1, Vec3D p2, Vec3D p3, double t)
        {
            double t2 = t * t;
            double t3 = t2 * t;
            return 0.5 * (
                2 * p1 +
                (-p0 + p2) * t +
                (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
                (-p0 + 3 * p1 - 3 * p2 + p3) * t3);
        }

        private static void GetRowProfileBlend(int row, int pCount, int s, out int pLo, out int pHi, out double wHi)
        {
            if (pCount <= 1)
            {
                pLo = pHi = 0;
                wHi = 0;
                return;
            }
            int stride = s + 1;
            int g = row / stride;
            int r = row % stride;
            if (r == 0)
            {
                pLo = pHi = g;
                wHi = 0;
                return;
            }
            pLo = g;
            pHi = g + 1;
            wHi = r / (double)stride;
        }
    }
}
