using System;
using System.Collections.Generic;
using System.Linq;
using Curves;
using GeoCore;

namespace Geo
{
    public static partial class LoftBuilder
    {
        private const double TolSq = 1e-16;
        private const double EpsLen = 1e-14;

        private static void CollectPreparedProfilesFromSketches(
            IReadOnlyList<PlotterSketcherCoordSys> sketches,
            double maxDeviation,
            LoftOptions options,
            out List<LoftPreparedProfile> prepared)
        {
            var flags = SketchTessellationFlags.ExcludeHelperGeometry;
            if (options.AllowOpenContour)
                flags |= SketchTessellationFlags.AllowOpenContour;

            prepared = new List<LoftPreparedProfile>(sketches.Count);
            foreach (var sketch in sketches)
            {
                var tess = sketch.Tessellate(maxDeviation, out var norms, out _, out _, maxDeviation, flags);
                if (tess.Count != 1)
                    throw new InvalidOperationException(
                        "Loft requires exactly one connected curve strip per sketch (no extra islands or disconnected loops). " +
                        "Merge sketches or remove extra geometry.");

                SketchStripTessellator.FlattenStripWithCreases(tess[0], norms[0], out var poly, out var pn, out var creases);

                CurveStrip2D analytic = null;
                if (options.ProfileSampling == LoftProfileSamplingSource.AnalyticCurveStrip)
                {
                    var stripCurves = sketch.GetCurves()
                        .Select(s => s.Where(c => !c.IsHelperGeometry).ToList())
                        .FirstOrDefault(s => s.Count > 0) ?? new List<Curve2D>();
                    if (stripCurves.Count > 0)
                        analytic = CurveStrip2D.FromSegments(stripCurves);
                }

                prepared.Add(new LoftPreparedProfile
                {
                    Poly = poly,
                    Norms = pn,
                    CreaseIdx = creases,
                    System = sketch.CoordinateSystem,
                    AnalyticStrip = analytic,
                    SeamU0 = 0
                });
            }
        }

        private static void PrepareProfilePolyline(List<Vec2D> poly, List<Vec2D> norms, out bool closed, out int removedClosingCount)
        {
            closed = SketchStripTessellator.IsClosedPolyline(poly);
            removedClosingCount = 0;
            if (closed && Vec2DOps.DistanceSquared(poly[0], poly[^1]) <= TolSq)
            {
                poly.RemoveAt(poly.Count - 1);
                norms.RemoveAt(norms.Count - 1);
                removedClosingCount = 1;
            }
        }

        private static List<int> RemapCreaseAfterPrepare(List<int> crease, int nBefore, int nAfter, int removedClosing)
        {
            if (crease == null || crease.Count == 0)
                return new List<int>();
            if (removedClosing == 0)
            {
                return crease
                    .Where(c => c >= 0 && c < nAfter)
                    .Distinct()
                    .OrderBy(x => x)
                    .ToList();
            }
            int nOld = nAfter + removedClosing;
            var set = new HashSet<int>();
            foreach (int c in crease)
            {
                int nc = c;
                if (c == nOld - 1)
                    nc = 0;
                else if (c < 0 || c >= nAfter)
                    continue;
                set.Add(nc);
            }
            return set.OrderBy(x => x).ToList();
        }

        private static List<int> RemapCreaseReverse(List<int> crease, int n)
        {
            if (crease == null || crease.Count == 0)
                return new List<int>();
            return crease.Select(j => n - 1 - j).Distinct().OrderBy(x => x).ToList();
        }

        private static List<int> RemapCreaseForRoll(List<int> crease, int n, int rollBy, bool closed)
        {
            if (crease == null || crease.Count == 0)
                return new List<int>();
            if (rollBy == 0)
                return new List<int>(crease);
            var mapped = new HashSet<int>();
            foreach (int j in crease)
            {
                int nj = closed
                    ? (j - rollBy + n) % n
                    : (j >= rollBy ? j - rollBy : j + (n - rollBy));
                if (nj >= 0 && nj < n)
                    mapped.Add(nj);
            }
            return mapped.OrderBy(x => x).ToList();
        }

        private static void RollPolylineInPlace(List<Vec2D> poly, List<Vec2D> norms, bool closed, int rollBy)
        {
            if (rollBy == 0)
                return;
            int n = poly.Count;
            var np = new List<Vec2D>(n);
            var nn = new List<Vec2D>(n);
            if (closed)
            {
                for (int k = 0; k < n; k++)
                {
                    int j = (rollBy + k) % n;
                    np.Add(poly[j]);
                    nn.Add(norms[j]);
                }
            }
            else
            {
                for (int j = rollBy; j < n; j++)
                {
                    np.Add(poly[j]);
                    nn.Add(norms[j]);
                }
                for (int j = 0; j < rollBy; j++)
                {
                    np.Add(poly[j]);
                    nn.Add(norms[j]);
                }
            }
            poly.Clear();
            norms.Clear();
            poly.AddRange(np);
            norms.AddRange(nn);
        }

        /// <summary>Vertex index to roll to so the seam parameter <paramref name="seamUAuth"/> is nearest a vertex.</summary>
        private static int NearestVertexRollIndexForSeamU(List<Vec2D> poly, List<Vec2D> norms, bool closed, double seamUAuth)
        {
            var strip = CurveStrip2D.FromTessellatedPolyline(poly, norms, closed);
            strip.EvaluatePositionAndVertexNormalAtNormalizedArcLength(Frac01(seamUAuth), out var seamPt, out _);
            int bestI = 0;
            double bestD = double.MaxValue;
            for (int i = 0; i < poly.Count; i++)
            {
                double d = Vec2DOps.DistanceSquared(poly[i], seamPt);
                if (d < bestD)
                {
                    bestD = d;
                    bestI = i;
                }
            }
            return bestI;
        }

        private static bool OrientClosedCounterClockwise(List<Vec2D> poly, List<Vec2D> norms, bool closed)
        {
            if (!closed || poly.Count < 3)
                return false;
            if (!Polygon.IsPolygonCCW(poly))
            {
                poly.Reverse();
                norms.Reverse();
                return true;
            }
            return false;
        }

        /// <summary>Maps authored-strip u into seam-parameter space (u=0 at seam).</summary>
        internal static double ToSeamU(double authoredU, double seamU0, bool closed)
        {
            if (!closed || Math.Abs(seamU0) < 1e-15)
                return authoredU;
            return Frac01(authoredU - seamU0);
        }

        /// <summary>Maps seam-parameter u to authored-strip evaluation parameter.</summary>
        internal static double ToAuthoredU(double seamU, double seamU0, bool closed)
        {
            if (!closed || Math.Abs(seamU0) < 1e-15)
                return seamU;
            return Frac01(seamU + seamU0);
        }

        internal static double Frac01(double u)
        {
            u %= 1.0;
            if (u < 0)
                u += 1.0;
            if (u >= 1.0 - 1e-15)
                return 0;
            return u;
        }

        /// <summary>
        /// Closest point on a closed (or open) polyline strip to <paramref name="hint"/>, as normalized arc length in [0,1).
        /// </summary>
        internal static double ClosestPointNormalizedU(List<Vec2D> poly, List<Vec2D> norms, bool closed, Vec2D hint)
        {
            var strip = CurveStrip2D.FromTessellatedPolyline(poly, norms, closed);
            double total = strip.TotalLength;
            if (total < EpsLen)
                return 0;

            // Dense sample + refine on best edge neighborhood.
            int nSample = Math.Clamp(Math.Max(poly.Count * 4, 64), 64, 512);
            double bestU = 0;
            double bestD = double.MaxValue;
            for (int i = 0; i <= nSample; i++)
            {
                double u = closed
                    ? (nSample == 0 ? 0 : (double)i / nSample)
                    : (nSample == 0 ? 0 : (double)i / nSample);
                if (!closed && u > 1)
                    u = 1;
                if (closed && i == nSample)
                    u = 0;
                strip.EvaluatePositionAndVertexNormalAtNormalizedArcLength(u, out var p, out _);
                double d = Vec2DOps.DistanceSquared(p, hint);
                if (d < bestD)
                {
                    bestD = d;
                    bestU = u;
                }
            }

            // Local refine around bestU
            double span = closed ? 2.0 / nSample : Math.Min(2.0 / nSample, 0.05);
            for (int iter = 0; iter < 8; iter++)
            {
                double uLo = closed ? Frac01(bestU - span) : Math.Max(0, bestU - span);
                double uHi = closed ? Frac01(bestU + span) : Math.Min(1, bestU + span);
                if (!closed && uHi < uLo)
                    (uLo, uHi) = (uHi, uLo);
                int refine = 16;
                for (int i = 0; i <= refine; i++)
                {
                    double t = (double)i / refine;
                    double u = closed
                        ? Frac01(uLo + t * (uHi >= uLo ? uHi - uLo : uHi + 1 - uLo))
                        : uLo + t * (uHi - uLo);
                    if (closed && uHi < uLo)
                        u = Frac01(uLo + t * (uHi + 1 - uLo));
                    strip.EvaluatePositionAndVertexNormalAtNormalizedArcLength(u, out var p, out _);
                    double d = Vec2DOps.DistanceSquared(p, hint);
                    if (d < bestD)
                    {
                        bestD = d;
                        bestU = u;
                    }
                }
                span *= 0.35;
            }

            return closed ? Frac01(bestU) : Math.Clamp(bestU, 0, 1);
        }

        private static double ComputeProfileSeamU0(
            int profileIndex,
            LoftOptions options,
            bool closed,
            List<Vec2D> poly,
            List<Vec2D> norms,
            LoftProfileSeamHint seamHint,
            List<Vec2D> prevPoly,
            List<Vec2D> prevNorms,
            bool prevClosed,
            double prevSeamU0,
            CoordinateSystem system,
            CoordinateSystem? prevSystem)
        {
            // Open profiles keep authored start/end — never reparameterize.
            if (!closed)
                return 0;

            if (seamHint.HasPoint)
                return ClosestPointNormalizedU(poly, norms, closed, seamHint.Point);

            switch (options.AlignmentMode)
            {
                case LoftAlignmentMode.AsAuthored:
                    return 0;
                case LoftAlignmentMode.MinimumTwist:
                    if (profileIndex > 0 && prevPoly != null && prevNorms != null && prevSystem.HasValue && prevClosed)
                        return ComputeMinimumTwistSeamU0(prevPoly, prevNorms, prevSeamU0, poly, norms, prevSystem.Value, system);
                    // First profile: same as origin-foot (nearest vertex parameter).
                    return VertexNormalizedU(poly, norms, closed, GetOriginFootRollIndex(poly));
                default: // OriginFootRoll — nearest vertex to sketch origin (legacy discrete foot).
                    return VertexNormalizedU(poly, norms, closed, GetOriginFootRollIndex(poly));
            }
        }

        private static int GetOriginFootRollIndex(List<Vec2D> poly)
        {
            var origin = new Vec2D(0, 0);
            int bestI = 0;
            double bestD = double.MaxValue;
            for (int i = 0; i < poly.Count; i++)
            {
                double d = Vec2DOps.DistanceSquared(poly[i], origin);
                if (d < bestD)
                {
                    bestD = d;
                    bestI = i;
                }
            }
            return bestI;
        }

        private static double VertexNormalizedU(List<Vec2D> poly, List<Vec2D> norms, bool closed, int vertexIndex)
        {
            var strip = CurveStrip2D.FromTessellatedPolyline(poly, norms, closed);
            double total = strip.TotalLength;
            if (total < EpsLen || vertexIndex <= 0)
                return 0;
            double[] distBuf = LineStrip2D.BuildDistanceBuffer(poly);
            return Frac01(distBuf[Math.Clamp(vertexIndex, 0, poly.Count - 1)] / total);
        }

        /// <summary>
        /// Continuous u0 on the current closed profile minimizing Σ‖p_cur(u)−p_prev(u)‖² in world space
        /// (previous profile already seam-aligned via <paramref name="prevSeamU0"/>).
        /// </summary>
        private static double ComputeMinimumTwistSeamU0(
            List<Vec2D> prevPoly,
            List<Vec2D> prevNorms,
            double prevSeamU0,
            List<Vec2D> curPoly,
            List<Vec2D> curNorms,
            CoordinateSystem prevCs,
            CoordinateSystem curCs)
        {
            var prevStrip = CurveStrip2D.FromTessellatedPolyline(prevPoly, prevNorms, closed: true);
            var curStrip = CurveStrip2D.FromTessellatedPolyline(curPoly, curNorms, closed: true);
            int nSample = Math.Min(64, Math.Min(prevPoly.Count, curPoly.Count));
            nSample = Math.Max(8, nSample);

            var prevW = new Vec3D[nSample];
            for (int j = 0; j < nSample; j++)
            {
                double uSeam = nSample <= 1 ? 0 : (double)j / nSample;
                double uAuth = ToAuthoredU(uSeam, prevSeamU0, closed: true);
                prevStrip.EvaluatePositionAndVertexNormalAtNormalizedArcLength(uAuth, out var p2, out _);
                prevW[j] = prevCs.PointTo3D(p2);
            }

            double TwistCost(double seamU0)
            {
                double c = 0;
                for (int j = 0; j < nSample; j++)
                {
                    double uSeam = nSample <= 1 ? 0 : (double)j / nSample;
                    double uAuth = ToAuthoredU(uSeam, seamU0, closed: true);
                    curStrip.EvaluatePositionAndVertexNormalAtNormalizedArcLength(uAuth, out var p2, out _);
                    c += Vec3DOps.DistanceSquared(curCs.PointTo3D(p2), prevW[j]);
                }
                return c;
            }

            // Coarse: vertex-parameter samples + origin foot
            double bestU0 = ClosestPointNormalizedU(curPoly, curNorms, true, new Vec2D(0, 0));
            double bestCost = TwistCost(bestU0);
            double[] distBuf = LineStrip2D.BuildDistanceBuffer(curPoly);
            double total = curStrip.TotalLength;
            for (int i = 0; i < curPoly.Count; i++)
            {
                double u0 = distBuf[i] / total;
                double c = TwistCost(u0);
                if (c < bestCost)
                {
                    bestCost = c;
                    bestU0 = u0;
                }
            }

            // Dense refine around best
            int coarse = 64;
            for (int i = 0; i < coarse; i++)
            {
                double u0 = (double)i / coarse;
                double c = TwistCost(u0);
                if (c < bestCost)
                {
                    bestCost = c;
                    bestU0 = u0;
                }
            }

            double span = 1.0 / coarse;
            for (int iter = 0; iter < 10; iter++)
            {
                double uLo = Frac01(bestU0 - span);
                double uHi = Frac01(bestU0 + span);
                int refine = 20;
                for (int i = 0; i <= refine; i++)
                {
                    double t = (double)i / refine;
                    double u0 = Frac01(uLo + t * (uHi >= uLo ? uHi - uLo : uHi + 1 - uLo));
                    if (uHi < uLo)
                        u0 = Frac01(uLo + t * (uHi + 1 - uLo));
                    double c = TwistCost(u0);
                    if (c < bestCost)
                    {
                        bestCost = c;
                        bestU0 = u0;
                    }
                }
                span *= 0.4;
            }

            return Frac01(bestU0);
        }
    }
}
