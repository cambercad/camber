using System;
using System.Collections.Generic;
using System.Linq;
using Curves;
using GeoCore;
using Geo.NurbsConstruction;
using NURBS;

namespace Geo;

public static partial class LoftBuilder
{
    // When both end sections describe the same number of boundary curves, keep
    // those semantic spans even if an intermediate section is one unsplit curve
    // (an ellipse is the common case). The sketch geometry is never modified.
    private static bool PrepareSampledCurveCorrespondence(List<LoftPreparedProfile> profiles, double deviation)
    {
        if (profiles.Count < 3 || profiles.Any(p => !p.Closed))
            return false;
        int spanCount = profiles[0].SourceCurves?.Count ?? 0;
        if (spanCount < 2 || profiles[^1].SourceCurves?.Count != spanCount ||
            profiles.All(p => p.SourceCurves?.Count == spanCount))
            return false;
        if (profiles[0].SourceCurves.Any(c => string.IsNullOrWhiteSpace(c.Name)) ||
            profiles[0].SourceCurves.Select(c => c.Name).Distinct().Count() != spanCount)
            return false;

        var authored = new List<List<List<Vec2D>>>(profiles.Count);
        int samplesPerSpan = 8;
        foreach (var profile in profiles)
        {
            if (profile.SourceCurves?.Count == spanCount)
            {
                var spans = new List<List<Vec2D>>(spanCount);
                foreach (var curve in profile.SourceCurves)
                {
                    var points = curve.Tessellate(Math.Max(deviation, 1e-6))
                        .Select(v => v.Position).ToList();
                    RemoveAdjacentDuplicates(points);
                    if (points.Count < 2)
                        return false;
                    samplesPerSpan = Math.Max(samplesPerSpan, points.Count - 1);
                    spans.Add(points);
                }
                authored.Add(spans);
            }
            else
            {
                var contour = new List<Vec2D>(profile.Poly);
                if (contour.Count > 1 && Vec2DOps.DistanceSquared(contour[0], contour[^1]) <= TolSq)
                    contour.RemoveAt(contour.Count - 1);
                if (contour.Count < 3)
                    return false;
                samplesPerSpan = Math.Max(samplesPerSpan, (int)Math.Ceiling((double)contour.Count / spanCount));
                authored.Add(new List<List<Vec2D>> { contour });
            }
        }
        if (samplesPerSpan > 2048)
            return false;

        // Match each section to the preceding one. Equal-count sections may
        // rotate/reverse by whole curves; a differently segmented contour may
        // move its seam continuously, with candidates at its tessellation knots.
        var aligned = new List<List<List<Vec2D>>>(profiles.Count);
        var candidatesByProfile = new List<List<List<List<Vec2D>>>>(profiles.Count);
        aligned.Add(ResampleAuthoredSpans(authored[0], spanCount, samplesPerSpan));
        candidatesByProfile.Add(new List<List<List<Vec2D>>> { aligned[0] });
        string[] firstNames = profiles[0].SourceCurves.Select(c => c.Name).ToArray();
        if (!Polygon.IsPolygonCCW(aligned[0].SelectMany((span, i) =>
                i == 0 ? span.Take(span.Count - 1) : span.Take(span.Count - 1)).ToList()))
        {
            aligned[0] = ReverseSpans(aligned[0]);
            firstNames = firstNames.Reverse().ToArray();
        }
        for (int i = 1; i < profiles.Count; i++)
        {
            var candidates = CorrespondenceCandidates(authored[i], spanCount, samplesPerSpan);
            candidatesByProfile.Add(candidates);
            List<List<Vec2D>> best = null;
            double bestCost = double.PositiveInfinity;
            foreach (var candidate in candidates)
            {
                double cost = CorrespondenceCost(aligned[i - 1], profiles[i - 1].System,
                    candidate, profiles[i].System);
                if (cost < bestCost)
                {
                    bestCost = cost;
                    best = candidate;
                }
            }
            if (best == null)
                return false;
            aligned.Add(best);
        }

        // Revisit interior sections against both neighbors. This avoids letting
        // a middle contour's seam be determined solely by the loft start.
        for (int pass = 0; pass < 3; pass++)
        for (int i = 1; i < profiles.Count; i++)
        {
            List<List<Vec2D>> best = aligned[i];
            double bestCost = double.PositiveInfinity;
            foreach (var candidate in candidatesByProfile[i])
            {
                double cost = CorrespondenceCost(aligned[i - 1], profiles[i - 1].System,
                    candidate, profiles[i].System);
                if (i + 1 < profiles.Count)
                    cost += CorrespondenceCost(candidate, profiles[i].System,
                        aligned[i + 1], profiles[i + 1].System);
                if (cost < bestCost)
                {
                    bestCost = cost;
                    best = candidate;
                }
            }
            aligned[i] = best;
        }

        int segmentCount = spanCount * samplesPerSpan;
        var knots = new double[segmentCount + 3];
        knots[0] = knots[1] = 0;
        for (int i = 1; i < segmentCount; i++)
            knots[i + 1] = (double)i / segmentCount;
        knots[^2] = knots[^1] = 1;

        for (int p = 0; p < profiles.Count; p++)
        {
            var controls = new Vec3D[segmentCount + 1];
            var flattened = new List<Vec2D>(segmentCount);
            int target = 0;
            for (int span = 0; span < spanCount; span++)
            {
                var points = ResamplePolyline(aligned[p][span], samplesPerSpan);
                for (int j = 0; j < samplesPerSpan; j++)
                {
                    flattened.Add(points[j]);
                    controls[target++] = profiles[p].System.PointTo3D(points[j]);
                }
            }
            controls[segmentCount] = controls[0];
            var profile = profiles[p];
            profile.Poly = flattened;
            profile.Norms = Enumerable.Repeat(new Vec2D(0, 0), flattened.Count).ToList();
            profile.Poly.Add(profile.Poly[0]);
            profile.Norms.Add(profile.Norms[0]);
            profile.CreaseIdx = new List<int>();
            profile.AnalyticStrip = null;
            profile.MatchedCurve = new BSplineCurve(1, controls, knots, true);
            profile.CorrespondenceLocked = true;
            profile.SeamU0 = 0;
        }
        profiles[0].MatchedCurveNames = firstNames;
        return true;
    }

    private static List<List<Vec2D>> ReverseSpans(List<List<Vec2D>> spans)
    {
        return spans.AsEnumerable().Reverse().Select(span =>
        {
            var reversed = new List<Vec2D>(span);
            reversed.Reverse();
            return reversed;
        }).ToList();
    }

    private static List<List<List<Vec2D>>> CorrespondenceCandidates(
        List<List<Vec2D>> authored, int spanCount, int samplesPerSpan)
    {
        var result = new List<List<List<Vec2D>>>();
        if (authored.Count == spanCount)
        {
            for (int shift = 0; shift < spanCount; shift++)
            for (int reverse = 0; reverse < 2; reverse++)
            {
                var spans = new List<List<Vec2D>>(spanCount);
                for (int i = 0; i < spanCount; i++)
                {
                    int index = reverse == 0
                        ? (i + shift) % spanCount
                        : (shift - i - 1 + spanCount * 2) % spanCount;
                    var points = new List<Vec2D>(authored[index]);
                    if (reverse != 0) points.Reverse();
                    spans.Add(ResamplePolyline(points, samplesPerSpan));
                }
                result.Add(spans);
            }
            return result;
        }

        var contour = authored[0];
        var cumulative = CumulativeLengths(contour, true, out double length);
        if (length <= EpsLen)
            return result;
        int seamStep = Math.Max(1, contour.Count / 128);
        foreach (double seam in cumulative.Take(contour.Count).Where((_, i) => i % seamStep == 0)
                     .Select(d => d / length).Distinct())
        for (int reverse = 0; reverse < 2; reverse++)
        {
            var spans = new List<List<Vec2D>>(spanCount);
            for (int i = 0; i < spanCount; i++)
            {
                var points = new List<Vec2D>(samplesPerSpan + 1);
                for (int j = 0; j <= samplesPerSpan; j++)
                {
                    double u = seam + (reverse == 0 ? 1 : -1) *
                        (i + (double)j / samplesPerSpan) / spanCount;
                    points.Add(PointAtClosedPolyline(contour, cumulative, length, u));
                }
                spans.Add(points);
            }
            result.Add(spans);
        }
        return result;
    }

    private static List<List<Vec2D>> ResampleAuthoredSpans(
        List<List<Vec2D>> authored, int spanCount, int samplesPerSpan)
    {
        if (authored.Count == spanCount)
            return authored.Select(path => ResamplePolyline(path, samplesPerSpan)).ToList();
        // Initial unmatched sections are not expected because end sections set
        // the match count; keep a deterministic fallback for future callers.
        return CorrespondenceCandidates(authored, spanCount, samplesPerSpan)[0];
    }

    private static double CorrespondenceCost(List<List<Vec2D>> a, CoordinateSystem aFrame,
        List<List<Vec2D>> b, CoordinateSystem bFrame)
    {
        double cost = 0;
        for (int span = 0; span < a.Count; span++)
        for (int j = 0; j <= 4; j++)
        {
            Vec2D pa = PointAtOpenPolyline(a[span], j * .25);
            Vec2D pb = PointAtOpenPolyline(b[span], j * .25);
            Vec3D delta = aFrame.PointTo3D(pa) - bFrame.PointTo3D(pb);
            cost += delta.LengthSquared();
        }
        return cost;
    }

    private static List<Vec2D> ResamplePolyline(IReadOnlyList<Vec2D> points, int segments)
    {
        var cumulative = CumulativeLengths(points, false, out double length);
        var result = new List<Vec2D>(segments + 1);
        for (int i = 0; i <= segments; i++)
            result.Add(PointAtOpenPolyline(points, cumulative, length, (double)i / segments));
        return result;
    }

    private static Vec2D PointAtOpenPolyline(IReadOnlyList<Vec2D> points, double u)
    {
        var cumulative = CumulativeLengths(points, false, out double length);
        return PointAtOpenPolyline(points, cumulative, length, u);
    }

    private static Vec2D PointAtOpenPolyline(IReadOnlyList<Vec2D> points,
        double[] cumulative, double length, double u)
    {
        double target = Math.Clamp(u, 0, 1) * length;
        int i = Array.BinarySearch(cumulative, target);
        if (i >= 0) return points[Math.Min(i, points.Count - 1)];
        i = Math.Clamp(~i - 1, 0, points.Count - 2);
        double span = cumulative[i + 1] - cumulative[i];
        double t = span <= EpsLen ? 0 : (target - cumulative[i]) / span;
        return points[i] * (1 - t) + points[i + 1] * t;
    }

    private static Vec2D PointAtClosedPolyline(IReadOnlyList<Vec2D> points,
        double[] cumulative, double length, double u)
    {
        u -= Math.Floor(u);
        double target = u * length;
        int i = Array.BinarySearch(cumulative, target);
        if (i >= 0) return points[i % points.Count];
        i = ~i - 1;
        if (i < 0) i = points.Count - 1;
        int next = (i + 1) % points.Count;
        double start = cumulative[i];
        double end = next == 0 ? length : cumulative[next];
        double t = end - start <= EpsLen ? 0 : (target - start) / (end - start);
        return points[i] * (1 - t) + points[next] * t;
    }

    private static double[] CumulativeLengths(IReadOnlyList<Vec2D> points, bool closed, out double total)
    {
        var cumulative = new double[points.Count];
        total = 0;
        for (int i = 1; i < points.Count; i++)
        {
            total += Math.Sqrt(Vec2DOps.DistanceSquared(points[i - 1], points[i]));
            cumulative[i] = total;
        }
        if (closed && points.Count > 1)
            total += Math.Sqrt(Vec2DOps.DistanceSquared(points[^1], points[0]));
        return cumulative;
    }

    private static void RemoveAdjacentDuplicates(List<Vec2D> points)
    {
        for (int i = points.Count - 1; i > 0; i--)
            if (Vec2DOps.DistanceSquared(points[i], points[i - 1]) <= TolSq)
                points.RemoveAt(i);
    }

    private static Func<INurbsSurface> BuildPreparedCurveLoftSupport(
        IReadOnlyList<LoftPreparedProfile> profiles, LoftStyle style, out double[] breaks)
    {
        var sections = SurfaceLoft.CompatibleSections(profiles.Select(p => p.MatchedCurve).ToArray());
        breaks = sections[0].Knots.Distinct().OrderBy(k => k).ToArray();
        return () => NurbsSurfaceFactory.LoftFromProfileCurves(sections, style);
    }
}
