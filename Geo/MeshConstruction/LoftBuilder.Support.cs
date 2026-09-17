using Curves;
using Geo.NurbsConstruction;
using GeoCore;
using GeoMeta;
using NURBS;

namespace Geo;

public static partial class LoftBuilder
{
    private static int FindFirstCurveEdge(LoftPreparedProfile profile, string name, int section, bool closed)
    {
        if (string.IsNullOrWhiteSpace(name))
            throw new ArgumentException($"Section {section + 1}: first curve name cannot be empty.");
        var candidates = profile.SourceCurves?.Where(c => c.Name == name).ToArray();
        if (candidates == null || candidates.Length != 1)
            throw new ArgumentException($"Section {section + 1}: first curve '{name}' must identify exactly one profile edge.");
        var curve = candidates[0];
        int count = closed ? profile.Poly.Count : profile.Poly.Count - 1;
        for (int i = 0; i < count; i++)
        {
            var a = profile.Poly[i];
            var b = profile.Poly[(i + 1) % profile.Poly.Count];
            if (a == curve.StartPosition && b == curve.EndPosition) return i;
        }
        throw new ArgumentException($"Section {section + 1}: first curve '{name}' must be one matched polygon edge.");
    }

    // Resolve names geometrically after seam rolling/orientation, so reversing a
    // sketch cannot rename the pressure face as the suction face.
    private static string[] MatchedSideNames(LoftPreparedProfile profile, string loftName)
    {
        if (profile.MatchedCurveNames != null)
            return profile.MatchedCurveNames.Select(name => EntityNaming.LoftSide(loftName, name)).ToArray();
        int count = profile.Closed ? profile.Poly.Count : profile.Poly.Count - 1;
        var result = new string[count];
        var used = new HashSet<string>();
        for (int i = 0; i < count; i++)
        {
            var a = profile.Poly[i];
            var b = profile.Poly[(i + 1) % profile.Poly.Count];
            var source = profile.SourceCurves?.FirstOrDefault(curve =>
                (curve.StartPosition == a && curve.EndPosition == b) ||
                (curve.StartPosition == b && curve.EndPosition == a));
            string label = string.IsNullOrEmpty(source?.Name) ? $"Edge{i + 1}" : source.Name;
            string name = EntityNaming.LoftSide(loftName, label);
            if (!used.Add(name))
                throw new ArgumentException("Loft section edges must have distinct names.");
            result[i] = name;
        }
        return result;
    }

    private static Func<INurbsSurface> BuildExactNamedCurveLoftSupport(
        IReadOnlyList<LoftPreparedProfile> prepared, LoftStyle style, out double[] constructionBreaks)
    {
        int count = prepared[0].SourceCurves.Count;
        var sections = new List<BSplineCurve>();
        foreach (var profile in prepared)
        {
            if (profile.FirstCurveName == null || profile.SourceCurves.Count != count)
                throw new ArgumentException("Matched curved sections require first_curves and equal curve counts.");
            int first = profile.SourceCurves.ToList().FindIndex(curve => curve.Name == profile.FirstCurveName);
            if (first < 0)
                throw new ArgumentException($"Section {sections.Count + 1}: first curve '{profile.FirstCurveName}' does not exist.");
            var curves = Enumerable.Range(0, count)
                .Select(i => profile.SourceCurves[(first + i) % count]).ToArray();
            if (curves.Any(c => string.IsNullOrWhiteSpace(c.Name)) || curves.Select(c => c.Name).Distinct().Count() != count)
                throw new ArgumentException("Matched section curves must have distinct nonempty names.");
            if (curves.Any(c => c is not (Line2D or Arc2D or Circle2D or Ellipse2D or Bezier2D or CubicHermiteSpline2D or BSpline2D)))
                throw new ArgumentException("Matched section curves require an exact NURBS representation.");
            profile.MatchedCurveNames = curves.Select(c => c.Name).ToArray();
            sections.Add(NurbsSurfaceFactory.ConcatenateCompatible(
                curves.Select(c => Curve2DToBSpline.ToBSplineCurve(c, profile.System)).ToArray()));
        }
        var compatible = SurfaceLoft.CompatibleSections(sections);
        if (compatible.Any(section => section.ControlPoints.Where((point, i) =>
                point.W != compatible[0].ControlPoints[i].W).Any()))
            throw new ArgumentException("Matched curved sections must share rational weights after knot compatibility.");
        for (int p = 0; p < prepared.Count; p++) prepared[p].MatchedCurve = compatible[p];
        constructionBreaks = Enumerable.Range(0, count + 1).Select(i => (double)i / count).ToArray();
        return () => NurbsSurfaceFactory.LoftFromProfileCurves(compatible, style);
    }

    /// <summary>
    /// Polygon profiles are linear in normalized arc length between their corner
    /// parameters. Subdividing at the union of those parameters makes their U
    /// knots compatible without changing any profile or fitting mesh samples.
    /// </summary>
    private static Func<INurbsSurface> BuildExactPolygonLoftSupport(
        IReadOnlyList<LoftPreparedProfile> prepared, LoftStyle style, out double[] constructionBreaks,
        bool matchingVertices = false)
    {
        if (matchingVertices && prepared.Any(p => p.SourceCurves?.Any(c => c is not Line2D) == true))
            return BuildExactNamedCurveLoftSupport(prepared, style, out constructionBreaks);
        if (matchingVertices)
        {
            int count = prepared[0].Poly.Count;
            if (prepared.Any(p => p.Poly.Count != count ||
                (p.AnalyticStrip != null && p.AnalyticStrip.Segments.Any(s => s is not Line2D))))
                throw new ArgumentException("MatchingVertices requires polygon sections with equal vertex counts.");
            int spans = prepared[0].Closed ? count : count - 1;
            constructionBreaks = Enumerable.Range(0, spans + 1).Select(i => (double)i / spans).ToArray();
            var matchedKnots = new[] { 0.0 }.Concat(constructionBreaks).Concat(new[] { 1.0 }).ToArray();
            var matchedProfiles = prepared.Select(p => new BSplineCurve(1,
                Enumerable.Range(0, spans + 1).Select(i => p.System.PointTo3D(p.Poly[i % count])).ToArray(),
                (double[])matchedKnots.Clone(), p.Closed)).ToList();
            return () => NurbsSurfaceFactory.LoftFromProfileCurves(matchedProfiles, style);
        }
        // A full conic has an exact rational parameterization shared by every
        // section. Use it for both sampling and support metadata. Mixing the
        // sketch's angle parameter with rational surface UVs makes refinement
        // and normals refer to different positions on the same ellipse.
        if (prepared.All(p => p.SeamU0 == 0 && p.AnalyticStrip != null &&
            p.SourceCurves?.Count == 1 && p.SourceCurves[0] is Circle2D or Ellipse2D))
        {
            var conics = SurfaceLoft.CompatibleSections(prepared.Select(p =>
                Curve2DToBSpline.ToBSplineCurve(p.SourceCurves[0], p.System)).ToArray());
            for (int p = 0; p < prepared.Count; p++)
            {
                prepared[p].MatchedCurve = conics[p];
                prepared[p].MatchedCurveNames = new[] { prepared[p].SourceCurves[0].Name };
            }
            constructionBreaks = conics[0].Knots.Distinct().ToArray();
            return () => NurbsSurfaceFactory.LoftFromProfileCurves(conics, style);
        }
        constructionBreaks = null;
        var strips = prepared.Select(profile => profile.AnalyticStrip ??
            CurveStrip2D.FromTessellatedPolyline(profile.Poly, profile.Norms, profile.Closed)).ToArray();
        if (strips.Any(strip => strip.Segments.Any(segment => segment is not Line2D)))
            return null;

        var breaks = new SortedSet<double> { 0, 1 };
        for (int p = 0; p < prepared.Count; p++)
        {
            foreach (double distance in strips[p].CumulativeLengthAtSegmentStart)
            {
                double authored = distance / strips[p].TotalLength;
                breaks.Add(ToSeamU(authored, prepared[p].SeamU0, prepared[p].Closed));
            }
        }
        var parameters = breaks.ToArray();
        constructionBreaks = parameters;
        var knots = new[] { 0.0 }.Concat(parameters).Concat(new[] { 1.0 }).ToArray();
        var profiles = new List<BSplineCurve>(prepared.Count);
        for (int p = 0; p < prepared.Count; p++)
        {
            var points = new Vec3D[parameters.Length];
            for (int u = 0; u < parameters.Length; u++)
            {
                double authored = ToAuthoredU(parameters[u], prepared[p].SeamU0, prepared[p].Closed);
                strips[p].EvaluatePositionAndNormalAtNormalizedArcLength(authored, out var point, out _);
                points[u] = prepared[p].System.PointTo3D(point);
            }
            profiles.Add(new BSplineCurve(1, points, (double[])knots.Clone(), prepared[p].Closed));
        }
        return () => NurbsSurfaceFactory.LoftFromProfileCurves(profiles, style);
    }
}
