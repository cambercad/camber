using Curves;
using GeoCore;
using NURBS;

namespace Geo.NurbsConstruction;

/// <summary>Interpolating sheet loft with optional boundary rails and end derivatives.</summary>
public static class SurfaceLoft
{
    /// <summary>
    /// Sections retain their common authored U parameterization. V assigns equal intervals
    /// to consecutive sections. End tangents are derivatives with respect to global V.
    /// Rails follow either section endpoint, in section order. A Hermite rail has one knot
    /// per section; a line rail is subdivided exactly. Neither sections nor rails are fitted.
    /// </summary>
    public static BSplineSurface Build(IReadOnlyList<BSplineCurve> sections,
        IReadOnlyList<Curve3D> guides = null, Vec3D? startTangent = null, Vec3D? endTangent = null)
    {
        if (sections == null || sections.Count < 2 || sections.Any(s => s == null))
            throw new ArgumentException("Surface loft requires at least two sections.", nameof(sections));
        ValidateTangent(startTangent, nameof(startTangent));
        ValidateTangent(endTangent, nameof(endTangent));
        sections = CompatibleSections(sections);
        var first = sections[0];
        int count = first.ControlPoints.Length;
        foreach (var section in sections)
        {
            for (int i = 0; i < count; i++)
            {
                var h = section.ControlPoints[i];
                if (!double.IsFinite(h.W) || h.W <= 0 || !Finite(Point(h)))
                    throw new ArgumentException("Section control points must be finite with positive weights.");
                // A common weight function preserves every rational section during skinning.
                if (h.W != first.ControlPoints[i].W)
                    throw new ArgumentException("Sections must have the same rational weights.");
            }
        }
        var columns = new List<BSplineCurve>(count);
        for (int i = 0; i < count; i++)
            columns.Add(NurbsSurfaceFactory.BuildHermiteVCurve(
                sections.Select(s => Point(s.ControlPoints[i])).ToArray(), startTangent, endTangent, forceCubic: true));

        var used = new HashSet<int>();
        foreach (var guide in guides ?? Array.Empty<Curve3D>())
        {
            if (guide == null) throw new ArgumentException("A guide cannot be null.", nameof(guides));
            BSplineCurve rail;
            if (guide is CubicHermiteSpline3D hermite && hermite.Points.Count == sections.Count)
                rail = Curve3DToBSpline.HermiteToBSpline(hermite);
            else if (guide is Line3D line)
            {
                var points = Enumerable.Range(0, sections.Count)
                    .Select(i => line.Start + (line.End - line.Start) * ((double)i / (sections.Count - 1))).ToArray();
                rail = NurbsSurfaceFactory.BuildHermiteVCurve(points, forceCubic: true);
            }
            else
                throw new ArgumentException("Boundary guides must be lines or Hermite curves with one knot per section.", nameof(guides));
            var endpoints = new[] { 0, count - 1 }.Where(i => Enumerable.Range(0, sections.Count).All(p =>
                SamePoint(rail.EvaluateUniform((double)p / (sections.Count - 1)), Point(sections[p].ControlPoints[i])))).ToArray();
            if (endpoints.Length != 1)
                throw new ArgumentException("Each guide must intersect the same endpoint of every section, in section order; ambiguous or interior guides are unsupported.", nameof(guides));
            int endpoint = endpoints[0];
            if (!used.Add(endpoint)) throw new ArgumentException("Only one guide may control each boundary.", nameof(guides));
            if (startTangent.HasValue && !SamePoint(RailDerivative(rail, false), startTangent.Value) ||
                endTangent.HasValue && !SamePoint(RailDerivative(rail, true), endTangent.Value))
                throw new ArgumentException("Guide derivatives conflict with the requested end tangents.", nameof(guides));
            columns[endpoint] = rail;
        }
        return new BSplineSurface(first.Degree, columns,
            first.ControlPoints.Select(h => h.W).ToArray(), (double[])first.Knots.Clone());
    }

    internal static BSplineCurve[] CompatibleSections(IReadOnlyList<BSplineCurve> sections)
    {
        foreach (var section in sections)
            if (section.ControlPoints.Length < 2 || section.Degree < 1 ||
                section.Knots.Take(section.Degree + 1).Any(k => k != 0) ||
                section.Knots.TakeLast(section.Degree + 1).Any(k => k != 1))
                throw new ArgumentException("Sections must be clamped curves parameterized from zero to one.");
        int degree = sections.Max(s => s.Degree);
        var curves = sections.Select(s => NurbsSurfaceFactory.ElevateDegree(s, degree)).ToArray();
        var multiplicities = curves.SelectMany(s => s.Knots.Distinct().Where(k => k > 0 && k < 1)
            .Select(k => (Knot: k, Count: s.Knots.Count(v => v == k))))
            .GroupBy(k => k.Knot).ToDictionary(g => g.Key, g => g.Max(k => k.Count));
        foreach (var knot in multiplicities.OrderBy(k => k.Key))
        {
            if (knot.Value > degree) throw new ArgumentException("Section curves must be continuous.");
            for (int i = 0; i < curves.Length; i++)
            {
                int missing = knot.Value - curves[i].Knots.Count(k => k == knot.Key);
                if (missing > 0) curves[i] = curves[i].InsertKnot(knot.Key, missing);
            }
        }
        return curves;
    }

    private static Vec3D RailDerivative(BSplineCurve rail, bool end)
    {
        var cp = rail.ControlPoints;
        return end
            ? (Point(cp[^1]) - Point(cp[^2])) * (rail.Degree / (1 - rail.Knots[cp.Length - 1]))
            : (Point(cp[1]) - Point(cp[0])) * (rail.Degree / rail.Knots[rail.Degree + 1]);
    }
    private static Vec3D Point(Vec4D h) => new(h.X / h.W, h.Y / h.W, h.Z / h.W);
    private static bool Finite(Vec3D p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z);
    // Validation only: never snap or alter an authored section or guide.
    private static bool SamePoint(Vec3D a, Vec3D b) => (a - b).Length() <=
        64 * 2.2204460492503131e-16 * Math.Max(1, Math.Max(a.Length(), b.Length()));
    private static void ValidateTangent(Vec3D? tangent, string name)
    {
        if (tangent.HasValue && (!Finite(tangent.Value) || tangent.Value.Length() == 0))
            throw new ArgumentException("End tangents must be finite, nonzero derivative vectors.", name);
    }
}
