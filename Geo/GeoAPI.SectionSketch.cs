using CSG;
using Curves;
using GeoCore;
using GeoMeta;

namespace Geo;

public partial class GeoAPI
{
    [APIDescription(@"SectionSketch(solid: AnchorMesh, plane: CoordinateSystem, name: str = None) -> PlotterSketcherCoordSys
Creates a registered sketch of a solid's cross-section. Each contiguous intersection with a source surface patch is a sampled curve in plane-local coordinates. The solid is unchanged. Coplanar face overlap is omitted.")]
    public PlotterSketcherCoordSys SectionSketch(AnchorMesh solid, CoordinateSystem plane, string name = null)
    {
        if (solid == null) throw new ArgumentNullException(nameof(solid));
        if (!solid.IsVolume) throw new ArgumentException("SectionSketch requires a solid volume.", nameof(solid));

        var sketch = new PlotterSketcherCoordSys(ResolveSketchName(name), plane);
        foreach (var strip in IntersectPlane(solid.Mesh, plane))
            AddPatchCurves(sketch, solid.Mesh, plane, strip);
        RegisterSketch(sketch);
        return sketch;
    }

    void AddPatchCurves(PlotterSketcherCoordSys sketch, MeshNormalUV source,
        CoordinateSystem plane, List<IntersectionSegmentEx> strip)
    {
        if (strip.Count == 0) return;
        var pieces = new List<List<IntersectionSegmentEx>>();
        int group = int.MinValue;
        foreach (var segment in strip)
        {
            int nextGroup = source.TrianglesEx[segment.TriIdA].GroupId;
            if (pieces.Count == 0 || nextGroup != group)
            {
                pieces.Add(new List<IntersectionSegmentEx>());
                group = nextGroup;
            }
            pieces[^1].Add(segment);
        }

        // A closed contour can enter its first patch again at the final segment.
        if (pieces.Count > 1 && source.TrianglesEx[pieces[0][0].TriIdA].GroupId ==
            source.TrianglesEx[pieces[^1][0].TriIdA].GroupId)
        {
            pieces[^1].AddRange(pieces[0]);
            pieces.RemoveAt(0);
        }

        foreach (var piece in pieces)
        {
            var points = new List<Vec2D>(piece.Count + 1) { Project(piece[0].StartPoint, plane) };
            for (int i = 0; i < piece.Count; i++) points.Add(Project(piece[i].EndPoint, plane));
            sketch.AddSampledCurve(points);
        }
    }

    List<List<IntersectionSegmentEx>> IntersectPlane(MeshNormalUV source, CoordinateSystem plane)
    {
        var unit = Exact(converter.SmallestUnit());
        var min = converter.OperatingSpace.Min;
        Rat3Hybrid Vector(Vec3D v) => new(Exact(v.X) / unit, Exact(v.Y) / unit, Exact(v.Z) / unit);
        var origin = new Rat3Hybrid((Exact(plane.Origin.X) - Exact(min.X)) / unit,
            (Exact(plane.Origin.Y) - Exact(min.Y)) / unit, (Exact(plane.Origin.Z) - Exact(min.Z)) / unit);
        var x = Vector(plane.X);
        var y = Vector(plane.Y);
        double span = (converter.OperatingSpace.Max - min).Length() * 2;
        Rat3Hybrid Point(double u, double v) => origin + x * Exact(u) + y * Exact(v);
        var cutter = new List<Rat3Hybrid> {
            Point(-span, -span), Point(span, -span), Point(span, span), Point(-span, span) };
        var strips = new List<List<IntersectionSegmentEx>>();
        Resolver.Resolve(BooleanOp.NoOpIntersectionContourOnly, source.PrecisionPositions, source.Triangles,
            cutter, new List<Tri> { new(0, 1, 2), new(0, 2, 3) }, out _, out _, out _, intersectionStrips: strips);
        return strips;
    }

    static BigRationalHybrid Exact(double value)
    {
        var rational = new BigRational(value);
        return new BigRationalHybrid(rational.Numerator, rational.Denominator);
    }

    Vec2D Project(Rat3Hybrid point, CoordinateSystem plane)
    {
        Vec3D relative = converter.Convert(point) - plane.Origin;
        return new Vec2D(Vec3DOps.Dot(relative, plane.X), Vec3DOps.Dot(relative, plane.Y));
    }
}
