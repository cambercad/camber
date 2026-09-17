using Curves;
using Geo;
using Geo.NurbsConstruction;
using GeoCore;
using NURBS;

namespace GeoTests;

public class SurfaceLoftTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);
    private static BSplineCurve Section(double z) => new(1,
        new[] { new Vec3D(0, 0, z), new Vec3D(10, 0, z) }, new double[] { 0, 0, 1, 1 });
    private static void Near(Vec3D a, Vec3D b) => Assert.InRange((a - b).Length(), 0, 1e-10);

    [Fact]
    public void UnguidedLoftInterpolatesEverySection()
    {
        BSplineCurve[] sections = [Section(0), Section(3), Section(10)];
        var surface = SurfaceLoft.Build(sections);
        for (int p = 0; p < sections.Length; p++)
            for (int i = 0; i <= 20; i++)
                Near(sections[p].EvaluateUniform(i / 20.0), surface.Evaluate(i / 20.0, p / 2.0));
    }

    [Fact]
    public void OptionalEndDerivativesControlBothEndsIncludingTwoSectionLofts()
    {
        var start = new Vec3D(0, 12, 10);
        var end = new Vec3D(0, -7, 10);
        var surface = SurfaceLoft.Build([Section(0), Section(10)], startTangent: start, endTangent: end);
        foreach (double u in new[] { 0, .25, .5, 1 })
        {
            Near(surface.EvaluateDV(u, 0), start);
            Near(surface.EvaluateDV(u, 1), end);
            Near(surface.Evaluate(u, 0), new Vec3D(10 * u, 0, 0));
            Near(surface.Evaluate(u, 1), new Vec3D(10 * u, 0, 10));
        }
        var multiple = SurfaceLoft.Build([Section(0), Section(4), Section(10)], startTangent: start, endTangent: end);
        Near(multiple.EvaluateDV(.5, 0), start);
        Near(multiple.EvaluateDV(.5, 1), end);
        Near(multiple.EvaluateDV(.5, Math.BitDecrement(.5)), multiple.EvaluateDV(.5, Math.BitIncrement(.5)));
        Assert.True(surface.Evaluate(.5, .5).Y > 2);
    }

    [Fact]
    public void BoundaryRailIsPreservedBetweenSectionsAndKeepsInteriorSections()
    {
        var guide = new CubicHermiteSpline3D([new(0, 0, 0), new(0, 0, 5), new(0, 0, 10)],
            new Vec3D?[] { new(0, 1, 1), new(0, -1, 1), new(0, 1, 1) });
        var surface = SurfaceLoft.Build([Section(0), Section(5), Section(10)], [guide]);
        for (int i = 0; i <= 100; i++)
            Near(guide.Evaluate(i / 100.0).Origin, surface.Evaluate(0, i / 100.0));
        for (int p = 0; p < 3; p++)
            for (int i = 0; i <= 20; i++)
                Near(surface.Evaluate(i / 20.0, p / 2.0), new Vec3D(i / 2.0, 0, p * 5));
    }

    [Fact]
    public void RejectsMissedSectionsDuplicateRailsAndConflictingTangents()
    {
        BSplineCurve[] sections = [Section(0), Section(5), Section(10)];
        var rail = new Line3D(new(0, 0, 0), new(0, 0, 10));
        Assert.Throws<ArgumentException>(() => SurfaceLoft.Build(sections, [rail, rail]));
        Assert.Throws<ArgumentException>(() => SurfaceLoft.Build([Section(0), Section(4), Section(10)], [rail]));
        Assert.Throws<ArgumentException>(() => SurfaceLoft.Build(sections, [rail], startTangent: new Vec3D(0, 1, 10)));
        Assert.Throws<ArgumentException>(() => SurfaceLoft.Build(sections, startTangent: new Vec3D()));
        Assert.Throws<ArgumentException>(() => SurfaceLoft.Build(sections, endTangent: new Vec3D(double.NaN, 0, 0)));
        SurfaceLoft.Build(sections, [rail], startTangent: new Vec3D(0, 0, 10), endTangent: new Vec3D(0, 0, 10));
    }

    [Fact]
    public void UnifiesSectionKnotsWithoutFittingAndRejectsIncompatibleWeights()
    {
        var a = new BSplineCurve(1, new[] { new Vec3D(0, 0, 0), new Vec3D(5, 2, 0), new Vec3D(10, 0, 0) }, new double[] { 0, 0, .5, 1, 1 });
        var b = new BSplineCurve(1, a.ControlPoints, new double[] { 0, 0, .7, 1, 1 });
        var surface = SurfaceLoft.Build([a, b]);
        for (int i = 0; i <= 100; i++)
        {
            Near(a.EvaluateUniform(i / 100.0), surface.Evaluate(i / 100.0, 0));
            Near(b.EvaluateUniform(i / 100.0), surface.Evaluate(i / 100.0, 1));
        }
        b = new BSplineCurve(1, new[] { new Vec3D(0, 0, 10), new Vec3D(10, 0, 10) }, new double[] { 1, 2 }, new double[] { 0, 0, 1, 1 });
        Assert.Throws<ArgumentException>(() => SurfaceLoft.Build([Section(0), b]));
    }

    [Fact]
    public void CompatibleRationalSectionsRemainExact()
    {
        var sections = new[] { 0.0, 5.0, 10.0 }.Select(z =>
            new BSplineCurve(2, new[] { new Vec3D(1, 0, z), new Vec3D(1, 1, z), new Vec3D(0, 1, z) },
                new[] { 1, Math.Sqrt(.5), 1 }, new double[] { 0, 0, 0, 1, 1, 1 })).ToArray();
        var surface = SurfaceLoft.Build(sections);
        for (int j = 0; j < sections.Length; j++)
            for (int i = 0; i <= 50; i++)
                Near(sections[j].EvaluateUniform(i / 50.0), surface.Evaluate(i / 50.0, j / 2.0));
    }

    [Fact]
    public void SheetUsesExistingAdaptiveMesherAndRetainsUvSupport()
    {
        var surface = SurfaceLoft.Build([Section(0), Section(10)], startTangent: new Vec3D(0, 10, 10));
        var mesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface, maxDeviation: .01);
        Assert.NotEmpty(mesh.Triangles);
        for (int i = 0; i < mesh.UV.Count; i++)
            Near(mesh.Points[i], surface.Evaluate(mesh.UV[i].X, mesh.UV[i].Y));
    }
    [Fact]
    public void PublicOperationProducesSheetWithStablePatchAndAnalyticSupport()
    {
        var part = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .01);
        var sections = new[] { 0.0, 10.0 }.Select(z =>
        {
            var sketch = new PlotterSketcherCoordSys("section" + z,
                new CoordinateSystem(new Vec3D(0, 0, z)), new Vec2D(0, 0));
            sketch.AppendLine(10, 0);
            return sketch;
        }).ToArray();
        var sheet = part.LoftSurface(sections, startTangent: new Vec3D(0, 8, 10), name: "blade");
        Assert.False(sheet.IsVolume);
        Assert.NotEmpty(sheet.Mesh.Triangles);
        Assert.Contains("blade-Side", sheet.surfaceMetaData.Keys);
        Assert.True(sheet.TryGetSurface("blade-Side", out var patch));
        Assert.NotNull(patch.NurbsSurface);
        for (int i = 0; i < patch.Uv.Count; i++)
            Assert.InRange((patch.Points[i] - patch.NurbsSurface.Evaluate(patch.Uv[i].X, patch.Uv[i].Y)).Length(), 0, .0001);
    }

    [Fact]
    public void MixedDegreesAreElevatedWithoutChangingTheSections()
    {
        var line = Section(0);
        var cubic = new BSplineCurve(3,
            new[] { new Vec3D(0, 0, 10), new Vec3D(2, 4, 10), new Vec3D(8, 4, 10), new Vec3D(10, 0, 10) },
            new double[] { 0, 0, 0, 0, 1, 1, 1, 1 });
        var surface = SurfaceLoft.Build([line, cubic]);
        for (int i = 0; i <= 100; i++)
        {
            Near(line.EvaluateUniform(i / 100.0), surface.Evaluate(i / 100.0, 0));
            Near(cubic.EvaluateUniform(i / 100.0), surface.Evaluate(i / 100.0, 1));
        }
    }

}
