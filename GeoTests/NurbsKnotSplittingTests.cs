using Curves;
using Geo;
using GeoSolver;
using GeoCore;
using NURBS;

namespace GeoTests;

public class NurbsKnotSplittingTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    private static BSplineCurve Circle() => new BSplineCircle(new Vec3D(0, 0, 0),
        new Vec3D(0, 0, 1), .7, new Vec3D(1, 0, 0)).GetNativeParamRangeCurve();

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void Split_AdjacentToRepeatedKnot_PreservesParametricCurve(int direction)
    {
        var curve = Circle();
        double u = direction < 0 ? Math.BitDecrement(.25) : direction > 0 ? Math.BitIncrement(.25) : .25;
        curve.Split(u, out var lower, out var upper);
        for (int i = 0; i <= 32; i++)
        {
            double t = i / 32.0;
            Assert.InRange((lower.EvaluateUniform(t) - curve.EvaluateUniform(u * t)).Length(), 0, 2e-14);
            Assert.InRange((upper.EvaluateUniform(t) - curve.EvaluateUniform(u + (1-u)*t)).Length(), 0, 2e-14);
        }
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void InsertKnot_AdjacentToRepeatedKnot_PreservesBothDistinctParameters(int direction)
    {
        var curve = Circle();
        double u = direction < 0 ? Math.BitDecrement(.25) : Math.BitIncrement(.25);
        var inserted = curve.InsertKnot(u);
        Assert.Contains(u, inserted.Knots);
        Assert.Equal(2, inserted.Knots.Count(k => k == .25));
        for (int i = 0; i <= 128; i++)
            Assert.InRange((inserted.EvaluateUniform(i/128.0) - curve.EvaluateUniform(i/128.0)).Length(), 0, 2e-14);
    }

    [Fact]
    public void ExtractRange_NearQuarterKnots_PreservesRequestedEndpointsAndShape()
    {
        var curve = Circle();
        double start = Math.BitIncrement(.25), end = Math.BitDecrement(.75);
        var section = curve.ExtractRange(start, end);
        for (int i = 0; i <= 32; i++)
        {
            double t = i / 32.0;
            Assert.InRange((section.EvaluateUniform(t) - curve.EvaluateUniform(start + (end-start)*t)).Length(), 0, 2e-14);
        }
    }

    [Fact]
    public void Loft_RoundedStockTransition_ProducesWatertightGeometryAndMetadata()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20, -40, -25), new Vec3D(250, 40, 40)), .035);
        double a = 31.3 / 2, b = 6.6 / 2, r = .7, d = r / Math.Sqrt(2);
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (double z in new[] { 0.0, 5.0 })
        {
            var frame = new CoordinateSystem(new Vec3D(0, 0, z),
                new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
            var sketch = new PlotterSketcherCoordSys($"section{z}", frame, new Vec2D(-a+r, -b));
            sketch.AppendLine(a-r, -b);
            sketch.AddArc(new Vec2D(a-r, -b), new Vec2D(a-r+d, -b+r-d), new Vec2D(a, -b+r));
            sketch.AddLine(new Vec2D(a, -b+r), new Vec2D(a, b-r));
            sketch.AddArc(new Vec2D(a, b-r), new Vec2D(a-r+d, b-r+d), new Vec2D(a-r, b));
            sketch.AddLine(new Vec2D(a-r, b), new Vec2D(-a+r, b));
            sketch.AddArc(new Vec2D(-a+r, b), new Vec2D(-a+r-d, b-r+d), new Vec2D(-a, b-r));
            sketch.AddLine(new Vec2D(-a, b-r), new Vec2D(-a, -b+r));
            sketch.AddArc(new Vec2D(-a, -b+r), new Vec2D(-a+r-d, -b+r-d), new Vec2D(-a+r, -b));
            sections.Add(sketch);
        }
        var loft = api.Loft(sections, new LoftOptions { CapEnds = true }, "rounded_stock", .035);
        Assert.True(MeshAnalysis.IsWatertightMesh(loft.Mesh.Positions, loft.Mesh.Triangles));
        foreach (var name in NurbsPatchValidation.AllNamedPatches(loft))
            Assert.NotNull(loft.surfaceMetaData[name].NurbsSurface);
    }
}
