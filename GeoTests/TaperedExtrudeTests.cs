using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class TaperedExtrudeTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void TaperedRectangleExtrusion_IsWatertightAndNarrowsAtTheTop()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .005);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "profile");
        sketch.AddRectangle(new Vec2D(0, 0), 6, 5);

        var solid = api.Extrude(sketch, 1, .005, "drafted", taperAngle: Math.PI / 18);

        Assert.True(solid.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions, solid.Mesh.Triangles, true));
        double offset = Math.Tan(Math.PI / 18);
        var top = solid.Mesh.Positions.Where(point => point.Z > .99).ToArray();
        Assert.NotEmpty(top);
        Assert.Equal(-3 + offset, top.Min(point => point.X), 2);
        Assert.Equal(3 - offset, top.Max(point => point.X), 2);
        Assert.Equal(-2.5 + offset, top.Min(point => point.Y), 2);
        Assert.Equal(2.5 - offset, top.Max(point => point.Y), 2);
        double topArea = (6 - 2 * offset) * (5 - 2 * offset);
        double expectedVolume = (30 + topArea + Math.Sqrt(30 * topArea)) / 3;
        Assert.Equal(expectedVolume, MeshAnalysis.ComputeSignedMeshVolume(
            solid.Mesh.Positions, solid.Mesh.Triangles), 1);
    }

    [Fact]
    public void TaperedExtrusion_RejectsAProfileWhoseOffsetSplitsOrCollapses()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .01);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "profile");
        sketch.AddRectangle(new Vec2D(0, 0), 1, 1);

        Assert.Throws<InvalidOperationException>(() =>
            api.Extrude(sketch, 10, .01, "collapsed", taperAngle: Math.PI / 4));
    }

    [Fact]
    public void TaperedCircularExtrusion_ShrinksItsTopRadius()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), .01);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "circle");
        sketch.AddCircle(new Vec2D(0, 0), 2);

        var solid = api.Extrude(sketch, 1, .01, "drafted_circle", taperAngle: Math.PI / 18);

        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions, solid.Mesh.Triangles, true));
        var top = solid.Mesh.Positions.Where(point => point.Z > .99).ToArray();
        Assert.NotEmpty(top);
        Assert.Equal(2 - Math.Tan(Math.PI / 18), top.Max(point => Math.Sqrt(point.X * point.X + point.Y * point.Y)), 2);
    }

    [Fact]
    public void TaperedSmallCircle_UsesRefinedClipperOffsetInsteadOfCollapsingAtMeshTolerance()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), .005);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "small_circle");
        sketch.AddCircle(new Vec2D(0, 0), .1);

        var solid = api.Extrude(sketch, -.5, .005, "drafted_small_circle", taperAngle: Math.PI / 18);

        Assert.True(solid.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions, solid.Mesh.Triangles, true));
        var end = solid.Mesh.Positions.Where(point => point.Z < -.49).ToArray();
        Assert.NotEmpty(end);
        Assert.Equal(.1 - .5 * Math.Tan(Math.PI / 18), end.Max(point => Math.Sqrt(point.X * point.X + point.Y * point.Y)), 2);
    }

    [Fact]
    public void TaperedExtrude_OffsetsMixedCurveTypesThroughTheSketchOffsetPipeline()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-8), new Vec3D(8)), .005);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "mixed_profile");
        var a = new Vec2D(-4, 0);
        var b = new Vec2D(-2, 0);
        var c = new Vec2D(0, 0);
        var d = new Vec2D(2, 0);
        var e = new Vec2D(2, 2);
        var f = new Vec2D(0, 3);
        var g = new Vec2D(-2, 2);
        sketch.SetCurves(new List<List<Curve2D>>
        {
            new()
            {
                new Line2D(a, b) { Name = "line" },
                new Arc2D(b, new Vec2D(-1, 1), c) { Name = "arc" },
                new Bezier2D(c, new Vec2D(.5, -.5), new Vec2D(1.5, -.5), d) { Name = "bezier" },
                new CubicHermiteSpline2D(new[] { d, new Vec2D(2.5, .8), e }) { Name = "hermite" },
                new BSpline2D(
                    new[] { e, new Vec2D(1.4, 2.6), new Vec2D(.6, 2.8), f },
                    new[] { 0.0, 0.0, 0.0, .5, 1.0, 1.0, 1.0 }, 2) { Name = "bspline" },
                new SampledCurve(
                    new List<Vec2D> { f, new Vec2D(-.7, 2.5), g },
                    new List<Vec2D> { new(0, 1), new(-1, 0), new(-1, -1) }) { Name = "sampled" },
                new Line2D(g, a) { Name = "closing_line" }
            }
        });

        var solid = api.Extrude(sketch, 1, .005, "mixed_draft", taperAngle: Math.PI / 36);

        Assert.True(solid.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions, solid.Mesh.Triangles, true));
    }

    [Fact]
    public void TaperedExtrude_OffsetsEllipseProfilesThroughTheSketchOffsetPipeline()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), .005);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "ellipse_profile");
        sketch.AddEllipse(new Vec2D(0, 0), new Vec2D(2, 0), 1);

        var solid = api.Extrude(sketch, .5, .005, "drafted_ellipse", taperAngle: Math.PI / 36);

        Assert.True(solid.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions, solid.Mesh.Triangles, true));
    }
}
