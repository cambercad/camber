using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class CubicHermiteGuideTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void ClosedAsymmetricChordsShareOneSeamDerivativeAndCopiesPreserveIt()
    {
        Vec3D[] points = [new(0), new(4, 0, 1), new(0, 2, 0), new(0)];
        Vec3D?[] directions = [new Vec3D(1, 0, 0), new Vec3D(0, 1, 1), new Vec3D(-1, 0, -1), new Vec3D(7, 0, 0)];
        var curve = new CubicHermiteSpline3D(points, directions, "periodic");
        double magnitude = .5 * ((points[1] - points[0]).Length() + (points[^1] - points[^2]).Length());
        Assert.True((curve.Tangents[0] - new Vec3D(magnitude, 0, 0)).Length() < 1e-12);
        Assert.Equal(curve.Tangents[0], curve.Tangents[^1]);
        Assert.Equal(curve.Tangents, curve.GetCopy().Tangents);
        Assert.Equal(curve.Evaluate(0).Origin, curve.Evaluate(1).Origin);
        Assert.Equal(curve.Evaluate(0).Tangent, curve.Evaluate(1).Tangent);
    }

    [Fact]
    public void ClosedMismatchedDirectionsAreRejected()
    {
        Assert.Throws<ArgumentException>(() => new CubicHermiteSpline3D(
            new[] { new Vec3D(0), new Vec3D(4, 0, 1), new Vec3D(0, 2, 0), new Vec3D(0) },
            new Vec3D?[] { new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(-1, 0, 0), new Vec3D(0, -1, 0) }));
    }

    [Fact]
    public void ParallelReferenceDirectionFailsClearly()
    {
        var curve = new Line3D(new Vec3D(0), new Vec3D(0, 0, 10));
        var sketch = new PlotterSketcherCoordSys("profile", CoordinateSystem.Default, new Vec2D(0));
        sketch.AddRectangle(new Vec2D(0), 1, 1);
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .03);
        Assert.Throws<ArgumentException>(() => api.ExtrudeAlongCurve(sketch, curve, referenceDirection: new Vec3D(0, 0, 1)));
    }

    [Fact]
    public void OpenDirectionsRetainExistingChordLengthConvention()
    {
        var curve = new CubicHermiteSpline3D(new[] { new Vec3D(0), new Vec3D(2, 0, 0), new Vec3D(2, 3, 0) },
            new Vec3D?[] { new Vec3D(7, 0, 0), new Vec3D(1, 1, 0), new Vec3D(0, 8, 0) });
        Assert.Equal(2, curve.Tangents[0].Length(), 12);
        Assert.Equal(2.5, curve.Tangents[1].Length(), 12);
        Assert.Equal(3, curve.Tangents[2].Length(), 12);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PeriodicNonplanarGuideSweepsWithoutEndCaps(bool reference)
    {
        const int spans = 24;
        var points = new List<Vec3D>();
        var directions = new List<Vec3D?>();
        for (int i = 0; i < spans; i++)
        {
            double a = 2 * Math.PI * i / spans;
            points.Add(new Vec3D(20 * Math.Cos(a), 20 * Math.Sin(a), 2 * Math.Sin(2 * a)));
            directions.Add(new Vec3D(-20 * Math.Sin(a), 20 * Math.Cos(a), 4 * Math.Cos(2 * a)));
        }
        points.Add(points[0]);
        directions.Add(directions[0]);
        var curve = new CubicHermiteSpline3D(points, directions, "cam_guide");
        var tangent = curve.Evaluate(0).Tangent;
        var radial = new Vec3D(1, 0, 0);
        var lateral = Vec3DOps.Cross(tangent, radial);
        var sketch = new PlotterSketcherCoordSys("profile",
            new CoordinateSystem(points[0], radial, lateral, tangent), new Vec2D(0));
        sketch.AddRectangle(new Vec2D(.25, -.2), 1, .6);
        var api = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), .03);
        var solid = api.ExtrudeAlongCurve(sketch, curve, maxDeviation: .03, name: "periodic_cam",
            referenceDirection: reference ? new Vec3D(0, 0, 1) : null);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(solid.Mesh);
        Assert.DoesNotContain(solid.groupIdToExtendedName.Values, n => n.EndsWith("ExtrudeTop") || n.EndsWith("ExtrudeBottom"));
        double volume = Math.Abs(solid.Mesh.Triangles.Sum(t => Vec3DOps.Dot(
            solid.Mesh.Positions[t.A],
            Vec3DOps.Cross(solid.Mesh.Positions[t.B], solid.Mesh.Positions[t.C])))) / 6;
        Assert.InRange(volume, 70, 85);
        var samples = curve.Tessellate(.03);
        Assert.True((samples[0].Up - samples[^1].Up).Length() < 1e-12);
        for (int i = 1; i < samples.Count; i++)
            Assert.True(Vec3DOps.Dot(samples[i - 1].Up, samples[i].Up) > .98);
    }
}
