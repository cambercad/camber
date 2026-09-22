using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class SectionSketchTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void CuboidMidPlaneProducesOneSampledCurvePerSidePatch()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .01);
        var block = api.CreateCuboid(new Vec3D(-2, -3, -1), new Vec3D(2, 3, 1), "block");
        var sketch = api.SectionSketch(block, CoordinateSystem.Default, "slice");

        Assert.Equal("slice", sketch.Name);
        var curves = sketch.GetAllCurves().Select(Assert.IsType<SampledCurve>).ToList();
        Assert.Equal(4, curves.Count);
        Assert.All(curves, curve => Assert.True(curve.Points.Count >= 2));
    }
}
