using Geo;
using GeoCore;

namespace GeoTests;

public class SketchSolveResultTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DetailedSolveReportsInconsistentFixedPoint(bool inconsistent)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .01);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "constraints");
        sketch.SolveAfterEveryConstraint = false;
        var line = sketch.AddCLine(new Vec2D(0, 0), new Vec2D(2, 0));
        sketch.FixPoint(line.CStart, new Vec2D(0, 0));
        sketch.FixPoint(line.CEnd, new Vec2D(2, 0));
        if (inconsistent)
            sketch.FixPoint(line.CStart, new Vec2D(1, 0));

        var result = sketch.SolveConstraintsDetailed();
        Assert.Equal(!inconsistent, result.Converged);
        if (inconsistent)
        {
            Assert.True(result.SumOfSquaredErrors > .1);
            Assert.False(string.IsNullOrWhiteSpace(result.Message));
        }
        else
        {
            Assert.Equal(0, result.SumOfSquaredErrors);
            Assert.Equal(new Vec2D(2, 0), line.CEnd.Evaluate());
        }
    }
}
