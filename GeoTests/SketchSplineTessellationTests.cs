using Curves;
using GeoCore;

namespace GeoTests;

public class SketchSplineTessellationTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StraightSplineNeedsOnlyEndpoints(bool rational)
    {
        var controls = new[] { new Vec2D(0, 0), new Vec2D(3, 0), new Vec2D(7, 0), new Vec2D(10, 0) };
        var knots = new double[] { 2, 2, 2, 2, 5, 5, 5, 5 };
        var spline = rational
            ? new BSpline2D(controls, knots, 3, new double[] { 1, 2, 3, 1 })
            : new BSpline2D(controls, knots, 3);
        var vertices = spline.Tessellate(.001);
        Assert.Equal(2, vertices.Count);
        Assert.Equal(new Vec2D(0, 0), vertices[0].Position);
        Assert.Equal(new Vec2D(10, 0), vertices[^1].Position);
        Assert.Equal(new double[] { 2, 2, 2, 2, 5, 5, 5, 5 }, knots);
    }

    [Fact]
    public void RationalArcRespondsToToleranceAndKeepsExactEndpoints()
    {
        const double radius = 1000;
        var spline = new BSpline2D(
            new[] { new Vec2D(radius, 0), new Vec2D(radius, radius), new Vec2D(0, radius) },
            new double[] { 0, 0, 0, 1, 1, 1 }, 2,
            new double[] { 1, Math.Sqrt(.5), 1 });
        var coarse = spline.Tessellate(1.0);
        var fine = spline.Tessellate(.01);
        Assert.True(fine.Count > coarse.Count);
        Assert.Equal(new Vec2D(radius, 0), fine[0].Position);
        Assert.Equal(new Vec2D(0, radius), fine[^1].Position);
        foreach (var (a, b) in fine.Zip(fine.Skip(1)))
        {
            // The midpoint of a circular arc chord is its maximum radial error.
            var midpoint = (a.Position + b.Position) * .5;
            Assert.InRange(radius - midpoint.Length(), 0, .01 + 1e-9);
            Assert.True(b.Uniform > a.Uniform);
            Assert.InRange(Math.Abs(a.Position.Length() - radius), 0, 1e-9);
        }
    }

    [Fact]
    public void ExplicitSampleCountIsStillHonored()
    {
        var spline = new BSpline2D(new[] { new Vec2D(0, 0), new Vec2D(10, 0) },
            new double[] { 0, 0, 1, 1 }, 1);
        Assert.Equal(8, spline.Tessellate(8).Count);
    }
}
