using GeoCore;

namespace GeoTests;

public class BezierFlatnessTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TranslatedStraightBezierNeedsOnlyEndpoints(bool nurbs)
    {
        Vec2D[] controls = [new(10, 7), new(13, 7), new(17, 7), new(20, 7)];
        var points = nurbs ? NURBS.BezierTessellator.Tessellate(.01, controls)
            : Curves.BezierTessellator.Tessellate(.01, controls);
        Assert.Equal(2, points.Count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClosedEndpointCurveCannotCollapseAndControlsStayUnchanged(bool nurbs)
    {
        Vec3D[] controls = [new(0, 0, 0), new(0, 3, 0), new(3, 3, 0), new(0, 0, 0)];
        var original = controls.ToArray();
        var points = nurbs ? NURBS.BezierTessellator.Tessellate(.01, controls)
            : Curves.BezierTessellator.Tessellate(.01, controls);
        Assert.True(points.Count > 2);
        Assert.Equal(original, controls);
        Assert.Equal(original[0], points[0]);
        Assert.Equal(original[^1], points[^1]);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CollinearOvershootIsPreserved(bool nurbs)
    {
        Vec3D[] controls = [new(0, 0, 0), new(4, 0, 0), new(4, 0, 0), new(1, 0, 0)];
        var original = controls.ToArray();
        var points = nurbs ? NURBS.BezierTessellator.Tessellate(.01, controls)
            : Curves.BezierTessellator.Tessellate(.01, controls);
        Assert.Contains(points, p => p.X > 3);
        Assert.Equal(original, controls);
    }
}
