using GeoCore;
using NURBS;

namespace GeoTests;

public class NurbsTrimDomainTests
{
    [Fact]
    public void AdaptiveSurfaceTessellationHonorsSuppliedTrimLoop()
    {
        var surface = new BSplineSurface(1, 1,
            new Vec3D[][] { [new(0, 0, 0), new(0, 1, 0)], [new(1, 0, 0), new(1, 1, 0)] },
            new[] { 0.0, 0, 1, 1 }, new[] { 0.0, 0, 1, 1 });
        List<Vec2D> border = [new(.2, .2), new(.8, .2), new(.8, .8), new(.2, .8)];
        var result = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface, [border]);
        Assert.NotEmpty(result.Triangles);
        foreach (var uv in result.UV)
        {
            Assert.InRange(uv.X, .2 - 1e-12, .8 + 1e-12);
            Assert.InRange(uv.Y, .2 - 1e-12, .8 + 1e-12);
        }
        double area = result.Triangles.Sum(t => Vec3DOps.Cross(
            result.Points[t.B] - result.Points[t.A], result.Points[t.C] - result.Points[t.A]).Length() / 2);
        Assert.InRange(area, .36 - 1e-12, .36 + 1e-12);
    }
}
