using GeoCore;

namespace GeoTests;

public class ExactSurfaceOffsetTests
{
    [Theory]
    [InlineData(0.0)]
    [InlineData(.125)]
    public void NonplanarOffsetRetainsDistinctSubgridVertices(double distance)
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)), 10000);
        var precise = new List<Rat3Hybrid>
        {
            new(0, 0, 0), new(new BigRationalHybrid(1, 1024), new BigRationalHybrid(0), new BigRationalHybrid(0)),
            new(0, 100, 0), new(0, 100, 100)
        };
        var normals = new List<Vec3D> { new(0, 0, 1), new(0, 0, 1), new(0, 0, 1), new(1, 0, 0) };
        var triangles = new List<Tri> { new(0, 1, 2), new(1, 3, 2) };
        var surface = new UVSurface(converter.Convert(precise), normals,
            Enumerable.Repeat(new Vec2D(0, 0), 4).ToList(), triangles, precise);
        Assert.False(surface.IsSurfacePlanar());
        var offset = surface.GetOffsetSurface(distance, converter);
        var separation = offset.PointsPrecise[1] - offset.PointsPrecise[0];
        separation.Simplify();
        Assert.Equal(precise[1] - precise[0], separation);
        Assert.Equal(4, offset.PointsPrecise.Distinct().Count());
        Assert.Equal(triangles, offset.Triangles);
        for (int i = 0; i < precise.Count; i++)
            Assert.InRange((offset.Points[i] - surface.Points[i] - normals[i] * distance).Length(), 0, 1e-12);
    }
}
