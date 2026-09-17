using GeoCore;

namespace GeoTests;

public class UVSurfacePlanarityTests
{
    [Theory]
    [InlineData(.6)]
    [InlineData(-.6)]
    public void PlanarOffsetIsExactlyPerpendicularToBothRationalTangents(double distance)
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        var origin = new Rat3Hybrid(500000, 500000, 500000);
        var first = new Rat3Hybrid(4000, 3000, 0);
        var second = new Rat3Hybrid(-5000, 0, 3000);
        var points = new List<Rat3Hybrid> { origin, origin + first, origin + second };
        var normal = new Vec3D(3, -4, 5).Normalized();
        var surface = new UVSurface(converter.Convert(points), Enumerable.Repeat(normal, 3).ToList(),
            new List<Vec2D> { new(0, 0), new(1, 0), new(0, 1) }, new List<Tri> { new(0, 1, 2) }, points);
        var shifted = surface.GetOffsetSurface(distance, converter);
        var displacement = shifted.PointsPrecise[0] - points[0];
        Assert.True(Rat3Hybrid.Dot(displacement, first) == BigRationalHybrid.Zero);
        Assert.True(Rat3Hybrid.Dot(displacement, second) == BigRationalHybrid.Zero);
        Assert.InRange(Vec3DOps.Dot(shifted.Points[0] - surface.Points[0], normal), distance - 1e-12, distance + 1e-12);
        Assert.Equal(origin, surface.PointsPrecise[0]);
        Assert.True(shifted.IsSurfacePlanar());
    }

    /// <summary>
    /// Loft caps and other meshes can contain float-valid triangles that become colinear in exact lattice space
    /// after <see cref="CoordinateConverter"/> quantization. Planarity checks must skip those, not throw.
    /// </summary>
    [Fact]
    public void IsSurfacePlanar_SkipsExactDegenerateTriangles_NoThrowAndTrueWhenCoplanar()
    {
        var pts = new List<Vec3D>
        {
            new Vec3D(0, 0, 0),
            new Vec3D(1, 0, 0),
            new Vec3D(2, 0, 0),
            new Vec3D(0, 1, 0)
        };
        var precise = new List<Rat3Hybrid>
        {
            new Rat3Hybrid(0, 0, 0),
            new Rat3Hybrid(1, 0, 0),
            new Rat3Hybrid(2, 0, 0),
            new Rat3Hybrid(0, 1, 0)
        };
        var tris = new List<Tri>
        {
            new Tri(0, 1, 2),
            new Tri(0, 1, 3)
        };
        var surf = new UVSurface(pts, null, null, tris, precise);
        Assert.True(surf.IsSurfacePlanar());
    }

    [Fact]
    public void IsSurfacePlanar_AllExactDegenerate_ReturnsFalse()
    {
        var pts = new List<Vec3D>
        {
            new Vec3D(0, 0, 0),
            new Vec3D(1, 0, 0),
            new Vec3D(2, 0, 0)
        };
        var precise = new List<Rat3Hybrid>
        {
            new Rat3Hybrid(0, 0, 0),
            new Rat3Hybrid(1, 0, 0),
            new Rat3Hybrid(2, 0, 0)
        };
        var tris = new List<Tri> { new Tri(0, 1, 2) };
        var surf = new UVSurface(pts, null, null, tris, precise);
        Assert.False(surf.IsSurfacePlanar());
    }

    [Fact]
    public void IsSurfacePlanar_NullPrecise_ReturnsFalse()
    {
        var pts = new List<Vec3D> { new Vec3D(0, 0, 0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0) };
        var tris = new List<Tri> { new Tri(0, 1, 2) };
        var surf = new UVSurface(pts, null, null, tris, (List<Rat3Hybrid>?)null);
        Assert.False(surf.IsSurfacePlanar());
    }
    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(-1, false)]
    [InlineData(-1, true)]
    public void DisconnectedParallelTrianglesMustShareTheSamePlane(int offset, bool reverse)
    {
        var precise = new List<Rat3Hybrid>
        {
            new(0,0,0),new(1,1,0),new(0,1,1),
            new(4+offset,4-offset,offset),
            new(5+offset,5-offset,offset),
            new(4+offset,5-offset,1+offset)
        };
        var points = precise.Select(p => new Vec3D(p.X.ToDouble(),p.Y.ToDouble(),p.Z.ToDouble())).ToList();
        var triangles = new List<Tri> { new(0,1,2),reverse ? new(3,5,4) : new(3,4,5) };
        var surface = new UVSurface(points,null,null,triangles,precise);
        Assert.Equal(offset == 0,surface.IsSurfacePlanar());
    }
}
