using GeoCore;

namespace GeoTests;

public class UVSurfacePlanarityTests
{
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
}
