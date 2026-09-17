using Geo;
using GeoCore;

namespace GeoTests;

public class CapProjectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ThinTiltedCapUsesAreaInsteadOfBoundingBox(bool reverse)
    {
        // Narrow Y extent is in the cap plane. Dropping Y projects this
        // perfectly valid 3D rectangle onto the line X = Z.
        var positions = new List<Rat3Hybrid> {
            new(100, -1, 100), new(120, -1, 120),
            new(120, 1, 120), new(100, 1, 100)
        };
        var ring = new List<int> { 0, 1, 2, 3 };
        if (reverse) ring.Reverse();
        var rings = new List<List<int>> { ring };
        MeshConstructionHelpers.DeterminePrincipalPlane(positions, rings, out int a, out int b, out bool flip);
        Assert.False(a == 0 && b == 2);
        var triangles = new List<Tri>();
        var groups = new List<int>();
        MeshConstructionHelpers.TriangulateAndEmitCap(positions, rings, 0, false, 7, triangles, groups);
        Assert.Equal(2, triangles.Count);
        Assert.All(groups, group => Assert.Equal(7, group));
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HoleFirstDoesNotInvertTheCap(bool reverse)
    {
        var positions = new List<Rat3Hybrid> {
            new(1,1,0), new(1,2,0), new(2,2,0), new(2,1,0),
            new(0,0,0), new(3,0,0), new(3,3,0), new(0,3,0)
        };
        var rings = new List<List<int>> { new() {0,1,2,3}, new() {4,5,6,7} };
        if (reverse) foreach (var ring in rings) ring.Reverse();
        MeshConstructionHelpers.DeterminePrincipalPlane(positions, rings, out int a, out int b, out bool flip);
        Assert.Equal(0, a);
        Assert.Equal(1, b);
        Assert.Equal(reverse, flip);
    }

    [Theory]
    [InlineData(-2048)]
    [InlineData(2048)]
    public void ProjectionAreaSelectionDoesNotUnderflowOrOverflow(int exponent)
    {
        var power = System.Numerics.BigInteger.One << Math.Abs(exponent);
        var s = exponent < 0 ? new BigRationalHybrid(1, power) : new BigRationalHybrid(power, 1);
        var zero = BigRationalHybrid.Zero;
        var positions = new List<Rat3Hybrid> { new(zero,zero,zero), new(s,zero,zero), new(s,zero,s), new(zero,zero,s) };
        var rings = new List<List<int>> { new() {0,1,2,3} };
        MeshConstructionHelpers.DeterminePrincipalPlane(positions, rings, out int a, out int b, out bool flip);
        Assert.Equal(0, a);
        Assert.Equal(2, b);
        Assert.False(flip);
    }
}
