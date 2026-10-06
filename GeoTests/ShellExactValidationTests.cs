using Geo;
using Geo.Shelling;
using GeoCore;

namespace GeoTests;

public sealed class ShellExactValidationTests
{
    private static MeshNormalUV Tetrahedron()
    {
        var origin = new Rat3Hybrid(new BigRationalHybrid(1L << 60), new(0), new(0));
        var points = new List<Rat3Hybrid> { origin, origin + new Rat3Hybrid(1, 0, 0),
            origin + new Rat3Hybrid(0, 1, 0), origin + new Rat3Hybrid(0, 0, 1) };
        return new MeshNormalUV {
            PrecisionPositions = points,
            Positions = points.Select(p => new Vec3D(p.X.ToDouble(), p.Y.ToDouble(), p.Z.ToDouble())).ToList(),
            Triangles = new List<Tri> { new(0, 2, 1), new(0, 1, 3), new(0, 3, 2), new(1, 2, 3) }
        };
    }

    [Fact]
    public void RoundedDisplayCoordinatesDoNotCollapseAnExactOffset()
    {
        var source = Tetrahedron();
        // X+1 rounds back to X in the display representation, but the exact
        // tetrahedron is watertight and has strictly positive volume.
        Assert.Equal(source.Positions[0], source.Positions[1]);
        Assert.Equal(0, MeshAnalysis.ComputeSignedMeshVolume(source.Positions, source.Triangles));
        ShellTopologyBuilder.ValidateOffset(source, Tetrahedron(), requireClosedVolume: true);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactCollapseAndReversalAreRejected(bool reverse)
    {
        var source = Tetrahedron();
        var offset = Tetrahedron();
        if (reverse) offset.Triangles[0] = new Tri(0, 1, 2);
        else offset.PrecisionPositions[1] = offset.PrecisionPositions[0];
        var error = Assert.Throws<InvalidOperationException>(() =>
            ShellTopologyBuilder.ValidateOffset(source, offset, requireClosedVolume: false));
        Assert.Contains(reverse ? "inverted" : "collapsed", error.Message);
    }

    [Theory]
    [InlineData(-1, 1)]
    [InlineData(0, 1)]
    [InlineData(1, -1)]
    public void FilteredVolumeComparisonMatchesExactVolume(int threshold, int expected)
    {
        var mesh = Tetrahedron();
        Assert.Equal(expected, MeshAnalysis.CompareSignedMeshVolume(mesh.PrecisionPositions, mesh.Triangles, new(threshold)));
        var exact = MeshAnalysis.ComputeSignedMeshVolume(mesh.PrecisionPositions, mesh.Triangles);
        Assert.Equal(0, MeshAnalysis.CompareSignedMeshVolume(mesh.PrecisionPositions, mesh.Triangles, exact));
        mesh.Triangles.Clear();
        Assert.Equal(-Math.Sign(threshold), MeshAnalysis.CompareSignedMeshVolume(mesh.PrecisionPositions, mesh.Triangles, new(threshold)));
    }
}
