using GeoCore;

namespace GeoTests;

public sealed class MeshConnectivityTests
{
    [Fact]
    public void ExactPositionComponents_TreatsVertexTouchingClosedMeshesAsSeparateBodies()
    {
        var positions = new List<Rat3Hybrid>
        {
            new(0, 0, 0), new(2, 0, 0), new(0, 2, 0), new(0, 0, 2),
            // This tetrahedron has a distinct vertex index at the same exact position.
            new(0, 0, 0), new(-2, 0, 0), new(0, -2, 0), new(0, 0, -2)
        };
        var tetrahedron = new[]
        {
            new Tri(0, 2, 1), new Tri(0, 1, 3), new Tri(1, 2, 3), new Tri(2, 0, 3)
        };
        var triangles = new List<Tri>(tetrahedron);
        foreach (var triangle in tetrahedron)
            triangles.Add(new Tri(triangle.A + 4, triangle.B + 4, triangle.C + 4));

        var components = MeshConnectivity.FindExactPositionComponents(positions, triangles);

        Assert.Equal(2, components.Count);
        Assert.All(components, component => Assert.Equal(4, component.Count));
    }

    [Fact]
    public void ExactPositionComponents_JoinsTrianglesAcrossAnExactlySharedEdge()
    {
        var positions = new List<Rat3Hybrid>
        {
            new(0, 0, 0), new(1, 0, 0), new(0, 1, 0),
            new(1, 0, 0), new(0, 0, 0), new(0, 0, 1)
        };
        var triangles = new List<Tri> { new(0, 1, 2), new(3, 4, 5) };

        var components = MeshConnectivity.FindExactPositionComponents(positions, triangles);

        var component = Assert.Single(components);
        Assert.Equal(2, component.Count);
    }
}
