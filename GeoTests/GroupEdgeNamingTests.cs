using Geo;
using GeoCore;

namespace GeoTests;

public sealed class GroupEdgeNamingTests
{
    [Fact]
    public void DisconnectedEdgesBetweenSameFacesAreAllNumbered()
    {
        var points = new List<Vec3D>
        {
            new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, -1, 0),
            new(3, 0, 0), new(4, 0, 0), new(3, 1, 0), new(3, -1, 0)
        };
        var triangles = new List<Tri>
        {
            new(0, 1, 2), new(1, 0, 3), new(4, 5, 6), new(5, 4, 7)
        };
        var groups = new List<int> { 1, 2, 1, 2 };
        var names = new Dictionary<int, string> { [1] = "A", [2] = "B" };

        var edges = GroupEdgeExtractor.ExtractGroupEdges(triangles, groups, points, null, names);
        Assert.Equal(2, edges.Count);
        Assert.Equal(new[] { "[A,B]_1", "[A,B]_2" }, edges.Select(edge => edge.Name));

        var graph = new EdgeGraph(triangles, groups, points, null, names);
        Assert.False(graph.TryGetEdge("[A,B]", out _));
        Assert.True(graph.TryGetEdge("[A,B]_1", out _));
        Assert.True(graph.TryGetEdge("[A,B]_2", out _));
    }
}
