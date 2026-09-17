using GeoCore;
using Remeshing;

namespace GeoTests;

public class CollinearFaceJunctionTests
{
    [Fact]
    public void CollinearEdgesDoNotEraseAnAdditionalFaceJunction()
    {
        var points = new List<Rat3Hybrid>
        {
            new(0, 0, 0), new(-1, 0, 0), new(1, 0, 0), new(-1, 1, 0), new(1, 1, 0), new(0, -1, 0)
        };
        // Edges 1 - 0 and 0 - 2 have the same adjacent group, but group 2
        // terminates at 0 between other spokes. Removing 0 changes that face.
        var triangles = new List<TriWithGroupId>
        {
            new() { A = 0, B = 1, C = 5, GroupId = 1 },
            new() { A = 0, B = 5, C = 2, GroupId = 1 },
            new() { A = 0, B = 2, C = 4, GroupId = 1 },
            new() { A = 0, B = 4, C = 3, GroupId = 2 },
            new() { A = 0, B = 3, C = 1, GroupId = 1 }
        };
        EdgeCollapser.CleanMesh(points, triangles, BigRationalHybrid.Zero,
            removeCollinearEdges: true, vertexMustBePreserved: new bool[points.Count]);
        var corner = Assert.Single(triangles.Where(t => t.A >= 0 && t.GroupId == 2));
        Assert.Contains(0, new[] { corner.A, corner.B, corner.C });
        Assert.Equal(5, triangles.Count(t => t.A >= 0));
    }
}
