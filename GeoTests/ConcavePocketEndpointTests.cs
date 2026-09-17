using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class ConcavePocketEndpointTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(0, false, false)]
    [InlineData(1, false, false)]
    [InlineData(2, false, false)]
    [InlineData(3, false, false)]
    [InlineData(0, true, false)]
    [InlineData(1, true, false)]
    [InlineData(2, true, false)]
    [InlineData(3, true, false)]
    [InlineData(0, false, true)]
    [InlineData(0, true, true)]
    public void BlindPocketBottomTreatmentsRetainThePocketInterior(int index, bool chamfer, bool network)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50, -50, -15), new Vec3D(350, 300, 130)), .04);
        var block = api.CreateCuboid(new Vec3D(-30, -22, -10), new Vec3D(30, 22, 4), "block");
        var cutter = api.CreateCuboid(new Vec3D(-8, -6, -5), new Vec3D(8, 6, 10), "pocket");
        var body = api.Boolean(block, cutter, BooleanOp.Difference, "body");
        var graph = new EdgeGraph(body.Mesh.Triangles, body.Mesh.GetTriangleGroups(),
            body.Mesh.Positions, body.Mesh.PrecisionPositions, body.groupIdToExtendedName);
        var bottomEdges = graph.Edges.Where(e => e.Name.Contains("pocket-ExtrudeBottom") &&
            body.groupIdToExtendedName[e.GroupIdA].StartsWith("pocket-") &&
            body.groupIdToExtendedName[e.GroupIdB].StartsWith("pocket-")).ToList();
        Assert.Equal(4, bottomEdges.Count);
        var edge = bottomEdges[index];
        var selected = network
            ? edge.StartNode.ConnectedEdges.Select(e => e.Name).ToList()
            : new List<string> { edge.Name };
        if (network) Assert.Equal(3, selected.Count);
        var result = chamfer ? api.Chamfer(body, selected, .6, .04, "chamfered")
            : api.Fillet(body, selected, .6, .04, "rounded");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        double added = MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles)
            - MeshAnalysis.ComputeSignedMeshVolume(body.Mesh.Positions, body.Mesh.Triangles);
        Assert.InRange(added, .1, 15);
    }
}
