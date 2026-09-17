using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class BlendClosureFaceTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcaveBlendEndClosureBelongsToExistingPlanarFace(bool round)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(40)), .01);
        var block = api.CreateCuboid(new Vec3D(0), new Vec3D(20, 20, 6), "block");
        var cutter = api.CreateCuboid(new Vec3D(10, 10, -2), new Vec3D(25, 25, 8), "notch");
        var body = api.Boolean(block, cutter, BooleanOp.Difference, "body");
        var graph = new EdgeGraph(body.Mesh.Triangles, body.Mesh.GetTriangleGroups(),
            body.Mesh.Positions, body.Mesh.PrecisionPositions, body.groupIdToExtendedName);
        var edge = Assert.Single(graph.Edges.Where(e => body.groupIdToExtendedName[e.GroupIdA].StartsWith("notch-") && body.groupIdToExtendedName[e.GroupIdB].StartsWith("notch-")));
        var result = round
            ? api.Fillet(body, new List<string> { edge.Name }, 2, .01, "rounded")
            : api.Chamfer(body, new List<string> { edge.Name }, 2, .01, "chamfered");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.Positions, result.Mesh.Triangles));
        var groups = result.Mesh.GetTriangleGroups();
        foreach (double z in new[] { 0.0, 6.0 })
        {
            var capGroups = Enumerable.Range(0, result.Mesh.Triangles.Count)
                .Where(i => {
                    var t = result.Mesh.Triangles[i];
                    return Math.Abs(result.Mesh.Positions[t.A].Z - z) < .001
                        && Math.Abs(result.Mesh.Positions[t.B].Z - z) < .001
                        && Math.Abs(result.Mesh.Positions[t.C].Z - z) < .001;
                }).Select(i => groups[i]).Distinct().ToList();
            Assert.True(capGroups.Count == 1, "Coplanar end closure must join the existing face: "
                + string.Join(", ", capGroups.Select(g => result.groupIdToExtendedName[g])));
            Assert.Equal(z == 0 ? "block-ExtrudeBottom" : "block-ExtrudeTop",
                result.groupIdToExtendedName[capGroups[0]]);
        }
    }
}
