using Geo;
using GeoCore;

namespace GeoTests;

public class ChamferCornerOrientationTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    public void ThreeEdgeChamferHasConsistentWindingAtEveryBoxCorner(int corner)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50, -50, -15), new Vec3D(350, 300, 130)), .04);
        var body = api.CreateCuboid(new CoordinateSystem(new Vec3D(-21, -17, -6)),
            new Vec3D(42, 34, 7), "flange");
        var graph = new EdgeGraph(body.Mesh.Triangles, body.Mesh.GetTriangleGroups(),
            body.Mesh.Positions, body.Mesh.PrecisionPositions, body.groupIdToExtendedName);
        var position = new Vec3D((corner & 1) == 0 ? -21 : 21,
            (corner & 2) == 0 ? -17 : 17, (corner & 4) == 0 ? -6 : 1);
        var edges = graph.Edges.Where(e => (e.StartNode.Position - position).Length() < .001
            || (e.EndNode.Position - position).Length() < .001).Select(e => e.Name).ToList();
        Assert.Equal(3, edges.Count);
        var result = api.Chamfer(body, edges, .7, .04, "corner_chamfer");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        int cornerTriangles = 0;
        for (int i = 0; i < result.Mesh.Triangles.Count; i++)
        {
            var data = result.Mesh.TrianglesEx[i];
            if (!result.groupIdToExtendedName[data.GroupId].Contains("ChamferCorner")) continue;
            cornerTriangles++;
            var triangle = result.Mesh.Triangles[i];
            var geometric = Vec3DOps.Cross(result.Mesh.Positions[triangle.B] - result.Mesh.Positions[triangle.A],
                result.Mesh.Positions[triangle.C] - result.Mesh.Positions[triangle.A]).Normalized();
            Assert.All(new[] { data.V0.Normal, data.V1.Normal, data.V2.Normal },
                normal => Assert.True(Vec3DOps.Dot(geometric, normal) > .99));
        }
        Assert.True(cornerTriangles > 0);
        double removed = MeshAnalysis.ComputeSignedMeshVolume(body.Mesh.Positions, body.Mesh.Triangles)
            - MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles);
        Assert.True(removed > 0);
    }
}
