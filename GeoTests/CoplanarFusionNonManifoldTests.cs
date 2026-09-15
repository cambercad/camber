using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class CoplanarFusionNonManifoldTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void AnchorMesh_UnionEdgeTouch_ConstructsWithoutThrow()
    {
        var union = CreateEdgeTouchUnion();

        Assert.NotEmpty(union.Mesh.Triangles);
        Assert.NotEmpty(NonManifoldEdges(union.Mesh.Triangles));
    }

    [Fact]
    public void FuseCoplanarPlanes_PreservesVolume_EdgeTouchUnion()
    {
        var union = CreateEdgeTouchUnion();

        double volume = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(union.Mesh.Positions, union.Mesh.Triangles));

        Assert.True(Math.Abs(3.0 - volume) < 1e-5, $"expected volume 3, got {volume:F8}");
    }

    [Fact]
    public void RetriangulateCoplanar_DoesNotCrossNonManifoldEdge()
    {
        var union = CreateEdgeTouchUnion();
        var triangles = union.Mesh.Triangles.ToList();
        var trianglesEx = union.Mesh.TrianglesEx.ToList();
        var nonManifoldBefore = NonManifoldEdges(triangles);
        double volumeBefore = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(union.Mesh.Positions, triangles));
        var planarGroups = trianglesEx.Select(t => t.GroupId).ToHashSet();

        CoplanarGroupRetriangulation.RetriangulateCoplanar(
            union.Mesh.PrecisionPositions,
            triangles,
            trianglesEx,
            planarGroups);

        var nonManifoldAfter = NonManifoldEdges(triangles);
        double volumeAfter = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(union.Mesh.Positions, triangles));

        Assert.NotEmpty(nonManifoldBefore);
        Assert.True(nonManifoldBefore.IsSubsetOf(nonManifoldAfter));
        Assert.True(Math.Abs(volumeBefore - volumeAfter) < 1e-5, $"volume changed from {volumeBefore:F8} to {volumeAfter:F8}");
    }

    private static AnchorMesh CreateEdgeTouchUnion()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0.0, 0, 0)), new Vec3D(1, 1, 1), "a");
        var b = api.CreateCuboid(new CoordinateSystem(new Vec3D(1, 1, 0)), new Vec3D(1, 1, 1), "b");
        var c = api.CreateCuboid(new CoordinateSystem(new Vec3D(2, 2, 0)), new Vec3D(1, 1, 1), "c");

        var ab = api.Boolean(a, b, BooleanOp.Union, "ab");
        return api.Boolean(ab, c, BooleanOp.Union, "abc");
    }

    private static HashSet<(int, int)> NonManifoldEdges(List<Tri> triangles)
    {
        var edgeToTriangles = AdjacencyEx.BuildEdgeToTrianglesMap(triangles);
        return edgeToTriangles
            .Where(kv => kv.Value.Count > 2)
            .Select(kv => kv.Key)
            .ToHashSet();
    }
}
