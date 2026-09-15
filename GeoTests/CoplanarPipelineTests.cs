using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class CoplanarPipelineTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void SingleCube_PreservesVolumeAndUvAfterConstruction()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(2)), 1e-4);
        var cube = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 1), "cube");

        MeshPipelineTestHelpers.AssertVolume(cube.Mesh, 1.0);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(cube.Mesh);
        MeshPipelineTestHelpers.AssertCornerUvConsistentWithinGroups(cube.Mesh);
    }

    [Fact]
    public void FaceTouchUnion_PreservesVolumeAndFusesCoplanarGroups()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(3)), 1e-4);
        var union = MeshPipelineTestHelpers.CreateFaceTouchUnion(api);

        MeshPipelineTestHelpers.AssertVolume(union.Mesh, 2.0);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(union.Mesh);
        MeshPipelineTestHelpers.AssertCornerUvConsistentWithinGroups(union.Mesh);

        int groupsAfterConstruction = MeshPipelineTestHelpers.CountDistinctGroups(union.Mesh);
        Assert.True(groupsAfterConstruction < 12, $"expected fused groups < 12, got {groupsAfterConstruction}");
    }

    [Fact]
    public void CoplanarGroupFusion_MergesAdjacentCoplanarFaceGroups()
    {
        // Two coplanar square patches (groups 0 and 1) sharing edge v1-v2.
        var positions = new List<Rat3Hybrid>
        {
            new(0, 0, 0), // 0
            new(1, 0, 0), // 1
            new(1, 1, 0), // 2
            new(0, 1, 0), // 3
            new(2, 0, 0), // 4
            new(2, 1, 0), // 5
        };

        var triangles = new List<Tri>
        {
            new(0, 1, 2),
            new(0, 2, 3),
            new(1, 4, 5),
            new(1, 5, 2),
        };

        var groupIdPerTriangle = new List<int> { 0, 0, 1, 1 };
        var groupNames = new Dictionary<int, string> { [0] = "left", [1] = "right" };
        var planarGroups = new HashSet<int> { 0, 1 };

        var representativeTriangles = CoplanarGroupFusion.FuseCoplanarGroups(
            triangles,
            groupIdPerTriangle,
            positions,
            groupNames,
            planarGroups,
            out var newGroupPerTriangle,
            out _);

        int groupsAfter = newGroupPerTriangle.Distinct().Count();
        Assert.Equal(1, groupsAfter);
        Assert.NotEmpty(representativeTriangles);
    }

    [Fact]
    public void RetriangulateCoplanar_PreservesVolumeOnFaceTouchUnion()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(3)), 1e-4);
        var union = MeshPipelineTestHelpers.CreateFaceTouchUnion(api);

        var triangles = union.Mesh.Triangles.ToList();
        var trianglesEx = union.Mesh.TrianglesEx.ToList();
        double volumeBefore = MeshPipelineTestHelpers.AbsVolume(union.Mesh);
        var planarGroups = trianglesEx.Select(t => t.GroupId).ToHashSet();

        CoplanarGroupRetriangulation.RetriangulateCoplanar(
            union.Mesh.PrecisionPositions,
            triangles,
            trianglesEx,
            planarGroups);

        double volumeAfter = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(union.Mesh.Positions, triangles));
        Assert.True(
            Math.Abs(volumeBefore - volumeAfter) < MeshPipelineTestHelpers.VolumeTolerance,
            $"volume changed from {volumeBefore:F8} to {volumeAfter:F8}");
        MeshPipelineTestHelpers.AssertCornerUvConsistentWithinGroups(union.Mesh);
    }

    [Fact]
    public void RetriangulateCoplanar_IsDeterministicAcrossParallelRuns()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(3)), 1e-4);
        var union = MeshPipelineTestHelpers.CreateFaceTouchUnion(api);

        static (List<Rat3Hybrid> positions, List<Tri> triangles, List<MeshTriangle<TriangleVertexNormalUV>> trianglesEx, HashSet<int> planarGroups)
            CloneMeshState(MeshNormalUV mesh)
        {
            var positions = new List<Rat3Hybrid>(mesh.PrecisionPositions);
            var triangles = mesh.Triangles.ToList();
            var trianglesEx = mesh.TrianglesEx.ToList();
            var planarGroups = trianglesEx.Select(t => t.GroupId).ToHashSet();
            return (positions, triangles, trianglesEx, planarGroups);
        }

        var run1 = CloneMeshState(union.Mesh);
        var run2 = CloneMeshState(union.Mesh);

        CoplanarGroupRetriangulation.RetriangulateCoplanar(run1.positions, run1.triangles, run1.trianglesEx, run1.planarGroups);
        CoplanarGroupRetriangulation.RetriangulateCoplanar(run2.positions, run2.triangles, run2.trianglesEx, run2.planarGroups);

        Assert.Equal(run1.triangles.Count, run2.triangles.Count);
        for (int i = 0; i < run1.triangles.Count; i++)
        {
            Assert.Equal(run1.triangles[i].A, run2.triangles[i].A);
            Assert.Equal(run1.triangles[i].B, run2.triangles[i].B);
            Assert.Equal(run1.triangles[i].C, run2.triangles[i].C);
            Assert.Equal(run1.trianglesEx[i].GroupId, run2.trianglesEx[i].GroupId);
        }

        double volume1 = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(union.Mesh.Positions, run1.triangles));
        double volume2 = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(union.Mesh.Positions, run2.triangles));
        Assert.True(Math.Abs(volume1 - volume2) < MeshPipelineTestHelpers.VolumeTolerance);
    }
}
