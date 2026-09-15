using CSG;
using Geo;
using Geo.BRep;
using GeoCore;
using GeoMeta;

namespace GeoTests;

[Collection("GeoAPISequential")]
public class DisconnectedGroupSplitTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void TwoDisjointSquares_SameGroup_BecomeTwoNamedFaces()
    {
        // Left square at x∈[0,1], right at x∈[3,4] — same group id, disconnected.
        var positions = new List<Vec3D>
        {
            new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0),
            new(3, 0, 0), new(4, 0, 0), new(4, 1, 0), new(3, 1, 0),
        };
        var triangles = new List<Tri>
        {
            new(0, 1, 2), new(0, 2, 3),
            new(4, 5, 6), new(4, 6, 7),
        };
        var groupIdPerTriangle = new List<int> { 10, 10, 10, 10 };
        var groupIdToName = new Dictionary<int, string> { [10] = "A" };
        var meta = new Dictionary<string, SurfaceMetaData>
        {
            ["A"] = new SurfaceMetaData(SurfaceType.Planar)
        };

        int nextId = 100;
        bool changed = DisconnectedGroupSplit.SplitDisconnectedGroups(
            triangles, positions, groupIdPerTriangle, groupIdToName, meta,
            count =>
            {
                int baseId = nextId;
                nextId += count;
                return baseId;
            });

        Assert.True(changed);
        Assert.Equal(2, groupIdPerTriangle.Distinct().Count());
        Assert.Equal("A", groupIdToName[10]);
        Assert.Equal("A_1", groupIdToName[100]);
        Assert.True(meta.ContainsKey("A"));
        Assert.True(meta.ContainsKey("A_1"));

        // Left island (smaller X) keeps original id 10.
        Assert.Equal(10, groupIdPerTriangle[0]);
        Assert.Equal(10, groupIdPerTriangle[1]);
        Assert.Equal(100, groupIdPerTriangle[2]);
        Assert.Equal(100, groupIdPerTriangle[3]);
    }

    [Fact]
    public void SquareWithHole_StaysOneNamedFace()
    {
        // Outer 0..3, inner hole 4..7 — ring of triangles, one connected component.
        var positions = new List<Vec3D>
        {
            new(0, 0, 0), new(3, 0, 0), new(3, 3, 0), new(0, 3, 0),
            new(1, 1, 0), new(2, 1, 0), new(2, 2, 0), new(1, 2, 0),
        };
        var triangles = new List<Tri>
        {
            new(0, 1, 5), new(0, 5, 4),
            new(1, 2, 6), new(1, 6, 5),
            new(2, 3, 7), new(2, 7, 6),
            new(3, 0, 4), new(3, 4, 7),
        };
        var groupIdPerTriangle = Enumerable.Repeat(7, triangles.Count).ToList();
        var groupIdToName = new Dictionary<int, string> { [7] = "face" };
        var meta = new Dictionary<string, SurfaceMetaData>();

        bool changed = DisconnectedGroupSplit.SplitDisconnectedGroups(
            triangles, positions, groupIdPerTriangle, groupIdToName, meta,
            count => throw new Exception("should not allocate"));

        Assert.False(changed);
        Assert.Single(groupIdPerTriangle.Distinct());
        Assert.Equal("face", groupIdToName[7]);
        Assert.DoesNotContain("face_1", groupIdToName.Values);
    }

    [Fact]
    public void SpatialSort_LeftmostIslandKeepsUnsuffixedName()
    {
        // Intentionally list the right island's triangles first in the mesh.
        var positions = new List<Vec3D>
        {
            new(5, 0, 0), new(6, 0, 0), new(6, 1, 0), // right
            new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), // left
        };
        var triangles = new List<Tri>
        {
            new(0, 1, 2), // right first
            new(3, 4, 5), // left second
        };
        var groupIdPerTriangle = new List<int> { 1, 1 };
        var groupIdToName = new Dictionary<int, string> { [1] = "patch" };

        int nextId = 50;
        DisconnectedGroupSplit.SplitDisconnectedGroups(
            triangles, positions, groupIdPerTriangle, groupIdToName, null,
            count =>
            {
                int baseId = nextId;
                nextId += count;
                return baseId;
            });

        Assert.Equal(1, groupIdPerTriangle[1]);   // left keeps origin id
        Assert.Equal(50, groupIdPerTriangle[0]);  // right gets new id
        Assert.Equal("patch", groupIdToName[1]);
        Assert.Equal("patch_1", groupIdToName[50]);
    }

    [Fact]
    public void FaceTouchUnion_StillFusesAdjacentCoplanarGroups()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(3)), 1e-4);
        var union = MeshPipelineTestHelpers.CreateFaceTouchUnion(api);

        MeshPipelineTestHelpers.AssertVolume(union.Mesh, 2.0);
        int groupsAfterConstruction = MeshPipelineTestHelpers.CountDistinctGroups(union.Mesh);
        Assert.True(groupsAfterConstruction < 12, $"expected fused groups < 12, got {groupsAfterConstruction}");
    }

    [Fact]
    public void SphereCubeThreeHoles_SplitsCylinderWalls_KeepsConnectedCubeFaces()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), 0.05);
        var cube = api.CreateCube(new CoordinateSystem(new Vec3D(-9, -9, -9)), 18.0, "cube");
        var sphere = api.CreateSphere(CoordinateSystem.Default, 10.0, 0.05, "sphere");
        var body = api.Boolean(sphere, cube, BooleanOp.Intersect, "body");

        var holeZ = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, -12)), 5.0, 24.0, 0.05, "hole_z");
        body = api.Boolean(body, holeZ, BooleanOp.Difference, "body");
        var holeX = api.CreateCylinder(
            new CoordinateSystem(
                new Vec3D(-12, 0, 0),
                new Vec3D(0, 1, 0),
                new Vec3D(0, 0, 1),
                new Vec3D(1, 0, 0)),
            5.0, 24.0, 0.05, "hole_x");
        body = api.Boolean(body, holeX, BooleanOp.Difference, "body");
        var holeY = api.CreateCylinder(
            new CoordinateSystem(
                new Vec3D(0, -12, 0),
                new Vec3D(0, 0, 1),
                new Vec3D(1, 0, 0),
                new Vec3D(0, 1, 0)),
            5.0, 24.0, 0.05, "hole_y");
        body = api.Boolean(body, holeY, BooleanOp.Difference, "body");

        body.EnsureCoplanarPostProcessed();

        var activeGroups = body.Mesh.GetTriangleGroups().ToHashSet();
        var patchNames = body.extendedNameToGroupId
            .Where(kv => activeGroups.Contains(kv.Value))
            .Select(kv => kv.Key)
            .ToList();

        // Each through-hole cylinder wall is cut into two disconnected half-shells.
        Assert.Contains("hole_z-Circle1", patchNames);
        Assert.Contains("hole_z-Circle1_1", patchNames);
        Assert.Contains("hole_x-Circle1", patchNames);
        Assert.Contains("hole_x-Circle1_1", patchNames);
        Assert.Contains("hole_y-Circle1", patchNames);
        Assert.Contains("hole_y-Circle1_1", patchNames);

        // Cube planar faces stay one connected patch each (holes, not islands).
        Assert.DoesNotContain(patchNames, n =>
            n.StartsWith("cube-", StringComparison.Ordinal) &&
            (n.EndsWith("_1", StringComparison.Ordinal) || n.Contains("_1_", StringComparison.Ordinal)));

        var brep = BRepAssembler.BuildFromAnchorMesh(body);
        int activePatchCount = patchNames.Count;
        Assert.Equal(activePatchCount, brep.Shell.Faces.Count);

        // Each BRep face has at most one outer loop (STEP WR2).
        foreach (var face in brep.Shell.Faces)
        {
            int outerCount = face.Loops.Count(l => l.IsOuter);
            Assert.True(outerCount <= 1,
                $"Face '{face.PatchName}' has {outerCount} outer loops");
        }
    }
}