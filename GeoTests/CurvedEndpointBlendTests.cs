using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public sealed class CurvedEndpointBlendTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, true)]
    [InlineData(true, false, true)]
    [InlineData(true, true, true)]
    public void RibOnTruncatedSphereTerminatesOnCurvedShoulder(bool chamfer, bool eraseMetadata, bool reentrant)
        => CheckRib(chamfer, eraseMetadata, reentrant, 1, false);

    [Theory]
    [InlineData(false, .1)]
    [InlineData(true, .1)]
    [InlineData(false, 10)]
    [InlineData(true, 10)]
    public void MetadataFreePocketEndpointIsIndependentOfPoseAndScale(bool chamfer, double scale)
        => CheckRib(chamfer, true, true, scale, true);

    private static void CheckRib(bool chamfer, bool eraseMetadata, bool reentrant, double scale, bool posed)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20 * scale), new Vec3D(20 * scale)), .03 * scale);
        var sphere = api.CreateSphere(new CoordinateSystem(new Vec3D(0, 0, reentrant ? 12 : 0) * scale), 10 * scale, .03 * scale, "sphere");
        var cap = api.CreateCuboid(new Vec3D(-12, -12, -12) * scale, new Vec3D(12, 12, 5) * scale, "cap");
        var baseBody = reentrant ? api.Boolean(cap, sphere, BooleanOp.Subtract, "base") :
            api.Boolean(sphere, cap, BooleanOp.Intersect, "base");
        var rib = api.CreateCuboid(new Vec3D(-12, -1, 4) * scale, new Vec3D(12, 1, 8) * scale, "rib");
        var solid = api.Boolean(baseBody, rib, BooleanOp.Union, "ribbed");
        if (posed)
        {
            var frame = new CoordinateSystem(new Vec3D(2, -3, 1) * scale,
                new Vec3D(.6, .8, 0), new Vec3D(-.48, .36, .8), new Vec3D(.64, -.48, .6));
            solid = api.CopyMesh(solid, CoordinateSystem.Default, frame, "posed_", SurfacePatchNameAffix.Prefix, "posed");
        }
        if (eraseMetadata) solid.surfaceMetaData.Clear();
        solid.EnsureCoplanarPostProcessed();
        int index = solid.GroupEdges.FindIndex(edge =>
        {
            string a = solid.groupIdToExtendedName[edge.GroupIdA];
            string b = solid.groupIdToExtendedName[edge.GroupIdB];
            // Fused faces carry combined names (cap-Line4&rib-Line4). Only a pure rib
            // side wall forms the concave rib/cap root edge; the fused outer wall is convex.
            static bool RibSide(string n) => n.Contains("rib-Line") && !n.Contains("cap-");
            return (a.Contains("cap-ExtrudeTop") && RibSide(b)) ||
                (b.Contains("cap-ExtrudeTop") && RibSide(a));
        });
        Assert.True(index >= 0);
        var edges = new List<string> { solid.GetEdgeReference(index) };
        var result = chamfer ? api.Chamfer(solid, edges, .3 * scale, .03 * scale, "beveled_root") :
            api.Fillet(solid, edges, .3 * scale, .03 * scale, "rounded_root");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles, true));
        double before = MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions, solid.Mesh.Triangles);
        double after = MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles);
        Assert.True(after > before);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void BeamEdgeEnteringCurvedBodyHasAClosedEndpoint(bool chamfer, bool eraseMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-32, -18, -18), new Vec3D(32, 18, 18)), .03);
        var beam = api.CreateCuboid(new Vec3D(-15, -3, -3), new Vec3D(30, 3, 3), "beam");
        var sphere = api.CreateSphere(new CoordinateSystem(new Vec3D(-15, 0, 0)), 15, .03, "sphere");
        var solid = api.Boolean(beam, sphere, BooleanOp.Union, "joined");
        if (eraseMetadata) solid.surfaceMetaData.Clear();
        solid.EnsureCoplanarPostProcessed();
        int index = solid.GroupEdges.FindIndex(edge =>
            solid.groupIdToExtendedName[edge.GroupIdA].Contains("beam-Line") &&
            solid.groupIdToExtendedName[edge.GroupIdB].Contains("beam-Line"));
        Assert.True(index >= 0);
        var edges = new List<string> { solid.GetEdgeReference(index) };
        var result = chamfer ? api.Chamfer(solid, edges, .4, .03, "beveled") :
            api.Fillet(solid, edges, .4, .03, "rounded");
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(result.Mesh);
        double before = MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions, solid.Mesh.Triangles);
        double after = MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles);
        Assert.InRange(after, 0, before - .01);
    }
}
