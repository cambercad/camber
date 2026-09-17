using CSG;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public class CurrentEdgeReferenceTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    private static (GeoAPI api, AnchorMesh mesh) CrossingCylinders()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .02);
        var hub = api.CreateCylinder(new CoordinateSystem(new Vec3D(0)), 5, 5, .02, "hub");
        var transverse = new CoordinateSystem(new Vec3D(-7, 0, 2.5),
            new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
        var crossing = api.CreateCylinder(transverse, 1, 14, .02, "crossing");
        var mesh = api.Boolean(hub, crossing, BooleanOp.Union, "joined");
        mesh.EnsureCoplanarPostProcessed();
        return (api, mesh);
    }

    [Fact]
    public void EnumeratedCurvedEdgesRoundTripWithoutRelaxingAncestorReferences()
    {
        var (_, mesh) = CrossingCylinders();
        var junctions = mesh.GroupEdges.Select((edge, index) => (edge, index))
            .Where(item => item.edge.Name.Contains("hub-Circle1") && item.edge.Name.Contains("crossing-Circle1"))
            .ToArray();
        Assert.Equal(2, junctions.Length);
        foreach (var (edge, index) in junctions)
        {
            string reference = mesh.GetEdgeReference(index);
            Assert.Contains("#current=", reference);
            Assert.Equal(reference, mesh.GetEdgeReference(index));
            Assert.Equal(edge.Name, Assert.Single(FaceLineageEdges.Resolve(mesh, new[] { reference })));
            Assert.Throws<NameCollisionException>(() => FaceLineageEdges.Resolve(mesh, new[] { edge.Name }));
        }
        Assert.Throws<NameCollisionException>(() => FaceLineageEdges.Resolve(mesh,
            new[] { "[hub-Circle1,crossing-Circle1]_1" }));
    }

    [Fact]
    public void CurrentReferencesDoNotTransferToCopiesOrRebuiltSolids()
    {
        var (api, mesh) = CrossingCylinders();
        int index = mesh.GroupEdges.FindIndex(edge => edge.Name.Contains("hub-Circle1") && edge.Name.Contains("crossing-Circle1"));
        string reference = mesh.GetEdgeReference(index);
        var copy = api.CopyMeshAsInstance(mesh, "copy");
        copy.EnsureCoplanarPostProcessed();
        Assert.Throws<NameCollisionException>(() => FaceLineageEdges.Resolve(copy, new[] { reference }));
        var (_, rebuilt) = CrossingCylinders();
        Assert.Throws<NameCollisionException>(() => FaceLineageEdges.Resolve(rebuilt, new[] { reference }));
        mesh.Rename("renamed");
        // The root labels belong to the construction operands, so renaming the
        // Boolean result need not change them. Its topology remains the same.
        Assert.Single(FaceLineageEdges.Resolve(mesh, new[] { reference }));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ScopedCurrentEdgeIsAcceptedByTheActualFeature(bool chamfer)
    {
        var (api, mesh) = CrossingCylinders();
        int index = mesh.GroupEdges.FindIndex(edge => edge.Name.Contains("crossing-Circle1") &&
            edge.Name.Contains("crossing-ExtrudeBottom"));
        string reference = mesh.GetEdgeReference(index);
        Assert.Contains("#current=", reference);
        var result = chamfer
            ? api.Chamfer(mesh, new() { reference }, .1, .02, "bevelled")
            : api.Fillet(mesh, new() { reference }, .1, .02, "rounded");
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(result.Mesh);
        double before = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(mesh.Mesh.Positions, mesh.Mesh.Triangles));
        double after = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles));
        Assert.InRange(before - after, .001, .1);
    }

    [Fact]
    public void MarkerTextInsideAnAuthoredFeatureNameIsNotAScopeSuffix()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .02);
        var block = api.CreateCuboid(new Vec3D(0), new Vec3D(3), "block#current=literal");
        string reference = block.GetEdgeReference(0);
        Assert.Equal(reference, Assert.Single(FaceLineageEdges.Resolve(block, new[] { reference })));
    }

    [Fact]
    public void OrdinaryNamedEdgesRemainUnqualifiedAndUsableByBothFeatures()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .02);
        var block = api.CreateCuboid(new Vec3D(0), new Vec3D(3), "block");
        string reference = block.GetEdgeReference(0);
        Assert.DoesNotContain("#current=", reference);
        var fillet = api.Fillet(block, new() { reference }, .1, .02, "fillet");
        var chamfer = api.Chamfer(block, new() { reference }, .1, .02, "chamfer");
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(fillet.Mesh);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(chamfer.Mesh);
    }
}
