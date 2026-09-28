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
    public void EnumeratedCurvedEdgesHaveUniqueReadableNames()
    {
        var (_, mesh) = CrossingCylinders();
        var junctions = mesh.GroupEdges.Select((edge, index) => (edge, index))
            .Where(item => item.edge.Name.Contains("hub-Circle1") && item.edge.Name.Contains("crossing-Circle1"))
            .ToArray();
        Assert.Equal(2, junctions.Length);
        Assert.Equal(mesh.GroupEdges.Count, mesh.GroupEdges.Select(edge => edge.Name).Distinct().Count());
        Assert.Equal(mesh.groupIdToExtendedName.Count, mesh.groupIdToExtendedName.Values.Distinct().Count());
        foreach (var (edge, index) in junctions)
        {
            string reference = mesh.GetEdgeReference(index);
            Assert.DoesNotContain("#current=", reference);
            Assert.DoesNotContain("unresolved:", reference);
            Assert.Equal(reference, mesh.GetEdgeReference(index));
            Assert.Equal(edge.Name, Assert.Single(FaceLineageEdges.Resolve(mesh, new[] { reference })));
        }
    }

    [Fact]
    public void CurrentNamesTransferWhenTopologyIsCopiedOrRebuilt()
    {
        var (api, mesh) = CrossingCylinders();
        int index = mesh.GroupEdges.FindIndex(edge => edge.Name.Contains("hub-Circle1") && edge.Name.Contains("crossing-Circle1"));
        string reference = mesh.GetEdgeReference(index);
        var copy = api.CopyMeshAsInstance(mesh, "copy");
        copy.EnsureCoplanarPostProcessed();
        Assert.Equal(reference, Assert.Single(FaceLineageEdges.Resolve(copy, new[] { reference })));
        int copiedIndex = copy.GroupEdges.FindIndex(edge => edge.Name == reference);
        Assert.True(copiedIndex >= 0);
        Assert.Equal(mesh.GroupEdges[index].EdgeSegments, copy.GroupEdges[copiedIndex].EdgeSegments);
        var (_, rebuilt) = CrossingCylinders();
        Assert.Equal(reference, Assert.Single(FaceLineageEdges.Resolve(rebuilt, new[] { reference })));
        mesh.Rename("renamed");
        // The root labels belong to the construction operands, so renaming the
        // Boolean result need not change them. Its topology remains the same.
        Assert.Single(FaceLineageEdges.Resolve(mesh, new[] { reference }));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurrentEdgeNameIsAcceptedByTheActualFeature(bool chamfer)
    {
        var (api, mesh) = CrossingCylinders();
        int index = mesh.GroupEdges.FindIndex(edge => edge.Name.Contains("crossing-Circle1") &&
            edge.Name.Contains("crossing-ExtrudeBottom"));
        string reference = mesh.GetEdgeReference(index);
        Assert.DoesNotContain("#current=", reference);
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
