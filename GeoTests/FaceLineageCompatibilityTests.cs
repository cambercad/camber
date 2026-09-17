using CSG;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public class FaceLineageCompatibilityTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void FusingSplitFacePublishesItsMergedStructuralIdentity()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .01);
        var left = api.CreateCuboid(new Vec3D(0), new Vec3D(6, 4, 2), "left");
        var slot = api.CreateCuboid(new Vec3D(2, -1, -1), new Vec3D(3, 5, 3), "slot");
        var cut = api.Boolean(left, slot, BooleanOp.Difference);
        cut.EnsureCoplanarPostProcessed();
        var right = api.CreateCuboid(new Vec3D(6, 0, 0), new Vec3D(12, 4, 2), "right");
        var fused = api.Boolean(cut, right, BooleanOp.Union, "left");
        fused.EnsureCoplanarPostProcessed();
        const string reference = "left-ExtrudeTop&right-ExtrudeTop{slot-Line2}";
        Assert.True(fused.TryGetSurface(reference, out _), string.Join(";", fused.groupIdToExtendedName.Values));
        AssemblyDatumResolver.ResolvePlane(fused, reference, api.Converter);
        Assert.Throws<NameCollisionException>(() =>
            AssemblyDatumResolver.ResolvePlane(fused, "left-ExtrudeTop", api.Converter));
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(fused.Mesh);

        var copy = api.CopyMeshAsInstance(fused, "copied");
        const string copiedReference = "copied-ExtrudeTop&right-ExtrudeTop{slot-Line2}";
        Assert.True(copy.TryGetSurface(copiedReference, out _));
        Assert.True(copy.surfaceMetaData.ContainsKey(copiedReference));
        AssemblyDatumResolver.ResolvePlane(copy, copiedReference, api.Converter);
        Assert.Throws<NameCollisionException>(() =>
            AssemblyDatumResolver.ResolvePlane(copy, "copied-ExtrudeTop", api.Converter));

        fused.Rename("renamed");
        const string renamedReference = "renamed-ExtrudeTop&right-ExtrudeTop{slot-Line2}";
        Assert.True(fused.TryGetSurface(renamedReference, out _));
        Assert.True(fused.surfaceMetaData.ContainsKey(renamedReference));
        AssemblyDatumResolver.ResolvePlane(fused, renamedReference, api.Converter);
        Assert.Throws<NameCollisionException>(() =>
            AssemblyDatumResolver.ResolvePlane(fused, "renamed-ExtrudeTop", api.Converter));
    }
}
