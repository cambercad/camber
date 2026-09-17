using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class BooleanAncestorNameTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters:false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AncestorAndQualifiedDescendantShareOneCanonicalNamePerGroup(bool reversed)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2),new Vec3D(8)),.01);
        var source = api.CreateCuboid(new Vec3D(0),new Vec3D(6,4,2),"source");
        var cutter = api.CreateCuboid(new Vec3D(1,-1,-1),new Vec3D(2,5,3),"cutter");
        var descendant = api.Boolean(source,cutter,BooleanOp.Difference,"descendant");
        descendant.EnsureCoplanarPostProcessed();
        var original = new Dictionary<string,int>(source.extendedNameToGroupId);
        var qualified = new Dictionary<string,int>(descendant.extendedNameToGroupId);
        Assert.Contains(qualified, pair => original.Any(old => old.Value==pair.Value && old.Key!=pair.Key));
        var result = reversed ? api.Boolean(descendant,source,BooleanOp.Intersect,"result")
            : api.Boolean(source,descendant,BooleanOp.Intersect,"result");
        result.EnsureCoplanarPostProcessed();
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions,result.Mesh.Triangles),39.999,40.001);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(result.Mesh);
        Assert.Equal(result.extendedNameToGroupId.Count,result.extendedNameToGroupId.Values.Distinct().Count());
        Assert.Equal(original,source.extendedNameToGroupId);
        Assert.Equal(qualified,descendant.extendedNameToGroupId);
        // A subsequent difference must also consume the merged mapping safely.
        var empty = api.Boolean(result,descendant,BooleanOp.Difference,"empty");
        Assert.Empty(empty.Mesh.Triangles);
    }
}
