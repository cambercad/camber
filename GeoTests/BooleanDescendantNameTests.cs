using CSG;
using Geo;
using GeoCore;
using GeoMeta;
namespace GeoTests;
public class BooleanDescendantNameTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void IndependentlySplitDescendantsCanBeIntersectedWithoutConflatingGroups(bool sameOwnerName, bool reservedScopedLabel)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2),new Vec3D(8)),.01);
        var source = api.CreateCuboid(new Vec3D(0),new Vec3D(6,4,2),"source");
        var firstCut = api.CreateCuboid(new Vec3D(1,-1,-1),new Vec3D(2,5,3),"firstCut");
        var secondCut = api.CreateCuboid(new Vec3D(3,-1,-1),new Vec3D(4,5,3),"secondCut");
        var a = api.Boolean(source,firstCut,BooleanOp.Difference,"firstBranch");
        var b = api.Boolean(source,secondCut,BooleanOp.Difference,"secondBranch");
        a.EnsureCoplanarPostProcessed(); b.EnsureCoplanarPostProcessed();
        // Structural split names distinguish these two cutters naturally. Model
        // an explicit duplicate display label to retain this test's independent
        // group-identity collision coverage rather than relying on ordinal names.
        foreach (var pair in b.extendedNameToGroupId.ToArray())
        {
            string alias = pair.Key.Replace("secondCut", "firstCut", StringComparison.Ordinal);
            if (alias == pair.Key) continue;
            b.extendedNameToGroupId.Remove(pair.Key);
            b.extendedNameToGroupId.Add(alias, pair.Value);
            b.groupIdToExtendedName[pair.Value] = alias;
            if (b.surfaceMetaData.Remove(pair.Key, out var metadata)) b.surfaceMetaData.Add(alias, metadata);
        }
        var conflicts = a.extendedNameToGroupId.Where(kv=>b.extendedNameToGroupId.TryGetValue(kv.Key,out int id)&&id!=kv.Value).ToArray();
        Assert.NotEmpty(conflicts);
        if (sameOwnerName) b.Rename(a.Name);
        if (reservedScopedLabel)
            foreach (var conflict in conflicts)
                a.surfaceMetaData.Add(a.Name+"-"+conflict.Key,new SurfaceMetaData(SurfaceType.Planar));
        var aNames = new Dictionary<string,int>(a.extendedNameToGroupId);
        var bNames = new Dictionary<string,int>(b.extendedNameToGroupId);
        var result = api.Boolean(a,b,BooleanOp.Intersect,"merged");
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions,result.Mesh.Triangles),31.999,32.001);
        foreach (var conflict in conflicts)
        {
            string firstScoped = a.Name + "-" + conflict.Key;
            string firstLabel = firstScoped + (reservedScopedLabel ? "_1" : "");
            string secondLabel = b.Name + "-" + conflict.Key + (sameOwnerName ? (reservedScopedLabel ? "_2" : "_1") : "");
            if (reservedScopedLabel)
            {
                Assert.True(result.surfaceMetaData.ContainsKey(firstScoped));
                Assert.NotSame(a.surfaceMetaData[firstScoped],result.surfaceMetaData[firstScoped]);
            }
            Assert.Equal(conflict.Value,result.extendedNameToGroupId[firstLabel]);
            Assert.Equal(b.extendedNameToGroupId[conflict.Key],result.extendedNameToGroupId[secondLabel]);
            Assert.NotEqual(result.extendedNameToGroupId[firstLabel],result.extendedNameToGroupId[secondLabel]);
            Assert.False(result.extendedNameToGroupId.ContainsKey(conflict.Key));
            Assert.NotSame(a.surfaceMetaData[conflict.Key],result.surfaceMetaData[firstLabel]);
            Assert.NotSame(b.surfaceMetaData[conflict.Key],result.surfaceMetaData[secondLabel]);
            Assert.Equal(a.surfaceMetaData[conflict.Key].SurfaceType,result.surfaceMetaData[firstLabel].SurfaceType);
        }
        // Every surviving face remains addressable through its resulting label.
        foreach (int id in result.Mesh.GetTriangleGroups().Distinct())
        {
            string label=result.extendedNameToGroupId.Single(kv=>kv.Value==id).Key;
            bool resolved=false;
            foreach (double u in new[] {0.0,.25,.5,.75,1.0})
            foreach (double v in new[] {0.0,.25,.5,.75,1.0})
                if (result.TryGetPointOnSurface(EntityNaming.FormatSurfacePointAddress(label,u,v),out var point))
                {
                    resolved=true;
                    Assert.True(double.IsFinite(point.X) && double.IsFinite(point.Y) && double.IsFinite(point.Z));
                }
            Assert.True(resolved,label);
        }
        Assert.Equal(aNames,a.extendedNameToGroupId);
        Assert.Equal(bNames,b.extendedNameToGroupId);
    }
}
