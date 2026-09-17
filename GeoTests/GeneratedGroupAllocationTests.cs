using System.Collections;
using Curves;
using Geo;
using GeoCore;
namespace GeoTests;

public class GeneratedGroupAllocationTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    static GeoAPI Api() => new(new Box3D(new Vec3D(-10),new Vec3D(20)),.02);
    static PlotterSketcherCoordSys Circle(double z=0)
    {
        var sketch=new PlotterSketcherCoordSys("profile",new CoordinateSystem(new Vec3D(0,0,z)),new Vec2D(0,0));
        sketch.AddCircle(new Vec2D(0,0),5);
        return sketch;
    }

    // A caller-supplied collection gates the first generator read. Before the
    // fix GeoAPI evaluated GetBaseGroupIndex BEFORE entering that generator:
    // all workers therefore held the same ID range, independently of timing.
    sealed class GatedProfiles(PlotterSketcherCoordSys[] profiles,Barrier gate)
        : IReadOnlyList<PlotterSketcherCoordSys>
    {
        int entered;
        public int Count {
            get {
                if (gate != null && Interlocked.Exchange(ref entered,1)==0) Assert.True(gate.SignalAndWait(TimeSpan.FromSeconds(30)), "Timed out waiting for all loft generators to reach the allocation barrier.");
                return profiles.Length;
            }
        }
        public PlotterSketcherCoordSys this[int index] => profiles[index];
        public IEnumerator<PlotterSketcherCoordSys> GetEnumerator() => ((IEnumerable<PlotterSketcherCoordSys>)profiles).GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }

    static Func<AnchorMesh> Feature(GeoAPI api,string kind,Barrier generatorGate,string objPath)
    {
        switch(kind)
        {
            case "extrude":
                var profile=Circle();
                return ()=>api.Extrude(profile,5,name:"body");
            case "loft":
                var profiles=new GatedProfiles([Circle(),Circle(5)],generatorGate);
                return ()=>api.Loft(profiles,new LoftOptions(),"body");
            case "revolve":
                var meridian=new PlotterSketcherCoordSys("section",CoordinateSystem.Default,new Vec2D(0,1));
                meridian.AppendLine(3,1);meridian.AppendLine(3,2);meridian.AppendLine(0,2);meridian.AppendLine(0,1);
                return ()=>api.Revolve(meridian,2*Math.PI,name:"body");
            case "copy":
                var source=api.CreateCube(CoordinateSystem.Default,2,"seed");
                source.EnsureCoplanarPostProcessed();
                return ()=>api.CopyMesh(source,CoordinateSystem.Default,new CoordinateSystem(new Vec3D(3,0,0)),
                    "copy_",SurfacePatchNameAffix.Prefix,"body");
            case "triangles":
                var original=api.CreateCube(CoordinateSystem.Default,2,"seed");
                return ()=>api.CreateFromTriangles(new(original.Mesh.Positions),new(original.Mesh.Triangles),"body");
            case "obj": return ()=>api.LoadWavefrontObjFile(objPath,name:"body");
            default: throw new ArgumentException(kind);
        }
    }

    [Theory]
    [InlineData("extrude")]
    [InlineData("loft")]
    [InlineData("revolve")]
    [InlineData("copy")]
    [InlineData("triangles")]
    [InlineData("obj")]
    public async Task ConcurrentIndependentFeaturesOwnDisjointIdsAndPreserveGeometryAndNames(string kind)
    {
        const int count=8;
        string path=Path.Combine(Path.GetTempPath(),"camber_allocation_"+Guid.NewGuid()+".obj");
        File.WriteAllText(path,"g named_sheet\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
        try
        {
            var baseline=Feature(Api(),kind,null,path)();
            baseline.EnsureCoplanarPostProcessed();
            using var start=new Barrier(count);
            using var generator=new Barrier(count);
            var features=Enumerable.Range(0,count).Select(_=>Feature(Api(),kind,kind=="loft"?generator:null,path)).ToArray();
            var tasks=features.Select(feature=>Task.Factory.StartNew(()=>{
                Assert.True(start.SignalAndWait(TimeSpan.FromSeconds(30)), "Timed out waiting for all feature workers to start.");
                var result=feature();result.EnsureCoplanarPostProcessed();return result;
            },CancellationToken.None,TaskCreationOptions.LongRunning,TaskScheduler.Default)).ToArray();
            await Task.WhenAll(tasks);
            var ids=tasks.SelectMany(t=>t.Result.groupIdToExtendedName.Keys).ToArray();
            Assert.Equal(ids.Length,ids.Distinct().Count());
            foreach(var task in tasks)
            {
                var result=task.Result;
                Assert.Equal(baseline.groupIdToExtendedName.Values.Order(),result.groupIdToExtendedName.Values.Order());
                Assert.Equal(baseline.Mesh.Triangles,result.Mesh.Triangles);
                Assert.Equal(baseline.Mesh.Positions,result.Mesh.Positions);
                Assert.Equal(baseline.Mesh.PrecisionPositions.Count,result.Mesh.PrecisionPositions.Count);
                for(int i=0;i<baseline.Mesh.PrecisionPositions.Count;i++)
                    Assert.True(baseline.Mesh.PrecisionPositions[i]==result.Mesh.PrecisionPositions[i]);
                for(int i=0;i<baseline.Mesh.TrianglesEx.Count;i++)
                {
                    var expected=baseline.Mesh.TrianglesEx[i];var actual=result.Mesh.TrianglesEx[i];
                    Assert.Equal(expected.V0,actual.V0);Assert.Equal(expected.V1,actual.V1);Assert.Equal(expected.V2,actual.V2);
                    Assert.Equal(baseline.groupIdToExtendedName[expected.GroupId],result.groupIdToExtendedName[actual.GroupId]);
                }
                Assert.Equal(baseline.surfaceMetaData.Keys.Order(),result.surfaceMetaData.Keys.Order());
                Assert.True(GeoAPI.GetBaseGroupIndex()>result.groupIdToExtendedName.Keys.Max());
            }
        }
        finally { File.Delete(path); }
    }
}
