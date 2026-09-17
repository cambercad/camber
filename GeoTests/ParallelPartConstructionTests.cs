using GeoMeta;
using Geo;
using GeoCore;

namespace GeoTests;

public class ParallelPartConstructionTests
{
    [Fact]
    public async Task IndependentBuildersRetainEverySketchMeshAndAssembly()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .1);
        var initialMeshes = api.GetMeshes();
        var initialSketches = api.GetSketches();
        var initialAssemblies = api.GetAssemblies();
        using var start = new ManualResetEventSlim();
        var workers = Enumerable.Range(0, 4).Select(worker => Task.Run(() =>
        {
            start.Wait();
            for (int item = 0; item < 30; item++)
            {
                string name = $"worker_{worker}_{item}";
                var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, name + "_sketch");
                sketch.AddRectangleFromCorners(new Vec2D(0), new Vec2D(2, 3));
                var solid = api.ExtrudeTwoSides(sketch, 2, 2, name: name + "_solid");
                var assembly = api.GetAssembly(name);
                assembly.AddPart(solid, new Vec3D(0));
                Assert.Same(solid, api.GetMeshFromName(name + "_solid"));
                Assert.Same(assembly, api.GetAssembly(name));
                Assert.Single(assembly.GetParts());
                // Enumerate snapshots while other workers register geometry.
                Assert.All(api.GetMeshes(), mesh => Assert.NotNull(mesh));
                Assert.All(api.GetSketches(), sk => Assert.NotNull(sk));
            }
        })).ToArray();
        start.Set();
        await Task.WhenAll(workers);
        Assert.Equal(120, api.GetMeshes().Count);
        Assert.Equal(120, api.GetSketches().Count);
        Assert.Equal(120, api.GetAssemblies().Count);
        Assert.Equal(120, api.GetMeshes().Select(m => m.Name).Distinct().Count());
        Assert.Empty(initialMeshes);
        Assert.Empty(initialSketches);
        Assert.Empty(initialAssemblies);
    }

    [Fact]
    public async Task ConcurrentAssemblyLookupReturnsOneInstance()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .1);
        var results = await Task.WhenAll(Enumerable.Range(0, 100)
            .Select(_ => Task.Run(() => api.GetAssembly("shared"))));
        Assert.All(results, assembly => Assert.Same(results[0], assembly));
        Assert.Single(api.GetAssemblies());
    }

    [Fact]
    public async Task ConcurrentDuplicateSketchRegistrationHasOneWinner()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .1);
        var results = await Task.WhenAll(Enumerable.Range(0, 100).Select(_ => Task.Run(() =>
        {
            try { api.GetPlotterSketcher(DefaultPlanes.OriginXY, "shared"); return true; }
            catch (NameCollisionException) { return false; }
        })));
        Assert.Single(results, success => success);
        Assert.Single(api.GetSketches());
    }
}
