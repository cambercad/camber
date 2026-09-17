using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

[Collection("GlobalCadState")]
public class ConcurrentCadOperationTests
{
    [Fact]
    public void ClearingRegistryDuringAutomaticPartCreationDoesNotReuseReservedNames()
    {
        var bounds = new Box3D(new Vec3D(-2), new Vec3D(2));
        Parallel.For(0, 8, worker =>
        {
            for (int iteration = 0; iteration < 250; iteration++)
            {
                if ((iteration + worker) % 3 == 0) GeoAPI.Clear();
                var part = new GeoAPI(bounds, .01);
                Assert.StartsWith("Part", part.Name);
            }
        });
    }

    [Fact]
    public void RegistryCleanupPreservesLiveBuildersAndAutomaticNames()
    {
        GeoAPI.Clear();
        var bounds = new Box3D(new Vec3D(-2), new Vec3D(3));
        var names = new System.Collections.Concurrent.ConcurrentBag<string>();
        Parallel.For(0, 8, worker =>
        {
            var api = new GeoAPI(bounds, .01);
            var first = api.CreateCube(CoordinateSystem.Default, 1);
            names.Add(first.Name);
            GeoAPI.Clear(resetNameCounters: false);
            var second = api.CreateCube(new CoordinateSystem(new Vec3D(.3, 0, 0)), 1);
            names.Add(second.Name);
            Assert.NotEqual(first.Name, second.Name);
            Assert.Same(first, api.GetMeshFromName(first.Name));
            Assert.Same(second, api.GetMeshFromName(second.Name));
            var joined = api.Boolean(first, second, BooleanOp.Union);
            Assert.True(MeshAnalysis.IsWatertightMesh(joined.Mesh.PrecisionPositions, joined.Mesh.Triangles));
        });
        Assert.Equal(16, names.Distinct().Count());
        GeoAPI.Clear();
    }

    [Fact]
    public void ConcurrentBooleansPreserveGeometryAndOwnTheirPairStatistics()
    {
        Parallel.For(0, 8, worker =>
        {
            var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(3)), .01,
                name: "parallel_boolean_" + Guid.NewGuid());
            var left = api.CreateCube(CoordinateSystem.Default, 1, "left");
            var right = api.CreateCube(new CoordinateSystem(new Vec3D(.3, 0, 0)), 1, "right");
            for (int iteration = 0; iteration < 12; iteration++)
            {
                var result = api.BatchUnion([left, right]);
                MeshTestHelpers.AssertValidMesh(result.Mesh.Positions, result.Mesh.Triangles);
                double volume = Math.Abs(result.Mesh.Triangles.Sum(t => Vec3DOps.Dot(
                    result.Mesh.Positions[t.A], Vec3DOps.Cross(result.Mesh.Positions[t.B], result.Mesh.Positions[t.C])))) / 6;
                Assert.InRange(volume, 1.2999, 1.3001);
                // LastResolveStats may describe any completed concurrent operation,
                // but each published snapshot must be internally consistent.
                var stats = Resolver.LastResolveStats;
                Assert.NotNull(stats);
                Assert.Equal(stats.BvhHitsAvB, stats.AabbRejects + stats.PairsProcessed);
                Assert.Equal(6 * stats.PairsProcessed, stats.SegTriAabbCulls + stats.SegTriFull);
            }
        });
    }
}
