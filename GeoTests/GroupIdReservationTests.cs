using System.Collections.Concurrent;
using Geo;
using GeoCore;

namespace GeoTests;

public class GroupIdReservationTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void ConcurrentReservationsNeverReuseAnyIdInVariableSizedRanges()
    {
        var ranges = new ConcurrentBag<(int First, int Count)>();
        Parallel.For(0, 16, worker =>
        {
            for (int iteration = 0; iteration < 100; iteration++)
            {
                int count = 1 + (worker + iteration) % 7;
                ranges.Add((GeoAPI.ReserveGroupIds(count), count));
            }
        });
        var ids = ranges.SelectMany(range => Enumerable.Range(range.First, range.Count)).ToArray();
        Assert.Equal(ids.Length, ids.Distinct().Count());
        Assert.True(GeoAPI.GetBaseGroupIndex() > ids.Max());
        Assert.Throws<ArgumentOutOfRangeException>(() => GeoAPI.ReserveGroupIds(-1));
    }

    [Fact]
    public void ReservedPatchCanSplitDuringConstructionWithoutReusingItsId()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(8)), .01);
        int patchId = GeoAPI.ReserveGroupIds(1);
        var positions = new List<Vec3D> {
            new(0,0,0), new(1,0,0), new(0,1,0),
            new(3,0,0), new(4,0,0), new(3,1,0) };
        var triangles = new List<Tri> { new(0,1,2), new(3,4,5) };
        var mesh = new MeshNormalUV(api.Converter, positions,
            Enumerable.Repeat(new Vec3D(0,0,1),6).ToList(),
            Enumerable.Repeat(new Vec2D(0,0),6).ToList(), triangles,
            new List<int> { patchId,patchId }, skipWatertightCheck:true);
        // This eager constructor allocates another group for the second island.
        // The generated patch must already own its ID before entering it.
        var patch = new AnchorMesh("disconnected_patch", mesh,
            new Dictionary<int,string> { [patchId]="patch" },
            new Dictionary<string,SurfaceMetaData> { ["patch"]=new(SurfaceType.Planar) },
            deferCoplanarPostProcess:false, skipCoplanarFusion:true, isVolume:false);
        Assert.Equal(2,patch.groupIdToExtendedName.Count);
        Assert.Contains(patchId,patch.groupIdToExtendedName.Keys);
        Assert.Equal(new[] { "patch", "patch_1" }, patch.groupIdToExtendedName.Values.Order().ToArray());
        Assert.Equal(2,patch.Mesh.GetTriangleGroups().Distinct().Count());
        var later = api.CreateCube(CoordinateSystem.Default, 1, "later_cube");
        Assert.Empty(patch.groupIdToExtendedName.Keys.Intersect(later.groupIdToExtendedName.Keys));
    }
}
