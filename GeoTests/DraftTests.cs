using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class DraftTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void SingleSideDraftKeepsNeutralCapAndOtherThreeSides()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .001);
        var block = api.CreateCuboid(new Vec3D(0, 0, 0), new Vec3D(4, 3, 2), "block");
        var side = block.extendedNameToGroupId.Single(kv =>
            block.Mesh.Triangles.Select((t, i) => (t, i))
                .Where(pair => block.Mesh.GetTriangleGroups()[pair.i] == kv.Value)
                .SelectMany(pair => new[] { pair.t.A, pair.t.B, pair.t.C })
                .All(v => Math.Abs(block.Mesh.Positions[v].X - 4) < 1e-6)).Key;
        var drafted = api.DraftPrismaticFaces(block, new List<string> { side }, new Plane3D(),
            new Vec3D(0, 0, 1), Math.PI / 18, name: "drafted");

        Assert.True(MeshAnalysis.IsWatertightMesh(drafted.Mesh.PrecisionPositions, drafted.Mesh.Triangles, true));
        var bottom = drafted.Mesh.Positions.Where(p => Math.Abs(p.Z) < 1e-4).ToArray();
        var top = drafted.Mesh.Positions.Where(p => Math.Abs(p.Z - 2) < 1e-4).ToArray();
        Assert.Equal(4, bottom.Max(p => p.X), 4);
        Assert.Equal(4 - 2 * Math.Tan(Math.PI / 18), top.Max(p => p.X), 3);
        Assert.Equal(0, top.Min(p => p.X), 4);
        Assert.Equal(0, top.Min(p => p.Y), 4);
        Assert.Equal(3, top.Max(p => p.Y), 4);
        Assert.Contains(side, drafted.extendedNameToGroupId.Keys);
        Assert.Equal(block.extendedNameToGroupId.Keys.Order(), drafted.extendedNameToGroupId.Keys.Order());
    }

    [Fact]
    public void RejectsNeutralPlaneInTheMiddleOfSolid()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .001);
        var block = api.CreateCuboid(new Vec3D(0, 0, 0), new Vec3D(4, 3, 2), "block");
        var side = block.extendedNameToGroupId.Keys.First();
        var plane = new Plane3D(new Vec3D(0, 0, 1), new Vec3D(0, 0, 1),
            new Vec3D(1, 0, 0), new Vec3D(0, 1, 0));
        Assert.Throws<NotSupportedException>(() => api.DraftPrismaticFaces(block,
            new List<string> { side }, plane, new Vec3D(0, 0, 1), .1));
    }
}
