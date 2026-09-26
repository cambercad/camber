using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class StandardHoleTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static GeoAPI Api() => new(new Box3D(new Vec3D(-2), new Vec3D(12)), .025);
    private static AnchorMesh Blank(GeoAPI api) =>
        api.CreateCuboid(new Vec3D(0, 0, 0), new Vec3D(10, 10, 6), "blank");
    private static CoordinateSystem Mouth => new(new Vec3D(5, 5, 6),
        new Vec3D(1, 0, 0), new Vec3D(0, -1, 0), new Vec3D(0, 0, -1));

    private static double Volume(AnchorMesh mesh)
    {
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(mesh.Mesh);
        return MeshAnalysis.ComputeSignedMeshVolume(mesh.Mesh.Positions, mesh.Mesh.Triangles);
    }

    [Fact]
    public void BlindAndThroughDrillsRemoveTheirSpecifiedDepths()
    {
        var api = Api();
        var blank = Blank(api);
        double before = Volume(blank);
        var blind = api.DrillHole(blank, Mouth, 2, 3, name: "blind");
        var through = api.DrillHole(blank, Mouth, 2, name: "through");
        Assert.InRange(before - Volume(blind), 3 * Math.PI * .95, 3 * Math.PI * 1.05);
        Assert.InRange(before - Volume(through), 6 * Math.PI * .95, 6 * Math.PI * 1.05);
        Assert.True(Volume(through) < Volume(blind));
    }

    [Fact]
    public void CounterboreAndCountersinkAddExpectedMouthVolume()
    {
        var api = Api();
        var blank = Blank(api);
        double before = Volume(blank);
        var bore = api.CounterboreHole(blank, Mouth, 2, 4, 4, 1, name: "counterbore");
        double expectedBore = Math.PI * (4 + 3);
        Assert.InRange(before - Volume(bore), expectedBore * .95, expectedBore * 1.05);

        var sink = api.CountersinkHole(blank, Mouth, 2, 4, 4, Math.PI / 2, name: "countersink");
        // One millimetre of 90-degree frustum replaces the top of a 2 mm bore.
        double frustum = Math.PI / 3 * (2 * 2 + 2 * 1 + 1 * 1);
        double expectedSink = 4 * Math.PI + frustum - Math.PI;
        Assert.InRange(before - Volume(sink), expectedSink * .95, expectedSink * 1.05);
    }

    [Fact]
    public void InvalidDimensionsFailBeforeBuildingCutters()
    {
        var api = Api();
        var blank = Blank(api);
        Assert.Throws<ArgumentOutOfRangeException>(() => api.DrillHole(blank, Mouth, 0, 2));
        Assert.Throws<ArgumentOutOfRangeException>(() => api.DrillHole(blank, Mouth, 2, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => api.CounterboreHole(blank, Mouth, 2, 4, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => api.CounterboreHole(blank, Mouth, 2, 4, 4, 4));
        Assert.Throws<ArgumentOutOfRangeException>(() => api.CountersinkHole(blank, Mouth, 2, 4, 4, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => api.CountersinkHole(blank, Mouth, 2, 1, 4, Math.PI / 2));
    }
}
