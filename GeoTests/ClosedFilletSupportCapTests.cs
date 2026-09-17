using Geo;
using GeoCore;

namespace GeoTests;

public class ClosedFilletSupportCapTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NearbyUnrelatedPlaneDoesNotTruncateClosedJunction(bool unrelatedPlate)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(30)), .01);
        var upright = api.CreateCylinder(new CoordinateSystem(new Vec3D(0)), 5, 10, .02, "upright");
        var branch = api.CreateCylinder(new CoordinateSystem(new Vec3D(-12, 0, 2.5), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0)), 2, 12, .02, "branch");
        var blank = api.Boolean(upright, branch, CSG.BooleanOp.Union, "joint");
        if (unrelatedPlate)
        {
            var plate = api.CreateCuboid(new Vec3D(20, -1, -1), new Vec3D(22, 1, .7), "unrelated");
            blank = api.Boolean(blank, plate, CSG.BooleanOp.Union, "with_plate");
        }
        blank.EnsureCoplanarPostProcessed();
        var edges = blank.GroupEdges.Select(e => e.Name)
            .Where(n => n.Contains("upright-Circle1") && n.Contains("branch-Circle1")).ToList();
        Assert.True(edges.Count == 1, string.Join(";", blank.GroupEdges.Select(e => e.Name)));
        var rounded = api.Fillet(blank, edges, 1, .02, "rounded");
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(rounded.Mesh);
        // The actual upright cap is at Z = 0. A remote plate's Z = .7 plane must
        // not truncate the junction; its fillet is present below that level.
        Assert.True(RayMeshExact.TryCast(rounded, api.Converter, new Vec3D(-5.1, 0, -2), new Vec3D(0, 0, 1), out var hit));
        Assert.Equal(api.Converter.Convert(api.Converter.Convert(new Vec3D(0))).Z, hit.Point.Z, 10);
        if (unrelatedPlate)
        {
            Assert.True(RayMeshExact.TryCast(rounded, api.Converter, new Vec3D(21, 0, 2), new Vec3D(0, 0, -1), out var plateHit));
            Assert.InRange(plateHit.Point.Z, .699, .701);
        }
    }
}
