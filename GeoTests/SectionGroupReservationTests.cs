using Geo;
using GeoCore;

namespace GeoTests;

public class SectionGroupReservationTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void DisconnectedSectionCapsCanMaterializeBeforeCreatingAnotherSolid()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .01);
        var left = api.CreateCuboid(new Vec3D(-3, -1, -1), new Vec3D(-1, 1, 1), "left");
        var right = api.CreateCuboid(new Vec3D(1, -1, -1), new Vec3D(3, 1, 1), "right");
        var source = api.Boolean(left, right, CSG.BooleanOp.Union, "two_bodies");
        var originalGroups = source.groupIdToExtendedName.ToArray();
        var section = new SectionView(new[] { source }, api.Converter, CoordinateSystem.Default);
        var cut = Assert.Single(section.Meshes);

        // Rendering/export materializes the deferred face topology. The two
        // cap islands allocate a second group, so the first must be reserved.
        cut.EnsureCoplanarPostProcessed();
        Assert.Equal(2, cut.groupIdToExtendedName.Values.Count(name => name.StartsWith("section_cap")));
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(cut.Mesh);
        Assert.Equal(originalGroups, source.groupIdToExtendedName.ToArray());
        var later = api.CreateCube(CoordinateSystem.Default, 1, "later");
        Assert.Empty(cut.groupIdToExtendedName.Keys.Intersect(later.groupIdToExtendedName.Keys));
    }
}
