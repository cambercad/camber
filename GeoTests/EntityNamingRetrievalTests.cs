using Curves;
using Geo;
using GeoCore;
using GeoMeta;
using GeoSolver;

namespace GeoTests;

/// <summary>
/// Integration tests for mesh-scoped name retrieval via GeoAPI.
/// </summary>
public sealed class EntityNamingRetrievalTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void TryGetEdgeFromName_FindsEdgeOnNonTopMesh()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), maxDeviation: 0.05);

        var first = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "First");
        first.AddLine(new Vec2D(0, 0), new Vec2D(5, 0));
        first.AddLine(new Vec2D(5, 0), new Vec2D(5, 5));
        first.AddLine(new Vec2D(5, 5), new Vec2D(0, 5));
        first.AddLine(new Vec2D(0, 5), new Vec2D(0, 0));
        var boxA = api.Extrude(first, 2.0, name: "boxA");

        var second = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "Second");
        second.AddLine(new Vec2D(0, 0), new Vec2D(3, 0));
        second.AddLine(new Vec2D(3, 0), new Vec2D(3, 3));
        second.AddLine(new Vec2D(3, 3), new Vec2D(0, 3));
        second.AddLine(new Vec2D(0, 3), new Vec2D(0, 0));
        api.Extrude(second, 1.0, name: "boxB");

        string edgeOnA = EntityNaming.FormatGroupEdgeName(
            EntityNaming.ExtrudeSide("boxA", "Line1"),
            EntityNaming.ExtrudeTop("boxA"));

        Assert.True(api.TryGetEdgeFromName("boxA:" + edgeOnA, out var edge));
        Assert.NotNull(edge);

        Assert.True(api.TryGetEdgeFromName(edgeOnA, out var edgeUnqualified));
        Assert.NotNull(edgeUnqualified);
    }

    [Fact]
    public void TryGetPointFromName_UnqualifiedLookup_SurvivesOlderMeshWithoutAnchor()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), maxDeviation: 0.05);

        var first = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "First");
        first.AddLine(new Vec2D(0, 0), new Vec2D(5, 0));
        first.AddLine(new Vec2D(5, 0), new Vec2D(5, 5));
        first.AddLine(new Vec2D(5, 5), new Vec2D(0, 5));
        first.AddLine(new Vec2D(0, 5), new Vec2D(0, 0));
        api.Extrude(first, 2.0, name: "boxA");

        var second = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "Second");
        second.AddLine(new Vec2D(0, 0), new Vec2D(3, 0));
        second.AddLine(new Vec2D(3, 0), new Vec2D(3, 3));
        second.AddLine(new Vec2D(3, 3), new Vec2D(0, 3));
        second.AddLine(new Vec2D(0, 3), new Vec2D(0, 0));
        api.Extrude(second, 1.0, name: "boxB");

        string patchA = EntityNaming.ExtrudeSide("boxA", "Line1");
        string patchB = EntityNaming.ExtrudeTop("boxA");
        string edgePoint = EntityNaming.FormatEdgePointAddress(patchA, patchB, 0.5);

        Assert.True(api.TryGetPointFromName(edgePoint, out var point));
        Assert.False(double.IsNaN(point.X));
    }

    [Fact]
    public void TryGetPointFromName_ResolvesLegacyEdgePointOnMesh()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), maxDeviation: 0.05);

        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "S");
        sketch.AddLine(new Vec2D(0, 0), new Vec2D(4, 0));
        sketch.AddLine(new Vec2D(4, 0), new Vec2D(4, 4));
        sketch.AddLine(new Vec2D(4, 4), new Vec2D(0, 4));
        sketch.AddLine(new Vec2D(0, 4), new Vec2D(0, 0));
        var mesh = api.Extrude(sketch, 1.0, name: "legacyBox");

        string patchA = EntityNaming.ExtrudeSide("legacyBox", "Line1");
        string patchB = EntityNaming.ExtrudeTop("legacyBox");
        string legacy = EntityNaming.FormatLegacyEdgePointName(patchA, patchB, 0, 0.5);

        Assert.True(api.TryGetPointFromName("legacyBox:" + legacy, out var point));
        Assert.False(double.IsNaN(point.X));
    }
}
