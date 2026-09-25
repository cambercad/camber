using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public sealed class LoftPlanarMetadataTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void RuledLoftOfStraightProfilesMarksExactlyPlanarSidePatches()
    {
        var part = CreatePart();
        var sections = new[]
        {
            Section("lower", 0, [new(-2, 0), new(2, 0), new(2, 4), new(-2, 4)]),
            Section("upper", 10, [new(-2, 0), new(2, 0), new(2, 4), new(-2, 4)])
        };

        var solid = part.Loft(sections, RuledOptions(), "planar", .005);

        foreach (string edge in new[] { "front", "right", "back", "left" })
        {
            var metadata = solid.surfaceMetaData[$"planar-Side-{edge}"];
            Assert.Equal(SurfaceType.Planar, metadata.SurfaceType);
            var plane = Assert.IsType<PlaneSurfaceParams>(metadata.PlaneParams);
            Assert.InRange(Math.Abs(plane.Normal.LengthSquared() - 1), 0, 1e-12);
            Assert.InRange(Math.Abs(plane.RefDir.LengthSquared() - 1), 0, 1e-12);
            Assert.InRange(Math.Abs(Vec3DOps.Dot(plane.Normal, plane.RefDir)), 0, 1e-12);
        }
    }

    [Fact]
    public void RuledLoftWithWarpedSideLeavesThatSideUnknown()
    {
        var part = CreatePart();
        var sections = new[]
        {
            Section("lower", 0, [new(-2, 0), new(2, 0), new(2, 4), new(-2, 4)]),
            Section("upper", 10, [new(-2, 0), new(2, 1), new(2, 5), new(-2, 4)])
        };

        var solid = part.Loft(sections, RuledOptions(), "warped", .005);

        Assert.Equal(SurfaceType.Unknown, solid.surfaceMetaData["warped-Side-front"].SurfaceType);
        Assert.Null(solid.surfaceMetaData["warped-Side-front"].PlaneParams);
    }

    [Fact]
    public void NearPlanarButNotExactlyPlanarRuledSideRemainsUnknown()
    {
        var part = CreatePart();
        var sections = new[]
        {
            Section("lower", 0, [new(-2, 0), new(2, 0), new(2, 4), new(-2, 4)]),
            Section("upper", 10, [new(-2, 0), new(2, .001), new(2, 4.001), new(-2, 4)])
        };

        var solid = part.Loft(sections, RuledOptions(), "near-planar", .005);

        Assert.Equal(SurfaceType.Unknown, solid.surfaceMetaData["near-planar-Side-front"].SurfaceType);
        Assert.Null(solid.surfaceMetaData["near-planar-Side-front"].PlaneParams);
    }

    private static GeoAPI CreatePart() =>
        new(new Box3D(new Vec3D(-20), new Vec3D(20)), .005, name: "loft-planarity-part");

    private static LoftOptions RuledOptions() => new()
    {
        Style = LoftStyle.Ruled,
        CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices,
        CapEnds = true
    };

    private static PlotterSketcherCoordSys Section(string name, double z, Vec2D[] points)
    {
        var sketch = new PlotterSketcherCoordSys(name, new CoordinateSystem(new Vec3D(0, 0, z)));
        string[] edgeNames = ["front", "right", "back", "left"];
        for (int i = 0; i < points.Length; i++)
            sketch.AddLine(points[i], points[(i + 1) % points.Length]).Name = edgeNames[i];
        return sketch;
    }
}
