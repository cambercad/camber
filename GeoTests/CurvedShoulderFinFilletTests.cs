using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public sealed class CurvedShoulderFinFilletTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void ReducedFinRootReproducesPartialTerminationLimitation()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-130, -130, -10),
            new Vec3D(130, 130, 115)), .15);
        var meridianFrame = new Plane3D(new Vec3D(0), new Vec3D(0, 1, 0),
            new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
        var meridian = api.GetPlotterSketcher(meridianFrame, "hub_meridian");
        meridian.AddLine(new Vec2D(0, 20), new Vec2D(0, 120));
        meridian.AddLine(new Vec2D(0, 120), new Vec2D(6, 120));
        meridian.AddLine(new Vec2D(6, 120), new Vec2D(6, 112));
        meridian.AddArc(new Vec2D(6, 112),
            new Vec2D(90 - 84 / Math.Sqrt(2), 112 - 84 / Math.Sqrt(2)), new Vec2D(90, 28));
        meridian.AddLine(new Vec2D(90, 28), new Vec2D(96, 28));
        meridian.AddLine(new Vec2D(96, 28), new Vec2D(96, 20));
        meridian.AddLine(new Vec2D(96, 20), new Vec2D(0, 20));
        var hub = api.Revolve(meridian, 2 * Math.PI, name: "revolved_hub");

        (double radius, double angle, double top, double thickness)[] stations =
        [
            (27.5, 0, 92, 2.8),
            (48, -9, 72, 2.5),
            (84, -24, 39, 2.1),
            (104, -33, 26, 1.9),
            (120, -40, 20, 1.8)
        ];
        var sections = new List<PlotterSketcherCoordSys>();
        for (int i = 0; i < stations.Length; i++)
        {
            var (radius, degrees, top, thickness) = stations[i];
            double theta = degrees * Math.PI / 180;
            double bottom = (radius <= 28 ? 90 : radius >= 112 ? 6 :
                90 - Math.Sqrt(84 * 84 - (112 - radius) * (112 - radius))) - 2;
            var frame = new CoordinateSystem(
                new Vec3D(radius * Math.Cos(theta), radius * Math.Sin(theta), bottom),
                new Vec3D(-Math.Sin(theta), Math.Cos(theta), 0), new Vec3D(0, 0, 1),
                new Vec3D(Math.Cos(theta), Math.Sin(theta), 0));
            var section = new PlotterSketcherCoordSys("fin_station_" + i, frame,
                new Vec2D(-thickness / 2, 0));
            section.AddRectangleFromCorners(new Vec2D(-thickness / 2, 0),
                new Vec2D(thickness / 2, top - bottom));
            sections.Add(section);
        }

        var fin = api.Loft(sections, new LoftOptions
        {
            Style = LoftStyle.SmoothCatmullRom,
            CorrespondenceMode = LoftCorrespondenceMode.MergedArcLengthAnchors,
            CreasePolicy = LoftCreasePolicy.None
        }, "fin");
        var joined = api.Boolean(hub, fin, BooleanOp.Union, "joined");
        joined.EnsureCoplanarPostProcessed();
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(joined.Mesh);

        int edge = joined.GroupEdges.FindIndex(item =>
            item.Name.Contains("revolved_hub-Line4") && item.Name.Contains("fin-Side"));
        Assert.True(edge >= 0, "The curved-shoulder/fin fixture must expose the intended root edge.");

        var error = Assert.Throws<InvalidOperationException>(() =>
            api.Fillet(joined, [joined.GetEdgeReference(edge)], 1.2, .15, "rounded_fin_root"));
        Assert.Contains("endpoint contour crosses the original trim face", error.Message);
    }
}
