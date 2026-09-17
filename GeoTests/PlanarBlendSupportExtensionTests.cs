using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class PlanarBlendSupportExtensionTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, 2)]
    [InlineData(true, 2)]
    public void RimChamferMeetingRoundedCornerIsIndependentOfBoundsAndSectionSampling(bool enlargedBounds, int spanSamples)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50, -50, -15),
            enlargedBounds ? new Vec3D(350, 300, 130) : new Vec3D(210, 150, 70)), .04);
        var housing = api.Loft(new List<PlotterSketcherCoordSys>
        {
            Rectangle("foot", 0, 32, 24), Rectangle("middle", 12, 30, 22), Rectangle("top", 24, 28, 20)
        }, new LoftOptions
        {
            FirstCurves = ["bottom", "bottom", "bottom"],
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices,
            ProfileSamplesU = spanSamples == 0 ? 0 : 32,
            VSubdivisionsPerSpan = spanSamples
        }, "housing", .04);
        var flange = api.Extrude(Rectangle("flange_profile", -6, 42, 34), 7, name: "flange");
        var body = api.Boolean(housing, flange, BooleanOp.Union);
        var rounded = api.Fillet(body, new List<string>
        {
            "[flange-bottom,flange-right]", "[flange-bottom,flange-ExtrudeTop]", "[flange-right,flange-ExtrudeTop]"
        }, 2, .04, "rounded_corner");
        var result = api.Chamfer(rounded, new List<string> { "[flange-left,flange-ExtrudeTop]" }, .7, .04, "meeting_treatments");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.Positions, result.Mesh.Triangles));
        double initialVolume = MeshAnalysis.ComputeSignedMeshVolume(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        double finalVolume = MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles);
        Assert.InRange(initialVolume - finalVolume, 1, 30);
        Assert.True(finalVolume > 0);
    }

    private static PlotterSketcherCoordSys Rectangle(string name, double z, double width, double height)
    {
        var frame = new CoordinateSystem(new Vec3D(0, 0, z), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sketch = new PlotterSketcherCoordSys(name, frame);
        Vec2D[] points = [new(-width / 2, -height / 2), new(width / 2, -height / 2), new(width / 2, height / 2), new(-width / 2, height / 2)];
        string[] names = ["bottom", "right", "top", "left"];
        for (int i = 0; i < points.Length; i++)
            sketch.AddLine(points[i], points[(i + 1) % points.Length]).Name = names[i];
        return sketch;
    }
}
