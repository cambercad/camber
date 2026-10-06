using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class PlanarBlendSupportExtensionTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AcutePrismCornerSupportsPartialAndCompleteBlends(bool chamfer, bool completeCorner)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10, -10, -10), new Vec3D(110, 30, 30)), .02);
        var sketch = new PlotterSketcherCoordSys("wedge", CoordinateSystem.Default);
        sketch.AddLine(new Vec2D(0, 0), new Vec2D(100, 0)).Name = "lower";
        sketch.AddLine(new Vec2D(100, 0), new Vec2D(100, 17.6326980708465)).Name = "back";
        sketch.AddLine(new Vec2D(100, 17.6326980708465), new Vec2D(0, 0)).Name = "upper";
        var solid = api.Extrude(sketch, 20, name: "prism");
        var edges = new List<string> { "[prism-lower,prism-upper]" };
        if (completeCorner)
        {
            edges.Add("[prism-lower,prism-ExtrudeTop]");
            edges.Add("[prism-upper,prism-ExtrudeTop]");
        }
        // A 10-degree corner needs a contact setback much larger than the
        // radius; it still fits comfortably within this 100 mm prism.
        var result = chamfer ? api.Chamfer(solid, edges, .25, .02, "bevel") :
            api.Fillet(solid, edges, .25, .02, "round");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.Positions, result.Mesh.Triangles));
        double before = MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions, solid.Mesh.Triangles);
        double after = MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles);
        Assert.InRange(after, 0, before - .01);
    }

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
