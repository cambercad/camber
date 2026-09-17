using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public sealed class ExtrudeTopSketchPlaneTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    /// <summary>
    /// Rectangle ExtrudeTop used to fail GetPlotterSketcher: PlaneFitter used triangle
    /// centroids only, which are collinear for a 2-triangle quad, yielding a wrong normal.
    /// </summary>
    [Fact]
    public void ExtrudeTwoSides_GetPlotterSketcher_OnExtrudeTop_Succeeds()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100, -80, -20), new Vec3D(100, 80, 40)), 0.01);

        var baseSk = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "base_profile");
        baseSk.AddRectangleFromCorners(new Vec2D(0, 0), new Vec2D(80.0, 60.0));
        api.ExtrudeTwoSides(baseSk, 10.0, 10.0, name: "base");

        Assert.True(api.TryGetPlaneFromName("base-ExtrudeTop", out var plane));
        Assert.True(Math.Abs(plane.Normal.Z) > 0.99);

        var lipSk = api.GetPlotterSketcher("base-ExtrudeTop", "lip_profile");
        Assert.Equal("lip_profile", lipSk.Name);
    }
}
