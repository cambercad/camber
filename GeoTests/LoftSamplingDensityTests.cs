using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class LoftSamplingDensityTests
{
    [Theory]
    [InlineData(LoftStyle.Hermite)]
    [InlineData(LoftStyle.Ruled)]
    [InlineData(LoftStyle.SmoothCatmullRom)]
    public void StraightSquareLoftDoesNotSubdivideStraightEdges(LoftStyle style)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .01);
        var profiles = new List<PlotterSketcherCoordSys>();
        foreach (double z in new[] { 0d, 10d })
        {
            var frame = new CoordinateSystem(new Vec3D(0, 0, z), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
            var section = new PlotterSketcherCoordSys("section_" + z, frame, new Vec2D(-5, -5));
            section.AddRectangleFromCorners(new Vec2D(-5, -5), new Vec2D(5, 5));
            profiles.Add(section);
        }
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(api.Converter, profiles, .05,
            new LoftOptions { Style = style }, output, "box", out _, 0);
        // Eight corners plus the existing two cap fan centers.
        Assert.Equal(10, output.Vertices.Distinct().Count());
        Assert.Equal(16, output.Triangles.Count);
    }
}
