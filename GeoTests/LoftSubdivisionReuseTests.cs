using Curves;
using Geo;
using Geo.NurbsConstruction;
using GeoCore;

namespace GeoTests;

public class LoftSubdivisionReuseTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(LoftStyle.Ruled)]
    [InlineData(LoftStyle.Hermite)]
    [InlineData(LoftStyle.SmoothCatmullRom)]
    public void ShiftedPolygonConstructionPreservesInteriorDeviation(LoftStyle style)
    {
        const double deviation = .002;
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(30)), .005);
        (double Width, double Height)[] sizes = [(2, 4), (5, 3), (3, 7)];
        var profiles = new List<PlotterSketcherCoordSys>();
        for (int station = 0; station < sizes.Length; station++)
        {
            var frame = new CoordinateSystem(new Vec3D(0, 0, station * 10),
                new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
            var profile = new PlotterSketcherCoordSys("profile_" + station, frame, new Vec2D(0, 0));
            profile.AddRectangleFromCorners(new Vec2D(0, 0), new Vec2D(sizes[station].Width, sizes[station].Height));
            profiles.Add(profile);
        }
        var options = new LoftOptions
        {
            Style = style, AlignmentMode = LoftAlignmentMode.AsAuthored,
            ProfileSamplesU = 19, CreasePolicy = LoftCreasePolicy.None,
            ProfileSeamPoints = sizes.Select(size => LoftProfileSeamHint.FromPoint(
                new Vec2D(.137 * 2 * (size.Width + size.Height), 0))).ToList()
        };
        var timer = System.Diagnostics.Stopwatch.StartNew();
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(api.Converter, profiles, deviation, options,
            output, "shifted", out var names, 0);
        Console.WriteLine($"{style}: raw generation {timer.Elapsed.TotalSeconds:G5} s, {output.Vertices.Count} vertices, {output.Triangles.Count} triangles.");
        var metadata = NurbsPatchMetadataBuilder.BuildLoftMetadata(profiles, deviation, options,
            "shifted", output.LoftSideSupportFactory);
        var mesh = new MeshNormalUV(api.Converter, output.Vertices, output.Normals, output.UVs,
            output.Triangles, output.TriangleGroups, output.PrecisePositions);
        var body = new AnchorMesh("shifted", mesh, names, metadata, false, preserveTriangulation: true);
        LoftTriangleDeviationTests.AssertTriangleInteriors(body, "shifted-Side", deviation, api.Converter.SmallestUnit());
    }
}
