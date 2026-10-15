using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

[Collection("GlobalCadState")]
public sealed class ExampleFeatureRegressionTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void TwistedRectangleLoftShellKeepsAllWallsAndOpening()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-32, -34, -3), new Vec3D(32, 18, 24)), .015);
        var sections = new List<PlotterSketcherCoordSys>();
        var starts = new List<string>();
        foreach (var (z, width, depth, degrees) in new[] {
            (0.0, 10.0, 7.0, 0.0), (5.0, 12.0, 8.0, 12.0),
            (10.0, 8.5, 6.0, 31.0), (16.0, 10.5, 7.5, 52.0) })
        {
            double angle = degrees * Math.PI / 180;
            var frame = new CoordinateSystem(new Vec3D(18, 0, z),
                new Vec3D(Math.Cos(angle), Math.Sin(angle), 0),
                new Vec3D(-Math.Sin(angle), Math.Cos(angle), 0), new Vec3D(0, 0, 1));
            var sketch = new PlotterSketcherCoordSys("section" + sections.Count, frame, new Vec2D(0));
            var points = new[] { new Vec2D(-width/2, -depth/2), new Vec2D(width/2, -depth/2),
                new Vec2D(width/2, depth/2), new Vec2D(-width/2, depth/2) };
            for (int i = 0; i < 4; i++)
            {
                var line = sketch.AddLine(points[i], points[(i+1)%4]);
                if (i == 0) starts.Add(line.Name);
            }
            sections.Add(sketch);
        }
        var blank = api.Loft(sections, new LoftOptions {
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices, FirstCurves = starts }, "vessel", .025);
        var shell = api.Shell(blank, .15, ["vessel-EndCap"], .025, "shell");
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        double volume = MeshAnalysis.ComputeSignedMeshVolume(shell.Mesh.Positions, shell.Mesh.Triangles);
        Assert.InRange(volume, 50, 150);
        Assert.Contains("ShellRim_vessel-EndCap", shell.extendedNameToGroupId.Keys);
    }
}
