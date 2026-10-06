using System.Text.Json;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class ClosedSplineLoftRegressionTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(1, LoftStyle.Ruled)]
    [InlineData(2, LoftStyle.Ruled)]
    [InlineData(2, LoftStyle.SmoothCatmullRom)]
    public void ClosedSplineCapsShareEverySideBoundaryEdge(int end, LoftStyle style)
    {
        using var fixture = JsonDocument.Parse(File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "Fixtures", "closed_spline_loft.json")));
        var part = new GeoAPI(new Box3D(new Vec3D(-100,-100,-80), new Vec3D(100,100,260)), .025);
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (int index in new[] { 0, end })
        {
            var record = fixture.RootElement[index];
            var frame = record.GetProperty("frame");
            Vec3D Vector(string key) { var v = frame.GetProperty(key); return new(v[0].GetDouble(),v[1].GetDouble(),v[2].GetDouble()); }
            var sketch = new PlotterSketcherCoordSys("section"+index, new CoordinateSystem(Vector("origin"),Vector("x"),Vector("y"),Vector("z")));
            var points = record.GetProperty("points").EnumerateArray().Select(p => new Vec2D(p[0].GetDouble(),p[1].GetDouble())).ToArray();
            var t = record.GetProperty("tangent");
            var tangent = new Vec2D(t[0].GetDouble(),t[1].GetDouble());
            sketch.AddCubicHermiteSpline(points,tangent,tangent).Name = "boundary";
            sections.Add(sketch);
        }
        var options = new LoftOptions { CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices, FirstCurves = ["boundary","boundary"], Style = style };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(part.Converter,sections,.025,options,output,"closed",out _);
        MeshAnalysis.IsWatertightMesh(output.Vertices,output.Triangles,out var bad);
        Assert.True(MeshAnalysis.IsWatertightMesh(output.PrecisePositions,output.Triangles), "Open edges: " + string.Join("; ", bad.Take(20)));
        var solid = part.Loft(sections,options,"closed",.025);
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions,solid.Mesh.Triangles)>0);
    }
}
