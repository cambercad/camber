using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class SlottedDrumTopologyTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void IndexedLighteningWindowsPreserveClosedDrum()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-90, -90, -30), new Vec3D(90, 160, 260)), .12);
        var ring = new PlotterSketcherCoordSys("ring", CoordinateSystem.Default, new Vec2D(0));
        ring.AddCircle(new Vec2D(0), 27);
        ring.AddCircle(new Vec2D(0), 20);
        var body = api.Extrude(ring, 220, name: "drum");
        double[] centers = [12, 61, 113, 165, 208];
        for (int i = 0; i < centers.Length; i++)
        {
            double length = i == 0 || i == 4 ? 16 : 28;
            double h = length / 2 - 2;
            var frame = new CoordinateSystem(new Vec3D(19, 0, centers[i]),
                new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
            var profile = new PlotterSketcherCoordSys($"window_{i}", frame, new Vec2D(0));
            profile.AddLine(new Vec2D(-2, -h), new Vec2D(-2, h));
            profile.AddArc(new Vec2D(-2, h), new Vec2D(0, length / 2), new Vec2D(2, h));
            profile.AddLine(new Vec2D(2, h), new Vec2D(2, -h));
            profile.AddArc(new Vec2D(2, -h), new Vec2D(0, -length / 2), new Vec2D(-2, -h));
            var cutter = api.Extrude(profile, 9, name: $"tool_{i}");
            var tools = api.BatchUnion(api.PatternCircular(cutter, 4, CoordinateSystem.Default, 2 * Math.PI, $"row_{i}").ToList());
            body = api.Boolean(body, tools, BooleanOp.Difference);
            Assert.True(MeshAnalysis.IsWatertightMesh(body.Mesh.PrecisionPositions, body.Mesh.Triangles), $"raw row {i}");
            body.EnsureCoplanarPostProcessed();
            Assert.True(MeshAnalysis.IsWatertightMesh(body.Mesh.PrecisionPositions, body.Mesh.Triangles), $"processed row {i}");
        }
    }
}
