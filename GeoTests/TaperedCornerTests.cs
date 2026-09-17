using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class TaperedCornerTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static AnchorMesh TaperedBody(GeoAPI api, double middleWarp = 0)
    {
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (var station in new[] { (Z: 0.0, Width: 32.0, Height: 24.0),
                    (Z: 12.0, Width: 30.0, Height: 22.0), (Z: 24.0, Width: 28.0, Height: 20.0) })
        {
            var section = new PlotterSketcherCoordSys("section" + sections.Count,
                new CoordinateSystem(new Vec3D(0, 0, station.Z)));
            double x = station.Width / 2, y = station.Height / 2;
            var corner = new Vec2D(x, -y + (station.Z == 12 ? middleWarp : 0));
            section.AddLine(new Vec2D(-x, -y), corner).Name = "bottom";
            section.AddLine(corner, new Vec2D(x, y)).Name = "right";
            section.AddLine(new Vec2D(x, y), new Vec2D(-x, y)).Name = "top";
            section.AddLine(new Vec2D(-x, y), new Vec2D(-x, -y)).Name = "left";
            sections.Add(section);
        }
        return api.Loft(sections, new LoftOptions
        {
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices,
            FirstCurves = new[] { "bottom", "bottom", "bottom" },
        }, "housing", .04);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.002)]
    public void MatchedSidesPreserveAuthoredPlanesWithoutFlatteningWarpedSections(double warp)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50, -50, -15), new Vec3D(350, 300, 130)), .04);
        var body = TaperedBody(api, warp);
        foreach (string name in new[] { "bottom", "right", "top", "left" })
        {
            Assert.True(body.TryGetSurface("housing-Side-" + name, out var face));
            bool expectedPlanar = warp == 0 || name != "bottom";
            Assert.Equal(expectedPlanar, face.IsSurfacePlanar());
            foreach (int i in face.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
            {
                var uv = face.Uv[i];
                Assert.InRange((face.Points[i] - face.NurbsSurface.Evaluate(uv.X, uv.Y)).Length(), 0, .001);
            }
        }
        Assert.True(MeshAnalysis.IsWatertightMesh(body.Mesh.PrecisionPositions, body.Mesh.Triangles));
    }

    [Theory]
    [InlineData(3)]
    [InlineData(5)]
    public void NonDyadicFaceAndSectionKnotsRetainExactPlanarJunctions(int sides)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50, -50, -15), new Vec3D(350, 300, 130)), .04);
        Vec2D[] polygon = sides == 3
            ? new[] { new Vec2D(-12, -9), new Vec2D(12, -9), new Vec2D(0, 12) }
            : new[] { new Vec2D(-12, -9), new Vec2D(10, -9), new Vec2D(12, -7),
                new Vec2D(12, 9), new Vec2D(-12, 9) };
        var profiles = new List<PlotterSketcherCoordSys>();
        for (int station = 0; station < 4; station++)
        {
            var sketch = new PlotterSketcherCoordSys("profile" + station,
                new CoordinateSystem(new Vec3D(station, 0, station * 8)));
            for (int edge = 0; edge < sides; edge++)
                sketch.AddLine(polygon[edge], polygon[(edge + 1) % sides]).Name = "edge" + edge;
            profiles.Add(sketch);
        }
        var body = api.Loft(profiles, new LoftOptions
        {
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices,
            FirstCurves = Enumerable.Repeat("edge0", profiles.Count).ToArray(),
        }, "polygon", .04);
        for (int edge = 0; edge < sides; edge++)
        {
            Assert.True(body.TryGetSurface("polygon-Side-edge" + edge, out var face));
            Assert.True(face.IsSurfacePlanar());
            foreach (int i in face.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
            {
                var uv = face.Uv[i];
                Assert.InRange((face.Points[i] - face.NurbsSurface.Evaluate(uv.X, uv.Y)).Length(), 0, .001);
            }
        }
        Assert.True(MeshAnalysis.IsWatertightMesh(body.Mesh.PrecisionPositions, body.Mesh.Triangles));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TaperedThreeEdgeCornerClosesBothRoundsAndChamfers(bool chamfer)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50, -50, -15), new Vec3D(350, 300, 130)), .04);
        var body = TaperedBody(api);
        var edges = new List<string> { "[housing-Side-bottom,housing-Side-right]",
            "[housing-Side-bottom,housing-EndCap]", "[housing-Side-right,housing-EndCap]" };
        var result = chamfer ? api.Chamfer(body, edges, 1, .04, "chamfered")
            : api.Fillet(body, edges, 1, .04, "rounded");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        double before = MeshAnalysis.ComputeSignedMeshVolume(body.Mesh.Positions, body.Mesh.Triangles);
        double after = MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles);
        Assert.InRange(after, 0, before);
    }
}
