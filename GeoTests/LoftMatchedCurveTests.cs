using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class LoftMatchedCurveTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static AnchorMesh Build(GeoAPI part, bool reverse, string first, double radius, double tolerance)
    {
        var sections = new List<PlotterSketcherCoordSys>();
        for (int station = 0; station < 3; station++)
        {
            double a = 8 - station, r = radius + station;
            var sketch = new PlotterSketcherCoordSys("section" + station,
                new CoordinateSystem(new Vec3D(station * 2, 0, station * 20)));
            sketch.AddLine(new(-20, 0), new(20, 0), CurveFlags.HelperGeometry).Name = "datum";
            if (!reverse)
            {
                sketch.AddLine(new(-a, -r), new(a, -r)).Name = "bottom";
                sketch.AddArc(new(a, -r), new(a + r, 0), new(a, r)).Name = "nose";
                sketch.AddLine(new(a, r), new(-a, r)).Name = "top";
                sketch.AddArc(new(-a, r), new(-a - r, 0), new(-a, -r)).Name = "tail";
            }
            else
            {
                sketch.AddArc(new(-a, -r), new(-a - r, 0), new(-a, r)).Name = "tail";
                sketch.AddLine(new(-a, r), new(a, r)).Name = "top";
                sketch.AddArc(new(a, r), new(a + r, 0), new(a, -r)).Name = "nose";
                sketch.AddLine(new(a, -r), new(-a, -r)).Name = "bottom";
            }
            sections.Add(sketch);
        }
        return part.Loft(sections, new LoftOptions
        {
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices,
            FirstCurves = Enumerable.Repeat(first, sections.Count).ToArray(),
        }, "capsule", tolerance);
    }

    [Theory]
    [InlineData(false, "bottom")]
    [InlineData(true, "bottom")]
    [InlineData(false, "nose")]
    [InlineData(true, "nose")]
    public void NamedArcAndLineSectionsKeepDistinctExactFaces(bool reverse, string first)
    {
        var part = new GeoAPI(new Box3D(new Vec3D(-50), new Vec3D(100)), .04);
        var solid = Build(part, reverse, first, 3, .04);
        Assert.Equal(6, solid.groupIdToExtendedName.Count);
        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.Positions, solid.Mesh.Triangles));
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions, solid.Mesh.Triangles) > 0);
        foreach (string label in new[] { "bottom", "nose", "top", "tail" })
        {
            Assert.True(solid.TryGetSurface("capsule-Side-" + label, out var face));
            var range = face.NurbsParamRange.Value;
            Assert.Equal(.25, range.UMax - range.UMin);
            foreach (int i in face.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
            {
                var uv = face.Uv[i];
                Assert.InRange(uv.X, range.UMin, range.UMax);
                Assert.InRange((face.Points[i] - face.NurbsSurface.Evaluate(uv.X, uv.Y)).Length(), 0, .001);
            }
            if (label is "nose" or "tail")
            {
                for (int station = 0; station < 3; station++)
                for (int i = 0; i <= 8; i++)
                {
                    double u = range.UMin + (range.UMax - range.UMin) * i / 8;
                    var point = face.NurbsSurface.Evaluate(u, station / 2.0);
                    double centerX = station * 2 + (label == "nose" ? 1 : -1) * (8 - station);
                    Assert.InRange(Math.Abs((point - new Vec3D(centerX, 0, station * 20)).Length() - (3 + station)), 0, 1e-10);
                }
            }
        }
        Assert.True(solid.TryGetSurface("capsule-Side-" + first, out var firstFace));
        Assert.Equal(0, firstFace.NurbsParamRange.Value.UMin);
        var start = firstFace.NurbsSurface.Evaluate(0, 0);
        var expected = first == "bottom" ? new Vec3D(reverse ? 8 : -8, -3, 0) : new Vec3D(8, reverse ? 3 : -3, 0);
        Assert.InRange((start - expected).Length(), 0, 1e-10);
    }

    [Fact]
    public void FaceNamesSurviveRadiusAndTessellationChanges()
    {
        var a = Build(new GeoAPI(new Box3D(new Vec3D(-50), new Vec3D(100)), .04), false, "nose", 3, .04);
        var b = Build(new GeoAPI(new Box3D(new Vec3D(-50), new Vec3D(100)), .12), false, "nose", 4, .12);
        Assert.Equal(a.groupIdToExtendedName.Values.Order(), b.groupIdToExtendedName.Values.Order());
    }
    [Fact]
    public void MissingCurveNamesAndIncompatibleRationalSectionsAreRejected()
    {
        var part = new GeoAPI(new Box3D(new Vec3D(-50), new Vec3D(100)), .04);
        Assert.Throws<ArgumentException>(() => Build(part, false, "missing", 3, .04));
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (double angle in new[] { Math.PI / 2, Math.PI })
        {
            var sketch = new PlotterSketcherCoordSys("arc" + sections.Count,
                new CoordinateSystem(new Vec3D(0, 0, sections.Count * 10)));
            Vec2D Point(double t) => new(Math.Cos(t) * 5, Math.Sin(t) * 5);
            sketch.AddArc(Point(0), Point(angle / 2), Point(angle)).Name = "arc";
            sketch.AddLine(Point(angle), Point(0)).Name = "chord";
            sections.Add(sketch);
        }
        var error = Assert.Throws<ArgumentException>(() => part.Loft(sections, new LoftOptions
        {
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices,
            FirstCurves = ["arc", "arc"],
        }, "incompatible", .04));
        Assert.Contains("rational weights", error.Message);
    }

}
