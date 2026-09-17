using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class LoftFaceProvenanceTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static AnchorMesh Build(GeoAPI part, double width, bool reverse, double tolerance, IReadOnlyList<string> firstCurves = null)
    {
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (int station in new[] { 0, 1, 2 })
        {
            var sketch = new PlotterSketcherCoordSys("section" + station,
                new CoordinateSystem(new Vec3D(0, 0, station * 5)));
            double w = width + station;
            Vec2D[] points = [new(-w, 0), new(w, 0), new(w / 2, 8), new(-w / 2, 8)];
            string[] names = ["root", "pressure", "tip", "suction"];
            foreach (int i in reverse ? new[] { 3, 2, 1, 0 } : new[] { 0, 1, 2, 3 })
            {
                var a = points[i];
                var b = points[(i + 1) % 4];
                sketch.AddLine(reverse ? b : a, reverse ? a : b).Name = names[i];
            }
            sections.Add(sketch);
        }
        return part.Loft(sections, new LoftOptions
        {
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices,
            Style = LoftStyle.SmoothCatmullRom,
            FirstCurves = firstCurves
        }, "vane", tolerance);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AuthoredEdgesProduceSeparateFacesAndFaithfulAnalyticDomains(bool reverse)
    {
        var part = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), .02);
        var solid = Build(part, 2, reverse, .02);
        Assert.Equal(6, solid.groupIdToExtendedName.Count);
        foreach (string label in new[] { "root", "pressure", "tip", "suction" })
        {
            string name = "vane-Side-" + label;
            Assert.Contains(name, solid.groupIdToExtendedName.Values);
            Assert.True(solid.TryGetSurface(name, out var face));
            Assert.NotEmpty(face.Triangles);
            Assert.NotNull(face.NurbsSurface);
            var range = face.NurbsParamRange.Value;
            Assert.Equal(.25, range.UMax - range.UMin);
            foreach (int index in face.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
            {
                var uv = face.Uv[index];
                Assert.InRange(uv.X, range.UMin, range.UMax);
                Assert.InRange((face.Points[index] - face.NurbsSurface.Evaluate(uv.X, uv.Y)).Length(), 0, .001);
                // Semantic provenance must survive section orientation reversal.
                if (label == "pressure") Assert.True(face.Points[index].X > 0);
                if (label == "suction") Assert.True(face.Points[index].X < 0);
            }
        }
        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.Positions, solid.Mesh.Triangles));
    }

    [Fact]
    public void FaceNamesSurviveDimensionAndTessellationChanges()
    {
        var partA = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), .02);
        var partB = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), .08);
        var a = Build(partA, 2, false, .02);
        var b = Build(partB, 3, false, .08);
        Assert.Equal(a.groupIdToExtendedName.Values.Order(), b.groupIdToExtendedName.Values.Order());
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NamedFirstCurvePreservesItsStartPointAndDirection(bool clockwise)
    {
        var part = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), .02);
        var solid = Build(part, 2, clockwise, .02, ["pressure", "pressure", "pressure"]);
        Assert.True(solid.TryGetSurface("vane-Side-pressure", out var face));
        Assert.Equal(0, face.NurbsParamRange.Value.UMin);
        var start = face.NurbsSurface.Evaluate(0, 0);
        var end = face.NurbsSurface.Evaluate(.25, 0);
        var lower = new Vec3D(2, 0, 0);
        var upper = new Vec3D(1, 8, 0);
        Assert.InRange((start - (clockwise ? upper : lower)).Length(), 0, 1e-10);
        Assert.InRange((end - (clockwise ? lower : upper)).Length(), 0, 1e-10);
        MeshTestHelpers.AssertValidMesh(solid.Mesh.Positions, solid.Mesh.Triangles);
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions, solid.Mesh.Triangles) > 0);
        foreach (var name in solid.groupIdToExtendedName.Values.Where(n => n.EndsWith("Cap")))
        {
            Assert.True(solid.TryGetSurface(name, out var cap));
            foreach (var triangle in cap.Triangles)
            {
                var normal = Vec3DOps.Cross(cap.Points[triangle.B] - cap.Points[triangle.A],
                    cap.Points[triangle.C] - cap.Points[triangle.A]).Normalized();
                Assert.True(Vec3DOps.Dot(normal, cap.Normals[triangle.A]) > .999);
            }
        }
    }

    [Fact]
    public void InvalidFirstCurveListsAreRejected()
    {
        var part = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), .02);
        Assert.Throws<ArgumentException>(() => Build(part, 2, false, .02, ["root"]));
        Assert.Throws<ArgumentException>(() => Build(part, 2, false, .02, ["root", "missing", "root"]));
        Assert.Throws<ArgumentException>(() => Build(part, 2, false, .02, ["root", "", "root"]));
    }

}
