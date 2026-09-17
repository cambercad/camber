using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class LoftNormalOrientationTests : IDisposable
{
    private static readonly string[] labels = ["bottom", "right", "top", "left"];

    private static AnchorMesh BuildTwistedLoft(GeoAPI part, bool clockwise, double tolerance)
    {
        (Vec3D origin, double width, double height, double angle)[] stations =
        [
            (new(0, 0, 0), 30, 10, 0),
            (new(0, 0, 14), 30, 10, 0),
            (new(10, 2, 40), 27, 8, 18),
            (new(-5, 5, 70), 23, 6, 42),
            (new(0, 7, 96), 18, 4, 65),
        ];
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (var station in stations)
        {
            double angle = station.angle * Math.PI / 180;
            var frame = new CoordinateSystem(station.origin,
                new Vec3D(Math.Cos(angle), Math.Sin(angle), 0),
                new Vec3D(-Math.Sin(angle), Math.Cos(angle), 0), new Vec3D(0, 0, 1));
            var sketch = new PlotterSketcherCoordSys("section" + sections.Count, frame);
            double x = station.width / 2, y = station.height / 2;
            Vec2D[] points = [new(-x, -y), new(x, -y), new(x, y), new(-x, y)];
            foreach (int i in clockwise ? new[] { 3, 2, 1, 0 } : new[] { 0, 1, 2, 3 })
                sketch.AddLine(points[clockwise ? (i + 1) % 4 : i],
                    points[clockwise ? i : (i + 1) % 4]).Name = labels[i];
            sections.Add(sketch);
        }
        return part.Loft(sections, new LoftOptions
        {
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices,
            FirstCurves = Enumerable.Repeat("bottom", sections.Count).ToArray(),
        }, "vane", tolerance);
    }

    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false, .04)]
    [InlineData(true, .04)]
    [InlineData(false, .15)]
    [InlineData(true, .15)]
    public void TwistedLoftNormalsFollowOneSidedSurfaceDerivatives(bool clockwise, double tolerance)
    {
        var part = new GeoAPI(new Box3D(new Vec3D(-50, -50, -15), new Vec3D(350, 300, 130)), tolerance);
        var solid = BuildTwistedLoft(part, clockwise, tolerance);
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions, solid.Mesh.Triangles) > 0);
        foreach (string label in labels)
        {
            Assert.True(solid.TryGetSurface("vane-Side-" + label, out var face));
            var range = face.NurbsParamRange.Value;
            foreach (int i in face.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
            {
                var uv = face.Uv[i];
                // At a crease, use the derivative from this face's side of the knot.
                double u = Math.Clamp(uv.X, Math.BitIncrement(range.UMin), Math.BitDecrement(range.UMax));
                var expected = face.NurbsSurface.EvaluateNormal(u, uv.Y).Normalized() * (clockwise ? -1 : 1);
                // Independently audit the evaluator using point differences. The
                // interval stays inside this face, so crease samples are one-sided.
                const double step = 1e-6;
                double u0 = Math.Max(range.UMin, u - step), u1 = Math.Min(range.UMax, u + step);
                double v0 = Math.Max(0, uv.Y - step), v1 = Math.Min(1, uv.Y + step);
                var du = face.NurbsSurface.Evaluate(u1, uv.Y) - face.NurbsSurface.Evaluate(u0, uv.Y);
                var dv = face.NurbsSurface.Evaluate(u, v1) - face.NurbsSurface.Evaluate(u, v0);
                var differenceNormal = Vec3DOps.Cross(du, dv).Normalized() * (clockwise ? -1 : 1);
                Assert.True(Vec3DOps.Dot(expected, differenceNormal) > .999999,
                    $"{label} uv={uv}: analytic normal disagrees with surface point differences");
                double alignment = Vec3DOps.Dot(expected, face.Normals[i]);
                Assert.True(alignment > .999999,
                    $"{label} uv={uv}: normal disagrees with oriented surface derivative ({alignment})");
            }
        }
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AllFourCurvedEdgesSupportEdgeTreatment(bool chamfer)
    {
        const double tolerance = .04;
        var part = new GeoAPI(new Box3D(new Vec3D(-50, -50, -15), new Vec3D(350, 300, 130)), tolerance);
        var original = BuildTwistedLoft(part, false, tolerance);
        var edges = Enumerable.Range(0, labels.Length)
            .Select(i => $"[vane-Side-{labels[i]},vane-Side-{labels[(i + 1) % labels.Length]}]").ToList();
        var treated = chamfer
            ? part.Chamfer(original, edges, .8, tolerance, "four_chamfers")
            : part.Fillet(original, edges, .8, tolerance, "four_rounds");
        MeshTestHelpers.AssertValidMesh(treated.Mesh.Positions, treated.Mesh.Triangles);
        Assert.True(MeshAnalysis.IsWatertightMesh(treated.Mesh.Positions, treated.Mesh.Triangles));
        double before = MeshAnalysis.ComputeSignedMeshVolume(original.Mesh.Positions, original.Mesh.Triangles);
        double after = MeshAnalysis.ComputeSignedMeshVolume(treated.Mesh.Positions, treated.Mesh.Triangles);
        Assert.InRange(after, before * .9, before - 10);
        string feature = chamfer ? "Chamfer" : "Blend";
        Assert.True(treated.groupIdToExtendedName.Values.Count(name => name.Contains(feature)) >= 4);
    }

}
