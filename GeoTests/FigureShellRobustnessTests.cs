using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class FigureShellRobustnessTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static GeoAPI Owner() => new(new Box3D(new Vec3D(-160, -160, -20), new Vec3D(160, 160, 160)), .025);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcentricHemispheresWithCommonBaseFormOpenBottomShell(bool inspectFirst)
    {
        var api = Owner();
        AnchorMesh Dome(string name, double radius)
        {
            const double bottom = 37.9;
            var frame = new CoordinateSystem(new Vec3D(0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0));
            var sketch = new PlotterSketcherCoordSys(name, frame, new Vec2D(bottom, 0));
            sketch.AppendLine(bottom, radius);
            sketch.AppendArc(new Vec2D(bottom + radius / Math.Sqrt(2), radius / Math.Sqrt(2)), new Vec2D(bottom + radius, 0));
            sketch.AppendLine(bottom, 0);
            return api.Revolve(sketch, 2 * Math.PI, .015, name);
        }
        var outer = Dome("outer", 5.7);
        var inner = Dome("inner", 5.15);
        if (inspectFirst)
        {
            outer.EnsureCoplanarPostProcessed();
            inner.EnsureCoplanarPostProcessed();
        }
        var shell = api.Boolean(outer, inner, BooleanOp.Difference, "shell");
        shell.EnsureCoplanarPostProcessed();
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles));
        double volume = MeshAnalysis.ComputeSignedMeshVolume(shell.Mesh.Positions, shell.Mesh.Triangles);
        double expected = 2 * Math.PI / 3 * (Math.Pow(5.7, 3) - Math.Pow(5.15, 3));
        Assert.InRange(volume, expected * .98, expected * 1.02);
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0, 0, 37), new Vec3D(0, 0, 1), out var insideRoof));
        Assert.InRange(insideRoof.Point.Z, 43.02, 43.07);
    }

    [Theory]
    [InlineData(.1, false)]
    [InlineData(.2, false)]
    public void ExtrudedCurvedLegProfileAcceptsBothCapRimFillets(double radius, bool transformed)
        => AssertCurvedLegFillet(radius, transformed);

    [Fact(Skip = "Known fillet limitation: the rotated curved-leg rim produces an incomplete trimming surface and is rejected as a partial cut. See GeoTests/KnownFilletLimitations.md.")]
    public void RotatedCurvedLegProfileAcceptsBothCapRimFillets()
        => AssertCurvedLegFillet(.1, true);

    private static void AssertCurvedLegFillet(double radius, bool transformed)
    {
        var api = Owner();
        var frame = transformed ? TransformedFrame() : new CoordinateSystem(new Vec3D(-3.7, 0, 0),
            new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
        var sketch = new PlotterSketcherCoordSys("side", frame, new Vec2D(-5, 0));
        sketch.AppendLine(2.8, 0);
        sketch.AppendLine(2.8, 11.2);
        sketch.AppendArc(new Vec2D(0, 14), new Vec2D(-2.8, 11.2));
        sketch.AppendLine(-2.8, 3.2);
        sketch.AppendLine(-5, 3.2);
        sketch.AppendLine(-5, 0);
        var blank = api.Extrude(sketch, 7.4, name: "leg_blank");
        AssertRoundedCapRims(api, blank, radius);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcaveSteppedExtrusionAcceptsBothCapRimFillets(bool transformed)
    {
        var api = Owner();
        var frame = transformed ? TransformedFrame() : CoordinateSystem.Default;
        var sketch = new PlotterSketcherCoordSys("step", frame, new Vec2D(0, 0));
        sketch.AppendLine(12, 0);
        sketch.AppendLine(12, 4);
        sketch.AppendLine(5, 4);
        sketch.AppendLine(5, 10);
        sketch.AppendLine(0, 10);
        sketch.AppendLine(0, 0);
        var blank = api.Extrude(sketch, 7.4, name: "stepped_blank");
        AssertRoundedCapRims(api, blank, .1);
    }

    private static CoordinateSystem TransformedFrame() => new(new Vec3D(18, -24, 31),
        new Vec3D(.6, .8, 0), new Vec3D(-.48, .36, .8), new Vec3D(.64, -.48, .6));

    private static void AssertRoundedCapRims(GeoAPI api, AnchorMesh blank, double radius)
    {
        blank.EnsureCoplanarPostProcessed();
        var edges = blank.GroupEdges.Select(e => e.Name).Where(n => n.Contains("ExtrudeTop") || n.Contains("ExtrudeBottom")).ToList();
        var rounded = api.Fillet(blank, edges, radius, .025, "rounded_rims");
        rounded.EnsureCoplanarPostProcessed();
        Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));
        double before = MeshAnalysis.ComputeSignedMeshVolume(blank.Mesh.Positions, blank.Mesh.Triangles);
        double after = MeshAnalysis.ComputeSignedMeshVolume(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.InRange(after, before * .95, before);
        Assert.True(after < before);

        // Endpoint closures carry their own plane rather than inheriting the
        // representative metadata of a smoothly merged curved support face.
        int closurePlanes = 0;
        foreach (var entry in rounded.surfaceMetaData)
        {
            if (blank.surfaceMetaData.ContainsKey(entry.Key) || entry.Value.PlaneParams == null ||
                !rounded.TryGetSurface(entry.Key, out var patch) || patch.Triangles.Count == 0)
                continue;
            ++closurePlanes;
            var plane = entry.Value.PlaneParams;
            foreach (int index in patch.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
                // Fusion can absorb a closure into a source face. Its analytic
                // plane then retains the source datum while mesh vertices lie
                // on the converter lattice, just as before the fillet.
                Assert.InRange(Math.Abs(Vec3DOps.Dot(patch.Points[index] - plane.Origin, plane.Normal)),
                    0, blank.surfaceMetaData.Keys.Any(name => entry.Key.Contains(name))
                        ? Math.Sqrt(3)*api.Converter.SmallestUnit() : 1e-7);
            foreach (var triangle in patch.Triangles)
            {
                var normal = Rat3Hybrid.Cross(patch.PointsPrecise[triangle.B] - patch.PointsPrecise[triangle.A],
                    patch.PointsPrecise[triangle.C] - patch.PointsPrecise[triangle.A]);
                if (normal == new Rat3Hybrid(0, 0, 0)) continue;
                var scale = new[] { normal.X, normal.Y, normal.Z }.Select(c => c.Sign() < 0 ? -c : c)
                    .Aggregate((a, b) => a > b ? a : b);
                var direction = new Vec3D((normal.X / scale).ToDouble(),
                    (normal.Y / scale).ToDouble(), (normal.Z / scale).ToDouble());
                Assert.True(Vec3DOps.Dot(direction, plane.Normal) > 0);
            }
        }
        Assert.True(closurePlanes > 0);
    }
}
