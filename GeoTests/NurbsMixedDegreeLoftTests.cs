using Curves;
using Geo;
using Geo.NurbsConstruction;
using GeoCore;
using GeoSolver;
using NURBS;

namespace GeoTests;

public class NurbsMixedDegreeLoftTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void Concatenate_LineArcAndCubic_PreservesEachParametricSegment()
    {
        var line = new BSplineLine(new Vec3D(0, 0, 0), new Vec3D(1, 0, 0));
        var arc = new BSplineCircle(new Vec3D(1, 1, 0), new Vec3D(0, 0, 1),
            1, new Vec3D(0, -1, 0), 0, .5).GetNativeParamRangeCurve();
        var cubic = new BezierCurve(arc.End, new Vec3D(2, 3, 0),
            new Vec3D(3, 1, 0), new Vec3D(4, 2, 0));
        BSplineCurve[] segments = [line, arc, cubic];
        var joined = NurbsSurfaceFactory.ConcatenateCompatible(segments);
        Assert.Equal(3, joined.Degree);
        for (int s = 0; s < segments.Length; s++)
            for (int i = 0; i <= 100; i++)
            {
                double u = i / 100.0;
                Assert.InRange((joined.EvaluateUniform((s + u) / segments.Length)
                    - segments[s].EvaluateUniform(u)).Length(), 0, 2e-12);
            }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void Concatenate_NonuniformSmoothSpans_PreservesShapeAndInput(int degree)
    {
        var points = Enumerable.Range(0, degree + 2)
            .Select(i => new Vec3D(i, i % 2, 0)).ToArray();
        var knots = Enumerable.Repeat(0.0, degree + 1).Concat(new[] { .35 })
            .Concat(Enumerable.Repeat(1.0, degree + 1)).ToArray();
        var source = new BSplineCurve(degree, points, knots);
        var end = source.End;
        var target = new BezierCurve(end, end + new Vec3D(1, 0, 0),
            end + new Vec3D(2, 1, 0), end + new Vec3D(3, 1, 0), end + new Vec3D(4, 0, 0));
        var joined = NurbsSurfaceFactory.ConcatenateCompatible(new[] { source, target });
        Assert.Equal(4, joined.Degree);
        Assert.Equal(degree + 2, source.ControlPoints.Length);
        Assert.Equal(knots, source.Knots);
        for (int i = 0; i <= 100; i++)
        {
            double u = i / 100.0;
            Assert.InRange((joined.EvaluateUniform(u / 2) - source.EvaluateUniform(u)).Length(), 0, 2e-12);
        }
    }

    [Fact]
    public void Loft_CapsulesWithLinesAndArcs_ProducesClosedVolumeAndNurbsMetadata()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .02);
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (double z in new[] { 0.0, 4.0, 8.0 })
        {
            var frame = new CoordinateSystem(new Vec3D(0, 0, z),
                new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
            var sketch = new PlotterSketcherCoordSys($"section{z}", frame, new Vec2D(0, 2));
            sketch.AppendLine(6, 2);
            sketch.AddArc(new Vec2D(6, 2), new Vec2D(8, 0), new Vec2D(6, -2));
            sketch.AddLine(new Vec2D(6, -2), new Vec2D(0, -2));
            sketch.AddArc(new Vec2D(0, -2), new Vec2D(-2, 0), new Vec2D(0, 2));
            sections.Add(sketch);
        }
        var loft = api.Loft(sections, new LoftOptions { CapEnds = true }, "capsule_loft", .02);
        Assert.True(MeshAnalysis.IsWatertightMesh(loft.Mesh.Positions, loft.Mesh.Triangles));
        foreach (var name in NurbsPatchValidation.AllNamedPatches(loft))
            Assert.NotNull(loft.surfaceMetaData[name].NurbsSurface);
    }
}
