using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class SketchSplineTessellationTests
{
    [Fact]
    public void RationalSketchCurveMatchesSpatialEvaluatorAndReversesItsNormal()
    {
        var controls = new[] { new Vec2D(2, 0), new Vec2D(2, 2), new Vec2D(0, 2) };
        var weights = new[] { 1.0, Math.Sqrt(.5), 1.0 };
        var spline = new BSpline2D(controls, new double[] { 2, 2, 2, 5, 5, 5 }, 2, weights);
        var spatial = new NURBS.BSplineCurve(2,
            new[] { new Vec3D(2, 0, 0), new Vec3D(2, 2, 0), new Vec3D(0, 2, 0) },
            weights, new double[] { 0, 0, 0, 1, 1, 1 });
        var reverse = spline.Reverse();
        for (int i = 0; i <= 20; i++)
        {
            double u = i / 20.0;
            var vertex = spline.EvaluateVertex(u);
            var expected = spatial.EvaluateUniform(u);
            Assert.InRange((vertex.Position - new Vec2D(expected.X, expected.Y)).Length(), 0, 1e-12);
            var reversed = reverse.EvaluateVertex(1-u);
            Assert.InRange((vertex.Position - reversed.Position).Length(), 0, 1e-12);
            Assert.InRange((vertex.Normal + reversed.Normal).Length(), 0, 1e-12);
            Assert.InRange(Math.Abs(vertex.Normal.Length()-1), 0, 1e-12);
            // CCW circle: the common right-hand curve normal points outward.
            Assert.InRange((vertex.Normal-vertex.Position*.5).Length(), 0, 1e-12);
        }
    }

    [Fact]
    public void SketchFactoriesRetainAnalyticCurvesAndProduceClosedSolid()
    {
        var api = new Geo.GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .005);
        try
        {
            var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "spline_profile");
            var bezier = sketch.AddBezier(new[] { new Vec2D(-2, 0), new Vec2D(-2, 2), new Vec2D(2, 2), new Vec2D(2, 0) });
            var spline = sketch.AddBSpline(new[] { new Vec2D(2, 0), new Vec2D(2, -2), new Vec2D(-2, -2), new Vec2D(-2, 0) });
            Assert.NotEmpty(bezier.Name);
            Assert.NotEmpty(spline.Name);
            Assert.NotEqual(bezier.Name, spline.Name);
            var solid = api.Extrude(sketch, 3, .005, "spline_solid");
            Assert.True(solid.IsVolume);
            Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions, solid.Mesh.Triangles, true));
        }
        finally { Geo.GeoAPI.Clear(resetNameCounters: false); }
    }

    [Fact]
    public void SketchSplineRejectsInvalidDataBeforeEvaluation()
    {
        var points = new[] { new Vec2D(0, 0), new Vec2D(1, 1) };
        Assert.Throws<ArgumentException>(() => new BSpline2D(points, new double[] { 0, 0, 1, 1 }, 1, new[] { 1.0, 0.0 }));
        Assert.Throws<ArgumentException>(() => new BSpline2D(points, new double[] { 0, 1, 0, 1 }, 1));
        Assert.Throws<ArgumentException>(() => new Bezier2D(new[] { new Vec2D(double.NaN, 0), points[1] }));
    }

    [Fact]
    public void NonunitRationalKnotDomainSurvivesExtrusionMetadata()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), .001);
        try
        {
            var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "rational_profile");
            sketch.AddBSpline(new[] { new Vec2D(2, 0), new Vec2D(2, 2), new Vec2D(0, 2) },
                2, new double[] { 2, 2, 2, 5, 5, 5 }, new[] { 1.0, Math.Sqrt(.5), 1.0 });
            sketch.AddLine(new Vec2D(0, 2), new Vec2D(0, 0));
            sketch.AddLine(new Vec2D(0, 0), new Vec2D(2, 0));
            var solid = api.Extrude(sketch, 1, .001, "quarter_cylinder");
            Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions, solid.Mesh.Triangles, true));
            Assert.InRange(Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions, solid.Mesh.Triangles)-Math.PI), 0, .01);
        }
        finally { GeoAPI.Clear(resetNameCounters: false); }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StraightSplineNeedsOnlyEndpoints(bool rational)
    {
        var controls = new[] { new Vec2D(0, 0), new Vec2D(3, 0), new Vec2D(7, 0), new Vec2D(10, 0) };
        var knots = new double[] { 2, 2, 2, 2, 5, 5, 5, 5 };
        var spline = rational
            ? new BSpline2D(controls, knots, 3, new double[] { 1, 2, 3, 1 })
            : new BSpline2D(controls, knots, 3);
        var vertices = spline.Tessellate(.001);
        Assert.Equal(2, vertices.Count);
        Assert.Equal(new Vec2D(0, 0), vertices[0].Position);
        Assert.Equal(new Vec2D(10, 0), vertices[^1].Position);
        Assert.Equal(new double[] { 2, 2, 2, 2, 5, 5, 5, 5 }, knots);
    }

    [Fact]
    public void RationalArcRespondsToToleranceAndKeepsExactEndpoints()
    {
        const double radius = 1000;
        var spline = new BSpline2D(
            new[] { new Vec2D(radius, 0), new Vec2D(radius, radius), new Vec2D(0, radius) },
            new double[] { 0, 0, 0, 1, 1, 1 }, 2,
            new double[] { 1, Math.Sqrt(.5), 1 });
        var coarse = spline.Tessellate(1.0);
        var fine = spline.Tessellate(.01);
        Assert.True(fine.Count > coarse.Count);
        Assert.Equal(new Vec2D(radius, 0), fine[0].Position);
        Assert.Equal(new Vec2D(0, radius), fine[^1].Position);
        foreach (var (a, b) in fine.Zip(fine.Skip(1)))
        {
            // The midpoint of a circular arc chord is its maximum radial error.
            var midpoint = (a.Position + b.Position) * .5;
            Assert.InRange(radius - midpoint.Length(), 0, .01 + 1e-9);
            Assert.True(b.Uniform > a.Uniform);
            Assert.InRange(Math.Abs(a.Position.Length() - radius), 0, 1e-9);
        }
    }

    [Fact]
    public void ExplicitSampleCountIsStillHonored()
    {
        var spline = new BSpline2D(new[] { new Vec2D(0, 0), new Vec2D(10, 0) },
            new double[] { 0, 0, 1, 1 }, 1);
        Assert.Equal(8, spline.Tessellate(8).Count);
    }
}
