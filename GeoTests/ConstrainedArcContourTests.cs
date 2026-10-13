using Curves;
using Geo;
using GeoCore;
using GeoSolver;

namespace GeoTests;

public class ConstrainedArcContourTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(0, 180)]
    [InlineData(90, -90)]
    [InlineData(170, 440)]
    [InlineData(-170, -440)]
    public void InteriorArcPointsMatchTheDirectedCircleAndHaveFiniteDerivatives(double start, double end)
    {
        var arc = new CArc2D(new Vec2D(3, 4), 7, start*Math.PI/180, end*Math.PI/180);
        foreach (double u in new[] { .25, .5, .75 })
        {
            var point = arc.Evaluate(u);
            var actual = point.Evaluate();
            var expected = arc.EvaluateVertex(u).Position;
            Assert.InRange((actual-expected).Length(), 0, 1e-10);
            foreach (var expr in new[] { point.Ex, point.Ey })
            {
                Assert.Equal(expr.Evaluate(), expr.Compile()(), 10);
                var parameter = arc.CStartDir.Ex.Value;
                var derivative = expr.PartialDerivative(parameter);
                double initial = parameter.Value, h = 1e-6;
                parameter.Value = initial+h;
                double plus = expr.Evaluate();
                parameter.Value = initial-h;
                double minus = expr.Evaluate();
                parameter.Value = initial;
                Assert.InRange(Math.Abs(derivative.Compile()()-(plus-minus)/(2*h)), 0, 1e-5);
                Assert.Equal(derivative.Evaluate(), expr.EvaluatePartialDerivative(parameter), 9);
            }
        }
    }

    [Fact]
    public void CoincidentArcDirectionsDoNotProduceAnArtificialSemicircle()
    {
        var arc = new CArc2D(new Vec2D(3, 4), 7, 0, 0);
        foreach (double u in new[] { .25, .5, .75 })
            Assert.InRange((arc.Evaluate(u).Evaluate()-arc.EvaluateVertex(u).Position).Length(), 0, 1e-10);
    }

    [Fact]
    public void SnappedSemicircleMidpointRemainsValidWhenTangencyIsAdded()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50), new Vec3D(50)), .01);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "snapped_slot");
        sketch.SolveAfterEveryConstraint = false;
        var axis = sketch.AddCLine(new Vec2D(18, 0), new Vec2D(30, 0), CurveFlags.HelperGeometry);
        var side = sketch.AddCLine(new Vec2D(4, -2), new Vec2D(16, -2));
        var arc = sketch.AddCArc(new Vec2D(16, -2), new Vec2D(18, 0), new Vec2D(16, 2));
        sketch.SetCoincident(side.CEnd, arc.CStart);
        sketch.SetCoincident(axis.CStart, arc.Evaluate(.5));
        sketch.SetTangent(side, arc);
        var result = sketch.SolveConstraintsDetailed();
        Assert.True(result.Converged, result.Message);
        Assert.InRange((arc.Evaluate(.5).Evaluate()-new Vec2D(18, 0)).Length(), 0, 1e-6);
    }


    [Theory]
    [InlineData(1)]
    [InlineData(10)]
    public void TangentCrownRemainsClosedAfterGeometryUpdate(double scale)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-4000), new Vec3D(4000)), .01);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "crown");
        sketch.SolveAfterEveryConstraint = false;
        Vec2D P(double x, double y) => new Vec2D(x * scale, y * scale);
        var left = sketch.AddCArc(P(-10.6, 314.1), P(-13, 319), P(-14, 325));
        var crown = sketch.AddCArc(P(-14, 325), P(0, 339), P(14, 325));
        var right = sketch.AddCArc(P(14, 325), P(13, 319), P(10.6, 314.1));
        var baseLine = sketch.AddCLine(P(10.6, 314.1), P(-10.6, 314.1));
        var centerLine = sketch.AddCLine(P(-20, 325), P(20, 325), CurveFlags.HelperGeometry);
        sketch.FixPoint(baseLine.CStart, P(10.6, 314.1));
        sketch.FixPoint(baseLine.CEnd, P(-10.6, 314.1));
        sketch.FixPoint(centerLine.CStart, P(-20, 325));
        sketch.FixPoint(centerLine.CEnd, P(20, 325));
        sketch.SetRadius(crown, 14 * scale);
        sketch.FixPoint(crown.CCenter, P(0, 325));
        sketch.SetPointOnLine(crown.CStart, centerLine);
        sketch.SetPointOnLine(crown.CEnd, centerLine);
        sketch.SetPointOnPoint(left.CEnd, crown.CStart);
        sketch.SetPointOnPoint(right.CStart, crown.CEnd);
        sketch.FixPoint(left.CStart, P(-10.6, 314.1));
        sketch.FixPoint(right.CEnd, P(10.6, 314.1));
        sketch.SetTangentCircles(crown, left);
        sketch.SetTangentCircles(crown, right);
        var result = sketch.SolveConstraintsDetailed();
        Assert.True(result.Converged, $"{result.Message}; SSE={result.SumOfSquaredErrors:R}");
        var solid = api.Extrude(sketch, 1, name: "closed_crown");
        MeshTestHelpers.AssertValidMesh(solid.Mesh.Positions, solid.Mesh.Triangles);
    }
}
