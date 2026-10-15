using Geo;
using GeoCore;
using GeoSolver;

namespace GeoTests;

public class TangentArcJoinTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LineArcEndpointTangencyFollowsAuthoredCoincidences(bool transitive)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .001);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "line_contact");
        sketch.SolveAfterEveryConstraint = false;
        var line = sketch.AddCLine(new Vec2D(0, 12), new Vec2D(10, 12));
        sketch.FixPoint(line.CStart, new Vec2D(0, 12));
        sketch.FixPoint(line.CEnd, new Vec2D(10, 12));
        var arc = sketch.AddCArc(new Vec2D(.2, 12.1), new Vec2D(2, 11), new Vec2D(3, 8));
        sketch.FixPoint(arc.CEnd, new Vec2D(3, 8));
        var contact = line.CStart;
        if (transitive)
        {
            var helper = sketch.AddCLine(new Vec2D(0, 12), new Vec2D(5, 10));
            sketch.FixPoint(helper.CEnd, new Vec2D(5, 10));
            sketch.SetPointOnPoint(helper.CStart, contact);
            contact = helper.CStart;
        }
        sketch.SetPointOnPoint(arc.CStart, contact);
        sketch.SetTangent(line, arc);
        var result = sketch.SolveConstraintsDetailed();
        Assert.True(result.Converged, $"{result.Message}; residual {result.SumOfSquaredErrors:R}");
        Assert.InRange(Math.Abs(arc.CRadius.Evaluate() - 3.125), 0, 1e-6);
        Assert.InRange(Math.Abs(arc.CCenter.Evaluate().X), 0, 1e-6);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void ArcEndpointOnCircleHasAccurateTangency(bool reverse, bool swap)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .001);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "contact");
        sketch.SolveAfterEveryConstraint = false;
        var circle = sketch.AddCCircle(new Vec2D(6, 6), 3);
        sketch.FixPoint(circle.CCenter, new Vec2D(6, 6));
        sketch.SetRadius(circle, 3);
        var arc = sketch.AddCArc(new Vec2D(reverse ? 3 : 0, reverse ? 8 : 12),
            new Vec2D(2, 11), new Vec2D(reverse ? 0 : 3, reverse ? 12 : 8));
        sketch.SetPointOnLine(arc.CCenter, sketch.OriginUnitY);
        sketch.FixPoint(reverse ? arc.CEnd : arc.CStart, new Vec2D(0, 12));
        sketch.SetPointOnCircle(reverse ? arc.CStart : arc.CEnd, circle);
        sketch.SetTangentCircles(swap ? circle : arc, swap ? arc : circle);
        var result = sketch.SolveConstraintsDetailed();
        Assert.True(result.Converged, $"{result.Message}; residual {result.SumOfSquaredErrors:R}");
        Assert.InRange(Math.Abs(arc.CRadius.Evaluate() - 3.5), 0, 1e-6);
        Assert.InRange(Math.Abs(arc.CCenter.Evaluate().X), 0, 1e-6);
        Assert.InRange(Math.Abs(arc.CCenter.Evaluate().Y - 8.5), 0, 1e-6);
    }


    [Theory]
    [InlineData(0, 1, false, false)]
    [InlineData(324, 1, false, false)]
    [InlineData(324, 1, true, false)]
    [InlineData(324, 1, false, true)]
    [InlineData(0, .1, true, true)]
    [InlineData(0, 10, false, true)]
    public void TangentShoulderKeepsDimensionedEndpoints(
        double height, double scale, bool reverse, bool tangentFirst)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-400), new Vec3D(400)), .01);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "tyre_section");
        sketch.SolveAfterEveryConstraint = false;
        Vec2D Point(double x, double y) => new Vec2D(x * scale, height + y * scale);
        var crown = sketch.AddCArc(Point(reverse ? -13 : 13, 0),
            Point(0, 13), Point(reverse ? 13 : -13, 0));
        var shoulder = sketch.AddCArc(reverse ? Point(13, 0) : Point(8.8, -9.9),
            Point(12, -6), reverse ? Point(8.8, -9.9) : Point(13, 0));
        var crownJoin = reverse ? crown.CEnd : crown.CStart;
        var shoulderJoin = reverse ? shoulder.CStart : shoulder.CEnd;
        var bead = reverse ? shoulder.CEnd : shoulder.CStart;
        sketch.FixPoint(crown.CCenter, Point(0, 0));
        sketch.SetRadius(crown, 13 * scale);
        sketch.FixPoint(crown.CStart, Point(reverse ? -13 : 13, 0));
        sketch.FixPoint(crown.CEnd, Point(reverse ? 13 : -13, 0));
        sketch.FixPoint(bead, Point(8.8, -9.9));
        if (tangentFirst)
            sketch.SetTangentCircles(crown, shoulder);
        sketch.SetPointOnPoint(shoulderJoin, crownJoin);
        if (!tangentFirst)
            sketch.SetTangentCircles(crown, shoulder);

        var result = sketch.SolveConstraintsDetailed();
        Assert.True(result.Converged, $"{result.Message}; residual {result.SumOfSquaredErrors:R}");
        Assert.InRange(Math.Abs(bead.Evaluate().X - 8.8 * scale), 0, 1e-6);
        Assert.InRange(Math.Abs(shoulderJoin.Evaluate().X - 13 * scale), 0, 1e-6);
        Assert.InRange(Math.Abs(shoulderJoin.Evaluate().Y - height), 0, 1e-6);
        // At the crown's rightmost point, tangency requires horizontal radii.
        Assert.InRange(Math.Abs(shoulder.CCenter.Evaluate().Y - height), 0, 1e-5);
        // Independent circle-through-bead solution, with a vertical tangent at the join.
        double expectedRadius = (4.2 * 4.2 + 9.9 * 9.9) / (2 * 4.2) * scale;
        Assert.InRange(Math.Abs(shoulder.CRadius.Evaluate() - expectedRadius), 0, 1e-5);
    }
}
