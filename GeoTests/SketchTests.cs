using Curves;
using Geo;
using GeoCore;
using GeoMeta;
using GeoSolver;
using GeoSolver.Sketcher;

namespace GeoTests;

/// <summary>
/// Each test covers one distinct constraint type available on ConstrainedSketcher.
/// After SolveConstraints() the test:
///   1. asserts error &lt; tolerance (solver quality),
///   2. directly inspects the geometric objects to verify the constraint is satisfied.
/// </summary>
public class SketchTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private const double SolverTol = 1e-6;
    private const double GeomTol   = 1e-4;


    // ── helpers ─────────────────────────────────────────────────────────────

    private static ConstrainedSketcher MakeSketch(string name = "sk")
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), 0.01);
        var s   = api.GetConstraintSketcher(DefaultPlanes.OriginXY, name);
        s.SolveAfterEveryConstraint = false;
        return s;
    }

    private static void AssertSolved(double error)
        => Assert.True(error < SolverTol, $"Solver error {error:e3} exceeds tolerance {SolverTol:e3}");

    private static void AssertNear(double expected, double actual, string label = "")
        => Assert.True(Math.Abs(actual - expected) < GeomTol,
            $"{label}: expected {expected}, got {actual} (delta {Math.Abs(actual - expected):e3})");

    private static double LineLength(CLine2D l)
    {
        var s = l.CStart.Evaluate(); var e = l.CEnd.Evaluate();
        return Math.Sqrt((e.X-s.X)*(e.X-s.X)+(e.Y-s.Y)*(e.Y-s.Y));
    }

    private static double Dot2(Vec2D a, Vec2D b) => a.X*b.X + a.Y*b.Y;

    private static Vec2D LineDir(CLine2D l)
    {
        var s = l.CStart.Evaluate(); var e = l.CEnd.Evaluate();
        var d = new Vec2D(e.X-s.X, e.Y-s.Y);
        double len = Math.Sqrt(d.X*d.X+d.Y*d.Y);
        return new Vec2D(d.X/len, d.Y/len);
    }

    private static Vec2D LineMid(CLine2D l)
    {
        var s = l.CStart.Evaluate();
        var e = l.CEnd.Evaluate();
        return new Vec2D((s.X + e.X) * 0.5, (s.Y + e.Y) * 0.5);
    }

    private static void AssertMidsCoincident(CLine2D a, CLine2D b, string label)
    {
        var ma = LineMid(a);
        var mb = LineMid(b);
        AssertNear(ma.X, mb.X, label + ".X");
        AssertNear(ma.Y, mb.Y, label + ".Y");
    }

    // ════════════════════════════════════════════════════════════════════════
    // FixPoint
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_FixPoint_PinsPrecisely()
    {
        var s   = MakeSketch();
        var l   = s.AddCLine(new Vec2D(3, 5), new Vec2D(6, 5));
        s.FixPoint(l.CStart, new Vec2D(1, 2));

        AssertSolved(s.SolveConstraints());

        var p = l.CStart.Evaluate();
        AssertNear(1, p.X, "FixPoint.X");
        AssertNear(2, p.Y, "FixPoint.Y");
    }

    // ════════════════════════════════════════════════════════════════════════
    // PointOnPoint / Coincident
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_PointOnPoint_MergesTwoPoints()
    {
        var s  = MakeSketch();
        var l1 = s.AddCLine(new Vec2D(0, 0), new Vec2D(3, 0));
        var l2 = s.AddCLine(new Vec2D(5, 0), new Vec2D(8, 0));   // gap between lines
        s.FixPoint(l1.CStart, new Vec2D(0, 0));
        s.SetPointOnPoint(l1.CEnd, l2.CStart);

        AssertSolved(s.SolveConstraints());

        var e1 = l1.CEnd.Evaluate();
        var s2 = l2.CStart.Evaluate();
        AssertNear(e1.X, s2.X, "PointOnPoint.X");
        AssertNear(e1.Y, s2.Y, "PointOnPoint.Y");
    }

    [Fact]
    public void Constraint_Coincident_LineEnd_CircleCardinalEast()
    {
        var s = MakeSketch();
        var line = s.AddCLine(new Vec2D(0, 0), new Vec2D(1, 1));
        var circle = s.AddCCircle(new Vec2D(0, 0), 2.0);
        s.FixPoint(line.CStart, new Vec2D(0, 0));
        s.FixPoint(circle.CCenter, new Vec2D(0, 0));
        s.SetRadius(circle, 2.0);
        s.SetCoincident(line.CEnd, circle.Evaluate(0.0));

        AssertSolved(s.SolveConstraints());

        var end = line.CEnd.Evaluate();
        AssertNear(2.0, end.X, "CardinalEast.X");
        AssertNear(0.0, end.Y, "CardinalEast.Y");
    }

    [Fact]
    public void Constraint_Coincident_LineEnd_CircleCardinalEast_WithRadius_DoesNotCollapse()
    {
        var s = MakeSketch();
        var line = s.AddCLine(new Vec2D(0, 0), new Vec2D(1, 1));
        var circle = s.AddCCircle(new Vec2D(5, 5), 2.0);
        s.SetRadius(circle, 2.0);
        s.SetCoincident(line.CEnd, circle.Evaluate(0.0));

        AssertSolved(s.SolveConstraints());

        var end = line.CEnd.Evaluate();
        var east = circle.Evaluate(0.0).Evaluate();
        AssertNear(east.X, end.X, "LockedRadiusEast.X");
        AssertNear(east.Y, end.Y, "LockedRadiusEast.Y");
        Assert.True(circle.CRadius.Evaluate() > 0.1, "Radius collapsed toward zero");
    }

    [Fact]
    public void Constraint_Coincident_CircleCardinal_WithoutRadius_LetsRadiusCollapse()
    {
        var s = MakeSketch();
        var line = s.AddCLine(new Vec2D(0, 0), new Vec2D(1, 1));
        var circle = s.AddCCircle(new Vec2D(5, 5), 2.0);
        s.SetCoincident(line.CEnd, circle.Evaluate(0.0));

        AssertSolved(s.SolveConstraints());

        Assert.True(circle.CRadius.Evaluate() < 0.01, "Expected degenerate radius collapse without radius lock");
    }

    [Fact]
    public void Constraint_Coincident_CircleCardinal_Incremental_FixedCenter_WithoutRadius()
    {
        var s = MakeSketch();
        s.SolveAfterEveryConstraint = true;
        var line = s.AddCLine(new Vec2D(0, 0), new Vec2D(1, 1));
        var circle = s.AddCCircle(new Vec2D(5, 5), 2.0);
        s.FixPoint(circle.CCenter, new Vec2D(5, 5));
        s.SetCoincident(line.CEnd, circle.Evaluate(0.5));

        var end = line.CEnd.Evaluate();
        var west = circle.Evaluate(0.5).Evaluate();
        AssertNear(west.X, end.X, "IncrementalWest.X");
        AssertNear(west.Y, end.Y, "IncrementalWest.Y");
        Assert.True(
            Math.Abs(end.X - 5.0) + Math.Abs(end.Y - 5.0) > 0.5,
            "Line end snapped to the circle center instead of @0.5.");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Horizontal
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Horizontal_MakesEndpointsShareY()
    {
        var s = MakeSketch();
        var l = s.AddCLine(new Vec2D(0, 0), new Vec2D(4, 1));   // initially slanted
        s.FixPoint(l.CStart, new Vec2D(0, 0));
        s.SetHorizontal(l);

        AssertSolved(s.SolveConstraints());

        AssertNear(l.CStart.Evaluate().Y, l.CEnd.Evaluate().Y, "Horizontal deltaY");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Vertical
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Vertical_MakesEndpointsShareX()
    {
        var s = MakeSketch();
        var l = s.AddCLine(new Vec2D(0, 0), new Vec2D(2, 5));   // slanted
        s.FixPoint(l.CStart, new Vec2D(0, 0));
        s.SetVertical(l);

        AssertSolved(s.SolveConstraints());

        AssertNear(l.CStart.Evaluate().X, l.CEnd.Evaluate().X, "Vertical deltaX");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Length (via SetDistancePointPoint)
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Length_SetsExactLineLength()
    {
        var s = MakeSketch();
        var l = s.AddCLine(new Vec2D(0, 0), new Vec2D(1, 0));
        s.FixPoint(l.CStart, new Vec2D(0, 0));
        s.SetHorizontal(l);
        s.SetLength(l, 7.0);

        AssertSolved(s.SolveConstraints());

        AssertNear(7.0, LineLength(l), "LineLength");
    }

    // ════════════════════════════════════════════════════════════════════════
    // HorizontalDistancePointPoint
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_HorizontalDistance_SetsExactDeltaX()
    {
        var s  = MakeSketch();
        var l  = s.AddCLine(new Vec2D(0, 0), new Vec2D(1, 3));
        s.FixPoint(l.CStart, new Vec2D(0, 0));
        s.SetHorizontalDistancePointPoint(l.CStart, l.CEnd, 5.0);

        AssertSolved(s.SolveConstraints());

        double dx = l.CEnd.Evaluate().X - l.CStart.Evaluate().X;
        AssertNear(5.0, dx, "HorizontalDistance dx");
    }

    // ════════════════════════════════════════════════════════════════════════
    // VerticalDistancePointPoint
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_VerticalDistance_SetsExactDeltaY()
    {
        var s = MakeSketch();
        var l = s.AddCLine(new Vec2D(0, 0), new Vec2D(3, 1));
        s.FixPoint(l.CStart, new Vec2D(0, 0));
        s.SetVerticalDistancePointPoint(l.CStart, l.CEnd, 6.0);

        AssertSolved(s.SolveConstraints());

        double dy = l.CEnd.Evaluate().Y - l.CStart.Evaluate().Y;
        AssertNear(6.0, dy, "VerticalDistance dy");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Perpendicular
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Perpendicular_MakesDotProductZero()
    {
        var s  = MakeSketch();
        var l1 = s.AddCLine(new Vec2D(0, 0), new Vec2D(4, 0));
        var l2 = s.AddCLine(new Vec2D(0, 0), new Vec2D(1, 1));  // not yet perpendicular
        s.FixPoint(l1.CStart, new Vec2D(0, 0));
        s.FixPoint(l2.CStart, new Vec2D(0, 0));
        s.SetPerpendicular(l1, l2);

        AssertSolved(s.SolveConstraints());

        AssertNear(0, Dot2(LineDir(l1), LineDir(l2)), "Perpendicular dot");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Parallel
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Parallel_MakesCrossProductZero()
    {
        var s  = MakeSketch();
        var l1 = s.AddCLine(new Vec2D(0, 0), new Vec2D(4, 0));
        var l2 = s.AddCLine(new Vec2D(0, 2), new Vec2D(4, 3));  // not yet parallel
        s.FixPoint(l1.CStart, new Vec2D(0, 0));
        s.FixPoint(l2.CStart, new Vec2D(0, 2));
        s.SetParallel(l1, l2);

        AssertSolved(s.SolveConstraints());

        // cross product of unit directions should be ~0 for parallel lines
        var d1 = LineDir(l1); var d2 = LineDir(l2);
        double cross = d1.X*d2.Y - d1.Y*d2.X;
        AssertNear(0, cross, "Parallel cross");
    }

    // ════════════════════════════════════════════════════════════════════════
    // EqualLength
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_EqualLength_BothLinesHaveSameLength()
    {
        var s  = MakeSketch();
        var l1 = s.AddCLine(new Vec2D(0, 0), new Vec2D(3, 0));
        var l2 = s.AddCLine(new Vec2D(0, 2), new Vec2D(7, 2));
        s.FixPoint(l1.CStart, new Vec2D(0, 0));
        s.FixPoint(l2.CStart, new Vec2D(0, 2));
        s.SetHorizontal(l1);
        s.SetHorizontal(l2);
        s.SetEqualLength(l1, l2);

        AssertSolved(s.SolveConstraints());

        AssertNear(LineLength(l1), LineLength(l2), "EqualLength");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Midpoint
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Midpoint_PointIsAtLineMidpoint()
    {
        var s    = MakeSketch();
        var l    = s.AddCLine(new Vec2D(0, 0), new Vec2D(6, 0));
        // Use the start of a helper segment as the "mid" point so the solver can move it
        var mark = s.AddCLine(new Vec2D(1, 0), new Vec2D(1, 0));
        s.FixPoint(l.CStart, new Vec2D(0, 0));
        s.FixPoint(l.CEnd,   new Vec2D(6, 0));
        s.SetMidpoint(mark.CStart, l);

        AssertSolved(s.SolveConstraints());

        var p = mark.CStart.Evaluate();
        AssertNear(3.0, p.X, "Midpoint.X");
        AssertNear(0.0, p.Y, "Midpoint.Y");
    }

    [Fact]
    public void Constraint_Coincident_TwoLineMids_FixedTarget_MovesTheOtherMid()
    {
        var s = MakeSketch();
        var moving = s.AddCLine(new Vec2D(0, 0), new Vec2D(2, 0));
        var target = s.AddCLine(new Vec2D(10, 4), new Vec2D(10, 8));
        s.FixPoint(target.CStart, new Vec2D(10, 4));
        s.FixPoint(target.CEnd, new Vec2D(10, 8));
        s.SetCoincident(moving.Evaluate(0.5), target.Evaluate(0.5));

        AssertSolved(s.SolveConstraints());

        AssertMidsCoincident(moving, target, "TwoMids");
        var mid = LineMid(moving);
        AssertNear(10.0, mid.X, "MovedMid.X");
        AssertNear(6.0, mid.Y, "MovedMid.Y");
        Assert.True(LineLength(moving) > 1.0, "Moving line collapsed");
        AssertNear(4.0, LineLength(target), "Target length");
    }

    [Fact]
    public void Constraint_Coincident_TwoLineMids_EachHasOneFixedEnd()
    {
        var s = MakeSketch();
        var a = s.AddCLine(new Vec2D(0, 0), new Vec2D(4, 0));
        var b = s.AddCLine(new Vec2D(8, 6), new Vec2D(12, 6));
        s.FixPoint(a.CStart, new Vec2D(0, 0));
        s.FixPoint(b.CStart, new Vec2D(8, 6));
        s.SetCoincident(a.Evaluate(0.5), b.Evaluate(0.5));

        AssertSolved(s.SolveConstraints());

        AssertMidsCoincident(a, b, "PinnedStartMids");
        Assert.True(LineLength(a) > 1.0, "A collapsed");
        Assert.True(LineLength(b) > 1.0, "B collapsed");
        AssertNear(0.0, a.CStart.Evaluate().X, "A start stayed");
        AssertNear(0.0, a.CStart.Evaluate().Y, "A start stayed");
        AssertNear(8.0, b.CStart.Evaluate().X, "B start stayed");
        AssertNear(6.0, b.CStart.Evaluate().Y, "B start stayed");
    }

    [Fact]
    public void Constraint_Coincident_ThreeLineMids_ShareOnePoint()
    {
        var s = MakeSketch();
        var a = s.AddCLine(new Vec2D(-4, 0), new Vec2D(-2, 0));
        var b = s.AddCLine(new Vec2D(4, 0), new Vec2D(6, 0));
        var c = s.AddCLine(new Vec2D(0, 4), new Vec2D(0, 6));
        s.FixPoint(a.CStart, new Vec2D(-4, 0));
        s.FixPoint(a.CEnd, new Vec2D(-2, 0));
        s.SetCoincident(a.Evaluate(0.5), b.Evaluate(0.5));
        s.SetCoincident(b.Evaluate(0.5), c.Evaluate(0.5));

        AssertSolved(s.SolveConstraints());

        AssertMidsCoincident(a, b, "AB");
        AssertMidsCoincident(b, c, "BC");
        var mid = LineMid(a);
        AssertNear(-3.0, mid.X, "SharedMid.X");
        AssertNear(0.0, mid.Y, "SharedMid.Y");
        Assert.True(LineLength(b) > 1.0, "B collapsed");
        Assert.True(LineLength(c) > 1.0, "C collapsed");
    }

    [Fact]
    public void Constraint_Coincident_TwoLineMids_IncrementalSolve()
    {
        var s = MakeSketch();
        s.SolveAfterEveryConstraint = true;
        var moving = s.AddCLine(new Vec2D(0, 0), new Vec2D(2, 2));
        var target = s.AddCLine(new Vec2D(8, 0), new Vec2D(12, 0));
        s.FixPoint(target.CStart, new Vec2D(8, 0));
        s.FixPoint(target.CEnd, new Vec2D(12, 0));
        s.SetCoincident(moving.Evaluate(0.5), target.Evaluate(0.5));

        AssertMidsCoincident(moving, target, "IncrementalMids");
        var mid = LineMid(moving);
        AssertNear(10.0, mid.X, "IncrementalMid.X");
        AssertNear(0.0, mid.Y, "IncrementalMid.Y");
        Assert.True(LineLength(moving) > 1.0, "Moving line collapsed");
    }

    // ════════════════════════════════════════════════════════════════════════
    // PointOnLine
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_PointOnLine_PointLiesOnInfiniteLine()
    {
        var s = MakeSketch();
        var l = s.AddCLine(new Vec2D(0, 0), new Vec2D(10, 0));
        // Use the start of a helper line as the "floating" point
        var ph = s.AddCLine(new Vec2D(5, 3), new Vec2D(5, 3));
        s.FixPoint(l.CStart, new Vec2D(0, 0));
        s.FixPoint(l.CEnd,   new Vec2D(10, 0));
        s.SetPointOnLine(ph.CStart, l);

        AssertSolved(s.SolveConstraints());

        // The line is horizontal at y=0; point y must become 0
        AssertNear(0, ph.CStart.Evaluate().Y, "PointOnLine.Y");
    }

    // ════════════════════════════════════════════════════════════════════════
    // DistancePointLine
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_DistancePointLine_PointAtSpecifiedDistance()
    {
        var s  = MakeSketch();
        var l  = s.AddCLine(new Vec2D(0, 0), new Vec2D(10, 0));
        var ph = s.AddCLine(new Vec2D(5, 1), new Vec2D(5, 1));
        s.FixPoint(l.CStart, new Vec2D(0, 0));
        s.FixPoint(l.CEnd,   new Vec2D(10, 0));
        // fix horizontal position of the point, leave vertical free for the distance constraint
        s.FixPoint(ph.CEnd, new Vec2D(5, 1));   // lock the degenerate end so solver has DOF on start
        s.SetDistancePointLine(ph.CStart, l, 3.0);

        AssertSolved(s.SolveConstraints());

        AssertNear(3.0, Math.Abs(ph.CStart.Evaluate().Y), "DistancePointLine dist");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Radius
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Radius_SetsCircleRadius()
    {
        var s = MakeSketch();
        var c = s.AddCCircle(new Vec2D(0, 0), 1.0);
        s.FixPoint(c.CCenter, new Vec2D(0, 0));
        s.SetRadius(c, 5.0);

        AssertSolved(s.SolveConstraints());

        AssertNear(5.0, c.Radius, "CircleRadius");
    }

    // ════════════════════════════════════════════════════════════════════════
    // EqualRadius
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_EqualRadius_BothCirclesHaveSameRadius()
    {
        var s  = MakeSketch();
        var c1 = s.AddCCircle(new Vec2D(0, 0), 2.0);
        var c2 = s.AddCCircle(new Vec2D(10, 0), 5.0);
        s.FixPoint(c1.CCenter, new Vec2D(0, 0));
        s.FixPoint(c2.CCenter, new Vec2D(10, 0));
        s.SetRadius(c1, 3.0);
        s.SetEqualRadius(c1, c2);

        AssertSolved(s.SolveConstraints());

        AssertNear(c1.Radius, c2.Radius, "EqualRadius");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Concentric
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Concentric_CirclesCentersSamePoint()
    {
        var s  = MakeSketch();
        var c1 = s.AddCCircle(new Vec2D(0, 0), 2.0);
        var c2 = s.AddCCircle(new Vec2D(3, 4), 5.0);
        s.FixPoint(c1.CCenter, new Vec2D(1, 1));
        s.SetConcentric(c1, c2);

        AssertSolved(s.SolveConstraints());

        var o1 = c1.CCenter.Evaluate();
        var o2 = c2.CCenter.Evaluate();
        AssertNear(o1.X, o2.X, "Concentric.X");
        AssertNear(o1.Y, o2.Y, "Concentric.Y");
    }

    // ════════════════════════════════════════════════════════════════════════
    // PointOnCircle
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_PointOnCircle_PointDistanceEqualsRadius()
    {
        var s  = MakeSketch();
        var c  = s.AddCCircle(new Vec2D(0, 0), 4.0);
        var ph = s.AddCLine(new Vec2D(1, 1), new Vec2D(1, 1));
        s.FixPoint(c.CCenter, new Vec2D(0, 0));
        s.SetRadius(c, 4.0);
        s.SetPointOnCircle(ph.CStart, c);

        AssertSolved(s.SolveConstraints());

        var pt  = ph.CStart.Evaluate();
        var cen = c.CCenter.Evaluate();
        double dist = Math.Sqrt((pt.X-cen.X)*(pt.X-cen.X)+(pt.Y-cen.Y)*(pt.Y-cen.Y));
        AssertNear(c.Radius, dist, "PointOnCircle dist");
    }

    // ════════════════════════════════════════════════════════════════════════
    // TangentLineCircle
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_TangentLineCircle_DistanceCenterToLineEqualsRadius()
    {
        var s = MakeSketch();
        // Horizontal line; circle centred at origin with r=3.
        // Tangency must move the line's y to ±3.
        var l = s.AddCLine(new Vec2D(-5, 5), new Vec2D(5, 5));
        var c = s.AddCCircle(new Vec2D(0, 0), 3.0);
        s.FixPoint(c.CCenter, new Vec2D(0, 0));
        s.SetRadius(c, 3.0);
        // Keep line horizontal; fix only x of start so y remains free for tangency
        s.SetHorizontal(l);
        s.SetHorizontalDistancePointPoint(c.CCenter, l.CStart, 5.0);
        s.SetTangent(l, c);

        AssertSolved(s.SolveConstraints());

        // distance from centre to the (horizontal) line equals radius
        var lineY   = l.CStart.Evaluate().Y;
        var centreY = c.CCenter.Evaluate().Y;
        AssertNear(c.Radius, Math.Abs(lineY - centreY), "TangentLine dist");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Symmetric
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Symmetric_PointsAreMirroredAboutLine()
    {
        // Use ConstraintSolver directly so we have full control over which params are free.
        // Axis: vertical line at x=0, start=(0,-10), end=(0,10) — all constants (frozen via Constant).
        var axisStart = CVec2D.Constant(0, -10);
        var axisEnd   = CVec2D.Constant(0, 10);
        var axis      = new CLine2D(new Vec2D(0, -10), new Vec2D(0, 10));
        // Two free points to be symmetrized
        var p1 = new CVec2D(-3, 4);   // will remain at (-3, 4) as it drives the axis
        var p2 = new CVec2D( 7, 4);   // solver must move to (3, 4)

        // Build a CLine2D whose CStart/CEnd are the exact CVec2D objects we want
        // by using the axis CLine2D with constants — but SymmetricAboutLine2d takes CLine2D.
        // Create a thin wrapper CLine2D with the constant endpoints replaced:
        var fakeAxis = new CLine2D(new Vec2D(0, -10), new Vec2D(0, 10));
        // Freeze all fakeAxis params so solver won't move them
        fakeAxis.CStart.Freeze();
        fakeAxis.CEnd.Freeze();
        // Freeze p1 so only p2 is moved by the solver
        p1.Freeze();

        var constraints = new List<GeoSolver.IBaseEquation>
        {
            new SymmetricAboutLine2d(p1, p2, fakeAxis)
        };

        double error = GeoSolver.ConstraintSolver.Solve(constraints);
        AssertSolved(error);

        var a = p1.Evaluate(); var b = p2.Evaluate();
        AssertNear(0, a.X + b.X, "Symmetric x-sum (mirror across x=0)");
        AssertNear(a.Y, b.Y,     "Symmetric y-equal");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Angle between two lines
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Angle_SetsExactAngleBetweenLines()
    {
        var s  = MakeSketch();
        var l1 = s.AddCLine(new Vec2D(0, 0), new Vec2D(4, 0));
        var l2 = s.AddCLine(new Vec2D(0, 0), new Vec2D(2, 2));
        s.FixPoint(l1.CStart, new Vec2D(0, 0));
        s.FixPoint(l2.CStart, new Vec2D(0, 0));
        s.SetHorizontal(l1);
        const double targetDeg = 60.0;
        s.SetAngleDegrees(l1, l2, targetDeg);

        AssertSolved(s.SolveConstraints());

        var d1 = LineDir(l1); var d2 = LineDir(l2);
        double cosAngle = Dot2(d1, d2);
        double angleDeg = Math.Acos(Math.Clamp(cosAngle, -1, 1)) * 180.0 / Math.PI;
        AssertNear(targetDeg, angleDeg, "AngleDegrees");
    }

    // ════════════════════════════════════════════════════════════════════════
    // Colinear
    // ════════════════════════════════════════════════════════════════════════
    [Fact]
    public void Constraint_Colinear_SecondLineSharesSupportingLine()
    {
        var s  = MakeSketch();
        var l1 = s.AddCLine(new Vec2D(0, 0), new Vec2D(4, 0));
        var l2 = s.AddCLine(new Vec2D(6, 1), new Vec2D(10, 1));  // shifted up by 1
        s.FixPoint(l1.CStart, new Vec2D(0, 0));
        s.FixPoint(l1.CEnd,   new Vec2D(4, 0));
        s.SetColinear(l1, l2);

        AssertSolved(s.SolveConstraints());

        // All four endpoints should share the same y
        var pts = new[] { l1.CStart.Evaluate(), l1.CEnd.Evaluate(),
                          l2.CStart.Evaluate(), l2.CEnd.Evaluate() };
        double y0 = pts[0].Y;
        foreach (var p in pts.Skip(1))
            AssertNear(y0, p.Y, "Colinear.Y");
    }

    [Fact]
    public void ConstraintCurves_ReceiveUniquePickNames()
    {
        var s = MakeSketch("FaceSketch");
        var line = s.AddCLine(new Vec2D(0, 0), new Vec2D(2, 0));
        var circle = s.AddCCircle(new Vec2D(1, 1), 0.5);
        Assert.Equal("Line1", line.Name);
        Assert.Equal("Circle1", circle.Name);
        Assert.Equal("FaceSketch:Line1@0.000",
            EntityNaming.Qualify(s.Name, EntityNaming.FormatSketchHandleAddress(line.Name, "line", 0)));
        Assert.Equal("FaceSketch:Circle1@center",
            EntityNaming.Qualify(s.Name, EntityNaming.FormatSketchHandleAddress(circle.Name, "circle", 2)));
    }

    // ════════════════════════════════════════════════════════════════════════
    // Named 3D model points as constant sketch references
    // ════════════════════════════════════════════════════════════════════════

    static void AssertConstantPoint(CVec2D point, double x, double y, string label)
    {
        Assert.True(ConstrainedSketcher.IsConstantPoint(point), label + " should be a constant");
        var uv = point.Evaluate();
        AssertNear(x, uv.X, label + ".X");
        AssertNear(y, uv.Y, label + ".Y");
    }

    static GeoAPI MakeApi() => new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), 0.01);

    static AnchorMesh ExtrudeUnitBox(GeoAPI api, string name)
    {
        var profile = api.GetPlotterSketcher(DefaultPlanes.OriginXY, name + "Profile");
        profile.AddLine(new Vec2D(0, 0), new Vec2D(4, 0));
        profile.AddLine(new Vec2D(4, 0), new Vec2D(4, 4));
        profile.AddLine(new Vec2D(4, 4), new Vec2D(0, 4));
        profile.AddLine(new Vec2D(0, 4), new Vec2D(0, 0));
        return api.Extrude(profile, 3.0, name: name);
    }

    [Fact]
    public void NamedModelPoint_OffPlane_ProjectsToConstantOnSketch()
    {
        var api = MakeApi();
        ExtrudeUnitBox(api, "box");
        string side = EntityNaming.ExtrudeSide("box", "Line1");
        string top = EntityNaming.ExtrudeTop("box");
        string name = EntityNaming.FormatEdgePointAddress(side, top, 0.5);

        Assert.True(api.TryGetPointFromName(name, out Vec3D world));
        Assert.True(Math.Abs(world.Z) > 1.0, "Fixture must use an off-plane top-edge point");

        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "OnXy");
        Vec2D expected = sketch.ProjectWorldPoint(world);
        CVec2D resolved = api.GetSketchConstraintPoint(sketch, name);

        AssertConstantPoint(resolved, expected.X, expected.Y, "Projected top-edge");
        Assert.True(sketch.TryGetConstraintPoint(name, out CVec2D same));
        AssertConstantPoint(same, expected.X, expected.Y, "Sketch.TryGetConstraintPoint");

        string qualified = "box:" + name;
        Assert.True(api.TryGetPointFromName(qualified, out Vec3D worldQ));
        AssertConstantPoint(
            sketch.GetConstraintPoint(qualified),
            sketch.ProjectWorldPoint(worldQ).X,
            sketch.ProjectWorldPoint(worldQ).Y,
            "Qualified mesh name");
    }

    [Fact]
    public void NamedSurfacePoint_And_SketchHandle_BothAcceptedByCoincident()
    {
        var api = MakeApi();
        ExtrudeUnitBox(api, "box");
        string top = EntityNaming.ExtrudeTop("box");
        string surface = EntityNaming.FormatSurfacePointAddress(top, 0.5, 0.5);
        Assert.True(api.TryGetPointFromName(surface, out Vec3D world));

        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "FaceSketch");
        var line = sketch.AddCLine(new Vec2D(0, 0), new Vec2D(1, 1));
        sketch.FixPoint(line.CStart, new Vec2D(0, 0));
        Vec2D expected = sketch.ProjectWorldPoint(world);

        sketch.SetCoincident(line.CEnd, surface);
        AssertSolved(sketch.SolveConstraints());

        var end = line.CEnd.Evaluate();
        AssertNear(expected.X, end.X, "LineEnd.X");
        AssertNear(expected.Y, end.Y, "LineEnd.Y");
        Assert.False(ConstrainedSketcher.IsConstantPoint(line.CEnd), "Sketch end stays a free parameter");
        Assert.True(ConstrainedSketcher.IsConstantPoint(sketch.GetConstraintPoint(surface)));
    }

    [Fact]
    public void NamedModelPoint_OnYzSketch_ProjectsXyWorldPoint()
    {
        var api = MakeApi();
        ExtrudeUnitBox(api, "box");
        string side = EntityNaming.ExtrudeSide("box", "Line1");
        string bottom = EntityNaming.ExtrudeBottom("box");
        string name = EntityNaming.FormatEdgePointAddress(side, bottom, 0.0);
        Assert.True(api.TryGetPointFromName(name, out Vec3D world));

        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginYZ, "OnYz");
        Vec2D expected = sketch.ProjectWorldPoint(world);
        AssertConstantPoint(sketch.GetConstraintPoint(name), expected.X, expected.Y, "YZ projection");

        var line = sketch.AddCLine(new Vec2D(8, 8), new Vec2D(9, 9));
        sketch.FixPoint(line.CStart);
        sketch.SetCoincident("Line1@1.000", name);
        AssertSolved(sketch.SolveConstraints());
        var end = line.CEnd.Evaluate();
        AssertNear(expected.X, end.X, "YZ LineEnd.X");
        AssertNear(expected.Y, end.Y, "YZ LineEnd.Y");
    }

    [Fact]
    public void SketchCreatedHandle_StaysLive_NotProjectedConstant()
    {
        var api = MakeApi();
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "Live");
        var line = sketch.AddCLine(new Vec2D(2, 3), new Vec2D(6, 7));
        CVec2D start = sketch.GetConstraintPoint("Line1@0.000");
        CVec2D mid = sketch.GetConstraintPoint("Live:Line1@0.500");
        Assert.False(ConstrainedSketcher.IsConstantPoint(start));
        Assert.False(ConstrainedSketcher.IsConstantPoint(mid));
        Assert.Same(line.CStart, start);
        var m = mid.Evaluate();
        AssertNear(4.0, m.X, "Mid.X");
        AssertNear(5.0, m.Y, "Mid.Y");
    }

    [Fact]
    public void CircleCenterHandle_StaysLive()
    {
        var sketch = MakeSketch("FaceSketch");
        var circle = sketch.AddCCircle(new Vec2D(3, 4), 2.0);
        CVec2D center = sketch.GetConstraintPoint("Circle1@center");
        CVec2D qualified = sketch.GetConstraintPoint("FaceSketch:Circle1@center");
        Assert.False(ConstrainedSketcher.IsConstantPoint(center));
        Assert.Same(circle.CCenter, center);
        Assert.Same(circle.CCenter, qualified);
        Vec2D uv = center.Evaluate();
        AssertNear(3.0, uv.X, "Center.X");
        AssertNear(4.0, uv.Y, "Center.Y");
    }

    [Fact]
    public void SketchOrigin_ResolvesConstantZero()
    {
        var sketch = MakeSketch("FaceSketch");
        CVec2D origin = sketch.GetConstraintPoint("Origin");
        CVec2D qualified = sketch.GetConstraintPoint("FaceSketch:Origin");
        CVec2D qualifiedLower = sketch.GetConstraintPoint("FaceSketch:origin");
        Assert.True(ConstrainedSketcher.IsConstantPoint(origin));
        Assert.True(ConstrainedSketcher.IsConstantPoint(qualified));
        Assert.True(ConstrainedSketcher.IsConstantPoint(qualifiedLower));
        AssertNear(0.0, origin.Evaluate().X, "Origin.X");
        AssertNear(0.0, origin.Evaluate().Y, "Origin.Y");
        AssertNear(0.0, qualified.Evaluate().X, "QualifiedOrigin.X");
        AssertNear(0.0, qualified.Evaluate().Y, "QualifiedOrigin.Y");
        AssertNear(0.0, qualifiedLower.Evaluate().X, "QualifiedOriginLower.X");
        AssertNear(0.0, qualifiedLower.Evaluate().Y, "QualifiedOriginLower.Y");
    }

    [Fact]
    public void NamedConstraints_DistanceAndPointOnLine_UseProjectedAnchor()
    {
        var api = MakeApi();
        ExtrudeUnitBox(api, "box");
        string side = EntityNaming.ExtrudeSide("box", "Line2");
        string top = EntityNaming.ExtrudeTop("box");
        string name = "box:" + EntityNaming.FormatEdgePointAddress(side, top, 0.25);
        Assert.True(api.TryGetPointFromName(name, out Vec3D world));

        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "Dims");
        Vec2D uv = sketch.ProjectWorldPoint(world);
        var line = sketch.AddCLine(new Vec2D(0, 0), new Vec2D(1, 0));
        sketch.FixPoint(line.CStart, new Vec2D(0, 0));
        sketch.SetHorizontal(line);
        sketch.SetDistance(line.CEnd, name, 2.0);
        AssertSolved(sketch.SolveConstraints());

        var end = line.CEnd.Evaluate();
        double d = Math.Sqrt((end.X - uv.X) * (end.X - uv.X) + (end.Y - uv.Y) * (end.Y - uv.Y));
        AssertNear(2.0, d, "Distance to projected model point");

        var through = sketch.AddCLine(new Vec2D(0, 0), new Vec2D(1, 0));
        sketch.FixPoint(through.CStart, new Vec2D(0, 0));
        sketch.SetPointOnLine(name, through);
        AssertSolved(sketch.SolveConstraints());
        var dir = LineDir(through);
        AssertNear(0.0, dir.X * uv.Y - dir.Y * uv.X, "PointOnLine collinear with projected point");
    }
}
