using Curves;
using Geo.NurbsConstruction;
using GeoCore;

namespace GeoTests;

public class CurveTests
{
    private const double Eps = 1e-6;

    [Fact]
    public void Bezier2D_AdaptiveTessellation_DoesNotMutateCurve()
    {
        var curve = new Bezier2D(
            new Vec2D(0, 0),
            new Vec2D(0, 2),
            new Vec2D(2, 2),
            new Vec2D(2, 0));
        var original = curve.ControlPoints.ToArray();

        var first = curve.Tessellate(0.01);
        var second = curve.Tessellate(0.01);

        Assert.Equal(original, curve.ControlPoints);
        Assert.Equal(first.Count, second.Count);
        for (int i = 0; i < first.Count; i++)
        {
            Assert.Equal(first[i].Position.X, second[i].Position.X, Eps);
            Assert.Equal(first[i].Position.Y, second[i].Position.Y, Eps);
        }
    }

    [Fact]
    public void Bezier3D_AdaptiveTessellation_DoesNotMutateControlPoints()
    {
        var controlPoints = new List<Vec3D>
        {
            new Vec3D(0, 0, 0),
            new Vec3D(0, 2, 1),
            new Vec3D(2, 2, 1),
            new Vec3D(2, 0, 0),
        };
        var original = controlPoints.ToArray();

        var first = BezierTessellator.Tessellate(0.01, controlPoints);
        var second = BezierTessellator.Tessellate(0.01, controlPoints);

        Assert.Equal(original, controlPoints);
        Assert.Equal(first, second);
    }

    [Fact]
    public void Bezier2D_Length_IntegratesUnnormalizedDerivative()
    {
        var curve = new Bezier2D(
            new Vec2D(0, 0),
            new Vec2D(2.0 / 3.0, 0),
            new Vec2D(4.0 / 3.0, 0),
            new Vec2D(2, 0));

        Assert.Equal(2, curve.Length(), Eps);
    }

    [Fact]
    public void Line3D_EvaluateAndLength_AreCorrect()
    {
        var line = new Line3D(new Vec3D(0, 0, 0), new Vec3D(0, 0, 10), "line");

        Assert.Equal(10, line.Length(), Eps);

        var pMid = line.Evaluate(0.5).Origin;
        Assert.Equal(0, pMid.X, Eps);
        Assert.Equal(0, pMid.Y, Eps);
        Assert.Equal(5, pMid.Z, Eps);
    }

    [Fact]
    public void Arc3D_Evaluate_StartMidEnd_AreValid()
    {
        var arc = new Arc3D(
            center: new Vec3D(0, 0, 0),
            x: new Vec3D(1, 0, 0),
            y: new Vec3D(0, 1, 0),
            radius: 2,
            startAngle: 0,
            sweepAngle: Math.PI * 0.5,
            name: "arc");

        var start = arc.Evaluate(0.0).Origin;
        var mid = arc.Evaluate(0.5).Origin;
        var end = arc.Evaluate(1.0).Origin;

        Assert.Equal(2, start.X, Eps);
        Assert.Equal(0, start.Y, Eps);

        Assert.InRange(mid.X, 1.3, 1.5);
        Assert.InRange(mid.Y, 1.3, 1.5);

        Assert.Equal(0, end.X, Eps);
        Assert.Equal(2, end.Y, Eps);
    }

    [Fact]
    public void ClockwiseArc3DNurbsFollowsShortSpan()
    {
        var start = new Vec3D(1, 0, 0);
        var mid = new Vec3D(Math.Sqrt(0.5), -Math.Sqrt(0.5), 0);
        var end = new Vec3D(0, -1, 0);
        var arc = new Arc3D(start, mid, end);
        Assert.True(Math.Abs(arc.SweepAngle) < Math.PI);

        var nurbs = Curve3DToBSpline.ToBSplineCurve(arc);
        Assert.True(nurbs.ControlPoints.Length <= 5, "got " + nurbs.ControlPoints.Length);
        double worst = 0;
        for (int i = 0; i <= 16; i++)
        {
            Vec3D p = nurbs.EvaluateUniform(i / 16.0);
            double best = double.MaxValue;
            for (int j = 0; j <= 32; j++)
                best = Math.Min(best, (p - arc.Evaluate(j / 32.0).Origin).Length());
            worst = Math.Max(worst, best);
        }
        Assert.True(worst < 0.02, "NURBS left the short 3D arc, err " + worst);
    }

    [Fact]
    public void Circle3D_Evaluate_ClosedCurveBehavior()
    {
        var circle = new Circle3D(
            center: new Vec3D(1, 2, 3),
            x: new Vec3D(1, 0, 0),
            y: new Vec3D(0, 1, 0),
            radius: 5,
            name: "circle");

        var p0 = circle.Evaluate(0.0).Origin;
        var p1 = circle.Evaluate(1.0).Origin;

        Assert.Equal(p0.X, p1.X, Eps);
        Assert.Equal(p0.Y, p1.Y, Eps);
        Assert.Equal(p0.Z, p1.Z, Eps);
    }

    [Fact]
    public void Helix3D_Evaluate_HasExpectedPitch()
    {
        var helix = new Helix3D(
            center: new Vec3D(0, 0, 0),
            x: new Vec3D(1, 0, 0),
            y: new Vec3D(0, 1, 0),
            radius: 1.0,
            zAdvancementPerRevolution: 2.0,
            numRevolutions: 3.0,
            name: "helix");

        var start = helix.Evaluate(0.0).Origin;
        var end = helix.Evaluate(1.0).Origin;

        Assert.Equal(0.0, start.Z, Eps);
        Assert.Equal(6.0, end.Z, Eps);
    }

    [Fact]
    public void Spiral3D_Evaluate_HasExpectedStartEndRadius()
    {
        var spiral = new Spiral3D(
            center: new Vec3D(0, 0, 0),
            x: new Vec3D(1, 0, 0),
            y: new Vec3D(0, 1, 0),
            startRadius: 1.0,
            endRadius: 3.0,
            zAdvancementPerRevolution: 0,
            numRevolutions: 1.0,
            name: "spiral");

        var start = spiral.Evaluate(0.0).Origin;
        var end = spiral.Evaluate(1.0).Origin;

        Assert.Equal(1.0, (start - spiral.Center).Length(), Eps);
        Assert.Equal(3.0, (end - spiral.Center).Length(), Eps);
        Assert.Equal(0.0, start.Z, Eps);
        Assert.Equal(3.0, end.X, Eps);
        Assert.Equal(0.0, end.Y, Eps);
    }

    [Fact]
    public void Spiral3D_ConstantRadius_MatchesHelix()
    {
        var helix = new Helix3D(
            center: new Vec3D(0, 0, 0),
            x: new Vec3D(1, 0, 0),
            y: new Vec3D(0, 1, 0),
            radius: 2.0,
            zAdvancementPerRevolution: 1.5,
            numRevolutions: 2.0,
            name: "helix");

        var spiral = new Spiral3D(
            center: new Vec3D(0, 0, 0),
            x: new Vec3D(1, 0, 0),
            y: new Vec3D(0, 1, 0),
            startRadius: 2.0,
            endRadius: 2.0,
            zAdvancementPerRevolution: 1.5,
            numRevolutions: 2.0,
            name: "spiral");

        for (int i = 0; i <= 10; i++)
        {
            double u = i / 10.0;
            var helixPoint = helix.Evaluate(u).Origin;
            var spiralPoint = spiral.Evaluate(u).Origin;
            Assert.Equal(helixPoint.X, spiralPoint.X, Eps);
            Assert.Equal(helixPoint.Y, spiralPoint.Y, Eps);
            Assert.Equal(helixPoint.Z, spiralPoint.Z, Eps);
        }
    }
}
