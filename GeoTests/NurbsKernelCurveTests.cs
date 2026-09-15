using GeoCore;
using NURBS;

namespace NURBS.Tests;

public class CurveTests
{
    [Fact]
    public void BSplineLine_EndpointsMatch()
    {
        var line = new BSplineLine(new Vec3D(0, 0, 0), new Vec3D(3, 4, 0));
        var start = line.EvaluateUniform(0);
        var end = line.EvaluateUniform(1);
        Assert.InRange((start - new Vec3D(0, 0, 0)).Length(), 0, 1e-9);
        Assert.InRange((end - new Vec3D(3, 4, 0)).Length(), 0, 1e-9);
        Assert.InRange(line.TotalArcLength, 4.9, 5.1);
    }

    [Fact]
    public void BSplineCircle_RadiusAtQuadrants()
    {
        var circle = new BSplineCircle(new Vec3D(0, 0, 0), new Vec3D(0, 0, 1), 2.0, new Vec3D(2, 0, 0));
        var curve = circle.GetNativeParamRangeCurve();
        double r0 = curve.EvaluateUniform(0).Length();
        double r25 = curve.EvaluateUniform(0.25).Length();
        Assert.InRange(r0, 1.98, 2.02);
        Assert.InRange(r25, 1.98, 2.02);
    }

    [Fact]
    public void BSplineCircle_MidChordNearCircle()
    {
        var circle = new BSplineCircle(new Vec3D(0, 0, 0), new Vec3D(0, 0, 1), 1.0, new Vec3D(1, 0, 0));
        var curve = circle.GetNativeParamRangeCurve();
        double midParam = curve.GetCurveParameter(0.5 * curve.TotalArcLength);
        var onCurve = curve.EvaluateUniform(midParam);
        Assert.InRange(onCurve.Length(), 0.98, 1.02);
    }

    [Fact]
    public void BSplineCurve_SplitPreservesEndpoints()
    {
        var curve = new BSplineCurve(1,
            new[] { new Vec3D(0, 0, 0), new Vec3D(2, 0, 0) },
            new[] { 0.0, 0.0, 1.0, 1.0 },
            closedCurve: false);
        curve.Split(0.5, out var left, out var right);
        Assert.InRange((left.EvaluateUniform(0) - new Vec3D(0, 0, 0)).Length(), 0, 1e-8);
        Assert.InRange((right.EvaluateUniform(1) - new Vec3D(2, 0, 0)).Length(), 0, 1e-8);
    }
}
