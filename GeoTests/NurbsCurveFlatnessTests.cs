using GeoCore;
using NURBS;

namespace GeoTests;

public class NurbsCurveFlatnessTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(3)]
    public void StraightMultiSpanCurveNeedsOnlyOneSegment(int degree)
    {
        var points = Enumerable.Range(0, 7).Select(i => new Vec3D(i, 0, 0)).ToArray();
        var curve = new BSplineCurve(degree, points, BSplineCurve.UniformKnotVector(degree, points.Length), false);
        Assert.Equal(2, curve.Tessellate(.01).Count);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(3)]
    public void LaterSpansCannotBeDiscardedAsStraight(int degree)
    {
        var points = Enumerable.Range(0, 7).Select(i => new Vec3D(i, i == 5 ? 4 : 0, 0)).ToArray();
        var curve = new BSplineCurve(degree, points, BSplineCurve.UniformKnotVector(degree, points.Length), false);
        CheckDeviation(curve, .01);
    }

    [Fact]
    public void CollinearOvershootMustNotCollapseToEndpointChord()
    {
        var curve = new BSplineCurve(3,
            new[] { new Vec3D(0, 0, 0), new Vec3D(4, 0, 0), new Vec3D(4, 0, 0), new Vec3D(1, 0, 0) },
            new[] { 0d, 0, 0, 0, 1, 1, 1, 1 }, false);
        CheckDeviation(curve, .01);
    }

    private static void CheckDeviation(BSplineCurve curve, double tolerance)
    {
        var mesh = curve.Tessellate(tolerance);
        Assert.True(mesh.Count > 2);
        for (int i = 0; i <= 1000; i++)
        {
            var p = curve.EvaluateUniform(i / 1000d);
            double distance = double.PositiveInfinity;
            for (int j = 1; j < mesh.Count; j++)
            {
                var delta = mesh[j] - mesh[j - 1];
                double length2 = delta.Dot(delta);
                double t = length2 == 0 ? 0 : Math.Clamp((p - mesh[j - 1]).Dot(delta) / length2, 0, 1);
                distance = Math.Min(distance, (p - mesh[j - 1] - delta * t).Length());
            }
            Assert.InRange(distance, 0, tolerance * 1.001);
        }
    }
}
