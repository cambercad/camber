using GeoCore;
using NURBS;

namespace NURBS.Tests;

public class DeBoorTests
{
    [Theory]
    [InlineData(new[] { 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0 }, 2, 0.0)]
    [InlineData(new[] { 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0 }, 2, 0.25)]
    [InlineData(new[] { 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0 }, 2, 0.75)]
    [InlineData(new[] { 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0 }, 2, 1.0)]
    [InlineData(new[] { 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0 }, 2, 1.5)]
    public void KnotIndex_MatchesBruteForceSearch(double[] knots, int degree, double w)
    {
        int expected = BruteForceKnotIndex(w, degree, knots);
        int actual = DeBoor.KnotIndex(w, degree, knots);
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void KnotIndex_EvaluationUnchangedOnUniformSurface()
    {
        var surface = new BSplinePlane(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(2, 0, 0),
            new Vec3D(0, 2, 0));

        for (int i = 0; i <= 10; i++)
        {
            for (int j = 0; j <= 10; j++)
            {
                double u = i / 10.0;
                double v = j / 10.0;
                var p = surface.Evaluate(u, v);
                var n = surface.EvaluateNormal(u, v);
                Assert.True(Vec3DOps.Length(n) > 0);
                Assert.InRange(Math.Abs(p.Z), 0, 1e-9);
            }
        }
    }

    [Fact]
    public void KnotIndexFromHint_MatchesKnotIndex_OnSequentialGrid()
    {
        double[] knots = { 0.0, 0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0 };
        const int degree = 2;
        int hint = -1;
        for (int i = 0; i <= 100; i++)
        {
            double w = i / 100.0;
            int expected = DeBoor.KnotIndex(w, degree, knots);
            int actual = DeBoor.KnotIndexFromHint(ref hint, w, degree, knots);
            Assert.Equal(expected, actual);
        }
    }

    [Fact]
    public void KnotIndexFromHint_MatchesKnotIndex_OnRandomJumpingParameters()
    {
        double[] knots = { 0.0, 0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0 };
        const int degree = 2;
        var rng = new Random(12345);
        int hint = -1;
        for (int i = 0; i < 500; i++)
        {
            double w = rng.NextDouble();
            int expected = DeBoor.KnotIndex(w, degree, knots);
            int actual = DeBoor.KnotIndexFromHint(ref hint, w, degree, knots);
            Assert.Equal(expected, actual);
        }
    }

    private static int BruteForceKnotIndex(double w, int degree, double[] knots)
    {
        int k = knots.Length - 1;
        while (k >= degree)
        {
            if (knots[k] <= w)
                break;
            --k;
        }
        return Math.Max(degree, Math.Min(k, knots.Length - degree - 2));
    }
}
