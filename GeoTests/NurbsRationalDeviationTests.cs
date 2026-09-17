using GeoCore;
using NURBS;

namespace GeoTests;

public class NurbsRationalDeviationTests
{
    [Theory]
    [InlineData(1, 1)]
    [InlineData(2, 3)]
    [InlineData(3, 2)]
    [InlineData(3, 4)]
    public void RationalResidualBoundsIndependentInteriorTriangles(int degreeU, int degreeV)
    {
        var random = new Random(721);
        var points = Enumerable.Range(0, degreeU + 1).Select(u =>
            Enumerable.Range(0, degreeV + 1).Select(v =>
                new Vec3D(10.0 * u / degreeU, 30.0 * v / degreeV, random.NextDouble() * 8)).ToArray()).ToArray();
        var weights = points.Select(row => row.Select(_ => .15 + 3 * random.NextDouble()).ToArray()).ToArray();
        double[] Knots(int degree) => Enumerable.Repeat(0d, degree + 1)
            .Concat(Enumerable.Repeat(1d, degree + 1)).ToArray();
        var surface = new BSplineSurface(degreeU, degreeV, points, weights, Knots(degreeU), Knots(degreeV));
        for (int patch = 0; patch < 20; patch++)
        {
            double u0 = random.NextDouble() * .8, v0 = random.NextDouble() * .8;
            double u1 = u0 + random.NextDouble() * (1 - u0), v1 = v0 + random.NextDouble() * (1 - v0);
            double bound = AdaptiveSurfaceSplitter.TriangleDeviationBound(surface, u0, v0, u1, v1);
            var uv = Enumerable.Range(0, 3).Select(_ =>
                new Vec2D(u0 + random.NextDouble() * (u1 - u0), v0 + random.NextDouble() * (v1 - v0))).ToArray();
            var vertices = uv.Select(p => surface.Evaluate(p.X, p.Y)).ToArray();
            for (int i = 0; i <= 8; i++)
            for (int j = 0; j <= 8 - i; j++)
            {
                double a = i / 8.0, b = j / 8.0, c = 1 - a - b;
                var parameter = uv[0] * a + uv[1] * b + uv[2] * c;
                var linear = vertices[0] * a + vertices[1] * b + vertices[2] * c;
                Assert.InRange((surface.Evaluate(parameter.X, parameter.Y) - linear).Length(), 0, bound + 1e-11);
            }
        }
    }
}
