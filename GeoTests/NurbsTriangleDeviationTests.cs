using GeoCore;
using NURBS;

namespace GeoTests;

public class NurbsTriangleDeviationTests
{
    [Theory]
    [InlineData(1.0)]
    [InlineData(7.0)]
    [InlineData(-3.0)]
    public void RationalPatchRetainsItsShapeAndInteriorAccuracyUnderHomogeneousScaling(double scale)
    {
        Vec3D[][] points =
        [
            [new(2, 0, 0), new(3, 0, 1)],
            [new(2, 2, 0), new(3, 3, 1)],
            [new(0, 2, 0), new(0, 3, 1)]
        ];
        double middle = Math.Sqrt(.5) * scale;
        double[][] weights = [[scale, scale], [middle, middle], [scale, scale]];
        var surface = new BSplineSurface(2, 1, points, weights,
            [0, 0, 0, 1, 1, 1], [0, 0, 1, 1]);
        const double deviation = .025;
        var mesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface, maxDeviation: deviation);
        AssertInteriorAccuracy(surface, mesh, deviation);
        foreach (var point in mesh.Points)
            Assert.InRange(Math.Abs(Math.Sqrt(point.X * point.X + point.Y * point.Y) - (2 + point.Z)), 0, 1e-11);
    }

    [Fact]
    public void NarrowParameterFeatureIsNotAcceptedBeforeMeetingGeometricDeviation()
    {
        Vec3D[][] points =
        [
            [new(0, 0, 0), new(0, 0, 1)],
            [new(.25, 3, 0), new(.25, 3, 1)],
            [new(.5, 0, 0), new(.5, 0, 1)],
            [new(1, 0, 0), new(1, 0, 1)],
            [new(2, 0, 0), new(2, 0, 1)]
        ];
        var surface = new BSplineSurface(2, 1, points,
            [0, 0, 0, .0005, .0005, 1, 1, 1], [0, 0, 1, 1]);
        const double deviation = .03;
        var mesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface, maxDeviation: deviation);
        AssertInteriorAccuracy(surface, mesh, deviation);
        Assert.InRange(mesh.Points.Max(point => point.Y), 1.5 - deviation, 1.5 + deviation);
    }

    internal static void AssertInteriorAccuracy(BSplineSurface surface, TriangulatedGeometry mesh, double deviation)
    {
        Assert.NotEmpty(mesh.Triangles);
        foreach (var triangle in mesh.Triangles)
        for (int i = 0; i <= 6; i++)
        for (int j = 0; j <= 6 - i; j++)
        {
            double a = i / 6.0, b = j / 6.0, c = 1 - a - b;
            var uv = mesh.UV[triangle.A] * a + mesh.UV[triangle.B] * b + mesh.UV[triangle.C] * c;
            var point = mesh.Points[triangle.A] * a + mesh.Points[triangle.B] * b + mesh.Points[triangle.C] * c;
            double error = (surface.Evaluate(uv.X, uv.Y) - point).Length();
            Assert.InRange(error, 0, deviation + 1e-10);
        }
    }
}
