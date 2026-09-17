using GeoCore;

namespace GeoTests;

public class SphereFitterTests
{
    [Theory]
    [InlineData(0, 0, 0, .5)]
    [InlineData(33, 17, 8, .5)]
    [InlineData(1000, -2000, 3000, .01)]
    public void OctantFitPreservesRadiusAndCenterAcrossTranslationsAndScales(double x, double y, double z, double radius)
    {
        var center = new Vec3D(x, y, z);
        var points = new List<Vec3D>();
        for (int i = 0; i <= 30; i++)
        {
            double angle = i * Math.PI / 60;
            double c = Math.Cos(angle), s = Math.Sin(angle);
            points.Add(center + new Vec3D(c, s, 0) * radius);
            points.Add(center + new Vec3D(0, c, s) * radius);
            points.Add(center + new Vec3D(s, 0, c) * radius);
        }
        Assert.True(SphereFitter.FitSphere(points, out var fittedCenter, out double fittedRadius));
        Assert.InRange((fittedCenter - center).Length(), 0, 1e-9);
        Assert.InRange(fittedRadius, radius - 1e-9, radius + 1e-9);
    }
}
