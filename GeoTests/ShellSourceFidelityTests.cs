using CSG;
using Geo;
using GeoCore;
using Xunit.Abstractions;

namespace GeoTests;

[Collection("GlobalCadState")]
public sealed class ShellSourceFidelityTests(ITestOutputHelper output) : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void RevolvedSphereAndBooleanContactRespectSurfaceTolerance()
    {
        const double tolerance = .005, radius = 5;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), tolerance);
        var sphere = api.CreateSphere(CoordinateSystem.Default, radius, tolerance, "sphere");
        double MaximumError(AnchorMesh body, string patch, bool includeInterior)
        {
            Assert.True(body.TryGetTopologySurface(patch, out var surface));
            double error = 0;
            foreach (var t in surface.Triangles)
            {
                var a = surface.Points[t.A]; var b = surface.Points[t.B]; var c = surface.Points[t.C];
                error = Math.Max(error, Math.Max(Math.Abs(a.Length() - radius),
                    Math.Max(Math.Abs(b.Length() - radius), Math.Abs(c.Length() - radius))));
                if (includeInterior)
                    for (int i = 0; i <= 4; i++)
                        for (int j = 0; j <= 4 - i; j++)
                            error = Math.Max(error, Math.Abs((a * (i / 4.0) + b * (j / 4.0) + c * ((4 - i - j) / 4.0)).Length() - radius));
            }
            return error;
        }
        double sourceError = MaximumError(sphere, "sphere-Arc1", true);
        var neck = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 3)), 2, 5, tolerance, "neck");
        var blank = api.Boolean(sphere, neck, BooleanOp.Union, "vessel");
        double contactError = MaximumError(blank, "sphere-Arc1", false);
        output.WriteLine($"Sphere interior error: {sourceError:G17}; Boolean sphere vertex error: {contactError:G17}; tolerance: {tolerance:G17}");
        double permitted = tolerance + 4 * Math.Sqrt(3) * api.Converter.SmallestUnit();
        Assert.InRange(sourceError, 0, permitted);
        Assert.InRange(contactError, 0, permitted);
    }
}
