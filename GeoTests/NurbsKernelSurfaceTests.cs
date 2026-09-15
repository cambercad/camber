using GeoCore;
using NURBS;

namespace NURBS.Tests;

public class SurfaceTests
{
    [Fact]
    public void BSplinePlane_CornersOnPlane()
    {
        var plane = new BSplinePlane(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0));
        foreach (var uv in new[] { (0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0) })
        {
            var p = plane.Evaluate(uv.Item1, uv.Item2);
            Assert.InRange(Math.Abs(p.Z), 0, 1e-9);
        }
    }

    [Fact]
    public void LinearExtrude_CornerHeight()
    {
        var profile = new BSplineCurve(1,
            new[] { new Vec3D(0, 0, 0), new Vec3D(1, 0, 0) },
            new[] { 0.0, 0.0, 1.0, 1.0 },
            closedCurve: false);
        var surface = new BSplineLinearExtrudeSurface(profile, new Vec3D(0, 0, 1), 2.0);
        var p0 = surface.Evaluate(0, 0);
        var p1 = surface.Evaluate(0, 1);
        var p2 = surface.Evaluate(1, 0);
        var p3 = surface.Evaluate(1, 1);
        double maxZ = Math.Max(Math.Max(p0.Z, p1.Z), Math.Max(p2.Z, p3.Z));
        double minZ = Math.Min(Math.Min(p0.Z, p1.Z), Math.Min(p2.Z, p3.Z));
        Assert.InRange(maxZ - minZ, 1.99, 2.01);
    }

    [Fact]
    public void Revolution_CylinderLikeRadius()
    {
        var profile = new BSplineCurve(1,
            new[] { new Vec3D(1, 0, 0), new Vec3D(1, 0, 2) },
            new[] { 0.0, 0.0, 1.0, 1.0 },
            closedCurve: false);
        var surface = new BSplineSurfaceOfRevolution(
            new Vec3D(0, 0, 0), new Vec3D(0, 0, 1), profile, new Vec3D(1, 0, 0));
        var p = surface.Evaluate(0.5, 0.25);
        double r = Math.Sqrt(p.X * p.X + p.Y * p.Y);
        Assert.InRange(r, 0.98, 1.02);
    }

    [Fact]
    public void Plane_NormalConsistentWithEvaluate()
    {
        var plane = new BSplinePlane(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(2, 0, 0),
            new Vec3D(0, 2, 0));
        var n = plane.EvaluateNormal(0.5, 0.5);
        if (n.Length() > 1e-10)
            n = n.Normalized();
        Assert.InRange(Math.Abs(n.Z), 0.99, 1.01);
    }

    [Fact]
    public void EvaluatePointAndNormal_MatchesSeparateEvaluateAndNormal()
    {
        var plane = new BSplinePlane(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(2, 0, 0),
            new Vec3D(0, 2, 0));
        var p = plane.Evaluate(0.3, 0.7);
        var n = plane.EvaluateNormal(0.3, 0.7);
        var combined = plane.EvaluatePointAndNormal(0.3, 0.7, out var nCombined);
        Assert.InRange((p - combined).Length(), 0, 1e-9);
        Assert.InRange((n - nCombined).Length(), 0, 1e-9);
    }
}
