using GeoCore;
using NURBS;

namespace NURBS.Tests;

public class TessellationTests
{
    [Fact]
    public void Plane_Tessellation_ProducesTriangles()
    {
        var plane = new BSplinePlane(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0));
        var geo = plane.Triangulate(maxDeviation: 0.05);
        Assert.True(geo.Triangles.Count > 0);
        Assert.True(geo.Points.Count >= 4);
    }

    [Fact]
    public void Plane_Tessellation_AreaNearFour()
    {
        var plane = new BSplinePlane(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0));
        var geo = plane.Triangulate(maxDeviation: 0.05);
        double area = ComputeTriangleAreaSum(geo.Points, geo.Triangles);
        Assert.InRange(area, 3.8, 4.2);
    }

    [Fact]
    public void CylinderRevolution_Tessellation_HasTriangles()
    {
        var profile = new BSplineCurve(1,
            new[] { new Vec3D(1, 0, 0), new Vec3D(1, 0, 1) },
            new[] { 0.0, 0.0, 1.0, 1.0 },
            closedCurve: false);
        var surface = new BSplineSurfaceOfRevolution(
            new Vec3D(0, 0, 0), new Vec3D(0, 0, 1), profile, new Vec3D(1, 0, 0));
        var geo = surface.Triangulate(maxDeviation: 0.05);
        Assert.True(geo.Triangles.Count > 10);
    }

    [Fact]
    public void Tessellation_RemovesZeroAreaTriangles()
    {
        var plane = new BSplinePlane(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0));
        var geo = plane.Triangulate(maxDeviation: 0.1);
        foreach (var tri in geo.Triangles)
        {
            var a = geo.Points[tri.A];
            var b = geo.Points[tri.B];
            var c = geo.Points[tri.C];
            double area2 = (b - a).Cross(c - a).Length();
            Assert.True(area2 > 1e-12);
        }
    }

    private static double ComputeTriangleAreaSum(IList<Vec3D> points, IList<Tri> triangles)
    {
        double sum = 0;
        foreach (var tri in triangles)
        {
            var a = points[tri.A];
            var b = points[tri.B];
            var c = points[tri.C];
            sum += 0.5 * (b - a).Cross(c - a).Length();
        }
        return sum;
    }
}
