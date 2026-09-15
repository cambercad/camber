using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class CircularEdgeAnchorTests : IDisposable
{
    public void Dispose()
    {
        GeoAPI.Clear();
    }

    [Fact]
    public void ExtrudedCircle_ClosedEdgeAnchors_AlignWithSketchAxes()
    {
        const double radius = 1.0;
        const double height = 2.0;
        const double maxDev = 0.01;

        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), maxDev);
        var cyl = api.CreateCylinder(CoordinateSystem.Default, radius, height, maxDev, "cyl");

        Assert.True(cyl.TryGetCylinderFromPatch("cyl-Circle1", out var cylinder) ||
                    TryFindCylindricalPatch(cyl, out cylinder));
        Assert.NotNull(cylinder);
        Assert.False(cyl.PreferLexClosedLoopStarts);

        Vec3D refDir = cylinder.RefDir.Normalized();
        Vec3D axis = cylinder.Axis.Normalized();
        Vec3D quarterDir = Vec3DOps.Cross(axis, refDir).Normalized();
        // Cylinder frame is RH: angle increases from RefDir toward Cross(Axis, RefDir)?
        // Circle2D: angle 0 = +X, 90° = +Y; extrude Frame Z = Axis, so quarter = Frame.Y.
        // Cross(Axis, RefDir) = Cross(Z, X) = Y — correct for RH.

        var closedCircular = cyl.GroupEdges
            .Where(e => e.LineStrips3D.Count > 0 && e.LineStrips3D[0].IsClosed())
            .ToList();
        Assert.True(closedCircular.Count >= 2, "Expected top and bottom circular edges");

        foreach (var edge in closedCircular)
        {
            var strip = edge.LineStrips3D[0];
            Vec3D c = ApproximateCentroid(strip);

            AssertNearAxis(strip.EvaluateUniform(0.00) - c, refDir, "u=0 (angle 0)");
            AssertNearAxis(strip.EvaluateUniform(0.25) - c, quarterDir, "u=0.25 (angle 90°)");
            AssertNearAxis(strip.EvaluateUniform(0.50) - c, -refDir, "u=0.50 (angle 180°)");
            AssertNearAxis(strip.EvaluateUniform(0.75) - c, -quarterDir, "u=0.75 (angle 270°)");
        }
    }

    [Fact]
    public void ExtrudedCircle_OnYZSketch_AnchorsFollowSketchLocalX()
    {
        const double radius = 0.75;
        const double maxDev = 0.01;

        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), maxDev);
        // OriginYZ: sketch X → world Y, sketch Y → world Z, normal → world X
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginYZ, "yzCircle");
        sketch.AddCircle(new Vec2D(0, 0), radius);
        var cyl = api.Extrude(sketch, 1.5, maxDev, "pipe");

        Assert.True(TryFindCylindricalPatch(cyl, out var cylinder));
        Vec3D refDir = cylinder.RefDir.Normalized();
        // Sketch local +X on OriginYZ is world +Y
        Assert.True(Vec3DOps.Dot(refDir, new Vec3D(0, 1, 0)) > 0.99,
            $"Expected cylinder RefDir along world +Y, got {refDir}");

        Vec3D axis = cylinder.Axis.Normalized();
        Vec3D quarterDir = Vec3DOps.Cross(axis, refDir).Normalized();

        foreach (var edge in cyl.GroupEdges.Where(e => e.LineStrips3D.Count > 0 && e.LineStrips3D[0].IsClosed()))
        {
            var strip = edge.LineStrips3D[0];
            Vec3D c = ApproximateCentroid(strip);
            AssertNearAxis(strip.EvaluateUniform(0.00) - c, refDir, "u=0");
            AssertNearAxis(strip.EvaluateUniform(0.25) - c, quarterDir, "u=0.25");
        }
    }

    private static int UniquePointCount(LineStrip3D strip)
    {
        int n = strip.Points.Count;
        if (n > 1 && Vec3DOps.DistanceSquared(strip.Points[0], strip.Points[n - 1]) < 1e-16)
            n--;
        return n;
    }

    private static bool TryFindCylindricalPatch(AnchorMesh mesh, out CylinderSurfaceParams cylinder)
    {
        cylinder = null;
        foreach (var kv in mesh.surfaceMetaData)
        {
            if (kv.Value.SurfaceType == SurfaceType.Cylindrical && kv.Value.CylinderParams != null)
            {
                cylinder = kv.Value.CylinderParams;
                return true;
            }
        }
        return false;
    }

    private static Vec3D ApproximateCentroid(LineStrip3D strip)
    {
        Vec3D sum = new Vec3D(0, 0, 0);
        int n = UniquePointCount(strip);
        for (int i = 0; i < n; i++)
            sum += strip.Points[i];
        return sum / n;
    }

    private static void AssertNearAxis(Vec3D radial, Vec3D expectedDir, string label)
    {
        double len = radial.Length();
        Assert.True(len > 1e-8, $"{label}: radial length near zero");
        Vec3D dir = radial / len;
        double alignment = Vec3DOps.Dot(dir, expectedDir.Normalized());
        Assert.True(alignment > 0.99,
            $"{label}: expected along {expectedDir}, got {dir} (dot={alignment:F4})");
    }
}
