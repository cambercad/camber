using CSG;
using Curves;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

[Collection("GeoAPISequential")]
public class SketchMeshProjectionTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void ProjectRectangleOntoCubeFace_HitsLieOnPlane_NamesPreserved()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(new CoordinateSystem(new Vec3D(-1, -1, -1)), 2.0, "cube");

        // Plane above the cube top (z=1); local +Z points toward the mesh (-world Z).
        var plane = new CoordinateSystem(
            new Vec3D(0, 0, 2),
            new Vec3D(1, 0, 0),
            new Vec3D(0, -1, 0),
            new Vec3D(0, 0, -1));
        var sk = api.GetPlotterSketcher(plane, "top_sk");
        sk.AddLine(new Vec2D(-0.4, -0.4), new Vec2D(0.4, -0.4)); // will be Line1 etc.
        sk.AddLine(new Vec2D(0.4, -0.4), new Vec2D(0.4, 0.4));
        sk.AddLine(new Vec2D(0.4, 0.4), new Vec2D(-0.4, 0.4));
        sk.AddLine(new Vec2D(-0.4, 0.4), new Vec2D(-0.4, -0.4));

        var projected = api.ProjectSketchOntoMesh(sk, cube, name: "proj");
        Assert.Equal(4, projected.Curves.Count);
        Assert.Contains(projected.Curves, c => c.Name == "proj-Line1");

        foreach (var strip in projected.Vertices)
        {
            foreach (var seg in strip)
            {
                foreach (var v in seg)
                {
                    Assert.True(Math.Abs(v.Hit.Z - 1.0) < 1e-4, $"hit Z={v.Hit.Z}");
                    Assert.True(Vec3DOps.Dot(v.SurfaceNormal, new Vec3D(0, 0, 1)) > 0.9);
                }
            }
        }
    }

    [Fact]
    public void ExtrudeProjected_PlanarSquare_VolumeMatchesAreaTimesHeight()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(new CoordinateSystem(new Vec3D(-2, -2, -1)), 4.0, "cube");

        var plane = new CoordinateSystem(
            new Vec3D(0, 0, 2),
            new Vec3D(1, 0, 0),
            new Vec3D(0, -1, 0),
            new Vec3D(0, 0, -1));
        var sk = api.GetPlotterSketcher(plane, "sq");
        double half = 0.5;
        sk.AddLine(new Vec2D(-half, -half), new Vec2D(half, -half));
        sk.AddLine(new Vec2D(half, -half), new Vec2D(half, half));
        sk.AddLine(new Vec2D(half, half), new Vec2D(-half, half));
        sk.AddLine(new Vec2D(-half, half), new Vec2D(-half, -half));

        var projected = api.ProjectSketchOntoMesh(sk, cube, name: "sq_proj");
        double height = 0.25;
        // Positive height follows surface normals (cube top normal is +Z, outward).
        var cutter = api.ExtrudeProjectedSketch(projected, height, "cutter");

        double expectedVolume = (2 * half) * (2 * half) * Math.Abs(height);
        MeshPipelineTestHelpers.AssertVolume(cutter.Mesh, expectedVolume, tolerance: 1e-3);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(cutter.Mesh);
    }

    [Fact]
    public void ProjectOntoCylinder_HitsAtRadius_UpParallelToRadial()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), 0.02);
        double radius = 10.0;
        var cyl = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, -15)), radius, 30.0, 0.02, "cyl");

        var plane = new CoordinateSystem(
            new Vec3D(radius + 5, 0, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(0, 0, -1),
            new Vec3D(-1, 0, 0));
        var sk = api.GetPlotterSketcher(plane, "feat");
        sk.AddLine(new Vec2D(-2, -3), new Vec2D(2, -3));
        sk.AddLine(new Vec2D(2, -3), new Vec2D(2, 3));
        sk.AddLine(new Vec2D(2, 3), new Vec2D(-2, 3));
        sk.AddLine(new Vec2D(-2, 3), new Vec2D(-2, -3));

        var projected = api.ProjectSketchOntoMesh(sk, cyl, name: "on_cyl");
        foreach (var strip in projected.Vertices)
        {
            foreach (var seg in strip)
            {
                foreach (var v in seg)
                {
                    double r = Math.Sqrt(v.Hit.X * v.Hit.X + v.Hit.Y * v.Hit.Y);
                    Assert.True(Math.Abs(r - radius) < 0.15, $"radius {r} vs {radius}");

                    Vec3D radial = new Vec3D(v.Hit.X, v.Hit.Y, 0).Normalized();
                    Assert.True(Vec3DOps.Dot(v.SurfaceNormal, radial) > 0.85,
                        $"Up not radial: n={v.SurfaceNormal}, radial={radial}");
                }
            }
        }

        // Negative height = into the solid (opposite outward normal).
        var cutter = api.ExtrudeProjectedSketch(projected, -1.5, "cutter");
        Assert.True(MeshPipelineTestHelpers.AbsVolume(cutter.Mesh) > 1e-3);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(cutter.Mesh);

        double hostVol = MeshPipelineTestHelpers.AbsVolume(cyl.Mesh);
        var diff = api.Boolean(cyl, cutter, BooleanOp.Difference, "diff");
        double diffVol = MeshPipelineTestHelpers.AbsVolume(diff.Mesh);
        double cutterVol = MeshPipelineTestHelpers.AbsVolume(cutter.Mesh);
        // Cutter may extend slightly outside; volume drop should be positive and below cutter volume.
        Assert.True(diffVol < hostVol);
        Assert.True(hostVol - diffVol > 0.1);
        Assert.True(hostVol - diffVol <= cutterVol + 0.5);
    }

    [Fact]
    public void ProjectMiss_ThrowsWithCurveName()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(new CoordinateSystem(new Vec3D(-0.5, -0.5, -0.5)), 1.0, "cube");

        // Plane below cube casting +Z — sketch is offset so rays miss the cube.
        var plane = new CoordinateSystem(
            new Vec3D(10, 0, -2),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(0, 0, 1));
        var sk = api.GetPlotterSketcher(plane, "miss_sk");
        sk.AddLine(new Vec2D(0, 0), new Vec2D(0.2, 0));
        sk.AddLine(new Vec2D(0.2, 0), new Vec2D(0.2, 0.2));
        sk.AddLine(new Vec2D(0.2, 0.2), new Vec2D(0, 0.2));
        sk.AddLine(new Vec2D(0, 0.2), new Vec2D(0, 0));

        var ex = Assert.Throws<InvalidOperationException>(() =>
            api.ProjectSketchOntoMesh(sk, cube, name: "miss"));
        Assert.Contains("Line", ex.Message, StringComparison.Ordinal);
        Assert.Contains("missed", ex.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void ProjectedCurve_UpMatchesSurfaceNormal()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(new CoordinateSystem(new Vec3D(-1, -1, -1)), 2.0, "cube");
        var plane = new CoordinateSystem(
            new Vec3D(0, 0, 3),
            new Vec3D(1, 0, 0),
            new Vec3D(0, -1, 0),
            new Vec3D(0, 0, -1));
        var sk = api.GetPlotterSketcher(plane, "sk");
        sk.AddLine(new Vec2D(-0.2, -0.2), new Vec2D(0.2, -0.2));
        sk.AddLine(new Vec2D(0.2, -0.2), new Vec2D(0.2, 0.2));
        sk.AddLine(new Vec2D(0.2, 0.2), new Vec2D(-0.2, 0.2));
        sk.AddLine(new Vec2D(-0.2, 0.2), new Vec2D(-0.2, -0.2));

        var projected = api.ProjectSketchOntoMesh(sk, cube, name: "p");
        foreach (var curve in projected.Curves)
        {
            foreach (var v in curve.Vertices)
            {
                Assert.True(Vec3DOps.Dot(v.Up.Normalized(), new Vec3D(0, 0, 1)) > 0.9);
            }
        }
    }
}
