using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class GeoAPITests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void CreateSphere_GeneratesMeshAndSceneNode()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), 1e-4);
        var sphere = api.CreateSphere(new CoordinateSystem(new Vec3D(0, 0, 0)), 1.0, 0.001, "sphere");

        AssertMesh(sphere);
        TestVisualization.Visualize(api, nameof(CreateSphere_GeneratesMeshAndSceneNode));
    }

    [Fact]
    public void Extrude_RectangleProfile_GeneratesMeshAndSceneNode()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(10)), 1e-4);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "rectProfile");
        sketch.AddRectangle(new Vec2D(0, 0), 1, 1);
        var mesh = api.Extrude(sketch, 1.0, 0.001, "rectExtrude");

        AssertMesh(mesh);
        TestVisualization.Visualize(api, nameof(Extrude_RectangleProfile_GeneratesMeshAndSceneNode));
    }

    [Fact]
    public void ExtrudeAlongCurve_Helix_GeneratesMeshAndSceneNode()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginZX, "profile");
        sketch.AddRectangle(new Vec2D(0, 0.25), 0.1, 0.1);

        var helix = new Helix3D(new Vec3D(0, 0, 0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), 0.25, 0.5, 2, rightHanded: true, name: "guide");
        var mesh = api.ExtrudeAlongCurve(sketch, helix, 0.0001, "helixExtrude");

        AssertMesh(mesh);
        TestVisualization.Visualize(api, nameof(ExtrudeAlongCurve_Helix_GeneratesMeshAndSceneNode));
    }

    [Fact]
    public void ExtrudeAlongClosedLoop_Circle_GeneratesMeshAndSceneNode()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), 1e-4);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginZX, "profile");
        sketch.AddRectangle(new Vec2D(0, 1), 0.2, 0.2);

        var circle = new Circle3D(new Vec3D(0, 0, 0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), 1.0, "guideCircle");
        var mesh = api.ExtrudeAlongCurve(sketch, circle, 0.001, "torusLike");

        AssertMesh(mesh);
        TestVisualization.Visualize(api, nameof(ExtrudeAlongClosedLoop_Circle_GeneratesMeshAndSceneNode));
    }

    [Fact]
    public void Revolve_Profile_GeneratesMeshAndSceneNode()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), 1e-4);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "revolveProfile");
        sketch.AddLine(new Vec2D(0, 0), new Vec2D(1, 0));
        sketch.AppendLine(new Vec2D(1, 1));
        sketch.AppendLine(new Vec2D(0, 1));
        sketch.AppendLine(new Vec2D(0, 0));

        var mesh = api.Revolve(sketch, 2 * Math.PI, 0.001, "revolve");

        AssertMesh(mesh);
        TestVisualization.Visualize(api, nameof(Revolve_Profile_GeneratesMeshAndSceneNode));
    }

    [Fact(Skip = "Edge naming is unstable in current branch; re-enable after edge graph stabilization.")]
    public void BlendEdges_CylinderUnion_GeneratesMeshAndSceneNode()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var a = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 0)), 0.5, 1, 0.0001, "a");
        var b = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 0)), 0.25, 2, 0.0001, "b");
        var c = api.Boolean(a, b, BooleanOp.Union, "c");

        var blended = api.Fillet(c, new List<string> { "[a-ExtrudeTop,b-Curve1]" }, 0.1, 0.0001, "blended");

        AssertMesh(blended);
        TestVisualization.Visualize(api, nameof(BlendEdges_CylinderUnion_GeneratesMeshAndSceneNode));
    }

    [Fact]
    public void Boolean_MultiCuboidUnion_GeneratesMeshAndSceneNode()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);

        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 3), "a");
        var b = api.CreateCuboid(new CoordinateSystem(new Vec3D(2, 0, 0)), new Vec3D(1, 1, 3), "b");
        var c = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 2, 0)), new Vec3D(1, 1, 3), "c");
        var d = api.CreateCuboid(new CoordinateSystem(new Vec3D(2, 2, 0)), new Vec3D(1, 1, 3), "d");

        var ab = api.Boolean(a, b, BooleanOp.Union, "ab");
        var cd = api.Boolean(c, d, BooleanOp.Union, "cd");
        var abcd = api.Boolean(ab, cd, BooleanOp.Union, "abcd");

        AssertMesh(abcd);
        TestVisualization.Visualize(api, nameof(Boolean_MultiCuboidUnion_GeneratesMeshAndSceneNode));
    }

    private static void AssertMesh(AnchorMesh mesh)
    {
        Assert.NotNull(mesh);
        Assert.NotNull(mesh.Mesh);
        Assert.NotEmpty(mesh.Mesh.Positions);
        Assert.NotEmpty(mesh.Mesh.Triangles);
    }
}
