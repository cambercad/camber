using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class RectangularDuctLoftTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void NestedRectangleToCircleLoftsMakeAThroughPassageWithCoplanarEnds()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50), new Vec3D(50)), .01);
        AnchorMesh Loft(string name, double x, double y, double radius)
        {
            var rectangle = new PlotterSketcherCoordSys(name + "Rectangle", CoordinateSystem.Default);
            rectangle.AddRectangleFromCorners(new Vec2D(-x, -y), new Vec2D(x, y));
            var frame = new CoordinateSystem(new Vec3D(0, 0, 45), new Vec3D(1, 0, 0),
                new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
            var circle = new PlotterSketcherCoordSys(name + "Circle", frame);
            circle.AddCircle(new Vec2D(0, 0), radius);
            return api.Loft([rectangle, circle], new LoftOptions { AlignmentMode = LoftAlignmentMode.MinimumTwist }, name, .01);
        }
        var outer = Loft("outer", 25, 18, 15);
        var inner = Loft("inner", 23, 16, 13);
        foreach (var mesh in new[] { outer, inner })
        {
            MeshTestHelpers.AssertValidMesh(mesh.Mesh.Positions, mesh.Mesh.Triangles);
            Assert.True(MeshAnalysis.IsWatertightMesh(mesh.Mesh.Positions, mesh.Mesh.Triangles));
        }
        var wall = api.Boolean(outer, inner, BooleanOp.Subtract, "wall");
        Assert.Same(wall, api.GetTopMesh());
        MeshTestHelpers.AssertValidMesh(wall.Mesh.Positions, wall.Mesh.Triangles);
        Assert.True(MeshAnalysis.IsWatertightMesh(wall.Mesh.Positions, wall.Mesh.Triangles));
    }
}

