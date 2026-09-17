using Geo;
using GeoCore;
using Curves;

namespace GeoTests;

public class FilletTrimRegressionTests
{
    [Fact]
    public void BladeVerticalRoundsRemainWatertight()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-420,-300,-50), new Vec3D(1420,300,1120)), .2);
        var frame = new CoordinateSystem(new Vec3D(1033,207,878), new Vec3D(1,0,0), new Vec3D(0,0,1), new Vec3D(0,-1,0));
        var points = new Vec2D[] { new(-6,5),new(4,7),new(8,2),new(12,-15),new(14,-42),new(9,-69),new(2,-87),new(-4,-91),new(-7,-88),new(-1,-73),new(4,-47),new(4,-22),new(-6,-2) };
        var sketch = new PlotterSketcherCoordSys("profile", frame, points[0]);
        for (int i=1;i<=points.Length;i++) sketch.AppendLine(points[i%points.Length].X, points[i%points.Length].Y);
        var blank = api.Extrude(sketch,14,name:"blade");
        var edges = Enumerable.Range(1,points.Length).Select(i=>$"[blade-Line{(i==1?points.Length:i-1)},blade-Line{i}]").ToList();
        var rounded = api.Fillet(blank,edges,1,.02,"rounded");
        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions,rounded.Mesh.Triangles);
    }
    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(.7, false)]
    public void CapsuleBottomRoundRemainsWatertight(double angle, bool selectOneEdge)
    {
        var bounds = angle == 0
            ? new Box3D(new Vec3D(-20,-40,-25), new Vec3D(250,40,40))
            : new Box3D(new Vec3D(-250), new Vec3D(250));
        var api = new GeoAPI(bounds, .035);
        var frame = new CoordinateSystem(new Vec3D(63,0,2.1),
            new Vec3D(Math.Cos(angle), Math.Sin(angle), 0),
            new Vec3D(-Math.Sin(angle), Math.Cos(angle), 0), new Vec3D(0,0,1));
        var sketch = new PlotterSketcherCoordSys("profile", frame, new Vec2D(0,4.8));
        sketch.AppendLine(109,4.8);
        sketch.AppendArc(new Vec2D(113.8,0), new Vec2D(109,-4.8));
        sketch.AppendLine(0,-4.8);
        sketch.AppendArc(new Vec2D(-4.8,0), new Vec2D(0,4.8));
        var blank = api.Extrude(sketch,5,name:"panel_tool");
        var edges = new List<string>();
        foreach (var name in blank.groupIdToExtendedName.Values)
            if (!name.Contains("Extrude")) edges.Add($"[panel_tool-ExtrudeBottom,{name}]");
        var sourceGroups = blank.Mesh.GetTriangleGroups().ToArray();
        var sourceNames = blank.groupIdToExtendedName.Values.ToArray();
        var rounded = api.Fillet(blank,selectOneEdge ? edges.Take(1).ToList() : edges,.65,.035,"rounded");
        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions,rounded.Mesh.Triangles);
        Assert.Equal(sourceGroups, blank.Mesh.GetTriangleGroups());
        foreach (string name in sourceNames)
        {
            Assert.Contains(name, rounded.groupIdToExtendedName.Values);
            Assert.Contains(rounded.extendedNameToGroupId[name], rounded.Mesh.GetTriangleGroups());
        }
        double Volume(AnchorMesh solid) => Math.Abs(solid.Mesh.Triangles.Sum(t =>
            Vec3DOps.Dot(solid.Mesh.Positions[t.A], Vec3DOps.Cross(solid.Mesh.Positions[t.B], solid.Mesh.Positions[t.C])))) / 6;
        Assert.InRange(Volume(blank) - Volume(rounded), 10, 35);
    }
    [Fact]
    public void TangentSideChainDoesNotEraseTeardropSharpTip()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .01);
        var sketch = new PlotterSketcherCoordSys("profile", CoordinateSystem.Default, new Vec2D(0,0));
        sketch.AppendLine(4,2);
        sketch.AppendArc(new Vec2D(5 + Math.Sqrt(5),0), new Vec2D(4,-2));
        sketch.AppendLine(0,0);
        var blank = api.Extrude(sketch,5,name:"drop");
        var rounded = api.Fillet(blank,new List<string> { "[drop-Line1,drop-Line2]" },.2,.01,"rounded");
        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions,rounded.Mesh.Triangles);
        Assert.Contains("drop-Arc1", rounded.groupIdToExtendedName.Values);
    }
}
