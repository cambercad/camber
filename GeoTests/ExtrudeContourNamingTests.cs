using Geo;
using GeoCore;

namespace GeoTests;

public class ExtrudeContourNamingTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SideNamesFollowSketchSegmentsAfterWindingNormalization(bool clockwise)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20),new Vec3D(20)),.01);
        var sketch = api.GetPlotterSketcher(new CoordinateSystem(new Vec3D(0)),"Profile");
        var points = new List<Vec2D> { new(0,0),new(9,0),new(7,5),new(0,3) };
        if (clockwise) points.Reverse();
        for (int i=0;i<points.Count;i++) sketch.AddLine(points[i],points[(i+1)%points.Count]);
        var solid = api.Extrude(sketch,4,name:"Block");
        var groups = solid.Mesh.GetTriangleGroups();
        for (int edge=0;edge<points.Count;edge++)
        {
            int group = solid.groupIdToExtendedName.Single(kv=>kv.Value=="Block-Line"+(edge+1)).Key;
            var a=points[edge]; var b=points[(edge+1)%points.Count];
            var triangles = solid.Mesh.Triangles.Where((_,i)=>groups[i]==group).ToList();
            Assert.NotEmpty(triangles);
            foreach(var triangle in triangles)
            foreach(int index in new[]{triangle.A,triangle.B,triangle.C})
            {
                var p=solid.Mesh.Positions[index];
                double cross=(p.X-a.X)*(b.Y-a.Y)-(p.Y-a.Y)*(b.X-a.X);
                Assert.True(Math.Abs(cross)<1e-6,$"Line {edge+1} assigned to wrong wall at {p}");
            }
        }
    }
}
