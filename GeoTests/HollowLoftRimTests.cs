using Curves;
using CSG;
using Geo;
using GeoCore;
namespace GeoTests;
public class HollowLoftRimTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);
    [Theory]
    [InlineData(0,false)]
    [InlineData(2,false)]
    [InlineData(3,false)]
    [InlineData(0,true)]
    [InlineData(2,true)]
    [InlineData(3,true)]
    public void CurvedEndpointSupportRetainsItsShape(int index,bool chamfer)
    {
        var p = new GeoAPI(new Box3D(new Vec3D(-50,-50,-15),new Vec3D(350,300,130)), .04);
        (double x,double y,double z,double w,double h,double a)[] stations = [(0,0,-1,32,26,0),(0,0,12,32,26,0),(7,0,38,26,22,25),(-6,4,64,22,26,55),(0,6,90,30,24,85)];
        AnchorMesh Loft(string name,bool inner)
        {
            var sections = new List<PlotterSketcherCoordSys>();
            for(int i=0;i<stations.Length;i++)
            {
                var (x,y,z,w,h,a)=stations[i];
                if(inner) {w-=7;h-=7;if(i==0) z-=10;if(i==4)z+=1;}
                double angle=a*Math.PI/180;
                var frame=new CoordinateSystem(new Vec3D(x,y,z),new Vec3D(Math.Cos(angle),Math.Sin(angle),0),new Vec3D(-Math.Sin(angle),Math.Cos(angle),0),new Vec3D(0,0,1));
                var s=new PlotterSketcherCoordSys(name+i,frame);
                Vec2D[] points=[new(-w/2,-h/2),new(w/2,-h/2),new(w/2,h/2),new(-w/2,h/2)];
                string[] labels=["bottom","right","top","left"];
                for(int j=0;j<4;j++)s.AddLine(points[j],points[(j+1)%4]).Name=labels[j];
                sections.Add(s);
            }
            return p.Loft(sections,new LoftOptions {CorrespondenceMode=LoftCorrespondenceMode.MatchingVertices,FirstCurves=Enumerable.Repeat("bottom",5).ToArray()},name,.04);
        }
        var outer=Loft("duct",false);var inner=Loft("passage",true);
        var body=p.Boolean(outer,inner,BooleanOp.Difference,"open_duct");
        var edges=new[]{"bottom","right","top","left"}.Select(s=>$"[passage-Side-{s},duct-EndCap]").ToList();
        double before=MeshAnalysis.ComputeSignedMeshVolume(body.Mesh.Positions,body.Mesh.Triangles);
        var result=chamfer?p.Chamfer(body,new List<string>{edges[index]},.6,.04,"rim")
            :p.Fillet(body,new List<string>{edges[index]},.6,.04,"rim");
        double after=MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions,result.Mesh.Triangles);
        Assert.InRange(after,before*.9,before-.1);
        MeshTestHelpers.AssertValidMesh(result.Mesh.Positions,result.Mesh.Triangles);
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.Positions,result.Mesh.Triangles));
    }
}
