using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class RoundedBladeCapTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void RoundedBladeProfilesRemainClosedOnTheAuthoritativeLattice()
    {
        var part = new GeoAPI(new Box3D(new Vec3D(-100,-100,-80),new Vec3D(100,100,260)),.025);
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (var (u,width) in new (double,double)[] { (0,.52),(.06,.90),(.10,1),(.25,1),(.5,1),(.75,1),(.93,.93),(1,.45) })
        {
            double theta=7*width/80, r=14.85, f=3.5;
            double alpha=Math.Asin(f/(r+f)), distance=Math.Sqrt((r+f)*(r+f)-f*f);
            Vec2D Polar(double radius,double angle) => new(radius*Math.Cos(angle),radius*Math.Sin(angle));
            var center=Polar(r+f,theta+alpha);
            var root=Polar(r,theta+alpha);
            var tangent=Polar(distance,theta);
            var direction=(tangent-center)+(root-center);
            var mid=center+direction*(f/direction.Length());
            var end=Polar(43,theta);
            Vec2D Mirror(Vec2D p) => new(p.X,-p.Y);
            double angle=(u==0 ? -3.04 : -360*25.0/104*u+.14)*Math.PI/180;
            var frame=new CoordinateSystem(new(0,0,25*u),new(Math.Cos(angle),Math.Sin(angle),0),new(-Math.Sin(angle),Math.Cos(angle),0),new(0,0,1));
            var sk=new PlotterSketcherCoordSys("blade"+sections.Count,frame);
            sk.AddArc(Mirror(root),Mirror(mid),Mirror(tangent));
            sk.AddLine(Mirror(tangent),Mirror(end));
            sk.AddArc(Mirror(end),new(43,0),end);
            sk.AddLine(end,tangent);
            sk.AddArc(tangent,mid,root);
            sk.AddLine(root,Mirror(root));
            sections.Add(sk);
        }
        var solid=part.Loft(sections,new LoftOptions { Style=LoftStyle.SmoothCatmullRom, CorrespondenceMode=LoftCorrespondenceMode.AnalyticFeatureAnchors },"blade",.025);
        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions,solid.Mesh.Triangles));
    }
}
