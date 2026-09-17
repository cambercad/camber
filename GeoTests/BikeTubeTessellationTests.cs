using Curves;
using Geo;
using GeoCore;
using Xunit.Abstractions;

namespace GeoTests;

public class BikeTubeTessellationTests(ITestOutputHelper output)
{
    [Theory]
    [InlineData(.2)]
    [InlineData(.05)]
    public void DownTubeLoftRespectsInteriorDeviationAndSmoothNormals(double deviation)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-420,-300,-50),new Vec3D(1420,300,1120)),deviation);
        Vec3D[] centers = [new(404,0,267),new(475,0,360),new(685,0,585),new(831,0,725)];
        double[] depths = [23,34,32,27], widths = [26,29,25,24];
        var sections = new List<PlotterSketcherCoordSys>();
        for(int i=0;i<centers.Length;i++)
        {
            var axis=(centers[Math.Min(i+1,centers.Length-1)]-centers[Math.Max(i-1,0)]).Normalized();
            var x=Vec3DOps.Cross(new Vec3D(0,1,0),axis).Normalized();
            var y=Vec3DOps.Cross(axis,x).Normalized();
            var sk=new PlotterSketcherCoordSys("station"+i,new CoordinateSystem(centers[i],x,y,axis));
            sk.AddEllipse(new Vec2D(0),new Vec2D(depths[i],0),widths[i]);
            sections.Add(sk);
        }
        var body=api.Loft(sections,new LoftOptions(),"tube",deviation);
        LoftTriangleDeviationTests.AssertTriangleInteriors(body,"tube-Side",deviation,api.Converter.SmallestUnit());
        Assert.True(body.TryGetSurface("tube-Side",out var face));
        double minimumDot=1;
        foreach(int i in face.Triangles.SelectMany(t=>new[]{t.A,t.B,t.C}).Distinct())
        {
            var uv=face.Uv[i];
            var expected=face.NurbsSurface.EvaluateNormal(uv.X,uv.Y).Normalized();
            minimumDot=Math.Min(minimumDot,Vec3DOps.Dot(expected,face.Normals[i]));
        }
        output.WriteLine($"deviation={deviation}, triangles={body.Mesh.Triangles.Count}, minimum normal alignment={minimumDot}");
        Assert.True(minimumDot>.99999);
    }
}
