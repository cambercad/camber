using Curves;
using Geo;
using GeoCore;
namespace GeoTests;
public class SweepGuideTangencyTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static GeoAPI Api()=>new(new Box3D(new Vec3D(-30),new Vec3D(30)),.001);
    [Theory]
    [InlineData(10,10)]
    [InlineData(.001,20)]
    public void KinkIsRejectedBeforeRegisteringAnIncorrectSweep(double lateral, double finalZ)
    {
        var api=Api();var profile=api.GetPlotterSketcher(DefaultPlanes.OriginXY,"profile");
        profile.AddRectangle(new Vec2D(-.5,-.5),1,1);
        var curves=new List<Curve3D>{
            new Line3D(new Vec3D(0),new Vec3D(0,0,10),"incoming"),
            new Line3D(new Vec3D(0,0,10),new Vec3D(lateral,0,finalZ),"outgoing")};
        var error=Assert.Throws<ArgumentException>(()=>api.ExtrudeAlongCurveStrip(profile,curves,name:"invalid"));
        Assert.Contains("incoming",error.Message);Assert.Contains("outgoing",error.Message);
        Assert.Contains("not tangent",error.Message);
        Assert.DoesNotContain(api.GetMeshes(),mesh=>mesh.Name=="invalid");
    }
    [Fact]
    public void ReversedTangentSegmentsPreserveAnObliqueStartingProfile()
    {
        var api=Api();
        var profile=api.GetPlotterSketcher(DefaultPlanes.OriginXY,"profile");
        profile.AddRectangle(new Vec2D(-.5,-.5),1,1);
        // The profile normal is intentionally oblique to the guide. Only joins
        // must be tangent; validation must not silently rotate this profile.
        var curves=new List<Curve3D>{
            new Line3D(new Vec3D(0),new Vec3D(5,0,10),"first"),
            new Line3D(new Vec3D(10,0,20),new Vec3D(5,0,10),"backwards")};
        var mesh=api.ExtrudeAlongCurveStrip(profile,curves,name:"oblique");
        var a=api.GetAssembly("check").AddPart(mesh,new Vec3D(0));
        var cap=a.AddPlaneDatum("oblique-ExtrudeTop");
        Assert.True(Math.Abs(Vec3DOps.Dot(cap.LocalNormal,new Vec3D(0,0,1)))>1-1e-10);
    }
    [Fact]
    public void ClosedTangentArcsRemainSupported()
    {
        var api=Api();
        var profile=api.GetPlotterSketcher(DefaultPlanes.OriginYZ,"ring_profile");
        profile.AddCircle(new Vec2D(0,10),.5);
        var curves=new List<Curve3D>{
            new Arc3D(new Vec3D(0),new Vec3D(0,0,1),new Vec3D(1,0,0),10,0,Math.PI,"half_a"),
            new Arc3D(new Vec3D(0),new Vec3D(0,0,1),new Vec3D(1,0,0),10,Math.PI,Math.PI,"half_b")};
        var mesh=api.ExtrudeAlongCurveStrip(profile,curves,maxDeviation:.05,name:"ring");
        Assert.NotEmpty(mesh.Mesh.Triangles);
    }
    [Fact]
    public void ClosingSeamIsValidatedEvenWhenInternalJoinIsTangent()
    {
        var api=Api();var profile=api.GetPlotterSketcher(DefaultPlanes.OriginXY,"profile");
        profile.AddCircle(new Vec2D(0),.5);
        var curves=new List<Curve3D>{
            new Line3D(new Vec3D(0),new Vec3D(0,0,10),"straight"),
            new CubicHermiteSpline3D(new[]{new Vec3D(0,0,10),new Vec3D(5,0,5),new Vec3D(0)},
                new Vec3D(0,0,1),new Vec3D(1,0,0),"return")};
        var error=Assert.Throws<ArgumentException>(()=>api.ExtrudeAlongCurveStrip(profile,curves,name:"invalid_closed"));
        Assert.Contains("return",error.Message);Assert.Contains("straight",error.Message);
        Assert.DoesNotContain(api.GetMeshes(),mesh=>mesh.Name=="invalid_closed");
    }
    [Fact]
    public void TangentLineArcLineHasCapNormalAlongFinalGuide()
    {
        var api=Api();var profile=api.GetPlotterSketcher(DefaultPlanes.OriginXY,"profile");
        profile.AddCircle(new Vec2D(0),.5);
        var curves=new List<Curve3D>{
            new Line3D(new Vec3D(0),new Vec3D(0,0,10),"entry"),
            new Arc3D(new Vec3D(10,0,10),new Vec3D(-1,0,0),new Vec3D(0,0,1),10,0,Math.PI/2,"bend"),
            new Line3D(new Vec3D(10,0,20),new Vec3D(20,0,20),"exit")};
        var mesh=api.ExtrudeAlongCurveStrip(profile,curves,maxDeviation:.02,name:"elbow");
        var body=api.GetAssembly("check").AddPart(mesh,new Vec3D(0));
        var cap=body.AddPlaneDatum("elbow-ExtrudeTop");
        Assert.True(Vec3DOps.Dot(cap.LocalNormal,new Vec3D(1,0,0))>1-1e-10);
        Assert.Equal(api.Converter.Convert(api.Converter.Convert(new Vec3D(20,0,20))).X,cap.LocalOrigin.X,10);
    }

}
