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
    [Theory]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(4, false)]
    [InlineData(4, true)]
    public void ClosedTangentArcsRemainSupported(int segments, bool reverseOrder)
    {
        var api=Api();
        // The profile is offset on a global plane; its plane origin is the
        // guide's center and cannot determine the correct starting segment.
        var profile=api.GetPlotterSketcher(DefaultPlanes.OriginYZ,"ring_profile");
        profile.AddCircle(new Vec2D(0,10),.5);
        var curves = Enumerable.Range(0, segments).Select(i => (Curve3D)new Arc3D(
            new Vec3D(0), new Vec3D(0,0,1), new Vec3D(1,0,0), 10,
            i * 2 * Math.PI / segments, 2 * Math.PI / segments, $"arc_{i}")).ToList();
        if (reverseOrder) curves.Reverse();
        var mesh=api.ExtrudeAlongCurveStrip(profile,curves,maxDeviation:.05,name:"ring");
        Assert.NotEmpty(mesh.Mesh.Triangles);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(mesh.Mesh);
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(mesh.Mesh.PrecisionPositions, mesh.Mesh.Triangles) > BigRationalHybrid.Zero);
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(mesh.Mesh.Positions, mesh.Mesh.Triangles), 35, 55);
        for (int i = 0; i < mesh.Mesh.Triangles.Count; i++)
        {
            var triangle = mesh.Mesh.Triangles[i];
            var center = (mesh.Mesh.Positions[triangle.A] + mesh.Mesh.Positions[triangle.B] + mesh.Mesh.Positions[triangle.C]) / 3;
            double angle = Math.Atan2(center.X, center.Z);
            if (angle < 0) angle += 2 * Math.PI;
            int guideIndex = Math.Min(segments - 1, (int)(angle / (2 * Math.PI / segments)));
            string groupName = mesh.groupIdToExtendedName[mesh.Mesh.TrianglesEx[i].GroupId];
            Assert.EndsWith($"-arc_{guideIndex}", groupName);
        }
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
