using Geo;
using GeoCore;
using GeoSolver.Sketcher;
namespace GeoTests;
public class AssemblyFacePlaneTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static GeoAPI Api() => new(new Box3D(new Vec3D(-420,-300,-50),new Vec3D(1420,300,1120)),.2);
    private static void AssertOnOrientedFace(GeoAPI api,AnchorMesh mesh,AssemblyPart body,string name)
    {
        var datum=body.AddPlaneDatum(name);
        Assert.True(mesh.TryGetSurface(name,out var surface));
        foreach(var tri in surface.Triangles)
        {
            var a=api.Converter.Convert(surface.PointsPrecise[tri.A]);var b=api.Converter.Convert(surface.PointsPrecise[tri.B]);var c=api.Converter.Convert(surface.PointsPrecise[tri.C]);
            var normal=Vec3DOps.Cross(b-a,c-a);
            if(normal.LengthSquared()==0)continue;
            normal.Normalize();
            Assert.True(Vec3DOps.Dot(normal,datum.LocalNormal)>.999999,name);
            Assert.InRange(Math.Abs(Vec3DOps.Dot(datum.LocalOrigin-a,normal)),0,1e-10);
        }
    }
    [Fact]
    public void ExtrudeCapDatumsUseActualOutwardFacesWithoutMutatingAnalyticMetadata()
    {
        var api=Api();var solid=api.CreateCuboid(new Vec3D(404,-46,267),new Vec3D(410,-44,273),"cap");
        var assembly=api.GetAssembly("planes");var body=assembly.AddPart(solid,new Vec3D(0));
        foreach(string name in new[]{"cap-ExtrudeBottom","cap-ExtrudeTop"})
        {
            var metadata=solid.surfaceMetaData[name].PlaneParams;
            var normal=metadata.Normal;var origin=metadata.Origin;
            AssertOnOrientedFace(api,solid,body,name);
            Assert.Equal(normal,metadata.Normal);Assert.Equal(origin,metadata.Origin);
        }
    }
    [Fact]
    public void RevolveShoulderDatumsFollowBothActualOutwardNormals()
    {
        var api=Api();var sk=api.GetPlotterSketcher(DefaultPlanes.OriginXY,"profile");
        sk.AddLine(new Vec2D(0,1),new Vec2D(0,2));sk.AddLine(new Vec2D(0,2),new Vec2D(3,2));
        sk.AddLine(new Vec2D(3,2),new Vec2D(3,1));sk.AddLine(new Vec2D(3,1),new Vec2D(0,1));
        var solid=api.Revolve(sk,2*Math.PI,.01,"shoulder");
        var assembly=api.GetAssembly("shoulders");var body=assembly.AddPart(solid,new Vec3D(0));
        AssertOnOrientedFace(api,solid,body,"shoulder-Line1");
        AssertOnOrientedFace(api,solid,body,"shoulder-Line3");
    }
    [Fact]
    public void OpposedActualBearingFacesSeatWithoutRotationOrOverlap()
    {
        var api=Api();
        var first=api.CreateCuboid(new Vec3D(0),new Vec3D(10,10,2),"first");
        var second=api.CreateCuboid(new Vec3D(0),new Vec3D(10,10,2),"second");
        var assembly=api.GetAssembly("bearing_faces");assembly.SolveAfterEveryConstraint=false;
        var a=assembly.AddPart(first,new Vec3D(0));var b=assembly.AddPart(second,new Vec3D(0,0,2));
        assembly.FixPart(a);
        var top=a.AddPlaneDatum("first-ExtrudeTop");var bottom=b.AddPlaneDatum("second-ExtrudeBottom");
        double expectedZ=top.LocalOrigin.Z-bottom.LocalOrigin.Z;
        assembly.SetConcentric(a.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)),
            b.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)));
        assembly.SetCoincidentOriented(top,bottom,true);
        assembly.SetParallel(a.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),
            b.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)));
        assembly.SolveConstraints();
        var pose=b.EvaluatePose();
        Assert.InRange(Math.Abs(pose.Position.Z-expectedZ),0,1e-8);
        Assert.InRange(Math.Abs(pose.Position.X)+Math.Abs(pose.Position.Y),0,1e-8);
        // A 100 mm² contact face times the 1e-8 mm pose tolerance.
        Assert.Empty(assembly.Interferences(1e-6));
    }

    [Fact]
    public void FaceDatumsStayLocalAfterPosedAndRepeatedOccurrencesSolve()
    {
        var api=Api();var solid=api.CreateCuboid(new Vec3D(0),new Vec3D(10,8,2),"shared");
        var assembly=api.GetAssembly("posed");assembly.SolveAfterEveryConstraint=false;
        var first=assembly.AddPart(solid,new Vec3D(50,30,20),new Quaternion(0,Math.Sin(.35),0,Math.Cos(.35)));
        var original=first.AddPlaneDatum("shared-ExtrudeTop");
        assembly.FixPart(first);assembly.SolveConstraints();
        var after=first.AddPlaneDatum("shared-ExtrudeTop");
        Assert.Equal(original.LocalOrigin,after.LocalOrigin);
        Assert.Equal(original.LocalNormal,after.LocalNormal);
        var second=assembly.AddPart(solid,new Vec3D(-20,10,50),new Quaternion(Math.Sin(.2),0,0,Math.Cos(.2)));
        assembly.FixPart(second);assembly.SolveConstraints();
        foreach(var body in new[]{first,second})
        {
            var datum=body.AddPlaneDatum("shared-ExtrudeTop");
            Assert.Equal(original.LocalOrigin,datum.LocalOrigin);
            Assert.Equal(original.LocalNormal,datum.LocalNormal);
        }
    }

}
