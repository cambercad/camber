using Geo;
using GeoCore;
namespace GeoTests;
public class AssemblyConvergenceAccuracyTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void ConsistentRedundantMatesDoNotAcceptAnUncorrectedSmallAngularError()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-2000),new Vec3D(2000)),.01);
        var a=api.CreateCube(CoordinateSystem.Default,1000,"fixed");
        var b=api.CreateCube(CoordinateSystem.Default,1000,"moving");
        var asm=api.GetAssembly("precision");
        asm.SolveAfterEveryConstraint=false;
        var ground=asm.AddPart(a,new Vec3D(0));
        var body=asm.AddPart(b,new Vec3D(0),new Quaternion(0,0,Math.Sin(2.5e-7),Math.Cos(2.5e-7)));
        asm.FixPart(ground);
        for(int i=0;i<3;i++)
            asm.SetParallel(ground.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),
                            body.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)));
        var result=asm.SolveConstraintsDetailed();
        Assert.True(result.Converged,result.Message);
        var direction=TransformMath.TransformDirection(body.EvaluatePose(),new Vec3D(1,0,0));
        Assert.InRange(Math.Abs(direction.Y),0,1e-9);
    }
}
