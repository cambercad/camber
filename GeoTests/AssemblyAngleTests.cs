using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class AssemblyAngleTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Theory]
    [InlineData(0,0.4,0)]
    [InlineData(0,0.4,1)]
    [InlineData(0,0.4,2)]
    [InlineData(0,2.4,0)]
    [InlineData(0,2.4,1)]
    [InlineData(3.141592653589793,0.4,0)]
    [InlineData(0,3.141592653589793,0)]
    [InlineData(3.141592653589793,0,0)]
    [InlineData(0,1.5707963267948966,0)]
    public void AngularMateEscapesCollinearStartAndPreservesHemisphere(double start,double target,int hingeY)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50),new Vec3D(50)),.01);
        var a = api.CreateCube(CoordinateSystem.Default,2,"fixed");
        var b = api.CreateCube(CoordinateSystem.Default,2,"moving");
        var asm = api.GetAssembly("angle");
        asm.SolveAfterEveryConstraint = false;
        var axis = (hingeY == 0 ? new Vec3D(0,0,1) : hingeY == 1 ? new Vec3D(0,1,0) : new Vec3D(0,1,1)).Normalized();
        var fixedBody = asm.AddPart(a,new Vec3D(0));
        var moving = asm.AddPart(b,new Vec3D(0),new Quaternion(
            axis.X*Math.Sin(start/2),axis.Y*Math.Sin(start/2),axis.Z*Math.Sin(start/2),Math.Cos(start/2)));
        asm.FixPart(fixedBody);
        asm.SetConcentric(fixedBody.AddAxisDatumAt(new Vec3D(0),axis),moving.AddAxisDatumAt(new Vec3D(0),axis));
        asm.SetCoincidentOriented(fixedBody.AddPlaneDatumAt(new Vec3D(0),axis),moving.AddPlaneDatumAt(new Vec3D(0),axis),false);
        asm.SetAngle(fixedBody.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),
                     moving.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),target);
        var result = asm.SolveConstraintsDetailed();
        Assert.True(result.Converged,$"{result.Message}; SSE={result.SumOfSquaredErrors}");
        var direction = TransformMath.TransformDirection(moving.EvaluatePose(),new Vec3D(1,0,0)).Normalized();
        Assert.InRange(Math.Abs(direction.X-Math.Cos(target)),0,1e-5);
        var bearing = TransformMath.TransformDirection(moving.EvaluatePose(),axis).Normalized();
        Assert.InRange(Math.Abs(Vec3DOps.Dot(bearing,axis)-1),0,1e-5);
    }
    [Fact]
    public void FixedSupplementaryAngleIsRejectedWithoutMovingBodies()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50),new Vec3D(50)),.01);
        var asm = api.GetAssembly("fixed_angle");
        asm.SolveAfterEveryConstraint = false;
        var a = asm.AddPart(api.CreateCube(CoordinateSystem.Default,2,"a"),new Vec3D(0));
        var b = asm.AddPart(api.CreateCube(CoordinateSystem.Default,2,"b"),new Vec3D(5,0,0),
            new Quaternion(0,0,Math.Sin(Math.PI/3),Math.Cos(Math.PI/3)));
        asm.FixPart(a);
        asm.FixPart(b);
        asm.SetAngle(a.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),
                     b.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),Math.PI/3);
        Assert.False(asm.SolveConstraintsDetailed().Converged);
        var direction = TransformMath.TransformDirection(b.EvaluatePose(),new Vec3D(1,0,0));
        Assert.InRange(Math.Abs(direction.X+.5),0,1e-12);
    }

}
