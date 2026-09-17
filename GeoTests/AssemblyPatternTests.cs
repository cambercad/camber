using Geo;
using GeoCore;
namespace GeoTests;
public class AssemblyPatternTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static GeoAPI Api() => new(new Box3D(new Vec3D(-100),new Vec3D(100)),.01);
    [Fact]
    public void LinearCopiesFollowSeedMotionThroughRecordedMates()
    {
        var api=Api();var mesh=api.CreateCube(CoordinateSystem.Default,1,"bolt");
        var a=api.GetAssembly("linear");a.SolveAfterEveryConstraint=false;
        var seed=a.AddPart(mesh,new Vec3D(0));
        var copies=a.PatternLinear(seed,3,new Vec3D(10,0,0));
        Assert.Same(seed,copies[0]);Assert.Equal(6,a.GetMateRecords().Count);
        Assert.All(copies,p=>Assert.Same(mesh,p.Mesh));
        a.FixPart(seed,new Vec3D(3,4,0),new Quaternion(0,0,Math.Sqrt(.5),Math.Sqrt(.5)));
        var solve=a.SolveConstraintsDetailed(); Assert.True(solve.Converged,$"{solve.Message}; SSE={solve.SumOfSquaredErrors}");
        for(int i=0;i<3;i++)
            Assert.InRange((copies[i].EvaluatePose().Position-new Vec3D(3,4+10*i,0)).Length(),0,1e-5);
    }
    [Fact]
    public void CircularPartialSweepIncludesBothEnds()
    {
        var api=Api();var mesh=api.CreateCube(CoordinateSystem.Default,1,"pin");
        var a=api.GetAssembly("circle");a.SolveAfterEveryConstraint=false;
        var seed=a.AddPart(mesh,new Vec3D(12,20,30));
        var axis=new CoordinateSystem(new Vec3D(10,20,30),new Vec3D(1,0,0),new Vec3D(0,1,0),new Vec3D(0,0,1));
        var copies=a.PatternCircular(seed,3,axis,Math.PI);a.FixPart(seed);
        var solve=a.SolveConstraintsDetailed(); Assert.True(solve.Converged,$"{solve.Message}; SSE={solve.SumOfSquaredErrors}");
        Assert.InRange((copies[1].EvaluatePose().Position-new Vec3D(10,22,30)).Length(),0,1e-6);
        Assert.InRange((copies[2].EvaluatePose().Position-new Vec3D(8,20,30)).Length(),0,1e-6);
    }
    [Fact]
    public void InvalidPatternDoesNotAddOccurrences()
    {
        var api=Api();var mesh=api.CreateCube(CoordinateSystem.Default,1,"pin");
        var a=api.GetAssembly("invalid");var seed=a.AddPart(mesh,new Vec3D(0));
        Assert.Throws<ArgumentOutOfRangeException>(()=>a.PatternLinear(seed,0,new Vec3D(1,0,0)));
        Assert.Single(a.GetParts());Assert.Empty(a.GetMateRecords());
    }
}
