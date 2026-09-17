using Geo;
using GeoCore;
namespace GeoTests;
public class SubassemblyPatternTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PatternsPreserveNestedRevoluteMatesAndIndependentBodies(bool circular)
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-100),new Vec3D(100)),.01);
        var housingMesh=api.CreateCube(CoordinateSystem.Default,2,"housing");
        var rotorMesh=api.CreateCube(CoordinateSystem.Default,1,"rotor");
        var housing=api.GetAssembly("housing_group");housing.SolveAfterEveryConstraint=false;
        var fixedPart=housing.AddPart(housingMesh,new Vec3D(0));housing.FixPart(fixedPart);
        var mechanism=api.GetAssembly("mechanism");mechanism.SolveAfterEveryConstraint=false;
        var housingOccurrence=mechanism.AddSubAssembly(housing,new Vec3D(0));mechanism.FixSubAssembly(housingOccurrence);
        var rotor=mechanism.AddPart(rotorMesh,new Vec3D(0));
        mechanism.SetConcentric(fixedPart.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)),rotor.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)));
        mechanism.SetCoincidentOriented(fixedPart.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)),rotor.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)),false);
        Assert.True(mechanism.SolveConstraintsDetailed().Converged);
        var parent=api.GetAssembly("parent");parent.SolveAfterEveryConstraint=false;
        var seed=parent.AddSubAssembly(mechanism,new Vec3D(10,0,0));
        var copies=circular?parent.PatternCircular(seed,3,CoordinateSystem.Default,Math.PI):parent.PatternLinear(seed,3,new Vec3D(10,0,0));
        Assert.Same(seed,copies[0]);Assert.Same(parent,mechanism.Parent);
        Assert.NotSame(mechanism,copies[1].Child);
        Assert.Single(copies[1].GetSubAssemblies());
        Assert.Equal(mechanism.GetMateRecords().Select(m=>m.Kind),copies[1].Child.GetMateRecords().Select(m=>m.Kind));
        parent.FixSubAssembly(seed);Assert.True(parent.SolveConstraintsDetailed().Converged);
        var expected=circular?new Vec3D(-10,0,0):new Vec3D(30,0,0);
        Assert.InRange((copies[2].EvaluatePose().Position-expected).Length(),0,1e-6);
        var clone=copies[1].Child;var clonedRotor=clone.GetParts()[0];var clonedHousing=clone.GetSubAssemblies()[0].GetParts()[0];
        clone.SetAngle(clonedHousing.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),clonedRotor.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),.3);
        Assert.True(clone.SolveConstraintsDetailed().Converged);
        var direction=TransformMath.TransformDirection(clonedRotor.EvaluatePose(),new Vec3D(1,0,0));
        Assert.InRange(Math.Abs(direction.X-Math.Cos(.3)),0,1e-7);
        Assert.Equal(new Vec3D(1,0,0),TransformMath.TransformDirection(rotor.EvaluatePose(),new Vec3D(1,0,0)));
    }
}
