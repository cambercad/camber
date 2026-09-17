using Geo;
using GeoCore;
using GeoSolver;
namespace GeoTests;
public class AssemblyDiagnosticsTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void ResidualsIdentifyTheUnsatisfiedMateWithoutMovingBodies()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-20),new Vec3D(20)),.01);
        var assembly=api.GetAssembly("diagnostics");assembly.SolveAfterEveryConstraint=false;
        var a=assembly.AddPart(api.CreateCube(new CoordinateSystem(new Vec3D(0)),1,"first"),new Vec3D(0));
        var b=assembly.AddPart(api.CreateCube(new CoordinateSystem(new Vec3D(0)),1,"second"),new Vec3D(0));
        assembly.FixPart(a);assembly.FixPart(b);
        assembly.SetCoincident(a.AddPointDatumAt(new Vec3D(0)),b.AddPointDatumAt(new Vec3D(0)));
        assembly.SetCoincident(a.AddPointDatumAt(new Vec3D(10,0,0)),b.AddPointDatumAt(new Vec3D(0)));
        var before=a.EvaluatePose();var report=assembly.GetMateResiduals();
        Assert.Equal(before,a.EvaluatePose());Assert.Equal(4,report.Count);
        Assert.All(report.Take(3),mate=>Assert.True(mate.Satisfied));
        Assert.False(report[3].Satisfied);Assert.Equal(3,report[3].Index);
        Assert.Equal("CoincidentPoints",report[3].Kind);
        Assert.Equal(10/assembly.CharacteristicLength,report[3].MaxResidual,12);
        Assert.Equal(NewtonSolver.ResidualTolerance(false),report[3].Tolerance);
        Assert.False(assembly.SolveConstraintsDetailed().Converged);
        Assert.False(assembly.GetMateResiduals()[3].Satisfied);
    }
    [Fact]
    public void RegularizedContactUsesItsOwnThresholdAlongsideEqualityMates()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-20),new Vec3D(20)),.01);
        var assembly=api.GetAssembly("contact_report");assembly.SolveAfterEveryConstraint=false;
        var a=assembly.AddPart(api.CreateCube(new CoordinateSystem(new Vec3D(0)),1,"body"),new Vec3D(0));
        assembly.FixPart(a);
        assembly.SetContact(a.AddPointDatumAt(new Vec3D(0)),a.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)));
        assembly.SetCoincident(a.AddPointDatumAt(new Vec3D(0)),a.AddPointDatumAt(new Vec3D(.00001,0,0)));
        var report=assembly.GetMateResiduals();
        Assert.Equal(NewtonSolver.ResidualTolerance(true),report[1].Tolerance);
        Assert.True(report[1].Satisfied); // Regularization contributes sqrt(epsilon).
        Assert.Equal(NewtonSolver.ResidualTolerance(false),report[2].Tolerance);
        Assert.False(report[2].Satisfied);
    }
    [Fact]
    public void ResidualSnapshotsAndEntityReferencesSurviveSubsequentSolves()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-20),new Vec3D(20)),.01);
        var assembly=api.GetAssembly("snapshots");assembly.SolveAfterEveryConstraint=false;
        var a=assembly.AddPart(api.CreateCube(new CoordinateSystem(new Vec3D(0)),1,"first"),new Vec3D(0));
        var b=assembly.AddPart(api.CreateCube(new CoordinateSystem(new Vec3D(0)),1,"second"),new Vec3D(0,0,4));
        assembly.FixPart(a);
        assembly.SetCoincidentOriented(a.AddPlaneDatum("first-ExtrudeTop"),b.AddPlaneDatum("second-ExtrudeBottom"),true);
        assembly.SetConcentric(a.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)),b.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)));
        assembly.SetParallel(a.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),b.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)));
        var before=assembly.GetMateResiduals();
        Assert.False(before[1].Satisfied);
        Assert.Equal(new[]{"first:first-ExtrudeTop","second:second-ExtrudeBottom"},before[1].Entities);
        var solved=assembly.SolveConstraintsDetailed();
        Assert.True(solved.Converged,solved.Message+"; "+string.Join(",",assembly.GetMateResiduals().Select(m=>m.Label+"="+m.MaxResidual)));
        Assert.True(assembly.GetMateResiduals()[1].Satisfied);
        Assert.False(before[1].Satisfied);
    }

}
