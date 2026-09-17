using Geo;
using GeoCore;
using GeoSolver;

namespace GeoTests;

public class UnderdeterminedAssemblyRegularizationTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    private sealed class WithFreeParametersAndEmptyRow(IEquationContainer inner, double constantResidual = 0) : IEquationContainer
    {
        public Param[] Free { get; } = [new Param(7), new Param(-3), new Param(11)];
        public int NumParameters => inner.NumParameters + Free.Length;
        public int NumEquations => inner.NumEquations + 1;
        public double Evaluate(int i) => i == inner.NumEquations ? constantResidual : inner.Evaluate(i);
        public double EvaluatJacobian(int i,int j) => i == inner.NumEquations || j >= inner.NumParameters ? 0 : inner.EvaluatJacobian(i,j);
        public Param GetParameter(int i) => i >= inner.NumParameters ? Free[i-inner.NumParameters] : inner.GetParameter(i);
        public int NumNonZerosInRow(int i) => i == inner.NumEquations ? 0 : inner.NumNonZerosInRow(i);
        public void EvaluateJacobianRow(int row,ref int indexer,IList<double> values,IList<int> columns)
        {
            if (row < inner.NumEquations) inner.EvaluateJacobianRow(row,ref indexer,values,columns);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RedundantRowsAndAnEmptyRowPreserveUnconstrainedParameters(bool sparse)
    {
        var x=new Param(.1); var y=new Param(.3);
        var residual=Expr.Parameter(x)+Expr.Parameter(y)-Expr.Constant(1);
        var equations=new WithFreeParametersAndEmptyRow(new EquationContainer([residual,residual],applySketchReductions:false));
        var result=NewtonSolver.NewtonSolveDetailed(equations,forceSparse:sparse);
        Assert.Equal(5,result.NumParameters); Assert.Equal(3,result.NumEquations);
        Assert.True(result.Converged,result.Message);
        Assert.InRange(x.Value,.4-1e-8,.4+1e-8);
        Assert.InRange(y.Value,.6-1e-8,.6+1e-8);
        Assert.Equal(new[]{7.0,-3.0,11.0},equations.Free.Select(p=>p.Value));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void InconsistentConstantRowDoesNotBecomeAReportedSolution(bool sparse)
    {
        var x=new Param(.1); var y=new Param(.3);
        var residual=Expr.Parameter(x)+Expr.Parameter(y)-Expr.Constant(1);
        var equations=new WithFreeParametersAndEmptyRow(new EquationContainer([residual,residual],applySketchReductions:false),1);
        var result=NewtonSolver.NewtonSolveDetailed(equations,forceSparse:sparse,maxNewtonIterations:8);
        Assert.False(result.Converged);
        Assert.Equal(new[]{7.0,-3.0,11.0},equations.Free.Select(p=>p.Value));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(14)]
    public void SinglePlaneMatesPreserveSlidingAndYawInDenseAndSparseAssemblies(int count)
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-20),new Vec3D(100)),.01);
        var solid=api.CreateCube(CoordinateSystem.Default,1,"body");
        var assembly=api.GetAssembly("free_plane_mates");
        assembly.SolveAfterEveryConstraint=false;
        var ground=assembly.AddPart(solid,new Vec3D(0)); assembly.FixPart(ground);
        var bodies=new List<AssemblyPart>(); const double yaw=.23;
        for(int i=0;i<count;i++)
        {
            var body=assembly.AddPart(solid,new Vec3D(3+i*2,-2,4),
                new Quaternion(0,0,Math.Sin(yaw/2),Math.Cos(yaw/2)));
            bodies.Add(body);
            assembly.SetCoincidentOriented(ground.AddPlaneDatum("body-ExtrudeTop"),
                body.AddPlaneDatum("body-ExtrudeBottom"),true);
        }
        var result=assembly.SolveConstraintsDetailed();
        Assert.Equal(count*6,result.NumParameters); Assert.Equal(count*4,result.NumEquations);
        Assert.True(result.Converged,result.Message);
        Assert.Equal(count+1,assembly.GetMateResiduals().Count); // Only ground and authored plane mates.
        Assert.All(assembly.GetMateResiduals(),mate=>Assert.True(mate.Satisfied));
        for(int i=0;i<count;i++)
        {
            var pose=assembly.WorldPoseOf(bodies[i]);
            var origin=TransformMath.TransformPoint(pose,new Vec3D(0));
            double seatedZ=ground.AddPlaneDatum("body-ExtrudeTop").LocalOrigin.Z
                -bodies[i].AddPlaneDatum("body-ExtrudeBottom").LocalOrigin.Z;
            // The distance equation uses the moving normal. Its transient
            // angular residual acts through the offset datum lever arm, so
            // world-coordinate accuracy includes that arm, not only body size.
            double leverArm=new Vec3D(3+i*2,-2,4).Length();
            double positionTolerance=(assembly.CharacteristicLength+leverArm)*NewtonSolver.ResidualTolerance(false);
            Assert.InRange(Math.Abs(origin.X-(3+i*2)),0,positionTolerance);
            Assert.InRange(Math.Abs(origin.Y+2),0,positionTolerance);
            Assert.InRange(Math.Abs(origin.Z-seatedZ),0,positionTolerance);
            var direction=TransformMath.TransformDirection(pose,new Vec3D(1,0,0));
            Assert.True((direction-new Vec3D(Math.Cos(yaw),Math.Sin(yaw),0)).Length()<1e-7);
            // A different in-plane position and yaw is still a solution. A
            // hidden anchor would pull this pose back on the next solve.
            bodies[i].Transform.SetPose(new Transform(new Vec3D(5+i*2,-5,seatedZ),
                new Quaternion(0,0,Math.Sin((yaw+.31)/2),Math.Cos((yaw+.31)/2))));
        }
        Assert.True(assembly.SolveConstraintsDetailed().Converged);
        for(int i=0;i<count;i++)
        {
            var pose=assembly.WorldPoseOf(bodies[i]);
            var origin=TransformMath.TransformPoint(pose,new Vec3D(0));
            Assert.Equal(5+i*2,origin.X,12); Assert.Equal(-5,origin.Y,12);
            var direction=TransformMath.TransformDirection(pose,new Vec3D(1,0,0));
            Assert.True((direction-new Vec3D(Math.Cos(yaw+.31),Math.Sin(yaw+.31),0)).Length()<1e-12);
        }
        Assert.Equal(count+1,assembly.GetMateResiduals().Count);
    }
}
