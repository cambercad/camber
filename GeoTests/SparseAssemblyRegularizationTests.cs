using Geo;
using GeoCore;
using GeoSolver;

namespace GeoTests;

public class SparseAssemblyRegularizationTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    private sealed class WithFreeParameter(IEquationContainer inner) : IEquationContainer
    {
        public Param Free { get; } = new Param(7);
        public int NumParameters => inner.NumParameters + 1;
        public int NumEquations => inner.NumEquations;
        public double Evaluate(int i) => inner.Evaluate(i);
        public double EvaluatJacobian(int i, int j) => j == inner.NumParameters ? 0 : inner.EvaluatJacobian(i, j);
        public Param GetParameter(int i) => i == inner.NumParameters ? Free : inner.GetParameter(i);
        public int NumNonZerosInRow(int i) => inner.NumNonZerosInRow(i);
        public void EvaluateJacobianRow(int row, ref int indexer, IList<double> values, IList<int> columns) =>
            inner.EvaluateJacobianRow(row, ref indexer, values, columns);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RedundantEquationsRetainFreeMotionInBothSolvePaths(bool sparse)
    {
        // Forty independent rank-one pairs cross the automatic sparse threshold.
        // Repeated equations constrain sums, leaving their differences free.
        var parameters = Enumerable.Range(0, 80).Select(_ => new Param(0)).ToArray();
        var expressions = new List<Expr>();
        for (int i = 0; i < parameters.Length; i += 2)
        {
            var residual = Expr.Parameter(parameters[i]) + Expr.Parameter(parameters[i + 1]) - Expr.Constant(1);
            expressions.Add(residual);
            expressions.Add(residual);
            expressions.Add(residual);
        }
        var equations = new WithFreeParameter(new EquationContainer(expressions, applySketchReductions: false));
        var result = NewtonSolver.NewtonSolveDetailed(equations,
            forceSparse: sparse, allowSoftOverconstrained: true);
        Assert.Equal(81, result.NumParameters);
        Assert.Equal(7, equations.Free.Value);
        Assert.True(result.Converged, result.Message);
        Assert.All(parameters, parameter => Assert.InRange(parameter.Value, .5 - 1e-8, .5 + 1e-8));
    }

    [Fact]
    public void BearingAssemblyAboveSparseThresholdCanRotateWithoutFixingOtherJoints()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50), new Vec3D(400)), .1);
        var shape = api.CreateCuboid(CoordinateSystem.Default, new Vec3D(2), "body");
        var assembly = api.GetAssembly("bearing_array");
        assembly.SolveAfterEveryConstraint = false;
        var ground = assembly.AddPart(shape, new Vec3D(0));
        assembly.FixPart(ground);
        var bodies = new List<AssemblyPart>();
        for (int i = 0; i < 14; i++)
        {
            var centre = new Vec3D(i * 20, 0, 0);
            var body = assembly.AddPart(shape, centre);
            bodies.Add(body);
            assembly.SetConcentric(ground.AddAxisDatumAt(centre, new Vec3D(0, 1, 0)),
                body.AddAxisDatumAt(new Vec3D(0), new Vec3D(0, 1, 0)));
            assembly.SetCoincidentOriented(ground.AddPlaneDatumAt(centre, new Vec3D(0, 1, 0)),
                body.AddPlaneDatumAt(new Vec3D(0), new Vec3D(0, 1, 0)), oppositeNormals: false);
        }
        assembly.SetAngle(ground.AddAxisDatumAt(new Vec3D(0), new Vec3D(1, 0, 0)),
            bodies[0].AddAxisDatumAt(new Vec3D(0), new Vec3D(1, 0, 0)), .17);
        var result = assembly.SolveConstraintsDetailed();
        Assert.Equal(84, result.NumParameters);
        Assert.True(result.Converged, result.Message);
        for (int i = 0; i < bodies.Count; i++)
        {
            var pose = assembly.WorldPoseOf(bodies[i]);
            var centre = TransformMath.TransformPoint(pose, new Vec3D(0));
            Assert.True((centre - new Vec3D(i * 20, 0, 0)).Length() < 1e-5);
            var direction = TransformMath.TransformDirection(pose, new Vec3D(1, 0, 0));
            Assert.InRange(direction.X, (i == 0 ? Math.Cos(.17) : 1) - 1e-7,
                (i == 0 ? Math.Cos(.17) : 1) + 1e-7);
        }
    }
}
