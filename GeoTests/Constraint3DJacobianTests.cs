using GeoCore;
using GeoSolver;
using GeoSolver.Kinematics;

namespace GeoTests;

/// <summary>
/// Analytic vs finite-difference checks for Gibbs charts, Bake, and directed-parallel
/// residuals. These catch Rest-as-constant capture and a zero 90° rotation Jacobian.
/// </summary>
public class Constraint3DJacobianTests
{
    private const double FdH = 1e-7;
    private const double JacTol = 1e-5;

    private static double FiniteDiff(Expr expr, Param p)
    {
        double saved = p.Value;
        p.Value = saved + FdH;
        double plus = expr.Evaluate();
        p.Value = saved - FdH;
        double minus = expr.Evaluate();
        p.Value = saved;
        return (plus - minus) / (2.0 * FdH);
    }

    [Fact]
    public void GibbsDirection_PartialMatchesFiniteDifference_AtIdentity()
    {
        CTransform body = CTransform.FromPose(new Vec3D(0), TransformMath.IdentityOrientation);
        CVec3D world = body.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(-1, 0, 0)));
        Param wz = body.RotationVector.Ez.Value;

        double adY = world.Ey.EvaluatePartialDerivative(wz);
        double fdY = FiniteDiff(world.Ey, wz);
        Assert.InRange(Math.Abs(adY - fdY), 0, JacTol);

        // Gibbs ω = tan(θ/2); dθ/dω = 2 at 0, so ∂(R_z(θ)(-X))_y / ∂ω_z = -2.
        Assert.InRange(adY, -2.02, -1.98);
    }

    [Fact]
    public void GibbsDirection_PartialMatchesFiniteDifference_At90DegChart()
    {
        CTransform body = CTransform.FromPose(new Vec3D(0), TransformMath.IdentityOrientation);
        body.RotationVector.Ez.SetValue(1.0);
        CVec3D world = body.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(-1, 0, 0)));
        Param wz = body.RotationVector.Ez.Value;

        Vec3D w = world.Evaluate();
        Assert.InRange(w.X, -0.02, 0.02);
        Assert.InRange(w.Y, -1.02, -0.98);

        double saved = wz.Value;
        const double h = 1e-5;
        double adForward = world.Ex.EvaluatePartialDerivative(wz);
        wz.Value = saved + h;
        double plus = world.Ex.Evaluate();
        wz.Value = saved - h;
        double minus = world.Ex.Evaluate();
        wz.Value = saved;
        double fdX = (plus - minus) / (2.0 * h);
        double adTree = world.Ex.PartialDerivative(wz).Evaluate();
        double adForwardAfterTree = world.Ex.EvaluatePartialDerivative(wz);

        Assert.InRange(fdX, 0.95, 1.05);
        Assert.InRange(Math.Abs(adForward - fdX), 0, 0.02);
        Assert.InRange(Math.Abs(adTree - fdX), 0, 0.05);
        Assert.InRange(Math.Abs(adForwardAfterTree - fdX), 0, 0.02);
    }

    [Fact]
    public void Bake_KeepsWorldDirectionExpressionInSyncWithEvaluate()
    {
        CTransform body = CTransform.FromPose(new Vec3D(4, 5, 6), TransformMath.IdentityOrientation);
        Vec3D local = new Vec3D(-1, 0, 0);
        CVec3D worldExpr = body.DirectionLocalToGlobal(CVec3D.Constant(local));

        body.RotationVector.Ez.SetValue(1.0);
        Vec3D before = worldExpr.Evaluate();
        Transform poseBefore = body.Evaluate();
        Vec3D expectedBefore = TransformMath.TransformDirection(in poseBefore, local);
        Assert.InRange(Math.Abs(before.X - expectedBefore.X), 0, 1e-9);
        Assert.InRange(Math.Abs(before.Y - expectedBefore.Y), 0, 1e-9);

        body.Bake();
        Assert.InRange(Math.Abs(body.RotationVector.Ez.Evaluate()), 0, 1e-15);

        Vec3D after = worldExpr.Evaluate();
        Transform poseAfter = body.Evaluate();
        Vec3D expectedAfter = TransformMath.TransformDirection(in poseAfter, local);
        Assert.InRange(Math.Abs(after.X - expectedAfter.X), 0, 1e-9);
        Assert.InRange(Math.Abs(after.Y - expectedAfter.Y), 0, 1e-9);
        Assert.InRange(Math.Abs(after.X - before.X), 0, 1e-9);
        Assert.InRange(Math.Abs(after.Y - before.Y), 0, 1e-9);
    }

    [Fact]
    public void DirectedParallel_Stacked90Deg_KeptRowsHaveRotationJacobian()
    {
        CTransform bodyA = CTransform.FromPose(new Vec3D(0), TransformMath.IdentityOrientation);
        CTransform bodyB = CTransform.FromPose(new Vec3D(0), TransformMath.IdentityOrientation);
        CVec3D n1 = bodyA.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(0, 1, 0)));
        CVec3D n2 = bodyB.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(-1, 0, 0)));

        var constraint = new DirectedParallelDirections3d(n1, n2, opposite: true);
        var equations = new List<Expr>();
        constraint.GenerateEquations(equations, 1.0);
        Assert.Equal(3, equations.Count);

        var container = new EquationContainer(equations, applySketchReductions: false);
        Param wz = bodyB.RotationVector.Ez.Value;
        int wzCol = -1;
        for (int j = 0; j < container.NumParameters; j++)
        {
            if (container.GetParameter(j) == wz)
                wzCol = j;
        }
        Assert.True(wzCol >= 0, "ω_z is a free Newton column");

        double[] b = new double[container.NumEquations];
        for (int i = 0; i < container.NumEquations; i++)
            b[i] = container.Evaluate(i);

        bool hasRotationStep = false;
        for (int i = 0; i < container.NumEquations; i++)
        {
            double ad = container.EvaluatJacobian(i, wzCol);
            double fd = FiniteDiff(equations[i], wz);
            Assert.InRange(Math.Abs(ad - fd), 0, 2e-5);
            if (Math.Abs(ad) > 0.5)
                hasRotationStep = true;
        }

        Assert.True(hasRotationStep, "directed-parallel residual has no ∂/∂ω_z at 90° stacked start");
        Assert.True(Math.Abs(b[0]) + Math.Abs(b[1]) + Math.Abs(b[2]) > 0.5, "90° start should have a large residual");
    }

    [Fact]
    public void GibbsDirection_PartialMatchesFiniteDifference_AtSmallGibbs()
    {
        CTransform body = CTransform.FromPose(new Vec3D(0), TransformMath.IdentityOrientation);
        body.RotationVector.Ez.SetValue(0.2);
        CVec3D world = body.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(-1, 0, 0)));
        Param wz = body.RotationVector.Ez.Value;

        const double h = 1e-5;
        double saved = wz.Value;
        wz.Value = saved + h;
        double plus = world.Ey.Evaluate();
        wz.Value = saved - h;
        double minus = world.Ey.Evaluate();
        wz.Value = saved;
        double fd = (plus - minus) / (2.0 * h);
        double ad = world.Ey.EvaluatePartialDerivative(wz);
        Assert.InRange(Math.Abs(ad - fd), 0, 0.02);
    }

    [Fact]
    public void PointLocalToGlobal_TranslationJacobianIsIdentity()
    {
        CTransform body = CTransform.FromPose(new Vec3D(1, 2, 3), TransformMath.IdentityOrientation);
        CVec3D world = body.PointLocalToGlobal(CVec3D.Constant(new Vec3D(10, 0, 0)));
        Param px = body.Position.Ex.Value;
        double ad = world.Ex.PartialDerivative(px).Evaluate();
        Assert.InRange(ad, 0.999, 1.001);
        Assert.InRange(Math.Abs(FiniteDiff(world.Ex, px) - ad), 0, JacTol);
    }
}
