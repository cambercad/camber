using GeoCore;
using GeoSolver;
using GeoSolver.Kinematics;

namespace GeoTests;

public class KinematicsTests
{
    private sealed class TransformRecorder : IUpdateTransform
    {
        public Transform Last;

        public void Update(Transform transform) => Last = transform;
    }

    [Fact]
    public void CQuaternion_Evaluate_UsesWComponent()
    {
        var q = new CQuaternion(
            Expr.Parameter(0.1),
            Expr.Parameter(0.2),
            Expr.Parameter(0.3),
            Expr.Parameter(0.4));

        Quaternion eval = q.Evaluate();

        Assert.Equal(0.1, eval.X, 6);
        Assert.Equal(0.2, eval.Y, 6);
        Assert.Equal(0.3, eval.Z, 6);
        Assert.Equal(0.4, eval.W, 6);
    }

    [Fact]
    public void CTransform_PointLocalToGlobal_MatchesTransformMath()
    {
        var pose = new Transform(new Vec3D(10, -2, 3), new Quaternion(0, 0.70710678, 0, 0.70710678));
        CTransform ct = CTransform.FromPose(pose.Position, pose.Orientation);
        Vec3D local = new Vec3D(1, 2, 3);

        Vec3D world = ct.PointLocalToGlobal(CVec3D.Constant(local)).Evaluate();
        Vec3D expected = TransformMath.TransformPoint(in pose, local);

        Assert.Equal(expected.X, world.X, 4);
        Assert.Equal(expected.Y, world.Y, 4);
        Assert.Equal(expected.Z, world.Z, 4);
    }

    [Fact]
    public void CTransform_DirectionLocalToGlobal_MatchesTransformMath()
    {
        var pose = new Transform(new Vec3D(0), new Quaternion(0, 0.70710678, 0, 0.70710678));
        CTransform ct = CTransform.FromPose(pose.Position, pose.Orientation);
        Vec3D local = new Vec3D(0, 0, 1);

        Vec3D world = ct.DirectionLocalToGlobal(CVec3D.Constant(local)).Evaluate();
        Vec3D expected = TransformMath.TransformDirection(in pose, local);

        Assert.Equal(expected.X, world.X, 4);
        Assert.Equal(expected.Y, world.Y, 4);
        Assert.Equal(expected.Z, world.Z, 4);
    }

    [Fact]
    public void CTransform_Evaluate_KeepsUnitOrientation()
    {
        var solver = new KinematicSolver();
        var body = new TransformRecorder();
        solver.AddRigidBody(body, new Transform(new Vec3D(0), new Quaternion(2, 0, 0, 0)));
        solver.SolveConstraints();

        Quaternion q = body.Last.Orientation;
        double lenSq = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
        Assert.InRange(lenSq, 0.99, 1.01);
    }

    [Fact]
    public void FixedTransformConstraint_LocksPose()
    {
        var solver = new KinematicSolver();
        var body = new TransformRecorder();
        var fixedPose = new Transform(new Vec3D(3, 4, 5), TransformMath.IdentityOrientation);
        RigidTransform<TransformRecorder> rigid = solver.AddRigidBody(body, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));

        solver.AddConstraint(new FixedTransformConstraint3d(rigid.Transform, fixedPose));
        solver.SolveConstraints();

        Transform solved = rigid.Transform.Evaluate();
        Assert.Equal(3, solved.Position.X, 4);
        Assert.Equal(4, solved.Position.Y, 4);
        Assert.Equal(5, solved.Position.Z, 4);
    }

    [Fact]
    public void CoincidentPoints_MovesSecondBody()
    {
        var solver = new KinematicSolver();
        var bodyA = new TransformRecorder();
        var bodyB = new TransformRecorder();

        var rigidA = solver.AddRigidBody(bodyA, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        var rigidB = solver.AddRigidBody(bodyB, new Transform(new Vec3D(20, 0, 0), TransformMath.IdentityOrientation));

        CVec3D worldA = rigidA.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0)));
        CVec3D worldB = rigidB.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0)));

        solver.AddConstraint(new FixedTransformConstraint3d(rigidA.Transform));
        solver.AddConstraint(new PointOnPoint3d(worldA, worldB));
        solver.SolveConstraints();

        Transform poseB = rigidB.Transform.Evaluate();
        Assert.InRange(poseB.Position.X, -0.5, 0.5);
        Assert.InRange(poseB.Position.Y, -0.5, 0.5);
        Assert.InRange(poseB.Position.Z, -0.5, 0.5);
    }

    // Parallel axis alignment is validated end-to-end in AssemblyTests.LJoint_TwoPlates_CoincidentHoleAxes.
}
