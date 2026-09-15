using GeoCore;
using GeoSolver;
using GeoSolver.Kinematics;

namespace GeoTests;

public class NewtonSolverAndContactTests
{
    private sealed class TransformRecorder : IUpdateTransform
    {
        public Transform Last;
        public void Update(Transform transform) => Last = transform;
    }

    [Fact]
    public void SquareSystem_Converges()
    {
        Param x = new Param(2.0);
        Param y = new Param(-1.0);
        var eqs = new List<Expr>
        {
            Expr.Parameter(x) - Expr.Constant(3),
            Expr.Parameter(y) - Expr.Constant(4),
        };
        var container = new EquationContainer(eqs);
        SolveResult result = NewtonSolver.NewtonSolveDetailed(container, forceSparse: false);

        Assert.True(result.Converged);
        Assert.Equal(3.0, x.Value, 6);
        Assert.Equal(4.0, y.Value, 6);
    }

    [Fact]
    public void Contact_OpenGap_StaysPositive_LambdaNearZero()
    {
        var solver = new KinematicSolver();
        var planeBody = new TransformRecorder();
        var pointBody = new TransformRecorder();

        var rigidPlane = solver.AddRigidBody(planeBody, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        var rigidPoint = solver.AddRigidBody(pointBody, new Transform(new Vec3D(0, 0, 5), TransformMath.IdentityOrientation));

        solver.AddConstraint(new FixedTransformConstraint3d(rigidPlane.Transform));

        CVec3D worldPoint = rigidPoint.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0)));
        CPlane3D worldPlane = new CPlane3D(
            rigidPlane.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0))),
            rigidPlane.Transform.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(0, 0, 1))));

        var contact = new ContactHalfSpace3d(worldPoint, worldPlane);
        solver.AddConstraint(contact);

        double gapBefore = contact.EvaluateSignedGap();
        Assert.True(gapBefore > 1.0);

        SolveResult result = solver.SolveConstraints();

        double gapAfter = contact.EvaluateSignedGap();
        Assert.True(gapAfter > 1.0, $"Open contact snapped; gap={gapAfter}");
        Assert.InRange(contact.Lambda.Value, -1e-6, 1e-3);
        Assert.True(result.SumOfSquaredErrors < 1e-6);
    }

    [Fact]
    public void Contact_Penetrating_ClosesWithNonNegativeLambda()
    {
        var solver = new KinematicSolver();
        var planeBody = new TransformRecorder();
        var pointBody = new TransformRecorder();

        var rigidPlane = solver.AddRigidBody(planeBody, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        var rigidPoint = solver.AddRigidBody(pointBody, new Transform(new Vec3D(0, 0, -2), TransformMath.IdentityOrientation));

        solver.AddConstraint(new FixedTransformConstraint3d(rigidPlane.Transform));

        CVec3D worldPoint = rigidPoint.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0)));
        CPlane3D worldPlane = new CPlane3D(
            rigidPlane.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0))),
            rigidPlane.Transform.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(0, 0, 1))));

        var contact = new ContactHalfSpace3d(worldPoint, worldPlane);
        solver.AddConstraint(contact);

        Assert.True(contact.EvaluateSignedGap() < 0);

        SolveResult result = solver.SolveConstraints();

        double gap = contact.EvaluateSignedGap();
        Assert.InRange(gap, -NewtonSolver.LENGTH_EPS * 10, NewtonSolver.LENGTH_EPS * 10);
        Assert.True(contact.Lambda.Value >= -1e-9);
        Assert.True(result.SumOfSquaredErrors < 1e-8 || Math.Abs(gap) < NewtonSolver.LENGTH_EPS * 10);
    }

    [Fact]
    public void Contact_ManySimultaneous_NoNetPenetration()
    {
        const int contactCount = 120;
        var solver = new KinematicSolver();
        var planeBody = new TransformRecorder();
        var pointBody = new TransformRecorder();

        var rigidPlane = solver.AddRigidBody(planeBody, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        // Drop the free body below the plane so all contacts start penetrating.
        var rigidPoint = solver.AddRigidBody(pointBody, new Transform(new Vec3D(0, 0, -1.5), TransformMath.IdentityOrientation));

        solver.AddConstraint(new FixedTransformConstraint3d(rigidPlane.Transform));

        CPlane3D worldPlane = new CPlane3D(
            rigidPlane.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0))),
            rigidPlane.Transform.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(0, 0, 1))));

        var contacts = new List<ContactHalfSpace3d>(contactCount);
        for (int i = 0; i < contactCount; i++)
        {
            double x = (i % 10) * 0.1;
            double y = (i / 10) * 0.1;
            CVec3D worldPoint = rigidPoint.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(x, y, 0)));
            var contact = new ContactHalfSpace3d(worldPoint, worldPlane);
            contacts.Add(contact);
            solver.AddConstraint(contact);
        }

        var sw = System.Diagnostics.Stopwatch.StartNew();
        SolveResult result = solver.SolveConstraints();
        sw.Stop();

        Assert.True(sw.ElapsedMilliseconds < 15000, $"Many-contact solve too slow: {sw.ElapsedMilliseconds} ms");

        for (int i = 0; i < contacts.Count; i++)
        {
            double g = contacts[i].EvaluateSignedGap();
            Assert.True(g >= -1e-3, $"Contact {i} still penetrating: g={g}");
            Assert.True(contacts[i].Lambda.Value >= -1e-8);
        }
        Assert.True(result.SumOfSquaredErrors < 1e-3 || contacts.All(c => c.EvaluateSignedGap() >= -1e-3));
    }
}
