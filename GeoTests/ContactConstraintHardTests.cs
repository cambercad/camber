using GeoCore;
using GeoSolver;
using GeoSolver.Kinematics;
using Xunit.Abstractions;

namespace GeoTests;

public class ContactConstraintHardTests
{
    private readonly ITestOutputHelper _output;

    public ContactConstraintHardTests(ITestOutputHelper output) => _output = output;

    private sealed class TransformRecorder : IUpdateTransform
    {
        public Transform Last;
        public void Update(Transform transform) => Last = transform;
    }

    private static RigidTransform<TransformRecorder> AddFixedPlane(
        KinematicSolver solver, out CPlane3D worldPlane, Vec3D origin, Vec3D normal)
    {
        var body = new TransformRecorder();
        var rigid = solver.AddRigidBody(body, new Transform(origin, TransformMath.IdentityOrientation));
        solver.AddConstraint(new FixedTransformConstraint3d(rigid.Transform));
        worldPlane = new CPlane3D(
            rigid.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0))),
            rigid.Transform.DirectionLocalToGlobal(CVec3D.Constant(normal)));
        return rigid;
    }

    private static ContactHalfSpace3d AddPointContact(
        KinematicSolver solver,
        RigidTransform<TransformRecorder> pointBody,
        CPlane3D plane,
        Vec3D localPoint)
    {
        CVec3D worldPoint = pointBody.Transform.PointLocalToGlobal(CVec3D.Constant(localPoint));
        var contact = new ContactHalfSpace3d(worldPoint, plane);
        solver.AddConstraint(contact);
        return contact;
    }

    [Fact]
    public void Contact_MixedOpenAndClosed_OnlyClosesPenetrating()
    {
        var solver = new KinematicSolver();
        AddFixedPlane(solver, out CPlane3D plane, new Vec3D(0), new Vec3D(0, 0, 1));

        var free = new TransformRecorder();
        // Free body owns both an open tip (above) and a penetrating tip (below) in local Z.
        var rigid = solver.AddRigidBody(free, new Transform(new Vec3D(0, 0, -1), TransformMath.IdentityOrientation));

        var closed = AddPointContact(solver, rigid, plane, new Vec3D(0, 0, 0));   // world z=-1
        var open = AddPointContact(solver, rigid, plane, new Vec3D(0, 0, 5));     // world z=4

        Assert.True(closed.EvaluateSignedGap() < 0);
        Assert.True(open.EvaluateSignedGap() > 0);

        SolveResult result = solver.SolveConstraints();

        Assert.True(closed.EvaluateSignedGap() >= -1e-4, $"penetrating stayed open: g={closed.EvaluateSignedGap()}");
        Assert.True(open.EvaluateSignedGap() > 0.5, $"open contact collapsed: g={open.EvaluateSignedGap()}");
        Assert.True(closed.Lambda.Value >= -1e-9);
        Assert.InRange(open.Lambda.Value, -1e-6, 1e-3);
        Assert.True(result.SumOfSquaredErrors < 1e-4);
    }

    [Fact]
    public void Contact_WithBilateralMates_StaysConsistent()
    {
        var solver = new KinematicSolver();
        AddFixedPlane(solver, out CPlane3D plane, new Vec3D(0), new Vec3D(0, 0, 1));

        var slider = new TransformRecorder();
        var rigid = solver.AddRigidBody(slider, new Transform(new Vec3D(1, 2, -3), TransformMath.IdentityOrientation));

        // Keep orientation fixed via unit-ish lock: pin X/Y of local origin on a world line by coinciding
        // only XY through two plane-like constraints — use FixedTransform for orientation components
        // by freezing after create via FixedTransform on a copy of orientation... simpler: distance of XY to axis 0
        // via PointOnPoint X and Y only — use FixedTransformConstraint for full pose but only after Z opens?
        // Practical: lock orientation by FixedTransform at identity orientation with free Z only via contact —
        // Fixed locks all 6; instead freeze orientation params:
        foreach (Param p in rigid.Transform.Orientation)
            p.Frozen = true;
        // Lock XY translation so only Z + λ remain free for contact closure.
        rigid.Transform.Position.Ex.Value.Frozen = true;
        rigid.Transform.Position.Ey.Value.Frozen = true;

        var contact = AddPointContact(solver, rigid, plane, new Vec3D(0));
        Assert.True(contact.EvaluateSignedGap() < 0);

        SolveResult result = solver.SolveConstraints();

        Assert.True(result.Converged || Math.Abs(contact.EvaluateSignedGap()) < 1e-3);
        Assert.InRange(contact.EvaluateSignedGap(), -1e-3, 1e-3);
        Assert.Equal(1.0, rigid.Transform.Position.Ex.Evaluate(), 5);
        Assert.Equal(2.0, rigid.Transform.Position.Ey.Evaluate(), 5);
        Assert.True(contact.Lambda.Value >= -1e-9);
    }

    [Fact]
    public void Contact_TiltedPlane_ClosesAlongNormal()
    {
        var solver = new KinematicSolver();
        // 45° plane: n = normalize(0,1,1), through origin.
        Vec3D n = new Vec3D(0, 1, 1);
        n = n / n.Length();
        AddFixedPlane(solver, out CPlane3D plane, new Vec3D(0), n);

        var free = new TransformRecorder();
        // Start on the negative side of the plane: point (0,0,-2) has n·p = -√2 < 0.
        var rigid = solver.AddRigidBody(free, new Transform(new Vec3D(0, 0, -2), TransformMath.IdentityOrientation));
        foreach (Param p in rigid.Transform.Orientation)
            p.Frozen = true;

        var contact = AddPointContact(solver, rigid, plane, new Vec3D(0));
        double g0 = contact.EvaluateSignedGap();
        Assert.True(g0 < 0);

        solver.SolveConstraints();

        double g = contact.EvaluateSignedGap();
        Assert.InRange(g, -1e-3, 1e-3);
        Assert.True(contact.Lambda.Value >= -1e-9);
        // Should have moved mostly in the normal direction (Y and Z jointly).
        Vec3D pos = rigid.Transform.Position.Evaluate();
        Assert.True(Math.Abs(pos.X) < 0.5, $"unexpected lateral X drift: {pos}");
    }

    [Fact]
    public void Contact_WarmStart_SecondSolveStaysClosed()
    {
        var solver = new KinematicSolver();
        AddFixedPlane(solver, out CPlane3D plane, new Vec3D(0), new Vec3D(0, 0, 1));

        var free = new TransformRecorder();
        var rigid = solver.AddRigidBody(free, new Transform(new Vec3D(0, 0, -2), TransformMath.IdentityOrientation));
        foreach (Param p in rigid.Transform.Orientation)
            p.Frozen = true;
        rigid.Transform.Position.Ex.Value.Frozen = true;
        rigid.Transform.Position.Ey.Value.Frozen = true;

        var contact = AddPointContact(solver, rigid, plane, new Vec3D(0));
        solver.SolveConstraints();
        double lambda1 = contact.Lambda.Value;
        double g1 = contact.EvaluateSignedGap();
        Assert.InRange(g1, -1e-3, 1e-3);

        // Nudge slightly into penetration and re-solve — λ should warm-start, gap should re-close.
        rigid.Transform.Position.Ez.Value.Value -= 0.25;
        Assert.True(contact.EvaluateSignedGap() < 0);

        var sw = System.Diagnostics.Stopwatch.StartNew();
        SolveResult second = solver.SolveConstraints();
        sw.Stop();

        Assert.InRange(contact.EvaluateSignedGap(), -1e-3, 1e-3);
        Assert.True(contact.Lambda.Value >= -1e-9);
        Assert.True(second.SumOfSquaredErrors < 1e-4);
        _output.WriteLine($"Warm re-solve: {sw.Elapsed.TotalMilliseconds:F2} ms, λ {lambda1:G4} → {contact.Lambda.Value:G4}");
    }

    [Fact]
    public void Contact_BothBodiesFree_RelativeClosure()
    {
        var solver = new KinematicSolver();
        // Plane body not fixed — only root the system by fixing one DOF set on plane XY and orientation.
        var planeBody = new TransformRecorder();
        var planeRigid = solver.AddRigidBody(planeBody, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        foreach (Param p in planeRigid.Transform.Orientation)
            p.Frozen = true;
        planeRigid.Transform.Position.Ex.Value.Frozen = true;
        planeRigid.Transform.Position.Ey.Value.Frozen = true;
        // Leave plane Z free.

        CPlane3D plane = new CPlane3D(
            planeRigid.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0))),
            planeRigid.Transform.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(0, 0, 1))));

        var pointBody = new TransformRecorder();
        var pointRigid = solver.AddRigidBody(pointBody, new Transform(new Vec3D(0, 0, -1.5), TransformMath.IdentityOrientation));
        foreach (Param p in pointRigid.Transform.Orientation)
            p.Frozen = true;
        pointRigid.Transform.Position.Ex.Value.Frozen = true;
        pointRigid.Transform.Position.Ey.Value.Frozen = true;

        var contact = AddPointContact(solver, pointRigid, plane, new Vec3D(0));
        Assert.True(contact.EvaluateSignedGap() < 0);

        solver.SolveConstraints();

        Assert.InRange(contact.EvaluateSignedGap(), -1e-3, 1e-3);
        Assert.True(contact.Lambda.Value >= -1e-9);
    }

    [Fact]
    public void Contact_DeepPenetration_Recovers()
    {
        var solver = new KinematicSolver();
        AddFixedPlane(solver, out CPlane3D plane, new Vec3D(0), new Vec3D(0, 0, 1));

        var free = new TransformRecorder();
        var rigid = solver.AddRigidBody(free, new Transform(new Vec3D(0, 0, -50), TransformMath.IdentityOrientation));
        foreach (Param p in rigid.Transform.Orientation)
            p.Frozen = true;
        rigid.Transform.Position.Ex.Value.Frozen = true;
        rigid.Transform.Position.Ey.Value.Frozen = true;

        var contact = AddPointContact(solver, rigid, plane, new Vec3D(0));
        Assert.True(contact.EvaluateSignedGap() < -40);

        SolveResult result = solver.SolveConstraints();

        Assert.InRange(contact.EvaluateSignedGap(), -5e-3, 5e-3);
        Assert.True(contact.Lambda.Value >= -1e-9);
        Assert.True(result.SumOfSquaredErrors < 1e-2 || Math.Abs(contact.EvaluateSignedGap()) < 5e-3);
    }

    [Fact]
    public void Contact_SparseForced_MatchesDenseClosure()
    {
        static (List<IBaseEquation> cons, List<ContactHalfSpace3d> contacts) Build()
        {
            var planeRecorder = new TransformRecorder();
            var pointRecorder = new TransformRecorder();
            CTransform planeT = CTransform.FromPose(new Vec3D(0), TransformMath.IdentityOrientation);
            CTransform pointT = CTransform.FromPose(new Vec3D(0, 0, -1.2), TransformMath.IdentityOrientation);
            foreach (Param p in pointT.Orientation)
                p.Frozen = true;

            var cons = new List<IBaseEquation>
            {
                new FixedTransformConstraint3d(planeT),
            };

            CPlane3D plane = new CPlane3D(
                planeT.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0))),
                planeT.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(0, 0, 1))));

            var contacts = new List<ContactHalfSpace3d>();
            for (int i = 0; i < 40; i++)
            {
                var c = new ContactHalfSpace3d(
                    pointT.PointLocalToGlobal(CVec3D.Constant(new Vec3D((i % 8) * 0.05, (i / 8) * 0.05, 0))),
                    plane);
                contacts.Add(c);
                cons.Add(c);
            }

            // Keep recorders unused but named to document intent.
            _ = planeRecorder;
            _ = pointRecorder;
            return (cons, contacts);
        }

        static SolveResult Solve(IList<IBaseEquation> cons, bool forceSparse)
        {
            var equations = new List<Expr>();
            HashSet<Param> nonNeg = null;
            foreach (var c in cons)
            {
                if (c is ContactHalfSpace3d contact)
                {
                    nonNeg ??= new HashSet<Param>();
                    nonNeg.Add(contact.Lambda);
                }
                c.GenerateEquations(equations, 1.0);
            }
            return NewtonSolver.NewtonSolveDetailed(
                new EquationContainer(equations), null, nonNeg, 1.0, forceSparse, allowSoftOverconstrained: true);
        }

        var (denseCons, denseContacts) = Build();
        var sw = System.Diagnostics.Stopwatch.StartNew();
        SolveResult dense = Solve(denseCons, forceSparse: false);
        sw.Stop();
        double denseMs = sw.Elapsed.TotalMilliseconds;
        double denseGap = denseContacts.Min(c => c.EvaluateSignedGap());

        var (sparseCons, sparseContacts) = Build();
        sw.Restart();
        SolveResult sparse = Solve(sparseCons, forceSparse: true);
        sw.Stop();
        double sparseMs = sw.Elapsed.TotalMilliseconds;
        double sparseGap = sparseContacts.Min(c => c.EvaluateSignedGap());

        Assert.True(denseGap >= -1e-3, $"dense minGap={denseGap}");
        Assert.True(sparseGap >= -1e-3, $"sparse minGap={sparseGap}");
        _output.WriteLine(
            $"Dense {denseMs:F1} ms gap={denseGap:G4} nP={dense.NumParameters} nE={dense.NumEquations}; " +
            $"Sparse {sparseMs:F1} ms gap={sparseGap:G4} nP={sparse.NumParameters} nE={sparse.NumEquations}");
    }

    [Fact]
    public void Contact_Performance_ScaleSweep()
    {
        int[] sizes = { 50, 120, 250, 500 };
        foreach (int n in sizes)
        {
            var solver = new KinematicSolver();
            AddFixedPlane(solver, out CPlane3D plane, new Vec3D(0), new Vec3D(0, 0, 1));
            var free = new TransformRecorder();
            var rigid = solver.AddRigidBody(free, new Transform(new Vec3D(0, 0, -1.5), TransformMath.IdentityOrientation));
            foreach (Param p in rigid.Transform.Orientation)
                p.Frozen = true;

            var contacts = new List<ContactHalfSpace3d>(n);
            for (int i = 0; i < n; i++)
            {
                contacts.Add(AddPointContact(solver, rigid, plane,
                    new Vec3D((i % 20) * 0.05, (i / 20) * 0.05, 0)));
            }

            // Warm-up once for JIT
            if (n == sizes[0])
                solver.SolveConstraints();

            // Rebuild fresh for timed run
            solver = new KinematicSolver();
            AddFixedPlane(solver, out plane, new Vec3D(0), new Vec3D(0, 0, 1));
            free = new TransformRecorder();
            rigid = solver.AddRigidBody(free, new Transform(new Vec3D(0, 0, -1.5), TransformMath.IdentityOrientation));
            foreach (Param p in rigid.Transform.Orientation)
                p.Frozen = true;
            contacts = new List<ContactHalfSpace3d>(n);
            for (int i = 0; i < n; i++)
                contacts.Add(AddPointContact(solver, rigid, plane, new Vec3D((i % 20) * 0.05, (i / 20) * 0.05, 0)));

            var sw = System.Diagnostics.Stopwatch.StartNew();
            SolveResult result = solver.SolveConstraints();
            sw.Stop();
            long totalMs = sw.ElapsedMilliseconds;

            // Re-solve rebuilds EquationContainer each call; λ warm-starts but symbolic J does not.
            sw.Restart();
            SolveResult warm = solver.SolveConstraints();
            sw.Stop();
            long warmMs = sw.ElapsedMilliseconds;

            double maxPen = contacts.Min(c => c.EvaluateSignedGap());
            _output.WriteLine(
                $"N={n,4}: first={totalMs,5} ms, re-solve={warmMs,4} ms, SSE={result.SumOfSquaredErrors:G3}, " +
                $"minGap={maxPen:G4}, conv={result.Converged}/{warm.Converged}, nE={result.NumEquations}, nP={result.NumParameters}");

            Assert.True(maxPen >= -2e-3, $"N={n} still penetrating: minGap={maxPen}");
            Assert.True(totalMs < 30_000, $"N={n} too slow: {totalMs} ms");
        }
    }

    [Fact]
    public void Contact_RedundantCoplanarContacts_RemainStable()
    {
        var solver = new KinematicSolver();
        AddFixedPlane(solver, out CPlane3D plane, new Vec3D(0), new Vec3D(0, 0, 1));

        var free = new TransformRecorder();
        var rigid = solver.AddRigidBody(free, new Transform(new Vec3D(0, 0, -0.8), TransformMath.IdentityOrientation));
        foreach (Param p in rigid.Transform.Orientation)
            p.Frozen = true;

        // Many contacts with identical local Z (full redundancy when body is rigid).
        var contacts = new List<ContactHalfSpace3d>();
        for (int i = 0; i < 30; i++)
            contacts.Add(AddPointContact(solver, rigid, plane, new Vec3D(i * 0.01, 0, 0)));

        SolveResult result = solver.SolveConstraints();
        foreach (var c in contacts)
        {
            Assert.True(c.EvaluateSignedGap() >= -1e-3);
            Assert.True(c.Lambda.Value >= -1e-8);
        }
        Assert.False(double.IsNaN(result.SumOfSquaredErrors));
        Assert.True(result.SumOfSquaredErrors < 1e-2);
    }
}
