using CSG;
using Curves;
using Geo;
using GeoCore;
using GeoSolver;
using GeoSolver.Kinematics;
using Xunit.Abstractions;

namespace GeoTests;

[CollectionDefinition("GeoAPISequential", DisableParallelization = true)]
public class GeoAPISequentialCollection
{
}

/// <summary>
/// Assembly Newton ladder: translation-only → small rotation → stacked identity.
/// Two plane mates (face + clocking) leave one slide DOF, so translation checks
/// only the constrained directions.
/// </summary>
[Collection("GeoAPISequential")]
public sealed class AssemblyMateLadderTests : IDisposable
{
    private const double Arm = 40;
    private const double PosTol = 0.75;
    private readonly ITestOutputHelper _output;

    public AssemblyMateLadderTests(ITestOutputHelper output) => _output = output;

    public void Dispose() => GeoAPI.Clear();

    private static Quaternion RotZ(double radians)
    {
        double h = 0.5 * radians;
        return new Quaternion(0, 0, Math.Sin(h), Math.Cos(h));
    }

    private static Transform ExpectedElbowB()
    {
        return new Transform(new Vec3D(Arm, Arm, 0), RotZ(Math.PI / 2.0));
    }

    private sealed class TransformRecorder : IUpdateTransform
    {
        public Transform Last;
        public void Update(Transform transform) => Last = transform;
    }

    private static void MateElbow(
        KinematicSolver solver,
        RigidTransform<TransformRecorder> outgoing,
        RigidTransform<TransformRecorder> incoming)
    {
        CVec3D outOrigin = outgoing.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(Arm, Arm, 0)));
        CVec3D outNormal = outgoing.Transform.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(0, 1, 0)));
        CVec3D inOrigin = incoming.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0)));
        CVec3D inNormal = incoming.Transform.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(-1, 0, 0)));
        CVec3D outNorthO = outgoing.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0)));
        CVec3D outNorthN = outgoing.Transform.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(0, 0, 1)));
        CVec3D inNorthO = incoming.Transform.PointLocalToGlobal(CVec3D.Constant(new Vec3D(0)));
        CVec3D inNorthN = incoming.Transform.DirectionLocalToGlobal(CVec3D.Constant(new Vec3D(0, 0, 1)));

        solver.AddConstraint(new DirectedParallelDirections3d(outNormal, inNormal, opposite: true));
        solver.AddConstraint(new PointOnPoint3d(outOrigin, inOrigin));
        solver.AddConstraint(new DirectedParallelDirections3d(outNorthN, inNorthN, opposite: false));
        solver.AddConstraint(new PointOnPlane3d(outNorthO, new CPlane3D(inNorthO, inNorthN)));
    }

    private void Dump(string label, SolveResult result, Transform pose)
    {
        _output.WriteLine(
            "{0}: converged={1} sse={2:G6} n={3} m={4} msg={5} pos=({6:F3},{7:F3},{8:F3})",
            label,
            result.Converged,
            result.SumOfSquaredErrors,
            result.NumParameters,
            result.NumEquations,
            result.Message ?? "",
            pose.Position.X,
            pose.Position.Y,
            pose.Position.Z);
    }

    [Fact]
    public void TwoElbows_TranslationOnly_RotationFrozen()
    {
        Transform target = ExpectedElbowB();
        var solver = new KinematicSolver();
        solver.IncludeCharacteristicLength(Arm);

        var bodyA = new TransformRecorder();
        var bodyB = new TransformRecorder();
        var rigidA = solver.AddRigidBody(bodyA, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        var rigidB = solver.AddRigidBody(bodyB, new Transform(
            target.Position + new Vec3D(0, 18, 7),
            target.Orientation));

        foreach (Param p in rigidB.Transform.Orientation)
            p.Frozen = true;

        solver.AddConstraint(new FixedTransformConstraint3d(rigidA.Transform));
        MateElbow(solver, rigidA, rigidB);

        SolveResult result = solver.SolveConstraints();
        Dump("translation_frozen_R", result, rigidB.Transform.Evaluate());

        Assert.True(result.Converged, result.Message ?? "not converged");
        Assert.True(result.SumOfSquaredErrors < 1e-6, "sse=" + result.SumOfSquaredErrors);

        Transform pose = rigidB.Transform.Evaluate();
        Assert.InRange(pose.Position.Y, Arm - PosTol, Arm + PosTol);
        Assert.InRange(pose.Position.Z, -PosTol, PosTol);
    }

    [Fact]
    public void TwoElbows_TranslationOnly_RotationFree()
    {
        Transform target = ExpectedElbowB();
        var solver = new KinematicSolver();
        solver.IncludeCharacteristicLength(Arm);

        var bodyA = new TransformRecorder();
        var bodyB = new TransformRecorder();
        var rigidA = solver.AddRigidBody(bodyA, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        var rigidB = solver.AddRigidBody(bodyB, new Transform(
            target.Position + new Vec3D(0, 18, 7),
            target.Orientation));

        solver.AddConstraint(new FixedTransformConstraint3d(rigidA.Transform));
        MateElbow(solver, rigidA, rigidB);

        SolveResult result = solver.SolveConstraints();
        Dump("translation_free_R", result, rigidB.Transform.Evaluate());

        Assert.True(result.Converged, result.Message ?? "not converged");
        Assert.True(result.SumOfSquaredErrors < 1e-6, "sse=" + result.SumOfSquaredErrors);

        Transform pose = rigidB.Transform.Evaluate();
        Assert.InRange(pose.Position.Y, Arm - PosTol, Arm + PosTol);
        Assert.InRange(pose.Position.Z, -PosTol, PosTol);
    }

    [Fact]
    public void TwoElbows_SmallRotationError()
    {
        Transform target = ExpectedElbowB();
        var solver = new KinematicSolver();
        solver.IncludeCharacteristicLength(Arm);

        var bodyA = new TransformRecorder();
        var bodyB = new TransformRecorder();
        var rigidA = solver.AddRigidBody(bodyA, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
            var rigidB = solver.AddRigidBody(bodyB, new Transform(
            target.Position,
            RotZ(Math.PI / 2.0 + 5.0 * Math.PI / 180.0)));

        solver.AddConstraint(new FixedTransformConstraint3d(rigidA.Transform));
        MateElbow(solver, rigidA, rigidB);

        SolveResult result = solver.SolveConstraints();
        Dump("small_rotation", result, rigidB.Transform.Evaluate());

        Assert.True(result.Converged, result.Message ?? "not converged");
        Assert.True(result.SumOfSquaredErrors < 1e-6, "sse=" + result.SumOfSquaredErrors);

        Transform pose = rigidB.Transform.Evaluate();
        Assert.InRange(pose.Position.Y, Arm - PosTol, Arm + PosTol);
        Assert.InRange(pose.Position.Z, -PosTol, PosTol);

        Vec3D worldInlet = TransformMath.TransformDirection(pose, new Vec3D(-1, 0, 0));
        worldInlet.Normalize();
        Assert.InRange(worldInlet.Y, -1.02, -0.98);
    }

    [Fact]
    public void TwoElbows_StackedIdentity_Last()
    {
        var solver = new KinematicSolver();
        solver.IncludeCharacteristicLength(Arm);

        var bodyA = new TransformRecorder();
        var bodyB = new TransformRecorder();
        var rigidA = solver.AddRigidBody(bodyA, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        var rigidB = solver.AddRigidBody(bodyB, new Transform(new Vec3D(0), TransformMath.IdentityOrientation));

        solver.AddConstraint(new FixedTransformConstraint3d(rigidA.Transform));
        MateElbow(solver, rigidA, rigidB);

        SolveResult result = solver.SolveConstraints();
        Dump("stacked_identity", result, rigidB.Transform.Evaluate());
        Transform pose = rigidB.Transform.Evaluate();
        Vec3D worldInlet = TransformMath.TransformDirection(pose, new Vec3D(-1, 0, 0));
        worldInlet.Normalize();
        _output.WriteLine("stacked inlet=({0:F4},{1:F4},{2:F4})", worldInlet.X, worldInlet.Y, worldInlet.Z);

        Assert.True(result.Converged, result.Message ?? "not converged");
        Assert.True(result.SumOfSquaredErrors < 1e-5, "sse=" + result.SumOfSquaredErrors);
        Assert.InRange(pose.Position.Y, Arm - PosTol, Arm + PosTol);
        Assert.InRange(pose.Position.Z, -PosTol, PosTol);
        Assert.InRange(worldInlet.Y, -1.02, -0.98);
    }

    [Fact]
    public void ThreeElbows_StackedIdentity_OpenChain()
    {
        var solver = new KinematicSolver();
        solver.IncludeCharacteristicLength(Arm);

        var rec = new TransformRecorder[3];
        var rigid = new RigidTransform<TransformRecorder>[3];
        for (int i = 0; i < 3; i++)
        {
            rec[i] = new TransformRecorder();
            rigid[i] = solver.AddRigidBody(rec[i], new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        }

        solver.AddConstraint(new FixedTransformConstraint3d(rigid[0].Transform));
        MateElbow(solver, rigid[0], rigid[1]);
        SolveResult first = solver.SolveConstraints();
        Dump("chain_1", first, rigid[1].Transform.Evaluate());
        Assert.True(first.Converged, first.Message ?? "joint 0-1 not converged");

        MateElbow(solver, rigid[1], rigid[2]);
        SolveResult second = solver.SolveConstraints();
        Dump("chain_2", second, rigid[2].Transform.Evaluate());
        Transform p1 = rigid[1].Transform.Evaluate();
        Transform p2 = rigid[2].Transform.Evaluate();
        _output.WriteLine("p1=({0:F3},{1:F3}) p2=({2:F3},{3:F3})", p1.Position.X, p1.Position.Y, p2.Position.X, p2.Position.Y);

        Assert.True(second.Converged, second.Message ?? "joint 1-2 not converged");
        Assert.InRange(p1.Position.X, Arm - PosTol, Arm + PosTol);
        Assert.InRange(p1.Position.Y, Arm - PosTol, Arm + PosTol);
        Assert.InRange(p2.Position.X, -PosTol, PosTol);
        Assert.InRange(p2.Position.Y, 2 * Arm - PosTol, 2 * Arm + PosTol);
    }

    [Fact]
    public void FlangedElbow_StraightInnerSweeps_ResolveAsCylinders()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-800), new Vec3D(800)), 0.4);
        AnchorMesh pipe = BuildFlangedElbow(api, "pipe1");

        AxisDatumLocal vertical = AssemblyDatumResolver.ResolveAxis(pipe, "pipe1-inner-v_line");
        AxisDatumLocal horizontal = AssemblyDatumResolver.ResolveAxis(pipe, "pipe1-inner-h_line");
        Vec3D vDir = vertical.Direction;
        Vec3D hDir = horizontal.Direction;
        vDir.Normalize();
        hDir.Normalize();

        Assert.True(Math.Abs(Math.Abs(vDir.Y) - 1.0) < 0.02, "top inner axis should follow the vertical guide");
        Assert.True(Math.Abs(Math.Abs(hDir.X) - 1.0) < 0.02, "bottom inner axis should follow the horizontal guide");
        Assert.False(pipe.TryGetCylinderFromPatch("pipe1-inner-bend", out _), "elbow inner is toroidal, not a cylinder");
    }

    [Fact]
    public void TwoPipes_TranslationOnly_NamedFlanges()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-800), new Vec3D(800)), 0.4);
        AnchorMesh pipe1 = BuildFlangedElbow(api, "pipe1");
        AnchorMesh pipe2 = api.CopyMeshAsInstance(pipe1, "pipe2");

        PlaneDatumLocal top = AssemblyDatumResolver.ResolvePlane(pipe1, "pipe1-top_flange-ExtrudeTop");
        PlaneDatumLocal bot = AssemblyDatumResolver.ResolvePlane(pipe2, "pipe2-bottom_flange-ExtrudeBottom");

        Vec3D wantNormal = top.Normal;
        wantNormal.Normalize();
        Vec3D fromNormal = bot.Normal;
        fromNormal.Normalize();
        Quaternion qAlign = RotationMapping(fromNormal, wantNormal);
        Vec3D rotatedBotOrigin = TransformMath.TransformDirection(new Transform(new Vec3D(0), qAlign), bot.Origin);
        Vec3D seedPos = top.Origin - rotatedBotOrigin;

        var asm = api.GetAssembly("two_pipe_t");
        asm.SolveAfterEveryConstraint = false;
        AssemblyPart bodyA = asm.AddPart(pipe1, new Vec3D(0));
        AssemblyPart bodyB = asm.AddPart(pipe2, seedPos + new Vec3D(0, 25, 12), qAlign);
        asm.FixPart(bodyA);
        asm.SetCoincidentOriented(
            bodyA.AddPlaneDatum("pipe1-top_flange-ExtrudeTop"),
            bodyB.AddPlaneDatum("pipe2-bottom_flange-ExtrudeBottom"),
            oppositeNormals: false);
        asm.SetCoincidentOriented(
            bodyA.AddPlaneDatum("pipe1-top_flange-north"),
            bodyB.AddPlaneDatum("pipe2-bottom_flange-north"),
            oppositeNormals: false);

        SolveResult result = asm.SolveConstraintsDetailed();
        Dump("two_pipes_translation", result, bodyB.EvaluatePose());

        Assert.True(result.Converged, result.Message ?? "not converged");
        Assert.True(result.SumOfSquaredErrors < 1e-4, "sse=" + result.SumOfSquaredErrors);

        Vec3D worldBot = TransformMath.TransformPoint(bodyB.EvaluatePose(), bot.Origin);
        Vec3D nTop = TransformMath.TransformDirection(bodyA.EvaluatePose(), top.Normal);
        nTop.Normalize();
        double gap = Math.Abs(nTop.Dot(worldBot - top.Origin));
        Assert.InRange(gap, 0, PosTol);
    }

    private static void AddNamedSquareFlange(PlotterSketcher sketch, double plate)
    {
        double h = plate * 0.5;
        sketch.AddLine(new Vec2D(-h, -h), new Vec2D(h, -h)).Name = "south";
        sketch.AddLine(new Vec2D(h, -h), new Vec2D(h, h)).Name = "east";
        sketch.AddLine(new Vec2D(h, h), new Vec2D(-h, h)).Name = "north";
        sketch.AddLine(new Vec2D(-h, h), new Vec2D(-h, -h)).Name = "west";
    }

    private static Quaternion RotationMapping(Vec3D from, Vec3D to)
    {
        from.Normalize();
        to.Normalize();
        double dot = Vec3DOps.Dot(from, to);
        if (dot > 0.999999)
            return TransformMath.IdentityOrientation;
        if (dot < -0.999999)
        {
            Vec3D ortho = Vec3DOps.GetOrthoNormal(from);
            return new Quaternion(ortho.X, ortho.Y, ortho.Z, 0);
        }

        Vec3D axis = Vec3DOps.Cross(from, to);
        double s = Math.Sqrt((1.0 + dot) * 2.0);
        double invs = 1.0 / s;
        return new Quaternion(axis.X * invs, axis.Y * invs, axis.Z * invs, s * 0.5);
    }

    private static AnchorMesh BuildFlangedElbow(GeoAPI api, string name, bool galleryScale = false)
    {
        double horizontalLen = galleryScale ? 150.0 : 80.0;
        double arcRadius = galleryScale ? 260.0 : 70.0;
        double pipeRadius = galleryScale ? 145.0 : 22.0;
        double pipeInnerRadius = galleryScale ? 105.0 : 16.0;
        double plate = galleryScale ? 360.0 : 70.0;
        double thick = galleryScale ? 50.0 : 12.0;

        var guide = api.GetPlotterSketcher(DefaultPlanes.OriginXY, name + "-guide");
        guide.AddLine(new Vec2D(0, 0), new Vec2D(-horizontalLen, 0)).Name = "h_line";
        double mid = 5.0 * Math.PI / 4.0;
        var arcMid = new Vec2D(
            -horizontalLen + arcRadius * Math.Cos(mid),
            arcRadius + arcRadius * Math.Sin(mid));
        guide.AddArc(
            new Vec2D(-horizontalLen, 0),
            arcMid,
            new Vec2D(-horizontalLen - arcRadius, arcRadius)).Name = "bend";
        guide.AddLine(
            new Vec2D(-horizontalLen - arcRadius, arcRadius),
            new Vec2D(-horizontalLen - arcRadius, arcRadius + horizontalLen)).Name = "v_line";

        var profile = api.GetPlotterSketcher(DefaultPlanes.OriginYZ, name + "-annulus");
        profile.AddCircle(new Vec2D(0, 0), pipeRadius).Name = "outer";
        profile.AddCircle(new Vec2D(0, 0), pipeInnerRadius).Name = "inner";

        var pipe = api.ExtrudeAlongSketch(profile, guide, name: name);

        var bottomSk = api.GetPlotterSketcher(name + "-ExtrudeBottom", name + "-bottom_flange_profile");
        AddNamedSquareFlange(bottomSk, plate);
        bottomSk.AddCircle(new Vec2D(0, 0), 0.5 * (pipeRadius + pipeInnerRadius)).Name = "bore";
        var bottomExt = api.ExtrudeTwoSides(bottomSk, 0.0, thick, name: name + "-bottom_flange");

        var topSk = api.GetPlotterSketcher(name + "-ExtrudeTop", name + "-top_flange_profile");
        AddNamedSquareFlange(topSk, plate);
        topSk.AddCircle(new Vec2D(0, 0), 0.5 * (pipeRadius + pipeInnerRadius)).Name = "bore";
        var topExt = api.ExtrudeTwoSides(topSk, thick, 0.0, name: name + "-top_flange");

        pipe = api.Boolean(pipe, bottomExt, BooleanOp.Union, name);
        pipe = api.Boolean(pipe, topExt, BooleanOp.Union, name);
        return pipe;
    }
}
