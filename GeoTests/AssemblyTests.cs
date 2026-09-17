using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class AssemblyTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private const double PosTol = 0.5;
    private const double VolRelTol = 0.02;


    private static GeoAPI MakeApi() =>
        new GeoAPI(new Box3D(new Vec3D(-300), new Vec3D(300)), 0.01);

    private static AnchorMesh BuildHoledPlate(
        GeoAPI api,
        string name,
        double width,
        double height,
        double thickness,
        double holeRadius,
        Vec2D holeCenter)
    {
        var plateSk = api.GetPlotterSketcher(DefaultPlanes.OriginXY, name + "_profile");
        plateSk.AddRectangleFromCorners(Vec2DOps.Zero, new Vec2D(width, height));

        var solid = api.ExtrudeTwoSides(plateSk, thickness * 0.5, thickness * 0.5, name: name + "_solid");

        var holeSk = api.GetPlotterSketcher(DefaultPlanes.OriginXY, name + "_holes");
        holeSk.AddCircle(holeCenter, holeRadius);
        var cutter = api.ExtrudeTwoSides(holeSk, thickness * 0.5, thickness * 0.5, name: name + "_cutter");

        return api.Boolean(solid, cutter, BooleanOp.Difference, name);
    }

    private static string FindPlanarTopPatch(AnchorMesh mesh)
    {
        foreach (var kv in mesh.surfaceMetaData)
        {
            if (kv.Key.EndsWith("-ExtrudeTop", StringComparison.Ordinal) &&
                kv.Value.SurfaceType == SurfaceType.Planar)
                return kv.Key;
        }

        foreach (var kv in mesh.surfaceMetaData)
        {
            if (kv.Value.SurfaceType == SurfaceType.Planar)
                return kv.Key;
        }

        throw new InvalidOperationException($"No planar patch on mesh '{mesh.Name}'.");
    }

    private static string FindCylindricalPatch(AnchorMesh mesh)
    {
        foreach (var kv in mesh.surfaceMetaData)
        {
            if (kv.Value.SurfaceType == SurfaceType.Cylindrical && kv.Value.CylinderParams != null)
                return kv.Key;
        }

        throw new InvalidOperationException($"No cylindrical patch on mesh '{mesh.Name}'.");
    }

    private static double MeshVolume(AnchorMesh mesh)
    {
        mesh.EnsureCoplanarPostProcessed();
        return Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(mesh.Mesh.Positions, mesh.Mesh.Triangles));
    }

    [Fact]
    public void VisualOutputKind_AssemblyAfterAddPart_MeshAfterNewRegister()
    {
        var api = MakeApi();
        Assert.Equal(GeoVisualOutputKind.Mesh, api.VisualOutputKind);
        Assert.Null(api.GetTopAssembly());

        var plateA = BuildHoledPlate(api, "plateA", 100, 40, 10, 5, new Vec2D(15, 20));
        Assert.Equal(GeoVisualOutputKind.Mesh, api.VisualOutputKind);

        var asm = api.GetAssembly("link");
        var part = asm.AddPart(plateA, new Vec3D(0));
        Assert.Equal(GeoVisualOutputKind.Assembly, api.VisualOutputKind);
        Assert.Same(asm, api.GetTopAssembly());
        Assert.Single(asm.GetParts());

        var plateB = BuildHoledPlate(api, "plateB", 100, 40, 10, 5, new Vec2D(15, 20));
        Assert.Equal(GeoVisualOutputKind.Mesh, api.VisualOutputKind);

        asm.SolveConstraints();
        Assert.Equal(GeoVisualOutputKind.Assembly, api.VisualOutputKind);
    }

    [Fact]
    public void GetAssembly_RegistersOnGeoAPI_GetAssembliesReturnsLiveList()
    {
        var api = MakeApi();
        var asm = api.GetAssembly("bearing_link");
        var same = api.GetAssembly("bearing_link");

        Assert.Same(asm, same);
        Assert.Single(api.GetAssemblies());
        Assert.Same(asm, api.GetAssemblies()[0]);

        var other = api.GetAssembly("bracket");
        Assert.Equal(2, api.GetAssemblies().Count);
        Assert.Contains(asm, api.GetAssemblies());
        Assert.Contains(other, api.GetAssemblies());
    }

    [Fact]
    public void AddPart_ReturnsHandle_RejectUnregisteredMesh()
    {
        var api = MakeApi();
        var asm = api.GetAssembly("asm1");
        var otherApi = MakeApi();
        var plate = BuildHoledPlate(otherApi, "foreign", 100, 40, 10, 5, new Vec2D(20, 20));

        Assert.Throws<ArgumentException>(() => asm.AddPart(plate, new Vec3D(0)));
    }

    [Fact]
    public void AddPart_Ownership_RejectsForeignAssemblyPart()
    {
        var api = MakeApi();
        var plateA = BuildHoledPlate(api, "plateA", 100, 40, 10, 5, new Vec2D(15, 20));
        var plateB = BuildHoledPlate(api, "plateB", 100, 40, 10, 5, new Vec2D(15, 20));

        var asm1 = api.GetAssembly("asm1");
        var asm2 = api.GetAssembly("asm2");
        var part1 = asm1.AddPart(plateA, new Vec3D(0));
        var part2 = asm2.AddPart(plateB, new Vec3D(0));

        Assert.Throws<ArgumentException>(() => asm1.FixPart(part2));
        Assert.Throws<ArgumentException>(() => asm1.SetCoincident(part1.AddPointDatumAt(new Vec3D(0)), part2.AddPointDatumAt(new Vec3D(0))));
    }

    [Fact]
    public void FixPart_LocksPoseAndPreservesVolume()
    {
        var api = MakeApi();
        var plate = BuildHoledPlate(api, "plate", 100, 40, 10, 5, new Vec2D(15, 20));
        double volumeBefore = MeshVolume(plate);

        var asm = api.GetAssembly("asm");
        asm.SolveAfterEveryConstraint = false;
        var part = asm.AddPart(plate, new Vec3D(0));
        asm.FixPart(part);
        asm.SolveConstraints();

        double volumeAfter = MeshVolume(plate);
        Assert.InRange(volumeAfter, volumeBefore * (1 - VolRelTol), volumeBefore * (1 + VolRelTol));

        Transform pose = part.EvaluatePose();
        Assert.InRange(pose.Position.X, -PosTol, PosTol);
        Assert.InRange(pose.Position.Y, -PosTol, PosTol);
        Assert.InRange(pose.Position.Z, -PosTol, PosTol);
    }

    [Fact]
    public void AddAxisDatum_CylindricalPatch_ResolvesHoleAxis()
    {
        var api = MakeApi();
        var plate = BuildHoledPlate(api, "plate", 100, 40, 10, 5, new Vec2D(15, 20));
        string patch = FindCylindricalPatch(plate);

        var asm = api.GetAssembly("asm");
        var part = asm.AddPart(plate, new Vec3D(0));
        var axis = part.AddAxisDatum(patch);

        Assert.True(axis.LocalDirection.LengthSquared() > 0.5);
        Assert.True(axis.LocalPoint.LengthSquared() >= 0);
    }

    [Fact]
    public void AddAxisDatum_PlanarPatch_ResolvesNormal()
    {
        var api = MakeApi();
        var plate = BuildHoledPlate(api, "plate", 100, 40, 10, 5, new Vec2D(15, 20));

        var asm = api.GetAssembly("asm");
        var part = asm.AddPart(plate, new Vec3D(0));
        string topPatch = FindPlanarTopPatch(plate);
        var axis = part.AddAxisDatum(topPatch);

        Vec3D worldDir = TransformMath.TransformDirection(part.EvaluatePose(), axis.LocalDirection);
        worldDir.Normalize();
        Assert.InRange(Math.Abs(worldDir.Z), 0.99, 1.01);
    }

    [Fact]
    public void AddAxisDatum_CylindricalPatch_UsesMidAxisPoint()
    {
        var api = MakeApi();
        var plate = BuildHoledPlate(api, "plate", 100, 40, 10, 5, new Vec2D(15, 20));

        var asm = api.GetAssembly("asm");
        var part = asm.AddPart(plate, new Vec3D(0));
        var axis = part.AddAxisDatum("Circle1");
        Vec3D world = TransformMath.TransformPoint(part.EvaluatePose(), axis.LocalPoint);

        Assert.InRange(world.X, 14, 16);
        Assert.InRange(world.Y, 19, 21);
        Assert.InRange(world.Z, -0.5, 0.5);
    }

    [Fact]
    public void AddAxisDatum_ShortCurveName_ResolvesCylindricalPatch()
    {
        var api = MakeApi();
        var plate = BuildHoledPlate(api, "plate", 100, 40, 10, 5, new Vec2D(15, 20));

        var asm = api.GetAssembly("asm");
        var part = asm.AddPart(plate, new Vec3D(0));
        var axis = part.AddAxisDatum("Circle1");

        Assert.True(axis.LocalDirection.LengthSquared() > 0.5);
    }

    [Fact]
    public void AddAxisDatum_UnknownReference_Throws()
    {
        var api = MakeApi();
        var plate = BuildHoledPlate(api, "plate", 100, 40, 10, 5, new Vec2D(15, 20));

        var asm = api.GetAssembly("asm");
        var part = asm.AddPart(plate, new Vec3D(0));

        Assert.Throws<ArgumentException>(() => part.AddAxisDatum("plate-NoSuchPatch"));
    }

    [Fact]
    public void Solve_UpdatesMeshInPlace_PreservesTopologyAndVolume()
    {
        var api = MakeApi();
        var plate = BuildHoledPlate(api, "plate", 100, 40, 10, 5, new Vec2D(15, 20));
        double volumeBefore = MeshVolume(plate);
        int triCountBefore = plate.Mesh.Triangles.Count;
        var positionsBefore = plate.Mesh.Positions.ToList();

        var asm = api.GetAssembly("asm");
        asm.SolveAfterEveryConstraint = false;
        var part = asm.AddPart(plate, new Vec3D(0));
        asm.FixPart(part, new Vec3D(30, 0, 0), TransformMath.IdentityOrientation);
        asm.SolveConstraints();

        Assert.Equal(triCountBefore, plate.Mesh.Triangles.Count);
        double volumeAfter = MeshVolume(plate);
        Assert.InRange(volumeAfter, volumeBefore * (1 - VolRelTol), volumeBefore * (1 + VolRelTol));

        bool moved = false;
        for (int i = 0; i < positionsBefore.Count; i++)
        {
            if (Vec3DOps.DistanceSquared(positionsBefore[i], plate.Mesh.Positions[i]) > 1e-6)
            {
                moved = true;
                break;
            }
        }
        Assert.True(moved, "Expected mesh vertices to move after solve.");
        Assert.InRange(plate.Mesh.Positions[0].X, 29, 31);
    }

    [Fact]
    public void BearingPlateAssembly_LJoint_MatesLeftHoles()
    {
        const double plateW = 200;
        const double plateH = 40;
        const double thickness = 10;
        const double holeInset = 20;

        var api = MakeApi();
        var plateA = BuildBearingPlate(api, "plate_a", "plateA", plateW, plateH, thickness, holeInset);
        var plateB = BuildBearingPlate(api, "plate_b", "plateB", plateW, plateH, thickness, holeInset);

        double volA = MeshVolume(plateA);
        double volB = MeshVolume(plateB);

        var asm = api.GetAssembly("bearing_link");
        asm.SolveAfterEveryConstraint = false;

        var bodyA = asm.AddPart(plateA, new Vec3D(0));
        var bodyB = asm.AddPart(
            plateB,
            new Vec3D(40, 0, 0),
            new Quaternion(0, 0, 0.70710678, 0.70710678));

        var jointA = bodyA.AddAxisDatum("Circle1");
        var jointB = bodyB.AddAxisDatum("Circle1");

        asm.FixPart(bodyA);
        asm.SetCoincident(jointA, jointB);
        asm.SetParallel(jointA, jointB);
        asm.SetCoincident(
            bodyA.AddPointDatumAt(new Vec3D(0)),
            bodyB.AddPointDatumAt(new Vec3D(0, plateH, 0)));
        asm.SolveConstraints();

        Vec3D holeA = TransformMath.TransformPoint(bodyA.EvaluatePose(), jointA.LocalPoint);
        Vec3D holeB = TransformMath.TransformPoint(bodyB.EvaluatePose(), jointB.LocalPoint);
        Assert.InRange(Math.Sqrt(Vec3DOps.DistanceSquared(holeA, holeB)), 0, PosTol);
        Assert.InRange(holeA.X, holeInset - PosTol, holeInset + PosTol);
        Assert.InRange(holeA.Y, plateH * 0.5 - PosTol, plateH * 0.5 + PosTol);

        Vec3D nA = TransformMath.TransformDirection(bodyA.EvaluatePose(), new Vec3D(0, 0, 1));
        Vec3D nB = TransformMath.TransformDirection(bodyB.EvaluatePose(), new Vec3D(0, 0, 1));
        nA.Normalize();
        nB.Normalize();
        Assert.InRange(Math.Abs(nA.Dot(nB)), 0.99, 1.01);

        Vec3D centroidA = MeshCentroid(plateA);
        Vec3D centroidB = MeshCentroid(plateB);
        Assert.True(centroidB.Y > centroidA.Y + plateH, "Plate B should extend along +Y from the elbow.");
        Assert.True(centroidB.X < plateW * 0.25, "Plate B should attach near the left end of plate A, not at mid-span.");

        Assert.InRange(MeshVolume(plateA), volA * (1 - VolRelTol), volA * (1 + VolRelTol));
        Assert.InRange(MeshVolume(plateB), volB * (1 - VolRelTol), volB * (1 + VolRelTol));
        Assert.True(asm.GetMateRecords().Count >= 4, "Expected mate records for L-joint assembly.");
    }

    [Fact]
    public void ClosedSquareLoop_FourElbows_OppositeFlangeAndClocking()
    {
        const double arm = 40;
        var api = MakeApi();
        var asm = api.GetAssembly("elbow_ring");
        asm.SolveAfterEveryConstraint = false;

        AssemblyPart[] bodies = new AssemblyPart[4];
        for (int i = 0; i < 4; i++)
        {
            var mesh = api.CreateCuboid(new Vec3D(0), new Vec3D(8, 8, 8), "elbow" + i);
            bodies[i] = asm.AddPart(mesh, new Vec3D(0));
        }

        asm.FixPart(bodies[0]);

        AssemblyPlaneDatum Inlet(AssemblyPart part) =>
            part.AddPlaneDatumAt(new Vec3D(0), new Vec3D(-1, 0, 0));
        AssemblyPlaneDatum Outlet(AssemblyPart part) =>
            part.AddPlaneDatumAt(new Vec3D(arm, arm, 0), new Vec3D(0, 1, 0));
        AssemblyPlaneDatum North(AssemblyPart part) =>
            part.AddPlaneDatumAt(new Vec3D(0), new Vec3D(0, 0, 1));

        void MateJoint(int outgoing, int incoming)
        {
            asm.SetCoincidentOriented(Outlet(bodies[outgoing]), Inlet(bodies[incoming]), oppositeNormals: true);
            asm.SetCoincidentOriented(North(bodies[outgoing]), North(bodies[incoming]), oppositeNormals: false);
            asm.SetCoincident(
                bodies[outgoing].AddPointDatumAt(new Vec3D(arm, arm, 0)),
                bodies[incoming].AddPointDatumAt(new Vec3D(0)));
        }

        for (int i = 1; i < 4; i++)
        {
            MateJoint(i - 1, i);
            asm.SolveConstraints();
        }

        MateJoint(3, 0);
        asm.SolveConstraints();

        Vec3D[] origin = new Vec3D[4];
        for (int i = 0; i < 4; i++)
            origin[i] = bodies[i].EvaluatePose().Position;

        double side = Math.Sqrt(2.0) * arm;
        Assert.InRange(Math.Sqrt(Vec3DOps.DistanceSquared(origin[0], origin[1])), side - PosTol, side + PosTol);
        Assert.InRange(Math.Sqrt(Vec3DOps.DistanceSquared(origin[1], origin[2])), side - PosTol, side + PosTol);
        Assert.InRange(Math.Sqrt(Vec3DOps.DistanceSquared(origin[2], origin[3])), side - PosTol, side + PosTol);
        Assert.InRange(Math.Sqrt(Vec3DOps.DistanceSquared(origin[3], origin[0])), side - PosTol, side + PosTol);

        Assert.InRange(origin[1].X, arm - PosTol, arm + PosTol);
        Assert.InRange(origin[1].Y, arm - PosTol, arm + PosTol);
        Assert.InRange(origin[2].X, -PosTol, PosTol);
        Assert.InRange(origin[2].Y, 2 * arm - PosTol, 2 * arm + PosTol);
        Assert.InRange(origin[3].X, -arm - PosTol, -arm + PosTol);
        Assert.InRange(origin[3].Y, arm - PosTol, arm + PosTol);

        for (int i = 0; i < 4; i++)
            Assert.InRange(origin[i].Z, -PosTol, PosTol);
    }

    [Fact]
    public void SetConcentric_PinAndHole_AxisLinesAreCoaxial()
    {
        var api = MakeApi();
        var plate = BuildBearingPlate(api, "plate", "plateA", 200, 40, 10, 20);
        var pin = api.CreateCylinder(
            new CoordinateSystem(new Vec3D(0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1)),
            4.5, 12, name: "pin");

        var asm = api.GetAssembly("asm");
        asm.SolveAfterEveryConstraint = false;
        var bodyPlate = asm.AddPart(plate, new Vec3D(0));
        var bodyPin = asm.AddPart(pin, new Vec3D(50, 0, 0));
        asm.FixPart(bodyPlate);

        var hole = bodyPlate.AddAxisDatum("Circle1");
        var pinAxis = bodyPin.AddAxisDatumAt(new Vec3D(0, 0, 6), new Vec3D(0, 0, 1));
        asm.SetConcentric(pinAxis, hole);
        asm.SolveConstraints();

        Vec3D holeWorld = TransformMath.TransformPoint(bodyPlate.EvaluatePose(), hole.LocalPoint);
        Vec3D pinWorld = TransformMath.TransformPoint(bodyPin.EvaluatePose(), pinAxis.LocalPoint);
        Vec3D holeDirection = TransformMath.TransformDirection(bodyPlate.EvaluatePose(), hole.LocalDirection);
        holeDirection.Normalize();
        double lineDistance = Vec3DOps.Cross(pinWorld - holeWorld, holeDirection).Length();
        Assert.InRange(lineDistance, 0, PosTol);
    }

    [Fact]
    public void SetDistance_Points_KeepsSeparation()
    {
        var api = MakeApi();
        var a = api.CreateCube(CoordinateSystem.Default, 10, "a");
        var b = api.CreateCube(CoordinateSystem.Default, 10, "b");

        var asm = api.GetAssembly("asm");
        asm.SolveAfterEveryConstraint = false;
        var bodyA = asm.AddPart(a, new Vec3D(0));
        var bodyB = asm.AddPart(b, new Vec3D(40, 0, 0));
        asm.FixPart(bodyA);

        var pA = bodyA.AddPointDatumAt(new Vec3D(10, 5, 5));
        var pB = bodyB.AddPointDatumAt(new Vec3D(0, 5, 5));
        asm.SetDistance(pA, pB, 25);
        asm.SolveConstraints();

        Vec3D wA = TransformMath.TransformPoint(bodyA.EvaluatePose(), pA.LocalPoint);
        Vec3D wB = TransformMath.TransformPoint(bodyB.EvaluatePose(), pB.LocalPoint);
        Assert.InRange(Math.Sqrt(Vec3DOps.DistanceSquared(wA, wB)), 24, 27);
    }

    [Fact]
    public void SetPerpendicular_Axes_Orthogonal()
    {
        var api = MakeApi();
        var a = api.CreateCube(CoordinateSystem.Default, 10, "a");
        var b = api.CreateCube(CoordinateSystem.Default, 10, "b");

        var asm = api.GetAssembly("asm");
        asm.SolveAfterEveryConstraint = false;
        var bodyA = asm.AddPart(a, new Vec3D(0));
        var bodyB = asm.AddPart(b, new Vec3D(20, 0, 0));
        asm.FixPart(bodyA);

        var axisA = bodyA.AddAxisDatumAt(new Vec3D(5, 5, 5), new Vec3D(1, 0, 0));
        var axisB = bodyB.AddAxisDatumAt(new Vec3D(5, 5, 5), new Vec3D(0, 0, 1));
        asm.SetPerpendicular(axisA, axisB);
        asm.SolveConstraints();

        Vec3D dirA = TransformMath.TransformDirection(bodyA.EvaluatePose(), axisA.LocalDirection);
        Vec3D dirB = TransformMath.TransformDirection(bodyB.EvaluatePose(), axisB.LocalDirection);
        dirA.Normalize();
        dirB.Normalize();
        Assert.InRange(Math.Abs(Vec3DOps.Dot(dirA, dirB)), 0, 0.05);
    }

    [Fact]
    public void SetCoincident_Planes_FlushFaces()
    {
        var api = MakeApi();
        var basePlate = BuildBearingPlate(api, "base", "base", 100, 40, 10, 20);
        var post = api.CreateCuboid(new Vec3D(0), new Vec3D(20, 20, 40), "post");

        var asm = api.GetAssembly("asm");
        asm.SolveAfterEveryConstraint = false;
        var bodyBase = asm.AddPart(basePlate, new Vec3D(0));
        var bodyPost = asm.AddPart(post, new Vec3D(10, 10, 30));
        asm.FixPart(bodyBase);

        var top = bodyBase.AddPlaneDatumAt(new Vec3D(50, 20, 5), new Vec3D(0, 0, 1));
        var bottom = bodyPost.AddPlaneDatumAt(new Vec3D(10, 10, 0), new Vec3D(0, 0, -1));
        asm.SetCoincident(top, bottom);
        asm.SolveConstraints();

        Vec3D topOrigin = TransformMath.TransformPoint(bodyBase.EvaluatePose(), top.LocalOrigin);
        Vec3D bottomOrigin = TransformMath.TransformPoint(bodyPost.EvaluatePose(), bottom.LocalOrigin);
        Assert.InRange(Math.Abs(topOrigin.Z - bottomOrigin.Z), 0, PosTol);
    }

    [Fact]
    public void SetContact_PenetratingPoint_ImprovesOrClosesGap()
    {
        var api = MakeApi();
        var plate = api.CreateCuboid(new Vec3D(0), new Vec3D(40, 40, 4), "contact_plate");
        var block = api.CreateCuboid(new Vec3D(0), new Vec3D(10, 10, 10), "contact_block");

        var asm = api.GetAssembly("contact_asm");
        asm.SolveAfterEveryConstraint = false;
        var bodyPlate = asm.AddPart(plate, new Vec3D(0));
        var bodyBlock = asm.AddPart(block, new Vec3D(15, 15, -8));
        asm.FixPart(bodyPlate);

        var plane = bodyPlate.AddPlaneDatumAt(new Vec3D(20, 20, 2), new Vec3D(0, 0, 1));
        var point = bodyBlock.AddPointDatumAt(new Vec3D(5, 5, 0));
        asm.SetContact(point, plane);

        double Gap()
        {
            Vec3D worldPoint = TransformMath.TransformPoint(bodyBlock.EvaluatePose(), point.LocalPoint);
            Vec3D worldOrigin = TransformMath.TransformPoint(bodyPlate.EvaluatePose(), plane.LocalOrigin);
            Vec3D worldNormal = TransformMath.TransformDirection(bodyPlate.EvaluatePose(), plane.LocalNormal);
            worldNormal = worldNormal / worldNormal.Length();
            return worldNormal.Dot(worldPoint - worldOrigin);
        }

        double gap0 = Gap();
        Assert.True(gap0 < 0);

        asm.SolveConstraints();

        Assert.Contains(asm.GetMateRecords(), m => m.Kind == AssemblyMateKind.Contact);
        double gap = Gap();
        // Full closure is covered by kinematics contact tests; Assembly API must reduce penetration.
        Assert.True(gap >= gap0 - 1e-9, $"Expected penetration not to worsen: before={gap0}, after={gap}");
        Assert.True(gap > gap0 + 0.1 || gap >= -PosTol, $"Expected gap improvement: before={gap0}, after={gap}");
    }

    private static AnchorMesh BuildBearingPlate(
        GeoAPI api,
        string prefix,
        string resultName,
        double width,
        double height,
        double thickness,
        double holeInset)
    {
        var profileSk = api.GetPlotterSketcher(DefaultPlanes.OriginXY, prefix + "_profile");
        profileSk.AddRectangleFromCorners(Vec2DOps.Zero, new Vec2D(width, height));
        var solid = api.ExtrudeTwoSides(profileSk, thickness * 0.5, thickness * 0.5, name: prefix + "_solid");

        var holeSk = api.GetPlotterSketcher(DefaultPlanes.OriginXY, prefix + "_holes");
        holeSk.AddCircle(new Vec2D(holeInset, height * 0.5), 5);
        holeSk.AddCircle(new Vec2D(width - holeInset, height * 0.5), 5);
        var cutter = api.ExtrudeTwoSides(holeSk, thickness * 0.5, thickness * 0.5, name: prefix + "_cutter");

        return api.Boolean(solid, cutter, BooleanOp.Difference, resultName);
    }

    private static Vec3D MeshCentroid(AnchorMesh mesh)
    {
        var c = new Vec3D(0);
        foreach (var p in mesh.Mesh.Positions)
            c += p;
        return c / mesh.Mesh.Positions.Count;
    }
}
