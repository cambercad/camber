using Geo;
using GeoCore;

namespace GeoTests;

public class AssemblyPlaneFrameTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void LargeComponentPlaneOffsetsConvergeWithoutRelaxingResidualTolerance()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-500), new Vec3D(1000)), .01);
        var board = api.CreateCuboid(new Vec3D(-250, -225, 758), new Vec3D(650, 225, 798), "board");
        var plate = api.CreateCuboid(new Vec3D(120, -55, 750), new Vec3D(280, 55, 758), "plate");
        var assembly = api.GetAssembly("mount");
        var grounded = assembly.AddPart(board, new Vec3D(0));
        var moving = assembly.AddPart(plate, new Vec3D(0));
        assembly.FixPart(grounded);
        assembly.SetCoincidentOriented(
            moving.AddPlaneDatumAt(new Vec3D(250.7847505355, -40.5550405956, 757.9988578569), new Vec3D(0, 0, 1)),
            grounded.AddPlaneDatumAt(new Vec3D(295.4999514099, -44.5000953901, 757.9999231599), new Vec3D(0, 0, -1)),
            oppositeNormals: true);
        assembly.SetDistance(
            moving.AddPlaneDatumAt(new Vec3D(279.9987797788, -55.0010550561, 749.9988498488), new Vec3D(1, 0, 0)),
            grounded.AddPlaneDatumAt(new Vec3D(309.9999662, 69.9999213999, 757.9999231599), new Vec3D(-1, 0, 0)), 30);
        var result = assembly.SolveConstraintsDetailed();
        Assert.True(result.Converged, result.Message);
        Assert.All(assembly.GetMateResiduals(), residual => Assert.True(residual.Satisfied, residual.Label));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NamedPointAndPlanarAxisRemainLocalAfterSharedBodyMaterialization(bool imported)
    {
        var source = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(80)), .01);
        var target = imported ? new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01) : source;
        var solid = source.CreateCuboid(new Vec3D(0), new Vec3D(2, 3, 4), "block");
        var assembly = target.GetAssembly("references");
        var first = assembly.AddPart(source, solid, new Vec3D(10, 20, 30));
        var second = assembly.AddPart(source, solid, new Vec3D(-20, 10, 5));
        var mesh = first.Mesh;
        var edge = mesh.GroupEdges[0].Name + "@0.250";
        var uv = GeoMeta.EntityNaming.FormatSurfacePointAddress("block-ExtrudeTop", .25, .5);
        var references = new[] { edge, uv, "block-ExtrudeTop" };
        var points = references.Select(reference => first.AddPointDatum(reference).LocalPoint).ToArray();
        var axis = first.AddAxisDatum("block-ExtrudeTop");
        assembly.FixPart(first);
        Assert.True(assembly.SolveConstraintsDetailed().Converged);
        mesh.Update(new Transform(new Vec3D(10, 20, 30), new Quaternion(0, Math.Sqrt(.5), 0, Math.Sqrt(.5))));
        foreach (var part in new[] { first, second })
        {
            for (int i = 0; i < references.Length; i++)
                Assert.InRange((points[i] - part.AddPointDatum(references[i]).LocalPoint).Length(), 0, 1e-12);
            var current = part.AddAxisDatum("block-ExtrudeTop");
            Assert.InRange((axis.LocalPoint - current.LocalPoint).Length(), 0, 1e-12);
            Assert.InRange((axis.LocalDirection - current.LocalDirection).Length(), 0, 1e-12);
        }
    }

    [Fact]
    public void HingedPlateDistanceRemainsAccurateWhenContactIsPresent()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01);
        var solid = api.CreateCuboid(new Vec3D(0), new Vec3D(80, 60, 6), "plate");
        var assembly = api.GetAssembly("hinge");
        assembly.SolveAfterEveryConstraint = false;
        var grounded = assembly.AddPart(solid, new Vec3D(0));
        var moving = assembly.AddPart(solid, new Vec3D(92, 0, 0),
            new Quaternion(Math.Sin(Math.PI/18), 0, 0, Math.Cos(Math.PI/18)));
        assembly.FixPart(grounded);
        assembly.SetParallel(grounded.AddAxisDatumAt(new Vec3D(0), new Vec3D(1, 0, 0)),
            moving.AddAxisDatumAt(new Vec3D(0), new Vec3D(1, 0, 0)));
        assembly.SetCoincident(grounded.AddPointDatumAt(new Vec3D(-40, -30, 6)),
            moving.AddPointDatumAt(new Vec3D(-40, -30, 0)));
        var plane = grounded.AddPlaneDatumAt(new Vec3D(0, 0, 6), new Vec3D(0, 0, 1));
        var back = moving.AddPointDatumAt(new Vec3D(-40, 30, 0));
        assembly.SetContact(back, plane);
        assembly.SetPointOnPlane(moving.AddPointDatumAt(new Vec3D(40, -30, 0)), plane);
        assembly.SetDistance(grounded.AddPointDatumAt(new Vec3D(-40, 30, 6)), back, 31.0582854123);
        Assert.True(assembly.SolveConstraintsDetailed().Converged);
        Assert.All(assembly.GetMateResiduals(), residual => Assert.True(residual.Satisfied, residual.Label));
    }


    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(false, false, true)]
    public void LocalFramePreservesAuthoredAxisOrUsesStableFallbackAfterDisplay(
        bool missingMetadata, bool degenerateDirection, bool nullMetadata = false)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01);
        var solid = api.CreateCuboid(new Vec3D(0), new Vec3D(2, 3, 4), "block");
        const string topName = "block-ExtrudeTop";
        var authoredDirection = new Vec3D(.6, .8, 0);
        solid.surfaceMetaData[topName].PlaneParams.RefDir =
            degenerateDirection ? new Vec3D(0, 0, 1) : authoredDirection;
        if (missingMetadata) solid.surfaceMetaData.Remove(topName);
        if (nullMetadata) solid.surfaceMetaData[topName] = null;
        var assembly = api.GetAssembly("frames");
        var first = assembly.AddPart(solid, new Vec3D(10, 20, 30));
        var before = first.GetPlaneFrame(topName);
        var pose = new Transform(new Vec3D(10, 20, 30), new Quaternion(0, Math.Sqrt(.5), 0, Math.Sqrt(.5)));
        solid.Update(pose);
        var second = assembly.AddPart(solid, new Vec3D(-20, 10, 5));
        foreach (var body in new[] { first, second })
        {
            var after = body.GetPlaneFrame(topName);
            Assert.Equal(before.Origin, after.Origin);
            Assert.Equal(before.X, after.X);
            Assert.Equal(before.Y, after.Y);
            Assert.Equal(before.Z, after.Z);
            Assert.InRange((after.Z - new Vec3D(0, 0, 1)).Length(), 0, 1e-12);
            Assert.InRange(Math.Abs(after.X.LengthSquared() - 1), 0, 1e-12);
            Assert.InRange(Math.Abs(Vec3DOps.Dot(after.X, after.Z)), 0, 1e-12);
            Assert.InRange((Vec3DOps.Cross(after.X, after.Y) - after.Z).Length(), 0, 1e-12);
            if (!missingMetadata && !degenerateDirection && !nullMetadata)
                Assert.InRange((after.X - authoredDirection).Length(), 0, 1e-12);
            var bottom = body.GetPlaneFrame("block-ExtrudeBottom");
            Assert.InRange((bottom.Z - new Vec3D(0, 0, -1)).Length(), 0, 1e-12);
        }
    }
}
