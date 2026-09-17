using CSG;
using Geo;
using Geo.BRep;
using GeoCore;

namespace GeoTests;

public class NurbsBooleanTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void Boolean_PreservesNurbsOnResultPatches()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var a = api.CreateCube(CoordinateSystem.Default, 1.0, "a");
        var b = api.CreateCube(new CoordinateSystem(new Vec3D(0.5, 0, 0)), 1.0, "b");

        Assert.True(a.surfaceMetaData.Values.Any(m => m.HasNurbs));

        var result = api.Boolean(a, b, BooleanOp.Union, "union");
        int withNurbs = result.surfaceMetaData.Values.Count(m => m.HasNurbs);
        Assert.True(withNurbs > 0);
    }

    [Fact]
    public void Boolean_Union_HasExtractableTrimLoops()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var a = api.CreateCube(CoordinateSystem.Default, 1.0, "a");
        var b = api.CreateCube(new CoordinateSystem(new Vec3D(0.3, 0, 0)), 1.0, "b");
        var result = api.Boolean(a, b, BooleanOp.Union, "union");

        var solid = BRepAssembler.BuildFromAnchorMesh(result);
        Assert.True(solid.Shell.Faces.Count > 0);
        foreach (var face in solid.Shell.Faces)
            Assert.True(face.Loops.Count > 0);
    }

    [Fact]
    public void Boolean_Difference_PreservesNurbsOnSurvivingPatches()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 2.0, "cube");
        var cyl = api.CreateCylinder(new CoordinateSystem(new Vec3D(1, 1, -0.5)), 0.4, 3.0, 0.01, "cyl");
        var diff = api.Boolean(cube, cyl, BooleanOp.Difference, "diff");

        var groups = diff.Mesh.GetTriangleGroups().Distinct().ToHashSet();
        int survivingWithNurbs = diff.extendedNameToGroupId.Count(kv =>
            groups.Contains(kv.Value)
            && diff.surfaceMetaData.TryGetValue(kv.Key, out var m)
            && m.HasNurbs);

        Assert.True(survivingWithNurbs > 0);
        var solid = BRepAssembler.BuildFromAnchorMesh(diff);
        Assert.True(solid.Shell.Faces.Count(f => f.Surface != null) >= survivingWithNurbs);
    }
}
