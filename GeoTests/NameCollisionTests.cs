using Curves;
using Geo;
using GeoCore;
using GeoMeta;
using GeoSolver.Sketcher;

namespace GeoTests;

public sealed class NameCollisionTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void RegisterMesh_ReplacesExistingMeshWithSameName()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), maxDeviation: 0.05);
        var first = api.CreateCube(CoordinateSystem.Default, 1.0, name: "dup");
        var second = api.CreateCube(CoordinateSystem.Default, 2.0, name: "dup");

        Assert.Same(second, api.GetMeshFromName("dup"));
        Assert.Equal(1, api.GetMeshes().Count);
        Assert.DoesNotContain(first, api.GetMeshes());
    }

    [Fact]
    public void RegisterSketch_ThrowsOnDuplicateSketchName()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), maxDeviation: 0.05);
        api.GetPlotterSketcher(DefaultPlanes.OriginXY, "same");
        var ex = Assert.Throws<NameCollisionException>(() => api.GetPlotterSketcher(DefaultPlanes.OriginXY, "same"));
        Assert.Contains("same", ex.Message);
    }

    [Fact]
    public void Rename_ReplacesExistingMeshWithSameName()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), maxDeviation: 0.05);

        AnchorMesh MakeBox(string meshName)
        {
            var sk = api.GetPlotterSketcher(DefaultPlanes.OriginXY, meshName + "_sk");
            sk.AddLine(new Vec2D(0, 0), new Vec2D(2, 0));
            sk.AddLine(new Vec2D(2, 0), new Vec2D(2, 2));
            sk.AddLine(new Vec2D(2, 2), new Vec2D(0, 2));
            sk.AddLine(new Vec2D(0, 2), new Vec2D(0, 0));
            return api.Extrude(sk, 1.0, name: meshName);
        }

        var meshA = MakeBox("uniqueA");
        var meshB = MakeBox("uniqueB");
        meshA.Rename("uniqueB");

        Assert.Equal("uniqueB", meshA.Name);
        Assert.Same(meshA, api.GetMeshFromName("uniqueB"));
        Assert.Equal(1, api.GetMeshes().Count);
        Assert.DoesNotContain(meshB, api.GetMeshes());
    }

    [Fact]
    public void TryGetCurveFromSketch_ThrowsWhenCurveNameAmbiguous()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), maxDeviation: 0.05);

        var sk1 = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "sk1");
        sk1.AddCLine(new Vec2D(0, 0), new Vec2D(1, 0));
        var sk2 = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "sk2");
        sk2.AddCLine(new Vec2D(0, 0), new Vec2D(1, 0));

        Assert.Throws<NameCollisionException>(() => api.TryGetCurveFromSketch("Line1", out _));
        Assert.True(api.TryGetCurveFromSketch("sk1:Line1", out _));
    }

    [Fact]
    public void TryGetPointOnSketchFromName_QualifiedAndAmbiguous()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), maxDeviation: 0.05);

        var sk1 = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "sk1");
        sk1.AddLine(new Vec2D(0, 0), new Vec2D(1, 0));
        var sk2 = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "sk2");
        sk2.AddLine(new Vec2D(0, 0), new Vec2D(1, 0));

        Assert.Throws<NameCollisionException>(() => api.TryGetPointOnSketchFromName("Line1@0.5", out _));
        Assert.True(api.TryGetPointOnSketchFromName("sk1:Line1@0.5", out var pt));
        Assert.Equal(0.5, pt.X, 3);
        Assert.Equal(0.0, pt.Y, 3);
    }

    [Fact]
    public void TryGetCPointOnSketchFromName_QualifiedAndAmbiguous()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), maxDeviation: 0.05);

        var sk1 = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "sk1");
        sk1.AddCLine(new Vec2D(0, 0), new Vec2D(1, 0));
        var sk2 = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "sk2");
        sk2.AddCLine(new Vec2D(0, 0), new Vec2D(1, 0));

        Assert.Throws<NameCollisionException>(() => api.TryGetCPointOnSketchFromName("Line1@0.5", out _));
        Assert.True(api.TryGetCPointOnSketchFromName("sk1:Line1@0.5", out _));
    }

    [Fact]
    public void BuildNameToGroupId_ThrowsOnDuplicatePatchName()
    {
        var groupToName = new Dictionary<int, string>
        {
            { 1, "same-patch" },
            { 2, "same-patch" },
        };
        Assert.Throws<NameCollisionException>(() => EntityNaming.BuildNameToGroupId(groupToName));
    }
}
