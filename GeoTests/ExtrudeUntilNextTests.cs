using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public sealed class ExtrudeUntilNextTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void FindExactPositionComponents_WeldsOnlyExactRationalSharedEdges()
    {
        var points = new List<Rat3Hybrid>
        {
            new(0, 0, 0), new(1, 0, 0), new(0, 1, 0),
            new(1, 0, 0), new(0, 0, 0), new(0, 0, 1),
            new(1000001, 1000000, 0), new(0, 0, 0), new(0, 1, 1)
        };
        var triangles = new List<Tri>
        {
            new(0, 1, 2), new(3, 4, 5), new(6, 7, 8)
        };

        var components = MeshConnectivity.FindExactPositionComponents(points, triangles);

        Assert.Equal(2, components.Count);
        Assert.Contains(components, component => component.Count == 2);
        Assert.Contains(components, component => component.Count == 1);
    }

    [Fact]
    public void ExtrudeUntilNext_TrimsAtNearestPlanarFace_AndIsWatertight()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(12)), 1e-4);
        var target = api.CreateCuboid(new Vec3D(-2, -2, 5), new Vec3D(2, 2, 10), "target");
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "profile");
        sketch.AddRectangle(new Vec2D(0, 0), 1, 1);
        var result = api.ExtrudeUntilNext(sketch, target, name: "until_next");

        Assert.True(result.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.Equal(5.0, SignedVolume(result), 3);

        Assert.InRange(result.Mesh.Positions.Min(p => p.Z), -1e-4, 1e-4);
        double maxZ = result.Mesh.Positions.Max(p => p.Z);
        Assert.True(maxZ >= 4.999 && maxZ <= 5.001,
            $"max z={maxZ:G17}; triangles={result.Mesh.Triangles.Count}; groups={string.Join(",", result.groupIdToExtendedName.Values)}");
    }

    [Fact]
    public void ExtrudeUntilNext_ThrowsWhenNoTargetFaceLiesAlongNormal()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(12)), 1e-4);
        var target = api.CreateCuboid(new Vec3D(2, 2, 5), new Vec3D(4, 4, 7), "target");
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "profile");
        sketch.AddRectangle(new Vec2D(0, 0), 1, 1);

        var error = Assert.Throws<InvalidOperationException>(() => api.ExtrudeUntilNext(sketch, target));
        Assert.Contains("no target face", error.Message);
    }

    [Fact]
    public void ExtrudeUntilNext_SearchesOppositeSketchNormalWhenThatFaceIsNearest()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-6), new Vec3D(12)), 1e-4);
        var target = api.CreateCuboid(new Vec3D(-2, -2, -2), new Vec3D(2, 2, 2), "target");
        var frame = new CoordinateSystem(new Vec3D(5, 0, 0),
            new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
        var sketch = api.GetPlotterSketcher(frame, "profile");
        sketch.AddRectangle(new Vec2D(-0.5, -0.5), 1, 1);

        var result = api.ExtrudeUntilNext(sketch, target);

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.Equal(3.0, SignedVolume(result), 3);
        Assert.InRange(result.Mesh.Positions.Min(point => point.X), 1.999, 2.001);
        Assert.InRange(result.Mesh.Positions.Max(point => point.X), 4.999, 5.001);
    }

    [Fact]
    public void ExtrudeUntilSurface_UsesSelectedFaceAndProducesWatertightSolid()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(12)), 1e-4);
        var target = api.CreateCuboid(new Vec3D(-2, -2, 5), new Vec3D(2, 2, 10), "target");
        string bottomFace = null;
        var groups = target.Mesh.GetTriangleGroups();
        for (int i = 0; i < target.Mesh.Triangles.Count; i++)
        {
            var tri = target.Mesh.Triangles[i];
            var points = target.Mesh.Positions;
            if (Math.Abs(points[tri.A].Z - 5) > 1e-4 ||
                Math.Abs(points[tri.B].Z - 5) > 1e-4 ||
                Math.Abs(points[tri.C].Z - 5) > 1e-4) continue;
            bottomFace = target.groupIdToExtendedName[groups[i]];
            break;
        }
        Assert.NotNull(bottomFace);
        var surface = api.ExtractFaceSurface(target, bottomFace);
        Assert.False(surface.IsVolume);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "profile");
        sketch.AddRectangle(new Vec2D(0, 0), 1, 1);

        var prism = api.Extrude(sketch, 10, name: "full_prism");
        foreach (var op in new[] {
            BooleanOp.AAsVolumeBAsTrimSurfaceKeepInTriNormalDirection,
            BooleanOp.AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection })
        {
            var half = api.Boolean(prism, surface, op);
            Assert.True(MeshAnalysis.IsWatertightMesh(half.Mesh.PrecisionPositions, half.Mesh.Triangles));
            Assert.Equal(5.0, SignedVolume(half), 3);
        }

        var result = api.ExtrudeUntilSurface(sketch, surface);

        Assert.True(result.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.Equal(5.0, SignedVolume(result), 3);

        var byName = api.ExtrudeUntilFace(sketch, target, bottomFace);
        Assert.True(MeshAnalysis.IsWatertightMesh(byName.Mesh.PrecisionPositions, byName.Mesh.Triangles));
        Assert.Equal(5.0, SignedVolume(byName), 3);
    }

    [Fact]
    public void ExtrudeUntilNext_CadQueryGalleryHalfTorus()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50), new Vec3D(50)), 0.05);
        var targetFrame = new CoordinateSystem(new Vec3D(0),
            new Vec3D(0, -1, 0), new Vec3D(1, 0, 0), new Vec3D(0, 0, 1));
        var targetProfile = api.GetPlotterSketcher(targetFrame, "gallery24_target_profile");
        targetProfile.AddCircle(new Vec2D(0, 20), 2);
        var target = api.Revolve(targetProfile, Math.PI, name: "gallery24_target");
        var profile = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "gallery24_profile");
        profile.AddRectangle(new Vec2D(0, 0), 20, 4);

        var result = api.ExtrudeUntilNext(profile, target, name: "gallery24_until_next");

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.InRange(result.Mesh.Positions.Max(p => p.Z), 19.5, 20.5);
    }

    [Fact]
    public void ExtrudeUntilSurface_TrimsAtCurvedSphereFace()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), 0.02);
        var sphereFrame = new CoordinateSystem(new Vec3D(0, 0, 5),
            new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sphere = api.CreateSphere(sphereFrame, 2, name: "sphere");
        string faceName = null;
        foreach (var name in sphere.groupIdToExtendedName.Values) { faceName = name; break; }
        Assert.NotNull(faceName);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "circle");
        sketch.AddCircle(new Vec2D(0, 0), 0.5);

        var result = api.ExtrudeUntilFace(sketch, sphere, faceName);

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.InRange(result.Mesh.Positions.Max(p => p.Z), 3.0, 3.2);
    }

    [Fact]
    public void ExtrudeUntilSurface_RejectsFaceThatDoesNotSpanProfile()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-6), new Vec3D(12)), 1e-4);
        var target = api.CreateCuboid(new Vec3D(-1, -1, 5), new Vec3D(1, 1, 6), "small_target");
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "wide_profile");
        sketch.AddRectangle(new Vec2D(0, 0), 4, 4);
        string faceName = null;
        foreach (var name in target.groupIdToExtendedName.Values)
            if (name.EndsWith("ExtrudeBottom")) { faceName = name; break; }
        Assert.NotNull(faceName);

        Assert.ThrowsAny<Exception>(() => api.ExtrudeUntilFace(sketch, target, faceName));
    }

    private static double SignedVolume(AnchorMesh mesh)
    {
        double volume = 0;
        foreach (var triangle in mesh.Mesh.Triangles)
        {
            var a = mesh.Mesh.Positions[triangle.A];
            var b = mesh.Mesh.Positions[triangle.B];
            var c = mesh.Mesh.Positions[triangle.C];
            volume += Vec3DOps.Dot(a, Vec3DOps.Cross(b, c)) / 6.0;
        }
        return Math.Abs(volume);
    }
}
