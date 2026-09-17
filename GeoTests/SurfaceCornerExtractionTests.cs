using Geo;
using GeoCore;

namespace GeoTests;

public class SurfaceCornerExtractionTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void CylinderUvSeamPreservesBothCoordinatesWithoutChangingParentTopology()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), .01);
        var cylinder = api.CreateCylinder(CoordinateSystem.Default, 1, 2, .01, "seam_cylinder");
        const string name = "seam_cylinder-Circle1";
        Assert.True(cylinder.TryGetSurface(name, out var surface));
        Assert.Contains(surface.Uv, uv => uv.X == 0);
        Assert.Contains(surface.Uv, uv => uv.X == 1);
        var zero = Enumerable.Range(0, surface.Uv.Count).Where(i => surface.Uv[i].X == 0).ToArray();
        var one = Enumerable.Range(0, surface.Uv.Count).Where(i => surface.Uv[i].X == 1).ToArray();
        Assert.Contains(zero, a => one.Any(b => surface.PointsPrecise[a] == surface.PointsPrecise[b]));

        Assert.True(cylinder.TryGetTopologySurface(name, out var topology));
        int group = cylinder.extendedNameToGroupId[name];
        var expected = cylinder.Mesh.Triangles.Where((_, i) => cylinder.Mesh.TrianglesEx[i].GroupId == group);
        Assert.Equal(expected, topology.Triangles);
        Assert.Same(cylinder.Mesh.PrecisionPositions, topology.PointsPrecise);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ExtractionPreservesDistinctCornerAttributesAndExactSourcePositions(bool hasExactPositions)
    {
        var a = new TriangleVertexNormalUV { Normal = new Vec3D(0, 0, 1), UV = new Vec2D(0, 0) };
        var b = new TriangleVertexNormalUV { Normal = new Vec3D(0, 1, 0), UV = new Vec2D(1, 0) };
        var mesh = new MeshNormalUV
        {
            Positions = [new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0)],
            PrecisionPositions = [new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0)],
            Triangles = [new(0, 1, 2), new(0, 2, 3)],
            TrianglesEx =
            [
                new() { V0 = a, V1 = a, V2 = a, GroupId = 1 },
                new() { V0 = b, V1 = a, V2 = a, GroupId = 1 }
            ]
        };
        if (!hasExactPositions) mesh.PrecisionPositions = null;
        var body = new AnchorMesh("source", mesh, new Dictionary<int, string> { [1] = "source-face" },
            new(), deferCoplanarPostProcess: false, isVolume: false, preserveTriangulation: true);
        Assert.True(body.TryGetSurface("source-face", out var surface));
        Assert.Equal(2, surface.Triangles.Count);
        for (int i = 0; i < mesh.Triangles.Count; i++)
        {
            var source = mesh.Triangles[i];
            var target = surface.Triangles[i];
            var sourceIndices = new[] { source.A, source.B, source.C };
            var targetIndices = new[] { target.A, target.B, target.C };
            var corners = new[] { mesh.TrianglesEx[i].V0, mesh.TrianglesEx[i].V1, mesh.TrianglesEx[i].V2 };
            for (int j = 0; j < 3; j++)
            {
                Assert.Equal(mesh.Positions[sourceIndices[j]], surface.Points[targetIndices[j]]);
                if (hasExactPositions)
                    Assert.True(mesh.PrecisionPositions[sourceIndices[j]] == surface.PointsPrecise[targetIndices[j]]);
                Assert.Equal(corners[j].UV, surface.Uv[targetIndices[j]]);
                Assert.Equal(corners[j].Normal, surface.Normals[targetIndices[j]]);
            }
        }
        Assert.NotEqual(surface.Triangles[0].A, surface.Triangles[1].A);
        if (hasExactPositions)
            Assert.True(surface.PointsPrecise[surface.Triangles[0].A] == surface.PointsPrecise[surface.Triangles[1].A]);
        else
            Assert.Null(surface.PointsPrecise);
        Assert.Equal(4, mesh.Positions.Count);
        Assert.Equal(a.UV, mesh.TrianglesEx[0].V0.UV);
        Assert.Equal(b.UV, mesh.TrianglesEx[1].V0.UV);
    }
}
