using Geo;
using GeoCore;

namespace GeoTests;

public class ExactMeshVertexIdentityTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AuthoritativePositionsDetermineIdentityAndPreserveCornerData(bool sameExact)
    {
        var converter = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .01).Converter;
        var display = new List<Vec3D> { new(0, 0, 0), new(sameExact ? .001 : 0, 0, 0), new(1, 0, 0), new(0, 1, 0) };
        var exact = new List<Rat3Hybrid> { new(0, 0, 0), new(sameExact ? 0 : 1, 0, 0), new(2, 0, 0), new(0, 2, 0) };
        var uv = new List<Vec2D> { new(0, 0), new(1, 0), new(0, 1), new(1, 1) };
        var normals = new List<Vec3D> { new(0, 0, 1), new(0, 1, 0), new(0, 0, 1), new(0, 0, 1) };
        var mesh = new MeshNormalUV(converter, display, normals, uv,
            [new(0, 2, 3), new(1, 2, 3), new(0, 1, 3)], [1, 1, 1], exact, skipWatertightCheck: true);
        Assert.Equal(sameExact ? 3 : 4, mesh.Positions.Count);
        Assert.Equal(sameExact ? 2 : 3, mesh.Triangles.Count);
        Assert.Equal(sameExact, mesh.Triangles[0].A == mesh.Triangles[1].A);
        Assert.Equal(uv[0], mesh.TrianglesEx[0].V0.UV);
        Assert.Equal(uv[1], mesh.TrianglesEx[1].V0.UV);
        Assert.Equal(normals[0], mesh.TrianglesEx[0].V0.Normal);
        Assert.Equal(normals[1], mesh.TrianglesEx[1].V0.Normal);
        Assert.All(mesh.Triangles, t => Assert.False(t.ContainsDuplicateIndex()));
        Assert.True(exact[1] == new Rat3Hybrid(sameExact ? 0 : 1, 0, 0));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WatertightValidationUsesTheAuthoritativeCoordinates(bool validExact)
    {
        var converter = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .01).Converter;
        var tetrahedron = new List<Vec3D> { new(0, 0, 0), new(2, 0, 0), new(0, 2, 0), new(0, 0, 2) };
        var display = validExact ? Enumerable.Repeat(new Vec3D(0), 4).ToList() : tetrahedron;
        var exact = tetrahedron.Select(p => validExact
            ? new Rat3Hybrid((int)p.X, (int)p.Y, (int)p.Z) : new Rat3Hybrid(0, 0, 0)).ToList();
        MeshNormalUV Build() => new(converter, display,
            Enumerable.Repeat(new Vec3D(0, 0, 1), 4).ToList(), Enumerable.Repeat(new Vec2D(0, 0), 4).ToList(),
            [new(0, 2, 1), new(0, 1, 3), new(0, 3, 2), new(1, 2, 3)], [1, 1, 1, 1], exact);
        if (validExact)
        {
            var mesh = Build();
            Assert.True(MeshAnalysis.IsWatertightMesh(mesh.PrecisionPositions, mesh.Triangles));
        }
        else
            Assert.Throws<InvalidOperationException>(Build);
    }
}
