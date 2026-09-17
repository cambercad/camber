using System.Numerics;
using Geo;
using GeoCore;

namespace GeoTests;

public class PreciseMeshDecompositionTests
{
    [Fact]
    public void ExistingDisplayOnlyDecompositionDoesNotRequireExactCoordinates()
    {
        var payload = new TriangleVertexNormalUV { Normal = new Vec3D(0, 0, 1), UV = new Vec2D(0, 0) };
        var mesh = new MeshNormalUV
        {
            Positions = [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0)],
            Triangles = [new(0, 1, 2)],
            TrianglesEx = [new() { V0 = payload, V1 = payload, V2 = payload, GroupId = 4 }]
        };
        mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out var groups);
        Assert.Equal(mesh.Positions, positions);
        Assert.Equal(mesh.Triangles, triangles);
        Assert.Equal(new[] { 4 }, groups);
        Assert.Equal(3, normals.Count);
        Assert.Equal(3, uvs.Count);
        Assert.Null(mesh.PrecisionPositions);
        Assert.Throws<InvalidOperationException>(() =>
            mesh.Decompose(out _, out _, out _, out _, out _, out _));
    }

    [Fact]
    public void DisplayEqualVerticesAndUvSeamsRetainTheirExactSourceCorners()
    {
        var epsilon = new BigRationalHybrid(BigInteger.One, BigInteger.One << 80);
        var half = new BigRationalHybrid(1, 2);
        var one = BigRationalHybrid.One;
        var zero = BigRationalHybrid.Zero;
        List<Rat3Hybrid> exact =
        [
            new(one, half, zero), // Unused display-equal point must never be selected.
            new(1, 0, 0),
            new(1, 1, 0),
            new(one + epsilon, half, zero),
            new(one + epsilon * new BigRationalHybrid(2), half, zero)
        ];
        var display = exact.Select(p => new Vec3D(p.X.ToDouble(), p.Y.ToDouble(), p.Z.ToDouble())).ToList();
        Assert.Equal(display[0], display[3]);
        Assert.Equal(display[0], display[4]);
        Assert.NotEqual(exact[0], exact[3]);
        Assert.NotEqual(exact[3], exact[4]);

        var ordinary = new TriangleVertexNormalUV { Normal = new Vec3D(0, 0, 1), UV = new Vec2D(0, 0) };
        var seam = new TriangleVertexNormalUV { Normal = new Vec3D(0, 0, 1), UV = new Vec2D(1, 0) };
        var normalSeam = new TriangleVertexNormalUV { Normal = new Vec3D(1, 0, 0), UV = new Vec2D(0, 0) };
        var mesh = new MeshNormalUV
        {
            Positions = display,
            PrecisionPositions = exact,
            Triangles = [new(1, 3, 2), new(1, 2, 4), new(1, 3, 2)],
            TrianglesEx =
            [
                new() { V0 = ordinary, V1 = ordinary, V2 = ordinary, GroupId = 7 },
                new() { V0 = seam, V1 = ordinary, V2 = ordinary, GroupId = 9 },
                new() { V0 = normalSeam, V1 = ordinary, V2 = ordinary, GroupId = 11 }
            ]
        };
        var saved = exact.Select(p => new Rat3Hybrid(in p)).ToArray();
        mesh.Decompose(out var positions, out var normals, out var uvs,
            out var triangles, out var groups, out var precise);

        Assert.Equal(new[] { 7, 9, 11 }, groups);
        Assert.Equal(6, positions.Count); // Four exact vertices plus UV and normal seams.
        Assert.Equal(positions.Count, precise.Count);
        Assert.Equal(positions.Count, normals.Count);
        Assert.Equal(positions.Count, uvs.Count);
        for (int i = 0; i < triangles.Count; ++i)
        {
            var source = mesh.Triangles[i];
            var actual = triangles[i];
            Assert.Equal(exact[source.A], precise[actual.A]);
            Assert.Equal(exact[source.B], precise[actual.B]);
            Assert.Equal(exact[source.C], precise[actual.C]);
            Assert.NotEqual(new Rat3Hybrid(0, 0, 0), Rat3Hybrid.Cross(
                precise[actual.B] - precise[actual.A], precise[actual.C] - precise[actual.A]));
        }
        Assert.NotEqual(triangles[0].A, triangles[1].A);
        Assert.NotEqual(uvs[triangles[0].A], uvs[triangles[1].A]);
        Assert.NotEqual(triangles[0].A, triangles[2].A);
        Assert.Equal(uvs[triangles[0].A], uvs[triangles[2].A]);
        Assert.NotEqual(normals[triangles[0].A], normals[triangles[2].A]);
        Assert.Equal(triangles[0].B, triangles[2].B);
        Assert.Equal(triangles[0].C, triangles[2].C);
        Assert.Equal(saved, mesh.PrecisionPositions);
        Assert.Equal(new[] { new Tri(1, 3, 2), new Tri(1, 2, 4), new Tri(1, 3, 2) }, mesh.Triangles);
    }
}
