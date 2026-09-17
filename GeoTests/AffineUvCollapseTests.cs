using Geo;
using GeoCore;
using Remeshing;

namespace GeoTests;

public class AffineUvCollapseTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CollinearCleanupPreservesParameterBreaksButRemovesAffineRedundancy(bool affine, bool oblique)
    {
        var positions = new List<Rat3Hybrid> { new(0, 0, 0), new(1, 0, 0), new(2, 0, 0), new(2, 1, 0), new(0, 1, 0) };
        if (oblique) positions = positions.Select(p => new Rat3Hybrid(p.X + p.Y, p.X - p.Y, p.X + p.Y + p.Y)).ToList();
        var triangles = new List<Tri> { new(0, 1, 4), new(1, 3, 4), new(1, 2, 3) };
        Vec2D[] uv = [new(0, 0), new(affine ? .5 : .25, 0), new(1, 0), new(1, 1), new(0, 1)];
        TriangleVertexNormalUV Vertex(int i) => new() { UV = uv[i], Normal = new(0, 0, 1) };
        var attributes = triangles.Select(t => new MeshTriangle<TriangleVertexNormalUV>
        {
            GroupId = 1, V0 = Vertex(t.A), V1 = Vertex(t.B), V2 = Vertex(t.C)
        }).ToList();
        var mutable = triangles.Select(t => new TriWithGroupId { A = t.A, B = t.B, C = t.C, GroupId = 1 }).ToList();
        EdgeCollapser.CleanMesh(positions, mutable, BigRationalHybrid.Zero,
            allowVertexRelocation: false, removeCollinearEdges: true,
            validateCollapseCallback: new AffineUvCollapseGuard(positions, triangles, attributes).Validate,
            vertexMustBePreserved: new bool[positions.Count]);
        var remaining = mutable.Where(t => t.A >= 0).ToList();
        Assert.Equal(affine ? 2 : 3, remaining.Count);
        Assert.Equal(!affine, remaining.Any(t => t.A == 1 || t.B == 1 || t.C == 1));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AnchorCleanupKeepsInteriorUvWitness(bool affine)
    {
        var points = new List<Vec3D>
        {
            new(0, 0, 0), new(1, 0, 0), new(2, 0, 0), new(2, 1, 0), new(0, 1, 0),
            new(0, 0, 2), new(1, 0, 2), new(0, 1, 2)
        };
        var triangles = new List<Tri> { new(0, 1, 4), new(1, 3, 4), new(1, 2, 3), new(5, 6, 7) };
        Vec2D[] uv = [new(0, 0), new(affine ? .5 : .25, 0), new(1, 0), new(1, 1), new(0, 1), new(0, 0), new(1, 0), new(0, 1)];
        TriangleVertexNormalUV Vertex(int i) => new() { UV = uv[i], Normal = new(0, 0, 1) };
        var mesh = new MeshNormalUV
        {
            Positions = points,
            PrecisionPositions = points.Select(p => new Rat3Hybrid((int)p.X, (int)p.Y, (int)p.Z)).ToList(),
            Triangles = triangles,
            TrianglesEx = triangles.Select((t, i) => new MeshTriangle<TriangleVertexNormalUV>
            {
                GroupId = i == 3 ? 2 : 1, V0 = Vertex(t.A), V1 = Vertex(t.B), V2 = Vertex(t.C)
            }).ToList()
        };
        // The second planar patch triggers ordinary cleanup. The first patch
        // retains a nonplanar-support parameterization, as a planar portion of
        // a loft can do; it must not be assigned replacement planar UVs.
        var body = new AnchorMesh("parameterized", mesh,
            new Dictionary<int, string> { [1] = "parameterized-side", [2] = "parameterized-cap" },
            new Dictionary<string, SurfaceMetaData> { ["parameterized-side"] = new(SurfaceType.Unknown) },
            isVolume: false);
        Assert.Equal(affine ? 2 : 3, mesh.TrianglesEx.Count(t => t.GroupId == 1));
        var witness = new Vec3D(1, .25, 0);
        bool found = false;
        for (int i = 0; i < mesh.Triangles.Count; i++)
        {
            if (mesh.TrianglesEx[i].GroupId != 1) continue;
            var weights = InterpolationHelpers.GetBarycentricWeights(witness, mesh.Triangles[i], points);
            if (weights.X < -1e-12 || weights.Y < -1e-12 || weights.Z < -1e-12) continue;
            var data = mesh.TrianglesEx[i];
            var actual = data.V0.UV * weights.X + data.V1.UV * weights.Y + data.V2.UV * weights.Z;
            Assert.Equal(affine ? .5 : .3125, actual.X, 12);
            Assert.Equal(.25, actual.Y, 12);
            found = true;
        }
        Assert.True(found);
    }
}
