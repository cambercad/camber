using Geo;
using GeoCore;

namespace GeoTests;

public class SupportSurfaceExtensionTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CurvedDomainRestorationHasOneFaithfulParameterChart(bool swapUv, bool transformed)
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-20), new Vec3D(20)));
        var support = new NURBS.BSplineSurface(2, 1,
            new[] { new[] { new Vec3D(0, 0, 0), new Vec3D(0, 2, 0) },
                new[] { new Vec3D(1, 0, 0), new Vec3D(1, 2, 0) },
                new[] { new Vec3D(2, 0, 2), new Vec3D(2, 2, 2) } },
            new double[] { 0, 0, 0, 1, 1, 1 }, new double[] { 0, 0, 1, 1 });
        var mapped = new Geo.NurbsConstruction.MeshUvMappedSurface(support, swapMeshUv: swapUv);
        var transform = CoordinateSystem.GetTransform(CoordinateSystem.Default,
            new CoordinateSystem(new Vec3D(3, -2, 1), new Vec3D(.6, .8, 0),
                new Vec3D(-.8, .6, 0), new Vec3D(0, 0, 1)));
        INurbsSurface source = transformed ? new TransformedNurbsSurface(mapped, transform) : mapped;
        var uv = new List<Vec2D>(); var positions = new List<Vec3D>(); var normals = new List<Vec3D>();
        for (int u = 0; u <= 8; u++)
            for (int v = 0; v <= 8; v++)
            {
                var parameter = new Vec2D(u / 16.0, v / 8.0);
                uv.Add(parameter); positions.Add(source.Evaluate(parameter.X, parameter.Y));
                normals.Add(source.EvaluateNormal(parameter.X, parameter.Y) * (swapUv ? -1 : 1));
            }
        var triangles = new List<Tri>();
        for (int u = 0; u < 8; u++)
            for (int v = 0; v < 8; v++)
            {
                int a = u * 9 + v;
                triangles.Add(new Tri(a, a + 9, a + 10)); triangles.Add(new Tri(a, a + 10, a + 1));
            }
        var mesh = new MeshNormalUV(converter, positions, normals, uv, triangles,
            Enumerable.Repeat(0, triangles.Count).ToList(), skipWatertightCheck: true);
        var original = new UVSurface(mesh.Positions, normals, uv, triangles, mesh.PrecisionPositions)
            { NurbsSurface = source };
        original.NurbsSurface = null;
        Assert.Same(original, SupportSurfaceExtension.RestoreDomain(original, mesh, 0, .04, converter));
        INurbsSurface incorrect = new Geo.NurbsConstruction.MeshUvMappedSurface(support,
            range: new ParametricRange(.25, .75, 0, 1), swapMeshUv: swapUv);
        original.NurbsSurface = transformed ? new TransformedNurbsSurface(incorrect, transform) : incorrect;
        var error = Assert.Throws<InvalidOperationException>(() =>
            SupportSurfaceExtension.RestoreDomain(original, mesh, 0, .04, converter));
        Assert.Contains("Support fidelity failed", error.Message);
        original.NurbsSurface = source;
        var restored = SupportSurfaceExtension.RestoreDomain(original, mesh, 0, .04, converter);
        Assert.True(restored.Triangles.Count > original.Triangles.Count);
        Assert.Equal(original.PointsPrecise, restored.PointsPrecise.Take(original.PointsPrecise.Count));
        Assert.Equal(original.Triangles, restored.Triangles.Take(original.Triangles.Count));
        if (!transformed) Assert.Same(support, restored.NurbsSurface);
        foreach (int vertex in restored.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
        {
            var parameter = restored.Uv[vertex];
            Assert.InRange((restored.NurbsSurface.Evaluate(parameter.X, parameter.Y) - restored.Points[vertex]).Length(),
                0, 2 * Math.Sqrt(3) * converter.SmallestUnit());
        }
        foreach (var edge in Adjacency.BuildEdgeList(restored.Triangles))
        {
            if (!edge.IsOnBorder) continue;
            var a = restored.Uv[edge.Start]; var b = restored.Uv[edge.End];
            Assert.True((a.X == 0 && b.X == 0) || (a.X == 1 && b.X == 1) ||
                (a.Y == 0 && b.Y == 0) || (a.Y == 1 && b.Y == 1));
        }
    }

    [Theory]
    [InlineData(1.0)]
    [InlineData(.5)]
    public void PlanarDomainRestorationPreservesExistingExactPatch(double width)
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-20), new Vec3D(20)));
        var support = new NURBS.BSplineSurface(1, 1,
            new[] { new[] { new Vec3D(0, 0, 0), new Vec3D(0, 10, 0) },
                new[] { new Vec3D(10, 0, 0), new Vec3D(10, 10, 0) } },
            new double[] { 0, 0, 1, 1 }, new double[] { 0, 0, 1, 1 });
        var uv = new List<Vec2D> { new(0, 0), new(width, 0), new(width, 1), new(0, 1) };
        var positions = uv.Select(p => support.Evaluate(p.X, p.Y)).ToList();
        var normals = Enumerable.Repeat(new Vec3D(0, 0, 1), 4).ToList();
        var triangles = new List<Tri> { new(0, 1, 2), new(0, 2, 3) };
        var mesh = new MeshNormalUV(converter, positions, normals, uv, triangles, new List<int> { 0, 0 }, skipWatertightCheck: true);
        // Topology extraction retains unused parent vertices. This unrelated
        // vertex coincides with a point needed by the restored half-domain but
        // carries no attributes for this patch.
        var unused = converter.Convert(support.Evaluate(1, 0));
        mesh.PrecisionPositions.Add(new Rat3Hybrid(unused.X, unused.Y, unused.Z));
        mesh.Positions.Add(converter.Convert(mesh.PrecisionPositions[^1]));
        normals.Add(new Vec3D(0));
        uv.Add(new Vec2D(0));
        var original = new UVSurface(mesh.Positions, normals, uv, mesh.Triangles, mesh.PrecisionPositions)
            { NurbsSurface = support };
        var restored = SupportSurfaceExtension.RestoreDomain(original, mesh, 0, .12, converter);
        foreach (int vertex in restored.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
            Assert.InRange(restored.Normals[vertex].Length(), .999999999, 1.000000001);
        if (width == 1)
            Assert.Same(original, restored);
        else
        {
            Assert.True(restored.Triangles.Count > original.Triangles.Count);
            Assert.Equal(original.PointsPrecise, restored.PointsPrecise.Take(original.PointsPrecise.Count));
            Assert.Equal(original.Triangles, restored.Triangles.Take(original.Triangles.Count));
            var cutEdge = Algorithms.Key(1, 2);
            Assert.DoesNotContain(Adjacency.BuildEdgeList(restored.Triangles),
                edge => edge.IsOnBorder && Algorithms.Key(edge.Start, edge.End) == cutEdge);
        }
    }

    [Fact]
    public void RestoredTrimmedLoftKeepsAuthoritativeTrianglesAndSewsContactEdges()
    {
        var (api, _, _, joined) = ImpellerLoftSupportTests.Fixture();
        Assert.True(joined.TryGetTopologySurface("blade_loft-Side", out var original));
        int group = joined.extendedNameToGroupId["blade_loft-Side"];
        var restored = SupportSurfaceExtension.RestoreDomain(original, joined.Mesh, group, .12, api.Converter);
        Assert.True(restored.Triangles.Count > original.Triangles.Count);
        Assert.Equal(original.PointsPrecise, restored.PointsPrecise.Take(original.PointsPrecise.Count));
        Assert.Equal(original.Triangles, restored.Triangles.Take(original.Triangles.Count));
        var originalBoundary = Adjacency.BuildEdgeList(original.Triangles).Where(edge => edge.IsOnBorder)
            .ToDictionary(edge => Algorithms.Key(edge.Start, edge.End));
        var remainingBoundary = Adjacency.BuildEdgeList(restored.Triangles).Where(edge => edge.IsOnBorder)
            .Select(edge => Algorithms.Key(edge.Start, edge.End)).ToHashSet();
        // Boolean contact edges are interior to the restored construction domain.
        // Only the two natural profile ends may remain on its boundary.
        foreach (var key in originalBoundary.Keys.Intersect(remainingBoundary))
        {
            var edge = originalBoundary[key];
            var a = original.Uv[edge.Start];
            var b = original.Uv[edge.End];
            Assert.True((a.Y == 0 && b.Y == 0) || (a.Y == 1 && b.Y == 1),
                $"A trimmed contact edge remains open: {edge}, UV {a} to {b}.");
        }
    }
}
