using GeoCore;
using NURBS;

namespace GeoTests;

public class NurbsBorderPrecisionTests
{
    [Theory]
    [InlineData(.125, 1)]
    [InlineData(1, .125)]
    public void PlanarPatchRespectsMaximumSpanInEitherDirection(double spanU, double spanV)
    {
        var surface = new BSplineSurface(1, 1,
            [new[] { new Vec3D(0, 0, 0), new Vec3D(0, 1, 0) },
             new[] { new Vec3D(1, 0, 0), new Vec3D(1, 1, 0) }],
            [0, 0, 1, 1], [0, 0, 1, 1]);
        var mesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface, out var patches, out _,
            maxDeviation: .01, maxSpanU: spanU, maxSpanV: spanV);
        Assert.NotEmpty(mesh.Triangles);
        foreach (var patch in patches.Where(p => p.IsLeave))
        {
            Assert.InRange(patch.MaxU - patch.MinU, 0, spanU);
            Assert.InRange(patch.MaxV - patch.MinV, 0, spanV);
        }
    }

    [Fact]
    public void NearbyKnotSamplesHaveUniqueFaithfulParameters()
    {
        double[] parameters = [0, Math.BitDecrement(Math.BitDecrement(.25)), Math.BitDecrement(.25), .25, 1];
        var controls = parameters.Select(u => new[] { new Vec3D(u, 0, 0), new Vec3D(u, 1, u) }).ToArray();
        var surface = new BSplineSurface(1, 1, controls,
            new[] { 0.0 }.Concat(parameters).Concat(new[] { 1.0 }).ToArray(), [0, 0, 1, 1]);
        var mesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface,
            maxDeviation: .01, maxSpanU: .1, maxSpanV: .125);
        Assert.Equal(mesh.UV.Count, mesh.UV.Select(uv => (uv.X, uv.Y)).Distinct().Count());
        for (int i = 0; i < mesh.UV.Count; i++)
            Assert.True((surface.Evaluate(mesh.UV[i].X, mesh.UV[i].Y) - mesh.Points[i]).Length() < 1e-12);
    }

    [Theory]
    [InlineData(0.5)]
    [InlineData(0.9999999999999999)]
    public void AdjacentParametersProduceNondegenerateBorderTriangles(double u)
    {
        var points = new List<Vec2D>
        {
            new(u, 0), new(Math.BitIncrement(u), 0),
            new(Math.BitIncrement(u), 1), new(u, 1)
        };
        var triangles = BoundaryClipper.TriangulateDefaultBorder(points, [new() { 0, 1, 2, 3 }])[0];
        Assert.Equal(2, triangles.Count);
        var used = triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct().Order().ToArray();
        Assert.Equal(new[] { 0, 1, 2, 3 }, used);
        foreach (var triangle in triangles)
        {
            var a = points[triangle.A];
            var b = points[triangle.B];
            var c = points[triangle.C];
            Assert.True((b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X) > 0);
        }
    }
}
