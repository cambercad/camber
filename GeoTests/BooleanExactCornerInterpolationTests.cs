using Geo;
using GeoCore;

namespace GeoTests;

public class BooleanExactCornerInterpolationTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void ExactBoundaryWeightsStayOnTheSourceEdge()
    {
        var points = new List<Rat3Hybrid> { new(0, 0, 0), new(8, 0, 0), new(0, 8, 0) };
        var weights = InterpolationHelpers.GetBarycentricWeights(new Rat3Hybrid(3, 5, 0), new Tri(0, 1, 2), points);
        Assert.Equal(0, weights.X);
        Assert.Equal(3.0 / 8, weights.Y);
        Assert.Equal(5.0 / 8, weights.Z);
    }

    [Fact]
    public void ExactAttributeInterpolationPreservesConstantSeamsAndSharedEdges()
    {
        TriangleVertexNormalUV Vertex(double u, double v) => new() { UV = new(u, v), Normal = new(0, 0, 1) };
        var positions = new List<Rat3Hybrid> { new(0, 0, 0), new(6, 0, 0), new(0, 6, 0) };
        var weights = InterpolationHelpers.GetExactBarycentricWeights(new Rat3Hybrid(4, 1, 0), new Tri(0, 1, 2), positions);
        // Separately rounded weights 1/6 + 4/6 + 1/6 sum to 1 - one ULP.
        var constant = Vertex(1, .1).InterpolateExact(Vertex(1, .7), Vertex(1, .2), weights);
        Assert.Equal(1, constant.UV.X);

        positions = [new(0, 0, 0), new(7, 0, 0), new(0, 7, 0), new(0, -7, 0)];
        var point = new Rat3Hybrid(2, 0, 0);
        var forward = InterpolationHelpers.GetExactBarycentricWeights(point, new Tri(0, 1, 2), positions);
        var reversed = InterpolationHelpers.GetExactBarycentricWeights(point, new Tri(1, 0, 3), positions);
        var first = Vertex(.1, .9).InterpolateExact(Vertex(.3, .2), Vertex(8, 9), forward);
        var second = Vertex(.3, .2).InterpolateExact(Vertex(.1, .9), Vertex(-8, -9), reversed);
        Assert.Equal(first.UV, second.UV);
    }

    [Fact]
    public void TrimmedImpellerCornersStayWithinTheirAuthoredSupportDomain()
    {
        var (_, _, _, joined) = ImpellerLoftSupportTests.Fixture();
        int side = joined.extendedNameToGroupId["blade_loft-Side"];
        foreach (var triangle in joined.Mesh.TrianglesEx.Where(t => t.GroupId == side))
        {
            foreach (var uv in new[] { triangle.V0.UV, triangle.V1.UV, triangle.V2.UV })
            {
                Assert.InRange(uv.X, 0, 1);
                Assert.InRange(uv.Y, 0, 1);
            }
        }
    }
}
