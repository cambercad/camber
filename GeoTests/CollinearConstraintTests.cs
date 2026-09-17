using GeoCore;

namespace GeoTests;

public class CollinearConstraintTests
{
    [Theory]
    [InlineData(false, false, false, false)]
    [InlineData(false, true, true, true)]
    [InlineData(true, false, true, true)]
    [InlineData(true, true, false, false)]
    public void ConstraintsThroughExistingVerticesPreserveAreaAndEdges(
        bool interior, bool reversed, bool optimize, bool vertical)
    {
        int y = interior ? 5 : 0;
        var points = new List<Rat2Hybrid> {
            new(0, 0), new(10, 0), new(10, 10), new(0, 10),
            new(2, y), new(8, y), new(4, y), new(6, y)
        };
        if (vertical) points = points.Select(p => new Rat2Hybrid(-p.Y, p.X)).ToList();
        // Include the same constraint in both directions: normalization must
        // retain one chain, independent of direction and redundant input.
        var constraint = reversed ? new Int2(5, 4) : new Int2(4, 5);
        var constraints = new List<Int2> { constraint, new(5, 4) };
        var original = constraints.ToArray();
        var triangles = Triangulator.TriangulatePolygon(points,
            new List<int> { 0, 1, 2, 3 }, constraints, new List<int> { 6, 7 }, optimize);
        Assert.Equal(original, constraints);

        BigRationalHybrid area = new(0);
        var edges = new HashSet<long>();
        var used = new HashSet<int>();
        foreach (var t in triangles)
        {
            var a = points[t.A]; var b = points[t.B]; var c = points[t.C];
            var signedArea = (b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X);
            Assert.True(signedArea.Sign() > 0);
            area += signedArea;
            edges.Add(Algorithms.Key(t.A, t.B));
            edges.Add(Algorithms.Key(t.B, t.C));
            edges.Add(Algorithms.Key(t.C, t.A));
            used.UnionWith(new[] { t.A, t.B, t.C });
        }
        Assert.True(area == new BigRationalHybrid(200));
        Assert.Equal(8, used.Count);
        foreach (var edge in new[] { (4, 6), (6, 7), (7, 5) })
            Assert.Contains(Algorithms.Key(edge.Item1, edge.Item2), edges);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NearbyVertexDoesNotSplitAnExactlyStraightConstraint(bool optimize)
    {
        var offset = new BigRationalHybrid(1, 1_000_000_000_000L);
        var nearY = new BigRationalHybrid(5) + offset;
        nearY.Simplify(); // Kernel point storage uses canonical rational values.
        var points = new List<Rat2Hybrid> {
            new(0, 0), new(10, 0), new(10, 10), new(0, 10),
            new(2, 5), new(8, 5), new(new BigRationalHybrid(5), nearY)
        };
        var triangles = Triangulator.TriangulatePolygon(points,
            new List<int> { 0, 1, 2, 3 }, new List<Int2> { new(4, 5) },
            new List<int> { 6 }, optimize);
        Assert.Contains(triangles, t => t.Contains(4) && t.Contains(5));
        Assert.Contains(triangles, t => t.Contains(6));
        BigRationalHybrid area = new(0);
        foreach (var t in triangles)
        {
            var a = points[t.A]; var b = points[t.B]; var c = points[t.C];
            var signedArea = (b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X);
            Assert.True(signedArea.Sign() > 0);
            area += signedArea;
        }
        Assert.True(area == new BigRationalHybrid(200));
    }

}
