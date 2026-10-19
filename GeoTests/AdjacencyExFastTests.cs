using GeoCore;
using Xunit;

namespace GeoTests;

public class AdjacencyExFastTests
{
    private static void AssertSame(TriangleAdjacency[] expected, TriangleAdjacency[] actual)
    {
        Assert.Equal(expected.Length, actual.Length);
        for (int i = 0; i < expected.Length; i++)
        {
            Assert.Equal(expected[i].NeighbourAB, actual[i].NeighbourAB);
            Assert.Equal(expected[i].NeighbourBC, actual[i].NeighbourBC);
            Assert.Equal(expected[i].NeighbourCA, actual[i].NeighbourCA);
        }
    }

    private static List<Rat3Hybrid> GridPoints(int n)
    {
        var points = new List<Rat3Hybrid>();
        for (int y = 0; y <= n; y++)
            for (int x = 0; x <= n; x++)
                points.Add(new Rat3Hybrid(x, y, (x * 7 + y * 3) % 5));
        return points;
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void RandomGridSubsets_MatchReference(int seed)
    {
        const int n = 40;
        var random = new Random(seed);
        var points = GridPoints(n);
        var triangles = new List<Tri>();
        for (int y = 0; y < n; y++)
            for (int x = 0; x < n; x++)
            {
                int a = y * (n + 1) + x, b = a + 1, c = a + n + 1, d = c + 1;
                if (random.Next(8) != 0) triangles.Add(random.Next(2) == 0 ? new Tri(a, b, d) : new Tri(b, d, a));
                if (random.Next(8) != 0) triangles.Add(random.Next(2) == 0 ? new Tri(a, d, c) : new Tri(c, a, d));
            }
        // Removed triangles are marked with negative indices by the resolver.
        triangles.Insert(5, new Tri(-1, -1, -1));
        triangles.Add(new Tri(-1, -1, -1));

        AssertSame(AdjacencyEx.BuildAdjacencyInformationReference(points, triangles),
            AdjacencyEx.BuildAdjacencyInformation(points, triangles));
    }

    [Fact]
    public void FourPageBook_PairsLikeReference()
    {
        var points = new List<Rat3Hybrid>
        {
            new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, 0, 1), new(0, -1, 0), new(0, 0, -1)
        };
        var triangles = new List<Tri> { new(0, 1, 2), new(1, 0, 3), new(0, 1, 4), new(1, 0, 5) };
        AssertSame(AdjacencyEx.BuildAdjacencyInformationReference(points, triangles),
            AdjacencyEx.BuildAdjacencyInformation(points, triangles));

        // Same book with the pages listed in a different order.
        triangles = new List<Tri> { new(0, 1, 4), new(1, 0, 3), new(1, 0, 5), new(0, 1, 2) };
        AssertSame(AdjacencyEx.BuildAdjacencyInformationReference(points, triangles),
            AdjacencyEx.BuildAdjacencyInformation(points, triangles));
    }

    [Fact]
    public void DuplicateTriangles_ResolveSlotsLikeReference()
    {
        var points = new List<Rat3Hybrid> { new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(1, 1, 1) };
        var triangles = new List<Tri> { new(0, 1, 2), new(0, 2, 1) };
        AssertSame(AdjacencyEx.BuildAdjacencyInformationReference(points, triangles),
            AdjacencyEx.BuildAdjacencyInformation(points, triangles));
    }

    [Fact]
    public void ThrowsLikeReference()
    {
        var points = GridPoints(2);
        var degenerate = new List<Tri> { new(0, 1, 4), new(1, 1, 4) };
        Assert.Throws<Exception>(() => AdjacencyEx.BuildAdjacencyInformationReference(points, degenerate));
        Assert.Throws<Exception>(() => AdjacencyEx.BuildAdjacencyInformation(points, degenerate));

        var open = new List<Tri> { new(0, 1, 4), new(1, 5, 4), new(0, 4, 3) };
        Assert.Throws<Exception>(() => AdjacencyEx.BuildAdjacencyInformationReference(points, open, true));
        Assert.Throws<Exception>(() => AdjacencyEx.BuildAdjacencyInformation(points, open, true));
    }
}
