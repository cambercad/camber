using GeoCore;

namespace GeoTests;

public class AdjacencyAngularOrderTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullTurnAndOppositeRaysHaveConsistentCyclicOrder(bool tilted)
    {
        var vertices = new List<Rat3Hybrid>
        {
            new(0, 0, 0), new(0, 0, 1), new(1, 0, 0), new(0, 1, 0),
            new(-1, 0, 0), new(0, -1, 0)
        };
        if (tilted)
            vertices = vertices.Select(p => new Rat3Hybrid(p.X + p.Y, p.Y + p.Z, p.Z)).ToList();
        var triangles = new List<Tri> { new(0, 2, 1), new(0, 1, 3), new(0, 4, 1), new(0, 1, 5) };
        foreach (var order in Permutations(new[] { 0, 1, 2, 3 }))
        {
            var sorted = order.ToList();
            AdjacencyEx.SortAroundEdge(vertices, triangles, sorted, 0, 1);
            for (int i = 0; i < sorted.Count; i++)
                Assert.Equal((sorted[0] + i) % sorted.Count, sorted[i]);
        }
    }

    [Fact]
    public void InvalidIncidentWindingStillFails()
    {
        var vertices = new List<Rat3Hybrid>
        {
            new(0, 0, 0), new(0, 0, 1), new(1, 0, 0), new(0, 1, 0),
            new(-1, 0, 0), new(0, -1, 0)
        };
        var triangles = new List<Tri> { new(0, 2, 1), new(0, 3, 1), new(0, 1, 4), new(0, 1, 5) };
        Assert.Throws<Exception>(() => AdjacencyEx.SortAroundEdge(vertices, triangles, new List<int> { 0, 1, 2, 3 }, 0, 1));
    }

    private static IEnumerable<int[]> Permutations(int[] values)
    {
        if (values.Length == 0)
        {
            yield return Array.Empty<int>();
            yield break;
        }
        foreach (int first in values)
            foreach (var rest in Permutations(values.Where(value => value != first).ToArray()))
                yield return new[] { first }.Concat(rest).ToArray();
    }
}
