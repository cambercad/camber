using Geo;
using GeoCore;

namespace GeoTests;

public class BlendContourDistanceTests
{
    [Fact]
    public void ExactBoundingBoxPruningPreservesExhaustivePolylineDistance()
    {
        List<Rat3Hybrid> source = Enumerable.Range(0, 17)
            .Select(i => new Rat3Hybrid(i * 3, (i * i * 7) % 19, (i * 11) % 13)).ToList();
        List<Rat3Hybrid> first = Enumerable.Range(0, 18)
            .Select(i => new Rat3Hybrid(i * 2, (i * 5 + 3) % 23, (i * i) % 17)).ToList();
        List<Rat3Hybrid> second = Enumerable.Range(0, 16)
            .Select(i => new Rat3Hybrid(i * 3 + 1, (i * i + 5) % 29, (i * 7) % 11)).ToList();
        first.Insert(4, first[3]); // A zero-length segment remains well-defined.

        var expected = MaxExhaustive(first, second, source);
        Assert.Equal(expected, BlendEdge.MaxSourceEdgeDeviationSquared(first, second, source));
        Assert.Equal(expected, BlendEdge.MaxSourceEdgeDeviationSquared(second, first, source));

        // Reversing either contour changes traversal order but not its exact distance.
        first.Reverse();
        second.Reverse();
        Assert.Equal(expected, BlendEdge.MaxSourceEdgeDeviationSquared(first, second, source));
    }

    private static BigRationalHybrid MaxExhaustive(List<Rat3Hybrid> first,
        List<Rat3Hybrid> second, List<Rat3Hybrid> source)
    {
        BigRationalHybrid maximum = BigRationalHybrid.Zero;
        Check(first, source);
        Check(second, source);
        Check(source, first);
        Check(source, second);
        return maximum;

        void Check(List<Rat3Hybrid> from, List<Rat3Hybrid> to)
        {
            foreach (var point in from)
            {
                BigRationalHybrid? nearest = null;
                for (int i = 1; i < to.Count; i++)
                {
                    var start = to[i - 1];
                    var vector = to[i] - start;
                    var lengthSquared = Rat3Hybrid.Dot(vector, vector);
                    if (lengthSquared == BigRationalHybrid.Zero) continue;
                    var parameter = Rat3Hybrid.Dot(point - start, vector) / lengthSquared;
                    if (parameter < BigRationalHybrid.Zero) parameter = BigRationalHybrid.Zero;
                    else if (parameter > BigRationalHybrid.One) parameter = BigRationalHybrid.One;
                    var displacement = point - (start + parameter * vector);
                    var squared = Rat3Hybrid.Dot(displacement, displacement);
                    if (!nearest.HasValue || squared < nearest.Value) nearest = squared;
                }
                Assert.True(nearest.HasValue);
                if (nearest.Value > maximum) maximum = nearest.Value;
            }
        }
    }
}
