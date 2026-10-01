using System.Numerics;
using GeoCore;

namespace GeoTests;

public class TriangulationPredicateReuseTests
{
    [Fact]
    public void CachedOrientationMatchesExactDeterminantForAllPermutationsAndNewSessions()
    {
        var random = new Random(31572);
        var context = new TriangulationContext<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>();
        var orders = new[] { (0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0) };
        for (int trial = 0; trial < 80; ++trial)
        {
            BigRationalHybrid Value() => new(random.Next(-1000, 1001),
                (BigInteger.One << (1 + trial * 3)) + random.Next(1, 100));
            var points = new List<Rat2Hybrid> { new(Value(), Value()), new(Value(), Value()), new(Value(), Value()) };
            if (trial % 4 == 0) points[2] = points[0] + points[1];
            if (trial % 5 == 0) points[1] = points[0];
            context.BeginSession(points);
            // Change the first requested permutation between sessions, then
            // revisit every permutation after the cache has been populated.
            for (int repeat = 0; repeat < 2; ++repeat)
                for (int i = 0; i < orders.Length; ++i)
                {
                    var (a, b, c) = orders[(i + trial) % orders.Length];
                    Assert.Equal(ExactOrientation(points[a], points[b], points[c]), context.Orient2D(a, b, c));
                }
            Assert.Equal(0, context.Orient2D(0, 0, 1));
            Assert.Equal(0, context.Orient2D(0, 1, 0));
            Assert.Equal(0, context.Orient2D(1, 0, 0));
        }
    }

    [Theory]
    [InlineData(0, 3, 1, 4, false)] // Same side, although the bounding boxes overlap.
    [InlineData(0, 6, 6, 0, true)] // Proper crossing.
    [InlineData(3, 3, 3, 5, true)] // Touching the interior of the other segment.
    [InlineData(1, 1, 4, 4, true)] // Collinear overlap.
    [InlineData(7, 7, 9, 9, false)] // Collinear disjoint segments.
    public void SegmentIntersectionPreservesCrossingTouchingAndCollinearCases(int x, int y, int u, int v, bool expected)
    {
        var points = new List<Rat2Hybrid> { new(0, 0), new(6, 6), new(x, y), new(u, v) };
        var arithmetic = new Rat2HybridArithmetic();
        for (int reverseA = 0; reverseA < 2; ++reverseA)
            for (int reverseB = 0; reverseB < 2; ++reverseB)
                Assert.Equal(expected, TriangulationHelper<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>
                    .SegmentsIntersectUnchecked(arithmetic, points, reverseA, 1 - reverseA,
                        new Int2(2 + reverseB, 3 - reverseB)));
    }

    private static int ExactOrientation(Rat2Hybrid a, Rat2Hybrid b, Rat2Hybrid c)
    {
        static (BigInteger Numerator, BigInteger Denominator) Difference(BigRationalHybrid x, BigRationalHybrid y) =>
            (x.Numerator() * y.Denominator() - y.Numerator() * x.Denominator(), x.Denominator() * y.Denominator());
        var ax = Difference(a.X, c.X);
        var ay = Difference(a.Y, c.Y);
        var bx = Difference(b.X, c.X);
        var by = Difference(b.Y, c.Y);
        return (ax.Numerator * by.Numerator * ay.Denominator * bx.Denominator -
                ay.Numerator * bx.Numerator * ax.Denominator * by.Denominator).Sign;
    }
}
