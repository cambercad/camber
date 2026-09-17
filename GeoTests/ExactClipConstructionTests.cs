using System.Numerics;
using CSG;
using GeoCore;

namespace GeoTests;

public class ExactClipConstructionTests
{
    private static BigRationalHybrid Fraction(long n, long d) => new(n, d);
    private static void AssertReduced(Rat3Hybrid point)
    {
        for (int axis = 0; axis < 3; axis++)
            Assert.Equal(BigInteger.One, BigInteger.GreatestCommonDivisor(
                point[axis].Numerator(), point[axis].Denominator()));
    }

    [Fact]
    public void PlaneOwnsReducedCoefficientsWithoutMutatingCaller()
    {
        var normal = new Rat3Hybrid(Fraction(14, 21), Fraction(-35, 49), Fraction(22, 33));
        var point = new Rat3Hybrid(Fraction(11, 13), Fraction(17, 19), Fraction(-23, 29));
        var numerators = Enumerable.Range(0, 3).Select(i => normal[i].Numerator()).ToArray();
        var denominators = Enumerable.Range(0, 3).Select(i => normal[i].Denominator()).ToArray();
        var plane = new Rat3HybridPlane(normal, point);
        AssertReduced(plane.Normal);
        Assert.Equal(BigInteger.One, BigInteger.GreatestCommonDivisor(plane.PlaneD.Numerator(), plane.PlaneD.Denominator()));
        Assert.True(plane.SignedDistancePointPlane(point) == BigRationalHybrid.Zero);
        for (int axis = 0; axis < 3; axis++)
        {
            Assert.Equal(numerators[axis], normal[axis].Numerator());
            Assert.Equal(denominators[axis], normal[axis].Denominator());
            Assert.True(plane.Normal[axis] == normal[axis]);
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    [InlineData(1000000)]
    public void RepeatedTiltedPlaneClipsKeepExactCanonicalEndpoints(int scale)
    {
        var origin = new Rat3Hybrid(Fraction(101, 103), Fraction(-107, 109), Fraction(113, 127));
        var direction = new Rat3Hybrid(Fraction(131 * (long)scale, 137), Fraction(139 * (long)scale, 149), Fraction(-151 * (long)scale, 157));
        var segment = new Rat3LineSegment(origin, origin + direction);
        Rat3Hybrid At(long numerator) => origin + direction * Fraction(numerator, 17);
        // Alternately clip opposite ends in a tilted rational coordinate frame.
        foreach (var (numerator, keepAfter) in new[] {(2L, true), (15L, false), (5L, true), (12L, false)})
        {
            var normal = keepAfter ? -direction : direction;
            segment = segment.Trim(new Rat3HybridPlane(normal, At(numerator)), out bool invalid);
            Assert.False(invalid);
            AssertReduced(keepAfter ? segment.A : segment.B);
        }
        Assert.True(segment.A == At(5));
        Assert.True(segment.B == At(12));
    }
}
