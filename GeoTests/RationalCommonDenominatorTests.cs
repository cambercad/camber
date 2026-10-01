using System.Numerics;
using GeoCore;

namespace GeoTests;

public class RationalCommonDenominatorTests
{
    [Theory]
    [InlineData(7, 3)]
    [InlineData(-7, 3)]
    [InlineData(7, -3)]
    [InlineData(-7, -3)]
    public void EqualDenominatorsDoNotGrowDuringAdditionAndSubtraction(int left, int right)
    {
        var denominator = BigInteger.One << 55;
        var a = new BigRational(left, denominator);
        var b = new BigRational(right, denominator);
        Assert.Equal(new BigRational(left + right, denominator), a + b);
        Assert.Equal(new BigRational(left - right, denominator), a - b);
        Assert.Equal(denominator, (a + b).Denominator);
        Assert.Equal(denominator, (a - b).Denominator);
    }

    [Fact]
    public void ZeroAndMixedDenominatorsKeepExactSemantics()
    {
        var half = new BigRational(1, 2);
        var third = new BigRational(1, 3);
        Assert.Equal(new BigRational(5, 6), half + third);
        Assert.Equal(new BigRational(1, 6), half - third);
        Assert.Equal(new BigRational(-1, 6), third - half);
        Assert.Equal(BigInteger.One, (half - half).Denominator);
        Assert.Equal(BigInteger.Zero, (half + -half).Numerator);
        var value = new BigRationalHybrid(1, 2) + new BigRationalHybrid(1, 2);
        value.Simplify();
        Assert.Equal(BigRationalHybrid.One, value);
    }

    [Theory]
    [InlineData(5, 6, 1, 2, 1, 3)]
    [InlineData(-5, 6, 1, 2, -1, 3)]
    [InlineData(5, 6, -1, 2, 1, 3)]
    public void RemainderAndDivRemFollowTruncatingRationalSemantics(
        int an, int ad, int bn, int bd, int expectedN, int expectedD)
    {
        var dividend = new BigRational(an, ad);
        var divisor = new BigRational(bn, bd);
        var expectedRemainder = new BigRational(expectedN, expectedD);
        Assert.Equal(expectedRemainder, dividend % divisor);
        var quotient = BigRational.DivRem(dividend, divisor, out var remainder);
        Assert.Equal(dividend / divisor, quotient);
        Assert.Equal(expectedRemainder, remainder);
    }
}
