using System.Numerics;
using GeoCore;

namespace GeoTests;

public class ExactDoubleConversionTests
{
    [Fact]
    public void FiniteDoublesPreserveTheirExactBinaryValues()
    {
        var cases = new (double value, BigInteger numerator, BigInteger denominator)[]
        {
            (0, 0, 1), (-0.0, 0, 1), (1, 1, 1), (.5, 1, 2), (-2.75, -11, 4),
            (.1, BigInteger.Parse("3602879701896397"), BigInteger.One << 55),
            (double.Epsilon, 1, BigInteger.One << 1074),
            (Math.ScaleB(1, -1022), 1, BigInteger.One << 1022),
            (double.MaxValue, ((BigInteger.One << 53)-1) << 971, 1),
        };
        foreach (var (value, numerator, denominator) in cases)
        {
            var actual = new BigRational(value);
            Assert.Equal(numerator, actual.Numerator);
            Assert.Equal(denominator, actual.Denominator);
        }
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void NonFiniteInputIsRejected(double value) =>
        Assert.Throws<ArgumentException>(() => new BigRational(value));
}
