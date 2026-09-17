using System.Numerics;
using GeoCore;

namespace GeoTests;

public class ExactPredicateRangeTests
{
    [Fact]
    public void DotSignDoesNotOverflowWhenThreeWideProductsAreAdded()
    {
        var large=new BigRationalHybrid(long.MaxValue);
        var negative=new BigRationalHybrid(-long.MaxValue);
        Assert.Equal(1,BigRationalHybrid.SignOfDot3(large,large,large,large,large,large));
        Assert.Equal(-1,BigRationalHybrid.SignOfDot3(large,negative,large,negative,large,negative));
        Assert.Equal(1,BigRationalHybrid.SignOfDot3(large,large,large,large,large,negative));
    }

    [Fact]
    public void OrientationRetainsSignsForThousandBitRationals()
    {
        var zero=new BigRationalHybrid(0);
        var huge=new BigRationalHybrid(BigInteger.One << 1200,(BigInteger.One << 997)+31);
        var tiny=new BigRationalHybrid(BigInteger.One,(BigInteger.One << 1600)+7);
        Assert.Equal(1,BigRationalHybrid.SignOfOrient3D(
            huge,zero,zero, zero,huge,zero, zero,zero,tiny, zero,zero,zero));
        Assert.Equal(-1,BigRationalHybrid.SignOfOrient3D(
            zero,huge,zero, huge,zero,zero, zero,zero,tiny, zero,zero,zero));
        Assert.Equal(0,BigRationalHybrid.SignOfOrient3D(
            huge,zero,zero, zero,huge,zero, tiny,tiny,zero, zero,zero,zero));
    }

    [Fact]
    public void OrientationDoesNotOverflowOnLargeIntegerCoordinates()
    {
        var zero = new BigRationalHybrid(0);
        foreach (long scale in new[] { 1L << 43, 1L << 50, long.MaxValue })
        {
            var n = new BigRationalHybrid(scale);
            Assert.Equal(1, BigRationalHybrid.SignOfOrient3D(
                n, zero, zero, zero, n, zero, zero, zero, n, zero, zero, zero));
            Assert.Equal(-1, BigRationalHybrid.SignOfOrient3D(
                zero, n, zero, n, zero, zero, zero, zero, n, zero, zero, zero));
        }
    }

    [Fact]
    public void InCircleEitherCertifiesCorrectSignOrDefersForWideCoordinates()
    {
        var zero = new BigRationalHybrid(0);
        foreach (long scale in new[] { 100L, int.MaxValue, 1L << 40, long.MaxValue })
        {
            var n = new BigRationalHybrid(scale);
            var negative = new BigRationalHybrid(-scale);
            bool certified = BigRationalHybrid.TryInCircleSign(
                n, zero, zero, n, negative, zero, zero, zero, out int sign);
            if (certified) Assert.Equal(1, sign);
            // Also exercise differences between opposite Int32 endpoints.
            certified = BigRationalHybrid.TryInCircleSign(
                n, n, negative, n, negative, negative, n, negative, out sign);
            if (certified) Assert.Equal(0, sign);
        }
    }

    [Fact]
    public void RationalSumsPreserveUnreducedValuesWithoutMultiplyingSharedDenominators()
    {
        var random = new Random(184);
        for (int i=0;i<120;i++)
        {
            var common = BigInteger.One << (i*17);
            var a = new BigInteger(random.Next(-1000000,1000000));
            var b = common * random.Next(1,1000);
            var c = new BigInteger(random.Next(-1000000,1000000));
            var d = common * random.Next(1,1000);
            var left = new BigRational(a,b);
            var right = new BigRational(c,d);
            foreach (int sign in new[] {-1,1})
            {
                var result = sign == 1 ? left+right : left-right;
                Assert.Equal((a*d+sign*c*b)*result.Denominator, result.Numerator*b*d);
                Assert.True(result.Denominator <= b/BigInteger.GreatestCommonDivisor(b,d)*d);
            }
        }
    }

    [Fact]
    public void RationalOrientationMatchesIndependentDeterminant()
    {
        var rng = new Random(7319);
        for (int trial = 0; trial < 150; trial++)
        {
            var values = new BigRational[12];
            var hybrid = new BigRationalHybrid[12];
            for (int i = 0; i < 12; i++)
            {
                BigInteger denominator = (BigInteger.One << (40 + trial % 160)) * rng.Next(1, 18);
                BigInteger numerator = denominator * rng.Next(-1000000, 1000000) + rng.Next(-8, 9);
                values[i] = new BigRational(numerator, denominator);
                hybrid[i] = new BigRationalHybrid(numerator, denominator);
            }
            var a = Enumerable.Range(0, 3).Select(i => values[i] - values[9+i]).ToArray();
            var b = Enumerable.Range(0, 3).Select(i => values[3+i] - values[9+i]).ToArray();
            var c = Enumerable.Range(0, 3).Select(i => values[6+i] - values[9+i]).ToArray();
            int expected = (a[0]*(b[1]*c[2]-b[2]*c[1])
                - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0])).Sign;
            Assert.Equal(expected, BigRationalHybrid.SignOfOrient3DExact(
                hybrid[0], hybrid[1], hybrid[2], hybrid[3], hybrid[4], hybrid[5],
                hybrid[6], hybrid[7], hybrid[8], hybrid[9], hybrid[10], hybrid[11]));
        }
    }
}
