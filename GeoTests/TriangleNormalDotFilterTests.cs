using System.Numerics;
using GeoCore;

namespace GeoTests;

public class TriangleNormalDotFilterTests
{
    private static BigRationalHybrid Rational(Random random, int kind)
    {
        switch (kind)
        {
            case 0: return new BigRationalHybrid(random.Next(-50, 51));
            case 1: return new BigRationalHybrid(new BigInteger(random.NextInt64(-1_000_000, 1_000_000)),
                new BigInteger(random.NextInt64(1, 1_000_000)));
            case 2:
                // Beyond the Int64 fast path, so the cached double approximation is used.
                return new BigRationalHybrid(new BigInteger(random.NextInt64(-1L << 60, 1L << 60)) * 1_000_003 + 17,
                    new BigInteger(random.NextInt64(1, 1L << 50)) * 999_983);
            case 3: return BigRationalHybrid.FromDouble((random.NextDouble() - .5) * 2000);
            default:
                // Multi-limb numerator and denominator (about 200 and 150 bits).
                return new BigRationalHybrid(new BigInteger(random.NextInt64(-1L << 60, 1L << 60)) * BigInteger.Pow(3, 90) + 12345,
                    new BigInteger(random.NextInt64(1, 1L << 50)) * BigInteger.Pow(7, 55) + 1);
        }
    }

    private static Rat3Hybrid Point(Random random, int kind) =>
        new(Rational(random, kind), Rational(random, kind), Rational(random, kind));

    private static int Reference(Rat3Hybrid a1, Rat3Hybrid b1, Rat3Hybrid c1, Rat3Hybrid a0, Rat3Hybrid b0, Rat3Hybrid c0) =>
        Rat3Hybrid.SignOfCrossDot(b1 - a1, c1 - a1, b0 - a0, c0 - a0);

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void FilteredSignMatchesExactArithmetic(int kind)
    {
        var random = new Random(1234 + kind);
        for (int i = 0; i < 3000; i++)
        {
            var a0 = Point(random, kind); var b0 = Point(random, kind); var c0 = Point(random, kind);
            var a1 = Point(random, kind); var b1 = Point(random, kind); var c1 = Point(random, kind);
            Assert.Equal(Reference(a1, b1, c1, a0, b0, c0), Rat3Hybrid.SignOfTriangleNormalDot(a1, b1, c1, a0, b0, c0));
            // Moving one corner slightly (the collapse situation) keeps most triangles nearly unchanged.
            var moved = new Rat3Hybrid(a0.X + new BigRationalHybrid(1, 1000), a0.Y, a0.Z);
            Assert.Equal(Reference(moved, b0, c0, a0, b0, c0), Rat3Hybrid.SignOfTriangleNormalDot(moved, b0, c0, a0, b0, c0));
        }
    }

    [Fact]
    public void ExactlyDegenerateAndNearlyDegenerateTrianglesAreDecidedExactly()
    {
        var random = new Random(77);
        for (int i = 0; i < 3000; i++)
        {
            var a0 = Point(random, 1); var b0 = Point(random, 1); var c0 = Point(random, 1);
            // Collinear new triangle: dot is exactly zero.
            var t = new BigRationalHybrid(random.Next(-9, 10), random.Next(1, 9));
            var c1 = a0 + (b0 - a0) * t;
            Assert.Equal(0, Rat3Hybrid.SignOfTriangleNormalDot(a0, b0, c1, a0, b0, c0));
            Assert.Equal(Reference(a0, b0, c1, a0, b0, c0), Rat3Hybrid.SignOfTriangleNormalDot(a0, b0, c1, a0, b0, c0));
            // Collinear up to a tiny perturbation far below double resolution.
            var tiny = new BigRationalHybrid(BigInteger.One, BigInteger.Pow(10, 40));
            var nearly = new Rat3Hybrid(c1.X + tiny, c1.Y, c1.Z);
            Assert.Equal(Reference(a0, b0, nearly, a0, b0, c0), Rat3Hybrid.SignOfTriangleNormalDot(a0, b0, nearly, a0, b0, c0));
        }
    }

    [Fact]
    public void ConvertedDoublesAreExactRationals()
    {
        var random = new Random(5);
        double[] special = { 0, -0.0, 1, -1, .1, double.Epsilon, double.MaxValue, -double.MaxValue, 1e-300, 123456789.125 };
        for (int i = 0; i < 5000; i++)
        {
            double v = i < special.Length ? special[i]
                : BitConverter.Int64BitsToDouble(random.NextInt64()) is var d && double.IsFinite(d) ? d : 1.5;
            var fast = BigRationalHybrid.FromDouble(v);
            // BigRational.ToDouble has never been exact for subnormal or near-overflow magnitudes.
            if (v == 0 || (Math.Abs(v) > 1e-290 && Math.Abs(v) < 1e290))
                Assert.Equal(v, fast.ToDouble());
            // Independent decomposition: v = sign * significand * 2^exponent.
            long bits = BitConverter.DoubleToInt64Bits(v);
            int exponentBits = (int)((bits >> 52) & 0x7ff);
            BigInteger significand = bits & 0xfffffffffffffL;
            int exponent = exponentBits == 0 ? -1074 : exponentBits - 1075;
            if (exponentBits != 0) significand |= BigInteger.One << 52;
            if (bits < 0) significand = -significand;
            BigInteger expectedNumerator = exponent >= 0 ? significand << exponent : significand;
            BigInteger expectedDenominator = exponent >= 0 ? BigInteger.One : BigInteger.One << -exponent;
            BigInteger gcd = BigInteger.GreatestCommonDivisor(expectedNumerator, expectedDenominator);
            if (gcd.IsZero) gcd = BigInteger.One;
            Assert.Equal(expectedNumerator / gcd, fast.Numerator());
            Assert.Equal(expectedDenominator / gcd, fast.Denominator());
            Assert.Equal(BigInteger.One, BigInteger.GreatestCommonDivisor(fast.Numerator(), fast.Denominator()) is var g && g.IsZero ? BigInteger.One : g);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void OrientAndInCircleFiltersNeverContradictExactArithmetic(int kind)
    {
        var random = new Random(900 + kind);
        int decided = 0;
        for (int i = 0; i < 4000; i++)
        {
            var a = Point(random, kind); var b = Point(random, kind); var c = Point(random, kind); var d = Point(random, kind);
            // Make every third case exactly coplanar / cocircular-ish so zero results are covered.
            if (i % 3 == 0) d = a + (b - a) * new BigRationalHybrid(random.Next(-5, 6), random.Next(1, 5)) + (c - a) * new BigRationalHybrid(random.Next(-5, 6), random.Next(1, 5));
            int exact = BigRationalHybrid.SignOfOrient3DExact(a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z, d.X, d.Y, d.Z);
            if (FloatFilter.TryOrient3DSign(a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z, d.X, d.Y, d.Z, out int filtered))
            {
                decided++;
                Assert.Equal(exact, filtered);
            }
            Assert.Equal(exact, BigRationalHybrid.SignOfOrient3D(a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z, d.X, d.Y, d.Z));

            var adx = a.X - d.X; var ady = a.Y - d.Y; var bdx = b.X - d.X; var bdy = b.Y - d.Y; var cdx = c.X - d.X; var cdy = c.Y - d.Y;
            var result = (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy) + (bdx * bdx + bdy * bdy) * (cdx * ady - adx * cdy)
                + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
            int exactCircle = result.Sign();
            if (FloatFilter.TryInCircleSign(a.X, a.Y, b.X, b.Y, c.X, c.Y, d.X, d.Y, out int circle))
                Assert.Equal(exactCircle, circle);
        }
        Assert.True(decided > 1000, "The filter should settle most random cases: " + decided);
    }
}
