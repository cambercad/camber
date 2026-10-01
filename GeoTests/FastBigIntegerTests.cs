using System.Numerics;
using GeoCore;

public class FastBigIntegerTests
{
    [Fact]
    public void ArithmeticMatchesBigIntegerAcrossBoundariesAndLargeValues()
    {
        var random = new Random(928471);
        BigInteger[] boundaries =
        [
            0, 1, -1, long.MinValue, long.MaxValue,
            (BigInteger)long.MaxValue + 1, (BigInteger)long.MinValue - 1,
            BigInteger.One << 64, -(BigInteger.One << 64),
            (BigInteger.One << 127) - 1, -(BigInteger.One << 127),
            (BigInteger.One << 512) + 17, -((BigInteger.One << 512) + 17),
            (BigInteger.One << 2048) + 257, -((BigInteger.One << 2048) + 257),
            (BigInteger.One << 4096) + 65537
        ];

        foreach (var a in boundaries)
            foreach (var b in boundaries)
                Check(a, b);

        for (int i = 0; i < 1500; i++)
        {
            byte[] aBytes = new byte[random.Next(1, 129)];
            byte[] bBytes = new byte[random.Next(1, 129)];
            random.NextBytes(aBytes);
            random.NextBytes(bBytes);
            var a = new BigInteger(aBytes, isUnsigned: true);
            var b = new BigInteger(bBytes, isUnsigned: true);
            if ((i & 1) != 0) a = -a;
            if ((i & 2) != 0) b = -b;
            Check(a, b);
        }
        for (int i = 0; i < 50; i++)
        {
            byte[] aBytes = new byte[random.Next(193, 513)];
            byte[] bBytes = new byte[random.Next(193, 513)];
            random.NextBytes(aBytes);
            random.NextBytes(bBytes);
            Check(new BigInteger(aBytes, isUnsigned: true), -new BigInteger(bBytes, isUnsigned: true));
        }
    }

    private static void Check(BigInteger a, BigInteger b)
    {
        var x = new FastBigInteger(a);
        var y = new FastBigInteger(b);
        Assert.Equal(a, x.ToBigInteger());
        Assert.Equal(b, y.ToBigInteger());
        Assert.Equal(a + b, (x + y).ToBigInteger());
        Assert.Equal(a - b, (x - y).ToBigInteger());
        Assert.Equal(a * b, (x * y).ToBigInteger());
        Assert.Equal(-a, (-x).ToBigInteger());
        Assert.Equal(Math.Sign(a.CompareTo(b)), Math.Sign(x.CompareTo(y)));
        Assert.Equal(BigInteger.GreatestCommonDivisor(a, b), FastBigInteger.GreatestCommonDivisor(x, y).ToBigInteger());
        if (b != 0)
        {
            Assert.Equal(a / b, (x / y).ToBigInteger());
            Assert.Equal(a % b, (x % y).ToBigInteger());
        }
        Assert.Equal(a, x.ToBigInteger()); // arithmetic never changes an aliased input
    }

    [Fact]
    public void BinaryGcdWorksWithoutRuntimeGcd()
    {
        BigInteger[] values =
        [
            0, 1, -1, long.MinValue,
            (BigInteger.One << 128) + 3,
            (BigInteger.One << 256) - 7,
            -((BigInteger.One << 1024) + 9),
            (BigInteger.One << 2048) + 17
        ];
        foreach (BigInteger a in values)
            foreach (BigInteger b in values)
                Assert.Equal(BigInteger.GreatestCommonDivisor(a, b),
                    FastBigInteger.BinaryGreatestCommonDivisor(new FastBigInteger(a), new FastBigInteger(b)).ToBigInteger());
    }

    [Fact]
    public void RationalOperationsMatchBigIntegerFormulas()
    {
        var random = new Random(482947);
        for (int i = 0; i < 400; i++)
        {
            BigInteger an = RandomValue(random), bn = RandomValue(random);
            BigInteger ad = BigInteger.Abs(RandomValue(random)) + 1;
            BigInteger bd = BigInteger.Abs(RandomValue(random)) + 1;
            var a = new BigRational(an, ad);
            var b = new BigRational(bn, bd);
            Assert.Equal(Math.Sign((an * bd).CompareTo(bn * ad)), Math.Sign(a.CompareTo(b)));
            Assert.Equal(new BigRational(an * bd + bn * ad, ad * bd), a + b);
            Assert.Equal(new BigRational(an * bd - bn * ad, ad * bd), a - b);
            Assert.Equal(new BigRational(an * bn, ad * bd), a * b);
            if (bn != 0) Assert.Equal(new BigRational(an * bd, ad * bn), a / b);
        }
    }

    [Fact]
    public void RationalSmallPathHandlesSignedInt128Overflow()
    {
        long[] values = [0, 1, -1, long.MinValue, long.MinValue + 1, long.MaxValue - 1, long.MaxValue];
        long[] denominators = [1, 2, 3, long.MaxValue - 1, long.MaxValue];
        foreach (long an in values)
            foreach (long bn in values)
                foreach (long ad in denominators)
                    foreach (long bd in denominators)
                    {
                        var a = new BigRational(an, ad);
                        var b = new BigRational(bn, bd);
                        Assert.Equal(new BigRational((BigInteger)an * bd + (BigInteger)bn * ad, (BigInteger)ad * bd), a + b);
                        Assert.Equal(new BigRational((BigInteger)an * bd - (BigInteger)bn * ad, (BigInteger)ad * bd), a - b);
                    }
    }

    [Fact]
    public void CrossProductSignMatchesExactFormulaIncludingInt128Overflow()
    {
        Assert.Equal(0, BigRationalHybrid.SignOfCrossProduct(
            default, new BigRationalHybrid(7), new BigRationalHybrid(0), new BigRationalHybrid(3)));
        long[] edges = [long.MinValue, long.MinValue + 1, -1, 0, 1, long.MaxValue - 1, long.MaxValue];
        foreach (long ux in edges)
            foreach (long vy in edges)
                foreach (long uy in edges)
                {
                    long vx = edges[(int)(((ulong)ux ^ (ulong)vy ^ (ulong)uy) % (ulong)edges.Length)];
                    int expected = ((BigInteger)ux * vy - (BigInteger)uy * vx).Sign;
                    Assert.Equal(expected, BigRationalHybrid.SignOfCrossProduct(
                        new BigRationalHybrid(ux), new BigRationalHybrid(vy),
                        new BigRationalHybrid(uy), new BigRationalHybrid(vx)));
                }

        var random = new Random(839472);
        for (int i = 0; i < 300; i++)
        {
            BigInteger[] n = [RandomValue(random), RandomValue(random), RandomValue(random), RandomValue(random)];
            BigInteger[] d = [BigInteger.Abs(RandomValue(random)) + 1, BigInteger.Abs(RandomValue(random)) + 1,
                BigInteger.Abs(RandomValue(random)) + 1, BigInteger.Abs(RandomValue(random)) + 1];
            var values = new BigRationalHybrid[4];
            for (int j = 0; j < 4; j++) values[j] = new BigRationalHybrid(n[j], d[j]);
            int expected = (n[0] * n[1] * d[2] * d[3] - n[2] * n[3] * d[0] * d[1]).Sign;
            Assert.Equal(expected, BigRationalHybrid.SignOfCrossProduct(values[0], values[1], values[2], values[3]));
        }
    }

    [Fact]
    public void MixedIntegerRationalArithmeticMatchesExactFormula()
    {
        long[] integers = [long.MinValue, -47, 0, 39, long.MaxValue];
        BigInteger[] numerators = [0, 1, -1, (BigInteger.One << 100) + 7, -((BigInteger.One << 130) + 3)];
        BigInteger[] denominators = [1, 3, (BigInteger.One << 80) + 11];
        foreach (long integer in integers)
            foreach (BigInteger numerator in numerators)
                foreach (BigInteger denominator in denominators)
                {
                    var whole = new BigRationalHybrid(integer);
                    var fraction = new BigRationalHybrid(numerator, denominator);
                    Assert.Equal(new BigRationalHybrid(numerator + (BigInteger)integer * denominator, denominator), fraction + whole);
                    Assert.Equal(new BigRationalHybrid(numerator + (BigInteger)integer * denominator, denominator), whole + fraction);
                    Assert.Equal(new BigRationalHybrid(numerator - (BigInteger)integer * denominator, denominator), fraction - whole);
                    Assert.Equal(new BigRationalHybrid((BigInteger)integer * denominator - numerator, denominator), whole - fraction);
                }
    }

    private static BigInteger RandomValue(Random random)
    {
        byte[] bytes = new byte[random.Next(1, 33)];
        random.NextBytes(bytes);
        BigInteger value = new(bytes, isUnsigned: true);
        return random.Next(2) == 0 ? value : -value;
    }
}
