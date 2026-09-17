using System.Numerics;
using GeoCore;

namespace GeoTests;

public class RationalEqualityTests
{
    public static IEnumerable<object[]> EquivalentValues()
    {
        foreach (var numerator in new BigInteger[]
        {
            0, 1, -1, long.MinValue, long.MaxValue,
            (BigInteger)long.MinValue - 1, (BigInteger)long.MaxValue + 1,
            BigInteger.One << 2048, -(BigInteger.One << 2048)
        })
        foreach (var denominator in new BigInteger[] { 1, 3, BigInteger.One << 80 })
            yield return new object[] { numerator, denominator };
    }

    [Theory]
    [MemberData(nameof(EquivalentValues))]
    public void EquivalentFractionsHaveValueEqualityAndStableHashes(BigInteger numerator, BigInteger denominator)
    {
        var a = new BigRationalHybrid(numerator, denominator);
        var b = new BigRationalHybrid(-numerator * 7, -denominator * 7);
        var simplified = new BigRationalHybrid(a);
        simplified.Simplify();
        var originalRepresentation = b.DebugString;
        Assert.True(a == b);
        Assert.True(a.Equals(b));
        Assert.True(b.Equals(a));
        Assert.True(a.Equals((object)simplified));
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
        Assert.Equal(a.GetHashCode(), simplified.GetHashCode());
        Assert.Single(new HashSet<BigRationalHybrid> { a, b, simplified });
        var dictionary = new Dictionary<BigRationalHybrid, string> { [b] = "found" };
        Assert.Equal("found", dictionary[simplified]);
        Assert.Equal(originalRepresentation, b.DebugString);
        // Simplifying a shared struct copy must not invalidate a stored key.
        var shared = b;
        shared.Simplify();
        Assert.Equal("found", dictionary[a]);

        var ra = new BigRational(numerator, denominator);
        var rb = new BigRational(numerator * 7, denominator * 7);
        Assert.Equal(ra, rb);
        Assert.Equal(ra.GetHashCode(), rb.GetHashCode());
        Assert.Single(new HashSet<BigRational> { ra, rb });
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SimplifyingACopyDoesNotMutateItsSource(bool explicitCopy)
    {
        var original = new BigRationalHybrid(14, 21);
        var copy = explicitCopy ? new BigRationalHybrid(original) : original;
        copy.Simplify();
        Assert.Equal(new BigInteger(14), original.Numerator());
        Assert.Equal(new BigInteger(21), original.Denominator());
        Assert.Equal(new BigInteger(2), copy.Numerator());
        Assert.Equal(new BigInteger(3), copy.Denominator());
        Assert.Equal(original, copy);
        Assert.Equal(original.GetHashCode(), copy.GetHashCode());
    }

    [Fact]
    public void UnequalValuesWithTheSameHashRemainDistinct()
    {
        var a = BigRationalHybrid.Zero;
        // Int64 hashes XOR its two 32-bit halves; these are deliberately equal.
        var b = new BigRationalHybrid(0x0000000100000001L);
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
        Assert.False(a.Equals(b));
        var keys = new Dictionary<BigRationalHybrid, string> { [a] = "zero", [b] = "large" };
        Assert.Equal("zero", keys[new BigRationalHybrid(0, 7)]);
        Assert.Equal("large", keys[new BigRationalHybrid(new BigInteger(0x0000000100000001L) * 3, 3)]);
    }

    [Fact]
    public void GeometryDeduplicationMergesOnlyExactlyEqualPoints()
    {
        var denominator = BigInteger.One << 100;
        var a = new Rat3Hybrid(new BigRationalHybrid(2, 4), new BigRationalHybrid(6, 2), BigRationalHybrid.Zero);
        var b = new Rat3Hybrid(new BigRationalHybrid(-3, -6), new BigRationalHybrid(3), BigRationalHybrid.Zero);
        var c = new Rat3Hybrid(new BigRationalHybrid(denominator / 2 + 1, denominator), new BigRationalHybrid(3), BigRationalHybrid.Zero);
        var map = DuplicatePointRemover.DuplicateMap(new[] { a, b, c });
        Assert.Equal(0, map[0]);
        Assert.Equal(0, map[1]);
        Assert.Equal(2, map[2]);
        var points = new CSG.NewPointCreator();
        Assert.Equal(points.GetIndex(a), points.GetIndex(b));
        Assert.NotEqual(points.GetIndex(a), points.GetIndex(c));
        Assert.Equal(new BigInteger(2), a.X.Numerator());
        Assert.Equal(new BigInteger(4), a.X.Denominator());
        Assert.Equal(2, new HashSet<Rat3Hybrid> { a, b, c }.Count);
    }

    [Fact]
    public void IndependentCopiesCanNormalizeWhileTheOriginalIsRead()
    {
        var factor = (BigInteger.One << 2048) + 1;
        var original = new BigRationalHybrid(14 * factor, 21 * factor);
        int hash = original.GetHashCode();
        Parallel.For(0, 256, _ =>
        {
            var copy = original;
            copy.Simplify();
            Assert.Equal(new BigRationalHybrid(2, 3), copy);
            Assert.Equal(hash, original.GetHashCode());
            Assert.Equal(14 * factor, original.Numerator());
            Assert.Equal(21 * factor, original.Denominator());
        });
    }

    [Fact]
    public void DefaultHybridEqualsExplicitZero()
    {
        var zero = default(BigRationalHybrid);
        Assert.True(zero.Equals(BigRationalHybrid.Zero));
        Assert.Equal(BigRationalHybrid.Zero.GetHashCode(), zero.GetHashCode());
    }

    [Fact]
    public void SubDoubleDifferencesRemainDistinctInCollections()
    {
        var denominator = BigInteger.One << 200;
        var a = new BigRationalHybrid(denominator, denominator);
        var b = new BigRationalHybrid(denominator + 1, denominator);
        Assert.False(a.Equals(b));
        Assert.False(b.Equals(a));
        Assert.Equal(2, new HashSet<BigRationalHybrid> { a, b }.Count);
        var pa = new Rat3Hybrid(a, BigRationalHybrid.Zero, BigRationalHybrid.One);
        var pb = new Rat3Hybrid(b, BigRationalHybrid.Zero, BigRationalHybrid.One);
        Assert.False(pa.Equals(pb));
        Assert.Equal(2, new HashSet<Rat3Hybrid> { pa, pb }.Count);
    }
}
