using System.Numerics;
using GeoCore;

namespace GeoTests;

/// <summary>
/// The Orient3D double filter must never return a sign that disagrees with the exact path.
/// Abstaining (returning false) is always allowed.
/// </summary>
public sealed class Orient3DFilterTests
{
    [Fact]
    public void DoubleFilter_NeverDisagreesWithExact_RandomRationals()
    {
        var rng = new Random(20260912);
        int filtered = 0;

        for (int i = 0; i < 8000; ++i)
        {
            BigRationalHybrid ax = RandomCoord(rng), ay = RandomCoord(rng), az = RandomCoord(rng);
            BigRationalHybrid bx = RandomCoord(rng), by = RandomCoord(rng), bz = RandomCoord(rng);
            BigRationalHybrid cx = RandomCoord(rng), cy = RandomCoord(rng), cz = RandomCoord(rng);
            BigRationalHybrid dx = RandomCoord(rng), dy = RandomCoord(rng), dz = RandomCoord(rng);

            AssertFilterAgrees(in ax, in ay, in az, in bx, in by, in bz, in cx, in cy, in cz, in dx, in dy, in dz,
                ref filtered);
        }

        Assert.True(filtered > 0, "Filter should certify at least some random orientations.");
        // Random points are rarely near-degenerate; abstaining is allowed but not required.
    }

    [Fact]
    public void DoubleFilter_NeverDisagreesWithExact_NearCoplanar()
    {
        var rng = new Random(7);
        int filtered = 0;

        for (int i = 0; i < 2000; ++i)
        {
            BigRationalHybrid ax = RandomCoord(rng, 20), ay = RandomCoord(rng, 20), az = new BigRationalHybrid(0);
            BigRationalHybrid bx = RandomCoord(rng, 20), by = RandomCoord(rng, 20), bz = new BigRationalHybrid(0);
            BigRationalHybrid cx = RandomCoord(rng, 20), cy = RandomCoord(rng, 20), cz = new BigRationalHybrid(0);
            BigRationalHybrid dx = RandomCoord(rng, 20), dy = RandomCoord(rng, 20);
            // Tiny off-plane offset so the exact sign is often nonzero but easy to lose in double.
            BigRationalHybrid dz = new BigRationalHybrid(new BigInteger(rng.Next(-3, 4)), new BigInteger(1_000_000_007));

            AssertFilterAgrees(in ax, in ay, in az, in bx, in by, in bz, in cx, in cy, in cz, in dx, in dy, in dz,
                ref filtered);
        }

        Assert.True(2000 - filtered > 0, "Near-coplanar inputs should usually abstain.");
    }

    [Fact]
    public void SignOfOrient3D_MatchesExact_WhenFilterFiresOrFallsBack()
    {
        var rng = new Random(99);
        for (int i = 0; i < 2000; ++i)
        {
            BigRationalHybrid ax = RandomCoord(rng), ay = RandomCoord(rng), az = RandomCoord(rng);
            BigRationalHybrid bx = RandomCoord(rng), by = RandomCoord(rng), bz = RandomCoord(rng);
            BigRationalHybrid cx = RandomCoord(rng), cy = RandomCoord(rng), cz = RandomCoord(rng);
            BigRationalHybrid dx = RandomCoord(rng), dy = RandomCoord(rng), dz = RandomCoord(rng);

            int published = BigRationalHybrid.SignOfOrient3D(
                in ax, in ay, in az, in bx, in by, in bz, in cx, in cy, in cz, in dx, in dy, in dz);
            int exact = BigRationalHybrid.SignOfOrient3DExact(
                in ax, in ay, in az, in bx, in by, in bz, in cx, in cy, in cz, in dx, in dy, in dz);
            Assert.Equal(exact, published);
        }
    }

    private static void AssertFilterAgrees(
        in BigRationalHybrid ax, in BigRationalHybrid ay, in BigRationalHybrid az,
        in BigRationalHybrid bx, in BigRationalHybrid by, in BigRationalHybrid bz,
        in BigRationalHybrid cx, in BigRationalHybrid cy, in BigRationalHybrid cz,
        in BigRationalHybrid dx, in BigRationalHybrid dy, in BigRationalHybrid dz,
        ref int filtered)
    {
        int exact = BigRationalHybrid.SignOfOrient3DExact(
            in ax, in ay, in az, in bx, in by, in bz, in cx, in cy, in cz, in dx, in dy, in dz);
        int filteredSign;
        if (BigRationalHybrid.TrySignOfOrient3DDoubleFilter(
                in ax, in ay, in az, in bx, in by, in bz, in cx, in cy, in cz, in dx, in dy, in dz,
                out filteredSign))
        {
            Assert.Equal(exact, filteredSign);
            filtered++;
        }
    }

    private static BigRationalHybrid RandomCoord(Random rng, int maxAbs = 500)
    {
        int mode = rng.Next(4);
        if (mode == 0)
            return new BigRationalHybrid(rng.Next(-maxAbs, maxAbs + 1));

        long num = rng.Next(-maxAbs * 17, maxAbs * 17 + 1);
        long den = rng.Next(1, maxAbs * 13 + 2);
        return new BigRationalHybrid(num, den);
    }
}
