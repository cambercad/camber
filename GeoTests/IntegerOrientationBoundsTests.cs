using System.Numerics;
using GeoCore;

namespace GeoTests;

public class IntegerOrientationBoundsTests
{
    private const long Scale = 1L << 20;
    private const long Limit = 1L << 40;

    // Keep an independent rational determinant as the oracle: this does not use
    // either the production LCM scaling or the bounded-integer implementation.
    private static BigRational Rational(BigRationalHybrid v) => new(v.Numerator(), v.Denominator());

    private static int Oracle(BigRationalHybrid[] q)
    {
        var a = Enumerable.Range(0, 3).Select(i => Rational(q[i]) - Rational(q[9+i])).ToArray();
        var b = Enumerable.Range(0, 3).Select(i => Rational(q[3+i]) - Rational(q[9+i])).ToArray();
        var c = Enumerable.Range(0, 3).Select(i => Rational(q[6+i]) - Rational(q[9+i])).ToArray();
        return (a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0])
            + a[2]*(b[0]*c[1]-b[1]*c[0])).Sign;
    }

    private static bool Check(BigRationalHybrid[] q)
    {
        int expected = Oracle(q);
        bool accepted = BigRationalHybrid.TrySignOfOrient3DIntegerBounds(
            q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8],q[9],q[10],q[11],out int sign);
        Assert.Equal(accepted ? expected : 0, sign);
        if (accepted) Assert.NotEqual(0, sign);
        Assert.Equal(expected, BigRationalHybrid.SignOfOrient3D(
            q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8],q[9],q[10],q[11]));
        return accepted;
    }

    private static BigRationalHybrid[] Scaled(params long[] coordinates) =>
        coordinates.Select(v => new BigRationalHybrid(v, Scale)).ToArray();

    [Theory]
    [InlineData(-1, false)]
    [InlineData(0, false)]
    [InlineData(1, true)]
    public void StrictCertificationThreshold(int offset, bool accepted)
    {
        // M=100: det = M*M*37-M*28+(48+offset) = E+offset,
        // E=36*M*M+72*M+48. Thus these are exactly adjacent integers,
        // including equality, at BOTH positive and negative thresholds.
        var q = Scaled(100,1,0, 0,100,1, 48+offset,28,37, 0,0,0);
        Assert.Equal(accepted, Check(q));
        for (int i=0;i<3;i++) (q[i],q[3+i])=(q[3+i],q[i]);
        Assert.Equal(accepted, Check(q));
    }

    [Fact]
    public void CoordinateRangeIncludesEndpointsAndRejectsNextScaledInteger()
    {
        foreach (int direction in new[] {-1,1})
        foreach (long offset in new[] {-1L,0,1})
        {
            long n=direction*(Limit+offset);
            var q=Scaled(n,0,0, 0,Limit,0, 0,0,Limit, 0,0,0);
            Assert.Equal(offset<=0, Check(q));
        }
        // Test every argument, including the translated origin, for rejection.
        for (int index=0;index<12;index++)
        foreach (int direction in new[] {-1,1})
        {
            var q=Scaled(Scale,0,0, 0,Scale,0, 0,0,Scale, 0,0,0);
            q[index]=new BigRationalHybrid(direction*(Limit+1),Scale);
            Assert.False(Check(q));
        }
    }

    [Fact]
    public void FractionalRemaindersAtCoordinateLimitTruncateTowardZero()
    {
        foreach (int direction in new[] {-1,1})
        foreach (int remainder in new[] {-1,0,1,2,3})
        {
            // q=+/-Limit is allowed even when x extends half a scaled unit
            // beyond it; q=+/-(Limit+1) must defer. Negative division matters.
            var q=Scaled(0,0,0, 0,Limit,0, 0,0,Limit, 0,0,0);
            q[0]=new BigRationalHybrid(direction*(2*Limit+remainder),2*Scale);
            Assert.Equal(remainder<2,Check(q));
        }
    }

    [Fact]
    public void OppositeRangeEndpointsDoNotOverflowIntermediateProducts()
    {
        foreach (int sign in new[] {-1,1})
        {
            var q=Scaled(Limit,Limit,-Limit, -Limit,Limit,Limit,
                Limit,-Limit,Limit, -Limit,-Limit,-Limit);
            if (sign<0) for(int i=0;i<3;i++) (q[i],q[3+i])=(q[3+i],q[i]);
            Assert.True(Check(q));
        }
    }

    [Fact]
    public void AllCoordinateLimitCornersMatchOracle()
    {
        // All 2^12 endpoint combinations: largest differences/products, both
        // determinant signs, repeated vertices and exact coplanarity.
        for(int mask=0;mask<(1<<12);mask++)
            Check(Scaled(Enumerable.Range(0,12)
                .Select(i=>(mask&(1<<i))==0 ? -Limit : Limit).ToArray()));
    }

    [Fact]
    public void MixedInt32BoundaryCoordinatesMatchOracle()
    {
        long[] limits={(long)int.MinValue-1,int.MinValue,(long)int.MinValue+1,
            -1,0,1,(long)int.MaxValue-1,int.MaxValue,(long)int.MaxValue+1};
        var random=new Random(9274);
        for(int trial=0;trial<500;trial++)
            Check(Enumerable.Range(0,12)
                .Select(_=>new BigRationalHybrid(limits[random.Next(limits.Length)])).ToArray());
    }

    [Fact]
    public void IntegerFastPathBoundaryAndFullInt64RangeMatchOracle()
    {
        foreach (long n in new[] {(long)int.MinValue-1,int.MinValue,(long)int.MinValue+1,
            (long)int.MaxValue-1,int.MaxValue,(long)int.MaxValue+1,long.MinValue,long.MaxValue})
        {
            var q=new long[] {n,0,0, 0,n,0, 0,0,n, 1,-1,1}
                .Select(v=>new BigRationalHybrid(v)).ToArray();
            Check(q);
        }
        // Exercise the integer conversion guard in the bound filter itself.
        foreach (long n in new[] {-Scale-1,-Scale,-Scale+1,Scale-1,Scale,Scale+1})
        {
            var q=new long[] {n,0,0,0,Scale,0,0,0,Scale,0,0,0}
                .Select(v=>new BigRationalHybrid(v)).ToArray();
            Assert.Equal(Math.Abs(n)<=Scale,Check(q));
        }
    }

    [Fact]
    public void TinySignedOffsetsAndExactCoplanarityDeferToExactArithmetic()
    {
        foreach (int bits in new[] {21,53,128,1024,4096})
        foreach (int sign in new[] {-1,0,1})
        {
            var q=Scaled(Scale,0,0,0,Scale,0,Scale,Scale,0,0,0,0);
            q[8]=new BigRationalHybrid(sign,(BigInteger.One<<bits)+7);
            Assert.Equal(sign,Oracle(q));
            Assert.False(Check(q));
        }
    }

    [Fact]
    public void ThousandBitTranslationsDoNotHideTinyDifferences()
    {
        foreach (int sign in new[] {-1,0,1})
        {
            var translation=new BigRationalHybrid(BigInteger.One<<1600,3);
            var tiny=new BigRationalHybrid(1,(BigInteger.One<<1100)+7);
            var q=Enumerable.Repeat(translation,12).ToArray();
            q[0]+=tiny; q[4]+=tiny; q[8]+=new BigRationalHybrid(sign)*tiny;
            Assert.Equal(sign,Oracle(q));
            Assert.False(Check(q));
        }
    }

    [Fact]
    public void EveryPointPermutationMatchesOracle()
    {
        foreach (long offset in new[] {-1L,0,1})
        {
            var original=Scaled(100,1,0,0,100,1,48+offset,28,37,0,0,0);
            for(int a=0;a<4;a++)
            for(int b=0;b<4;b++)
            for(int c=0;c<4;c++)
            for(int d=0;d<4;d++)
            {
                int[] order={a,b,c,d};
                if(order.Distinct().Count()!=4) continue;
                var q=order.SelectMany(point=>original.Skip(3*point).Take(3)).ToArray();
                // Changing the origin can change the conservative bound, but
                // must never change the exact answer or certify a wrong sign.
                Check(q);
            }
        }
    }

    [Fact]
    public void FractionalErrorsNearCertificationThresholdMatchOracle()
    {
        var random=new Random(7291);
        foreach (long offset in new[] {-1L,0,1})
        for(int trial=0;trial<200;trial++)
        {
            long[] integers={100,1,0,0,100,1,48+offset,28,37,0,0,0};
            var q=integers.Select(n=>new BigRationalHybrid(
                1000*n+random.Next(-999,1000),1000*Scale)).ToArray();
            Check(q);
        }
    }

    [Fact]
    public void EquivalentUnreducedAndNegativeDenominatorRepresentationsAgree()
    {
        var original=Scaled(100,1,0,0,100,1,49,28,37,0,0,0);
        foreach(int sign in new[] {-1,1})
        {
            BigInteger factor=sign*((BigInteger.One<<1200)+37);
            var q=original.Select(v=>new BigRationalHybrid(v.Numerator()*factor,v.Denominator()*factor)).ToArray();
            Assert.Equal(Oracle(original),Oracle(q));
            Assert.True(Check(q));
            for(int i=0;i<12;i++) q[i].Simplify();
            Assert.True(Check(q));
        }
    }

    [Fact]
    public void LegacyFilterEntryPointAlsoUsesExactIntegerBounds()
    {
        var q=Scaled(100,1,0,0,100,1,49,28,37,0,0,0);
#pragma warning disable CS0618 // Deliberately exercise the compatibility API.
        bool accepted=BigRationalHybrid.TrySignOfOrient3DDoubleFilter(
            q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8],q[9],q[10],q[11],out int sign);
#pragma warning restore CS0618
        Assert.True(accepted);
        Assert.Equal(Oracle(q),sign);
    }

    [Fact]
    public void RationalRoundingAndCancellationMatchOracle()
    {
        var random=new Random(81259);
        int certified=0, deferred=0;
        for(int trial=0;trial<1000;trial++)
        {
            var q=new BigRationalHybrid[12];
            BigInteger denominator=(BigInteger.One<<(21+trial%180))+random.Next(1,1000);
            for(int i=0;i<12;i++)
            {
                long whole=random.Next(-1000,1001);
                // Include signed values on both sides of scaled integer boundaries.
                q[i]=new BigRationalHybrid(whole*denominator+random.Next(-2,3),denominator*Scale);
            }
            if(trial%3==0) for(int i=0;i<3;i++) q[6+i]=q[i]+q[3+i]-q[9+i];
            if(Check(q)) certified++; else deferred++;
        }
        Assert.True(certified>100);
        Assert.True(deferred>100);
    }
}
