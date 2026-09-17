using System.Numerics;
using CSG;
using GeoCore;

namespace GeoTests;

public class ResolverTriangleBoundsTests
{
    private static void Check(Rat3Hybrid a, Rat3Hybrid b, Rat3Hybrid c)
    {
        var points = new NewPointCreator();
        var triangle = new ResolverTriangle(points.GetIndex(a), points.GetIndex(b), points.GetIndex(c), 0, points);
        var actual = triangle.GetBounds(points);
        // Previous exact-extrema algorithm is the equivalence oracle.
        var min = a;
        var max = a;
        foreach (var p in new[] { b, c })
        {
            min = new Rat3Hybrid(BigRationalHybrid.Min(min.X,p.X),BigRationalHybrid.Min(min.Y,p.Y),BigRationalHybrid.Min(min.Z,p.Z));
            max = new Rat3Hybrid(BigRationalHybrid.Max(max.X,p.X),BigRationalHybrid.Max(max.Y,p.Y),BigRationalHybrid.Max(max.Z,p.Z));
        }
        var expected = min.GetBox(); expected.Extend(max.GetBox());
        Assert.Equal(expected.Min,actual.Min);
        Assert.Equal(expected.Max,actual.Max);
        // Independently check the actual enclosure with rational arithmetic.
        foreach (var p in new[] { a, b, c })
        for (int axis = 0; axis < 3; axis++)
        {
            Assert.True(new BigRationalHybrid(axis == 0 ? actual.Min.X : axis == 1 ? actual.Min.Y : actual.Min.Z) <= p[axis]);
            Assert.True(new BigRationalHybrid(axis == 0 ? actual.Max.X : axis == 1 ? actual.Max.Y : actual.Max.Z) >= p[axis]);
        }
    }

    [Fact]
    public void IntegerBoundariesBothSignsAndTinyOffsetsPreserveExactBox()
    {
        var tiny = new BigRationalHybrid(1,(BigInteger.One<<1600)+7);
        foreach (int boundary in new[] { -2147483646,-101,-1,0,1,101,2147483646 })
        {
            var n = new BigRationalHybrid(boundary);
            var values = new[] { n-tiny,n,n+tiny };
            for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++)
            for (int c = 0; c < 3; c++)
                Check(new(values[a],values[b],values[c]),new(values[c],values[a],values[b]),new(values[b],values[c],values[a]));
        }
    }

    [Fact]
    public void ArbitraryDenominatorsAndVertexPermutationsPreserveExactBox()
    {
        var rng = new Random(37517);
        for (int trial = 0; trial < 300; trial++)
        {
            BigRationalHybrid Value()
            {
                BigInteger den = (BigInteger.One << (1+trial%500)) + rng.Next(1,100);
                return new BigRationalHybrid(den*rng.Next(-10000,10001)+rng.Next(-100,101),den);
            }
            var a = new Rat3Hybrid(Value(),Value(),Value());
            var b = new Rat3Hybrid(Value(),Value(),Value());
            var c = new Rat3Hybrid(Value(),Value(),Value());
            Check(a,b,c); Check(c,a,b); Check(b,c,a);
            Check(a,a,a); // Degenerate triangles still have a conservative box.
        }
    }
}
