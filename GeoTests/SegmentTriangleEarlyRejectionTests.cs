using CSG;
using GeoCore;
using System.Numerics;

namespace GeoTests;

public class SegmentTriangleEarlyRejectionTests
{
    [Fact]
    public void SharedPairPredicatesMatchUncachedIntersectionsExactly()
    {
        var a = new Rat3Hybrid[] { new(0, 0, 0), new(10, 0, 0), new(0, 10, 0) };
        Check(a, [new(2, 2, -1), new(2, 2, 1), new(7, 1, 0)]);
        Check(a, [new(2, 2, 0), new(7, 2, 0), new(2, 7, 0)]);
        Check(a, [new(0, 0, 0), new(0, 0, 5), new(-5, 0, 0)]);
        Check(a, a);
        var random = new Random(29157);
        for (int i = 0; i < 250; i++)
        {
            BigInteger denominator = (BigInteger.One << (1 + i % 160)) + 7;
            Rat3Hybrid Point() => new(new BigRationalHybrid(random.Next(-100, 101), denominator),
                new BigRationalHybrid(random.Next(-100, 101), denominator + 2),
                new BigRationalHybrid(random.Next(-100, 101), denominator + 4));
            Check([Point(), Point(), Point()], [Point(), Point(), Point()]);
        }

        static void Check(Rat3Hybrid[] first, Rat3Hybrid[] second)
        {
            // Populate lazily in each direction, including the transposed wedge
            // lookups and repeated vertex-plane signs on the second pass.
            Span<sbyte> signs = stackalloc sbyte[15];
            signs.Fill(2);
            for (int pass = 0; pass < 2; pass++)
            {
                var edge = pass == 0 ? first : second;
                var face = pass == 0 ? second : first;
                for (int i = 0; i < 3; i++)
                {
                    var expected = TriangleSegmentIntersector.SegmentIntersectsTriangle(edge[i], edge[(i + 1) % 3],
                        face[0], face[1], face[2], out var point, out bool boundary, out bool start, out bool end);
                    var actual = TriangleSegmentIntersector.SegmentIntersectsTriangle(edge[i], edge[(i + 1) % 3],
                        face[0], face[1], face[2], out var cachedPoint, out bool cachedBoundary, out bool cachedStart,
                        out bool cachedEnd, null, signs, i, pass == 1);
                    Assert.Equal(expected, actual);
                    Assert.Equal(boundary, cachedBoundary);
                    Assert.Equal(start, cachedStart);
                    Assert.Equal(end, cachedEnd);
                    if (actual == SegmentTriangleIntersectionType.Intersect)
                    {
                        Assert.Equal(point, cachedPoint);
                        for (int axis = 0; axis < 3; axis++)
                            Assert.Equal(BitConverter.DoubleToInt64Bits(point[axis].ToDouble()),
                                BitConverter.DoubleToInt64Bits(cachedPoint[axis].ToDouble()));
                    }
                }
            }
        }
    }

    [Fact]
    public void ContradictoryExactWedgesRejectWithoutBoundaryFlags()
    {
        var random = new Random(8137);
        int rejected = 0;
        Rat3Hybrid Point() => new(new BigRationalHybrid(random.Next(-1000,1001),997),
            new BigRationalHybrid(random.Next(-1000,1001),991),
            new BigRationalHybrid(random.Next(-1000,1001),983));
        for (int i = 0; i < 2000; i++)
        {
            var start=Point(); var end=Point(); var a=Point(); var b=Point(); var c=Point();
            int ab=TriangleSegmentIntersector.Orient3DSign(start,end,a,b);
            int bc=TriangleSegmentIntersector.Orient3DSign(start,end,b,c);
            if (ab*bc >= 0) continue;
            rejected++;
            var result=TriangleSegmentIntersector.SegmentIntersectsTriangle(start,end,a,b,c,
                out _,out bool boundary,out bool onStart,out bool onEnd);
            Assert.Equal(SegmentTriangleIntersectionType.NoIntersection,result);
            Assert.False(boundary); Assert.False(onStart); Assert.False(onEnd);
        }
        Assert.True(rejected>500);
    }

    [Fact]
    public void ZeroWedgesPreserveBoundaryEndpointAndCoplanarCases()
    {
        var a=new Rat3Hybrid(0,0,0); var b=new Rat3Hybrid(10,0,0); var c=new Rat3Hybrid(0,10,0);
        foreach (int winding in new[] {1,-1})
        {
            var tb=winding==1?b:c; var tc=winding==1?c:b;
            var result=TriangleSegmentIntersector.SegmentIntersectsTriangle(new(5,0,-1),new(5,0,1),a,tb,tc,
                out var hit,out bool boundary,out bool start,out bool end);
            Assert.Equal(SegmentTriangleIntersectionType.Intersect,result);
            hit.Simplify();
            Assert.Equal(new Rat3Hybrid(5,0,0),hit); Assert.True(boundary); Assert.False(start); Assert.False(end);
            result=TriangleSegmentIntersector.SegmentIntersectsTriangle(new(2,2,0),new(2,2,1),a,tb,tc,
                out hit,out boundary,out start,out end);
            Assert.Equal(SegmentTriangleIntersectionType.Intersect,result);
            hit.Simplify();
            Assert.Equal(new Rat3Hybrid(2,2,0),hit); Assert.True(start); Assert.False(end);
            result=TriangleSegmentIntersector.SegmentIntersectsTriangle(new(2,2,0),new(3,3,0),a,tb,tc,
                out _,out _,out _,out _);
            Assert.Equal(SegmentTriangleIntersectionType.Coplanar,result);
        }
    }
}
