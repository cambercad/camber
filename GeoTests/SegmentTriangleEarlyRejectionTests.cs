using CSG;
using GeoCore;

namespace GeoTests;

public class SegmentTriangleEarlyRejectionTests
{
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
