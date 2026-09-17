using CSG;
using GeoCore;

namespace GeoTests;

public class SplitterArrangementTests
{
    [Fact]
    public void CrossingPointOnSharedConstraintIsPropagatedToBothTriangles()
    {
        Rat3Hybrid Point(int x,int y)=>new(x,y,0);
        var surface=new Surface([new Tri(0,1,2),new Tri(0,2,3)],
            [Point(0,0),Point(4,0),Point(4,4),Point(0,4)]);
        List<List<LineSegmentOnTriangle>> curves=[
            [new(Point(0,0),Point(4,4),0)],
            [new(Point(4,4),Point(0,0),1)],
            [new(Point(0,2),Point(2,2),1)]];
        var regions=Splitter.SplitUsingLineStrip(surface,curves,
            throwOnInvalidTriangleIndex:true,validateTrimCurves:true);
        Assert.Equal(3,regions.Count);
        Assert.All(regions,region=>Assert.Contains(region.PointsPrecise,p=>p==Point(2,2)));
        var twiceArea=new BigRationalHybrid(0);
        foreach(var region in regions)
            foreach(var t in region.Triangles)
            {
                var a=region.PointsPrecise[t.A];var b=region.PointsPrecise[t.B];var c=region.PointsPrecise[t.C];
                var area=(b.X-a.X)*(c.Y-a.Y)-(b.Y-a.Y)*(c.X-a.X);
                Assert.True(area>new BigRationalHybrid(0));twiceArea+=area;
            }
        Assert.True(twiceArea==new BigRationalHybrid(32));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CrossingTrimCurvesPartitionTheSurfaceWithoutLosingArea(bool tilted, bool overlapping)
    {
        Rat3Hybrid Point(int x, int y) => new(new BigRationalHybrid(x), new BigRationalHybrid(y),
            tilted ? new BigRationalHybrid(x, 3) + new BigRationalHybrid(y, 7) : new BigRationalHybrid(0));
        var surface = new Surface([new Tri(0,1,2)], [Point(0,0),Point(8,0),Point(0,8)]);
        List<List<LineSegmentOnTriangle>> curves = [
            [new(Point(2,0),Point(2,6),0)],
            [new(Point(0,2),Point(6,2),0)]
        ];
        if (overlapping)
        {
            curves.Add([new(Point(2,4),Point(2,1),0)]);
            curves.Add([new(Point(2,3),Point(2,3),0)]);
        }
        var regions = Splitter.SplitUsingLineStrip(surface, curves,
            throwOnInvalidTriangleIndex:true, validateTrimCurves:true);
        Assert.Equal(4,regions.Count);
        var twiceArea = new BigRationalHybrid(0);
        foreach (var region in regions)
        {
            Assert.Contains(region.PointsPrecise,p=>p==Point(2,2));
            foreach (var triangle in region.Triangles)
            {
                var a=region.PointsPrecise[triangle.A];
                var b=region.PointsPrecise[triangle.B];
                var c=region.PointsPrecise[triangle.C];
                var signedArea=(b.X-a.X)*(c.Y-a.Y)-(b.Y-a.Y)*(c.X-a.X);
                Assert.True(signedArea>new BigRationalHybrid(0));
                twiceArea+=signedArea;
            }
        }
        Assert.True(twiceArea==new BigRationalHybrid(64));
        Assert.Single(surface.Triangles);
        Assert.Equal(3,surface.PointsPrecise.Count);
    }
}
