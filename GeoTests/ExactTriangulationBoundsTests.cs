using GeoCore;
using System.Numerics;

namespace GeoTests;

public class ExactTriangulationBoundsTests
{
    [Fact]
    public void InteriorPointInRationalSliverIsNotRejectedByFloatingBounds()
    {
        var third = new BigRationalHybrid(1,3);
        var tiny = new BigRationalHybrid(1,BigInteger.Pow(10,400));
        var low = third-tiny;
        var high = third+tiny;
        var points = new List<Rat2Hybrid> {
            new(low,new BigRationalHybrid(0)),new(high,new BigRationalHybrid(0)),
            new(high,new BigRationalHybrid(3)),new(low,new BigRationalHybrid(3)),
            new(third,new BigRationalHybrid(1))
        };
        // Kernel point stores supply canonical rational coordinates.
        for(int i=0;i<points.Count;i++) { var point=points[i];point.Simplify();points[i]=point; }
        var triangles = Triangulator.TriangulatePolygon(points,
            new List<int>{0,1,2,3},new List<Int2>(),new List<int>{4},false);
        Assert.Contains(triangles,t => t.A==4 || t.B==4 || t.C==4);
        BigRationalHybrid area = new(0);
        foreach(var t in triangles)
        {
            var a=points[t.A];var b=points[t.B];var c=points[t.C];
            var twice=(b.X-a.X)*(c.Y-a.Y)-(b.Y-a.Y)*(c.X-a.X);
            Assert.True(twice.Sign()>0);
            area+=twice;
        }
        Assert.Equal(0,area.CompareTo((high-low)*new BigRationalHybrid(6)));
    }

    [Theory]
    [InlineData(1,3)]
    [InlineData(-1,3)]
    [InlineData(7,10)]
    [InlineData(1,8)]
    public void FloatingBoundsEncloseExactCoordinates(long numerator,long denominator)
    {
        var value = new BigRationalHybrid(numerator,denominator);
        var point = new Rat2Hybrid(value,-value);
        var bounds = new Rat2HybridArithmetic().GetBounds(point);
        var exact = new BigRational(numerator,denominator);
        Assert.True(new BigRational(bounds.Min.X) <= exact);
        Assert.True(new BigRational(bounds.Max.X) >= exact);
        Assert.True(new BigRational(bounds.Min.Y) <= -exact);
        Assert.True(new BigRational(bounds.Max.Y) >= -exact);
    }
}
