using System.Numerics;
using GeoCore;

namespace GeoTests;

public class PointIntegerBoundsTests
{
    [Theory]
    [InlineData(int.MinValue)]
    [InlineData(int.MaxValue)]
    public void ExactIntegerEndpointsDoNotWrap(int value)
    {
        foreach (var coordinate in new[] {new BigRationalHybrid(value),new BigRationalHybrid(value,1)})
        {
            var box = new Rat3Hybrid(coordinate,coordinate,coordinate).GetBox();
            Assert.True(box.Min.X <= value && box.Max.X >= value);
            Assert.True(box.Min.Y <= value && box.Max.Y >= value);
            Assert.True(box.Min.Z <= value && box.Max.Z >= value);
        }
    }

    [Theory]
    [InlineData(int.MinValue, -1)]
    [InlineData(int.MaxValue, 1)]
    public void UnrepresentableCoordinatesFailInsteadOfWrapping(int boundary, int direction)
    {
        BigInteger denominator = (BigInteger.One<<1000)+7;
        var coordinate = new BigRationalHybrid(boundary*denominator+direction,denominator);
        Assert.Throws<OverflowException>(() => new Rat3Hybrid(coordinate,coordinate,coordinate).GetBox());
        coordinate = new BigRationalHybrid((long)boundary+direction);
        Assert.Throws<OverflowException>(() => new Rat3Hybrid(coordinate,coordinate,coordinate).GetBox());
    }

    [Theory]
    [InlineData(int.MinValue, 1)]
    [InlineData(int.MaxValue, -1)]
    public void FractionsImmediatelyInsideIntegerDomainRemainEnclosed(int boundary, int direction)
    {
        BigInteger denominator = (BigInteger.One << 1600) + 7;
        var coordinate = new BigRationalHybrid(boundary * denominator + direction, denominator);
        var box = new Rat3Hybrid(coordinate, coordinate, coordinate).GetBox();
        Assert.True(new BigRationalHybrid(box.Min.X) <= coordinate);
        Assert.True(new BigRationalHybrid(box.Max.X) >= coordinate);
        Assert.Equal(direction > 0 ? boundary : boundary - 1, box.Min.X);
        Assert.Equal(direction > 0 ? boundary + 1 : boundary, box.Max.X);
    }

    [Fact]
    public void ArbitraryLengthOutOfDomainValuesFailExplicitly()
    {
        foreach (int sign in new[] { -1, 1 })
        {
            var coordinate = new BigRationalHybrid(sign * (BigInteger.One << 4096), 7);
            Assert.Throws<OverflowException>(() => new Rat3Hybrid(coordinate, coordinate, coordinate).GetBox());
        }
    }

    [Theory]
    [InlineData(-4,1,-5,-4)]
    [InlineData(-7,2,-4,-3)]
    [InlineData(-1,3,-1,0)]
    [InlineData(0,1,0,1)]
    [InlineData(1,3,0,1)]
    [InlineData(7,2,3,4)]
    [InlineData(4,1,4,5)]
    public void ExistingConservativePaddingIsPreserved(int numerator,int denominator,int lower,int upper)
    {
        var coordinate=new BigRationalHybrid(numerator,denominator);
        var box=new Rat3Hybrid(coordinate,coordinate,coordinate).GetBox();
        Assert.Equal(new Int3(lower),box.Min);
        Assert.Equal(new Int3(upper),box.Max);
    }
}
