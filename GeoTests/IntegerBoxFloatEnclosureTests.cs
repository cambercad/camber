using GeoCore;

namespace GeoTests;

public class IntegerBoxFloatEnclosureTests
{
    private static void Check(Box3I exact)
    {
        var bounds = exact.GetBoundsF();
        // BigRational converts the represented binary float exactly; comparing
        // float directly with int would round the integer and hide this bug.
        Assert.True(new BigRational(bounds.Min.X) <= new BigRational(exact.Min.X));
        Assert.True(new BigRational(bounds.Min.Y) <= new BigRational(exact.Min.Y));
        Assert.True(new BigRational(bounds.Min.Z) <= new BigRational(exact.Min.Z));
        Assert.True(new BigRational(bounds.Max.X) >= new BigRational(exact.Max.X));
        Assert.True(new BigRational(bounds.Max.Y) >= new BigRational(exact.Max.Y));
        Assert.True(new BigRational(bounds.Max.Z) >= new BigRational(exact.Max.Z));
    }

    [Theory]
    [InlineData(int.MinValue)]
    [InlineData(int.MinValue+1)]
    [InlineData(-16777217)]
    [InlineData(0)]
    [InlineData(16777217)]
    [InlineData(int.MaxValue-1)]
    [InlineData(int.MaxValue)]
    public void PointBoxesEncloseIntegersAtFloatPrecisionBoundaries(int value)
        => Check(new Box3I(new Int3(value),new Int3(value)));

    [Fact]
    public void FullInt32SpansDoNotOverflowSizeCalculation()
        => Check(new Box3I(new Int3(int.MinValue),new Int3(int.MaxValue)));

    [Fact]
    public void NarrowBoxesAcrossFullInt32DomainRemainConservative()
    {
        var rng = new Random(83391);
        for(int i=0;i<2000;i++)
        {
            int x=(int)rng.NextInt64(int.MinValue,(long)int.MaxValue-1024);
            int width=rng.Next(1024);
            Check(new Box3I(new Int3(x),new Int3(x+width)));
        }
    }
}
