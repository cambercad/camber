using GeoCore;

public class CoordinateConverterTests
{
    [Fact]
    public void NonCubicOperatingSpaceUsesOneIsotropicLatticeScale()
    {
        var box = new Box3D(new Vec3D(-10, -5, -2.5), new Vec3D(10, 5, 2.5), 0);
        var converter = new CoordinateConverter(box, operatingSpaceSlices: 101);

        Assert.Equal(0.2, converter.SmallestUnit(), 12);

        var integerBox = converter.IntegerBoundingBox;
        Assert.Equal(100, integerBox.Max.X);
        Assert.Equal(50, integerBox.Max.Y);
        Assert.Equal(25, integerBox.Max.Z);

        // Equal world-space displacements on each axis map to equal lattice steps.
        var origin = converter.Convert(box.Min);
        Assert.Equal(10, converter.Convert(new Vec3D(-8, -5, -2.5)).X - origin.X);
        Assert.Equal(10, converter.Convert(new Vec3D(-10, -3, -2.5)).Y - origin.Y);
        Assert.Equal(10, converter.Convert(new Vec3D(-10, -5, -0.5)).Z - origin.Z);

        var latticePoint = new Int3(37, 19, 8);
        var roundTrip = converter.Convert(converter.Convert(latticePoint));
        Assert.Equal(latticePoint.X, roundTrip.X);
        Assert.Equal(latticePoint.Y, roundTrip.Y);
        Assert.Equal(latticePoint.Z, roundTrip.Z);
    }
}
