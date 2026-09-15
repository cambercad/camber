namespace GeoTests;

public sealed class ChamferAnalyticTests
{
    [Theory]
    [InlineData(1.0, 0.25, 0.96875)]          // 1 - d²/2
    [InlineData(1.0, 0.2, 0.98)]              // 1 - 0.02
    [InlineData(3.0, 0.2, 2.94)]              // 3 - 3·d²/2
    public void SingleEdge_RemovedVolume(double boxVolume, double d, double expectedVolume)
    {
        double edgeLength = boxVolume; // cuboid tests use edge length equal to box extent along that edge
        double actual = ChamferAnalytic.BoxVolumeAfterSingleEdgeChamfer(boxVolume, edgeLength, d);
        Assert.Equal(expectedVolume, actual, precision: 12);
    }

    [Theory]
    [InlineData(1.0, 0.25, 0.9166666666666666)] // 1 - 3d²/2 + 2d³/3
    [InlineData(1.0, 0.2, 0.9453333333333333)]
    public void CubeThreeEdgeCorner_ExactVolume(double side, double d, double expectedVolume)
    {
        double actual = ChamferAnalytic.CubeVolumeAfterThreeEdgeCornerChamfer(side, d);
        Assert.Equal(expectedVolume, actual, precision: 12);
    }

    [Theory]
    [InlineData(1.0, 3.0, 0.2, 2.9053333333333333)]
    [InlineData(1.0, 3.0, 0.25, 2.8541666666666665)]
    public void Cuboid1x1xL_ThreeEdgeCorner_ExactVolume(
        double shortSide, double longSide, double d, double expectedVolume)
    {
        double boxVolume = shortSide * shortSide * longSide;
        double actual = ChamferAnalytic.CuboidCornerVolumeAfterThreeEdgeChamfer(
            shortSide, longSide, d, boxVolume);
        Assert.Equal(expectedVolume, actual, precision: 12);
    }
}
