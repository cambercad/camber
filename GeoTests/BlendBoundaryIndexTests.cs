using System.Numerics;
using Geo;
using GeoCore;

namespace GeoTests;

public class BlendBoundaryIndexTests
{
    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    public void CoverageAcrossTriangleSeamsWorksInEveryProjectionAndWinding(int axis, bool reverse)
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        Rat3Hybrid Point(int x, int y) => axis switch
        {
            0 => new(0, x, y),
            1 => new(y, 0, x),
            2 => new(x, y, 0),
            _ => new(x, y, x + y)
        };
        var points = new List<Rat3Hybrid> { Point(0, 0), Point(12, 0), Point(12, 12), Point(0, 12) };
        var volume = new MeshNormalUV
        {
            PrecisionPositions = points,
            Positions = converter.Convert(points),
            Triangles = reverse ? [new(0, 2, 1), new(0, 3, 2)] : [new(0, 1, 2), new(0, 2, 3)]
        };
        var patchPoints = new List<Rat3Hybrid> { Point(1, 3), Point(11, 3), Point(6, 11) };
        var patch = new UVSurface(converter.Convert(patchPoints),
            Enumerable.Repeat(new Vec3D(0, 0, 1), 3).ToList(),
            [new(0, 0), new(1, 0), new(0, 1)], [new(0, 1, 2)], patchPoints);
        var names = new[] { ("patch", (SurfaceMetaData)null) };
        EdgeBlendPipeline.ValidateBlendPatchBoundaries([patch], names, volume, converter);
        volume.Triangles.RemoveAt(1);
        var error = Assert.Throws<InvalidOperationException>(() =>
            EdgeBlendPipeline.ValidateBlendPatchBoundaries([patch], names, volume, converter));
        Assert.Contains("interior open boundary", error.Message);
    }

    [Theory]
    [InlineData(2000000000)]
    [InlineData(-2000000000)]
    public void SpatialCandidatesRetainFractionalCoverageAndRejectSubFloatGaps(int origin)
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        var fraction = new BigRationalHybrid(BigInteger.One, BigInteger.One << 80);
        var start = new BigRationalHybrid(origin) + fraction;
        var precise = new List<Rat3Hybrid>
        {
            new(start, start, fraction), new(start + new BigRationalHybrid(256), start, fraction),
            new(start, start + new BigRationalHybrid(256), fraction)
        };
        var volume = new MeshNormalUV
        {
            PrecisionPositions = precise,
            Positions = converter.Convert(precise),
            Triangles = [new(0, 1, 2)]
        };
        UVSurface Patch(BigRationalHybrid height)
        {
            var points = new List<Rat3Hybrid>
            {
                new(start, start, height), new(start + new BigRationalHybrid(128), start, height),
                new(start, start + new BigRationalHybrid(128), height)
            };
            return new UVSurface(converter.Convert(points),
                Enumerable.Repeat(new Vec3D(0, 0, 1), 3).ToList(),
                [new(0, 0), new(1, 0), new(0, 1)], [new(0, 1, 2)], points);
        }
        var names = new[] { ("patch", (SurfaceMetaData)null) };
        EdgeBlendPipeline.ValidateBlendPatchBoundaries([Patch(fraction)], names, volume, converter);
        var error = Assert.Throws<InvalidOperationException>(() =>
            EdgeBlendPipeline.ValidateBlendPatchBoundaries([Patch(fraction * new BigRationalHybrid(2))], names, volume, converter));
        Assert.Contains("interior open boundary", error.Message);
    }

    [Fact]
    public void NoPatchesNeedsNoSpatialTree()
    {
        EdgeBlendPipeline.ValidateBlendPatchBoundaries([], [], new MeshNormalUV(), default);
    }

    [Fact]
    public void OpenPatchAgainstEmptyVolumeReportsMissingCoverage()
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        var points = new List<Rat3Hybrid> { new(0, 0, 0), new(1, 0, 0), new(0, 1, 0) };
        var patch = new UVSurface(converter.Convert(points),
            Enumerable.Repeat(new Vec3D(0, 0, 1), 3).ToList(),
            [new(0, 0), new(1, 0), new(0, 1)], [new(0, 1, 2)], points);
        var volume = new MeshNormalUV { PrecisionPositions = [], Triangles = [] };
        var error = Assert.Throws<InvalidOperationException>(() =>
            EdgeBlendPipeline.ValidateBlendPatchBoundaries([patch],
                [("patch", (SurfaceMetaData)null)], volume, converter));
        Assert.Contains("interior open boundary", error.Message);
    }
}
