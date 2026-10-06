using Geo;
using GeoCore;

namespace GeoTests;

public class FilletStripPrecisionTests
{
    [Theory]
    [InlineData(1e-4)]
    [InlineData(1e-12)]
    [InlineData(1e-18)]
    public void ArcSamplingHonorsDeviationWithoutSubtractiveCancellation(double deviation)
    {
        const double angle = Math.PI / 2;
        int segments = BlendEdge.CalculateArcSegments(angle, 1, deviation) - 1;
        double Sagitta(int count) => 2 * Math.Pow(Math.Sin(angle / (4.0 * count)), 2);
        Assert.True(Sagitta(segments) <= deviation);
        Assert.True(Sagitta(segments - 1) > deviation);
    }

    [Fact]
    public void UnrepresentableArcSampleCountDoesNotSilentlyUseFourSamples()
    {
        Assert.Throws<OverflowException>(() => BlendEdge.CalculateArcSegments(Math.PI / 2, 1, 1e-24));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void ExactStripGeometryAndNormalsAreIndependentOfTranslation(bool translated, bool shallow)
    {
        var cc = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        var shift = new BigRationalHybrid(1L << 60);
        var origin = translated ? new Rat3Hybrid(shift, shift, shift) : new Rat3Hybrid(0, 0, 0);
        var centers = new List<Rat3Hybrid> { origin, origin + new Rat3Hybrid(0, 0, 10) };
        var a = new List<Rat3Hybrid> { centers[0] + new Rat3Hybrid(1, 0, 0), centers[1] + new Rat3Hybrid(1, 0, 0) };
        var endRadial = shallow ? new Rat3Hybrid(new BigRationalHybrid(1),
            new BigRationalHybrid(1, 100_000_000), new BigRationalHybrid(0)) : new Rat3Hybrid(0, 1, 0);
        var b = new List<Rat3Hybrid> { centers[0] + endRadial, centers[1] + endRadial };
        var graph = new GraphEdge("edge", 0, 1, [], new LineStrip3D(cc.Convert(centers)), centers);
        var blend = new BlendEdge(graph, 0, 1, cc.SmallestUnit())
        {
            BlendType = EdgeBlendType.Convex,
            CenterCurve = centers, CenterCurveVec3 = cc.Convert(centers),
            ProjectedBoundaryCurveA = a, ProjectedBoundaryCurveAVec3D = cc.Convert(a),
            ProjectedBoundaryCurveB = b, ProjectedBoundaryCurveBVec3D = cc.Convert(b)
        };
        blend.BuildFilletStripSurface(cc, cc.SmallestUnit() / 20);
        var surface = blend.RawSurface;
        int rowLength = surface.PointsPrecise.Count / 2;
        Assert.Equal(a[0], surface.PointsPrecise[0]);
        Assert.Equal(b[0], surface.PointsPrecise[rowLength - 1]);
        for (int i = 0; i < surface.PointsPrecise.Count; ++i)
        {
            Assert.Equal(cc.Convert(surface.PointsPrecise[i]), surface.Points[i]);
            var radial = surface.PointsPrecise[i] - centers[i / rowLength];
            var expectedNormal = new Vec3D(radial.X.ToDouble(), radial.Y.ToDouble(), radial.Z.ToDouble()).Normalized();
            Assert.True((surface.Normals[i] - expectedNormal).Length() < 1e-10);
            Assert.True(double.IsFinite(surface.Uv[i].X));
            Assert.Equal(i < rowLength ? 0 : 1, surface.Uv[i].X);
        }
    }
}
