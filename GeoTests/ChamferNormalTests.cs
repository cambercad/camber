using Geo;
using GeoCore;

namespace GeoTests;

public class ChamferNormalTests
{
    [Fact]
    public void SubDoubleRailSpacingStillHasAWellDefinedNormal()
    {
        var cc = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        var origin = new Rat3Hybrid(500000, 500000, 500000);
        var tiny = new BigRationalHybrid(System.Numerics.BigInteger.One, System.Numerics.BigInteger.One << 60);
        var a = new List<Rat3Hybrid> { origin, origin + new Rat3Hybrid(tiny, BigRationalHybrid.Zero, BigRationalHybrid.Zero), origin + new Rat3Hybrid(100, 0, 0) };
        var b = a.Select(point => point + new Rat3Hybrid(0, 100, 0)).ToList();
        var center = a.Select(point => point + new Rat3Hybrid(0, 50, 0)).ToList();
        var aWorld = cc.Convert(a);
        Assert.Equal(aWorld[0], aWorld[1]);
        Assert.True(a[0] != a[1]);
        var graph = new GraphEdge("edge", 0, 1, new List<Int2>(), new LineStrip3D(cc.Convert(center)), center);
        var edge = new BlendEdge(graph, 0, 1, .1)
        {
            BlendType = EdgeBlendType.Convex,
            CenterCurve = center, CenterCurveVec3 = cc.Convert(center),
            ProjectedBoundaryCurveA = a, ProjectedBoundaryCurveAVec3D = aWorld,
            ProjectedBoundaryCurveB = b, ProjectedBoundaryCurveBVec3D = cc.Convert(b),
        };
        edge.BuildChamferStripSurface(cc, .01);
        Assert.All(edge.RawSurface.Normals, normal => Assert.Equal(new Vec3D(0, 0, -1), normal));
        Assert.Equal(a[1], edge.RawSurface.PointsPrecise[2]);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(0.4)]
    public void ClosedRuledChamferNormalsFollowEachRailAndAgreeAtSeam(double twist)
    {
        var cc = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        List<Rat3Hybrid> Exact(List<Vec3D> points) => cc.Convert(points)
            .Select(p => new Rat3Hybrid(p.X, p.Y, p.Z)).ToList();
        const int count = 24;
        var a = new List<Vec3D>();
        var b = new List<Vec3D>();
        for (int i = 0; i < count; i++)
        {
            double angle = 2 * Math.PI * i / count;
            a.Add(new Vec3D(Math.Cos(angle), Math.Sin(angle), 0));
            b.Add(new Vec3D(2 * Math.Cos(angle + twist), 2 * Math.Sin(angle + twist), 1));
        }
        a.Add(a[0]); b.Add(b[0]);
        var center = a.Zip(b, (x, y) => (x + y) / 2).ToList();
        var graph = new GraphEdge("edge", 0, 1, new List<Int2>(), new LineStrip3D(center), Exact(center));
        var edge = new BlendEdge(graph, 0, 1, .1) {
            BlendType = EdgeBlendType.Convex,
            CenterCurve = Exact(center), CenterCurveVec3 = center,
            ProjectedBoundaryCurveA = Exact(a), ProjectedBoundaryCurveAVec3D = a,
            ProjectedBoundaryCurveB = Exact(b), ProjectedBoundaryCurveBVec3D = b,
        };
        edge.BuildChamferStripSurface(cc, .01);
        for (int i = 0; i <= count; i++)
        {
            double angle = 2 * Math.PI * i / count;
            var across = b[i] - a[i];
            var tangentA = new Vec3D(-Math.Sin(angle), Math.Cos(angle), 0);
            var tangentB = new Vec3D(-Math.Sin(angle + twist), Math.Cos(angle + twist), 0);
            Assert.True(Vec3DOps.Dot(Vec3DOps.Cross(across, tangentA).Normalized(), edge.RawSurface.Normals[2*i]) > .999999);
            Assert.True(Vec3DOps.Dot(Vec3DOps.Cross(across, tangentB).Normalized(), edge.RawSurface.Normals[2*i+1]) > .999999);
        }
        Assert.Equal(edge.RawSurface.Normals[0], edge.RawSurface.Normals[^2]);
        Assert.Equal(edge.RawSurface.Normals[1], edge.RawSurface.Normals[^1]);
    }
}
