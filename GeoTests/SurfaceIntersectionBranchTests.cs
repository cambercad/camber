using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class SurfaceIntersectionBranchTests
{
    [Fact]
    public void IntersectingConnectedGraphPatchesCanProduceTwoOpenCurves()
    {
        var xs = new[] { -2, 0, 2 };
        var ys = new[] { -2, 0, 2 };
        var plane = CreateGraph(xs, ys, _ => 0);
        var curved = CreateGraph(xs, ys, x => x * x - 1);
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-2, -2, -1),
            new Vec3D(2, 2, 3)), 1000);
        var strips = Intersector.IntersectSurfaceStrips(plane, curved, converter);

        Assert.Equal(2, strips.Count);
        foreach (var strip in strips)
        {
            Assert.NotEmpty(strip.intersectionA);
            Assert.NotEqual(strip.intersectionA[0].PointStart, strip.intersectionA[^1].PointEnd);
            Assert.Equal(strip.intersectionA.Count, strip.intersectionB.Count);
            for (int i = 1; i < strip.intersectionA.Count; i++)
                Assert.Equal(strip.intersectionA[i - 1].PointEnd, strip.intersectionA[i].PointStart);
        }

        var intendedEdge = new List<Rat3Hybrid>
        {
            new(new BigRationalHybrid(1, 2), new BigRationalHybrid(-2), BigRationalHybrid.Zero),
            new(new BigRationalHybrid(1, 2), BigRationalHybrid.Zero, BigRationalHybrid.Zero),
            new(new BigRationalHybrid(1, 2), new BigRationalHybrid(2), BigRationalHybrid.Zero)
        };
        var sourceEdge = new GraphEdge("x_positive", 0, 1, [],
            new LineStrip3D(converter.Convert(intendedEdge)), intendedEdge);
        var blend = new BlendEdge(sourceEdge, 0, 1, .25);
        Assert.True(blend.ComputeSpineAndBoundaries(
            plane, curved, plane, curved, plane, curved, converter));
        Assert.All(blend.CenterCurve, point => Assert.Equal(new BigRationalHybrid(1, 2), point.X));

        // Both source surfaces are individually connected; the two branches are
        // caused by their geometry, not by disconnected mesh components.
        Assert.Single(MeshConnectivity.FindExactPositionComponents(plane.PointsPrecise, plane.Triangles));
        Assert.Single(MeshConnectivity.FindExactPositionComponents(curved.PointsPrecise, curved.Triangles));
    }

    private static UVSurface CreateGraph(int[] xs, int[] ys, Func<int, int> height)
    {
        var points = new List<Rat3Hybrid>();
        var triangles = new List<Tri>();
        foreach (int y in ys)
            foreach (int x in xs)
                points.Add(new Rat3Hybrid(x, y, height(x)));

        int Index(int x, int y) => y * xs.Length + x;
        for (int y = 0; y < ys.Length - 1; y++)
            for (int x = 0; x < xs.Length - 1; x++)
            {
                int a = Index(x, y);
                int b = Index(x + 1, y);
                int c = Index(x + 1, y + 1);
                int d = Index(x, y + 1);
                triangles.Add(new Tri(a, b, c));
                triangles.Add(new Tri(a, c, d));
            }

        var converter = new CoordinateConverter(new Box3D(new Vec3D(-2, -2, -1),
            new Vec3D(2, 2, 3)), 1000);
        return new UVSurface(converter.Convert(points), null, null, triangles, points);
    }
}
