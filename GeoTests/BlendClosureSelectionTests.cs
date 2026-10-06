using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class BlendClosureSelectionTests
{
    private static BlendEdge Edge() => new(
        new GraphEdge("test", 0, 1, [], null!, []), 0, 1, 1);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CutCanBeSubdividedAtItsMidpoint(bool transformed)
    {
        Rat3Hybrid Point(int x, int y) => transformed
            ? new Rat3Hybrid(1000000 + x + y, 2 * x - y, 3 * x + 2 * y)
            : new Rat3Hybrid(x, y, 0);
        var points = new List<Rat3Hybrid> { Point(0, -1), Point(0, 0), Point(0, 1), Point(-1, 0), Point(1, 0) };
        var kept = new Surface([new(0, 1, 3), new(1, 2, 3)], points, 7);
        var removed = new Surface([new(1, 0, 4), new(2, 1, 4)], points, 7);
        var cut = new List<LineSegmentOnTriangle> { new(points[0], points[2], 7) };
        var normal = transformed ? new Rat3Hybrid(1, 1, 0) : new Rat3Hybrid(1, 0, 0);
        Assert.Same(kept, Edge().SelectSurfaceAgainstNormalDirection(
            [removed, kept], cut, [normal], [points[0]]));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void InteriorPointOrForeignParentIsNotCutAdjacency(bool foreignParent)
    {
        var points = new List<Rat3Hybrid> { new(0, 0, 0), new(2, 0, 0), new(0, 2, 0) };
        var candidate = new Surface([new(0, 1, 2)], points, foreignParent ? 8 : 7);
        var start = foreignParent ? points[0] : new Rat3Hybrid(1, 1, 0) / new BigRationalHybrid(4);
        var end = foreignParent ? points[1] : new Rat3Hybrid(2, 1, 0) / new BigRationalHybrid(4);
        var error = Assert.Throws<InvalidOperationException>(() =>
            Edge().SelectSurfaceAgainstNormalDirection([candidate], [new(start, end, 7)],
                [new(0, -1, 0)], [start]));
        Assert.Contains("does not bound a region", error.Message);
    }
}
