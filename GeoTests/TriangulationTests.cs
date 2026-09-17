using CSG;
using GeoCore;

namespace GeoTests;

public class TriangulationTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void InteriorPointCollinearWithDistantBoundaryEdgeIsNotOnBoundary(bool reverse)
    {
        var points = new List<Rat2Hybrid> {new(0,0),new(6,0),new(6,6),new(3,3),new(0,6)};
        if (reverse) points.Reverse();
        var indices = Enumerable.Range(0,points.Count).ToList();
        Assert.Equal(PointInPolygonResult.Inside,
            PolygonOps<Rat2HybridArithmetic,Rat2Hybrid,BigRationalHybrid>.IsPointInPolygon(points,indices,new Rat2Hybrid(1,1)));
        Assert.Equal(PointInPolygonResult.Outside,
            PolygonOps<Rat2HybridArithmetic,Rat2Hybrid,BigRationalHybrid>.IsPointInPolygon(points,indices,new Rat2Hybrid(3,5)));
        Assert.Equal(PointInPolygonResult.OnPolyBorder,
            PolygonOps<Rat2HybridArithmetic,Rat2Hybrid,BigRationalHybrid>.IsPointInPolygon(points,indices,new Rat2Hybrid(4,4)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HoleOccludedFromOriginalBorderCanBridgeToAnAlreadyJoinedHole(bool reverse)
    {
        static List<Rat2Hybrid> Rectangle(int x, int y, int width, int height) => new() {
            new(x,y),new(x+width,y),new(x+width,y+height),new(x,y+height) };
        var loops = new List<List<Rat2Hybrid>> {
            Rectangle(0,0,20,20), Rectangle(8,8,4,4),
            Rectangle(14,4,4,4), Rectangle(14,14,4,4),
            Rectangle(2,14,4,4), Rectangle(2,2,4,4) };
        var holes = loops.Skip(1).ToArray();
        if (reverse)
        {
            loops.Reverse();
            foreach (var loop in loops) loop.Reverse();
        }
        var points = loops.SelectMany(loop => loop).ToList();
        var triangles = Triangulator.TriangulatePolygon(loops);
        AssertAllTriangleIndicesInRange(triangles, points.Count);
        Assert.True(TotalTriangleAreaTimesTwo(triangles, points) == new BigRationalHybrid(640));
        foreach (var triangle in triangles)
        {
            var a=points[triangle.A]; var b=points[triangle.B]; var c=points[triangle.C];
            Assert.True(SignedArea2DTimesTwo(a,b,c).Sign()>0);
            var center = new Rat2Hybrid((a.X+b.X+c.X)/new BigRationalHybrid(3),
                (a.Y+b.Y+c.Y)/new BigRationalHybrid(3));
            foreach (var hole in holes)
                Assert.NotEqual(PointInPolygonResult.Inside,
                    PolygonOps<Rat2HybridArithmetic,Rat2Hybrid,BigRationalHybrid>.IsPointInPolygon(
                        hole,Enumerable.Range(0,hole.Count).ToList(),center));
        }
    }

    [Fact]
    public void TriangulatePolygon_ConvexSquare_ReturnsTwoTriangles()
    {
        var points = new List<Rat2Hybrid>
        {
            new(0, 0),
            new(1, 0),
            new(1, 1),
            new(0, 1)
        };

        var triangles = Triangulator.TriangulatePolygon(points);

        Assert.Equal(2, triangles.Count);
        AssertAllTriangleIndicesInRange(triangles, points.Count);
    }

    [Fact]
    public void TriangulatePolygon_ConcavePentagon_ReturnsNMinusTwoTriangles()
    {
        var points = new List<Rat2Hybrid>
        {
            new(0, 0),
            new(2, 0),
            new(2, 2),
            new(1, 1), // concave notch
            new(0, 2)
        };

        var triangles = Triangulator.TriangulatePolygon(points);

        Assert.Equal(points.Count - 2, triangles.Count);
        AssertAllTriangleIndicesInRange(triangles, points.Count);
    }

    [Fact]
    public void TriangulatePolygon_WithHole_ReturnsNonEmptyTriangles()
    {
        var outer = new List<Rat2Hybrid>
        {
            new(0, 0),
            new(10, 0),
            new(10, 10),
            new(0, 10)
        };

        var hole = new List<Rat2Hybrid>
        {
            new(3, 3),
            new(3, 7),
            new(7, 7),
            new(7, 3)
        };

        var triangles = Triangulator.TriangulatePolygon(new List<List<Rat2Hybrid>> { outer, hole });
        var allPoints = new List<Rat2Hybrid>();
        allPoints.AddRange(outer);
        allPoints.AddRange(hole);

        Assert.NotEmpty(triangles);
        AssertAllTriangleIndicesInRange(triangles, allPoints.Count);
        Assert.True(BigRationalHybrid.Sign(TotalTriangleAreaTimesTwo(triangles, allPoints)) != 0);
    }

    [Fact]
    public void TriangulatePolygon_WithTwoHoles_ReturnsNonEmptyTriangles()
    {
        var outer = new List<Rat2Hybrid>
        {
            new(0, 0),
            new(12, 0),
            new(12, 16),
            new(0, 16)
        };

        var topHole = new List<Rat2Hybrid>
        {
            new(4, 9),
            new(4, 14),
            new(10, 14),
            new(10, 9)
        };

        var bottomHole = new List<Rat2Hybrid>
        {
            new(4, 2),
            new(4, 7),
            new(10, 7),
            new(10, 2)
        };

        var triangles = Triangulator.TriangulatePolygon(new List<List<Rat2Hybrid>> { outer, topHole, bottomHole });
        var allPoints = new List<Rat2Hybrid>();
        allPoints.AddRange(outer);
        allPoints.AddRange(topHole);
        allPoints.AddRange(bottomHole);

        Assert.NotEmpty(triangles);
        AssertAllTriangleIndicesInRange(triangles, allPoints.Count);
        Assert.True(BigRationalHybrid.Sign(TotalTriangleAreaTimesTwo(triangles, allPoints)) != 0);
    }

    [Fact]
    public void TriangulatePolygon_WithConstraint_ReturnsTriangles()
    {
        var points = new List<Rat2Hybrid>
        {
            new(0, 0),
            new(2, 0),
            new(2, 2),
            new(0, 2),
            new(1, 1)
        };
        var border = new List<int> { 0, 1, 2, 3 };
        var constraints = new List<Int2> { new(0, 4), new(2, 4) };

        var triangles = Triangulator.TriangulatePolygon(points, border, constraints);

        Assert.NotEmpty(triangles);
        AssertAllTriangleIndicesInRange(triangles, points.Count);
    }

    private static void AssertAllTriangleIndicesInRange(IEnumerable<Tri> triangles, int count)
    {
        foreach (var tri in triangles)
        {
            Assert.InRange(tri.A, 0, count - 1);
            Assert.InRange(tri.B, 0, count - 1);
            Assert.InRange(tri.C, 0, count - 1);
            Assert.True(tri.A != tri.B && tri.B != tri.C && tri.A != tri.C);
        }
    }

    private static BigRationalHybrid TotalTriangleAreaTimesTwo(IEnumerable<Tri> triangles, IList<Rat2Hybrid> points)
    {
        BigRationalHybrid area2 = new BigRationalHybrid(0);
        foreach (var tri in triangles)
        {
            var a = points[tri.A];
            var b = points[tri.B];
            var c = points[tri.C];
            area2 += BigRationalHybrid.Abs(SignedArea2DTimesTwo(a, b, c));
        }

        return area2;
    }

    private static BigRationalHybrid SignedArea2DTimesTwo(in Rat2Hybrid pa, in Rat2Hybrid pb, in Rat2Hybrid pc)
    {
        // (pb - pa) x (pc - pa)
        var bax = pb.X - pa.X;
        var bay = pb.Y - pa.Y;
        var cax = pc.X - pa.X;
        var cay = pc.Y - pa.Y;
        return bax * cay - bay * cax;
    }
}
