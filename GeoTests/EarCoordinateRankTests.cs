using System.Numerics;
using GeoCore;

namespace GeoTests;

public class EarCoordinateRankTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ExactRanksPreserveNotchesAndEqualCoordinatesBelowDoubleResolution(bool tinyY, bool append)
    {
        var tiny = new BigRationalHybrid(BigInteger.One, BigInteger.Pow(10, 100));
        var third = new BigRationalHybrid(1, 3);
        var yScale = tinyY ? tiny : BigRationalHybrid.One;
        var corners = new (int X, int Y)[] { (0, 0), (3, 0), (3, 3), (2, 3), (2, 1), (1, 1), (1, 3), (0, 3) };
        var points = corners.Select(c => new Rat2Hybrid(third + new BigRationalHybrid(c.X) * tiny,
            new BigRationalHybrid(c.Y) * yScale)).ToList();
        for (int i = 0; i < points.Count; i++)
        {
            var point = points[i];
            point.Simplify();
            points[i] = point;
        }
        var triangulator = new TriangulationEarClipping<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>();
        for (int pass = 0; pass < 2; pass++)
        {
            // Reusing the triangulator must rebuild ranks for the new ordering.
            if (pass == 1)
                points = points.Skip(3).Concat(points.Take(3)).ToList();
            triangulator.Initialize(points);
            // The public point list can be edited between initialization and execution.
            var rotated = points.Skip(1).Concat(points.Take(1)).ToArray();
            for (int i = 0; i < points.Count; i++)
                points[i] = rotated[i];
            List<Tri> triangles;
            if (append)
            {
                var context = new TriangulationContext<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>();
                context.BeginSession(points);
                triangulator.TriangulateAppend(context);
                var output = context.GetType().GetProperty("Triangles",
                    System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                triangles = (List<Tri>)output!.GetValue(context)!;
            }
            else
            {
                triangles = new List<Tri>();
                triangulator.Triangulate(triangles);
            }
            Assert.Equal(points.Count - 2, triangles.Count);
            var area = BigRationalHybrid.Zero;
            foreach (var triangle in triangles)
            {
                var a = points[triangle.A];
                var b = points[triangle.B];
                var c = points[triangle.C];
                var twice = (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
                Assert.True(twice.Sign() > 0);
                area += twice;
                area.Simplify();
                var center = new Rat2Hybrid((a.X + b.X + c.X) / new BigRationalHybrid(3),
                    (a.Y + b.Y + c.Y) / new BigRationalHybrid(3));
                Assert.Equal(PointInPolygonResult.Inside,
                    PolygonOps<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>.IsPointInPolygon(
                        points, Enumerable.Range(0, points.Count).ToList(), center));
            }
            Assert.Equal(0, (area - new BigRationalHybrid(14) * tiny * yScale).Sign());
        }
    }
}
