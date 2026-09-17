using GeoCore;
using NURBS;

namespace GeoTests;

public class NurbsSubdomainDeviationTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RectangleBoundCoversInteriorTrianglesAndDisplacedVertices(bool rational)
    {
        Vec3D[][] points =
        [
            [new(0, 0, 0), new(0, 1, 0)],
            [new(1, 0, 2), new(1, 1, -1)],
            [new(2, 0, 0), new(2, 1, 1)]
        ];
        double[][] weights = rational ? [[1, 1], [.7, .8], [1, 1]] : [[1, 1], [1, 1], [1, 1]];
        var surface = new BSplineSurface(2, 1, points, weights,
            [0, 0, 0, 1, 1, 1], [0, 0, 1, 1]);
        var original = surface.ControlPoints.Select(row => row.ToArray()).ToArray();
        double bound = AdaptiveSurfaceSplitter.TriangleDeviationBound(surface, .17, .23, .61, .79);
        Assert.True(double.IsFinite(bound) && bound > 0);
        Assert.True(bound < AdaptiveSurfaceSplitter.TriangleDeviationBound(surface, 0, 0, 1, 1));
        Vec2D[] uv = [new(.2, .28), new(.59, .31), new(.4, .76)];
        Vec3D[] displacement = [new(0, 0, .013), new(.007, 0, 0), new(0, -.009, 0)];
        var vertices = uv.Select((point, i) => surface.Evaluate(point.X, point.Y) + displacement[i]).ToArray();
        double vertexBudget = displacement.Max(point => point.Length());
        for (int i = 0; i <= 12; i++)
        for (int j = 0; j <= 12 - i; j++)
        {
            double a = i / 12.0, b = j / 12.0, c = 1 - a - b;
            var parameter = uv[0] * a + uv[1] * b + uv[2] * c;
            var actual = vertices[0] * a + vertices[1] * b + vertices[2] * c;
            double error = (surface.Evaluate(parameter.X, parameter.Y) - actual).Length();
            Assert.InRange(error, 0, bound + vertexBudget);
        }
        for (int row = 0; row < original.Length; row++)
            Assert.Equal(original[row], surface.ControlPoints[row]);
    }

    [Theory]
    [InlineData(1e-16, .9)]
    [InlineData(.5, .9999999999999999)]
    [InlineData(.49999999999999994, .9)]
    [InlineData(.5000000000000001, .9)]
    public void DistinctParametersNearKnotsRemainDistinct(double minU, double maxU)
    {
        var surface = new BSplineSurface(1, 1,
            new Vec3D[][] { [new(0, 0, 0), new(0, 1, 0)],
                [new(.5, 0, 0), new(.5, 1, 0)], [new(1, 0, 0), new(1, 1, 0)] },
            [0, 0, .5, 1, 1], [0, 0, 1, 1]);
        var bound = AdaptiveSurfaceSplitter.TriangleDeviationBound(surface, minU, .2, maxU, .8);
        Assert.InRange(bound, 0, 1e-14);
    }

    [Fact]
    public void EmptyOrOutOfDomainRectanglesRejectWithoutChangingTheSurface()
    {
        var surface = new BSplineSurface(1, 1,
            new Vec3D[][] { [new(0, 0, 0), new(0, 1, 0)], [new(1, 0, 0), new(1, 1, 0)] },
            [0, 0, 1, 1], [0, 0, 1, 1]);
        Assert.Throws<ArgumentException>(() => AdaptiveSurfaceSplitter.TriangleDeviationBound(surface, .2, 0, .2, 1));
        Assert.Throws<ArgumentException>(() => AdaptiveSurfaceSplitter.TriangleDeviationBound(surface, -.1, 0, 1, 1));
        Assert.Throws<ArgumentException>(() => AdaptiveSurfaceSplitter.TriangleDeviationBound(surface, 0, 0, double.NaN, 1));
        Assert.Equal(0, AdaptiveSurfaceSplitter.TriangleDeviationBound(surface, .1, .2, .8, .9), 12);
    }
}
