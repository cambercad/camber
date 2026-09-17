using GeoCore;
using NURBS;

namespace GeoTests;

public class NurbsAffineDirectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DegreeElevationDoesNotSubdivideStraightExtrusionDirection(bool transpose)
    {
        var linear = ExtrudedParabola(1, transpose);
        var elevated = ExtrudedParabola(3, transpose);
        const double deviation = .025;
        var reference = AdaptiveSurfaceSplitter.TriangulateAdaptive(linear, maxDeviation: deviation);
        var mesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(elevated, maxDeviation: deviation);
        Assert.Equal(reference.Triangles.Count, mesh.Triangles.Count);
        Assert.All(mesh.UV, uv => Assert.True((transpose ? uv.X : uv.Y) is 0 or 1));
        NurbsTriangleDeviationTests.AssertInteriorAccuracy(elevated, mesh, deviation);
    }

    [Fact]
    public void CollinearControlsWithNonlinearSpacingStillRespectTriangleTolerance()
    {
        var surface = ExtrudedParabola(3, false, nonlinear: true);
        const double deviation = .025;
        var mesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface, maxDeviation: deviation);
        Assert.Contains(mesh.UV, uv => uv.Y > 0 && uv.Y < 1);
        NurbsTriangleDeviationTests.AssertInteriorAccuracy(surface, mesh, deviation);
    }

    [Fact]
    public void RoundoffInElevatedStraightDirectionDoesNotAddRows()
    {
        var surface = ExtrudedParabola(3, false);
        foreach (var row in surface.ControlPoints)
            row[1] += new Vec4D(0, 0, 1e-12, 0);
        const double deviation = .025;
        var mesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface, maxDeviation: deviation);
        Assert.All(mesh.UV, uv => Assert.True(uv.Y is 0 or 1));
        NurbsTriangleDeviationTests.AssertInteriorAccuracy(surface, mesh, deviation);
    }

    [Fact]
    public void TwistedBilinearPatchRefinesBothDirections()
    {
        var surface = new BSplineSurface(1, 1,
            new Vec3D[][] { [new(0, 0, 0), new(0, 20, 0)],
                [new(20, 0, 0), new(20, 20, 20)] },
            [0, 0, 1, 1], [0, 0, 1, 1]);
        const double deviation = .02;
        var mesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface, maxDeviation: deviation);
        int columns = mesh.UV.Select(uv => uv.X).Distinct().Count();
        int rows = mesh.UV.Select(uv => uv.Y).Distinct().Count();
        Assert.InRange(columns, 3, 33);
        Assert.InRange(rows, 3, 33);
        NurbsTriangleDeviationTests.AssertInteriorAccuracy(surface, mesh, deviation);
    }

    [Theory]
    [InlineData(1.0)]
    [InlineData(1000.0)]
    [InlineData(-3.0)]
    public void RationalArcExtrusionDensityDoesNotGrowWithExtrusionLength(double weightScale)
    {
        BSplineSurface Surface(double length) => new(2, 1,
            new Vec3D[][] { [new(10, 0, 0), new(10, 0, length)],
                [new(10, 10, 0), new(10, 10, length)],
                [new(0, 10, 0), new(0, 10, length)] },
            new double[][] { [weightScale, weightScale],
                [Math.Sqrt(.5) * weightScale, Math.Sqrt(.5) * weightScale],
                [weightScale, weightScale] }, [0, 0, 0, 1, 1, 1], [0, 0, 1, 1]);
        const double deviation = .04;
        var shortSurface = Surface(1);
        var longSurface = Surface(1000);
        var shortMesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(shortSurface, maxDeviation: deviation);
        var longMesh = AdaptiveSurfaceSplitter.TriangulateAdaptive(longSurface, maxDeviation: deviation);
        Assert.Equal(shortMesh.Triangles.Count, longMesh.Triangles.Count);
        Assert.InRange(longMesh.Triangles.Count, 2, 256);
        Assert.All(longMesh.UV, uv => Assert.True(uv.Y is 0 or 1));
        NurbsTriangleDeviationTests.AssertInteriorAccuracy(longSurface, longMesh, deviation);
    }

    private static BSplineSurface ExtrudedParabola(int extrusionDegree, bool transpose, bool nonlinear = false)
    {
        Vec3D[] profile = [new(0, 0, 0), new(5, 10, 0), new(10, 0, 0)];
        var points = profile.Select(p => Enumerable.Range(0, extrusionDegree + 1)
            .Select(i => p + new Vec3D(0, 0, nonlinear && i == 1 ? 1 : 30.0 * i / extrusionDegree))
            .ToArray()).ToArray();
        double[] profileKnots = [0, 0, 0, 1, 1, 1];
        var extrusionKnots = Enumerable.Repeat(0.0, extrusionDegree + 1)
            .Concat(Enumerable.Repeat(1.0, extrusionDegree + 1)).ToArray();
        return transpose
            ? new BSplineSurface(extrusionDegree, 2,
                Enumerable.Range(0, extrusionDegree + 1).Select(v => points.Select(row => row[v]).ToArray()).ToArray(),
                extrusionKnots, profileKnots)
            : new BSplineSurface(2, extrusionDegree, points, profileKnots, extrusionKnots);
    }
}
