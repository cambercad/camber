using Geo.NurbsConstruction;
using GeoCore;

namespace GeoTests;

public class MeshPatchSurfaceFitterTests
{
    [Fact]
    public void ErrorIncludesVerticesOmittedFromFittingSubsample()
    {
        var patch = Grid(40, 40, (u, v) => new Vec3D(u, v, 0), withUv: true);
        // Not the first vertex in its UV bin or on the fitter's regular stride.
        int spike = 20 * 41 + 20;
        patch.Points[spike] = new Vec3D(.5, .5, 1);
        Assert.True(MeshPatchSurfaceFitter.TryFit(patch, .001, out var fit));
        double error = 0;
        for (int i = 0; i < patch.Points.Count; i++)
        {
            var p = patch.Uv[i];
            error = Math.Max(error, (fit.Surface.Evaluate(p.X, p.Y) - patch.Points[i]).Length());
        }
        Assert.InRange(error, .99, 1.01);
        Assert.Equal(error, fit.MaxError, 10);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MeanValueChartUnwrapsNonGraphPatchWithoutMetadata(bool reverseWinding)
    {
        var patch = CylinderStrip();
        if (reverseWinding)
            for (int i = 0; i < patch.Triangles.Count; i++)
            {
                var t = patch.Triangles[i]; patch.Triangles[i] = new Tri(t.A, t.C, t.B);
            }
        var pointsBefore = patch.Points.ToArray();
        var trianglesBefore = patch.Triangles.ToArray();
        Assert.True(MeanValueParameterization.TryCreate(patch, out var uv));
        Assert.Equal(patch.Points.Count, uv.Count);
        Assert.True(MeanValueParameterization.HasConsistentOrientation(patch.Triangles, uv, 1));
        foreach (var p in uv.Values)
        {
            Assert.InRange(p.X, 0, 1); Assert.InRange(p.Y, 0, 1);
        }
        Assert.Equal(pointsBefore, patch.Points);
        Assert.Equal(trianglesBefore, patch.Triangles);
        Assert.Null(patch.NurbsSurface);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FitterUsesMeanValueFallbackForMissingOrFoldedUv(bool foldedUv)
    {
        var patch = CylinderStrip();
        if (foldedUv)
        {
            patch.Uv = new List<Vec2D>(patch.Points.Count);
            foreach (var p in patch.Points) patch.Uv.Add(new Vec2D(p.X, p.Y));
        }
        Assert.True(MeshPatchSurfaceFitter.TryFit(patch, .02, out var fit));
        Assert.True(fit.InventedUv);
        Assert.True(MeanValueParameterization.HasConsistentOrientation(patch.Triangles, fit.VertexUv));
        double error = 0;
        foreach (var (id, uv) in fit.VertexUv)
            error = Math.Max(error, (fit.Surface.Evaluate(uv.X, uv.Y) - patch.Points[id]).Length());
        Assert.Equal(error, fit.MaxError, 10);
        Assert.True(double.IsFinite(fit.MaxError));
    }

    [Fact]
    public void ValidUvIsPreservedAndPlaneFitIsAccurate()
    {
        var patch = Grid(8, 8, (u, v) => new Vec3D(u, v, 2 * u - v), withUv: true);
        Assert.True(MeshPatchSurfaceFitter.TryFit(patch, 1e-6, out var fit));
        Assert.False(fit.InventedUv);
        Assert.InRange(fit.MaxError, 0, 1e-6);
    }

    [Theory]
    [InlineData(.000001)]
    [InlineData(1000000)]
    public void MeanValueChartIsInvariantUnderRigidMotionAndUniformScale(double scale)
    {
        var original = CylinderStrip(); var moved = CylinderStrip();
        for (int i = 0; i < moved.Points.Count; i++)
        {
            var p = moved.Points[i];
            moved.Points[i] = new Vec3D(-p.Z + 3, p.X - 2, p.Y + 1) * scale;
        }
        Assert.True(MeanValueParameterization.TryCreate(original, out var before));
        Assert.True(MeanValueParameterization.TryCreate(moved, out var after));
        foreach (var (id, uv) in before)
        {
            Assert.Equal(uv.X, after[id].X, 10); Assert.Equal(uv.Y, after[id].Y, 10);
        }
    }

    [Theory]
    [InlineData(.2, .7)]
    [InlineData(.0000001, .7)]
    public void MeanValueCoordinatesReproduceAffineMaps(double x, double y)
    {
        var patch = new UVSurface(new List<Vec3D>
            { new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0), new(x, y, 0) },
            null, null, new List<Tri> { new(0, 1, 4), new(1, 2, 4), new(2, 3, 4), new(3, 0, 4) });
        Assert.True(MeanValueParameterization.TryCreate(patch, out var uv));
        // Equal boundary lengths map the square to an affine diamond.
        Assert.Equal(1 - .5 * x - .5 * y, uv[4].X, 10);
        Assert.Equal(.5 + .5 * x - .5 * y, uv[4].Y, 10);
    }

    [Fact]
    public void NonfiniteSamplesCannotProduceAnApparentlyAccurateFit()
    {
        var patch = Grid(8, 8, (u, v) => new Vec3D(u, v, 0), withUv: true);
        patch.Points[30] = new Vec3D(double.NaN, 0, 0);
        Assert.False(MeshPatchSurfaceFitter.TryFit(patch, .001, out _));
    }

    [Fact]
    public void DiskParameterizerRejectsHolesAndNonmanifoldEdges()
    {
        var hole = Grid(3, 3, (u, v) => new Vec3D(u, v, 0));
        hole.Triangles.RemoveRange(8, 2); // Central quad.
        Assert.False(MeanValueParameterization.TryCreate(hole, out _));
        var nonmanifold = Grid(2, 2, (u, v) => new Vec3D(u, v, 0));
        nonmanifold.Triangles.Add(nonmanifold.Triangles[0]);
        Assert.False(MeanValueParameterization.TryCreate(nonmanifold, out _));
    }

    [Fact]
    public void DiskParameterizerRejectsCollapsedGeometryAndInconsistentWinding()
    {
        var collapsed = Grid(2, 2, (u, v) => new Vec3D(u, v, 0));
        collapsed.Points[1] = collapsed.Points[0];
        Assert.False(MeanValueParameterization.TryCreate(collapsed, out _));
        var reversed = Grid(2, 2, (u, v) => new Vec3D(u, v, 0));
        var t = reversed.Triangles[0]; reversed.Triangles[0] = new Tri(t.A, t.C, t.B);
        Assert.False(MeanValueParameterization.TryCreate(reversed, out _));
    }

    [Fact]
    public void OrientationUsesExactStoredCoordinatesRatherThanAreaTolerance()
    {
        var tiny = new Dictionary<int, Vec2D>
        {
            [0] = new Vec2D(0, 0), [1] = new Vec2D(1e-200, 0), [2] = new Vec2D(0, 1e-200)
        };
        Assert.True(MeanValueParameterization.HasConsistentOrientation(new[] { new Tri(0, 1, 2) }, tiny, 1));
        tiny[2] = new Vec2D(1e-200, 0);
        Assert.False(MeanValueParameterization.HasConsistentOrientation(new[] { new Tri(0, 1, 2) }, tiny));
    }

    private static UVSurface CylinderStrip() => Grid(16, 8,
        (u, v) => new Vec3D(Math.Cos(5.2 * u - 2.6), 2 * v, Math.Sin(5.2 * u - 2.6)));

    private static UVSurface Grid(int nu, int nv, Func<double, double, Vec3D> point, bool withUv = false)
    {
        var points = new List<Vec3D>(); var uv = withUv ? new List<Vec2D>() : null;
        for (int i = 0; i <= nu; i++)
            for (int j = 0; j <= nv; j++)
            {
                points.Add(point(i / (double)nu, j / (double)nv));
                uv?.Add(new Vec2D(i / (double)nu, j / (double)nv));
            }
        var triangles = new List<Tri>();
        for (int i = 0; i < nu; i++)
            for (int j = 0; j < nv; j++)
            {
                int a = i * (nv + 1) + j;
                triangles.Add(new Tri(a, a + nv + 1, a + nv + 2));
                triangles.Add(new Tri(a, a + nv + 2, a + 1));
            }
        return new UVSurface(points, null, uv, triangles);
    }
}
