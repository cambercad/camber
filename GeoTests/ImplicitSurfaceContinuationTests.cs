using GeoCore;
using NURBS;
using Geo.NurbsConstruction;
using Geo.Shelling;

namespace GeoTests;

public class ImplicitSurfaceContinuationTests
{
    [Theory]
    [InlineData(.2, 1, false)]
    [InlineData(-.2, 1, false)]
    [InlineData(-.2, 10, true)]
    public void OneSidedNurbsSamplesContinueAcrossTrimAndRespectSignedOffset(double offset, double scale, bool moved)
    {
        // A genuine bicubic Bezier surface, not an analytic sphere/cylinder fixture.
        var net = new Vec3D[4][];
        for (int i = 0; i < 4; i++)
        {
            net[i] = new Vec3D[4];
            for (int j = 0; j < 4; j++)
                net[i][j] = new Vec3D(i / 3.0, j / 3.0,
                    i * (i - 1) / 6.0 + .3 * j * (j - 1) * (j - 2) / 6.0 + .2 * i * j / 9.0);
        }
        var source = new BSplineSurface(3, 3, net,
            new double[] { 0, 0, 0, 0, 1, 1, 1, 1 }, new double[] { 0, 0, 0, 0, 1, 1, 1, 1 });
        Vec3D Rotate(Vec3D p) => moved ? new Vec3D(-p.Z, p.X, -p.Y) : p;
        Vec3D Point(Vec3D p) => Rotate(p) * scale + (moved ? new Vec3D(5, -3, 7) : new Vec3D(0));
        var samples = new List<(Vec3D Point, Vec3D Normal)>();
        for (int i = 0; i <= 6; i++)
            for (int j = 0; j <= 6; j++)
            {
                double u = i / 12.0, v = j / 12.0;
                samples.Add((Point(source.Evaluate(u, v)), Rotate(source.EvaluateNormal(u, v))));
            }
        double x = -.15, y = .2;
        var expectedNormal = Rotate(new Vec3D(-(2 * x + .2 * y), -(.9 * y * y + .2 * x), 1).Normalized());
        var expected = Point(new Vec3D(x, y, x * x + .3 * y * y * y + .2 * x * y)) + expectedNormal * offset * scale;
        Assert.True(ImplicitSurfaceContinuation.TryFit(samples, 1e-8 * scale, out var implicitGuide));
        Assert.True(implicitGuide.TryOffsetTangent(expected, offset * scale, out var implicitTangent));
        Assert.InRange((implicitTangent.Point - expected).Length(), 0, 1e-8 * scale);
        Assert.InRange((implicitTangent.Normal - expectedNormal).Length(), 0, 1e-8);
        Assert.False(implicitGuide.TryOffsetTangent(Point(new Vec3D(10, 10, 0)), offset * scale, out _));

        // Exercise the actual multi-support resolver, with no stored NURBS/UV
        // evaluator on the patch. The two trim planes force continuation into
        // the missing domain, rather than accepting an initial tangent miter.
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-20 * scale), new Vec3D(20 * scale)));
        var precise = new List<Rat3Hybrid>(); var positions = new List<Vec3D>();
        var normals = new List<Vec3D>(); var uv = new List<Vec2D>();
        foreach (var sample in samples)
        {
            var lattice = converter.Convert(sample.Point);
            var exact = new Rat3Hybrid(lattice.X, lattice.Y, lattice.Z);
            precise.Add(exact); positions.Add(converter.Convert(exact));
            normals.Add(sample.Normal); uv.Add(new Vec2D(0));
        }
        var triangles = new List<Tri>();
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
            {
                int a = i * 7 + j;
                triangles.Add(new Tri(a, a + 7, a + 8)); triangles.Add(new Tri(a, a + 8, a + 1));
            }
        var patch = new UVSurface(positions, normals, uv, triangles, precise);
        var adapter = new MeshShellSurfaceAdapter();
        var meshSupport = (MeshShellSupport)adapter.CreateOffsetSupport(null, patch, normals[0], -offset * scale, converter);
        meshSupport.ConfigureContinuation(offset * scale, .002 * scale, .5 * scale);
        var supports = new IShellSurfaceSupport[] { meshSupport,
            new PlanarShellSupport(new PlaneSurfaceParams { Origin = expected, Normal = Rotate(new Vec3D(1, 0, 0)) }),
            new PlanarShellSupport(new PlaneSurfaceParams { Origin = expected, Normal = Rotate(new Vec3D(0, 1, 0)) }) };
        Assert.True(adapter.TryResolveVertex(0, positions[0], supports, out var resolved));
        Assert.InRange((resolved - expected).Length(), 0, .002 * scale);
        Assert.Equal(precise, patch.PointsPrecise);
    }

    [Fact]
    public void InsufficientOrInconsistentSamplesAreRejected()
    {
        var samples = new List<(Vec3D Point, Vec3D Normal)> { (new(0), new(0, 0, 1)) };
        Assert.False(ImplicitSurfaceContinuation.TryFit(samples, .001, out _));
        for (int i = 1; i < 20; i++) samples.Add((new Vec3D(i / 20.0, 0, 0), new Vec3D(0, 0, 1)));
        Assert.False(ImplicitSurfaceContinuation.TryFit(samples, .001, out _));
    }

    [Fact]
    public void OffsetCrossingTheLocalFocalSurfaceIsRejected()
    {
        var samples = new List<(Vec3D Point, Vec3D Normal)>();
        for (int i = -3; i <= 3; i++)
            for (int j = -3; j <= 3; j++)
            {
                double x = i / 10.0, y = j / 10.0;
                samples.Add((new Vec3D(x, y, x * x + y * y), new Vec3D(-2 * x, -2 * y, 1).Normalized()));
            }
        Assert.True(ImplicitSurfaceContinuation.TryFit(samples, 1e-8, out var implicitGuide));
        Assert.True(implicitGuide.TryOffsetTangent(new Vec3D(0, 0, .2), .2, out _));
        Assert.False(implicitGuide.TryOffsetTangent(new Vec3D(0, 0, .75), .75, out _));
    }

    [Fact]
    public void InaccurateFitsAndInvalidSamplesAreNotAccepted()
    {
        var samples = new List<(Vec3D Point, Vec3D Normal)>();
        for (int i = 0; i <= 8; i++)
            for (int j = 0; j <= 8; j++)
                samples.Add((new Vec3D(i / 10.0, j / 10.0, .1 * Math.Sin(5 * i)), new Vec3D(0, 0, 1)));
        Assert.False(ImplicitSurfaceContinuation.TryFit(samples, .001, out _));
        samples[0] = (new Vec3D(double.NaN, 0, 0), new Vec3D(0, 0, 1));
        Assert.False(ImplicitSurfaceContinuation.TryFit(samples, .001, out _));
    }
}
