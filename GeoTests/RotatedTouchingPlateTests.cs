using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class RotatedTouchingPlateTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void DeterministicOrientationSweep_PreservesContactsAndGaps()
    {
        // Irrational-looking rotations exercise the double-to-exact boundary;
        // include both face contact and separated bodies at each orientation.
        var random = new Random(70321);
        for (int i = 0; i < 24; i++)
        {
            double angle = (random.NextDouble() - .5) * 2 * Math.PI;
            double azimuth = (random.NextDouble() - .5) * 2 * Math.PI;
            foreach (double gap in new[] { 0.0, .01 })
            {
                TouchingPerpendicularPlates_UnionRemainsManifold(angle, azimuth, gap);
            }
        }
    }

    [Theory]
    [InlineData(0, 0, 0)]
    [InlineData(.750929, 0, 0)]
    [InlineData(.3, .6, 0)]
    [InlineData(1.1, -.4, 0)]
    [InlineData(.750929, 0, .01)]
    public void TouchingPerpendicularPlates_UnionRemainsManifold(double angle, double azimuth, double gap)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-420, -300, -50),
                                      new Vec3D(1420, 300, 1120)), .1);
        var axis = new Vec3D(Math.Sin(angle)*Math.Cos(azimuth),
            Math.Sin(angle)*Math.Sin(azimuth), Math.Cos(angle));
        var lateral = new Vec3D(-Math.Sin(azimuth), Math.Cos(azimuth), 0);
        var outward = Vec3DOps.Cross(axis, lateral);
        var origin = new Vec3D(475, 0, 360) + axis*35 + outward*78;
        var back = api.GetPlotterSketcher(new CoordinateSystem(origin-outward*38,
            lateral, axis, Vec3DOps.Cross(lateral, axis)), "Back");
        back.AddRectangleFromCorners(new Vec2D(-15, -3), new Vec2D(15, 125));
        var foot = api.GetPlotterSketcher(new CoordinateSystem(origin-axis*3,
            lateral, outward, Vec3DOps.Cross(lateral, outward)), "Foot");
        foot.AddRectangleFromCorners(new Vec2D(-15, -38+gap), new Vec2D(15, 20));
        var a = api.Extrude(back, 2, name: "BackPlate");
        var b = api.Extrude(foot, 3, name: "FootPlate");
        var union = api.Boolean(a, b, BooleanOp.Union, "CageBracket");
        Assert.True(MeshAnalysis.IsWatertightMesh(union.Mesh.Positions, union.Mesh.Triangles), "Raw union must be watertight");
        union.EnsureCoplanarPostProcessed();
        Assert.True(MeshAnalysis.IsWatertightMesh(union.Mesh.Positions, union.Mesh.Triangles));
        // Compare exact signed six-volumes in kernel coordinates. No floating
        // tolerance may conceal lost material or a sliver at the shared face.
        static BigRationalHybrid SixVolume(MeshNormalUV mesh)
        {
            var sum = new BigRationalHybrid(0);
            var points = mesh.PrecisionPositions;
            foreach (var t in mesh.Triangles)
                sum += Rat3Hybrid.Dot(points[t.A], Rat3Hybrid.Cross(points[t.B], points[t.C]));
            sum.Simplify();
            return sum;
        }
        Assert.True(SixVolume(a.Mesh) + SixVolume(b.Mesh) == SixVolume(union.Mesh),
            "Exact rational volume must be conserved");
        // Commutativity and zero-volume contact are separate invariants:
        // welding a small gap or losing a shared face must not pass by chance.
        var reversed = api.Boolean(b, a, BooleanOp.Union, "ReversedBracket");
        reversed.EnsureCoplanarPostProcessed();
        Assert.True(MeshAnalysis.IsWatertightMesh(reversed.Mesh.Positions, reversed.Mesh.Triangles));
        Assert.True(SixVolume(reversed.Mesh) == SixVolume(union.Mesh));
        var intersection = api.Boolean(a, b, BooleanOp.Intersect, "ContactOnly");
        Assert.Empty(intersection.Mesh.Triangles);
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-420, -300, -50), new Vec3D(1420, 300, 1120)));
        // Preserve the existing operating-space resolution. The frame-plane
        // quantization must not introduce movement beyond a few lattice cells.
        foreach (var solid in new[] { a, b })
            for (int i = 0; i < solid.Mesh.Positions.Count; i++)
                Assert.InRange((converter.Convert(solid.Mesh.PrecisionPositions[i]) - solid.Mesh.Positions[i]).Length(),
                    0, 3 * converter.SmallestUnit());
        // A gap must remain a gap; face contact must produce a single shell.
        var triangles = union.Mesh.Triangles;
        var parents = Enumerable.Range(0, triangles.Count).ToArray();
        int Root(int x) { while (parents[x] != x) x = parents[x]; return x; }
        var edges = new Dictionary<(int, int), int>();
        for (int i = 0; i < triangles.Count; i++)
        {
            var t = triangles[i];
            foreach (var (a0, b0) in new[] { (t.A, t.B), (t.B, t.C), (t.C, t.A) })
            {
                var edge = (Math.Min(a0, b0), Math.Max(a0, b0));
                if (edges.TryGetValue(edge, out int other)) parents[Root(i)] = Root(other);
                else edges[edge] = i;
            }
        }
        Assert.Equal(gap == 0 ? 1 : 2, Enumerable.Range(0, triangles.Count).Select(Root).Distinct().Count());
    }
}
