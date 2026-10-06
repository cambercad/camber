using GeoCore;

namespace GeoTests;

public class SurfaceExtensionAttachmentTests
{
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(true, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(true, false, true)]
    [InlineData(false, true, true)]
    [InlineData(true, true, true)]
    public void CollarRetainsExactBoundaryEvenBelowLatticeResolution(bool curved, bool largeTranslation, bool reversed)
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-20), new Vec3D(20)));
        var precise = new List<Rat3Hybrid>();
        var uv = new List<Vec2D>();
        var normals = new List<Vec3D>();
        var shift = largeTranslation ? new BigRationalHybrid(1L << 60) : BigRationalHybrid.Zero;
        for (int x = 0; x < 3; x++)
            for (int y = 0; y < 3; y++)
            {
                // Non-lattice source vertices must survive without quantization.
                precise.Add(new Rat3Hybrid(shift + new BigRationalHybrid(x * 20 + 1, 3),
                    shift + new BigRationalHybrid(y * 20 + 1, 3),
                    curved ? new BigRationalHybrid(x * y, 3) : BigRationalHybrid.Zero));
                uv.Add(new Vec2D(x / 2.0, y / 2.0));
                normals.Add(new Vec3D(curved ? -y / 20.0 : 0, curved ? -x / 20.0 : 0, 1).Normalized() * (reversed ? -1 : 1));
            }
        var triangles = new List<Tri>();
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
            {
                int a = x * 3 + y;
                triangles.Add(reversed ? new Tri(a, a + 4, a + 3) : new Tri(a, a + 3, a + 4));
                triangles.Add(reversed ? new Tri(a, a + 1, a + 4) : new Tri(a, a + 4, a + 1));
            }
        var source = new UVSurface(converter.Convert(precise), normals, uv, triangles, precise);
        var result = source.GetExtendedSurface(converter.SmallestUnit() * .01, converter, out var collar);
        Assert.Null(result.NurbsSurface); // Metadata-free geometry is supported.
        Assert.Equal(9, source.PointsPrecise.Count);
        Assert.Equal(8, source.Triangles.Count);
        Assert.Equal(source.PointsPrecise, result.PointsPrecise.Take(9));
        Assert.Equal(source.Points, result.Points.Take(9));
        Assert.Equal(source.Triangles, result.Triangles.Take(8));
        Assert.Equal(result.Triangles.Skip(8), collar.Triangles);
        for (int i = 9; i < result.Points.Count; i++)
        {
            Assert.Equal(converter.Convert(result.PointsPrecise[i]), result.Points[i]);
            Assert.InRange(result.Normals[i].Length(), .999999999, 1.000000001);
        }
        AssertConformingAttachment(source, result);
    }

    internal static void AssertConformingAttachment(UVSurface source, UVSurface result)
    {
        var uses = new Dictionary<(int, int), List<(int, int)>>();
        static (int, int) Key(int a, int b) => a < b ? (a, b) : (b, a);
        foreach (var triangle in result.Triangles)
        {
            var a = result.PointsPrecise[triangle.A];
            Assert.False(Rat3Hybrid.Cross(result.PointsPrecise[triangle.B] - a,
                result.PointsPrecise[triangle.C] - a).IsZero());
            Add(triangle.A, triangle.B); Add(triangle.B, triangle.C); Add(triangle.C, triangle.A);
        }
        void Add(int a, int b)
        {
            var key = Key(a, b);
            if (!uses.TryGetValue(key, out var list)) uses.Add(key, list = new());
            list.Add((a, b));
        }
        foreach (var edge in Adjacency.BuildEdgeList(source.Triangles))
        {
            if (!edge.IsOnBorder) continue;
            var attachment = uses[Key(edge.Start, edge.End)];
            Assert.Equal(2, attachment.Count);
            Assert.Equal(attachment[0], (attachment[1].Item2, attachment[1].Item1));
        }
        foreach (var (edge, owners) in uses)
        {
            Assert.InRange(owners.Count, 1, 2);
            if (owners.Count == 2) Assert.Equal(owners[0], (owners[1].Item2, owners[1].Item1));
            var a = result.PointsPrecise[edge.Item1];
            var direction = result.PointsPrecise[edge.Item2] - a;
            var lengthSquared = Rat3Hybrid.Dot(direction, direction);
            for (int i = 0; i < result.PointsPrecise.Count; i++)
            {
                if (i == edge.Item1 || i == edge.Item2) continue;
                var delta = result.PointsPrecise[i] - a;
                if (!Rat3Hybrid.Cross(direction, delta).IsZero()) continue;
                var along = Rat3Hybrid.Dot(delta, direction);
                Assert.False(along.Sign() > 0 && along < lengthSquared, "Exact T-junction in support extension.");
            }
        }
    }
}
