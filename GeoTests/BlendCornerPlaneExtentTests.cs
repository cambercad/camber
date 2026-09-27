using Geo;
using GeoCore;

namespace GeoTests;

public class BlendCornerPlaneExtentTests
{
    [Fact]
    public void CornerTrimPlaneExactlyCoversProjectedTargetBounds()
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        var targetPoints = new List<Rat3Hybrid>
        {
            new(-700000, 200000, 0), new(800000, -900000, 100000), new(100000, 700000, -500000)
        };
        var target = new UVSurface(converter.Convert(targetPoints),
            Enumerable.Repeat(new Vec3D(0, 0, 1), targetPoints.Count).ToList(),
            new List<Vec2D> { new(0, 0), new(1, 0), new(0, 1) },
            new List<Tri> { new(0, 1, 2) }, targetPoints);
        var origin = new Rat3Hybrid(0, 0, 0);
        var normal = new Rat3Hybrid(1, 2, 3);

        var plane = BlendCorner.PlaneCoveringSurface(target, origin, normal, converter);

        foreach (var point in plane.PointsPrecise)
            Assert.Equal(BigRationalHybrid.Zero, Rat3Hybrid.Dot(point - origin, normal));
        var minX = plane.PointsPrecise.Select(point => point.X).Aggregate((a, b) => a.CompareTo(b) < 0 ? a : b);
        var maxX = plane.PointsPrecise.Select(point => point.X).Aggregate((a, b) => a.CompareTo(b) > 0 ? a : b);
        var minY = plane.PointsPrecise.Select(point => point.Y).Aggregate((a, b) => a.CompareTo(b) < 0 ? a : b);
        var maxY = plane.PointsPrecise.Select(point => point.Y).Aggregate((a, b) => a.CompareTo(b) > 0 ? a : b);
        foreach (var point in targetPoints)
        {
            Assert.InRange(point.X.CompareTo(minX), 0, 1);
            Assert.InRange(point.X.CompareTo(maxX), -1, 0);
            Assert.InRange(point.Y.CompareTo(minY), 0, 1);
            Assert.InRange(point.Y.CompareTo(maxY), -1, 0);
        }
        var triangle = plane.Triangles[0];
        var actualNormal = Rat3Hybrid.Cross(plane.PointsPrecise[triangle.B] - plane.PointsPrecise[triangle.A],
            plane.PointsPrecise[triangle.C] - plane.PointsPrecise[triangle.A]);
        Assert.True(Rat3Hybrid.Dot(actualNormal, normal).Sign() > 0);
    }

    [Fact]
    public void RoundedBlendBoundaryRejectsTJunctions()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), .01);
        var converter = api.Converter;
        var volume = api.CreateCuboid(new Vec3D(-2), new Vec3D(2), "volume");
        UVSurface Surface(List<Rat3Hybrid> precise) => new(
            converter.Convert(precise),
            Enumerable.Repeat(new Vec3D(0, 0, 1), precise.Count).ToList(),
            Enumerable.Range(0, precise.Count).Select(i => new Vec2D(i, 0)).ToList(),
            new List<Tri> { new(0, 1, 2) }, precise);
        var longEdge = Surface(new List<Rat3Hybrid> { new(0, 0, 0), new(2, 0, 0), new(0, 1, 0) });
        var subdividedEdge = Surface(new List<Rat3Hybrid> { new(0, 0, 0), new(1, 0, 0), new(1, -1, 0) });
        var names = new List<(string Name, GeoCore.SurfaceMetaData Metadata)>
        {
            ("long", null), ("subdivided", null)
        };

        var error = Assert.Throws<InvalidOperationException>(() =>
            EdgeBlendPipeline.ValidateBlendPatchBoundaries(
                new[] { longEdge, subdividedEdge }, names, volume.Mesh, converter));

        Assert.Contains("T-junction", error.Message);
    }

    [Fact]
    public void RoundedBlendBoundaryRejectsLoopsJoinedOnlyAtOneVertex()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), .01);
        var converter = api.Converter;
        var volume = api.CreateCuboid(new Vec3D(-2), new Vec3D(2), "volume");
        UVSurface Surface(List<Rat3Hybrid> precise) => new(
            converter.Convert(precise),
            Enumerable.Repeat(new Vec3D(0, 0, 1), precise.Count).ToList(),
            Enumerable.Range(0, precise.Count).Select(i => new Vec2D(i, 0)).ToList(),
            new List<Tri> { new(0, 1, 2) }, precise);
        var first = Surface(new List<Rat3Hybrid>
        {
            new(0, 0, 2), new(1, 0, 2), new(0, 1, 2)
        });
        var second = Surface(new List<Rat3Hybrid>
        {
            new(0, 0, 2), new(-1, 0, 2), new(0, -1, 2)
        });

        var error = Assert.Throws<InvalidOperationException>(() =>
            EdgeBlendPipeline.ValidateBlendPatchBoundaries(
                new[] { first, second }, new[] { ("first", (GeoCore.SurfaceMetaData)null), ("second", (GeoCore.SurfaceMetaData)null) },
                volume.Mesh, converter));

        Assert.Contains("closed contour", error.Message);
        Assert.Contains("valence 4", error.Message);
    }
}
