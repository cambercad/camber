using Geo;
using GeoCore;

namespace GeoTests;

public class CollapsedCylinderFilletTests
{
    static double Volume(AnchorMesh solid) => Math.Abs(solid.Mesh.Triangles.Sum(t =>
        Vec3DOps.Dot(solid.Mesh.Positions[t.A], Vec3DOps.Cross(solid.Mesh.Positions[t.B], solid.Mesh.Positions[t.C])))) / 6;

    static EdgeGraph Graph(AnchorMesh solid) => new EdgeGraph(solid.Mesh.Triangles,
        solid.Mesh.GetTriangleGroups(), solid.Mesh.Positions, solid.Mesh.PrecisionPositions,
        solid.groupIdToExtendedName);

    [Theory]
    [InlineData(.5, .5, .7, false, true, false)]
    [InlineData(.5, .5, .7, true, true, false)]
    [InlineData(.5, .5, 0, false, false, false)]
    [InlineData(.5, .5, 0, true, false, false)]
    [InlineData(.5, 3, 0, false, false, false)]
    [InlineData(.5, 3, 0, true, false, false)]
    [InlineData(.5, 3, .7, false, true, false)]
    [InlineData(.5, 3, .7, true, true, false)]
    [InlineData(.75, 3, .4, false, true, false)]
    [InlineData(.5, 3, 0, false, false, true)]
    [InlineData(.5, 3, .7, false, true, true)]
    [InlineData(.5, 3, .7, true, true, true)]
    public void ExplicitCylinderRimAtEqualRadiusProducesHemisphere(double radius, double height,
        double angle, bool bottom, bool translated, bool wide)
    {
        var origin = translated ? new Vec3D(1033, 207, 878) : new Vec3D(0);
        var bounds = wide ? new Box3D(new Vec3D(-2000), new Vec3D(2000)) :
            new Box3D(origin - new Vec3D(2), origin + new Vec3D(5));
        var api = new GeoAPI(bounds, wide ? .2 : 1e-4);
        double deviation = wide ? .02 : .0001;
        double radialTolerance = deviation + 2 * Math.Sqrt(3) * api.Converter.SmallestUnit();
        var axis = new Vec3D(Math.Sin(angle), 0, Math.Cos(angle));
        var frame = new CoordinateSystem(origin, new Vec3D(Math.Cos(angle), 0, -Math.Sin(angle)), new Vec3D(0, 1, 0), axis);
        var cylinder = api.CreateCylinder(frame, radius, height, deviation, "cylinder");
        var face = cylinder.groupIdToExtendedName.Single(p => p.Value == "cylinder-Extrude" + (bottom ? "Bottom" : "Top")).Key;
        var rim = Graph(cylinder).Edges.Where(e => e.GroupIdA == face || e.GroupIdB == face).ToList();
        Assert.Single(rim);
        var rounded = api.Fillet(cylinder, new List<string> { rim[0].Name }, radius, deviation, "hemisphere");
        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        // Shift tetrahedra to the part datum to avoid world-origin cancellation.
        double actual = Math.Abs(rounded.Mesh.Triangles.Sum(t => Vec3DOps.Dot(rounded.Mesh.Positions[t.A] - origin,
            Vec3DOps.Cross(rounded.Mesh.Positions[t.B] - origin, rounded.Mesh.Positions[t.C] - origin)))) / 6;
        var expected = Math.PI * radius * radius * (height - radius) + 2 * Math.PI * radius * radius * radius / 3;
        double surfaceArea = 2 * Math.PI * radius * (height - radius) + 3 * Math.PI * radius * radius;
        Assert.InRange(actual, expected - surfaceArea * radialTolerance, expected + surfaceArea * radialTolerance);
        var center = origin + axis * (bottom ? radius : height - radius);
        var pole = bottom ? -axis : axis;
        var cap = rounded.Mesh.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct()
            .Select(i => rounded.Mesh.Positions[i]).Where(p => Vec3DOps.Dot(p - center, pole) > 1e-4).ToList();
        Assert.NotEmpty(cap);
        Assert.All(cap, p => Assert.InRange((p - center).Length(), radius - radialTolerance, radius + radialTolerance));
        Assert.InRange(cap.Max(p => Vec3DOps.Dot(p - center, pole)), radius - radialTolerance, radius + radialTolerance);
        var sphere = Assert.Single(rounded.surfaceMetaData.Values.Where(m => m.SphereParams != null));
        Assert.Equal(radius, sphere.SphereParams.Radius);
        Assert.InRange((sphere.SphereParams.Center - center).Length(), 0, 1e-9);
    }

    [Fact]
    public void HemisphereRetainsOtherFacesAboveTheContactPlane()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(5)), 1e-4);
        var cylinder = api.CreateCylinder(new CoordinateSystem(new Vec3D(0)), .5, 3, .0001, "cylinder");
        var foot = api.CreateCuboid(new CoordinateSystem(new Vec3D(-1, -1, -1)), new Vec3D(2, 2, 1.5), "foot");
        var tower = api.CreateCuboid(new CoordinateSystem(new Vec3D(.8, -.2, 0)), new Vec3D(.4, .4, 4), "tower");
        var blank = api.BatchUnion(new List<AnchorMesh> { cylinder, foot, tower });
        var top = blank.groupIdToExtendedName.Single(p => p.Value == "cylinder-ExtrudeTop").Key;
        var rim = Assert.Single(Graph(blank).Edges.Where(e => e.GroupIdA == top || e.GroupIdB == top));
        var rounded = api.Fillet(blank, new List<string> { rim.Name }, .5, .0001, "rounded");
        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.InRange(Volume(blank) - Volume(rounded), Math.PI / 24 - .002, Math.PI / 24 + .002);
        var towerTop = rounded.groupIdToExtendedName.Single(p => p.Value == "tower-ExtrudeTop").Key;
        var vertices = rounded.Mesh.TrianglesEx.Where(t => t.GroupId == towerTop).ToList();
        Assert.NotEmpty(vertices);
        Assert.InRange(rounded.Mesh.Positions.Max(p => p.Z), 3.99999, 4.00001);
    }

}
