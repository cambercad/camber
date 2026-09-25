using Geo;
using GeoCore;
using Curves;

namespace GeoTests;

public class CollapsedCylinderFilletTests
{
    static double Volume(AnchorMesh solid) => Math.Abs(solid.Mesh.Triangles.Sum(t =>
        Vec3DOps.Dot(solid.Mesh.Positions[t.A], Vec3DOps.Cross(solid.Mesh.Positions[t.B], solid.Mesh.Positions[t.C])))) / 6;

    static EdgeGraph Graph(AnchorMesh solid) => new EdgeGraph(solid.Mesh.Triangles,
        solid.Mesh.GetTriangleGroups(), solid.Mesh.Positions, solid.Mesh.PrecisionPositions,
        solid.groupIdToExtendedName);

    static int CountTriangleComponents(AnchorMesh mesh, int groupId)
    {
        var triangles = mesh.Mesh.TrianglesEx.Select((triangle, index) => (triangle, index))
            .Where(pair => pair.triangle.GroupId == groupId).ToList();
        var byVertex = new Dictionary<int, List<int>>();
        foreach (var pair in triangles)
            foreach (int vertex in new[] { mesh.Mesh.Triangles[pair.index].A, mesh.Mesh.Triangles[pair.index].B, mesh.Mesh.Triangles[pair.index].C })
            {
                if (!byVertex.TryGetValue(vertex, out var incident)) byVertex[vertex] = incident = new List<int>();
                incident.Add(pair.index);
            }

        var unvisited = triangles.Select(pair => pair.index).ToHashSet();
        int components = 0;
        while (unvisited.Count > 0)
        {
            components++;
            var queue = new Queue<int>();
            queue.Enqueue(unvisited.First());
            while (queue.Count > 0)
            {
                int index = queue.Dequeue();
                if (!unvisited.Remove(index)) continue;
                var triangle = mesh.Mesh.Triangles[index];
                foreach (int vertex in new[] { triangle.A, triangle.B, triangle.C })
                    foreach (int neighbor in byVertex[vertex])
                        if (unvisited.Contains(neighbor)) queue.Enqueue(neighbor);
            }
        }
        return components;
    }

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

    [Fact]
    public void NearHemisphereUsesAnalyticLimitWhenTorusGapIsWithinDeviation()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(5)), 1e-4);
        const double cylinderRadius = .5;
        const double requestedRadius = .499;
        const double deviation = .002;
        var cylinder = api.CreateCylinder(new CoordinateSystem(new Vec3D(0)), cylinderRadius, 3, .0001, "cylinder");
        var foot = api.CreateCuboid(new CoordinateSystem(new Vec3D(-1, -1, -1)), new Vec3D(2, 2, 1), "foot");
        var body = api.BatchUnion(new List<AnchorMesh> { cylinder, foot });
        var top = body.groupIdToExtendedName.Single(p => p.Value == "cylinder-ExtrudeTop").Key;
        var rim = Assert.Single(Graph(body).Edges.Where(e => e.GroupIdA == top || e.GroupIdB == top));
        Assert.Contains(body.surfaceMetaData.Values, m => m.CylinderParams != null && m.CylinderParams.Radius == cylinderRadius);

        var rounded = api.Fillet(body, new List<string> { rim.Name }, requestedRadius, deviation, "rounded");

        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.True(rounded.IsVolume);
        var sphere = Assert.Single(rounded.surfaceMetaData.Values.Where(m => m.SphereParams != null));
        Assert.Equal(cylinderRadius, sphere.SphereParams.Radius);
        Assert.Equal(2.5, sphere.SphereParams.Center.Z, 10);
    }

    [Fact]
    public void DisjointNearHemisphereRimsUseTheAnalyticLimitTogether()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-4), new Vec3D(5)), 1e-4);
        const double radius = .5;
        const double requestedRadius = .499;
        const double deviation = .002;
        var baseMesh = api.CreateCuboid(new CoordinateSystem(new Vec3D(-3, -1, -1)), new Vec3D(6, 2, 1), "base");
        var left = api.CreateCylinder(new CoordinateSystem(new Vec3D(-1.2, 0, 0)), radius, 3, .0001, "left");
        var right = api.CreateCylinder(new CoordinateSystem(new Vec3D(1.2, 0, 0)), radius, 3, .0001, "right");
        var body = api.BatchUnion(new List<AnchorMesh> { baseMesh, left, right });
        Assert.DoesNotContain(body.surfaceMetaData.Values, m => m.SphereParams != null);
        var topFaces = body.groupIdToExtendedName.Where(p => p.Value is "left-ExtrudeTop" or "right-ExtrudeTop")
            .Select(p => p.Key).ToHashSet();
        var rims = Graph(body).Edges.Where(e => topFaces.Contains(e.GroupIdA) || topFaces.Contains(e.GroupIdB)).ToList();
        Assert.True(rims.Count == 2, string.Join("; ", rims.Select(e =>
            $"{e.Name} ({body.groupIdToExtendedName[e.GroupIdA]} | {body.groupIdToExtendedName[e.GroupIdB]})")));

        var blending = new EdgeBlending();
        int groupIdOffset = 0;
        var rounded = blending.BlendEdges(body, rims.Select(e => e.Name).ToList(), requestedRadius,
            api.Converter, deviation, ref groupIdOffset);

        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.True(rounded.IsVolume);
        var spheres = rounded.surfaceMetaData.Values.Where(m => m.SphereParams != null).ToList();
        Assert.All(spheres, m => Assert.Equal(radius, m.SphereParams.Radius));
        Assert.Equal(2, spheres.Select(m => Math.Round(m.SphereParams.Center.X, 8)).Distinct().Count());
        Assert.Contains(spheres, m => Math.Abs(m.SphereParams.Center.X + 1.2) < 1e-9);
        Assert.Contains(spheres, m => Math.Abs(m.SphereParams.Center.X - 1.2) < 1e-9);
    }

    [Fact]
    public void MultiCircleSketchExtrusionFilletsAllDisjointCapsTogether()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-4), new Vec3D(130, 15, 5)), .01);
        const double cylinderRadius = .6725;
        const double baseHeight = 1.6725;
        var baseMesh = api.CreateCuboid(new CoordinateSystem(new Vec3D(0)), new Vec3D(127.5, 10, baseHeight), "base");
        const int circleCount = 31;
        // Use the official Braille string's 31 positions in one sketch.
        var sketch = api.GetPlotterSketcher(new Plane3D(new Vec3D(0, 0, baseHeight),
            new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0)), "dotsSketch");
        var centers = new (double X, double Y)[]
        {
            (2.5, 2.5), (8.5, 2.5), (12, 7.5), (12, 5), (14.5, 7.5),
            (20.5, 2.5), (24, 7.5), (24, 5), (24, 2.5), (26.5, 5),
            (32.5, 2.5), (36, 7.5), (38.5, 5), (44.5, 2.5), (48, 7.5),
            (50.5, 5), (56.5, 2.5), (62.5, 2.5), (68.5, 2.5), (72, 7.5),
            (74.5, 7.5), (80.5, 2.5), (86.5, 2.5), (92.5, 2.5), (96, 7.5),
            (104.5, 2.5), (110.5, 2.5), (116.5, 2.5), (120, 7.5),
            (122.5, 7.5), (122.5, 5),
        };
        Assert.Equal(circleCount, centers.Length);
        foreach (var center in centers) sketch.AddCircle(new Vec2D(center.X + 2.5, center.Y), cylinderRadius);
        var dots = api.Extrude(sketch, cylinderRadius, .01, "ext1");
        var body = api.BatchUnion(new List<AnchorMesh> { baseMesh, dots });
        var topGroups = body.groupIdToExtendedName.Where(pair => pair.Value.StartsWith("ext1-ExtrudeTop", StringComparison.Ordinal))
            .Select(pair => pair.Key).ToHashSet();
        var topEdges = Graph(body).Edges.Where(edge => topGroups.Contains(edge.GroupIdA) || topGroups.Contains(edge.GroupIdB)).ToList();
        Assert.Equal(circleCount, topGroups.Count);
        Assert.Equal(circleCount, topEdges.Count);
        Assert.All(topGroups, group => Assert.Equal(1, CountTriangleComponents(body, group)));

        var rounded = api.Fillet(body, topEdges.Select(edge => edge.Name).ToList(),
            cylinderRadius - .001, .01, "rounded");

        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.True(rounded.IsVolume);
        var spheres = rounded.surfaceMetaData.Values.Where(meta => meta.SphereParams != null).ToList();
        Assert.Equal(circleCount, spheres.Select(meta => Math.Round(meta.SphereParams.Center.X, 8) +
            100 * Math.Round(meta.SphereParams.Center.Y, 8)).Distinct().Count());
        Assert.All(spheres, meta => Assert.InRange(Math.Abs(meta.SphereParams.Center.Z - baseHeight), 0, .01));
    }

}
