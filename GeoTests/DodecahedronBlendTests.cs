using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class DodecahedronBlendTests
{
    [Theory]
    [InlineData(.05)]
    [InlineData(.1)]
    public void EveryDodecahedronEdgeFilletPreservesTheSolid(double radius)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .001);
        var body = BuildDodecahedron(api);
        var graph = new EdgeGraph(body.Mesh.Triangles, body.Mesh.GetTriangleGroups(),
            body.Mesh.Positions, body.Mesh.PrecisionPositions, body.groupIdToExtendedName);
        Assert.Equal(30, graph.Edges.Count);
        var edges = new List<string>();
        foreach (var edge in graph.Edges) edges.Add(edge.Name);

        var rounded = api.Fillet(body, edges, radius, .001, "rounded_dodecahedron");

        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));
        double before = SignedVolume(body.Mesh);
        double after = SignedVolume(rounded.Mesh);
        Assert.InRange(after, before * .99, before);
    }

    [Fact]
    public void DodecahedronBlendIsIndependentOfFaceAndEdgeOrderAndRigidPose()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .001);
        double referenceVolume = RoundedVolume(BuildDodecahedron(api), api, shuffleEdges: false);

        double reorderedVolume = RoundedVolume(BuildDodecahedron(api, faceOrderSeed: 17),
            api, shuffleEdges: true);
        Assert.InRange(Math.Abs(reorderedVolume - referenceVolume), 0, referenceVolume * 1e-9);

        var pose = new Transform(new Vec3D(.25, -.375, .5),
            new Quaternion(0, 0, Math.Sin(Math.PI / 8), Math.Cos(Math.PI / 8)));
        double transformedVolume = RoundedVolume(BuildDodecahedron(api, faceOrderSeed: 29, pose: pose),
            api, shuffleEdges: true);
        Assert.InRange(Math.Abs(transformedVolume - referenceVolume), 0, referenceVolume * .01);
    }

    [Fact]
    public void DodecahedronBlendScalesWithModelAndTolerance()
    {
        double referenceNormalizedVolume = 0;
        double referenceVolumeRatio = 0;
        foreach (double scale in new[] { 1.0, .1, 10.0 })
        {
            double tolerance = .001 * scale;
            var api = new GeoAPI(new Box3D(new Vec3D(-10 * scale), new Vec3D(20 * scale)), tolerance);
            var body = BuildDodecahedron(api, scale: scale);
            var graph = new EdgeGraph(body.Mesh.Triangles, body.Mesh.GetTriangleGroups(),
                body.Mesh.Positions, body.Mesh.PrecisionPositions, body.groupIdToExtendedName);
            Assert.Equal(30, graph.Edges.Count);
            var edges = new List<string>();
            foreach (var edge in graph.Edges) edges.Add(edge.Name);

            var rounded = api.Fillet(body, edges, .1 * scale, tolerance,
                $"rounded_dodecahedron_{scale}");
            MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
            Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));

            double before = SignedVolume(body.Mesh);
            double after = SignedVolume(rounded.Mesh);
            double volumeRatio = after / before;
            double normalizedVolume = after / (scale * scale * scale);
            Assert.InRange(volumeRatio, .99, 1.0);
            if (scale == 1)
            {
                referenceNormalizedVolume = normalizedVolume;
                referenceVolumeRatio = volumeRatio;
            }
            else
            {
                Assert.InRange(Math.Abs(normalizedVolume - referenceNormalizedVolume),
                    0, referenceNormalizedVolume * .01);
                Assert.InRange(Math.Abs(volumeRatio - referenceVolumeRatio), 0, .01);
            }
        }
    }

    [Fact]
    public void DodecahedronRejectsObviouslyExcessiveFilletRadius()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .001);
        var body = BuildDodecahedron(api);
        var graph = new EdgeGraph(body.Mesh.Triangles, body.Mesh.GetTriangleGroups(),
            body.Mesh.Positions, body.Mesh.PrecisionPositions, body.groupIdToExtendedName);
        var edges = new List<string>();
        foreach (var edge in graph.Edges) edges.Add(edge.Name);

        Assert.ThrowsAny<InvalidOperationException>(() =>
            api.Fillet(body, edges, 100, .001, "invalid_dodecahedron"));
        Assert.True(MeshAnalysis.IsWatertightMesh(body.Mesh.PrecisionPositions, body.Mesh.Triangles));
    }

    private static double RoundedVolume(AnchorMesh body, GeoAPI api, bool shuffleEdges)
    {
        var graph = new EdgeGraph(body.Mesh.Triangles, body.Mesh.GetTriangleGroups(),
            body.Mesh.Positions, body.Mesh.PrecisionPositions, body.groupIdToExtendedName);
        Assert.Equal(30, graph.Edges.Count);
        var edges = new List<string>();
        foreach (var edge in graph.Edges) edges.Add(edge.Name);
        if (shuffleEdges)
        {
            var random = new Random(2718);
            for (int i = edges.Count - 1; i > 0; i--)
            {
                int j = random.Next(i + 1);
                (edges[i], edges[j]) = (edges[j], edges[i]);
            }
        }

        var rounded = api.Fillet(body, edges, .1, .001, "rounded_dodecahedron");
        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));
        return SignedVolume(rounded.Mesh);
    }

    private static double SignedVolume(MeshNormalUV mesh)
    {
        double volume = 0;
        foreach (var triangle in mesh.Triangles)
            volume += Vec3DOps.Dot(mesh.Positions[triangle.A],
                Vec3DOps.Cross(mesh.Positions[triangle.B], mesh.Positions[triangle.C])) / 6;
        return volume;
    }

    private static AnchorMesh BuildDodecahedron(GeoAPI api, int faceOrderSeed = 0,
        Transform? pose = null, double scale = 1)
    {
        double phi = (1 + Math.Sqrt(5)) / 2;
        var ico = new List<Vec3D>();
        foreach (double a in new[] { -1.0, 1.0 })
            foreach (double b in new[] { -1.0, 1.0 }) ico.Add(new Vec3D(0, a, b * phi));
        foreach (double a in new[] { -1.0, 1.0 })
            foreach (double b in new[] { -1.0, 1.0 }) ico.Add(new Vec3D(a, b * phi, 0));
        foreach (double a in new[] { -1.0, 1.0 })
            foreach (double b in new[] { -1.0, 1.0 }) ico.Add(new Vec3D(a * phi, 0, b));

        double shortest = double.PositiveInfinity;
        for (int a = 0; a < ico.Count; a++)
            for (int b = a + 1; b < ico.Count; b++)
                shortest = Math.Min(shortest, (ico[a] - ico[b]).LengthSquared());
        var triangles = new List<(int A, int B, int C)>();
        var dual = new List<Vec3D>();
        for (int a = 0; a < ico.Count; a++)
            for (int b = a + 1; b < ico.Count; b++)
                for (int c = b + 1; c < ico.Count; c++)
                    if (Math.Abs((ico[a] - ico[b]).LengthSquared() - shortest) < 1e-8 &&
                        Math.Abs((ico[b] - ico[c]).LengthSquared() - shortest) < 1e-8 &&
                        Math.Abs((ico[c] - ico[a]).LengthSquared() - shortest) < 1e-8)
                    {
                        triangles.Add((a, b, c));
                        dual.Add((ico[a] + ico[b] + ico[c]) / 3);
                    }
        Assert.Equal(20, dual.Count);
        double radius = 0;
        foreach (var point in dual) radius = Math.Max(radius, point.Length());
        for (int i = 0; i < dual.Count; i++) dual[i] *= 8 * scale / radius;

        var faces = new List<AnchorMesh>();
        for (int vertex = 0; vertex < ico.Count; vertex++)
        {
            var ring = new List<Vec3D>();
            for (int face = 0; face < triangles.Count; face++)
                if (triangles[face].A == vertex || triangles[face].B == vertex || triangles[face].C == vertex)
                    ring.Add(dual[face]);
            Assert.Equal(5, ring.Count);
            var center = new Vec3D(0);
            foreach (var point in ring) center += point;
            center /= ring.Count;
            var u = (ring[0] - center).Normalized();
            var v = Vec3DOps.Cross(ico[vertex].Normalized(), u);
            ring.Sort((first, second) =>
            {
                var d1 = first - center;
                var d2 = second - center;
                return Math.Atan2(Vec3DOps.Dot(d1, v), Vec3DOps.Dot(d1, u))
                    .CompareTo(Math.Atan2(Vec3DOps.Dot(d2, v), Vec3DOps.Dot(d2, u)));
            });
            if (pose.HasValue)
                for (int i = 0; i < ring.Count; i++)
                    ring[i] = TransformMath.TransformPoint(pose.Value, ring[i]);
            faces.Add(api.CreateFromTriangles(ring,
                new List<Tri> { new(0, 1, 2), new(0, 2, 3), new(0, 3, 4) },
                $"dodeca_face_{vertex + 1}"));
        }
        if (faceOrderSeed != 0)
        {
            var random = new Random(faceOrderSeed);
            for (int i = faces.Count - 1; i > 0; i--)
            {
                int j = random.Next(i + 1);
                (faces[i], faces[j]) = (faces[j], faces[i]);
            }
        }
        return api.Sew(faces, makeSolid: true, name: "dodecahedron");
    }
}
