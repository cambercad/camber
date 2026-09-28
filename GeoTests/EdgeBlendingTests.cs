using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class EdgeBlendingTests
{
    private static double ComputeMeshVolume(MeshNormalUV mesh)
    {
        // Signed tetrahedron volume sum; assumes watertight, consistently oriented mesh.
        double vol6 = 0.0;
        var positions = mesh.Positions;
        var tris = mesh.Triangles;
        for (int i = 0; i < tris.Count; i++)
        {
            var t = tris[i];
            Vec3D a = positions[t.A];
            Vec3D b = positions[t.B];
            Vec3D c = positions[t.C];
            vol6 += Vec3DOps.Dot(a, Vec3DOps.Cross(b, c));
        }
        return Math.Abs(vol6) / 6.0;
    }

    private static void AssertApproximately(double actual, double expected, double relTol = 5e-4, double absTol = 1e-6)
    {
        double tol = Math.Max(absTol, Math.Abs(expected) * relTol);
        Assert.InRange(actual, expected - tol, expected + tol);
    }

    private static string FindFirstGroupName(AnchorMesh mesh, Func<string, bool> predicate)
    {
        foreach (var kv in mesh.groupIdToExtendedName)
        {
            if (predicate(kv.Value))
                return kv.Value;
        }
        throw new Exception("No matching group name found. Groups: " +
                            string.Join(", ", mesh.groupIdToExtendedName.OrderBy(k => k.Key).Select(k => k.Value)));
    }

    private static List<string> EdgeNameCandidates(AnchorMesh mesh, string groupA, string groupB)
    {
        int groupIdA = mesh.extendedNameToGroupId[groupA];
        int groupIdB = mesh.extendedNameToGroupId[groupB];
        var graph = new EdgeGraph(mesh.Mesh.Triangles, mesh.Mesh.GetTriangleGroups(),
            mesh.Mesh.Positions, mesh.Mesh.PrecisionPositions, mesh.groupIdToExtendedName);
        var names = new List<string>();
        foreach (var edge in graph.GetEdgesBetweenGroups(groupIdA, groupIdB))
            names.Add(edge.Name);
        return names;
    }

    public static string ResolveBlendableEdgeNamePublic(GeoAPI api, AnchorMesh mesh, string groupA, string groupB,
        double blendRadius, double maxDiscretizationDeviation) =>
        ResolveBlendableEdgeName(api, mesh, groupA, groupB, blendRadius, maxDiscretizationDeviation);

    /// <summary>
    /// First candidate that successfully blends alone on <paramref name="mesh"/> (input mesh is not modified).
    /// </summary>
    private static string ResolveBlendableEdgeName(GeoAPI api, AnchorMesh mesh, string groupA, string groupB,
        double blendRadius, double maxDiscretizationDeviation)
    {
        Exception? last = null;
        foreach (var edgeName in EdgeNameCandidates(mesh, groupA, groupB))
        {
            try
            {
                _ = api.Fillet(mesh, new List<string> { edgeName }, blendRadius, maxDiscretizationDeviation,
                    name: "_edgeNameProbe");
                return edgeName;
            }
            catch (Exception ex)
            {
                last = ex;
            }
        }

        throw new Exception(
            $"Could not resolve any blendable edge between '{groupA}' and '{groupB}'. Last error: {last?.Message}",
            last);
    }

    private static AnchorMesh BlendFirstMatchingEdgeName(GeoAPI api, AnchorMesh mesh, string groupA, string groupB,
        double blendRadius, double maxDiscretizationDeviation, string name)
    {
        Exception? last = null;
        foreach (var edgeName in EdgeNameCandidates(mesh, groupA, groupB))
        {
            try
            {
                return api.Fillet(mesh, new List<string> { edgeName }, blendRadius, maxDiscretizationDeviation, name);
            }
            catch (Exception ex)
            {
                last = ex;
            }
        }

        throw new Exception(
            $"Could not blend any candidate edge between '{groupA}' and '{groupB}'. Last error: {last?.Message}",
            last);
    }

    [Fact]
    public void BlendEdges_Cuboid_MultipleEdges_VolumeIsStable()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);

        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 3), "a");

        // Note: group naming in the current library uses "Line1..Line4" (old samples used "CurveX").
        string line3 = FindFirstGroupName(a, n => n.Contains("a-Line3"));
        string line4 = FindFirstGroupName(a, n => n.Contains("a-Line4"));
        string top = FindFirstGroupName(a, n => n.Contains("a-ExtrudeTop"));

        // Blend three adjacent edges, like the original MainForm sample intended.
        var b1 = BlendFirstMatchingEdgeName(api, a, line3, line4, blendRadius: 0.5, maxDiscretizationDeviation: 0.0001, name: "tmp1");
        var b2 = BlendFirstMatchingEdgeName(api, b1, line3, top, blendRadius: 0.5, maxDiscretizationDeviation: 0.0001, name: "tmp2");
        var blended = BlendFirstMatchingEdgeName(api, b2, line4, top, blendRadius: 0.5, maxDiscretizationDeviation: 0.0001, name: "a_blended");

        double vol = ComputeMeshVolume(blended.Mesh);

        AssertApproximately(vol, expected: 2.7602458393360045);

        TestVisualization.Visualize(api, nameof(BlendEdges_Cuboid_MultipleEdges_VolumeIsStable));
    }

    [Fact]
    public void BlendEdges_Cuboid_MultipleEdges_SingleCall_VolumeIsStable()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);

        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 3), "a");

        string line3 = FindFirstGroupName(a, n => n.Contains("a-Line3"));
        string line4 = FindFirstGroupName(a, n => n.Contains("a-Line4"));
        string top = FindFirstGroupName(a, n => n.Contains("a-ExtrudeTop"));

        const double blendRadius = 0.5;
        const double maxDiscretizationDeviation = 0.0001;

        string e34 = ResolveBlendableEdgeName(api, a, line3, line4, blendRadius, maxDiscretizationDeviation);
        string e3t = ResolveBlendableEdgeName(api, a, line3, top, blendRadius, maxDiscretizationDeviation);
        string e4t = ResolveBlendableEdgeName(api, a, line4, top, blendRadius, maxDiscretizationDeviation);

        var blended = api.Fillet(a, new List<string> { e34, e3t, e4t }, blendRadius, maxDiscretizationDeviation,
            name: "a_blended_one_call");

        double vol = ComputeMeshVolume(blended.Mesh);

        // Same three edges as BlendEdges_Cuboid_MultipleEdges_VolumeIsStable, but one BlendEdges call:
        // volume differs slightly from the chained path (≈2.760…) due to solver/mesh history.
        AssertApproximately(vol, expected: 2.7524633708004145);

        TestVisualization.Visualize(api, nameof(BlendEdges_Cuboid_MultipleEdges_SingleCall_VolumeIsStable));
    }

    [Fact]
    public void BlendEdges_UnionOfCylinders_VolumeIsStable()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);

        var a = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 0)), 0.5, 1, 0.0001, "a");
        var b = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 0)), 0.25, 2, 0.0001, "b");
        var c = api.Boolean(a, b, BooleanOp.Union, "c");

        string aTop = FindFirstGroupName(c, n => n.Contains("a-ExtrudeTop"));
        string bCircle1 = FindFirstGroupName(c, n => n.Contains("b-Circle1"));

        var blended = BlendFirstMatchingEdgeName(api, c, aTop, bCircle1,
            blendRadius: 0.1, maxDiscretizationDeviation: 0.0001, name: "c_blended");

        double vol = ComputeMeshVolume(blended.Mesh);

        AssertApproximately(vol, expected: 0.98511707282132444);

        TestVisualization.Visualize(api, nameof(BlendEdges_UnionOfCylinders_VolumeIsStable));
    }

    [Fact]
    public void BlendEdges_UnionOfCuboids_VolumeIsStable()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);

        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 3), "a");
        var b = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 3, 1), "b");
        var c = api.Boolean(a, b, BooleanOp.Union, "c");

        string aLine3 = FindFirstGroupName(c, n => n.Contains("a-Line3"));
        string bTop = FindFirstGroupName(c, n => n.Contains("b-ExtrudeTop"));

        var blended = BlendFirstMatchingEdgeName(api, c, aLine3, bTop,
            blendRadius: 0.5, maxDiscretizationDeviation: 0.0001, name: "c_blended");

        double vol = ComputeMeshVolume(blended.Mesh);

        AssertApproximately(vol, expected: 5.0537033992135747);

        TestVisualization.Visualize(api, nameof(BlendEdges_UnionOfCuboids_VolumeIsStable));
    }

    [Fact]
    public void AllThirtyEdgesOfSewnIcosahedronCanBeRoundedAtFiveWayCorners()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .001);
        double phi = (1 + Math.Sqrt(5)) / 2;
        var vertices = new List<Vec3D>();
        foreach (double a in new[] { -1.0, 1.0 })
            foreach (double b in new[] { -1.0, 1.0 }) vertices.Add(new Vec3D(0, a, b * phi));
        foreach (double a in new[] { -1.0, 1.0 })
            foreach (double b in new[] { -1.0, 1.0 }) vertices.Add(new Vec3D(a, b * phi, 0));
        foreach (double a in new[] { -1.0, 1.0 })
            foreach (double b in new[] { -1.0, 1.0 }) vertices.Add(new Vec3D(a * phi, 0, b));
        double scale = 8 / vertices.Max(point => point.Length());
        vertices = vertices.Select(point => point * scale).ToList();

        double shortest = double.PositiveInfinity;
        for (int i = 0; i < vertices.Count; i++)
            for (int j = i + 1; j < vertices.Count; j++)
                shortest = Math.Min(shortest, (vertices[i] - vertices[j]).LengthSquared());
        var faces = new List<AnchorMesh>();
        int faceIndex = 0;
        for (int a = 0; a < vertices.Count; a++)
            for (int b = a + 1; b < vertices.Count; b++)
                for (int c = b + 1; c < vertices.Count; c++)
                {
                    if (Math.Abs((vertices[a] - vertices[b]).LengthSquared() - shortest) > 1e-8 ||
                        Math.Abs((vertices[b] - vertices[c]).LengthSquared() - shortest) > 1e-8 ||
                        Math.Abs((vertices[c] - vertices[a]).LengthSquared() - shortest) > 1e-8) continue;
                    faces.Add(api.CreateFromTriangles(new() { vertices[a], vertices[b], vertices[c] },
                        new() { new Tri(0, 1, 2) }, $"icosa_face_{faceIndex++}"));
                }

        Assert.Equal(20, faces.Count);
        var body = api.Sew(faces, makeSolid: true, name: "icosahedron");
        var graph = new EdgeGraph(body.Mesh.Triangles, body.Mesh.GetTriangleGroups(),
            body.Mesh.Positions, body.Mesh.PrecisionPositions, body.groupIdToExtendedName);
        Assert.Equal(30, graph.Edges.Count);

        var rounded = api.Fillet(body, graph.Edges.Select(edge => edge.Name).ToList(), .1, .001,
            "rounded_icosahedron");

        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));
        double originalVolume = ComputeMeshVolume(body.Mesh);
        double roundedVolume = ComputeMeshVolume(rounded.Mesh);
        Assert.InRange(roundedVolume, originalVolume * .99, originalVolume);
        Assert.Equal(30, rounded.groupIdToExtendedName.Values.Count(name => name.StartsWith(
            "BlendEdge_", StringComparison.Ordinal)));
    }

    [Fact]
    public void AllTwelveEdgesOfSewnOctahedronCanBeRoundedAtFourWayCorners()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .001);
        var vertices = new List<Vec3D>
        {
            new(8, 0, 0), new(-8, 0, 0), new(0, 8, 0),
            new(0, -8, 0), new(0, 0, 8), new(0, 0, -8)
        };
        double shortest = double.PositiveInfinity;
        for (int i = 0; i < vertices.Count; i++)
            for (int j = i + 1; j < vertices.Count; j++)
                shortest = Math.Min(shortest, (vertices[i] - vertices[j]).LengthSquared());

        var faces = new List<AnchorMesh>();
        int faceIndex = 0;
        for (int a = 0; a < vertices.Count; a++)
            for (int b = a + 1; b < vertices.Count; b++)
                for (int c = b + 1; c < vertices.Count; c++)
                {
                    if (Math.Abs((vertices[a] - vertices[b]).LengthSquared() - shortest) > 1e-8 ||
                        Math.Abs((vertices[b] - vertices[c]).LengthSquared() - shortest) > 1e-8 ||
                        Math.Abs((vertices[c] - vertices[a]).LengthSquared() - shortest) > 1e-8) continue;
                    faces.Add(api.CreateFromTriangles(new() { vertices[a], vertices[b], vertices[c] },
                        new() { new Tri(0, 1, 2) }, $"octa_face_{faceIndex++}"));
                }

        Assert.Equal(8, faces.Count);
        var body = api.Sew(faces, makeSolid: true, name: "octahedron");
        var graph = new EdgeGraph(body.Mesh.Triangles, body.Mesh.GetTriangleGroups(),
            body.Mesh.Positions, body.Mesh.PrecisionPositions, body.groupIdToExtendedName);
        Assert.Equal(12, graph.Edges.Count);

        var rounded = api.Fillet(body, graph.Edges.Select(edge => edge.Name).ToList(), .1, .001,
            "rounded_octahedron");

        MeshTestHelpers.AssertValidMesh(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));
        double originalVolume = ComputeMeshVolume(body.Mesh);
        double roundedVolume = ComputeMeshVolume(rounded.Mesh);
        Assert.InRange(roundedVolume, originalVolume * .99, originalVolume);
    }

}

