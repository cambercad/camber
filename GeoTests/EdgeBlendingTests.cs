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

    /// <summary>
    /// EdgeBlending matches by exact edge-name string. Try common formats: "[A,B]", "[B,A]", optional "_i" suffix.
    /// </summary>
    private static List<string> EdgeNameCandidates(string groupA, string groupB)
    {
        var candidates = new List<string>
        {
            $"[{groupA},{groupB}]",
            $"[{groupB},{groupA}]",
        };
        for (int i = 0; i < 8; i++)
        {
            candidates.Add($"[{groupA},{groupB}]_{i}");
            candidates.Add($"[{groupB},{groupA}]_{i}");
        }
        return candidates;
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
        foreach (var edgeName in EdgeNameCandidates(groupA, groupB))
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
        foreach (var edgeName in EdgeNameCandidates(groupA, groupB))
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
}

