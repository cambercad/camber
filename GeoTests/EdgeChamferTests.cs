using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class EdgeChamferTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    private static double ComputeMeshVolume(MeshNormalUV mesh) =>
        Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(mesh.Positions, mesh.Triangles));

    private static void AssertVolumeExact(double actual, double expectedExact, string context, double absTol = 5e-5)
    {
        double err = Math.Abs(actual - expectedExact);
        Assert.True(err <= absTol,
            $"{context}: expected exact volume {expectedExact}, got {actual}, |err|={err} (tol={absTol})");
    }

    private static void AssertMeshSolid(MeshNormalUV mesh, string context)
    {
        Assert.True(mesh.Triangles.Count > 0, $"{context}: mesh has no triangles");
        Assert.True(
            MeshAnalysis.AreTrianglesConsistentlyOriented(mesh.Triangles),
            $"{context}: triangle winding is inconsistent");
        Assert.True(
            MeshAnalysis.IsWatertightMesh(mesh.Positions, mesh.Triangles),
            $"{context}: mesh is not watertight");
        Assert.True(
            MeshAnalysis.ComputeSignedMeshVolume(mesh.Positions, mesh.Triangles) > 0,
            $"{context}: signed volume should be positive");
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

    public static string ResolveChamferableEdgeNamePublic(GeoAPI api, AnchorMesh mesh, string groupA, string groupB,
        double chamferDistance, double maxDiscretizationDeviation) =>
        ResolveChamferableEdgeName(api, mesh, groupA, groupB, chamferDistance, maxDiscretizationDeviation);

    private static string ResolveChamferableEdgeName(GeoAPI api, AnchorMesh mesh, string groupA, string groupB,
        double chamferDistance, double maxDiscretizationDeviation)
    {
        Exception? last = null;
        foreach (var edgeName in EdgeNameCandidates(groupA, groupB))
        {
            try
            {
                _ = api.Chamfer(mesh, new List<string> { edgeName }, chamferDistance, maxDiscretizationDeviation,
                    name: "_edgeNameProbe");
                return edgeName;
            }
            catch (Exception ex)
            {
                last = ex;
            }
        }

        throw new Exception(
            $"Could not resolve any chamferable edge between '{groupA}' and '{groupB}'. Last error: {last?.Message}",
            last);
    }

    private static AnchorMesh ChamferFirstMatchingEdgeName(GeoAPI api, AnchorMesh mesh, string groupA, string groupB,
        double chamferDistance, double maxDiscretizationDeviation, string name)
    {
        Exception? last = null;
        foreach (var edgeName in EdgeNameCandidates(groupA, groupB))
        {
            try
            {
                return api.Chamfer(mesh, new List<string> { edgeName }, chamferDistance, maxDiscretizationDeviation, name);
            }
            catch (Exception ex)
            {
                last = ex;
            }
        }

        throw new Exception(
            $"Could not chamfer any candidate edge between '{groupA}' and '{groupB}'. Last error: {last?.Message}",
            last);
    }

    private static (string line3, string line4, string top) CuboidEdgeGroups(AnchorMesh mesh, string prefix)
    {
        return (
            FindFirstGroupName(mesh, n => n.Contains($"{prefix}-Line3")),
            FindFirstGroupName(mesh, n => n.Contains($"{prefix}-Line4")),
            FindFirstGroupName(mesh, n => n.Contains($"{prefix}-ExtrudeTop")));
    }

    [Theory]
    [InlineData(0.25)]
    [InlineData(0.2)]
    [InlineData(0.125)]
    public void ChamferEdges_UnitCube_SingleEdge_MatchesExactVolume(double chamferDistance)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var cube = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 1), "cube");
        var (line3, line4, _) = CuboidEdgeGroups(cube, "cube");

        const double maxDeviation = 1e-5;
        string edge = ResolveChamferableEdgeName(api, cube, line3, line4, chamferDistance, maxDeviation);
        var chamfered = api.Chamfer(cube, new List<string> { edge }, chamferDistance, maxDeviation, "cube_chamfered");

        double expected = ChamferAnalytic.BoxVolumeAfterSingleEdgeChamfer(1.0, edgeLength: 1.0, chamferDistance);
        double actual = ComputeMeshVolume(chamfered.Mesh);

        AssertMeshSolid(chamfered.Mesh, nameof(ChamferEdges_UnitCube_SingleEdge_MatchesExactVolume));
        AssertVolumeExact(actual, expected, $"unit cube single edge d={chamferDistance}");
    }

    [Theory]
    [InlineData(0.25)]
    [InlineData(0.2)]
    public void ChamferEdges_UnitCube_ThreeEdgesAtCorner_MatchesExactVolume(double chamferDistance)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var cube = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 1), "cube");
        var (line3, line4, top) = CuboidEdgeGroups(cube, "cube");

        const double maxDeviation = 1e-5;
        string e34 = ResolveChamferableEdgeName(api, cube, line3, line4, chamferDistance, maxDeviation);
        string e3t = ResolveChamferableEdgeName(api, cube, line3, top, chamferDistance, maxDeviation);
        string e4t = ResolveChamferableEdgeName(api, cube, line4, top, chamferDistance, maxDeviation);

        var chamfered = api.Chamfer(cube, new List<string> { e34, e3t, e4t }, chamferDistance, maxDeviation,
            "cube_corner_chamfered");

        double expected = ChamferAnalytic.CubeVolumeAfterThreeEdgeCornerChamfer(1.0, chamferDistance);
        double actual = ComputeMeshVolume(chamfered.Mesh);

        AssertMeshSolid(chamfered.Mesh, nameof(ChamferEdges_UnitCube_ThreeEdgesAtCorner_MatchesExactVolume));
        AssertVolumeExact(actual, expected, $"unit cube three-edge corner d={chamferDistance}");
    }

    [Theory]
    [InlineData(0.25)]
    [InlineData(0.2)]
    public void ChamferEdges_Cuboid1x1x3_SingleVerticalEdge_MatchesExactVolume(double chamferDistance)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var box = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 3), "box");
        var (line3, line4, _) = CuboidEdgeGroups(box, "box");

        const double maxDeviation = 1e-5;
        string edge = ResolveChamferableEdgeName(api, box, line3, line4, chamferDistance, maxDeviation);
        var chamfered = api.Chamfer(box, new List<string> { edge }, chamferDistance, maxDeviation, "box_chamfered");

        double expected = ChamferAnalytic.BoxVolumeAfterSingleEdgeChamfer(3.0, edgeLength: 3.0, chamferDistance);
        double actual = ComputeMeshVolume(chamfered.Mesh);

        AssertMeshSolid(chamfered.Mesh, nameof(ChamferEdges_Cuboid1x1x3_SingleVerticalEdge_MatchesExactVolume));
        AssertVolumeExact(actual, expected, $"1x1x3 cuboid single edge d={chamferDistance}");
    }

    [Theory]
    [InlineData(0.25)]
    [InlineData(0.2)]
    public void ChamferEdges_Cuboid1x1x3_ThreeEdgesAtCorner_MatchesExactVolume(double chamferDistance)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var box = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 3), "box");
        var (line3, line4, top) = CuboidEdgeGroups(box, "box");

        const double maxDeviation = 1e-5;
        string e34 = ResolveChamferableEdgeName(api, box, line3, line4, chamferDistance, maxDeviation);
        string e3t = ResolveChamferableEdgeName(api, box, line3, top, chamferDistance, maxDeviation);
        string e4t = ResolveChamferableEdgeName(api, box, line4, top, chamferDistance, maxDeviation);

        var chamfered = api.Chamfer(box, new List<string> { e34, e3t, e4t }, chamferDistance, maxDeviation,
            "box_corner_chamfered");

        double expected = ChamferAnalytic.CuboidCornerVolumeAfterThreeEdgeChamfer(
            shortSide: 1.0, longSide: 3.0, chamferDistance, boxVolume: 3.0);
        double actual = ComputeMeshVolume(chamfered.Mesh);

        AssertMeshSolid(chamfered.Mesh, nameof(ChamferEdges_Cuboid1x1x3_ThreeEdgesAtCorner_MatchesExactVolume));
        AssertVolumeExact(actual, expected, $"1x1x3 cuboid three-edge corner d={chamferDistance}");
    }

    [Fact]
    public void ChamferEdges_Cuboid_SingleEdge_VolumeConvergesWithFinerTessellation()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var box = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 3), "box");
        var (line3, line4, _) = CuboidEdgeGroups(box, "box");

        const double chamferDistance = 0.2;
        double expected = ChamferAnalytic.BoxVolumeAfterSingleEdgeChamfer(3.0, edgeLength: 3.0, chamferDistance);

        string edge = ResolveChamferableEdgeName(api, box, line3, line4, chamferDistance, 1e-4);
        var coarse = api.Chamfer(box, new List<string> { edge }, chamferDistance, maxDeviation: 1e-4, "coarse");
        var fine = api.Chamfer(box, new List<string> { edge }, chamferDistance, maxDeviation: 1e-5, "fine");

        double volCoarse = ComputeMeshVolume(coarse.Mesh);
        double volFine = ComputeMeshVolume(fine.Mesh);

        AssertVolumeExact(volFine, expected, "fine tessellation");
        Assert.True(Math.Abs(volCoarse - volFine) < 2e-4,
            $"coarse/fine volumes should converge: coarse={volCoarse}, fine={volFine}");
    }

    [Fact]
    public void ChamferEdges_UnionOfCylinders_RemovesMaterialAndStaysWatertight()
    {
        // Curved seam — no closed-form volume; check solid invariants and monotonic material removal.
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);

        var a = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 0)), 0.5, 1, 0.0001, "a");
        var b = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 0)), 0.25, 2, 0.0001, "b");
        var union = api.Boolean(a, b, BooleanOp.Union, "c");

        double volBefore = ComputeMeshVolume(union.Mesh);

        string aTop = FindFirstGroupName(union, n => n.Contains("a-ExtrudeTop"));
        string bCircle1 = FindFirstGroupName(union, n => n.Contains("b-Circle1"));

        var chamfered = ChamferFirstMatchingEdgeName(api, union, aTop, bCircle1,
            chamferDistance: 0.1, maxDiscretizationDeviation: 1e-4, name: "c_chamfered");

        double volAfter = ComputeMeshVolume(chamfered.Mesh);

        AssertMeshSolid(chamfered.Mesh, nameof(ChamferEdges_UnionOfCylinders_RemovesMaterialAndStaysWatertight));
        Assert.True(Math.Abs(volAfter - volBefore) > 1e-6, "chamfer should change the solid volume");
        Assert.True(volAfter > 0.9 * volBefore && volAfter < 1.1 * volBefore,
            $"small chamfer on a curved union seam should only slightly change volume (before={volBefore}, after={volAfter})");
    }
}
