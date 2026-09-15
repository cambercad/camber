using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public class RevolverTests
{
    [Fact]
    public void FullRevolve_OpenContour_SphereShape_IsWatertight()
    {
        // Semicircle profile with endpoints on axis (Y=0) -> sphere-like shape
        var profile = MeshTestHelpers.MakeSemicircleProfile(16, 1.0);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            profile, 2 * Math.PI,
            triangles, vertices, normals, uv, groups, maxError: 0.1);

        Assert.True(numGroups >= 1);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);

        // Sphere of radius 1 should be bounded by [-1, 1] in all axes
        MeshTestHelpers.AssertBoundingBox(vertices,
            new Vec3D(-1, -1, -1), new Vec3D(1, 1, 1), 0.15);
    }

    [Fact]
    public void FullRevolve_ClosedContour_TorusShape_IsWatertight()
    {
        // Circle profile offset from axis -> torus
        // In the revolver, X = along axis, Y = distance from axis. A torus needs Y > 0 everywhere.
        // Circle at (0, 3) in profile plane: centered on axis at X=0, radius from axis = 3, tube radius = 0.5
        var profile = MeshTestHelpers.MakeCircleProfile(16, 0.5, centerX: 0.0, centerY: 3.0);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            profile, 2 * Math.PI,
            triangles, vertices, normals, uv, groups, maxError: 0.1);

        Assert.True(numGroups >= 1);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);

        // Torus with major radius 3, minor radius 0.5: X in [-0.5, 0.5], Y/Z in [-3.5, 3.5]
        MeshTestHelpers.AssertBoundingBox(vertices,
            new Vec3D(-0.5, -3.5, -3.5), new Vec3D(0.5, 3.5, 3.5), 0.2);
    }

    [Fact]
    public void FullRevolve_MultiSegmentProfile_IsWatertight()
    {
        // Two-segment profile: straight line + arc, endpoints on axis
        var profile1 = new List<Vec2D>
        {
            new Vec2D(-1, 0),
            new Vec2D(-1, 1),
            new Vec2D(0, 1.5)
        };
        var profile2 = new List<Vec2D>
        {
            new Vec2D(0, 1.5),
            new Vec2D(1, 1),
            new Vec2D(1, 0)
        };

        var profiles = new List<List<Vec2D>> { profile1, profile2 };
        var profileNormals = new List<List<Vec2D>>
        {
            Extruder.WeightedNormals2D(profile1),
            Extruder.WeightedNormals2D(profile2)
        };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            profiles, profileNormals, 2 * Math.PI,
            triangles, vertices, normals, uv, groups, maxError: 0.1);

        Assert.Equal(2, numGroups);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    [Fact]
    public void FullRevolve_NamedGroups_ContainsExpectedNames()
    {
        var profile = MeshTestHelpers.MakeSemicircleProfile(8, 1.0);
        var profiles = new List<List<Vec2D>> { profile };
        var profileNormals = new List<List<Vec2D>> { Extruder.WeightedNormals2D(profile) };
        var profileNames = new List<string> { "ArcProfile" };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            profiles, profileNormals, 2 * Math.PI,
            triangles, vertices, normals, uv, groups, maxError: 0.1,
            profileNames, "MySphere", out var groupToName);

        Assert.True(numGroups >= 1);
        Assert.Contains("MySphere-ArcProfile", groupToName.Values);
    }

    [Fact]
    public void Revolve_InvalidAngle_Throws()
    {
        var profile = MeshTestHelpers.MakeSemicircleProfile(8, 1.0);
        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        Assert.Throws<ArgumentException>(() =>
            Revolver.GenerateRevolvedMesh(profile, 0,
                triangles, vertices, normals, uv, groups, maxError: 0.1));

        Assert.Throws<ArgumentException>(() =>
            Revolver.GenerateRevolvedMesh(profile, 3 * Math.PI,
                triangles, vertices, normals, uv, groups, maxError: 0.1));
    }

    // --- Partial revolve tests ---

    [Theory]
    [InlineData(Math.PI / 2)]   // 90 degrees
    [InlineData(Math.PI)]       // 180 degrees
    [InlineData(3 * Math.PI / 2)] // 270 degrees
    public void PartialRevolve_OpenContour_IsWatertight(double angle)
    {
        var profile = MeshTestHelpers.MakeSemicircleProfile(12, 1.0);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            profile, angle,
            triangles, vertices, normals, uv, groups, maxError: 0.05);

        // Should have 1 side group + 2 cap groups
        Assert.Equal(3, numGroups);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    [Theory]
    [InlineData(Math.PI / 2)]
    [InlineData(Math.PI)]
    [InlineData(3 * Math.PI / 2)]
    public void PartialRevolve_ClosedContour_IsWatertight(double angle)
    {
        // Closed contour: circle at (0, 3) with radius 0.5 (doesn't touch axis)
        var profile = MeshTestHelpers.MakeCircleProfile(12, 0.5, centerX: 0.0, centerY: 3.0);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            profile, angle,
            triangles, vertices, normals, uv, groups, maxError: 0.05);

        Assert.Equal(3, numGroups);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    [Fact]
    public void PartialRevolve_90Degrees_OpenContour_BoundingBox()
    {
        var profile = MeshTestHelpers.MakeSemicircleProfile(12, 1.0);

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        Revolver.GenerateRevolvedMesh(
            profile, Math.PI / 2,
            triangles, vertices, normals, uv, groups, maxError: 0.05);

        // Quarter sphere: X in [-1, 1], Y in [0, 1], Z in [0, 1]
        MeshTestHelpers.AssertBoundingBox(vertices,
            new Vec3D(-1, 0, 0), new Vec3D(1, 1, 1), 0.15);
    }

    [Fact]
    public void PartialRevolve_NamedGroups_ContainsCapNames()
    {
        var profile = MeshTestHelpers.MakeSemicircleProfile(8, 1.0);
        var profiles = new List<List<Vec2D>> { profile };
        var profileNormals = new List<List<Vec2D>> { Extruder.WeightedNormals2D(profile) };
        var profileNames = new List<string> { "Profile" };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            profiles, profileNormals, Math.PI,
            triangles, vertices, normals, uv, groups, maxError: 0.1,
            profileNames, "HalfSphere", out var groupToName);

        Assert.Equal(3, numGroups);
        Assert.Contains("HalfSphere-Profile", groupToName.Values);
        Assert.Contains(EntityNaming.RevolveStartCap("HalfSphere"), groupToName.Values);
        Assert.Contains(EntityNaming.RevolveEndCap("HalfSphere"), groupToName.Values);
    }

    [Fact]
    public void PartialRevolve_MultiSegment_OpenContour_IsWatertight()
    {
        // Two-segment open profile, endpoints on axis
        var profile1 = new List<Vec2D>
        {
            new Vec2D(-1, 0),
            new Vec2D(-1, 0.8),
            new Vec2D(0, 1.2)
        };
        var profile2 = new List<Vec2D>
        {
            new Vec2D(0, 1.2),
            new Vec2D(1, 0.8),
            new Vec2D(1, 0)
        };

        var profiles = new List<List<Vec2D>> { profile1, profile2 };
        var profileNormals = new List<List<Vec2D>>
        {
            Extruder.WeightedNormals2D(profile1),
            Extruder.WeightedNormals2D(profile2)
        };

        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            profiles, profileNormals, Math.PI,
            triangles, vertices, normals, uv, groups, maxError: 0.05);

        // 2 side groups + 2 cap groups = 4
        Assert.Equal(4, numGroups);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    [Fact]
    public void FullRevolve_ClosedOuterWithHole_IsWatertightAndMatchesPappusVolume()
    {
        // Meridian annulus: outer rect X∈[-1,1], Y∈[1,2]; hole X∈[-0.5,0.5], Y∈[1.2,1.8].
        // Area A = 2 − 0.6 = 1.4, centroid ȳ = 1.5 → V = 2π·ȳ·A (Pappus).
        var outer = new List<Vec2D>
        {
            new Vec2D(-1, 1),
            new Vec2D(1, 1),
            new Vec2D(1, 2),
            new Vec2D(-1, 2),
            new Vec2D(-1, 1)
        };
        var hole = new List<Vec2D>
        {
            new Vec2D(-0.5, 1.2),
            new Vec2D(-0.5, 1.8),
            new Vec2D(0.5, 1.8),
            new Vec2D(0.5, 1.2),
            new Vec2D(-0.5, 1.2)
        };

        var contours = new List<List<List<Vec2D>>>
        {
            new List<List<Vec2D>> { outer },
            new List<List<Vec2D>> { hole }
        };
        var contourNormals = new List<List<List<Vec2D>>>
        {
            new List<List<Vec2D>> { Extruder.WeightedNormals2D(outer) },
            new List<List<Vec2D>> { Extruder.WeightedNormals2D(hole) }
        };
        var names = new List<List<string>>
        {
            new List<string> { "Outer" },
            new List<string> { "Hole" }
        };

        var converter = MeshTestHelpers.MakeConverter(20);
        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            converter, contours, contourNormals, 2 * Math.PI,
            triangles, vertices, normals, uv, groups, precise, maxError: 0.02,
            names, "AnnulusRev", out var groupToName);

        Assert.Equal(2, numGroups);
        Assert.Contains("AnnulusRev-Outer", groupToName.Values);
        Assert.Contains("AnnulusRev-Hole", groupToName.Values);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);

        double expected = 2.0 * Math.PI * 1.5 * 1.4;
        double vol = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(vertices, triangles));
        Assert.True(Math.Abs(vol - expected) < expected * 0.02,
            $"Volume {vol} vs expected Pappus {expected}");
    }

    [Theory]
    [InlineData(Math.PI / 2)]
    [InlineData(Math.PI)]
    public void PartialRevolve_ClosedOuterWithHole_IsWatertight(double angle)
    {
        var outer = MeshTestHelpers.MakeCircleProfile(16, 1.0, centerX: 0.0, centerY: 3.0);
        var hole = MeshTestHelpers.MakeCircleProfile(12, 0.35, centerX: 0.0, centerY: 3.0);

        var contours = new List<List<List<Vec2D>>>
        {
            new List<List<Vec2D>> { outer },
            new List<List<Vec2D>> { hole }
        };
        var contourNormals = new List<List<List<Vec2D>>>
        {
            new List<List<Vec2D>> { Extruder.WeightedNormals2D(outer) },
            new List<List<Vec2D>> { Extruder.WeightedNormals2D(hole) }
        };
        var names = new List<List<string>>
        {
            new List<string> { "Outer" },
            new List<string> { "Hole" }
        };

        var converter = MeshTestHelpers.MakeConverter(20);
        var triangles = new List<Tri>();
        var vertices = new List<Vec3D>();
        var normals = new List<Vec3D>();
        var uv = new List<Vec2D>();
        var groups = new List<int>();
        var precise = new List<Rat3Hybrid>();

        int numGroups = Revolver.GenerateRevolvedMesh(
            converter, contours, contourNormals, angle,
            triangles, vertices, normals, uv, groups, precise, maxError: 0.05,
            names, "TubeSector", out var groupToName);

        Assert.Equal(4, numGroups); // 2 sides + 2 caps
        Assert.Contains(EntityNaming.RevolveStartCap("TubeSector"), groupToName.Values);
        Assert.Contains(EntityNaming.RevolveEndCap("TubeSector"), groupToName.Values);
        MeshTestHelpers.AssertValidMesh(vertices, triangles);
    }

    [Fact]
    public void GeoApi_Revolve_SketchWithHole_IsWatertightPositiveVolume()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), 1e-4);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "annulus");

        // Outer rectangle in meridian (X along axis, Y = radius)
        sketch.AddLine(new Vec2D(-1, 1), new Vec2D(1, 1));
        sketch.AppendLine(new Vec2D(1, 2));
        sketch.AppendLine(new Vec2D(-1, 2));
        sketch.AppendLine(new Vec2D(-1, 1));

        sketch.MoveToPointAndStartNewCurveStrip(new Vec2D(-0.5, 1.2));
        sketch.AddLine(new Vec2D(-0.5, 1.2), new Vec2D(-0.5, 1.8));
        sketch.AppendLine(new Vec2D(0.5, 1.8));
        sketch.AppendLine(new Vec2D(0.5, 1.2));
        sketch.AppendLine(new Vec2D(-0.5, 1.2));

        var mesh = api.Revolve(sketch, 2 * Math.PI, 0.02, "AnnulusApi");
        MeshTestHelpers.AssertValidMesh(mesh.Mesh.Positions, mesh.Mesh.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(mesh.Mesh.Positions, mesh.Mesh.Triangles);
        Assert.True(vol > 0, $"Expected positive volume, got {vol}");
        double expected = 2.0 * Math.PI * 1.5 * 1.4;
        Assert.True(Math.Abs(vol - expected) < expected * 0.03,
            $"Volume {vol} vs expected Pappus {expected}");
    }
}
