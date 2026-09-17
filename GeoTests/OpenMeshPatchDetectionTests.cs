using Geo;
using GeoCore;

namespace GeoTests;

/// <summary>
/// Angle-based patch detection and auto-normals must work on open (non-watertight) sheets,
/// which is the load path for STL trim surfaces with requireWatertight=False.
/// </summary>
public class OpenMeshPatchDetectionTests
{
    /// <summary>
    /// Flat open quad (2 tris, boundary edges). After vertex dedup this is not watertight.
    /// </summary>
    private static void BuildOpenFlatQuad(out List<Vec3D> points, out List<Tri> triangles)
    {
        // STL-style duplicated vertices (6 corners for 2 tris).
        points = new List<Vec3D>
        {
            new Vec3D(0, 0, 0), new Vec3D(1, 0, 0), new Vec3D(1, 1, 0),
            new Vec3D(0, 0, 0), new Vec3D(1, 1, 0), new Vec3D(0, 1, 0),
        };
        triangles = new List<Tri>
        {
            new Tri(0, 1, 2),
            new Tri(3, 4, 5),
        };
    }

    /// <summary>
    /// Two planar wings meeting at a 90° crease — open sheet, two patches under a 45° threshold.
    /// </summary>
    private static void BuildOpenCreasedSheet(out List<Vec3D> points, out List<Tri> triangles)
    {
        // Wing A in XY (z=0): (0,0,0)-(1,0,0)-(1,1,0)-(0,1,0)
        // Wing B in XZ (y=0): (0,0,0)-(1,0,0)-(1,0,1)-(0,0,1)
        // Shared crease along X at y=z=0.
        points = new List<Vec3D>
        {
            new Vec3D(0, 0, 0), new Vec3D(1, 0, 0), new Vec3D(1, 1, 0),
            new Vec3D(0, 0, 0), new Vec3D(1, 1, 0), new Vec3D(0, 1, 0),

            new Vec3D(0, 0, 0), new Vec3D(1, 0, 0), new Vec3D(1, 0, 1),
            new Vec3D(0, 0, 0), new Vec3D(1, 0, 1), new Vec3D(0, 0, 1),
        };
        triangles = new List<Tri>
        {
            new Tri(0, 1, 2),
            new Tri(3, 4, 5),
            new Tri(6, 7, 8),
            new Tri(9, 10, 11),
        };
    }

    [Fact]
    public void AutoDetectPatches_OpenFlatQuad_SinglePatch()
    {
        BuildOpenFlatQuad(out var points, out var triangles);
        Assert.False(MeshAnalysis.IsWatertightMesh(points, triangles));

        int numGroups = AutoGroups.AutoDetectPatches(
            points, triangles, 25.0.ToRadians(), out var groupPerTriangle, baseGroupIndex: 0);

        Assert.Equal(1, numGroups);
        Assert.Equal(2, groupPerTriangle.Count);
        Assert.Equal(groupPerTriangle[0], groupPerTriangle[1]);
    }

    [Fact]
    public void AutoDetectPatches_OpenCreasedSheet_TwoPatches()
    {
        BuildOpenCreasedSheet(out var points, out var triangles);
        Assert.False(MeshAnalysis.IsWatertightMesh(points, triangles));

        int numGroups = AutoGroups.AutoDetectPatches(
            points, triangles, 45.0.ToRadians(), out var groupPerTriangle, baseGroupIndex: 7);

        Assert.Equal(2, numGroups);
        Assert.All(groupPerTriangle, g => Assert.True(g == 7 || g == 8));
        Assert.Equal(groupPerTriangle[0], groupPerTriangle[1]); // wing A
        Assert.Equal(groupPerTriangle[2], groupPerTriangle[3]); // wing B
        Assert.NotEqual(groupPerTriangle[0], groupPerTriangle[2]);
    }

    [Fact]
    public void AutoNormals_OpenFlatQuad_DoesNotThrow()
    {
        BuildOpenFlatQuad(out var points, out var triangles);
        var normals = AutoNormals.ComputeNormals(points, triangles, 25.0.ToRadians());
        Assert.Equal(triangles.Count * 3, normals.Count);
        Assert.All(normals, n => Assert.True(n.LengthSquared() > 1e-12));
    }

    [Fact]
    public void AutoNormals_OpenCreasedSheet_DoesNotThrow()
    {
        BuildOpenCreasedSheet(out var points, out var triangles);
        var normals = AutoNormals.ComputeNormals(points, triangles, 45.0.ToRadians());
        Assert.Equal(triangles.Count * 3, normals.Count);
        Assert.All(normals, n => Assert.True(n.LengthSquared() > 1e-12));
    }

    [Fact]
    public void LoadStlFile_OpenCreasedSheet_DetectsPatchesAndMarksSurface()
    {
        BuildOpenCreasedSheet(out var points, out var triangles);

        string path = Path.Combine(Path.GetTempPath(), "csg_open_crease_" + Guid.NewGuid().ToString("N") + ".stl");
        try
        {
            StlWriter.WriteAscii(path, points, triangles);

            var api = new GeoAPI(new Box3D(new Vec3D(-1, -1, -1), new Vec3D(2, 2, 2)), 0.01);
            var mesh = api.LoadStlFile(path, 45.0, name: "TrimSheet", requireWatertight: false);

            Assert.False(mesh.IsVolume);
            Assert.Equal(2, mesh.extendedNameToGroupId.Count);
            Assert.Equal(4, mesh.Mesh.Triangles.Count);
        }
        finally
        {
            if (File.Exists(path))
                File.Delete(path);
        }
    }
}
