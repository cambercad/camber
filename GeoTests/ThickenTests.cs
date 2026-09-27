using Geo;
using GeoCore;
using GeoMeta;
using Curves;

namespace GeoTests;

public sealed class ThickenTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void PlanarFaceThickensInEitherDirectionAndOnBothSides()
    {
        var api = Api();
        var block = api.CreateCuboid(new Vec3D(0), new Vec3D(10, 8, 3), "block");
        string top = FaceAtZ(block, 3);
        var face = api.ExtractFaceSurface(block, top, "top_surface");

        var positive = api.Thicken(face, 2, name: "positive");
        var negative = api.Thicken(face, -2, name: "negative");
        var both = api.Thicken(face, 2, bothSides: true, name: "both");

        AssertSolid(positive, 160);
        AssertSolid(negative, 160);
        AssertSolid(both, 320);
        Assert.InRange(positive.Mesh.Positions.Min(p => p.Z), 2.9999, 3.0001);
        Assert.InRange(positive.Mesh.Positions.Max(p => p.Z), 4.9999, 5.0001);
        Assert.InRange(negative.Mesh.Positions.Min(p => p.Z), .9999, 1.0001);
        Assert.InRange(negative.Mesh.Positions.Max(p => p.Z), 2.9999, 3.0001);
        Assert.InRange(both.Mesh.Positions.Min(p => p.Z), .9999, 1.0001);
        Assert.InRange(both.Mesh.Positions.Max(p => p.Z), 4.9999, 5.0001);
        Assert.Contains(EntityNaming.ThickenPositive(top), positive.surfaceMetaData.Keys);
        Assert.Contains(EntityNaming.ThickenNegative(top), positive.surfaceMetaData.Keys);
        Assert.Contains(EntityNaming.ThickenRim(1), positive.surfaceMetaData.Keys);
    }

    [Fact]
    public void CylindricalFaceUsesAnalyticOffsetAndClosesBothCircularBoundaries()
    {
        var api = Api();
        var cylinder = api.CreateCylinder(CoordinateSystem.Default, 5, 7, .005, "cylinder");
        string side = cylinder.surfaceMetaData.Single(pair =>
            pair.Value.SurfaceType == SurfaceType.Cylindrical).Key;
        var surface = api.ExtractFaceSurface(cylinder, side, "cylinder_surface");

        var result = api.Thicken(surface, 1, name: "thick_cylinder");

        Assert.True(result.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(result.Mesh.PrecisionPositions,
            result.Mesh.Triangles));
        double expected = Math.PI * (36 - 25) * 7;
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles),
            expected * .99, expected * 1.01);
        var outer = result.surfaceMetaData[EntityNaming.ThickenPositive(side)].CylinderParams;
        Assert.NotNull(outer);
        Assert.Equal(6, outer.Radius, 8);
        Assert.Equal(2, result.surfaceMetaData.Keys.Count(name =>
            name.StartsWith(EntityNaming.ThickenRimPrefix, StringComparison.Ordinal)));
    }

    [Fact]
    public void CurvedNurbsSheetUsesGeneralMeshOffsetFallback()
    {
        var api = Api();
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (double z in new[] { 0.0, 8.0 })
        {
            var sketch = new PlotterSketcherCoordSys("section" + z,
                new CoordinateSystem(new Vec3D(0, 0, z)), new Vec2D(0));
            sketch.AppendLine(6, 0);
            sections.Add(sketch);
        }
        var surface = api.LoftSurface(sections, startTangent: new Vec3D(0, 5, 8),
            endTangent: new Vec3D(0, -4, 8), name: "curved_sheet", maxDeviation: .03);

        var result = api.Thicken(surface, .4, name: "curved_thickening");

        Assert.True(result.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(result.Mesh.PrecisionPositions,
            result.Mesh.Triangles));
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.PrecisionPositions,
            result.Mesh.Triangles).Sign() > 0);
    }

    [Fact]
    public void RejectsSolidsZeroThicknessAndCollapsedAnalyticOffsets()
    {
        var api = Api();
        var cylinder = api.CreateCylinder(CoordinateSystem.Default, 2, 4, .01, "cylinder");
        Assert.Throws<ArgumentException>(() => api.Thicken(cylinder, 1));
        string side = cylinder.surfaceMetaData.Single(pair =>
            pair.Value.SurfaceType == SurfaceType.Cylindrical).Key;
        var surface = api.ExtractFaceSurface(cylinder, side);
        Assert.Throws<ArgumentOutOfRangeException>(() => api.Thicken(surface, 0));
        Assert.Throws<ArgumentException>(() => api.Thicken(surface, -2));
    }

    private static GeoAPI Api() => new(new Box3D(new Vec3D(-20), new Vec3D(30)), 1e-4);

    private static string FaceAtZ(AnchorMesh mesh, double z)
    {
        for (int i = 0; i < mesh.Mesh.Triangles.Count; i++)
        {
            Tri triangle = mesh.Mesh.Triangles[i];
            if (Math.Abs(mesh.Mesh.Positions[triangle.A].Z - z) < 1e-8 &&
                Math.Abs(mesh.Mesh.Positions[triangle.B].Z - z) < 1e-8 &&
                Math.Abs(mesh.Mesh.Positions[triangle.C].Z - z) < 1e-8)
                return mesh.groupIdToExtendedName[mesh.Mesh.TrianglesEx[i].GroupId];
        }
        throw new InvalidOperationException("Face not found.");
    }

    private static void AssertSolid(AnchorMesh mesh, double expectedVolume)
    {
        Assert.True(mesh.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(mesh.Mesh.PrecisionPositions, mesh.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(mesh.Mesh.PrecisionPositions,
            mesh.Mesh.Triangles));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(mesh.Mesh.Positions, mesh.Mesh.Triangles),
            expectedVolume - .001, expectedVolume + .001);
    }
}
