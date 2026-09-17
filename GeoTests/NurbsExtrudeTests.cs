using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class NurbsExtrudeTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void Extrude_Cube_AllPatchesHaveNurbs()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 1.0, "cube");

        foreach (var patchName in NurbsPatchValidation.AllNamedPatches(cube))
        {
            Assert.True(cube.surfaceMetaData.TryGetValue(patchName, out var meta), patchName);
            Assert.NotNull(meta.NurbsSurface);
        }
    }

    [Fact]
    public void Extrude_Cube_PatchesAlignWithNurbs()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 1.0, "cube");

        foreach (var patchName in NurbsPatchValidation.AllNamedPatches(cube))
        {
            try
            {
                NurbsPatchValidation.AssertPatchOnNurbsSurface(cube, patchName,
                    distanceTolerance: 0.05, uvTolerance: 0.05, tessellationDeviation: 0.01);
            }
            catch (Exception ex)
            {
                throw new Exception($"Patch '{patchName}' failed: {ex.Message}", ex);
            }
        }
    }

    [Fact]
    public void Extrude_Cylinder_VolumeUnchanged()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cyl = api.CreateCylinder(CoordinateSystem.Default, 1.0, 2.0, 0.01, "cyl");

        double volume = MeshAnalysis.ComputeSignedMeshVolume(cyl.Mesh.Positions, cyl.Mesh.Triangles);
        double expected = Math.PI * 1.0 * 1.0 * 2.0;
        Assert.InRange(volume, expected * 0.98, expected * 1.02);

        foreach (var patchName in NurbsPatchValidation.AllNamedPatches(cyl))
            Assert.NotNull(cyl.surfaceMetaData[patchName].NurbsSurface);
    }

    [Fact]
    public void Curve2DToBSpline_LineMidpoint_MatchesMeshArcLength()
    {
        var frame = CoordinateSystem.Default;
        var line = new Line2D(new Vec2D(-0.5, -0.5), new Vec2D(0.5, -0.5));
        var curve = Geo.NurbsConstruction.Curve2DToBSpline.LineToBSpline(line, frame);

        var midMesh = line.EvaluateVertex(0.5).Position;
        var midNurbs = curve.EvaluateByArcLength(0.5 * curve.TotalArcLength);
        var midMesh3 = frame.PointTo3D(midMesh);

        Assert.InRange((midNurbs - midMesh3).Length(), 0, 1e-6);
    }
}
