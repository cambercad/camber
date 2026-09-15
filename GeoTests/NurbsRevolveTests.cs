using Geo;
using GeoCore;

namespace GeoTests;

public class NurbsRevolveTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void Revolve_CylinderRevolve_AllSidePatchesHaveNurbs()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cyl = api.CreateCylinderRevolve(CoordinateSystem.Default, 1.0, 2.0, 0.05, "cylRev");

        foreach (var patchName in NurbsPatchValidation.AllNamedPatches(cyl))
            Assert.NotNull(cyl.surfaceMetaData[patchName].NurbsSurface);
    }

    [Fact]
    public void Revolve_CylinderRevolve_SidePatchesAlignWithNurbs()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cyl = api.CreateCylinderRevolve(CoordinateSystem.Default, 1.0, 2.0, 0.05, "cylRev");

        foreach (var patchName in NurbsPatchValidation.AllNamedPatches(cyl))
        {
            if (!cyl.surfaceMetaData[patchName].HasNurbs)
                continue;
            NurbsPatchValidation.AssertPatchOnNurbsSurface(cyl, patchName,
                distanceTolerance: 0.15, uvTolerance: -1, tessellationDeviation: 0.05);
        }
    }
}
