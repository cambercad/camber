using Curves;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public class NurbsLoftTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void Loft_RuledSquare_AllPatchesHaveNurbs()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, 2), cs0.X, cs0.Y, cs0.Z);

        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);

        var sk1 = new PlotterSketcherCoordSys("B", cs1, sq[0]);
        sk1.AppendLine(sq[1].X, sq[1].Y);
        sk1.AppendLine(sq[2].X, sq[2].Y);
        sk1.AppendLine(sq[3].X, sq[3].Y);
        sk1.AppendLine(sq[4].X, sq[4].Y);

        var options = new LoftOptions { Style = LoftStyle.Ruled, ProfileSamplesU = 8, CapEnds = true };
        var loft = api.Loft(new List<PlotterSketcherCoordSys> { sk0, sk1 }, options, "loft", 1e-4);

        foreach (var patchName in NurbsPatchValidation.AllNamedPatches(loft))
            Assert.NotNull(loft.surfaceMetaData[patchName].NurbsSurface);
    }

    [Fact]
    public void Loft_RuledSquare_SideAlignsWithNurbs()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, 2), cs0.X, cs0.Y, cs0.Z);

        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);

        var sk1 = new PlotterSketcherCoordSys("B", cs1, sq[0]);
        sk1.AppendLine(sq[1].X, sq[1].Y);
        sk1.AppendLine(sq[2].X, sq[2].Y);
        sk1.AppendLine(sq[3].X, sq[3].Y);
        sk1.AppendLine(sq[4].X, sq[4].Y);

        var options = new LoftOptions { Style = LoftStyle.Ruled, ProfileSamplesU = 8, CapEnds = true };
        var loft = api.Loft(new List<PlotterSketcherCoordSys> { sk0, sk1 }, options, "loft", 1e-4);

        NurbsPatchValidation.AssertPatchOnNurbsSurface(loft, EntityNaming.LoftSide("loft"),
            distanceTolerance: 0.15, uvTolerance: -1, tessellationDeviation: 0.02);
    }
}
