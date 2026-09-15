using Curves;
using Geo;
using Geo.BRep;
using Geo.NurbsConstruction;
using GeoCore;
using GeoMeta;
using NURBS;

namespace GeoTests;

public class NurbsLazySurfaceTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void CreateCube_MetadataRegisteredButNotMaterialized()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 1.0, "cube");

        foreach (var patchName in NurbsPatchValidation.AllNamedPatches(cube))
        {
            var meta = cube.surfaceMetaData[patchName];
            Assert.True(meta.HasNurbs, patchName);
            Assert.False(meta.IsNurbsMaterialized, patchName);
        }
    }

    [Fact]
    public void LazyMetadata_MaterializesOnceAndCaches()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 1.0, "cube");
        string patch = NurbsPatchValidation.AllNamedPatches(cube).First();
        var meta = cube.surfaceMetaData[patch];

        var first = meta.NurbsSurface;
        Assert.NotNull(first);
        Assert.True(meta.IsNurbsMaterialized);
        Assert.Same(first, meta.NurbsSurface);
    }

    [Fact]
    public void BuildSweepStripMetadata_NotMaterializedUntilAccessed()
    {
        var (curveMeta, contour, names, guideSegments, guideNames, profileFrame) =
            NurbsPerformanceTestsHelper.BuildSweepStripFixture(4, 3, 12);

        var result = NurbsPatchMetadataBuilder.BuildSweepStripMetadata(
            curveMeta, profileFrame, guideSegments, guideNames, contour, names,
            operationName: "lazy", hasEndCaps: false, twistRatePerExtrudeDistance: 0);

        Assert.All(result.Values, meta =>
        {
            Assert.True(meta.HasNurbs);
            Assert.False(meta.IsNurbsMaterialized);
        });

        var any = result.Values.First();
        Assert.NotNull(any.NurbsSurface);
        Assert.True(any.IsNurbsMaterialized);
    }

    [Fact]
    public void Clone_BeforeMaterialize_IndependentLazyInstances()
    {
        var meta = new SurfaceMetaData(SurfaceType.Planar, () => new BSplinePlane(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0)));

        var clone = meta.Clone();
        Assert.False(meta.IsNurbsMaterialized);
        Assert.False(clone.IsNurbsMaterialized);

        var a = meta.NurbsSurface;
        var b = clone.NurbsSurface;
        Assert.NotNull(a);
        Assert.NotNull(b);
        Assert.NotSame(a, b);
    }

    [Fact]
    public void Clone_AfterMaterialize_SharesBuiltSurface()
    {
        var meta = new SurfaceMetaData(SurfaceType.Planar, () => new BSplinePlane(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0)));

        var built = meta.NurbsSurface;
        var clone = meta.Clone();
        Assert.Same(built, clone.NurbsSurface);
    }

    [Fact]
    public void BRepExport_MaterializesLazySurfaces()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 1.0, "cube");
        Assert.All(cube.surfaceMetaData.Values.Where(m => m.HasNurbs), m => Assert.False(m.IsNurbsMaterialized));

        var solid = BRepAssembler.BuildFromAnchorMesh(cube);
        Assert.True(solid.Shell.Faces.Count(f => f.Surface != null) >= 4);
        Assert.All(cube.surfaceMetaData.Values.Where(m => m.HasNurbs), m => Assert.True(m.IsNurbsMaterialized));
    }

    [Fact]
    public void LazyExtrude_StillAlignsWithMeshAfterMaterialize()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 1.0, "cube");

        foreach (var patchName in NurbsPatchValidation.AllNamedPatches(cube))
        {
            NurbsPatchValidation.AssertPatchOnNurbsSurface(cube, patchName,
                distanceTolerance: 0.05, uvTolerance: 0.05, tessellationDeviation: 0.01);
        }
    }

    [Fact]
    public void LoftMetadata_LazyUntilAccessed()
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

        string side = EntityNaming.LoftSide("loft");
        var sideMeta = loft.surfaceMetaData[side];
        Assert.True(sideMeta.HasNurbs);
        Assert.False(sideMeta.IsNurbsMaterialized);

        Assert.NotNull(sideMeta.NurbsSurface);
        Assert.True(sideMeta.IsNurbsMaterialized);
    }
}

/// <summary>Shared fixture builder for lazy/performance tests.</summary>
internal static class NurbsPerformanceTestsHelper
{
    public static (
        Dictionary<string, CurveMetaData> curveMeta,
        List<List<List<Vec2D>>> contour,
        List<List<string>> names,
        List<IReadOnlyList<CoordinateSystem>> guideSegments,
        List<string> guideNames,
        CoordinateSystem profileFrame) BuildSweepStripFixture(
            int numSegments,
            int numGuideSegments,
            int framesPerGuideSegment)
    {
        var curveMeta = new Dictionary<string, CurveMetaData>();
        var loops = new List<List<Vec2D>>();
        var segmentNames = new List<string>();
        double side = 1.0;
        for (int i = 0; i < numSegments; i++)
        {
            double a0 = 2 * Math.PI * i / numSegments;
            double a1 = 2 * Math.PI * (i + 1) / numSegments;
            var p0 = new Vec2D(0.5 * side * Math.Cos(a0), 0.5 * side * Math.Sin(a0));
            var p1 = new Vec2D(0.5 * side * Math.Cos(a1), 0.5 * side * Math.Sin(a1));
            string segName = $"seg{i}";
            loops.Add(new List<Vec2D> { p0, p1 });
            segmentNames.Add(segName);
            curveMeta[segName] = new CurveMetaData(CurveType.Line2D, new Line2D(p0, p1));
        }

        var contour = new List<List<List<Vec2D>>> { loops };
        var names = new List<List<string>> { segmentNames };

        var guideSegments = new List<IReadOnlyList<CoordinateSystem>>();
        var guideNames = new List<string>();
        int totalFrames = numGuideSegments * framesPerGuideSegment;
        for (int g = 0; g < numGuideSegments; g++)
        {
            var segmentFrames = new List<CoordinateSystem>(framesPerGuideSegment);
            for (int f = 0; f < framesPerGuideSegment; f++)
            {
                int index = g * framesPerGuideSegment + f;
                double t = index / (double)(totalFrames - 1);
                segmentFrames.Add(new CoordinateSystem(
                    new Vec3D(t * 3, Math.Sin(t * 2 * Math.PI), 0),
                    new Vec3D(1, 0, 0),
                    new Vec3D(0, 1, 0),
                    new Vec3D(0, 0, 1)));
            }
            guideSegments.Add(segmentFrames);
            guideNames.Add($"guide{g}");
        }

        return (curveMeta, contour, names, guideSegments, guideNames, new CoordinateSystem());
    }
}
