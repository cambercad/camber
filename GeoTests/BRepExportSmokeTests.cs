using CSG;
using Curves;
using Geo;
using Geo.BRep;
using GeoCore;

namespace GeoTests;

/// <summary>Broader export smoke tests beyond the unit cube.</summary>
public class BRepExportSmokeTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    private static (int faces, int exportable, string stepPath, string igesPath) ExportAndInspect(GeoAPI api, AnchorMesh mesh, string label)
    {
        var solid = BRepAssembler.BuildFromAnchorMesh(mesh);
        int exportable = solid.Shell.Faces.Count(f => f.TessellationSurface != null);
        string stepPath = Path.Combine(Path.GetTempPath(), $"csg_{label}_{Guid.NewGuid():N}.stp");
        string igesPath = Path.Combine(Path.GetTempPath(), $"csg_{label}_{Guid.NewGuid():N}.igs");
        api.SaveStepFile(mesh, stepPath);
        mesh.EnsureCoplanarPostProcessed();
        Geo.Export.IgesExporter.Export(mesh, igesPath);
        return (solid.Shell.Faces.Count, exportable, stepPath, igesPath);
    }

    [Fact]
    public void Export_Cylinder_HasCapAndSideFaces()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cyl = api.CreateCylinder(CoordinateSystem.Default, 1.0, 2.0, 0.01, "cyl");
        var (faces, exportable, step, iges) = ExportAndInspect(api, cyl, "cyl");
        try
        {
            Assert.True(faces >= 3, $"expected cap+side faces, got {faces}");
            Assert.True(exportable >= 3);
            Assert.True(new FileInfo(step).Length > 500);
            Assert.True(new FileInfo(iges).Length > 200);
            string stepText = File.ReadAllText(step);
            Assert.Contains("MANIFOLD_SOLID_BREP", stepText);
        }
        finally
        {
            if (File.Exists(step)) File.Delete(step);
            if (File.Exists(iges)) File.Delete(iges);
        }
    }

    [Fact]
    public void Export_BooleanUnion_OverlappingCubes_ProducesFaces()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var a = api.CreateCube(CoordinateSystem.Default, 1.0, "a");
        var b = api.CreateCube(new CoordinateSystem(new Vec3D(0.3, 0, 0)), 1.0, "b");
        var union = api.Boolean(a, b, BooleanOp.Union, "union");
        var (faces, exportable, step, iges) = ExportAndInspect(api, union, "union");
        try
        {
            Assert.True(faces > 0);
            Assert.True(exportable > 0);
            Assert.True(File.Exists(step));
            Assert.True(File.Exists(iges));
        }
        finally
        {
            if (File.Exists(step)) File.Delete(step);
            if (File.Exists(iges)) File.Delete(iges);
        }
    }

    [Fact]
    public void Export_BooleanDifference_CubeMinusCylinder_ProducesFaces()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 2.0, "cube");
        var cyl = api.CreateCylinder(new CoordinateSystem(new Vec3D(1, 1, -0.5)), 0.4, 3.0, 0.01, "cyl");
        var diff = api.Boolean(cube, cyl, BooleanOp.Difference, "diff");
        var solid = BRepAssembler.BuildFromAnchorMesh(diff);
        Assert.True(solid.Shell.Faces.Count > 0);
        string step = Path.Combine(Path.GetTempPath(), $"csg_diff_{Guid.NewGuid():N}.stp");
        try
        {
            api.SaveStepFile(diff, step);
            Assert.True(new FileInfo(step).Length > 500);
        }
        finally
        {
            if (File.Exists(step)) File.Delete(step);
        }
    }

    [Fact]
    public void Export_RuledLoft_ProducesStepFile()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var sk0 = new PlotterSketcherCoordSys("s0", CoordinateSystem.Default);
        sk0.AddLine(new Vec2D(0, 0), new Vec2D(1, 0));
        sk0.AddLine(new Vec2D(1, 0), new Vec2D(1, 1));
        sk0.AddLine(new Vec2D(1, 1), new Vec2D(0, 1));
        sk0.AddLine(new Vec2D(0, 1), new Vec2D(0, 0));
        var sk1 = new PlotterSketcherCoordSys("s1", new CoordinateSystem(new Vec3D(0, 0, 2)));
        sk1.AddLine(new Vec2D(0, 0), new Vec2D(1, 0));
        sk1.AddLine(new Vec2D(1, 0), new Vec2D(1, 1));
        sk1.AddLine(new Vec2D(1, 1), new Vec2D(0, 1));
        sk1.AddLine(new Vec2D(0, 1), new Vec2D(0, 0));
        var loft = api.Loft(new List<PlotterSketcherCoordSys> { sk0, sk1 },
            new LoftOptions { Style = LoftStyle.Ruled, ProfileSamplesU = 8, CapEnds = true }, "loft", 1e-4);
        var solid = BRepAssembler.BuildFromAnchorMesh(loft);
        string path = Path.Combine(Path.GetTempPath(), $"csg_loft_{Guid.NewGuid():N}.stp");
        try
        {
            Assert.True(solid.Shell.Faces.Count > 0);
            api.SaveStepFile(loft, path);
            Assert.True(new FileInfo(path).Length > 500);
        }
        finally
        {
            if (File.Exists(path)) File.Delete(path);
        }
    }

    [Fact]
    public void Export_RevolveCylinder_ParsesAsStep()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var rev = api.CreateCylinderRevolve(CoordinateSystem.Default, 1.0, 2.0, 0.01, "rev");
        string path = TempPath(".stp");
        try
        {
            api.SaveStepFile(rev, path);
            BRepExportTestHelper.AssertStepFileWritten(path);
        }
        finally
        {
            if (File.Exists(path)) File.Delete(path);
        }
    }

    [Fact]
    public void Export_BlendedCuboid_StillWritesStepForAnalyticFaces()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 3), "a");
        string line3 = a.extendedNameToGroupId.Keys.First(k => k.Contains("a-Line3"));
        string line4 = a.extendedNameToGroupId.Keys.First(k => k.Contains("a-Line4"));
        string top = a.extendedNameToGroupId.Keys.First(k => k.Contains("a-ExtrudeTop"));
        string e34 = EdgeBlendingTests.ResolveBlendableEdgeNamePublic(api, a, line3, line4, 0.2, 0.001);
        string e3t = EdgeBlendingTests.ResolveBlendableEdgeNamePublic(api, a, line3, top, 0.2, 0.001);
        var blended = api.Fillet(a, new List<string> { e34, e3t }, 0.2, 0.001, "blended");
        var solid = BRepAssembler.BuildFromAnchorMesh(blended);
        Assert.True(solid.Shell.Faces.Count > 0);
        string path = TempPath(".stp");
        try
        {
            api.SaveStepFile(blended, path);
            Assert.True(new FileInfo(path).Length > 500);
            BRepExportTestHelper.AssertStepFileValid(path, minFaces: 1);
        }
        finally
        {
            if (File.Exists(path)) File.Delete(path);
        }
    }

    private static string TempPath(string ext) =>
        Path.Combine(Path.GetTempPath(), $"csg_smoke_{Guid.NewGuid():N}{ext}");

    [Fact]
    public void Export_StepContainsTrimTopology_NotJustSurfaces()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 1.0, "cube");
        string path = Path.Combine(Path.GetTempPath(), $"csg_topo_{Guid.NewGuid():N}.stp");
        try
        {
            api.SaveStepFile(cube, path);
            string text = File.ReadAllText(path);
            Assert.Contains("ADVANCED_FACE", text);
            Assert.Contains("EDGE_LOOP", text);
            Assert.Contains("FACE_OUTER_BOUND", text);
            Assert.Contains("EDGE_CURVE", text);
        }
        finally
        {
            if (File.Exists(path)) File.Delete(path);
        }
    }
}
