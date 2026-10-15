using Curves;
using Curves.Base;
using Geo;
using GeoCore;
using GeoMeta;
using GeoSolver;
using GeoSolver.Sketcher;

namespace GeoTests;

public class UsdaSvgExchangeTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void UsdaRoundTrip_KeepsVolumeNormalsAndSixPatches()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 1.0, "cube");
        string path = Path.Combine(Path.GetTempPath(), "camber_usda_" + Guid.NewGuid().ToString("N") + ".usda");
        try
        {
            api.SaveUsdaFile(cube, path);
            var loaded = api.LoadUsdaFile(path, name: "roundtrip");
            Assert.True(loaded.IsVolume);
            loaded.Mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out _);
            Assert.Equal(positions.Count, normals.Count);
            Assert.Equal(positions.Count, uvs.Count);
            Assert.Equal(cube.Mesh.Triangles.Count, triangles.Count);
            double volume = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(positions, triangles));
            Assert.InRange(volume, 0.99, 1.01);
            Assert.Equal(6, loaded.groupIdToExtendedName.Count);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void UsdaQuad_IsTriangulatedAndNamedFromSubset()
    {
        string path = Path.Combine(Path.GetTempPath(), "camber_quad_" + Guid.NewGuid().ToString("N") + ".usda");
        try
        {
            File.WriteAllText(path, """
                #usda 1.0
                (
                    upAxis = "Z"
                    metersPerUnit = 1
                )
                def Mesh "plate"
                {
                    int[] faceVertexCounts = [4]
                    int[] faceVertexIndices = [0, 1, 2, 3]
                    point3f[] points = [(0, 0, 0), (2, 0, 0), (2, 1, 0), (0, 1, 0)]
                    def GeomSubset "top"
                    {
                        uniform token elementType = "face"
                        int[] indices = [0]
                    }
                }
                """);
            var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(4)), 1e-4);
            var mesh = api.LoadUsdaFile(path, name: "plate");
            Assert.False(mesh.IsVolume);
            Assert.Equal(2, mesh.Mesh.Triangles.Count);
            Assert.Contains("top", mesh.groupIdToExtendedName.Values);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void UsdaConcaveNgon_KeepsWindingAndArea()
    {
        string path = Path.Combine(Path.GetTempPath(), "camber_ngon_" + Guid.NewGuid().ToString("N") + ".usda");
        try
        {
            File.WriteAllText(path, """
                #usda 1.0
                def Mesh "notch"
                {
                    int[] faceVertexCounts = [5]
                    int[] faceVertexIndices = [0, 1, 2, 3, 4]
                    point3f[] points = [(0, 0, 0), (2, 0, 0), (0.5, 0.5, 0), (2, 1, 0), (0, 1, 0)]
                }
                """);
            var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(4)), 1e-4);
            var mesh = api.LoadUsdaFile(path, name: "notch");
            mesh.Mesh.Decompose(out var points, out _, out _, out var triangles, out _);
            Assert.Equal(3, triangles.Count);
            double area = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(points, triangles));
            // Flat in Z, so volume is 0. Use the projected polygon area instead.
            double projected = 0;
            foreach (Tri tri in triangles)
            {
                Vec3D a = points[tri.A], b = points[tri.B], c = points[tri.C];
                projected += (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
            }
            projected = Math.Abs(projected) * 0.5;
            Assert.InRange(projected, 1.2, 1.3);
            Assert.Equal(0, area, 6);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void UsdaYUpAndMeters_ConvertToZUpMetres()
    {
        string path = Path.Combine(Path.GetTempPath(), "camber_yup_" + Guid.NewGuid().ToString("N") + ".usda");
        try
        {
            File.WriteAllText(path, """
                #usda 1.0
                (
                    upAxis = "Y"
                    metersPerUnit = 0.001
                )
                def Mesh "tri"
                {
                    int[] faceVertexCounts = [3]
                    int[] faceVertexIndices = [0, 1, 2]
                    point3f[] points = [(0, 0, 0), (1000, 0, 0), (0, 2000, 0)]
                }
                """);
            var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(4)), 1e-4);
            var mesh = api.LoadUsdaFile(path, name: "tri");
            mesh.Mesh.Decompose(out var points, out _, out _, out _, out _);
            Assert.Contains(points, p => Math.Abs(p.X - 1) < 1e-9 && Math.Abs(p.Z) < 1e-9);
            Assert.Contains(points, p => Math.Abs(p.Z - 2) < 1e-9 && Math.Abs(p.Y) < 1e-9);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void UsdaFaceVaryingNormals_KeepBothSidesOfACrease()
    {
        string path = Path.Combine(Path.GetTempPath(), "camber_crease_" + Guid.NewGuid().ToString("N") + ".usda");
        try
        {
            File.WriteAllText(path, """
                #usda 1.0
                (
                    upAxis = "Z"
                )
                def Mesh "crease"
                {
                    int[] faceVertexCounts = [3, 3]
                    int[] faceVertexIndices = [0, 1, 2, 0, 2, 3]
                    point3f[] points = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
                    normal3f[] normals = [(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, -1), (0, 0, -1), (0, 0, -1)] (
                        interpolation = "faceVarying"
                    )
                    def GeomSubset "front"
                    {
                        uniform token elementType = "face"
                        int[] indices = [0]
                    }
                    def GeomSubset "back"
                    {
                        uniform token elementType = "face"
                        int[] indices = [1]
                    }
                }
                """);
            UsdaMesh raw = Assert.Single(UsdaReader.Read(path).Meshes);
            Assert.Equal(2, raw.Triangles.Count);
            Assert.Equal(6, raw.Points.Count);
            Assert.Equal(6, raw.Normals.Count);
            Assert.Contains(raw.Normals, n => n.Z > 0.9);
            Assert.Contains(raw.Normals, n => n.Z < -0.9);
            var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(3)), 1e-4);
            var mesh = api.LoadUsdaFile(path, name: "crease");
            Assert.Equal("back&front", mesh.ResolveLocalPatchNamePublic("front"));
            Assert.Equal("back&front", mesh.ResolveLocalPatchNamePublic("back"));
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void SvgEllipse_RoundTripsCenterAxesAndMinorRadius()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-8), new Vec3D(8)), 1e-4);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "oval");
        sketch.AddEllipse(new Vec2D(1, 2), new Vec2D(3, 1), 0.5).Name = "oval";
        string path = Path.Combine(Path.GetTempPath(), "camber_ellipse_" + Guid.NewGuid().ToString("N") + ".svg");
        try
        {
            api.SaveSvgFile(sketch, path);
            var loaded = api.GetSketcherFromSvg(path, DefaultPlanes.OriginXY, "oval2");
            Ellipse2D ellipse = Assert.IsType<Ellipse2D>(Assert.Single(loaded.GetAllCurves()));
            Assert.Equal(1, ellipse.Center.X, 5);
            Assert.Equal(2, ellipse.Center.Y, 5);
            Assert.Equal(0.5, ellipse.MinorAxisLength, 5);
            double major = ellipse.MajorAxis.Length();
            Assert.Equal(Math.Sqrt(10), major, 4);
            double align = Math.Abs(ellipse.MajorAxis.X * 3 + ellipse.MajorAxis.Y * 1) / (major * Math.Sqrt(10));
            Assert.InRange(align, 0.98, 1.01);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void SvgRectAndPolygon_ImportLinesAndEllipticalCorners()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(8)), 1e-4);
        string path = Path.Combine(Path.GetTempPath(), "camber_rect_" + Guid.NewGuid().ToString("N") + ".svg");
        try
        {
            File.WriteAllText(path, """
                <svg xmlns="http://www.w3.org/2000/svg">
                  <rect x="0" y="0" width="4" height="2" rx="0.4" ry="0.2"/>
                  <polygon points="0,0 3,0 3,1"/>
                </svg>
                """);
            var loaded = api.GetSketcherFromSvg(path, DefaultPlanes.OriginXY, "sheet", flipY: true);
            var curves = loaded.GetAllCurves();
            Assert.Equal(7, curves.Count(curve => curve is Line2D));
            var corners = curves.OfType<Ellipse2D>().ToArray();
            Assert.Equal(4, corners.Length);
            Assert.Equal(0.4, corners[0].MajorAxis.Length(), 4);
            Assert.Equal(0.2, corners[0].MinorAxisLength, 4);
            var polygon = curves.Skip(8).OfType<Line2D>().ToArray();
            Assert.Equal(3, polygon.Length);
            Assert.Equal(3, polygon[1].EndPosition.X, 5);
            Assert.Equal(-1, polygon[1].EndPosition.Y, 5);
            Assert.Equal(0, polygon[2].EndPosition.X, 5);
            Assert.Equal(0, polygon[2].EndPosition.Y, 5);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void SvgRoundTrip_RestoresLineCircleArcAndBezier()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "profile");
        sketch.AddLine(new Vec2D(0, 0), new Vec2D(4, 1)).Name = "base";
        sketch.AddCircle(new Vec2D(2, 3), 1.5).Name = "hole";
        sketch.AddArc(new Vec2D(1, 0), new Vec2D(0.70710678118, 0.70710678118), new Vec2D(0, 1)).Name = "fillet";
        sketch.AddBezier(new[] { new Vec2D(0, 0), new Vec2D(1, 2), new Vec2D(3, 2), new Vec2D(4, 0) }).Name = "bend";
        string path = Path.Combine(Path.GetTempPath(), "camber_sketch_" + Guid.NewGuid().ToString("N") + ".svg");
        try
        {
            api.SaveSvgFile(sketch, path, includeAnnotations: false);
            string text = File.ReadAllText(path);
            Assert.DoesNotContain("data-kind=\"distance\"", text);
            var loaded = api.GetSketcherFromSvg(path, DefaultPlanes.OriginXY, "back");
            Assert.Equal(4, loaded.GetAllCurves().Count);
            Line2D line = Assert.IsType<Line2D>(loaded.GetAllCurves()[0]);
            Assert.Equal(4, line.EndPosition.X, 6);
            Assert.Equal(1, line.EndPosition.Y, 6);
            Circle2D circle = Assert.IsType<Circle2D>(loaded.GetAllCurves()[1]);
            Assert.Equal(1.5, circle.Radius, 6);
            Assert.Equal(3, circle.Center.Y, 6);
            Arc2D arc = Assert.IsType<Arc2D>(loaded.GetAllCurves()[2]);
            Assert.Equal(1, arc.Radius, 3);
            Assert.InRange(arc.OnCurveCenterPosition.X, 0.6, 0.8);
            Assert.InRange(arc.OnCurveCenterPosition.Y, 0.6, 0.8);
            Bezier2D bezier = Assert.IsType<Bezier2D>(loaded.GetAllCurves()[3]);
            Assert.Equal(4, bezier.ControlPoints.Length);
            Assert.Equal(2, bezier.ControlPoints[1].Y, 5);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void SvgAnnotations_AreDrawnAndIgnoredOnImport()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(8)), 1e-3);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "dim");
        CLine2D line = sketch.AddCLine(new Vec2D(0, 0), new Vec2D(5, 0));
        sketch.SetDistance(line.CStart, line.CEnd, 5);
        CCircle2D circle = sketch.AddCCircle(new Vec2D(1, 2), 1);
        sketch.SetRadius(circle, 1);
        string annotated = Path.Combine(Path.GetTempPath(), "camber_dim_" + Guid.NewGuid().ToString("N") + ".svg");
        string plain = Path.Combine(Path.GetTempPath(), "camber_plain_" + Guid.NewGuid().ToString("N") + ".svg");
        try
        {
            api.SaveSvgFile(sketch, annotated, includeAnnotations: true);
            api.SaveSvgFile(sketch, plain, includeAnnotations: false);
            string text = File.ReadAllText(annotated);
            Assert.Contains("data-kind=\"distance\"", text);
            Assert.Contains("data-kind=\"radius\"", text);
            Assert.Contains(">5<", text);
            Assert.Contains(">R1<", text);
            Assert.DoesNotContain("data-camber", File.ReadAllText(plain));
            var loaded = api.GetSketcherFromSvg(annotated, DefaultPlanes.OriginXY, "imported");
            Assert.Equal(2, loaded.GetAllCurves().Count);
            Assert.Equal(5, loaded.GetAllCurves()[0].EndPosition.X, 5);
            Assert.Equal(1, ((Circle2D)loaded.GetAllCurves()[1]).Radius, 5);
        }
        finally
        {
            File.Delete(annotated);
            File.Delete(plain);
        }
    }

    [Fact]
    public void SvgConstruction_RoundTripsAndRawYCanStayUnflipped()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var sketch = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "guide");
        sketch.AddLine(new Vec2D(0, 0), new Vec2D(2, 0), CurveFlags.HelperGeometry);
        string path = Path.Combine(Path.GetTempPath(), "camber_guide_" + Guid.NewGuid().ToString("N") + ".svg");
        string raw = Path.Combine(Path.GetTempPath(), "camber_raw_" + Guid.NewGuid().ToString("N") + ".svg");
        try
        {
            api.SaveSvgFile(sketch, path);
            var loaded = api.GetSketcherFromSvg(path, DefaultPlanes.OriginXY, "guide2");
            Assert.True(loaded.GetAllCurves()[0].IsHelperGeometry);
            File.WriteAllText(raw, "<svg xmlns=\"http://www.w3.org/2000/svg\"><line x1=\"0\" y1=\"0\" x2=\"1\" y2=\"2\"/></svg>");
            var flipped = api.GetSketcherFromSvg(raw, DefaultPlanes.OriginXY, "flipped", flipY: true);
            var rawY = api.GetSketcherFromSvg(raw, DefaultPlanes.OriginXY, "rawy", flipY: false);
            Assert.Equal(-2, flipped.GetAllCurves()[0].EndPosition.Y, 6);
            Assert.Equal(2, rawY.GetAllCurves()[0].EndPosition.Y, 6);
        }
        finally
        {
            File.Delete(path);
            File.Delete(raw);
        }
    }
}
