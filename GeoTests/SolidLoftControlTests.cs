using Curves;
using Geo;
using GeoCore;
using NURBS;

namespace GeoTests;

public sealed class SolidLoftControlTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);
    private static GeoAPI Api() => new(new Box3D(new Vec3D(-100), new Vec3D(100)), .01, name: "controlled-loft");
    private static PlotterSketcherCoordSys Square(double z, double scale = 1, Vec3D shift = default)
    {
        var sketch = new PlotterSketcherCoordSys("section" + z, new CoordinateSystem(shift + new Vec3D(0, 0, z * scale)));
        Vec2D[] points = [new(-2 * scale, -2 * scale), new(2 * scale, -2 * scale), new(2 * scale, 2 * scale), new(-2 * scale, 2 * scale)];
        for (int i = 0; i < 4; i++) sketch.AddLine(points[i], points[(i + 1) % 4]);
        return sketch;
    }
    private static BSplineSurface Side(AnchorMesh mesh) => Assert.IsType<BSplineSurface>(mesh.surfaceMetaData.Values.First(m => m.SurfaceType == SurfaceType.Unknown).NurbsSurface);
    private static void Near(Vec3D a, Vec3D b, double tolerance = 1e-9) => Assert.InRange((a - b).Length(), 0, tolerance);
    private static void Closed(AnchorMesh mesh)
    {
        Assert.True(MeshAnalysis.IsWatertightMesh(mesh.Mesh.PrecisionPositions, mesh.Mesh.Triangles));
        Assert.All(mesh.Mesh.Positions, p => Assert.True(double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z)));
    }

    private static void MeshMatchesSupport(AnchorMesh mesh)
    {
        var surface = Side(mesh);
        var groups = mesh.Mesh.GetTriangleGroups();
        for (int i = 0; i < mesh.Mesh.Triangles.Count; i++)
        {
            string name = mesh.groupIdToExtendedName[groups[i]];
            if (!name.Contains("-Side")) continue;
            var triangle = mesh.Mesh.Triangles[i];
            var corner = mesh.Mesh.TrianglesEx[i];
            foreach (var (index, vertex) in new[] { (triangle.A, corner.V0), (triangle.B, corner.V1), (triangle.C, corner.V2) })
            {
                Near(mesh.Mesh.Positions[index], surface.Evaluate(vertex.UV.X, vertex.UV.Y), .0005);
                Assert.True(double.IsFinite(vertex.Normal.X) && double.IsFinite(vertex.Normal.Y) && double.IsFinite(vertex.Normal.Z));
                Assert.InRange(vertex.Normal.Length(), .999999, 1.000001);
            }
        }
    }

    [Fact]
    public void EndDerivativesControlExactSolidSupportAndMeshRemainsClosed()
    {
        var start = new Vec3D(0, 6, 10);
        var end = new Vec3D(0, -3, 10);
        var options = new LoftOptions { StartTangent = start, EndTangent = end };
        var mesh = Api().Loft([Square(0), Square(10)], options, "tangent");
        // Metadata describes the built solid, even if the caller reuses options.
        options.StartTangent = new Vec3D(10, 0, 10);
        Closed(mesh);
        MeshMatchesSupport(mesh);
        var side = Side(mesh);
        foreach (double u in new[] { .1, .35, .7, .95 })
        {
            Near(side.EvaluateDV(u, 0), start);
            Near(side.EvaluateDV(u, 1), end);
            Near((side.Evaluate(u, 1e-6) - side.Evaluate(u, 0)) / 1e-6, start, 3e-5);
        }
        Assert.InRange(side.Evaluate(.1, .5).Y, -1, 3);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void GuidesControlClosedSeamAndInteriorCornerExactly(int corner)
    {
        double x = corner == 0 ? -2 : 2;
        var rail = new CubicHermiteSpline3D([new(x, -2, 0), new(x, -2, 5), new(x, -2, 10)],
            new Vec3D?[] { new(0, 1, 1), new(0, -1, 1), new(0, 1, 1) });
        var api = Api();
        var mesh = api.Loft([Square(0), Square(5), Square(10)], new LoftOptions { Guides = [rail] }, "guided");
        Closed(mesh);
        MeshMatchesSupport(mesh);
        var side = Side(mesh);
        for (int i = 0; i <= 40; i++)
        {
            double v = i / 40.0;
            Near(side.Evaluate(corner / 4.0, v), rail.Evaluate(v).Origin);
            if (corner == 0) Near(side.Evaluate(1, v), rail.Evaluate(v).Origin);
        }
        var reference = Side(api.Loft([Square(0), Square(5), Square(10)], new LoftOptions(), "reference"));
        for (int p = 0; p < 3; p++)
            for (int i = 0; i <= 20; i++) Near(side.Evaluate(i / 20.0, p / 2.0), reference.Evaluate(i / 20.0, p / 2.0));
    }

    [Fact]
    public void RationalCircularProfilesRetainExactSectionsWithMultipleGuides()
    {
        var api = Api();
        PlotterSketcherCoordSys Circle(double z)
        {
            var sketch = new PlotterSketcherCoordSys("circle" + z, new CoordinateSystem(new Vec3D(0, 0, z)));
            sketch.AddCircle(new Vec2D(0), 2);
            return sketch;
        }
        var seam = new CubicHermiteSpline3D([new(2, 0, 0), new(2, 0, 5), new(2, 0, 10)],
            new Vec3D?[] { new(.2, 0, 1), new(-.2, 0, 1), new(.2, 0, 1) });
        var quarter = new CubicHermiteSpline3D([new(0, 2, 0), new(0, 2, 5), new(0, 2, 10)],
            new Vec3D?[] { new(0, .2, 1), new(0, -.2, 1), new(0, .2, 1) });
        var mesh = api.Loft([Circle(0), Circle(5), Circle(10)], new LoftOptions { Guides = [seam, quarter] }, "circular");
        Closed(mesh);
        MeshMatchesSupport(mesh);
        var support = Side(mesh);
        for (int i = 0; i <= 30; i++)
        {
            double v = i / 30.0;
            Near(support.Evaluate(0, v), seam.Evaluate(v).Origin);
            Near(support.Evaluate(.25, v), quarter.Evaluate(v).Origin);
            for (int p = 0; p < 3; p++)
            {
                var point = support.Evaluate(v, p / 2.0);
                Assert.InRange(Math.Abs(point.X * point.X + point.Y * point.Y - 4), 0, 1e-10);
                Assert.InRange(Math.Abs(point.Z - p * 5), 0, 1e-10);
            }
        }
    }

    [Fact]
    public void ControlsAreCovariantUnderTranslationAndUniformScale()
    {
        var api = Api();
        var tangent = new Vec3D(3, 2, 10);
        var a = Side(api.Loft([Square(0), Square(10)], new LoftOptions { StartTangent = tangent }, "a"));
        var offset = new Vec3D(7, -8, 2);
        var b = Side(api.Loft([Square(0, 2, offset), Square(10, 2, offset)], new LoftOptions { StartTangent = tangent * 2 }, "b"));
        foreach (double u in new[] { .05, .3, .7 })
            foreach (double v in new[] { 0, .2, .5, .8, 1 }) Near(a.Evaluate(u, v) * 2 + offset, b.Evaluate(u, v));
    }

    [Fact]
    public void InvalidGuidesTangentsAndRuledControlsAreRejected()
    {
        var api = Api();
        PlotterSketcherCoordSys[] sections = [Square(0), Square(5), Square(10)];
        var rail = new Line3D(new(-2, -2, 0), new(-2, -2, 10));
        Assert.Throws<ArgumentException>(() => api.Loft(sections, new LoftOptions { Guides = [rail, rail] }));
        Assert.Throws<ArgumentException>(() => api.Loft(sections, new LoftOptions { Guides = [null] }));
        Assert.Throws<ArgumentException>(() => api.Loft([Square(0), Square(4), Square(10)], new LoftOptions { Guides = [rail] }));
        Assert.Throws<ArgumentException>(() => api.Loft(sections, new LoftOptions { Guides = [new Line3D(new(0, 0, 0), new(0, 0, 10))] }));
        Assert.Throws<ArgumentException>(() => api.Loft(sections, new LoftOptions { StartTangent = new Vec3D(double.NaN, 0, 1) }));
        Assert.Throws<ArgumentException>(() => api.Loft(sections, new LoftOptions { EndTangent = new Vec3D() }));
        Assert.Throws<ArgumentException>(() => api.Loft(sections, new LoftOptions { Style = LoftStyle.Ruled, Guides = [rail] }));
        Assert.Throws<ArgumentException>(() => api.Loft(sections, new LoftOptions { Guides = [rail], StartTangent = new Vec3D(0, 1, 10) }));
    }

    [Fact]
    public void GuideAndRationalSectionsBothRemainExactWhenSectionWeightsDiffer()
    {
        var profiles=Enumerable.Range(0,3).Select(p=>new BSplineCurve(1,
            new[] {new Vec3D(-2,-2,p*5),new Vec3D(2,-2,p*5),new Vec3D(2,2,p*5),new Vec3D(-2,2,p*5),new Vec3D(-2,-2,p*5)},
            new[] {1.0,new[] {1.0,.8,1.2}[p],1.0,1.0,1.0},new[] {0.0,0,.25,.5,.75,1,1},true)).ToArray();
        var rail=new CubicHermiteSpline3D([new(2,-2,0),new(2,-2,5),new(2,-2,10)],
            new Vec3D?[] {new(0,1,1),new(0,-1,1),new(0,1,1)});
        var side=Geo.NurbsConstruction.SurfaceLoft.BuildSolid(profiles,[rail],null,null);
        for(int p=0;p<3;p++)
            for(int i=0;i<=40;i++) Near(profiles[p].EvaluateUniform(i/40.0),side.Evaluate(i/40.0,p/2.0));
        for(int i=0;i<=40;i++) Near(rail.Evaluate(i/40.0).Origin,side.Evaluate(.25,i/40.0));
    }

    [Fact]
    public void VaryingClosedEndpointWeightRatiosNeedOneExactSeamGuide()
    {
        var profiles=Enumerable.Range(0,3).Select(p=>new BSplineCurve(1,
            new[] {new Vec3D(-2,-2,p*5),new Vec3D(2,-2,p*5),new Vec3D(2,2,p*5),new Vec3D(-2,2,p*5),new Vec3D(-2,-2,p*5)},
            new[] {1.0,1,1,1,new[] {1.0,2,1}[p]},new[] {0.0,0,.25,.5,.75,1,1},true)).ToArray();
        Assert.Throws<ArgumentException>(()=>Geo.NurbsConstruction.SurfaceLoft.BuildSolid(profiles,null,null,null));
        var rail=new Line3D(new(-2,-2,0),new(-2,-2,10));
        var side=Geo.NurbsConstruction.SurfaceLoft.BuildSolid(profiles,[rail],null,null);
        for(int i=0;i<=40;i++)
        {
            Near(side.Evaluate(0,i/40.0),side.Evaluate(1,i/40.0));
            Near(rail.Evaluate(i/40.0).Origin,side.Evaluate(0,i/40.0));
        }
    }
}
