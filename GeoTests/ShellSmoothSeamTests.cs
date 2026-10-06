using Curves;
using Geo;
using Geo.Shelling;
using GeoCore;

namespace GeoTests;

[Collection("GlobalCadState")]
public sealed class ShellSmoothSeamTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedJunctionSolvesAllSupportsRatherThanTheirInitialTangentPlanes(bool translated)
    {
        var shift = translated ? new Vec3D(12, -7, 4) : new Vec3D(0);
        var supports = new IShellSurfaceSupport[] {
            new SphericalShellSupport(new SphereSurfaceParams { Center = shift + new Vec3D(-2, 0, 0), Radius = 3 }),
            new SphericalShellSupport(new SphereSurfaceParams { Center = shift + new Vec3D(2, 0, 0), Radius = 3 }),
            new PlanarShellSupport(new PlaneSurfaceParams { Origin = shift + new Vec3D(0, 0, 1), Normal = new(0, 0, 1) })
        };
        Assert.True(ShellSurfaceRegistry.AnalyticV1.TryResolveVertex(0, shift + new Vec3D(0, 3, 1),
            supports, 1e-8, out var point));
        Assert.InRange((point - (shift + new Vec3D(0, 2, 1))).Length(), 0, 1e-8);
        Array.Reverse(supports);
        Assert.True(ShellSurfaceRegistry.AnalyticV1.TryResolveVertex(0, shift + new Vec3D(0, 3, 1),
            supports, 1e-8, out var reversed));
        Assert.InRange((point - reversed).Length(), 0, 1e-8);
    }

    [Fact]
    public void DisjointCurvedSupportsDoNotAcceptAnInitialTangentIntersection()
    {
        var supports = new IShellSurfaceSupport[] {
            new SphericalShellSupport(new SphereSurfaceParams { Center = new(-5, 0, 0), Radius = 1 }),
            new SphericalShellSupport(new SphereSurfaceParams { Center = new(5, 0, 0), Radius = 1 })
        };
        Assert.False(ShellSurfaceRegistry.AnalyticV1.TryResolveVertex(0, new Vec3D(0, 2, 0),
            supports, 1e-8, out _));
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void ShallowSupportAngleDoesNotMeanParallel(int planeCount)
    {
        var planes = new List<IShellSurfaceSupport> {
            new PlanarShellSupport(new PlaneSurfaceParams { Origin = new(1, 0, 0), Normal = new(1, 0, 0) }),
            new PlanarShellSupport(new PlaneSurfaceParams { Origin = new(1, 2, 0), Normal = new(1, 1e-12, 0) })
        };
        if (planeCount == 3)
            planes.Add(new PlanarShellSupport(new PlaneSurfaceParams { Origin = new(0, 0, 3), Normal = new(0, 0, 1) }));
        Assert.True(ShellSurfaceRegistry.AnalyticV1.TryResolveVertex(0, new Vec3D(0, 0, 3),
            planes, 1e-8, out var point));
        Assert.InRange((point - new Vec3D(1, 2, 3)).Length(), 0, 1e-12);
        planes[1] = new PlanarShellSupport(new PlaneSurfaceParams { Origin = new(2, 0, 0), Normal = new(1, 0, 0) });
        Assert.False(ShellSurfaceRegistry.AnalyticV1.TryResolveVertex(0, new Vec3D(0), planes, 1e-8, out _));
    }

    [Fact]
    public void DuplicateTangentSupportsAtSmoothSeamDoNotCollapseShell()
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        var adapter = new MeshShellSurfaceAdapter();
        var sourcePoint = new Vec3D(1, 2, 3);
        IShellSurfaceSupport Support(Vec3D normal, double thickness)
        {
            var u = normal.X == 1 ? new Vec3D(0, 1, 0) : new Vec3D(1, 0, 0);
            var v = Vec3DOps.Cross(normal, u);
            var surface = new UVSurface(
                new List<Vec3D> { sourcePoint, sourcePoint + u, sourcePoint + v },
                new List<Vec3D> { normal, normal, normal },
                new List<Vec2D> { new(0), new(1, 0), new(0, 1) },
                new List<Tri> { new(0, 1, 2) },
                new List<Rat3Hybrid> { Exact(sourcePoint), Exact(sourcePoint + u), Exact(sourcePoint + v) });
            return adapter.CreateOffsetSupport(null, surface, normal, thickness, converter);
        }
        Rat3Hybrid Exact(Vec3D point)
        {
            var p = converter.Convert(point);
            return new Rat3Hybrid(p.X, p.Y, p.Z);
        }
        var side = Support(new Vec3D(1, 0, 0), .5);
        var otherSide = Support(new Vec3D(1, 0, 0), .5);
        var cap = Support(new Vec3D(0, 0, 1), .5);
        Assert.True(adapter.TryResolveVertex(0, sourcePoint, new[] { side, otherSide, cap }, out var point));
        Assert.InRange((point - new Vec3D(.5, 2, 2.5)).Length(), 0, 2 * converter.SmallestUnit());
        Assert.False(adapter.TryResolveVertex(0, sourcePoint,
            new[] { side, Support(new Vec3D(1, 0, 0), .75), cap }, out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void RoundedRectangleLoftShellAcceptsTangentLineNurbsSeams(bool outward, bool eraseMetadata)
    {
        var (api, blank) = RoundedRectangleLoft();
        if (eraseMetadata) blank.surfaceMetaData.Clear();
        var shell = api.Shell(blank, .4, new List<string> { "blank-EndCap" }, outward: outward);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(shell.Mesh.Positions, shell.Mesh.Triangles), 300, 500);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void RoundedRectangleLoftCapRimsAcceptBlends(bool chamfer, bool transformed)
    {
        var (api, blank) = RoundedRectangleLoft();
        if (transformed)
            blank = api.CopyMesh(blank, CoordinateSystem.Default,
                new CoordinateSystem(new Vec3D(2,-3,1), new Vec3D(.6,.8,0),
                    new Vec3D(-.48,.36,.8), new Vec3D(.64,-.48,.6)),
                "posed_", SurfacePatchNameAffix.Prefix, "posed");
        var rims = new List<string>();
        foreach (var edge in blank.GroupEdges)
            if (edge.Name.Contains("StartCap") || edge.Name.Contains("EndCap")) rims.Add(edge.Name);
        Assert.Equal(16, rims.Count);
        var result = chamfer ? api.Chamfer(blank, rims, .4, .01, "rounded")
            : api.Fillet(blank, rims, .4, .01, "rounded");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles, true));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles), 2000, 2250);
    }

    private static (GeoAPI Api, AnchorMesh Blank) RoundedRectangleLoft()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), .01);
        var sections = new List<PlotterSketcherCoordSys>();
        var starts = new List<string>();
        for (int i = 0; i < 3; i++)
        {
            var sketch = new PlotterSketcherCoordSys("section" + i,
                new CoordinateSystem(new Vec3D(0, 0, i * 8)), new Vec2D(-4, -6));
            Vec2D[][] corners =
            [
                [new(-4,-6), new(4,-6), new(6,-6), new(6,-4)],
                [new(6,-4), new(6,4), new(6,6), new(4,6)],
                [new(4,6), new(-4,6), new(-6,6), new(-6,4)],
                [new(-6,4), new(-6,-4), new(-6,-6), new(-4,-6)]
            ];
            foreach (var corner in corners)
            {
                var line = sketch.AddLine(corner[0], corner[1]);
                if (starts.Count == i) starts.Add(line.Name);
                sketch.AddBSpline(new[] { corner[1], corner[2], corner[3] }, 2,
                    weights: new[] { 1.0, Math.Sqrt(.5), 1.0 });
            }
            sections.Add(sketch);
        }
        var blank = api.Loft(sections, new LoftOptions {
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices, FirstCurves = starts }, "blank");
        return (api, blank);
    }
}
