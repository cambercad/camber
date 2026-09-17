using CSG;
using Geo;
using GeoCore;
using Remeshing;

namespace GeoTests;

public class MeshCleanTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void OverlappingCuboidUnion_CollapsesCollinearGroupEdges()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(4)), 1e-4);
        var a = api.CreateCuboid(new Vec3D(-0.25, -0.25, -0.25), new Vec3D(1.25, 0.25, 0.25), "a");
        var b = api.CreateCuboid(new Vec3D(0.75, -0.25, -0.25), new Vec3D(2.25, 0.25, 0.25), "b");
        var union = api.Boolean(a, b, BooleanOp.Union, "ab");
        union.EnsureCoplanarPostProcessed();

        MeshPipelineTestHelpers.AssertVolume(union.Mesh, 0.625);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(union.Mesh);
        Assert.Equal(8, MeshPipelineTestHelpers.CountUsedVertices(union.Mesh));
        Assert.Equal(0, MeshPipelineTestHelpers.CountCollinearInteriorGroupEdgeVertices(union));
    }

    [Fact]
    public void AnchorMesh_Cuboid_PreservesUvAfterCleanPipeline()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(2)), 1e-4);
        var cube = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 1), "cube");

        MeshPipelineTestHelpers.AssertCornerUvConsistentWithinGroups(cube.Mesh);
        MeshPipelineTestHelpers.AssertVolume(cube.Mesh, 1.0);
    }

    [Fact]
    public void Rat3Hybrid_AreOppositeCollinear_DetectsAntiParallelDirections()
    {
        var forward = new Rat3Hybrid(2, 0, 0);
        var backward = new Rat3Hybrid(-3, 0, 0);
        var sideways = new Rat3Hybrid(0, 1, 0);
        var sameDirection = new Rat3Hybrid(4, 0, 0);

        Assert.True(Rat3Hybrid.AreOppositeCollinear(forward, backward));
        Assert.False(Rat3Hybrid.AreOppositeCollinear(forward, sideways));
        Assert.False(Rat3Hybrid.AreOppositeCollinear(forward, sameDirection));
    }

    [Fact]
    public void EdgeCollapser_RemovesCollinearMiddleVertex()
    {
        var positions = new List<Rat3Hybrid>
        {
            new Rat3Hybrid(0, 0, 0), // A
            new Rat3Hybrid(1, 0, 0), // B (collinear middle)
            new Rat3Hybrid(2, 0, 0), // C
            new Rat3Hybrid(1, 1, 0), // D
        };

        var triangles = new List<TriWithGroupId>
        {
            new() { A = 0, B = 1, C = 3, GroupId = 0 },
            new() { A = 1, B = 2, C = 3, GroupId = 0 },
        };

        EdgeCollapser.CleanMesh(
            positions,
            triangles,
            minDist: BigRationalHybrid.Zero,
            removeCollinearEdges: true,
            vertexMustBePreserved: new bool[positions.Count]);

        int activeTriangles = triangles.Count(t => t.A >= 0);
        Assert.Equal(1, activeTriangles);
        Assert.DoesNotContain(triangles, t => t.A >= 0 && (t.A == 1 || t.B == 1 || t.C == 1));
    }

    [Fact]
    public void EdgeCollapser_CollinearRemoval_IsDeterministicAcrossRuns()
    {
        static (List<Rat3Hybrid> positions, List<TriWithGroupId> triangles) CloneCollinearFan()
        {
            var positions = new List<Rat3Hybrid>
            {
                new(0, 0, 0),
                new(1, 0, 0),
                new(2, 0, 0),
                new(1, 1, 0),
            };
            var triangles = new List<TriWithGroupId>
            {
                new() { A = 0, B = 1, C = 3, GroupId = 0 },
                new() { A = 1, B = 2, C = 3, GroupId = 0 },
            };
            return (positions, triangles);
        }

        var run1 = CloneCollinearFan();
        var run2 = CloneCollinearFan();
        var preserve = new bool[run1.positions.Count];

        EdgeCollapser.CleanMesh(run1.positions, run1.triangles, BigRationalHybrid.Zero, removeCollinearEdges: true, vertexMustBePreserved: preserve);
        EdgeCollapser.CleanMesh(run2.positions, run2.triangles, BigRationalHybrid.Zero, removeCollinearEdges: true, vertexMustBePreserved: preserve);

        var active1 = run1.triangles.Where(t => t.A >= 0).Select(t => (t.A, t.B, t.C)).OrderBy(x => x).ToList();
        var active2 = run2.triangles.Where(t => t.A >= 0).Select(t => (t.A, t.B, t.C)).OrderBy(x => x).ToList();
        Assert.Equal(active1, active2);
    }

    [Fact]
    public void EdgeCollapser_PreservesVertexMarkedByUvDiscontinuity()
    {
        var positions = new List<Rat3Hybrid>
        {
            new Rat3Hybrid(0, 0, 0),
            new Rat3Hybrid(1, 0, 0),
            new Rat3Hybrid(2, 0, 0),
            new Rat3Hybrid(1, 1, 0),
        };

        var triangles = new List<TriWithGroupId>
        {
            new() { A = 0, B = 1, C = 3, GroupId = 0 },
            new() { A = 1, B = 2, C = 3, GroupId = 0 },
        };

        bool[] preserve = new bool[positions.Count];
        preserve[1] = true;

        EdgeCollapser.CleanMesh(
            positions,
            triangles,
            minDist: BigRationalHybrid.Zero,
            removeCollinearEdges: true,
            vertexMustBePreserved: preserve);

        int activeTriangles = triangles.Count(t => t.A >= 0);
        Assert.Equal(2, activeTriangles);
        Assert.Contains(triangles, t => t.A >= 0 && (t.A == 1 || t.B == 1 || t.C == 1));
    }

    [Fact]
    public void FaceTouchUnion_StillHasNonManifoldEdges_AndStableVolumeAfterConstruction()
    {
        var union = CreateEdgeTouchUnion();

        Assert.NotEmpty(MeshPipelineTestHelpers.NonManifoldEdges(union.Mesh.Triangles));
        MeshPipelineTestHelpers.AssertVolume(union.Mesh, 3.0);
        MeshPipelineTestHelpers.AssertCornerUvConsistentWithinGroups(union.Mesh);
    }

    private static AnchorMesh CreateEdgeTouchUnion()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0.0, 0, 0)), new Vec3D(1, 1, 1), "a");
        var b = api.CreateCuboid(new CoordinateSystem(new Vec3D(1, 1, 0)), new Vec3D(1, 1, 1), "b");
        var c = api.CreateCuboid(new CoordinateSystem(new Vec3D(2, 2, 0)), new Vec3D(1, 1, 1), "c");
        var ab = api.Boolean(a, b, BooleanOp.Union, "ab");
        return api.Boolean(ab, c, BooleanOp.Union, "abc");
    }
}
