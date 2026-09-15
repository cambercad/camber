using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

internal static class MeshPipelineTestHelpers
{
    public const double VolumeTolerance = 1e-5;
    public const double UvToleranceSquared = 1e-10;

    public static double AbsVolume(MeshNormalUV mesh) =>
        Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(mesh.Positions, mesh.Triangles));

    public static void AssertVolume(MeshNormalUV mesh, double expected, double tolerance = VolumeTolerance)
    {
        double volume = AbsVolume(mesh);
        Assert.True(
            Math.Abs(expected - volume) < tolerance,
            $"expected volume {expected}, got {volume:F8}");
    }

    public static void AssertWatertightAllowTouch(MeshNormalUV mesh)
    {
        Assert.True(
            MeshAnalysis.IsWatertightMesh(mesh.Positions, mesh.Triangles, allowTouch: true),
            "Mesh is not watertight (allowTouch).");
    }

    public static void AssertCornerUvConsistentWithinGroups(MeshNormalUV mesh)
    {
        var uvsByGroupVertex = new Dictionary<(int GroupId, int VertexId), List<Vec2D>>();

        for (int i = 0; i < mesh.Triangles.Count; i++)
        {
            var tri = mesh.Triangles[i];
            var triEx = mesh.TrianglesEx[i];
            int groupId = triEx.GroupId;

            RecordUv(uvsByGroupVertex, groupId, tri.A, triEx.V0.UV);
            RecordUv(uvsByGroupVertex, groupId, tri.B, triEx.V1.UV);
            RecordUv(uvsByGroupVertex, groupId, tri.C, triEx.V2.UV);
        }

        foreach (var entry in uvsByGroupVertex)
        {
            Vec2D reference = entry.Value[0];
            for (int i = 1; i < entry.Value.Count; i++)
            {
                Assert.True(
                    Vec2DOps.DistanceSquared(reference, entry.Value[i]) <= UvToleranceSquared,
                    $"UV mismatch at group={entry.Key.GroupId}, vertex={entry.Key.VertexId}");
            }
        }
    }

    public static int CountDistinctGroups(MeshNormalUV mesh) =>
        mesh.GetTriangleGroups().Distinct().Count();

    public static int CountUsedVertices(MeshNormalUV mesh)
    {
        var used = new HashSet<int>();
        for (int i = 0; i < mesh.Triangles.Count; i++)
        {
            var tri = mesh.Triangles[i];
            used.Add(tri.A);
            used.Add(tri.B);
            used.Add(tri.C);
        }
        return used.Count;
    }

    public static int CountCollinearInteriorGroupEdgeVertices(AnchorMesh mesh)
    {
        int count = 0;
        if (mesh.GroupEdges == null)
            return 0;

        for (int e = 0; e < mesh.GroupEdges.Count; e++)
        {
            var strips = mesh.GroupEdges[e].LineStrips3D;
            if (strips == null)
                continue;
            for (int s = 0; s < strips.Count; s++)
            {
                var strip = strips[s];
                if (strip == null || strip.Points == null || strip.Points.Count < 3)
                    continue;

                var pts = strip.Points;
                bool closed = strip.IsClosed();
                int vertexCount = closed ? pts.Count - 1 : pts.Count;
                if (vertexCount < 3)
                    continue;

                int interiorCount = closed ? vertexCount : vertexCount - 2;
                int start = closed ? 0 : 1;
                for (int k = 0; k < interiorCount; k++)
                {
                    int i = (start + k) % vertexCount;
                    int prev = (i - 1 + vertexCount) % vertexCount;
                    int next = (i + 1) % vertexCount;
                    Vec3D d0 = pts[i] - pts[prev];
                    Vec3D d1 = pts[next] - pts[i];
                    if (Vec3DOps.Cross(d0, d1).LengthSquared() < 1e-20)
                        count++;
                }
            }
        }
        return count;
    }

    public static HashSet<(int, int)> NonManifoldEdges(IList<Tri> triangles)
    {
        var edgeToTriangles = AdjacencyEx.BuildEdgeToTrianglesMap(triangles.ToList());
        return edgeToTriangles
            .Where(kv => kv.Value.Count > 2)
            .Select(kv => kv.Key)
            .ToHashSet();
    }

    public static AnchorMesh CreateFaceTouchUnion(GeoAPI api)
    {
        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 1), "a");
        var b = api.CreateCuboid(new CoordinateSystem(new Vec3D(1, 0, 0)), new Vec3D(1, 1, 1), "b");
        var union = api.Boolean(a, b, BooleanOp.Union, "ab");
        union.EnsureCoplanarPostProcessed();
        return union;
    }

    private static void RecordUv(
        Dictionary<(int GroupId, int VertexId), List<Vec2D>> map,
        int groupId,
        int vertexId,
        Vec2D uv)
    {
        var key = (groupId, vertexId);
        if (!map.TryGetValue(key, out var list))
        {
            list = new List<Vec2D>();
            map[key] = list;
        }
        list.Add(uv);
    }
}
