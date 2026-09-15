#if DEBUG
using System.Text;
using GeoCore;

namespace CSG;

/// <summary>DEBUG probes inside <see cref="Resolver.Resolve"/> to locate where non-manifold topology appears.</summary>
internal static class ResolverTopologyDebug
{
    public static void AssertManifold(string stage, IReadOnlyList<Rat3Hybrid> positions, IReadOnlyList<Tri> triangles)
    {
        if (triangles == null || triangles.Count == 0)
            return;

        MeshDegeneracyDebug.AssertExactTopology(
            positions,
            triangles,
            groupPerTriangle: null,
            context: stage,
            allowNonManifoldEdges: true);
    }

    public static string Summarize(string stage, IReadOnlyList<Rat3Hybrid> positions, IReadOnlyList<Tri> triangles)
    {
        if (triangles == null)
            return $"{stage}: null triangles";

        int dupIndex = 0;
        int zeroArea = 0;
        var edgeTris = new Dictionary<(int, int), List<int>>();

        for (int i = 0; i < triangles.Count; i++)
        {
            var t = triangles[i];
            if (t.A < 0)
                continue;
            if (t.ContainsDuplicateIndex())
            {
                dupIndex++;
                continue;
            }

            if (positions != null)
            {
                var a = positions[t.A];
                var b = positions[t.B];
                var c = positions[t.C];
                var cross = Rat3Hybrid.Cross(b - a, c - a);
                if (cross.LengthSquared().Sign() == 0)
                    zeroArea++;
            }

            AddEdge(edgeTris, t.A, t.B, i);
            AddEdge(edgeTris, t.B, t.C, i);
            AddEdge(edgeTris, t.C, t.A, i);
        }

        int nonManifoldEdges = 0;
        int maxValence = 0;
        (int, int) worstEdge = default;
        foreach (var kv in edgeTris)
        {
            if (kv.Value.Count > maxValence)
            {
                maxValence = kv.Value.Count;
                worstEdge = kv.Key;
            }
            if (kv.Value.Count > 2)
                nonManifoldEdges++;
        }

        return $"{stage}: tris={triangles.Count} dupIndex={dupIndex} zeroArea={zeroArea} " +
               $"nonManifoldEdges={nonManifoldEdges} maxEdgeValence={maxValence} worstEdge=({worstEdge.Item1},{worstEdge.Item2})";
    }

    public static void LogStage(string stage, IReadOnlyList<Rat3Hybrid> positions, IReadOnlyList<Tri> triangles)
    {
        Console.WriteLine(Summarize(stage, positions, triangles));
    }

    public static void LogNonManifoldEdges(
        string stage,
        IReadOnlyList<Rat3Hybrid> positions,
        IReadOnlyList<Tri> triangles,
        IReadOnlyList<SourceTriangle> sources,
        int changeTriId,
        InsideResult[] insideResults = null)
    {
        if (triangles == null)
            return;

        var edgeTris = new Dictionary<(int, int), List<int>>();
        for (int i = 0; i < triangles.Count; i++)
        {
            var t = triangles[i];
            if (t.A < 0)
                continue;
            AddEdge(edgeTris, t.A, t.B, i);
            AddEdge(edgeTris, t.B, t.C, i);
            AddEdge(edgeTris, t.C, t.A, i);
        }

        foreach (var kv in edgeTris)
        {
            if (kv.Value.Count > 2)
                DumpEdgeTriangles(stage, positions, triangles, sources, changeTriId, kv.Key.Item1, kv.Key.Item2, insideResults);
        }
    }

    public static void DumpEdgeTriangles(
        string stage,
        IReadOnlyList<Rat3Hybrid> positions,
        IReadOnlyList<Tri> triangles,
        IReadOnlyList<SourceTriangle> sources,
        int changeTriId,
        int vtxA,
        int vtxB,
        InsideResult[] insideResults = null)
    {
        if (triangles == null || positions == null)
            return;

        if (vtxA > vtxB)
            Algorithms.Swap(ref vtxA, ref vtxB);

        var sb = new StringBuilder();
        sb.AppendLine($"{stage}: edge ({vtxA},{vtxB})");
        sb.AppendLine($"  vtxA={positions[vtxA]}");
        sb.AppendLine($"  vtxB={positions[vtxB]}");

        for (int i = 0; i < triangles.Count; i++)
        {
            var t = triangles[i];
            if (t.A < 0)
                continue;
            if (!EdgeOnTriangle(vtxA, vtxB, t))
                continue;

            string mesh = i < changeTriId ? "A" : "B";
            string src = sources != null && i < sources.Count
                ? $"srcTri={sources[i].SourceTriangleIndex} origin={sources[i].MeshOrigin}"
                : "src=?";
            string inside = insideResults != null && i < insideResults.Length
                ? $" inside={insideResults[i]}"
                : "";
            sb.AppendLine($"  tri[{i}] ({t.A},{t.B},{t.C}) mesh={mesh} {src}{inside}");
        }

        Console.WriteLine(sb.ToString());
    }

    private static bool EdgeOnTriangle(int a, int b, Tri t)
    {
        return (t.A == a && t.B == b) || (t.B == a && t.C == b) || (t.C == a && t.A == b) ||
               (t.B == a && t.A == b) || (t.C == a && t.B == b) || (t.A == a && t.C == b);
    }

    private static void AddEdge(Dictionary<(int, int), List<int>> edgeTris, int a, int b, int triIndex)
    {
        if (a > b)
            Algorithms.Swap(ref a, ref b);
        if (!edgeTris.TryGetValue((a, b), out var list))
        {
            list = new List<int>();
            edgeTris.Add((a, b), list);
        }
        list.Add(triIndex);
    }
}
#endif
