#if DEBUG
using System.Text;

namespace GeoCore;

/// <summary>
/// DEBUG-only exact (Rat3Hybrid / BigRational) mesh topology checks.
/// Degenerate or non-manifold configuration should not occur when boolean output is exact.
/// </summary>
public static class MeshDegeneracyDebug
{
    public static void AssertExactTopology(
        IReadOnlyList<Rat3Hybrid> positions,
        IReadOnlyList<Tri> triangles,
        IReadOnlyList<int> groupPerTriangle,
        string context,
        bool allowNonManifoldEdges = false)
    {
        var report = BuildReport(positions, triangles, groupPerTriangle, context, allowNonManifoldEdges);
        if (report != null)
            throw new InvalidOperationException(report);
    }

    public static void ThrowDuplicateIndexTriangle(int triIndex, in Tri tri, string context)
    {
        throw new InvalidOperationException(
            $"Duplicate vertex index in triangle (exact degeneracy) [{context}]: " +
            $"tri[{triIndex}] = ({tri.A}, {tri.B}, {tri.C})");
    }

    public static void ThrowNonManifoldEdge(
        int vertA,
        int vertB,
        int tri0,
        int tri1,
        int triNew,
        TriEdgeType edgeType,
        IReadOnlyList<Rat3Hybrid> positions,
        IReadOnlyList<Tri> triangles,
        string context)
    {
        var sb = new StringBuilder();
        sb.AppendLine($"Non-manifold edge (>{2} triangles share one undirected edge) [{context}].");
        sb.AppendLine("The legacy Adjacency message blamed duplicate indices; inspect the triangles below.");
        sb.AppendLine($"  edge ({vertA}, {vertB})  edgeType={edgeType}");
        sb.AppendLine($"  existing triangles: {tri0}, {tri1};  new triangle: {triNew}");
        if (positions != null)
        {
            sb.AppendLine($"  P[{vertA}] = {FormatRat(positions[vertA])}");
            sb.AppendLine($"  P[{vertB}] = {FormatRat(positions[vertB])}");
        }
        if (positions != null && triangles != null)
        {
            AppendTriangle(sb, "  tri0", positions, triangles, tri0, null);
            AppendTriangle(sb, "  tri1", positions, triangles, tri1, null);
            AppendTriangle(sb, "  triNew", positions, triangles, triNew, null);
        }
        throw new InvalidOperationException(sb.ToString());
    }

    private static string BuildReport(
        IReadOnlyList<Rat3Hybrid> positions,
        IReadOnlyList<Tri> triangles,
        IReadOnlyList<int> groupPerTriangle,
        string context,
        bool allowNonManifoldEdges)
    {
        var issues = new List<string>();

        for (int i = 0; i < triangles.Count; i++)
        {
            var tri = triangles[i];
            if (tri.A < 0)
                continue;

            if (tri.ContainsDuplicateIndex())
            {
                issues.Add($"duplicate-index tri[{i}] = ({tri.A}, {tri.B}, {tri.C}){GroupSuffix(groupPerTriangle, i)}");
                continue;
            }

            if (positions != null && HasZeroExactArea(positions, tri))
            {
                issues.Add(
                    $"zero-exact-area tri[{i}] = ({tri.A}, {tri.B}, {tri.C}){GroupSuffix(groupPerTriangle, i)}  " +
                    FormatTriangleVertices(positions, tri));
            }
        }

        if (positions != null)
            CollectCoincidentVertexPairs(positions, issues, maxReport: 8);

        CollectInvalidEdges(triangles, issues, maxReport: 8, allowNonManifoldEdges);

        if (issues.Count == 0)
            return null;

        var sb = new StringBuilder();
        sb.AppendLine($"Exact mesh topology violation [{context}] ({issues.Count} issue(s), BigRational positions):");
        for (int i = 0; i < issues.Count; i++)
            sb.AppendLine("  " + issues[i]);
        return sb.ToString();
    }

    private static bool HasZeroExactArea(IReadOnlyList<Rat3Hybrid> positions, in Tri tri)
    {
        var a = positions[tri.A];
        var b = positions[tri.B];
        var c = positions[tri.C];
        var ab = b - a;
        var ac = c - a;
        var cross = Rat3Hybrid.Cross(ab, ac);
        return cross.LengthSquared().Sign() == 0;
    }

    private static void CollectCoincidentVertexPairs(IReadOnlyList<Rat3Hybrid> positions, List<string> issues, int maxReport)
    {
        int reported = 0;
        for (int i = 0; i < positions.Count && reported < maxReport; i++)
        {
            for (int j = i + 1; j < positions.Count && reported < maxReport; j++)
            {
                if (positions[i] == positions[j])
                {
                    issues.Add(
                        $"coincident-vertices indices {i} and {j} share exact position {FormatRat(positions[i])} " +
                        "(should be welded to one index)");
                    reported++;
                }
            }
        }
    }

    private static void CollectInvalidEdges(
        IReadOnlyList<Tri> triangles,
        List<string> issues,
        int maxReport,
        bool allowNonManifoldEdges)
    {
        var edgeTris = new Dictionary<(int, int), List<int>>();
        for (int i = 0; i < triangles.Count; i++)
        {
            var t = triangles[i];
            if (t.A < 0)
                continue;
            if (t.ContainsDuplicateIndex())
                continue;
            AddEdge(edgeTris, t.A, t.B, i);
            AddEdge(edgeTris, t.B, t.C, i);
            AddEdge(edgeTris, t.C, t.A, i);
        }

        int reported = 0;
        foreach (var kv in edgeTris)
        {
            if (allowNonManifoldEdges)
            {
                if ((kv.Value.Count & 1) == 0)
                    continue;
                issues.Add(
                    $"open-or-odd-valence-edge ({kv.Key.Item1}, {kv.Key.Item2}) used by {kv.Value.Count} triangles: " +
                    string.Join(", ", kv.Value));
                if (++reported >= maxReport)
                    break;
            }
            else
            {
                if (kv.Value.Count <= 2)
                    continue;
                issues.Add(
                    $"non-manifold-edge ({kv.Key.Item1}, {kv.Key.Item2}) used by {kv.Value.Count} triangles: " +
                    string.Join(", ", kv.Value));
                if (++reported >= maxReport)
                    break;
            }
        }
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

    private static void AppendTriangle(
        StringBuilder sb,
        string label,
        IReadOnlyList<Rat3Hybrid> positions,
        IReadOnlyList<Tri> triangles,
        int triIndex,
        IReadOnlyList<int> groupPerTriangle)
    {
        if (triIndex < 0 || triIndex >= triangles.Count)
            return;
        var tri = triangles[triIndex];
        sb.Append(label);
        sb.Append($"[{triIndex}] = ({tri.A}, {tri.B}, {tri.C}){GroupSuffix(groupPerTriangle, triIndex)}: ");
        sb.AppendLine(FormatTriangleVertices(positions, tri));
    }

    private static string FormatTriangleVertices(IReadOnlyList<Rat3Hybrid> positions, in Tri tri) =>
        $"A={FormatRat(positions[tri.A])}  B={FormatRat(positions[tri.B])}  C={FormatRat(positions[tri.C])}";

    private static string FormatRat(in Rat3Hybrid p) => $"({p.X}, {p.Y}, {p.Z})";

    private static string GroupSuffix(IReadOnlyList<int> groupPerTriangle, int triIndex)
    {
        if (groupPerTriangle == null || triIndex < 0 || triIndex >= groupPerTriangle.Count)
            return "";
        return $" group={groupPerTriangle[triIndex]}";
    }
}
#endif
