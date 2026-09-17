using GeoCore;
using GeoMeta;

namespace Geo;

internal static class FaceLineageEdges
{
    private const string CurrentMarker = "#current=";

    internal static string Reference(AnchorMesh mesh, GroupEdge edge)
    {
        bool ambiguous = EntityNaming.TryParseGroupEdgeAddress(edge.Name, out var pair, requireFullMatch: true) &&
            (mesh.AmbiguousFaceReferences.Contains(pair.PatchA) || mesh.AmbiguousFaceReferences.Contains(pair.PatchB));
        return ambiguous ? edge.Name + CurrentMarker + mesh.CurrentEdgeReferenceScope.ToString("N") : edge.Name;
    }

    private static string ResolveCurrent(AnchorMesh mesh, string reference, ref EdgeGraph graph)
    {
        int marker = reference.LastIndexOf(CurrentMarker, StringComparison.Ordinal);
        string name = reference[..marker];
        if (!Guid.TryParseExact(reference[(marker + CurrentMarker.Length)..], "N", out Guid scope) ||
            scope != mesh.CurrentEdgeReferenceScope)
            throw new NameCollisionException($"Current edge reference '{reference}' belongs to another or rebuilt solid; reacquire its edge references.");
        var source = mesh.GroupEdges.SingleOrDefault(edge => edge.Name == name);
        if (source == null)
            throw new NameCollisionException($"Current edge reference '{reference}' is no longer present; reacquire its edge references.");
        // Names with ordinals may be reordered when a graph is regenerated after
        // rigid placement. Match the same topological segments, not its ordinal.
        graph ??= new EdgeGraph(mesh.Mesh.Triangles, mesh.Mesh.GetTriangleGroups(), mesh.Mesh.Positions,
            mesh.Mesh.PrecisionPositions, mesh.groupIdToExtendedName);
        static long SegmentKey(Int2 segment) =>
            ((long)Math.Min(segment.X, segment.Y) << 32) | (uint)Math.Max(segment.X, segment.Y);
        var segments = source.EdgeSegments.Select(SegmentKey).ToHashSet();
        var matches = graph.Edges.Where(edge =>
            segments.SetEquals(edge.EdgeSegments.Select(SegmentKey))).ToArray();
        if (matches.Length != 1)
            throw new NameCollisionException($"Current edge reference '{reference}' no longer identifies one edge; reacquire its edge references.");
        return matches[0].Name;
    }

    internal static List<string> Resolve(AnchorMesh mesh, IEnumerable<string> references)
    {
        EdgeGraph graph = null;
        var result = new List<string>();
        foreach (string reference in references)
        {
            if (reference.LastIndexOf(CurrentMarker, StringComparison.Ordinal) > reference.LastIndexOf(']'))
            {
                result.Add(ResolveCurrent(mesh, reference, ref graph));
                continue;
            }
            if (!EntityNaming.TryParseGroupEdgeAddress(reference, out var pair, requireFullMatch: true) ||
                (!mesh.AmbiguousFaceReferences.Contains(pair.PatchA) && !mesh.AmbiguousFaceReferences.Contains(pair.PatchB)))
            {
                mesh.ValidateEntityReference(reference);
                result.Add(reference);
                continue;
            }
            // The parser represents both an absent suffix and explicit _0 as
            // index zero. Only an unindexed relation may expand ancestors:
            // accepting any explicit ordinal would silently change its target.
            if (!reference.TrimEnd().EndsWith("]", StringComparison.Ordinal))
                throw new NameCollisionException($"Indexed support-pair reference '{reference}' cannot resolve split ancestors; use an unindexed unique support pair or a provenance-qualified edge.");
            // An authored support-pair edge is a relation, not a finite face
            // datum. It is valid only when that relation selects one current
            // connected edge. Ordinal diagnostic fragment labels never expand.
            HashSet<int> Supports(string patch)
            {
                var descendants = mesh.FaceLineages.Where(item => item.Value.Roots.Contains(patch))
                    .Select(item => item.Key).ToHashSet();
                if (descendants.Count > 0) return descendants;
                mesh.ValidateEntityReference(patch);
                if (mesh.extendedNameToGroupId.TryGetValue(patch, out int id)) return new() { id };
                return new();
            }
            var first = Supports(pair.PatchA);
            var second = Supports(pair.PatchB);
            graph ??= new EdgeGraph(mesh.Mesh.Triangles, mesh.Mesh.GetTriangleGroups(), mesh.Mesh.Positions,
                mesh.Mesh.PrecisionPositions, mesh.groupIdToExtendedName);
            var edges = graph.Edges.Where(edge =>
                (first.Contains(edge.GroupIdA) && second.Contains(edge.GroupIdB)) ||
                (first.Contains(edge.GroupIdB) && second.Contains(edge.GroupIdA))).ToArray();
            if (edges.Length != 1)
                throw new NameCollisionException($"Support-pair reference '{reference}' selects {edges.Length} connected edges; use an unambiguous provenance-qualified edge.");
            result.Add(edges[0].Name);
        }
        return result;
    }
}
