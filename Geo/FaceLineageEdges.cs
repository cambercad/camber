using GeoCore;
using GeoMeta;

namespace Geo;

internal static class FaceLineageEdges
{
    internal static List<string> Resolve(AnchorMesh mesh, IEnumerable<string> references)
    {
        EdgeGraph graph = null;
        var result = new List<string>();
        foreach (string reference in references)
        {
            if (!EntityNaming.TryParseGroupEdgeAddress(reference, out var pair, requireFullMatch: true))
            {
                mesh.ValidateEntityReference(reference);
                result.Add(reference);
                continue;
            }

            // Current patch names are unique. Resolve their exact edge names
            // before considering ancestor support-pair aliases.
            bool currentFaces = mesh.extendedNameToGroupId.ContainsKey(pair.PatchA) &&
                mesh.extendedNameToGroupId.ContainsKey(pair.PatchB);
            if (currentFaces)
            {
                string currentName = null;
                foreach (var edge in mesh.GroupEdges)
                    if (EntityNaming.MatchesGroupEdgeName(edge.Name, reference))
                    {
                        currentName = edge.Name;
                        break;
                    }
                if (currentName != null)
                {
                    result.Add(currentName);
                    continue;
                }

                if (reference.TrimEnd().EndsWith("]", StringComparison.Ordinal))
                {
                    int count = 0;
                    foreach (var edge in mesh.GroupEdges)
                        if ((edge.GroupIdA == mesh.extendedNameToGroupId[pair.PatchA] &&
                             edge.GroupIdB == mesh.extendedNameToGroupId[pair.PatchB]) ||
                            (edge.GroupIdB == mesh.extendedNameToGroupId[pair.PatchA] &&
                             edge.GroupIdA == mesh.extendedNameToGroupId[pair.PatchB]))
                            count++;
                    if (count > 1)
                        throw new NameCollisionException($"Support-pair reference '{reference}' selects {count} connected edges; use a numbered current edge name.");
                }
                throw new NameCollisionException($"Current edge '{reference}' is not present on this solid.");
            }

            if (!mesh.AmbiguousFaceReferences.Contains(pair.PatchA) &&
                !mesh.AmbiguousFaceReferences.Contains(pair.PatchB))
            {
                mesh.ValidateEntityReference(reference);
                result.Add(reference);
                continue;
            }
            if (!reference.TrimEnd().EndsWith("]", StringComparison.Ordinal))
                throw new NameCollisionException($"Indexed support-pair reference '{reference}' cannot resolve split ancestors; use a current edge name.");

            HashSet<int> Supports(string patch)
            {
                var descendants = new HashSet<int>();
                foreach (var item in mesh.FaceLineages)
                    if (item.Value.Roots.Contains(patch)) descendants.Add(item.Key);
                if (descendants.Count > 0) return descendants;
                mesh.ValidateEntityReference(patch);
                if (mesh.extendedNameToGroupId.TryGetValue(patch, out int id)) descendants.Add(id);
                return descendants;
            }
            var first = Supports(pair.PatchA);
            var second = Supports(pair.PatchB);
            graph ??= new EdgeGraph(mesh.Mesh.Triangles, mesh.Mesh.GetTriangleGroups(), mesh.Mesh.Positions,
                mesh.Mesh.PrecisionPositions, mesh.groupIdToExtendedName);
            GraphEdge match = null;
            int matches = 0;
            foreach (var edge in graph.Edges)
                if ((first.Contains(edge.GroupIdA) && second.Contains(edge.GroupIdB)) ||
                    (first.Contains(edge.GroupIdB) && second.Contains(edge.GroupIdA)))
                {
                    match = edge;
                    matches++;
                }
            if (matches != 1)
                throw new NameCollisionException($"Support-pair reference '{reference}' selects {matches} connected edges; use a current edge name.");
            result.Add(match.Name);
        }
        return result;
    }
}
