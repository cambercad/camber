using GeoCore;
using GeoMeta;

namespace Geo;

internal static class FaceLineageNaming
{
    internal static Dictionary<int, FaceLineage> Apply(MeshNormalUV mesh,
        Dictionary<string, int> names, Dictionary<string, SurfaceMetaData> metadata,
        IReadOnlyList<FaceLineage> triangles, Dictionary<int, FaceLineage> inherited,
        HashSet<string> obsolete)
    {
        if (triangles == null) return inherited;
        var oldNames = names.ToDictionary(pair => pair.Value, pair => pair.Key);
        var oldGroups = mesh.GetTriangleGroups();
        var groups = new List<int>(oldGroups);
        var lineages = new Dictionary<int, FaceLineage>();
        var used = new Dictionary<(int Source, string Key), int>();
        var assignedSource = new HashSet<int>();
        names.Clear();
        for (int i = 0; i < groups.Count; i++)
        {
            int source = oldGroups[i];
            FaceLineage lineage = triangles[i];
            var key = (source, lineage.Reference);
            if (!used.TryGetValue(key, out int id))
            {
                id = assignedSource.Add(source) ? source : GeoAPI.ReserveGroupIds(1);
                used.Add(key, id);
                lineages[id] = lineage;
                string label = lineage.Split && lineage.Supported ? lineage.Reference : oldNames[source];
                bool assignedCollision = names.TryGetValue(label, out int previous) && previous != id;
                if (assignedCollision || (label != oldNames[source] && oldNames.Values.Contains(label)))
                {
                    if (assignedCollision) obsolete.Add(label);
                    int suffix = 1;
                    string baseLabel = label;
                    do
                        label = EntityNaming.FormatPatchComponentName(baseLabel, suffix++);
                    while (names.ContainsKey(label) || oldNames.Values.Contains(label));
                }
                names[label] = id;
                if (oldNames[source] != label)
                {
                    obsolete.Add(oldNames[source]);
                    if (metadata.TryGetValue(oldNames[source], out var data)) metadata[label] = data?.Clone();
                }
            }
            groups[i] = id;
        }
        mesh.SetTriangleGroups(groups);
        return lineages;
    }
}
