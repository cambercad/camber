using Geo;
using GeoCore;

namespace GeoPy;

/// <summary>Canonical mesh names shared by Python queries and display packing.</summary>
internal static class MeshSelectableNames
{
    static string Prefix(AnchorMesh mesh) => mesh.Name ?? "mesh";

    public static string CurveName(AnchorMesh mesh, int edgeIndex, int stripIndex)
    {
        var edge = mesh.GroupEdges[edgeIndex];
        string reference = mesh.GetEdgeReference(edgeIndex);
        if (edge.LineStrips3D.Count > 1)
            reference += "_" + stripIndex;
        return Prefix(mesh) + ":" + reference;
    }

    internal static string PatchName(AnchorMesh mesh, int groupId)
    {
        return mesh.GetCurrentPatchName(groupId);
    }

    public static List<string> Patches(AnchorMesh mesh)
    {
        mesh.EnsureCoplanarPostProcessed();
        var result = new List<string>();
        var seen = new HashSet<int>();
        foreach (int groupId in mesh.Mesh.GetTriangleGroups())
            if (seen.Add(groupId)) result.Add(Prefix(mesh) + ":" + PatchName(mesh, groupId));
        return result;
    }

    public static List<string> Curves(AnchorMesh mesh)
    {
        mesh.EnsureCoplanarPostProcessed();
        var result = new List<string>();
        if (mesh.GroupEdges == null) return result;
        for (int edgeIndex = 0; edgeIndex < mesh.GroupEdges.Count; edgeIndex++)
        {
            var edge = mesh.GroupEdges[edgeIndex];
            if (edge.LineStrips3D == null) continue;
            for (int i = 0; i < edge.LineStrips3D.Count; i++)
                result.Add(CurveName(mesh, edgeIndex, i));
        }
        return result;
    }

    public static List<string> Points(AnchorMesh mesh)
    {
        mesh.EnsureCoplanarPostProcessed();
        var namesByPosition = new Dictionary<(double X, double Y, double Z), string>();
        void Add(string name, Vec3D point)
        {
            var key = (point.X, point.Y, point.Z);
            if (!namesByPosition.TryGetValue(key, out string previous) ||
                string.CompareOrdinal(name, previous) < 0)
                namesByPosition[key] = name;
        }

        if (mesh.GroupEdges == null) return new List<string>();
        for (int edgeIndex = 0; edgeIndex < mesh.GroupEdges.Count; edgeIndex++)
        {
            var edge = mesh.GroupEdges[edgeIndex];
            if (edge.LineStrips3D == null) continue;
            for (int i = 0; i < edge.LineStrips3D.Count; i++)
            {
                var strip = edge.LineStrips3D[i];
                string curve = CurveName(mesh, edgeIndex, i);
                Add(curve + "@0.000", strip.EvaluateUniform(0.0));
                Add(curve + "@0.500", strip.EvaluateUniform(0.5));
                Add(curve + "@1.000", strip.EvaluateUniform(1.0));
                if (strip.IsClosed())
                {
                    Add(curve + "@0.250", strip.EvaluateUniform(0.25));
                    Add(curve + "@0.750", strip.EvaluateUniform(0.75));
                }
            }
        }
        return namesByPosition.OrderBy(item => item.Key.X).ThenBy(item => item.Key.Y).ThenBy(item => item.Key.Z)
            .Select(item => item.Value).ToList();
    }

    public static List<string> FeatureReferences(AnchorMesh mesh, IEnumerable<string> names)
    {
        mesh.EnsureCoplanarPostProcessed();
        var result = new List<string>();
        string prefix = Prefix(mesh) + ":";
        foreach (string name in names)
        {
            if (name == null) throw new ArgumentException("Curve name is null.", nameof(names));
            if (!name.StartsWith(prefix, StringComparison.Ordinal))
            {
                result.Add(name);
                continue;
            }
            string local = name.Substring(prefix.Length);
            bool found = false;
            for (int edgeIndex = 0; edgeIndex < mesh.GroupEdges.Count; edgeIndex++)
            {
                var edge = mesh.GroupEdges[edgeIndex];
                if (edge.LineStrips3D == null) continue;
                for (int i = 0; i < edge.LineStrips3D.Count; i++)
                {
                    if (CurveName(mesh, edgeIndex, i) == name) { result.Add(mesh.GetEdgeReference(edgeIndex)); found = true; break; }
                }
                if (found) break;
            }
            if (!found) result.Add(local);
        }
        return result;
    }
}
