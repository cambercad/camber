using CSG;
using GeoCore;

namespace Geo;

/// <summary>Consumes transient CSG incidence; retains only one lineage value per output triangle.</summary>
internal sealed class BooleanFaceLineage
{
    private readonly AnchorMesh first;
    private readonly AnchorMesh second;
    internal List<FaceLineage> Output { get; private set; }

    internal BooleanFaceLineage(AnchorMesh first, AnchorMesh second)
    {
        this.first = first;
        this.second = second;
    }

    internal void Classify(BooleanFragments fragments)
    {
        int count = fragments.Triangles.Count;
        var sourceFaces = new FaceLineage[count];
        var planar = new bool[count];
        var edgeFaces = new Dictionary<(int, int), List<int>>();
        var triangleEdges = new (int, int)[count][];
        for (int i = 0; i < count; i++)
        {
            var source = fragments.Sources[i];
            var mesh = source.MeshOrigin == MeshOrigin.MeshA ? first : second;
            int id = mesh.Mesh.TrianglesEx[source.SourceTriangleIndex].GroupId;
            sourceFaces[i] = mesh.FaceLineages[id];
            planar[i] = mesh.surfaceMetaData.TryGetValue(mesh.groupIdToExtendedName[id], out var metadata) &&
                metadata?.SurfaceType == SurfaceType.Planar;
            var triangle = fragments.Triangles[i];
            triangleEdges[i] = new[] { Edge(triangle.A, triangle.B), Edge(triangle.B, triangle.C), Edge(triangle.C, triangle.A) };
            foreach (var edge in triangleEdges[i])
            {
                if (!edgeFaces.TryGetValue(edge, out var incident)) edgeFaces.Add(edge, incident = new());
                incident.Add(i);
            }
        }

        // Components are formed within the same source face ancestry and same
        // classification. Touching/nonmanifold edges do not join regions.
        var component = Enumerable.Repeat(-1, count).ToArray();
        var regions = new List<List<int>>();
        for (int seed = 0; seed < count; seed++)
        {
            if (fragments.Sources[seed].MeshOrigin != MeshOrigin.MeshA || component[seed] >= 0) continue;
            int id = regions.Count;
            var region = new List<int>();
            regions.Add(region);
            var queue = new Queue<int>();
            queue.Enqueue(seed);
            component[seed] = id;
            while (queue.TryDequeue(out int current))
            {
                region.Add(current);
                foreach (var edge in triangleEdges[current])
                {
                    if (edgeFaces[edge].Count(other => fragments.Sources[other].MeshOrigin == MeshOrigin.MeshA) != 2)
                        continue;
                    var sameFace = edgeFaces[edge].Where(other =>
                        fragments.Sources[other].MeshOrigin == MeshOrigin.MeshA &&
                        sourceFaces[other].RootKey == sourceFaces[current].RootKey).ToArray();
                    if (sameFace.Length != 2) continue;
                    foreach (int other in sameFace)
                        if (component[other] < 0 && fragments.Retained[other] == fragments.Retained[current])
                        {
                            component[other] = id;
                            queue.Enqueue(other);
                        }
                }
            }
        }

        var boundaries = regions.Select(_ => new HashSet<string>(StringComparer.Ordinal)).ToArray();
        var newSeparators = regions.Select(_ => new HashSet<string>(StringComparer.Ordinal)).ToArray();
        var uncertain = new HashSet<int>();
        for (int id = 0; id < regions.Count; id++)
        {
            if (fragments.Retained[regions[id][0]]) continue;
            var neighbors = new HashSet<int>();
            var interfaces = new List<((int, int) Edge, int Region)>();
            foreach (int triangle in regions[id])
                foreach (var edge in triangleEdges[triangle])
                {
                    bool manifold = edgeFaces[edge].Count(other => fragments.Sources[other].MeshOrigin == MeshOrigin.MeshA) == 2;
                    foreach (int other in edgeFaces[edge])
                        if (component[other] >= 0 && fragments.Retained[other] &&
                            sourceFaces[other].RootKey == sourceFaces[triangle].RootKey)
                        {
                            if (!manifold) { uncertain.Add(component[other]); continue; }
                            neighbors.Add(component[other]);
                            interfaces.Add((edge, component[other]));
                        }
                }
            if (neighbors.Count < 2) continue; // A hole borders only one surviving region.
            foreach (var (edge, region) in interfaces)
            {
                var cutterFaces = edgeFaces[edge].Where(index =>
                    fragments.Sources[index].MeshOrigin == MeshOrigin.MeshB && fragments.Retained[index]).ToArray();
                if (cutterFaces.Length == 0) uncertain.Add(region);
                foreach (int cutter in cutterFaces)
                    newSeparators[region].UnionWith(sourceFaces[cutter].Roots);
            }
        }

        // Prune old split ancestors that no longer physically border this region.
        // This is what prevents optional earlier cuts from becoming permanent prefixes.
        for (int id = 0; id < regions.Count; id++)
        {
            if (!fragments.Retained[regions[id][0]]) continue;
            foreach (int triangle in regions[id])
                foreach (var edge in triangleEdges[triangle])
                {
                    if (edgeFaces[edge].Count(other => fragments.Sources[other].MeshOrigin == MeshOrigin.MeshA) > 2)
                    {
                        uncertain.Add(id);
                        continue;
                    }
                    foreach (int other in edgeFaces[edge])
                        if (fragments.Retained[other] && sourceFaces[other].RootKey != sourceFaces[triangle].RootKey)
                            boundaries[id].UnionWith(sourceFaces[other].Roots);
                }
        }
        var keptCounts = regions.Where(region => fragments.Retained[region[0]])
            .GroupBy(region => sourceFaces[region[0]].RootKey).ToDictionary(group => group.Key, group => group.Count());
        var regionLineage = new Dictionary<int, FaceLineage>();
        for (int id = 0; id < regions.Count; id++)
        {
            int seed = regions[id][0];
            if (!fragments.Retained[seed]) continue;
            var old = FaceLineage.Merge(regions[id].Select(index => sourceFaces[index]));
            var separators = old.Separators.Where(boundaries[id].Contains).Concat(newSeparators[id]);
            bool split = old.Split || keptCounts[old.RootKey] > 1;
            // An unchanged curved face still has its unambiguous creation
            // identity. Planar provenance restrictions apply only to splits.
            regionLineage[id] = new FaceLineage(old.Roots, separators, split,
                old.Supported && (!split ||
                    (regions[id].All(index => planar[index]) && !uncertain.Contains(id))));
        }
        Output = new List<FaceLineage>();
        for (int i = 0; i < count; i++)
            if (fragments.Retained[i])
                Output.Add(component[i] >= 0 ? regionLineage[component[i]] : sourceFaces[i]);
    }

    private static (int, int) Edge(int a, int b) => a < b ? (a, b) : (b, a);
}
