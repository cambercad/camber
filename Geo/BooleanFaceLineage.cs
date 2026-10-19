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

    internal void ClassifyReference(BooleanFragments fragments)
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

    private static readonly bool VerifyAgainstReference =
        Environment.GetEnvironmentVariable("CAMBER_VERIFY_LINEAGE") == "1";

    private static T[] AsArray<T>(IReadOnlyList<T> list)
    {
        if (list is T[] array) return array;
        var copy = new T[list.Count];
        for (int i = 0; i < copy.Length; i++) copy[i] = list[i];
        return copy;
    }

    /// <summary>
    /// Same result as <see cref="ClassifyReference"/> but on flat int arrays: edge incidence is a
    /// CSR table built from an open-addressing hash, group data is cached per source group and
    /// string keys are interned to ints. Iteration order of every set/list is preserved.
    /// </summary>
    internal void Classify(BooleanFragments fragments)
    {
        if (VerifyAgainstReference)
        {
            ClassifyReference(fragments);
            var expected = Output;
            ClassifyFast(fragments);
            var actual = Output;
            if (expected.Count != actual.Count) throw new Exception("lineage verify: count");
            for (int i = 0; i < expected.Count; i++)
            {
                var e = expected[i];
                var a = actual[i];
                if (e.RootKey != a.RootKey || e.Split != a.Split || e.Supported != a.Supported ||
                    e.Reference != a.Reference || !e.Separators.SequenceEqual(a.Separators) ||
                    !e.Roots.SequenceEqual(a.Roots))
                    throw new Exception("lineage verify: mismatch at " + i);
            }
            Console.WriteLine("  lineage verified: " + expected.Count);
            return;
        }
        ClassifyFast(fragments);
    }

    private sealed class GroupInfo
    {
        public FaceLineage Lineage;
        public bool Planar;
        public int RootKeyId;
        public int LineageId;
    }

    private void ClassifyFast(BooleanFragments fragments)
    {
        int count = fragments.Triangles.Count;
        var triangles = AsArray(fragments.Triangles);
        var sources = AsArray(fragments.Sources);
        var retained = AsArray(fragments.Retained);

        // ---- Per-triangle source group data (cached per source group) ----
        var infoByGroupA = new Dictionary<int, GroupInfo>();
        var infoByGroupB = new Dictionary<int, GroupInfo>();
        var rootKeyIds = new Dictionary<string, int>(StringComparer.Ordinal);
        var lineageIds = new Dictionary<FaceLineage, int>(ReferenceEqualityComparer.Instance);
        var lineageById = new List<FaceLineage>();
        var rootKeyById = new List<string>();
        var triInfo = new GroupInfo[count];
        bool[] isA = new bool[count];
        for (int i = 0; i < count; i++)
        {
            var source = sources[i];
            bool fromA = source.MeshOrigin == MeshOrigin.MeshA;
            isA[i] = fromA;
            var mesh = fromA ? first : second;
            int id = mesh.Mesh.TrianglesEx[source.SourceTriangleIndex].GroupId;
            var cache = fromA ? infoByGroupA : infoByGroupB;
            if (!cache.TryGetValue(id, out var info))
            {
                var lineage = mesh.FaceLineages[id];
                info = new GroupInfo
                {
                    Lineage = lineage,
                    Planar = mesh.surfaceMetaData.TryGetValue(mesh.groupIdToExtendedName[id], out var metadata) &&
                        metadata?.SurfaceType == SurfaceType.Planar
                };
                if (!rootKeyIds.TryGetValue(lineage.RootKey, out info.RootKeyId))
                {
                    info.RootKeyId = rootKeyIds.Count;
                    rootKeyIds.Add(lineage.RootKey, info.RootKeyId);
                    rootKeyById.Add(lineage.RootKey);
                }
                if (!lineageIds.TryGetValue(lineage, out info.LineageId))
                {
                    info.LineageId = lineageById.Count;
                    lineageIds.Add(lineage, info.LineageId);
                    lineageById.Add(lineage);
                }
                cache.Add(id, info);
            }
            triInfo[i] = info;
        }
        var rootKeyOf = new int[count];
        var lineageOf = new int[count];
        for (int i = 0; i < count; i++)
        {
            rootKeyOf[i] = triInfo[i].RootKeyId;
            lineageOf[i] = triInfo[i].LineageId;
        }

        // ---- Edge incidence (CSR): edge id per (triangle, side), incident entries in (i, side) order ----
        int entries = count * 3;
        var edgeOf = new int[entries];
        int edgeCount = BuildEdgeIds(triangles, edgeOf);
        var edgeStart = new int[edgeCount + 1];
        for (int k = 0; k < entries; k++) edgeStart[edgeOf[k] + 1]++;
        var aCount = new int[edgeCount];
        for (int k = 0; k < entries; k++)
            if (isA[k / 3]) aCount[edgeOf[k]]++;
        for (int e = 0; e < edgeCount; e++) edgeStart[e + 1] += edgeStart[e];
        var fill = new int[edgeCount];
        var edgeTris = new int[entries];
        for (int k = 0; k < entries; k++)
        {
            int e = edgeOf[k];
            edgeTris[edgeStart[e] + fill[e]++] = k / 3;
        }

        // ---- Regions: connected MeshA triangles with identical source ancestry and retention ----
        var component = new int[count];
        Array.Fill(component, -1);
        var regions = new List<List<int>>();
        var queue = new Queue<int>();
        for (int seed = 0; seed < count; seed++)
        {
            if (!isA[seed] || component[seed] >= 0) continue;
            int id = regions.Count;
            var region = new List<int>();
            regions.Add(region);
            queue.Clear();
            queue.Enqueue(seed);
            component[seed] = id;
            while (queue.TryDequeue(out int current))
            {
                region.Add(current);
                for (int side = 0; side < 3; side++)
                {
                    int e = edgeOf[current * 3 + side];
                    if (aCount[e] != 2) continue;
                    // aCount == 2: exactly two MeshA entries; both must share the current root key.
                    int first2 = -1, second2 = -1;
                    bool ok = true;
                    for (int p = edgeStart[e]; p < edgeStart[e + 1]; p++)
                    {
                        int other = edgeTris[p];
                        if (!isA[other]) continue;
                        if (rootKeyOf[other] != rootKeyOf[current]) { ok = false; break; }
                        if (first2 < 0) first2 = other; else second2 = other;
                    }
                    if (!ok || second2 < 0) continue;
                    if (component[first2] < 0 && retained[first2] == retained[current])
                    {
                        component[first2] = id;
                        queue.Enqueue(first2);
                    }
                    if (component[second2] < 0 && retained[second2] == retained[current])
                    {
                        component[second2] = id;
                        queue.Enqueue(second2);
                    }
                }
            }
        }

        int regionCount = regions.Count;
        var boundaries = new HashSet<string>[regionCount];
        var newSeparators = new HashSet<string>[regionCount];
        var uncertain = new HashSet<int>();

        // ---- Removed regions: which surviving regions do they separate, and by which cutter faces ----
        var neighbors = new HashSet<int>();
        var interfaces = new List<(int Edge, int Region)>();
        for (int id = 0; id < regionCount; id++)
        {
            if (retained[regions[id][0]]) continue;
            neighbors.Clear();
            interfaces.Clear();
            foreach (int triangle in regions[id])
                for (int side = 0; side < 3; side++)
                {
                    int e = edgeOf[triangle * 3 + side];
                    bool manifold = aCount[e] == 2;
                    for (int p = edgeStart[e]; p < edgeStart[e + 1]; p++)
                    {
                        int other = edgeTris[p];
                        if (component[other] >= 0 && retained[other] && rootKeyOf[other] == rootKeyOf[triangle])
                        {
                            if (!manifold) { uncertain.Add(component[other]); continue; }
                            neighbors.Add(component[other]);
                            interfaces.Add((e, component[other]));
                        }
                    }
                }
            if (neighbors.Count < 2) continue; // A hole borders only one surviving region.
            foreach (var (edge, region) in interfaces)
            {
                bool any = false;
                for (int p = edgeStart[edge]; p < edgeStart[edge + 1]; p++)
                {
                    int index = edgeTris[p];
                    if (isA[index] || !retained[index]) continue;
                    any = true;
                    (newSeparators[region] ??= new HashSet<string>(StringComparer.Ordinal))
                        .UnionWith(triInfo[index].Lineage.Roots);
                }
                if (!any) uncertain.Add(region);
            }
        }

        // ---- Retained regions: prune old split ancestors that no longer physically border the region ----
        var seenLineage = new HashSet<int>();
        for (int id = 0; id < regionCount; id++)
        {
            if (!retained[regions[id][0]]) continue;
            seenLineage.Clear();
            foreach (int triangle in regions[id])
                for (int side = 0; side < 3; side++)
                {
                    int e = edgeOf[triangle * 3 + side];
                    if (aCount[e] > 2)
                    {
                        uncertain.Add(id);
                        continue;
                    }
                    for (int p = edgeStart[e]; p < edgeStart[e + 1]; p++)
                    {
                        int other = edgeTris[p];
                        if (retained[other] && rootKeyOf[other] != rootKeyOf[triangle] &&
                            seenLineage.Add(lineageOf[other]))
                            (boundaries[id] ??= new HashSet<string>(StringComparer.Ordinal))
                                .UnionWith(lineageById[lineageOf[other]].Roots);
                    }
                }
        }

        var keptCounts = new Dictionary<string, int>(StringComparer.Ordinal);
        for (int id = 0; id < regionCount; id++)
        {
            var region = regions[id];
            if (!retained[region[0]]) continue;
            string key = rootKeyById[rootKeyOf[region[0]]];
            keptCounts[key] = keptCounts.TryGetValue(key, out int n) ? n + 1 : 1;
        }

        var regionLineage = new FaceLineage[regionCount];
        var stamp = new int[lineageById.Count];
        Array.Fill(stamp, -1);
        var distinct = new List<FaceLineage>();
        for (int id = 0; id < regionCount; id++)
        {
            var region = regions[id];
            int seed = region[0];
            if (!retained[seed]) continue;
            distinct.Clear();
            bool allPlanar = true;
            foreach (int index in region)
            {
                int lid = lineageOf[index];
                if (stamp[lid] != id) { stamp[lid] = id; distinct.Add(lineageById[lid]); }
                allPlanar &= triInfo[index].Planar;
            }
            var old = FaceLineage.Merge(distinct);
            var separators = new List<string>();
            var boundarySet = boundaries[id];
            if (boundarySet != null)
                foreach (var s in old.Separators)
                    if (boundarySet.Contains(s)) separators.Add(s);
            if (newSeparators[id] != null) separators.AddRange(newSeparators[id]);
            bool split = old.Split || keptCounts[old.RootKey] > 1;
            regionLineage[id] = new FaceLineage(old.Roots, separators, split,
                old.Supported && (!split || (allPlanar && !uncertain.Contains(id))));
        }
        var output = new List<FaceLineage>();
        for (int i = 0; i < count; i++)
            if (retained[i])
                output.Add(component[i] >= 0 ? regionLineage[component[i]] : triInfo[i].Lineage);
        Output = output;
    }

    /// <summary>Assigns a dense id to every undirected edge; entry k is side (k % 3) of triangle (k / 3).</summary>
    private static int BuildEdgeIds(Tri[] triangles, int[] edgeOf)
    {
        int count = triangles.Length;
        int capacity = 16;
        while (capacity < count * 2) capacity <<= 1;
        var keys = new long[capacity];
        var ids = new int[capacity];
        Array.Fill(ids, -1);
        int mask = capacity - 1;
        int next = 0;
        for (int i = 0; i < count; i++)
        {
            var t = triangles[i];
            for (int side = 0; side < 3; side++)
            {
                int a = side == 0 ? t.A : side == 1 ? t.B : t.C;
                int b = side == 0 ? t.B : side == 1 ? t.C : t.A;
                if (a > b) (a, b) = (b, a);
                long key = ((long)a << 32) | (uint)b;
                int slot = (int)(((ulong)key * 0x9E3779B97F4A7C15UL) >> 32) & mask;
                while (true)
                {
                    int id = ids[slot];
                    if (id < 0)
                    {
                        keys[slot] = key;
                        ids[slot] = id = next++;
                        edgeOf[i * 3 + side] = id;
                        break;
                    }
                    if (keys[slot] == key)
                    {
                        edgeOf[i * 3 + side] = id;
                        break;
                    }
                    slot = (slot + 1) & mask;
                }
                // load factor: next <= 3*count <= 1.5 * capacity worst case; grow when over 70%
                if (next * 10 > capacity * 7)
                {
                    int newCapacity = capacity << 1;
                    var newKeys = new long[newCapacity];
                    var newIds = new int[newCapacity];
                    Array.Fill(newIds, -1);
                    int newMask = newCapacity - 1;
                    for (int s = 0; s < capacity; s++)
                    {
                        if (ids[s] < 0) continue;
                        int ns = (int)(((ulong)keys[s] * 0x9E3779B97F4A7C15UL) >> 32) & newMask;
                        while (newIds[ns] >= 0) ns = (ns + 1) & newMask;
                        newKeys[ns] = keys[s];
                        newIds[ns] = ids[s];
                    }
                    keys = newKeys;
                    ids = newIds;
                    capacity = newCapacity;
                    mask = newMask;
                }
            }
        }
        return next;
    }
}
