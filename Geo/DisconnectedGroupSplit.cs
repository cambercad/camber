using GeoCore;
using GeoMeta;

namespace Geo
{
    /// <summary>
    /// Splits a triangle group that consists of multiple edge-connected islands into
    /// separately named faces. Component 0 keeps the original name and group id;
    /// further components get <c>origin_1</c>, <c>origin_2</c>, … and new group ids.
    /// </summary>
    public static class DisconnectedGroupSplit
    {
        /// <summary>
        /// Same-group, edge-adjacent BFS into connected components (skips non-manifold edges).
        /// Shared by planar retriangulation and face-component naming.
        /// </summary>
        public static List<List<int>> SplitGroupIntoComponents(
            List<Tri> triangles,
            List<int> triIndices,
            int groupId,
            TriangleAdjacency[] globalAdjacency,
            List<int> groupIdPerTriangle,
            HashSet<(int, int)> nonManifoldEdges)
        {
            var triSet = new HashSet<int>(triIndices);
            var visited = new HashSet<int>();
            var result = new List<List<int>>();

            foreach (int startTriIndex in triIndices)
            {
                if (!visited.Add(startTriIndex))
                    continue;

                var component = new List<int>();
                var queue = new Queue<int>();
                queue.Enqueue(startTriIndex);

                while (queue.Count > 0)
                {
                    int triIndex = queue.Dequeue();
                    component.Add(triIndex);

                    var tri = triangles[triIndex];
                    TryQueueNeighbour(tri.A, tri.B);
                    TryQueueNeighbour(tri.B, tri.C);
                    TryQueueNeighbour(tri.C, tri.A);

                    void TryQueueNeighbour(int v1, int v2)
                    {
                        if (nonManifoldEdges != null && nonManifoldEdges.Contains(NormalizeEdge(v1, v2)))
                            return;

                        int neighbour = GetNeighbourAcrossEdge(tri, globalAdjacency[triIndex], v1, v2);
                        if (neighbour < 0 || !triSet.Contains(neighbour) || groupIdPerTriangle[neighbour] != groupId)
                            return;

                        if (visited.Add(neighbour))
                            queue.Enqueue(neighbour);
                    }
                }

                result.Add(component);
            }

            return result;
        }

        /// <summary>
        /// For every group with more than one connected island: keep component 0 on the
        /// original id/name; assign new ids and <see cref="EntityNaming.FormatPatchComponentName"/>
        /// names to the rest. Clones <paramref name="surfaceMetaData"/> entries for new names.
        /// </summary>
        /// <param name="allocateGroupIds">
        /// Called with the number of new ids needed; returns the first new id
        /// (caller also advances its global allocator).
        /// </param>
        /// <returns>True if any group was split.</returns>
        public static bool SplitDisconnectedGroups(
            List<Tri> triangles,
            List<Vec3D> positions,
            List<int> groupIdPerTriangle,
            Dictionary<int, string> groupIdToName,
            Dictionary<string, SurfaceMetaData> surfaceMetaData,
            Func<int, int> allocateGroupIds)
        {
            if (triangles == null || groupIdPerTriangle == null || groupIdToName == null)
                return false;
            if (triangles.Count == 0 || triangles.Count != groupIdPerTriangle.Count)
                return false;

            TriangleAdjacency[] globalAdjacency = Adjacency.BuildAdjacencyInformation(triangles);
            var edgeToTriangles = AdjacencyEx.BuildEdgeToTrianglesMap(triangles);
            var nonManifoldEdges = AdjacencyEx.CollectNonManifoldEdges(edgeToTriangles);
            var triangleIdsPerGroup = CoplanarGroupFusion.ExtractTriangleIdsPerGroup(groupIdPerTriangle);

            // Collect renames first so we can allocate a contiguous block of new ids.
            var pending = new List<(int oldGroupId, string originName, List<List<int>> components)>();
            foreach (var kv in triangleIdsPerGroup.OrderBy(x => x.Key))
            {
                int groupId = kv.Key;
                if (!groupIdToName.TryGetValue(groupId, out string originName))
                    continue;

                var components = SplitGroupIntoComponents(
                    triangles, kv.Value, groupId, globalAdjacency, groupIdPerTriangle, nonManifoldEdges);
                if (components.Count <= 1)
                    continue;

                SortComponentsSpatially(components, triangles, positions);
                pending.Add((groupId, originName, components));
            }

            if (pending.Count == 0)
                return false;

            int newIdCount = 0;
            foreach (var item in pending)
                newIdCount += item.components.Count - 1;

            int nextId = allocateGroupIds(newIdCount);

            foreach (var (oldGroupId, originName, components) in pending)
            {
                // Component 0 keeps oldGroupId / originName.
                for (int i = 1; i < components.Count; i++)
                {
                    int newGroupId = nextId++;
                    string newName = EntityNaming.FormatPatchComponentName(originName, i);

                    if (groupIdToName.ContainsKey(newGroupId))
                        throw new NameCollisionException($"Group id already in use: {newGroupId}");
                    if (groupIdToName.Values.Contains(newName))
                        throw new NameCollisionException($"Patch name already registered: '{newName}'.");

                    groupIdToName[newGroupId] = newName;

                    if (surfaceMetaData != null && surfaceMetaData.TryGetValue(originName, out var meta))
                        surfaceMetaData[newName] = meta.Clone();

                    foreach (int triIndex in components[i])
                        groupIdPerTriangle[triIndex] = newGroupId;
                }
            }

            return true;
        }

        /// <summary>
        /// Lexicographic min vertex (X, Y, Z) — same key as <c>GroupEdgeExtractor.SortGroupEdgesSpatially</c>.
        /// </summary>
        public static void SortComponentsSpatially(
            List<List<int>> components,
            List<Tri> triangles,
            List<Vec3D> positions)
        {
            if (positions == null || components.Count <= 1)
                return;

            components.Sort((a, b) =>
            {
                Vec3D minA = MinVertexInComponent(a, triangles, positions);
                Vec3D minB = MinVertexInComponent(b, triangles, positions);
                if (minA.X != minB.X)
                    return minA.X.CompareTo(minB.X);
                if (minA.Y != minB.Y)
                    return minA.Y.CompareTo(minB.Y);
                return minA.Z.CompareTo(minB.Z);
            });
        }

        private static Vec3D MinVertexInComponent(List<int> triIndices, List<Tri> triangles, List<Vec3D> positions)
        {
            Vec3D? smallest = null;
            foreach (int triIndex in triIndices)
            {
                var tri = triangles[triIndex];
                Consider(positions[tri.A]);
                Consider(positions[tri.B]);
                Consider(positions[tri.C]);
            }

            return smallest ?? new Vec3D(0, 0, 0);

            void Consider(Vec3D p)
            {
                if (smallest == null ||
                    p.X < smallest.Value.X ||
                    (p.X == smallest.Value.X && p.Y < smallest.Value.Y) ||
                    (p.X == smallest.Value.X && p.Y == smallest.Value.Y && p.Z < smallest.Value.Z))
                {
                    smallest = p;
                }
            }
        }

        private static int GetNeighbourAcrossEdge(Tri tri, TriangleAdjacency adjacency, int v1, int v2)
        {
            if (EdgeMatches(tri.A, tri.B, v1, v2))
                return adjacency.NeighbourAB;
            if (EdgeMatches(tri.B, tri.C, v1, v2))
                return adjacency.NeighbourBC;
            if (EdgeMatches(tri.C, tri.A, v1, v2))
                return adjacency.NeighbourCA;
            return -1;
        }

        private static bool EdgeMatches(int e1, int e2, int v1, int v2) =>
            (e1 == v1 && e2 == v2) || (e1 == v2 && e2 == v1);

        private static (int, int) NormalizeEdge(int v1, int v2) =>
            v1 < v2 ? (v1, v2) : (v2, v1);
    }
}
