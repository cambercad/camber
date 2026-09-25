using System;
using System.Collections.Generic;

namespace GeoCore
{
    /// <summary>Edge-connected triangle components, optionally welded by exact rational positions.</summary>
    public static class MeshConnectivity
    {
        public static List<List<int>> FindConnectedTriangleComponents(
            IList<Tri> triangles,
            IList<int> triangleIndices,
            TriangleAdjacency[] adjacency,
            IList<int> groupIdPerTriangle = null,
            int? groupId = null,
            HashSet<(int, int)> nonManifoldEdges = null)
        {
            if (triangles == null) throw new ArgumentNullException(nameof(triangles));
            if (triangleIndices == null) throw new ArgumentNullException(nameof(triangleIndices));
            if (adjacency == null || adjacency.Length != triangles.Count)
                throw new ArgumentException("Adjacency must contain one entry per triangle.", nameof(adjacency));
            if (groupId.HasValue && (groupIdPerTriangle == null || groupIdPerTriangle.Count != triangles.Count))
                throw new ArgumentException("A group id is required for every triangle when filtering by group.", nameof(groupIdPerTriangle));

            var included = new HashSet<int>(triangleIndices);
            var visited = new HashSet<int>();
            var components = new List<List<int>>();

            foreach (int start in triangleIndices)
            {
                if (start < 0 || start >= triangles.Count || start >= adjacency.Length)
                    throw new ArgumentOutOfRangeException(nameof(triangleIndices), $"Triangle index {start} is out of range.");
                if (groupId.HasValue && groupIdPerTriangle[start] != groupId.Value)
                    continue;
                if (!visited.Add(start))
                    continue;

                var component = new List<int>();
                var queue = new Queue<int>();
                queue.Enqueue(start);
                while (queue.Count > 0)
                {
                    int index = queue.Dequeue();
                    component.Add(index);
                    var triangle = triangles[index];
                    QueueAcross(triangle.A, triangle.B, adjacency[index].NeighbourAB);
                    QueueAcross(triangle.B, triangle.C, adjacency[index].NeighbourBC);
                    QueueAcross(triangle.C, triangle.A, adjacency[index].NeighbourCA);

                    void QueueAcross(int a, int b, int neighbour)
                    {
                        if (neighbour < 0 || neighbour >= triangles.Count || !included.Contains(neighbour) ||
                            (nonManifoldEdges != null && nonManifoldEdges.Contains(NormalizeEdge(a, b))) ||
                            (groupId.HasValue && (groupIdPerTriangle == null || groupIdPerTriangle[neighbour] != groupId.Value)))
                            return;
                        if (visited.Add(neighbour))
                            queue.Enqueue(neighbour);
                    }
                }
                components.Add(component);
            }
            return components;
        }

        /// <summary>
        /// Finds edge-connected components after merging vertices whose precise rational
        /// positions are exactly equal. Merely touching at a vertex does not connect triangles.
        /// </summary>
        public static List<List<int>> FindExactPositionComponents(
            IList<Rat3Hybrid> precisePositions,
            IList<Tri> triangles)
        {
            if (precisePositions == null) throw new ArgumentNullException(nameof(precisePositions));
            if (triangles == null) throw new ArgumentNullException(nameof(triangles));
            if (triangles.Count == 0) return new List<List<int>>();

            var pointMap = DuplicatePointRemover.DuplicateMap(precisePositions);
            var welded = new List<Tri>(triangles.Count);
            for (int i = 0; i < triangles.Count; i++)
            {
                var triangle = triangles[i];
                if (triangle.A < 0 || triangle.B < 0 || triangle.C < 0 ||
                    triangle.A >= precisePositions.Count || triangle.B >= precisePositions.Count || triangle.C >= precisePositions.Count)
                    throw new ArgumentException($"Triangle {i} references a vertex outside the precise-position array.", nameof(triangles));

                int a = pointMap[triangle.A], b = pointMap[triangle.B], c = pointMap[triangle.C];
                if (a == b || b == c || c == a)
                    throw new ArgumentException($"Exact-position welding collapses triangle {i}.", nameof(triangles));
                welded.Add(new Tri(a, b, c));
            }

            var adjacency = Adjacency.BuildAdjacencyInformation(welded, out _, skipInvalidEdges: true);
            var allIndices = new List<int>(welded.Count);
            for (int i = 0; i < welded.Count; i++) allIndices.Add(i);
            return FindConnectedTriangleComponents(welded, allIndices, adjacency);
        }

        private static (int, int) NormalizeEdge(int a, int b) => a < b ? (a, b) : (b, a);
    }
}
