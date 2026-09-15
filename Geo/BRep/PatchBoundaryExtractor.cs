using GeoCore;

namespace Geo.BRep
{
    public static class PatchBoundaryExtractor
    {
        public delegate bool PatchUvLookup(int fromVertex, int toVertex, out Vec2D uv);

        public sealed class BoundaryLoop
        {
            public List<int> VertexIndices = new();
            public List<Vec3D> Positions = new();
            public List<Vec2D> UvPoints = new();
        }

        public static List<BoundaryLoop> ExtractBoundaryLoops(
            IList<Vec3D> positions,
            IList<Tri> allTriangles,
            IList<int> groupIdPerTriangle,
            int groupId,
            PatchUvLookup patchUvLookup = null)
        {
            var triIndices = new List<int>();
            for (int i = 0; i < allTriangles.Count; i++)
            {
                if (groupIdPerTriangle[i] == groupId)
                    triIndices.Add(i);
            }

            if (triIndices.Count == 0)
                return new List<BoundaryLoop>();

            var boundaryEdges = ExtractBoundaryEdges(allTriangles, triIndices);
            if (boundaryEdges.Count == 0)
                return new List<BoundaryLoop>();

            return ChainBoundaryLoops(boundaryEdges, positions, patchUvLookup);
        }

        private static List<Int2> ExtractBoundaryEdges(IList<Tri> triangles, List<int> triIndices)
        {
            var edgeCount = new Dictionary<long, int>();
            var edgeVerts = new Dictionary<long, Int2>();

            foreach (int triIndex in triIndices)
            {
                var tri = triangles[triIndex];
                CountEdge(edgeCount, edgeVerts, tri.A, tri.B);
                CountEdge(edgeCount, edgeVerts, tri.B, tri.C);
                CountEdge(edgeCount, edgeVerts, tri.C, tri.A);
            }

            var boundary = new List<Int2>();
            foreach (var kv in edgeCount)
            {
                if (kv.Value == 1)
                    boundary.Add(edgeVerts[kv.Key]);
            }
            return boundary;
        }

        private static void CountEdge(Dictionary<long, int> edgeCount, Dictionary<long, Int2> edgeVerts, int a, int b)
        {
            long key = Algorithms.Key(a, b);
            if (!edgeVerts.ContainsKey(key))
                edgeVerts[key] = new Int2(a, b);
            edgeCount.TryGetValue(key, out int c);
            edgeCount[key] = c + 1;
        }

        private static List<BoundaryLoop> ChainBoundaryLoops(
            List<Int2> boundaryEdges,
            IList<Vec3D> positions,
            PatchUvLookup patchUvLookup)
        {
            var adjacency = new Dictionary<int, List<int>>();
            foreach (var e in boundaryEdges)
            {
                AddAdj(adjacency, e.X, e.Y);
                AddAdj(adjacency, e.Y, e.X);
            }

            var used = new HashSet<long>();
            var loops = new List<BoundaryLoop>();

            foreach (var startEdge in boundaryEdges)
            {
                long startKey = Algorithms.Key(startEdge.X, startEdge.Y);
                if (used.Contains(startKey))
                    continue;

                var loop = new BoundaryLoop();
                int current = startEdge.X;
                int next = startEdge.Y;
                loop.VertexIndices.Add(current);
                AddLoopPoint(loop, positions, current, patchUvLookup, current, next);

                int guard = 0;
                while (guard++ < boundaryEdges.Count + 2)
                {
                    used.Add(Algorithms.Key(current, next));
                    loop.VertexIndices.Add(next);
                    AddLoopPoint(loop, positions, next, patchUvLookup, current, next);
                    if (next == startEdge.X && loop.VertexIndices.Count > 2)
                        break;

                    if (!adjacency.TryGetValue(next, out var neighbors))
                        break;

                    int candidate = -1;
                    foreach (int n in neighbors)
                    {
                        long k = Algorithms.Key(next, n);
                        if (!used.Contains(k))
                        {
                            candidate = n;
                            break;
                        }
                    }

                    if (candidate < 0)
                        break;
                    current = next;
                    next = candidate;
                }

                if (loop.VertexIndices.Count >= 3)
                    loops.Add(loop);
            }

            return loops;
        }

        private static void AddLoopPoint(
            BoundaryLoop loop,
            IList<Vec3D> positions,
            int vertexIndex,
            PatchUvLookup patchUvLookup,
            int edgeFrom,
            int edgeTo)
        {
            loop.Positions.Add(positions[vertexIndex]);
            if (patchUvLookup != null && edgeFrom >= 0 && edgeTo >= 0)
            {
                if (vertexIndex == edgeTo && patchUvLookup(edgeFrom, edgeTo, out var uv))
                    loop.UvPoints.Add(uv);
                else if (vertexIndex == edgeFrom && patchUvLookup(edgeTo, edgeFrom, out uv))
                    loop.UvPoints.Add(uv);
                else
                    loop.UvPoints.Add(new Vec2D(0, 0));
            }
            else
                loop.UvPoints.Add(new Vec2D(0, 0));
        }

        private static void AddAdj(Dictionary<int, List<int>> adjacency, int from, int to)
        {
            if (!adjacency.TryGetValue(from, out var list))
            {
                list = new List<int>();
                adjacency[from] = list;
            }
            list.Add(to);
        }
    }
}
