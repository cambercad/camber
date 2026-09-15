using GeoCore;

namespace Geo
{
    public static class AutoNormals
    {
        /// <summary>
        /// Computes smooth normals with sharp edge detection based on angle threshold.
        /// Supports closed solids and open sheets (manifold-with-boundary): boundary vertex
        /// fans are open strips; non-manifold fans are split into separate strips.
        /// </summary>
        /// <param name="points">Input vertices</param>
        /// <param name="triangles">Input triangles</param>
        /// <param name="angleThresholdRadians">Angle threshold for sharp edge detection</param>
        /// <returns>Array with 3 normals per triangle (one for each corner)</returns>
        public static List<Vec3D> ComputeNormals(List<Vec3D> points, List<Tri> triangles, double angleThresholdRadians)
        {
            var duplicateMap = DuplicatePointRemover.DuplicateMap(points);
            List<Tri> mappedTris = DuplicatePointRemover.MapTriangles(triangles, duplicateMap);

            int numTris = mappedTris.Count;

            Vec3D[] triNormals = new Vec3D[numTris];
            for (int i = 0; i < numTris; i++)
            {
                var tri = mappedTris[i];
                triNormals[i] = GeometricAlgorithms.ComputeTriangleNormal(points[tri.A], points[tri.B], points[tri.C]).Normalized();
            }

            var vertexToTriangles = BuildVertexToTrianglesMap(mappedTris);
            TriangleEdge[] edges;
            var triangleAdjacency = Adjacency.BuildAdjacencyInformation(mappedTris, out edges);

            int l = numTris * 3;
            List<Vec3D> result = new List<Vec3D>(l);
            for (int i = 0; i < l; ++i)
                result.Add(new Vec3D(0));

            for (int vertexIndex = 0; vertexIndex < points.Count; vertexIndex++)
            {
                if (!vertexToTriangles.TryGetValue(vertexIndex, out var adjacentTriangles))
                    continue;
                if (adjacentTriangles.Count == 0)
                    continue;

                var fans = SortTrianglesInFanOrder(vertexIndex, adjacentTriangles, mappedTris);
                for (int i = 0; i < fans.Count; ++i)
                {
                    ComputeVertexNormals(points, vertexIndex, fans[i].Order, fans[i].Closed, mappedTris,
                        triNormals, triangleAdjacency, angleThresholdRadians, result);
                }
            }

            return result;
        }

        private class FanOrder
        {
            public List<int> Order;
            public bool Closed;

            public FanOrder(List<int> order, bool closed)
            {
                Order = order;
                Closed = closed;
            }
        }

        private static List<FanOrder> SortTrianglesInFanOrder(int vertexIndex, List<int> triangleIndices, List<Tri> mappedTris)
        {
            List<Int2> triangleEdges = new List<Int2>(triangleIndices.Count);
            Dictionary<Int2, int> edgeSet = new Dictionary<Int2, int>(triangleIndices.Count);
            for (int i = 0; i < triangleIndices.Count; i++)
            {
                int triIndex = triangleIndices[i];
                var tri = mappedTris[triIndex];
                var edge = tri.GetRemaining(vertexIndex);
                triangleEdges.Add(edge);
                edgeSet.Add(edge, triIndex);
            }

            // Non-manifold vertices (degree > 2 in the opposite-edge graph): split those
            // endpoints with synthetic ids so the connector yields open/closed manifold fans.
            var degree = new Dictionary<int, int>();
            for (int i = 0; i < triangleEdges.Count; i++)
            {
                var e = triangleEdges[i];
                degree.TryGetValue(e.X, out int dx); degree[e.X] = dx + 1;
                degree.TryGetValue(e.Y, out int dy); degree[e.Y] = dy + 1;
            }

            int syntheticId = int.MinValue;
            for (int i = 0; i < triangleEdges.Count; i++)
            {
                var oldEdge = triangleEdges[i];
                int newX = oldEdge.X, newY = oldEdge.Y;

                if (degree[oldEdge.X] > 2) newX = syntheticId++;
                if (degree[oldEdge.Y] > 2) newY = syntheticId++;

                if (newX != oldEdge.X || newY != oldEdge.Y)
                {
                    var newEdge = new Int2(newX, newY);
                    int tri = edgeSet[oldEdge];
                    edgeSet.Remove(oldEdge);
                    edgeSet[newEdge] = tri;
                    triangleEdges[i] = newEdge;
                }
            }

            var parts = HashSegmentConnector.ConnectAndResolve(triangleEdges, delegate (Int2 e) { return e.X; }, delegate (Int2 e) { return e.Y; }, out var closedList);

            List<FanOrder> result = new List<FanOrder>(parts.Count);
            for (int i = 0; i < parts.Count; ++i)
                result.Add(new FanOrder(ProcessStrip(edgeSet, parts[i], closedList[i]), closedList[i]));

            return result;
        }

        private static List<int> ProcessStrip(Dictionary<Int2, int> edgeSet, List<int> strip, bool closed)
        {
            if (closed)
                strip.Add(strip[0]);

            List<int> sortedTriangleIndices = new List<int>(strip.Count);
            for (int j = 1; j < strip.Count; ++j)
            {
                Int2 edge;
                edge.X = strip[j - 1];
                edge.Y = strip[j];
                int triId;
                if (!edgeSet.TryGetValue(edge, out triId))
                {
                    Algorithms.Swap(ref edge.X, ref edge.Y);
                    if (!edgeSet.TryGetValue(edge, out triId))
                        throw new Exception();
                }

                sortedTriangleIndices.Add(triId);
            }

            return sortedTriangleIndices;
        }

        private static Int2 GetRemaining(this Tri tri, int vertexToExclude)
        {
            if (vertexToExclude == tri.A)
                return new Int2(tri.B, tri.C);
            if (vertexToExclude == tri.B)
                return new Int2(tri.C, tri.A);
            if (vertexToExclude == tri.C)
                return new Int2(tri.A, tri.B);

            throw new Exception("Vertex is not part of the triangle");
        }

        private static Dictionary<int, List<int>> BuildVertexToTrianglesMap(List<Tri> triangles)
        {
            var vertexToTriangles = new Dictionary<int, List<int>>();

            for (int triIndex = 0; triIndex < triangles.Count; triIndex++)
            {
                var tri = triangles[triIndex];

                if (!vertexToTriangles.ContainsKey(tri.A))
                    vertexToTriangles[tri.A] = new List<int>();
                vertexToTriangles[tri.A].Add(triIndex);

                if (!vertexToTriangles.ContainsKey(tri.B))
                    vertexToTriangles[tri.B] = new List<int>();
                vertexToTriangles[tri.B].Add(triIndex);

                if (!vertexToTriangles.ContainsKey(tri.C))
                    vertexToTriangles[tri.C] = new List<int>();
                vertexToTriangles[tri.C].Add(triIndex);
            }

            return vertexToTriangles;
        }

        private static void ComputeVertexNormals(List<Vec3D> points, int vertexIndex, List<int> adjacentTriangles, bool closedLoop,
            List<Tri> triangles, Vec3D[] triNormals, TriangleAdjacency[] triangleAdjacency, double angleThreshold, List<Vec3D> result)
        {
            int numTriangles = adjacentTriangles.Count;
            if (numTriangles == 0)
                return;

            bool[] hasSharpEdge = new bool[numTriangles];
            int start = 0;
            for (int i = 0; i < numTriangles; i++)
            {
                int triIndex = adjacentTriangles[i];
                int nextIndex = (i + 1) % numTriangles;
                int nextTriIndex = adjacentTriangles[nextIndex];

                if (TrianglesShareEdge(triIndex, nextTriIndex, triangleAdjacency))
                {
                    double angle = Vec3DOps.Angle(triNormals[triIndex], triNormals[nextTriIndex]);
                    if (angle > angleThreshold)
                    {
                        hasSharpEdge[i] = true;
                        if (start == 0)
                            start = nextIndex;
                    }
                }
                else
                {
                    hasSharpEdge[i] = true;
                    if (start == 0)
                        start = nextIndex;
                }
            }

            int end = numTriangles + start;
            if (!closedLoop)
            {
                start = 0;
                end = numTriangles;
                hasSharpEdge[numTriangles - 1] = true;
            }

            Vec3D smoothedNormal = new Vec3D(0, 0, 0);
            int first = start;
            for (int k = start; k < end; k++)
            {
                int i = k % numTriangles;

                var triId = adjacentTriangles[i];
                double angleWeight = TriangleAngleAtVertex(points, triangles[triId], vertexIndex);
                smoothedNormal += angleWeight * triNormals[triId];

                if (hasSharpEdge[i])
                {
                    if (smoothedNormal.LengthSquared() > 1e-30)
                        smoothedNormal.Normalize();
                    else
                        smoothedNormal = triNormals[triId];

                    for (int j = first; j <= k; ++j)
                    {
                        triId = adjacentTriangles[j % numTriangles];
                        var tri = triangles[triId];
                        var localId = tri.IndexOf(vertexIndex);
                        int target = triId * 3 + localId;
                        result[target] = smoothedNormal;
                    }

                    smoothedNormal = new Vec3D(0, 0, 0);
                    first = k + 1;
                }
            }

            if (first < numTriangles + start)
            {
                if (smoothedNormal.LengthSquared() > 1e-30)
                    smoothedNormal.Normalize();
                for (int j = first; j < numTriangles + start; ++j)
                {
                    var triId = adjacentTriangles[j % numTriangles];
                    var tri = triangles[triId];
                    var localId = tri.IndexOf(vertexIndex);
                    int target = triId * 3 + localId;
                    result[target] = smoothedNormal.LengthSquared() > 1e-30 ? smoothedNormal : triNormals[triId];
                }
            }
        }

        private static double TriangleAngleAtVertex(List<Vec3D> points, Tri tri, int vertexIndex)
        {
            var remaining = tri.GetRemaining(vertexIndex);
            var p = points[vertexIndex];
            return Vec3DOps.Angle(points[remaining.X] - p, points[remaining.Y] - p);
        }

        private static bool TrianglesShareEdge(int triIndex1, int triIndex2, TriangleAdjacency[] adjacency)
        {
            var adj = adjacency[triIndex1];
            return adj.NeighbourAB == triIndex2 || adj.NeighbourBC == triIndex2 || adj.NeighbourCA == triIndex2;
        }
    }
}
