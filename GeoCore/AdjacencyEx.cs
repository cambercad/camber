namespace GeoCore
{
    // This class supports the case where more than 2 triangles share a single edge BUT
    // only if the configuration is valid - so an even number of triangles must be sharing the edge
    // and when they are sorted around the edge, each pair must have opposite winding
    public static class AdjacencyEx
    {
        public static TriangleAdjacency[] BuildAdjacencyInformation(List<Rat3Hybrid> vertices, List<Tri> triangles, bool throwOnOpenEdge = false)
        {
            var edgeToTriangles = BuildEdgeToTrianglesMap(triangles, vertices.Count + triangles.Count);

            foreach (var v in edgeToTriangles)
            {
                var list = v.Value;
                if (list.Count > 2)
                    SortAroundEdge(vertices, triangles, list, v.Key.Item1, v.Key.Item2);
            }

            int l = triangles.Count;
            TriangleAdjacency[] adj = new TriangleAdjacency[l];
            for (int i = 0; i < l; ++i)
                adj[i] = new TriangleAdjacency() { NeighbourAB = -1, NeighbourBC = -1, NeighbourCA = -1 };

            foreach (var v in edgeToTriangles)
            {
                var e = v.Value;
                if(throwOnOpenEdge)
                    if (e.Count % 2 != 0)
                        throw new Exception();

                if (e.Count == 1)
                {
                    // Nothing to do
                }
                else if (e.Count == 2)
                {
                    Tri t1 = triangles[e[0]];
                    Tri t2 = triangles[e[1]];

                    adj[e[0]].AddNeighbour(t1, t2, e[1]);
                    adj[e[1]].AddNeighbour(t2, t1, e[0]);
                }
                else if(e.Count > 2)
                {
                    if (e.Count % 2 != 0)
                        throw new Exception();
                    for (int i = 0; i < e.Count; i += 2)
                    {
                        Tri t1 = triangles[e[i + 0]];
                        Tri t2 = triangles[e[i + 1]];

                        adj[e[i + 0]].AddNeighbour(t1, t2, e[i + 1]);
                        adj[e[i + 1]].AddNeighbour(t2, t1, e[i + 0]);
                    }
                }
            }

            return adj;
        }

        public static Dictionary<(int, int), List<int>> BuildEdgeToTrianglesMap(List<Tri> triangles)
        {
            return BuildEdgeToTrianglesMap(triangles, triangles.Count * 3);
        }

        public static HashSet<(int, int)> CollectNonManifoldEdges(Dictionary<(int, int), List<int>> edgeToTriangles)
        {
            var result = new HashSet<(int, int)>();
            foreach (var kv in edgeToTriangles)
            {
                if (kv.Value.Count != 2)
                    result.Add(kv.Key);
            }

            return result;
        }

        public static bool IsNonManifoldEdge(Dictionary<(int, int), List<int>> edgeToTriangles, int v1, int v2)
        {
            var edge = NormalizeEdge(v1, v2);
            return !edgeToTriangles.TryGetValue(edge, out var tris) || tris.Count != 2;
        }

        public static (int, int) NormalizeEdge(int v1, int v2)
        {
            return v1 < v2 ? (v1, v2) : (v2, v1);
        }

        private static Dictionary<(int, int), List<int>> BuildEdgeToTrianglesMap(List<Tri> triangles, int capacity)
        {
            var edgeToTriangles = new Dictionary<(int, int), List<int>>(capacity);

            for (int i = 0; i < triangles.Count; ++i)
            {
                var t = triangles[i];
                var a = t.A;
                var b = t.B;
                var c = t.C;

                if (a < 0)
                    continue;
                if (a == b || b == c || a == c)
                    throw new Exception();

                AddEdge(edgeToTriangles, a, b, i);
                AddEdge(edgeToTriangles, b, c, i);
                AddEdge(edgeToTriangles, c, a, i);
            }

            return edgeToTriangles;
        }

        private static void AddEdge(Dictionary<(int, int), List<int>> edgeToTriangles, int v1, int v2, int triId)
        {
            // Ensure consistent edge representation (smaller index first)
            var edge = NormalizeEdge(v1, v2);

            if (edgeToTriangles.TryGetValue(edge, out var list))
                list.Add(triId);
            else            
                edgeToTriangles.Add(edge, new List<int>() { triId });            
        }



        public static void SortAroundEdge(List<Rat3Hybrid> vertices, List<Tri> allTris, List<int> tris, int edgeStart, int edgeEnd)
        {
            if (tris.Count <= 2) return;

            if (tris.Count % 2 != 0)
                throw new Exception("Valid, watertight geometry must have an even number of triangles sharing the same edge");

            var A = vertices[edgeStart];
            var B = vertices[edgeEnd];
            var e = B - A;

            tris.Sort((it1, it2) =>
            {
                var t1 = allTris[it1];
                var t2 = allTris[it2];
                var remaining1 = t1.GetRemaining(edgeStart, edgeEnd);
                var remaining2 = t2.GetRemaining(edgeStart, edgeEnd);
                var v1 = vertices[remaining1] - A;
                var v2 = vertices[remaining2] - A;

                // orientation test around axis AB
                var cross = Rat3Hybrid.Cross(v1, v2);

                var sign = Rat3Hybrid.Dot(e, cross).Sign();

                if (sign > 0) return -1; // t1 before t2
                if (sign < 0) return 1;  // t2 before t1

                // Collinear case (same angle) → tie-break by distance
                var d1 = Rat3Hybrid.Dot(v1, v1);
                var d2 = Rat3Hybrid.Dot(v2, v2);

                return d1.CompareTo(d2);
            });

            //Validate - neighboring triangles must ALWAYS have alternating directions of their shared edge
            for (int i = 0; i < tris.Count; ++i)
            {
                var curr = i;
                var next = (i + 1) % tris.Count;

                var x = EdgeSameDir(allTris[tris[curr]], edgeStart, edgeEnd);
                var y = EdgeSameDir(allTris[tris[next]], edgeStart, edgeEnd);

                if (x == y)
                    throw new Exception();
            }

            //Apply cyclic shift such that always 0,1 and 2,3 etc. form and adjacency pair
            var (a, b) = FindSmallestAnglePair(vertices, allTris, tris, edgeStart, edgeEnd, out var normals);

            //// Check if the normals of a and b are facing each other
            //var nA = normals[a];
            //var nB = normals[b];
            //var dot = Rat3Hybrid.Dot(nA, nB).Sign();
            //if (dot == 0)
            //    throw new Exception();
            //bool closestFaceEachOther = dot < 0;

            var distanceSign = SignedDistancePointPlane(A, normals[a], vertices[allTris[tris[b]].GetRemaining(edgeStart, edgeEnd)]).Sign();
            if (distanceSign == 0)
                throw new Exception();
            bool closestFaceEachOther = distanceSign > 0;

            CyclicShiftInPlace(closestFaceEachOther ? a : b, tris);
        }

        private static BigRationalHybrid SignedDistancePointPlane(Rat3Hybrid pointOnPlane, Rat3Hybrid planeNormal, Rat3Hybrid queryPoint)
        {
            return queryPoint.X * planeNormal.X + queryPoint.Y * planeNormal.Y + queryPoint.Z * planeNormal.Z - Rat3Hybrid.Dot(pointOnPlane, planeNormal);
        }

        private static void CyclicShiftInPlace<T>(int newFirstElement, List<T> tris)
        {
            for (int i = 0; i < newFirstElement; ++i)
                tris.Add(tris[i]);

            if (newFirstElement > 0)
                tris.RemoveRange(0, newFirstElement);
        }

        private static bool EdgeSameDir(Tri t, int edgeStart, int edgeEnd)
        {
            int idS = t.IndexOf(edgeStart);
            int idE = t.IndexOf(edgeEnd);
            if (idS < 0 || idE < 0)
                throw new Exception();

            if (idS > idE)
                idE += 3;
            return idE - idS == 1;
        }

        static Rat3Hybrid Normal(Rat3Hybrid A, Rat3Hybrid B, Rat3Hybrid C)
        {
            Rat3Hybrid e = B - A;
            Rat3Hybrid v = C - A;
            return Rat3Hybrid.Cross(e, v);
        }
        public static (int i, int j) FindSmallestAnglePair(List<Rat3Hybrid> vertices, List<Tri> allTris, List<int> tris, int edgeStart, int edgeEnd, out Rat3Hybrid[] normals)
        {
            int n = tris.Count;
            if (n < 2) throw new Exception("Need at least 2 triangles");

            // Precompute normals
            normals = new Rat3Hybrid[n];
            for (int i = 0; i < n; i++)
            {
                var tri = allTris[tris[i]];
                normals[i] = Normal(vertices[tri.A], vertices[tri.B], vertices[tri.C]);
            }

            int bestI = 0;
            int bestJ = 1;

            // We'll compare fractions:
            // (dot^2) / (len1^2 * len2^2)
            // using cross multiplication to avoid division

            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n; // cyclic!

                var n1 = normals[i];
                var n2 = normals[j];

                var dot = Rat3Hybrid.Dot(n1, n2);

                var len1 = Rat3Hybrid.Dot(n1, n1);
                var len2 = Rat3Hybrid.Dot(n2, n2);

                // current score
                var lhs = dot * dot;

                // best score
                var bn1 = normals[bestI];
                var bn2 = normals[bestJ];

                var bdot = Rat3Hybrid.Dot(bn1, bn2);
                var blen1 = Rat3Hybrid.Dot(bn1, bn1);
                var blen2 = Rat3Hybrid.Dot(bn2, bn2);

                var rhs = bdot * bdot;

                // Compare:
                // lhs / (len1*len2)  vs rhs / (blen1*blen2)
                // Cross multiply:

                var left = lhs * blen1 * blen2;
                var right = rhs * len1 * len2;

                // Larger cos² means smaller angle
                if (left > right)
                {
                    bestI = i;
                    bestJ = j;
                }
            }

            return (bestI, bestJ);
        }
    }
}
