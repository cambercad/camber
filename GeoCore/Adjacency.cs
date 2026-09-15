namespace GeoCore
{
    public struct TriangleAdjacency
    {
        public int NeighbourAB;
        public int NeighbourBC;
        public int NeighbourCA;

        public void AddNeighbour(Tri self, Tri neighbour, int neighbourIndex)
        {
            if (neighbour.ContainsAll(self.B, self.C))
                NeighbourBC = neighbourIndex;
            else if (neighbour.ContainsAll(self.C, self.A))
                NeighbourCA = neighbourIndex;
            else if (neighbour.ContainsAll(self.A, self.B))
                NeighbourAB = neighbourIndex;
            /*if (!neighbour.Contains(self.A))
                NeighbourBC = neighbourIndex;
            else if (!neighbour.Contains(self.B))
                NeighbourCA = neighbourIndex;
            else if (!neighbour.Contains(self.C))
                NeighbourAB = neighbourIndex;*/
            /*if (!self.Contains(neighbour.A))
                NeighbourBC = neighbourIndex;
            else if (!self.Contains(neighbour.B))
                NeighbourCA = neighbourIndex;
            else if (!self.Contains(neighbour.C))
                NeighbourAB = neighbourIndex;*/
        }

        public bool Contains(int triId)
        {
            return NeighbourAB == triId || 
                NeighbourBC == triId || 
                NeighbourCA == triId;
        }

        public override string ToString()
        {
            return NeighbourAB + "   " + NeighbourBC + "   " + NeighbourCA;
        }
    }

    public enum TriEdgeType //: byte
    {
        None, AB, BC, CA
    }

    public struct TriangleEdge
    {
        public int Start, End;

        public int NeighbourIndex1;
        public int NeighbourIndex2;
        public TriEdgeType Neighbour1EdgeType;
        public TriEdgeType Neighbour2EdgeType;
        public bool Reversed;

        public TriangleEdge(int start, int end, int neighbourIndex1, TriEdgeType neighbour1EdgeType, bool reversed)
        {
            Start = start;
            End = end;

            NeighbourIndex1 = neighbourIndex1;
            NeighbourIndex2 = -1;
            Neighbour1EdgeType = neighbour1EdgeType;
            Neighbour2EdgeType = TriEdgeType.None;
            Reversed = reversed;
        }

        public void Reverse()
        {
            var tmp = Start;
            Start = End;
            End = tmp;
            Reversed = !Reversed;
        }

        public bool IsOnBorder { get { return NeighbourIndex2 == -1 || NeighbourIndex1 == -1; } }

        public void SwapNeighbours()
        {
            int tmp = NeighbourIndex1;
            NeighbourIndex1 = NeighbourIndex2;
            NeighbourIndex2 = tmp;

            TriEdgeType tm = Neighbour1EdgeType;
            Neighbour1EdgeType = Neighbour2EdgeType;
            Neighbour2EdgeType = tm;
        }
    }


    public class TriangleWithAdjacency
    {
        public int A;
        public int B;
        public int C;
        public int NeighbourAB;
        public int NeighbourBC;
        public int NeighbourCA;


        public TriangleWithAdjacency() { }

        public TriangleWithAdjacency(int a, int b, int c, int neighbourAB, int neighbourBC, int neighbourCA)
        {
            A = a; B = b; C = c;
            NeighbourAB = neighbourAB;
            NeighbourBC = neighbourBC;
            NeighbourCA = neighbourCA;
        }

        public void SetNeighbour(/*Tri self, */TriangleWithAdjacency neighbour, int neighbourIndex)
        {
            if (neighbour.ContainsAll(B, C))
                NeighbourBC = neighbourIndex;
            else if (neighbour.ContainsAll(C, A))
                NeighbourCA = neighbourIndex;
            else if (neighbour.ContainsAll(A, B))
                NeighbourAB = neighbourIndex;


            if ((NeighbourAB == NeighbourBC && NeighbourAB >= 0 && NeighbourBC >= 0) ||
                (NeighbourAB == NeighbourCA && NeighbourAB >= 0 && NeighbourCA >= 0) ||
                (NeighbourBC == NeighbourCA && NeighbourBC >= 0 && NeighbourCA >= 0))
            {

            }

            //else
            //{
            //    throw new Exception();
            //}
        }

        public void SetNeighbour(/*Tri self, */Tri neighbour, int neighbourIndex)
        {
            if (neighbour.ContainsAll(B, C))
            {
                if (NeighbourBC != -1)
                {

                }
                NeighbourBC = neighbourIndex;
            }
            else if (neighbour.ContainsAll(C, A))
            {
                if (NeighbourCA != -1)
                {

                }
                NeighbourCA = neighbourIndex;
            }
            else if (neighbour.ContainsAll(A, B))
            {
                if (NeighbourAB != -1)
                {

                }
                NeighbourAB = neighbourIndex;
            }
            /*if (!self.Contains(neighbour.A))
                NeighbourBC = neighbourIndex;
            else if (!self.Contains(neighbour.B))
                NeighbourCA = neighbourIndex;
            else if (!self.Contains(neighbour.C))
                NeighbourAB = neighbourIndex;*/
        }

        public void SetNeighbour(int first, int second, int newIndex)
        {
            if ((A == first && B == second) || (A == second && B == first))
                NeighbourAB = newIndex;
            else if ((B == first && C == second) || (B == second && C == first))
                NeighbourBC = newIndex;
            else if ((C == first && A == second) || (C == second && A == first))
                NeighbourCA = newIndex;
            else throw new Exception();
        }

        //public bool ContainsAll(int index1, int index2) { return Contains(index1) && Contains(index2); }
        public bool ContainsAll(int index1, int index2) { return (A == index1 || B == index1 || C == index1) && (A == index2 || B == index2 || C == index2); }
        public bool Contains(int index) { return A == index || B == index || C == index; }

        public int GetRemaining(int first, int second)
        {
            //bool a = A == first || A == second;
            //bool b = B == first || B == second;
            //if (a && b) return C;
            //if (a) return B;
            //return A;
            if ((A == first && B == second) || (A == second && B == first))
                return C;
            if ((B == first && C == second) || (B == second && C == first))
                return A;
            if ((C == first && A == second) || (C == second && A == first))
                return B;
            throw new Exception();
        }

        public void ClearNeighbours()
        {
            NeighbourAB = -1; NeighbourBC = -1; NeighbourCA = -1;
        }

        public void Replace(int oldIndex, int newIndex)
        {
            if (A == oldIndex) A = newIndex;
            else if (B == oldIndex) B = newIndex;
            else if (C == oldIndex) C = newIndex;
            else throw new Exception();
        }

        //private static void Swap<T>(ref T a, ref T b)
        //{
        //    T tmp = a;
        //    a = b;
        //    b = tmp;
        //}

        //public void Sort()
        //{
        //    if (A > B) { Swap(ref A, ref B); }
        //    if (B > C) { Swap(ref B, ref C); }
        //    if (A > B) { Swap(ref A, ref B); }
        //}
    }

    public static class Adjacency
    {
        public static List<List<int>> ExtractBoundaryLoops(IList<Tri> triangles, out List<bool> closed)
        {
            return ExtractBoundaryLoops(triangles, out closed, out _);
        }

        public static List<List<int>> ExtractBoundaryLoops(IList<Tri> triangles, out List<bool> closed, out TriangleEdge[] edges)
        {
            edges = BuildEdgeList(triangles);

            List<TriangleEdge> borderEdges = new List<TriangleEdge>((int)(Math.Sqrt(edges.Length) + 4));
            for (int i = 0; i < edges.Length; ++i)
            {
                TriangleEdge e = edges[i];
                if (e.IsOnBorder)
                    borderEdges.Add(edges[i]);
            }

            if (borderEdges.Count == 0)
            {
                closed = new List<bool>();
                return new List<List<int>>();
            }

            return HashSegmentConnector.ConnectAndResolve(borderEdges, delegate (TriangleEdge e) { return e.Start; }, delegate (TriangleEdge e) { return e.End; }, out closed);
        }

        public static TriangleWithAdjacency[] BuildAdjacencyInformationEx(IList<Tri> triangles, bool skipInvalidEdges = false)
        {
            TriangleEdge[] edges = BuildEdgeList(triangles, skipInvalidEdges);
            return BuildAdjacencyInformationEx(triangles, edges);
        }

        public static TriangleWithAdjacency[] BuildAdjacencyInformationEx(IList<Tri> triangles, out TriangleEdge[] edges, bool skipInvalidEdges = false)
        {
            edges = BuildEdgeList(triangles, skipInvalidEdges);
            return BuildAdjacencyInformationEx(triangles, edges);
        }

        public static TriangleWithAdjacency[] BuildAdjacencyInformationEx(IList<Tri> triangles, TriangleEdge[] edges)
        {
            int l = triangles.Count;
            TriangleWithAdjacency[] adj = new TriangleWithAdjacency[l];
            for (int i = 0; i < l; ++i)
            {
                Tri tri = triangles[i];
                adj[i] = new TriangleWithAdjacency() { A = tri.A, B = tri.B, C = tri.C, NeighbourAB = -1, NeighbourBC = -1, NeighbourCA = -1 };
            }

            l = edges.Length;
            for (int i = 0; i < l; ++i)
            {
                TriangleEdge e = edges[i];
                if (e.NeighbourIndex2 != -1)
                {
                    Tri t1 = triangles[e.NeighbourIndex1];
                    Tri t2 = triangles[e.NeighbourIndex2];

                    adj[e.NeighbourIndex1].SetNeighbour(/*t1, */t2, e.NeighbourIndex2);
                    adj[e.NeighbourIndex2].SetNeighbour(/*t2, */t1, e.NeighbourIndex1);
                }
            }

            return adj;
        }

        public static TriangleEdge[] BuildEdgeList(IList<Tri> triangles, bool skipInvalidEdges = false)
        {
            return BuildEdgeList(delegate (int index) { return triangles[index]; }, 0, triangles.Count, skipInvalidEdges);
        }

        public static TriangleEdge[] BuildEdgeList(Func<int, Tri> triangles, int triStart, int triEnd, bool skipInvalidEdges = false)
        {
            int counter;
            Dictionary<int, Dictionary<int, TriangleEdge>> edges = BuildEdgeListDict(triangles, triStart, triEnd, out counter, skipInvalidEdges);
            TriangleEdge[] res = new TriangleEdge[counter];
            counter = 0;
            foreach (Dictionary<int, TriangleEdge> outer in edges.Values)
                foreach (TriangleEdge inner in outer.Values)
                    res[counter++] = inner;
            return res;
        }

        public static TriangleAdjacency[] BuildAdjacencyInformation(IList<Tri> triangles)
        {
            TriangleEdge[] edges;
            return BuildAdjacencyInformation(triangles, out edges);
        }

        public static TriangleAdjacency[] BuildAdjacencyInformation(IList<Tri> triangles, out TriangleEdge[] edges, bool skipInvalidEdges = false)
        {
            edges = BuildEdgeList(triangles, skipInvalidEdges);
            return BuildAdjacencyInformation(triangles, edges);
        }


        public static TriangleAdjacency[] BuildAdjacencyInformation(IList<Tri> triangles, TriangleEdge[] edges)
        {
            int l = triangles.Count;
            TriangleAdjacency[] adj = new TriangleAdjacency[l];
            for (int i = 0; i < l; ++i)
                adj[i] = new TriangleAdjacency() { NeighbourAB = -1, NeighbourBC = -1, NeighbourCA = -1 };

            l = edges.Length;
            for (int i = 0; i < l; ++i)
            {
                TriangleEdge e = edges[i];
                if (e.NeighbourIndex2 >= 0)
                {
                    Tri t1 = triangles[e.NeighbourIndex1];
                    Tri t2 = triangles[e.NeighbourIndex2];

                    adj[e.NeighbourIndex1].AddNeighbour(t1, t2, e.NeighbourIndex2);
                    adj[e.NeighbourIndex2].AddNeighbour(t2, t1, e.NeighbourIndex1);
                }
            }

            return adj;
        }


        private static Dictionary<int, Dictionary<int, TriangleEdge>> BuildEdgeListDict(Func<int, Tri> triangles, int triStart, int triEnd, out int counter, bool skipInvalidEdges = false)
        {
            //TODO: change outer dictionary to array
            Dictionary<int, Dictionary<int, TriangleEdge>> edges = new Dictionary<int, Dictionary<int, TriangleEdge>>();
            counter = 0;



            //int l = triangles.Count;
            //for (int i = 0; i < l; ++i)
            for (int i = triStart; i < triEnd; ++i)
            {
                Tri tri = triangles(i);
                if (tri.A < 0 || tri.ContainsDuplicateIndex())
                    continue;

                Dictionary<int, TriangleEdge> dict;
                TriangleEdge edge;

                int s, e;
                bool reversed;

                if (tri.A < tri.B) { s = tri.A; e = tri.B; reversed = false; }
                else { s = tri.B; e = tri.A; reversed = true; }
                if (edges.TryGetValue(s, out dict))
                {
                    if (dict.TryGetValue(e, out edge))
                    {
                        if (edge.NeighbourIndex2 != -1)
                        {
                            if (skipInvalidEdges)
                            {
                                edge.NeighbourIndex1 = -2;
                                edge.NeighbourIndex2 = -2;
                                edge.Neighbour2EdgeType = TriEdgeType.None; ;
                                //Write back (Edge is a value type, a struct!!!)
                                dict[e] = edge;
                            }
                            else
                                throw new Exception("This can happen if there are triangle with duplicate indices (degenerate zero area triangle)");
                        }
                        else
                        {
                            edge.NeighbourIndex2 = i;
                            edge.Neighbour2EdgeType = TriEdgeType.AB;
                            //Write back (Edge is a value type, a struct!!!)
                            dict[e] = edge;
                        }
                    }
                    else
                    {
                        dict.Add(e, new TriangleEdge(s, e, i, TriEdgeType.AB, reversed)); ++counter;
                    }
                }
                else
                {
                    edges.Add(s, new Dictionary<int, TriangleEdge> { { e, new TriangleEdge(s, e, i, TriEdgeType.AB, reversed) } }); ++counter;
                }

                if (tri.B < tri.C) { s = tri.B; e = tri.C; reversed = false; }
                else { s = tri.C; e = tri.B; reversed = true; }
                if (edges.TryGetValue(s, out dict))
                {
                    if (dict.TryGetValue(e, out edge))
                    {
                        if (edge.NeighbourIndex2 != -1)
                        {
                            if (skipInvalidEdges)
                            {
                                edge.NeighbourIndex1 = -2;
                                edge.NeighbourIndex2 = -2;
                                edge.Neighbour2EdgeType = TriEdgeType.None; ;
                                //Write back (Edge is a value type, a struct!!!)
                                dict[e] = edge;
                            }
                            else
                                throw new Exception("This can happen if there are triangle with duplicate indices (degenerate zero area triangle)");
                        }
                        edge.NeighbourIndex2 = i;
                        edge.Neighbour2EdgeType = TriEdgeType.BC;
                        //Write back (Edge is a value type, a struct!!!)
                        dict[e] = edge;
                    }
                    else
                    {
                        dict.Add(e, new TriangleEdge(s, e, i, TriEdgeType.BC, reversed)); ++counter;
                    }
                }
                else
                {
                    edges.Add(s, new Dictionary<int, TriangleEdge> { { e, new TriangleEdge(s, e, i, TriEdgeType.BC, reversed) } }); ++counter;
                }

                if (tri.C < tri.A) { s = tri.C; e = tri.A; reversed = false; }
                else { s = tri.A; e = tri.C; reversed = true; }
                if (edges.TryGetValue(s, out dict))
                {
                    if (dict.TryGetValue(e, out edge))
                    {
                        if (edge.NeighbourIndex2 != -1)
                        {
                            if (skipInvalidEdges)
                            {
                                edge.NeighbourIndex1 = -2;
                                edge.NeighbourIndex2 = -2;
                                edge.Neighbour2EdgeType = TriEdgeType.None; ;
                                //Write back (Edge is a value type, a struct!!!)
                                dict[e] = edge;
                            }
                            else
                                throw new Exception("This can happen if there are triangle with duplicate indices (degenerate zero area triangle)");
                        }
                        edge.NeighbourIndex2 = i;
                        edge.Neighbour2EdgeType = TriEdgeType.CA;
                        //Write back (Edge is a value type, a struct!!!)
                        dict[e] = edge;
                    }
                    else
                    {
                        dict.Add(e, new TriangleEdge(s, e, i, TriEdgeType.CA, reversed)); ++counter;
                    }
                }
                else
                {
                    edges.Add(s, new Dictionary<int, TriangleEdge> { { e, new TriangleEdge(s, e, i, TriEdgeType.CA, reversed) } }); ++counter;
                }
            }
            return edges;
        }
    }
}
