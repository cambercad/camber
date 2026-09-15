using GeoCore;

namespace Geo
{
    public static class DelaunayOptimizer<Arithmetic, Vec> where Arithmetic : struct, ITriangulationOptimizerArithmetic<Vec>
    {
        private static Arithmetic arithmetic = new Arithmetic();

        public static void Optimize(IList<Vec> vertices, IList<Tri> triangles, List<Int2> segments)
        {
            TriangleWithAdjacency[] adjacency = Adjacency.BuildAdjacencyInformationEx(triangles);
            for (int i = 0; i < segments.Count; ++i)
            {
                Int2 s = segments[i];
                if (s.X == s.Y)
                    continue; // This represents a point and can be skipped
                for (int j = 0; j < adjacency.Length; ++j)
                {
                    if (adjacency[j].ContainsAll(s.X, s.Y))
                        adjacency[j].SetNeighbour(s.X, s.Y, -1);
                }
            }

            //HashSet<long> lockedEdges = new HashSet<long>(segments.Count);
            //for (int i = 0; i < segments.Count; ++i)
            //{
            //    var s = segments[i];
            //    lockedEdges.Add(EdgeKey(s.X, s.Y));
            //}

            Optimize(vertices, triangles, adjacency/*, lockedEdges*/);
        }

        public static void Optimize(List<List<Vec>> vertices, IList<Tri> triangles)
        {
            int sum = 0;
            for(int i=0;i<vertices.Count; ++i) 
                sum += vertices[i].Count;
            List<Vec> tmp = new List<Vec>(sum);
            for (int i = 0; i < vertices.Count; ++i)
                tmp.AddRange(vertices[i]);
            Optimize(tmp, triangles);
        }

        public static void Optimize(IList<Vec> vertices, IList<Tri> triangles)
        {
            TriangleWithAdjacency[] adjacency = Adjacency.BuildAdjacencyInformationEx(triangles);

            //Debug: find border
            //List<int> dbg = new List<int>();
            //for (int i = 0; i < adjacency.Length; ++i)
            //{
            //    TriangleWithAdjacency t = adjacency[i];
            //    if (t.NeighbourAB < 0)
            //        dbg.Add(i);
            //    if (t.NeighbourBC < 0)
            //        dbg.Add(i);
            //    if (t.NeighbourCA < 0)
            //        dbg.Add(i);
            //}

            int count = triangles.Count;

            Optimize(vertices, triangles, adjacency/*, null*/);

            if (triangles.Count != count)
                throw new Exception();
        }

        /*public static long EdgeKey(int a, int b)
        {
            if (a < b)
                return (((long)a) << 32) | ((long)b);
            else
                return (((long)b) << 32) | ((long)a);
        }

        private static bool EdgeIsNotLocked(HashSet<long> lockedEdges, int a, int b)
        {
            if (lockedEdges == null)
                return false;
            var key = EdgeKey(a, b);
            return lockedEdges.Contains(key);
        }*/

        private static void Optimize(IList<Vec> vertices, IList<Tri> triangles, TriangleWithAdjacency[] adjacency/*, HashSet<long> lockedEdges*/)
        {
            var inCircleCache = new Dictionary<(int, int, int, int), InCircleResult>();

            bool success = true;
            int counter = 0;
            while (success)
            {
                success = false;

                for (int i = 0; i < adjacency.Length; ++i)
                {
                    TriangleWithAdjacency tri = adjacency[i];
                    if (/*EdgeIsNotLocked(lockedEdges, tri.A, tri.B) &&*/ FlipIfRequired(i, tri, tri.NeighbourAB, tri.A, tri.B, vertices, adjacency, inCircleCache))
                    {
                        success = true;
                        // The adjacency might have changed
                        tri = adjacency[i];
                    }
                    if (/*EdgeIsNotLocked(lockedEdges, tri.B, tri.C) &&*/ FlipIfRequired(i, tri, tri.NeighbourBC, tri.B, tri.C, vertices, adjacency, inCircleCache))
                    {
                        success = true;                    
                    }

                    if (/*EdgeIsNotLocked(lockedEdges, tri.C, tri.A) &&*/ FlipIfRequired(i, tri, tri.NeighbourCA, tri.C, tri.A, vertices, adjacency, inCircleCache))
                    {
                        success = true;
                        // The adjacency might have changed
                        tri = adjacency[i];
                    }

                }
                ++counter;
                if (counter >= adjacency.Length)
                {
                    break; //Algorithm terminates after maximum n^2 operations (Delaunay-Flip-Algorithm), it is suspicious if we reach this point
                }
            }

            for (int i = 0; i < triangles.Count; ++i)
            {
                TriangleWithAdjacency tri = adjacency[i];
                triangles[i] = new Tri(tri.A, tri.B, tri.C);
            }
        }

        private static bool FlipIfRequired(int i, TriangleWithAdjacency tri, int neighbour, int a, int b, IList<Vec> vertices, IList<TriangleWithAdjacency> adjacency, Dictionary<(int, int, int, int), InCircleResult> inCircleCache)
        {
            if (neighbour >= 0)
            {
                TriangleWithAdjacency tri2 = adjacency[/*tri.NeighbourAB*/neighbour];

                //int a = tri.A;
                //int b = tri.B;


                int ear = tri.GetRemaining(a, b);
                int ear2 = tri2.GetRemaining(a, b);


                //Evaluate the Delaunay-Condition
                //if (PointInCircle2(vertices[ear2], vertices[tri.A], vertices[tri.B], vertices[tri.C], eps))
                if (InCircleCached(vertices, tri.A, tri.B, tri.C, ear2, inCircleCache) == InCircleResult.Inside)
                {
                    tri.Replace(a, ear2);
                    tri2.Replace(b, ear);

                    int triAB = tri.NeighbourAB;
                    int triBC = tri.NeighbourBC;
                    int triCA = tri.NeighbourCA;

                    int tri2AB = tri2.NeighbourAB;
                    int tri2BC = tri2.NeighbourBC;
                    int tri2CA = tri2.NeighbourCA;

                    tri.ClearNeighbours();
                    tri2.ClearNeighbours();

                    //FixNeighbourInfo2(adjacency, i, tri, triAB, triBC, triCA, tri2AB, tri2BC, tri2CA);
                    //FixNeighbourInfo2(adjacency, neighbour, tri2, triAB, triBC, triCA, tri2AB, tri2BC, tri2CA);


                    FixNeighbourInfo(adjacency, i, triAB);
                    FixNeighbourInfo(adjacency, i, triBC);
                    FixNeighbourInfo(adjacency, i, triCA);
                    FixNeighbourInfo(adjacency, i, tri2AB);
                    FixNeighbourInfo(adjacency, i, tri2BC);
                    FixNeighbourInfo(adjacency, i, tri2CA);

                    FixNeighbourInfo(adjacency, neighbour, tri2AB);
                    FixNeighbourInfo(adjacency, neighbour, tri2BC);
                    FixNeighbourInfo(adjacency, neighbour, tri2CA);
                    FixNeighbourInfo(adjacency, neighbour, triAB);
                    FixNeighbourInfo(adjacency, neighbour, triBC);
                    FixNeighbourInfo(adjacency, neighbour, triCA);


                    return true;
                }
            }
            return false;
        }

        private static InCircleResult InCircleCached(IList<Vec> vertices, int ia, int ib, int ic, int id, Dictionary<(int, int, int, int), InCircleResult> inCircleCache)
        {
            var key = (ia, ib, ic, id);
            if (inCircleCache.TryGetValue(key, out InCircleResult cached))
                return cached;

            InCircleResult result = arithmetic.InCircle(vertices[ia], vertices[ib], vertices[ic], vertices[id]);
            inCircleCache[key] = result;
            return result;
        }

        //public static int InCircle(Rat2Hybrid pa, Rat2Hybrid pb, Rat2Hybrid pc, Rat2Hybrid pd)
        //{
        //    BigRationalHybrid adx, ady, bdx, bdy, cdx, cdy;
        //    BigRationalHybrid abdet, bcdet, cadet;
        //    BigRationalHybrid alift, blift, clift;

        //    adx = pa.X - pd.X;
        //    ady = pa.Y - pd.Y;
        //    bdx = pb.X - pd.X;
        //    bdy = pb.Y - pd.Y;
        //    cdx = pc.X - pd.X;
        //    cdy = pc.Y - pd.Y;

        //    abdet = adx * bdy - bdx * ady;
        //    bcdet = bdx * cdy - cdx * bdy;
        //    cadet = cdx * ady - adx * cdy;
        //    alift = adx * adx + ady * ady;
        //    blift = bdx * bdx + bdy * bdy;
        //    clift = cdx * cdx + cdy * cdy;

        //    BigRationalHybrid result = alift * bcdet + blift * cadet + clift * abdet;

        //    if (result > BigRationalHybrid.Zero)
        //        return 1;
        //    else if (result < BigRationalHybrid.Zero)
        //        return -1;
        //    else
        //        return 0;
        //}

        private static void FixNeighbourInfo(IList<TriangleWithAdjacency> adj, int a, int b)
        {
            if (a >= 0 && b >= 0 && a != b)
            {
                TriangleWithAdjacency first = adj[a];
                TriangleWithAdjacency second = adj[b];

                //Tri firstTri = new Tri(first.A, first.B, first.C);
                //Tri secondTri = new Tri(second.A, second.B, second.C);

                first.SetNeighbour(second/*secondTri*/, b);
                second.SetNeighbour(first/*firstTri*/, a);
            }
        }
    }
}
