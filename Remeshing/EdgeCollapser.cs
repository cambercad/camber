using GeoCore;

namespace Remeshing
{
    public struct FullTriangle
    {
        public Rat3Hybrid A;
        public Rat3Hybrid B;
        public Rat3Hybrid C;

        public FullTriangle(Rat3Hybrid a, Rat3Hybrid b, Rat3Hybrid c)
        {
            A = a;
            B = b;
            C = c;
        }
    }

    public struct TriWithGroupId
    {
        public int A;
        public int B;
        public int C;
        public int GroupId;

        public bool ContainsAll(int a, int b, int c)
        {
            return Contains(a) && Contains(b) && Contains(c);
        }

        public bool ContainsAll(int a, int b)
        {
            return Contains(a) && Contains(b);
        }

        public bool Contains(int a)
        {
            return A == a || B == a || C == a;
        }

        public void Replace(int oldVal, int newVal)
        {
            if (A == oldVal)
                A = newVal;
            if (B == oldVal)
                B = newVal;
            if (C == oldVal)
                C = newVal;
        }
    }


    public static class EdgeCollapser
    {
        [ThreadStatic] private static int[] s_listOpScratch;

        private static int[] EnsureListScratch(int minSize)
        {
            int[] scratch = s_listOpScratch;
            if (scratch == null || scratch.Length < minSize)
                s_listOpScratch = scratch = new int[Math.Max(minSize, 32)];
            return scratch;
        }

        private static void UniqueSorted(List<int> list)
        {
            if (list == null || list.Count <= 1)
                return;
            list.Sort();
            int prev = list[0];
            int counter = 1;
            for (int j = 1; j < list.Count; ++j)
            {
                int value = list[j];
                if (prev != value)
                    list[counter++] = value;
                prev = value;
            }
            if (list.Count != counter)
                list.RemoveRange(counter, list.Count - counter);
        }

        private static int CollectCommon(List<int> a, List<int> b, int[] scratch)
        {
            int count = 0;
            if (a.Count <= b.Count)
            {
                for (int i = 0; i < a.Count; ++i)
                {
                    int v = a[i];
                    for (int j = 0; j < b.Count; ++j)
                    {
                        if (b[j] == v)
                        {
                            scratch[count++] = v;
                            break;
                        }
                    }
                }
            }
            else
            {
                for (int i = 0; i < b.Count; ++i)
                {
                    int v = b[i];
                    for (int j = 0; j < a.Count; ++j)
                    {
                        if (a[j] == v)
                        {
                            scratch[count++] = v;
                            break;
                        }
                    }
                }
            }
            return count;
        }

        private static int CollectUnion(List<int> a, List<int> b, int[] scratch)
        {
            int count = 0;
            for (int i = 0; i < a.Count; ++i)
                scratch[count++] = a[i];
            for (int i = 0; i < b.Count; ++i)
            {
                int v = b[i];
                bool found = false;
                for (int j = 0; j < a.Count; ++j)
                {
                    if (a[j] == v)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                    scratch[count++] = v;
            }
            return count;
        }

        public static void CleanMesh(IList<Rat3Hybrid> points, List<TriWithGroupId> triangles,
            BigRationalHybrid minDist, 
            bool allowVertexRelocation = false,
            bool removeCollinearEdges = true,
            Func<Dictionary<int, FullTriangle>, bool> validateCollapseCallback = null,
            bool[] vertexMustBePreserved = null)
        {

            //Build map that lists all edges a vertex is connected to
            int numPoints = points.Count;
            List<int>[] connectedVertices = new List<int>[numPoints];
            List<int>[] connectedTriangles = new List<int>[numPoints];
            for (int i = 0; i < numPoints; ++i)
            {
                connectedVertices[i] = new List<int>();
                connectedTriangles[i] = new List<int>();
            }

            int numTris = triangles.Count;
            for (int i = 0; i < numTris; ++i)
            {
                TriWithGroupId t = triangles[i]; if (t.A < 0) continue;
                connectedVertices[t.A].Add(t.B);
                connectedVertices[t.A].Add(t.C);

                connectedVertices[t.B].Add(t.A);
                connectedVertices[t.B].Add(t.C);

                connectedVertices[t.C].Add(t.A);
                connectedVertices[t.C].Add(t.B);

                connectedTriangles[t.A].Add(i);
                connectedTriangles[t.B].Add(i);
                connectedTriangles[t.C].Add(i);
            }

            // Neighbor lists must be unique: CollectCommon counts duplicates, which makes
            // TryEdgeCollapse reject collinear rail vertices (intersectCount > 2).
            for (int i = 0; i < connectedVertices.Length; ++i)
            {
                UniqueSorted(connectedVertices[i]);
                UniqueSorted(connectedTriangles[i]);
            }



            ////Debug
            //Random r = new Random();
            //int counter = 0;
            //for (int i = 0; i < triangles.Count; ++i)
            //{
            //    int id = r.Next(triangles.Count);
            //    Tri tri = triangles[id];
            //    if (tri.A >= 0)
            //    {
            //        bool b = TryCollapse(connectedVertices, tri.A, tri.B, 0.5 * (points[tri.A] + points[tri.B]), connectedTriangles, points, triangles);
            //        if (b)
            //            ++counter;

            //        //Verify(points, triangles, connectedVertices, connectedTriangles);
            //    }
            //}







#if Check
            for (int i = 0; i < triangles.Count; ++i)
            {
                TriWithGroupId tri = triangles[i]; if (tri.A < 0) continue;
                if (tri.A == tri.B || tri.A == tri.C || tri.B == tri.C)
                    throw new Exception();
            }

            for (int i = 0; i < connectedVertices.Length; ++i)
            {
                var list = connectedVertices[i];
                if (list.Contains(i))
                    throw new Exception();
                for (int j = 0; j < list.Count; ++j)
                    if (!connectedVertices[list[j]].Contains(i))
                        throw new Exception();
            }
#endif





            //Collect all edges
            //Dictionary<long, Int2> edges = new Dictionary<long, Int2>();
            //for (int i = 0; i < triangles.Count; ++i)
            //{
            //    Tri tri = triangles[i];
            //    long key = BuildKey(tri.A, tri.B);
            //    if (!edges.ContainsKey(key))
            //        edges.Add(key, new Int2(tri.A, tri.B));

            //    key = BuildKey(tri.B, tri.C);
            //    if (!edges.ContainsKey(key))
            //        edges.Add(key, new Int2(tri.B, tri.C));

            //    key = BuildKey(tri.C, tri.A);
            //    if (!edges.ContainsKey(key))
            //        edges.Add(key, new Int2(tri.C, tri.A));
            //}


            if (minDist.Sign() > 0)
            {

                var minDist2 = minDist * minDist;
                //Remove edges that are too short
                bool success = true;
                int numRemoved2 = 0;
                while (success)
                {
                    success = false;
                    for (int i = 0; i < connectedVertices.Length; ++i)
                    {
                        if (connectedVertices[i] == null || connectedVertices[i].Count == 0)
                            continue;

                        var p = points[i];
                        List<int> list = connectedVertices[i];
                        for (int j = 0; j < list.Count; ++j)
                        {
                            var q = points[list[j]];

                            var dx = p.X - q.X;
                            var dy = p.Y - q.Y;
                            var dz = p.Z - q.Z;
                            var l2 = dx * dx + dy * dy + dz * dz;
                            if (l2 < minDist2)
                            {
                                // Skip if either vertex must be preserved
                                if (vertexMustBePreserved != null && (vertexMustBePreserved[i] || vertexMustBePreserved[list[j]]))
                                    continue;
                                
                                if (allowVertexRelocation && TryEdgeCollapse(connectedVertices, i, list[j], (p + q) / new BigRationalHybrid(2), connectedTriangles, points, triangles, false, validateCollapseCallback))
                                {
                                    success = true;
                                    ++numRemoved2;
                                    break;
                                }
                                if (TryEdgeCollapse(connectedVertices, i, list[j], p, connectedTriangles, points, triangles, false, validateCollapseCallback))
                                {
                                    success = true;
                                    ++numRemoved2;
                                    break;
                                }
                                if (TryEdgeCollapse(connectedVertices, i, list[j], q, connectedTriangles, points, triangles, false, validateCollapseCallback))
                                {
                                    success = true;
                                    ++numRemoved2;
                                    break;
                                }
                                if (allowVertexRelocation && TryEdgeCollapse(connectedVertices, i, list[j], (p + q) / new BigRationalHybrid(2), connectedTriangles, points, triangles, true, validateCollapseCallback)) //This will return true for sure because the normal check is suppressed
                                {
                                    success = true;
                                    ++numRemoved2;
                                    break;
                                }
                                else
                                {

                                }
                            }
                        }
                    }
                }


                //Debug: Verify that not 2 neighbouring vertices are closer than minDist
                for (int i = 0; i < connectedVertices.Length; ++i)
                {
                    if (connectedVertices[i] == null || connectedVertices[i].Count == 0)
                        continue;


                    var p = points[i];
                    List<int> list = connectedVertices[i];
                    for (int j = 0; j < list.Count; ++j)
                    {
                        var q = points[list[j]];

                        var l2 = (p - q).LengthSquared();
                        if (l2 < minDist2)
                        {
                            //throw new Exception();
                        }
                    }
                }
            }

            if (removeCollinearEdges)
            {
                //Remove collinear edges using dirty vertex tracking
                //Only re-check vertices affected by collapses, not the entire mesh
                int numRemoved = 0;
                
                // Use Queue for O(1) dequeue + HashSet for O(1) membership check
                Queue<int> dirtyQueue = new Queue<int>();
                HashSet<int> dirtySet = new HashSet<int>();
                for (int i = 0; i < connectedVertices.Length; ++i)
                {
                    if (connectedVertices[i] != null && connectedVertices[i].Count > 0)
                    {
                        dirtyQueue.Enqueue(i);
                        dirtySet.Add(i);
                    }
                }
                
                // Reused across dirty-vertex iterations; grown when a higher valence appears.
                Rat3Hybrid[] collinearDirectionScratch = Array.Empty<Rat3Hybrid>();

                // Process dirty vertices until none remain
                while (dirtyQueue.Count > 0)
                {
                    int i = dirtyQueue.Dequeue();
                    dirtySet.Remove(i);
                    
                    List<int> list = connectedVertices[i];
                    if (list == null || list.Count == 0)
                        continue;

                    // Skip vertices that must be preserved (UV seams, normal discontinuities)
                    if (vertexMustBePreserved != null && vertexMustBePreserved[i])
                        continue;

                    int valence = list.Count;
                    if (collinearDirectionScratch.Length < valence)
                        collinearDirectionScratch = new Rat3Hybrid[valence];

                    int x, y;
                    if (!FindCollinearEdges(i, list, points, collinearDirectionScratch, out x, out y))
                        continue;

                    var pI = points[i];

                    //if (numPlanes <= 2)
                    {
                        // Skip if either endpoint must be preserved
                        if (vertexMustBePreserved != null && (vertexMustBePreserved[x] || vertexMustBePreserved[y]))
                            continue;

                        // Safety check: both edges (x,i) and (i,y) must be adjacent to the same groups
                        HashSet<int> groupsEdgeXI = GetAdjacentGroups(x, i, connectedTriangles, triangles);
                        HashSet<int> groupsEdgeIY = GetAdjacentGroups(i, y, connectedTriangles, triangles);
                        
                        if (!AreSameGroups(groupsEdgeXI, groupsEdgeIY))
                            continue; // Skip this collinear edge - groups don't match

                        var dx = (points[x] - pI).LengthSquared();
                        var dy = (points[y] - pI).LengthSquared();

                        if (dx.Sign() == 0 || dy.Sign() == 0)
                            throw new Exception();

                        if (dx /*<*/> dy)
                        {
                            int tmp = x;
                            x = y;
                            y = tmp;
                        }

                        // Capture neighbors before collapse for dirty marking
                        List<int> neighborsBeforeCollapse = new List<int>(connectedVertices[x]);
                        neighborsBeforeCollapse.AddRange(list);

                        bool collapsed = false;
                        int survivor = -1;
                        
                        if (TryEdgeCollapse(connectedVertices, x, i, points[x], connectedTriangles, points, triangles, false, validateCollapseCallback, true))
                        {
                            ++numRemoved;
                            collapsed = true;
                            survivor = x;
                        }
                        else if (TryEdgeCollapse(connectedVertices, y, i, points[y], connectedTriangles, points, triangles, false, validateCollapseCallback, true))
                        {
                            ++numRemoved;
                            collapsed = true;
                            survivor = y;
                        }
                        
                        if (collapsed)
                        {
                            // Mark affected vertices as dirty
                            if (!dirtySet.Contains(survivor))
                            {
                                dirtyQueue.Enqueue(survivor);
                                dirtySet.Add(survivor);
                            }
                            foreach (int neighbor in neighborsBeforeCollapse)
                            {
                                if (connectedVertices[neighbor] != null && connectedVertices[neighbor].Count > 0 && !dirtySet.Contains(neighbor))
                                {
                                    dirtyQueue.Enqueue(neighbor);
                                    dirtySet.Add(neighbor);
                                }
                            }
                        }
                    }
                }
            }
        }

        private static bool TryCollapse(List<int>[] connectedVertices, int a, int b, int c, List<int>[] connectedTriangles, List<Rat3Hybrid> points,
            IList<TriWithGroupId> triangles, Func<Dictionary<int, FullTriangle>, bool> validateCollapseCallback = null)
        {
            if (!TryEdgeCollapse(connectedVertices, a, b, points[a], connectedTriangles, points, triangles, false, validateCollapseCallback))
            {
                if (!TryEdgeCollapse(connectedVertices, a, b, points[b], connectedTriangles, points, triangles, false, validateCollapseCallback))
                {
                    if (!TryEdgeCollapse(connectedVertices, a, c, points[a], connectedTriangles, points, triangles, false, validateCollapseCallback))
                    {
                        if (!TryEdgeCollapse(connectedVertices, a, c, points[c], connectedTriangles, points, triangles, false, validateCollapseCallback))
                        {
                            if (!TryEdgeCollapse(connectedVertices, b, c, points[b], connectedTriangles, points, triangles, false, validateCollapseCallback))
                            {
                                if (!TryEdgeCollapse(connectedVertices, b, c, points[c], connectedTriangles, points, triangles, false, validateCollapseCallback))
                                {
                                    return false;
                                }
                            }
                        }
                    }
                }
            }
            return true;
        }

        private static bool FindCollinearEdges(
            int index,
            List<int> connectedVertices,
            IList<Rat3Hybrid> points,
            Rat3Hybrid[] directionScratch,
            out int aId,
            out int bId)
        {
            var p = points[index];
            int l = connectedVertices.Count;
            if (l < 2)
            {
                aId = -1;
                bId = -1;
                return false;
            }

            if (directionScratch.Length < l)
                throw new ArgumentException("Scratch buffer is too small.", nameof(directionScratch));

            for (int i = 0; i < l; ++i)
            {
                var direction = points[connectedVertices[i]] - p;
                if (direction.IsZero())
                    throw new InvalidOperationException(
                        $"Collinear-edge search at vertex {index}: neighbor {connectedVertices[i]} coincides with the vertex position.");
                directionScratch[i] = direction;
            }

            for (int i = 0; i < l; ++i)
            {
                var dirA = directionScratch[i];
                for (int j = i + 1; j < l; ++j)
                {
                    if (Rat3Hybrid.AreOppositeCollinear(dirA, directionScratch[j]))
                    {
                        aId = connectedVertices[i];
                        bId = connectedVertices[j];
                        return true;
                    }
                }
            }

            aId = -1;
            bId = -1;
            return false;
        }

        public struct Rat3Plane
        {
            public Rat3Hybrid Normal;
            public BigRationalHybrid PlaneD;

            public BigRationalHybrid SignedDistance(Rat3Hybrid point)
            {
                return Rat3Hybrid.Dot(Normal, point) + PlaneD;
            }
        }

        private static List<Rat3Plane> GetPlanes(IList<Rat3Hybrid> points,
            IList<TriWithGroupId> tris, List<int> triangleCandiates)
        {
            List<Rat3Plane> planes = new List<Rat3Plane>();
            CollectPlanes(points, tris, triangleCandiates, planes);
            return planes;
        }

        private static void CollectPlanes(IList<Rat3Hybrid> points,
            IList<TriWithGroupId> tris, List<int> triangleCandiates, List<Rat3Plane> planes)
        {
            for (int i = 0; i < triangleCandiates.Count; ++i)
            {
                int id = triangleCandiates[i];
                TriWithGroupId tri = tris[id];

                var a = points[tri.A];
                var b = points[tri.B];
                var c = points[tri.C];

                bool success = false;
                for (int j = 0; j < planes.Count; ++j)
                {
                    var p = planes[j];
                    if (AreCoplanar(p, a, b, c))
                    {
                        success = true;
                        break;
                    }
                }

                if (!success)
                {
                    var normal = Rat3Hybrid.Cross(b - a, c - a);
                    if (!normal.IsZero())
                    {
                        // Don't normalize for rational arithmetic
                        BigRationalHybrid planeD = -Rat3Hybrid.Dot(normal, a);
                        planes.Add(new Rat3Plane { Normal = normal, PlaneD = planeD });
                    }
                }
            }
        }

        private static bool AreCoplanar(Rat3Plane p, Rat3Hybrid a, Rat3Hybrid b, Rat3Hybrid c)
        {
            return p.SignedDistance(a).Sign() == 0 && 
                   p.SignedDistance(b).Sign() == 0 && 
                   p.SignedDistance(c).Sign() == 0;
        }

        /// <summary>
        /// Gets the set of group IDs adjacent to an edge (a,b)
        /// Returns the groups of triangles that contain this edge
        /// </summary>
        private static HashSet<int> GetAdjacentGroups(int a, int b, List<int>[] connectedTriangles, IList<TriWithGroupId> triangles)
        {
            HashSet<int> groups = new HashSet<int>();
            
            // Find triangles containing vertex a
            var trisA = connectedTriangles[a];
            if (trisA != null)
            {
                foreach (int triIdx in trisA)
                {
                    var tri = triangles[triIdx];
                    if (tri.A < 0) continue; // Skip deleted triangles
                    
                    // Check if this triangle contains both a and b (i.e., contains the edge)
                    if (tri.ContainsAll(a, b))
                    {
                        groups.Add(tri.GroupId);
                    }
                }
            }
            
            return groups;
        }

        /// <summary>
        /// Checks if two sets of group IDs are the same (order doesn't matter)
        /// </summary>
        private static bool AreSameGroups(HashSet<int> groups1, HashSet<int> groups2)
        {
            if (groups1.Count != groups2.Count)
                return false;
            
            return groups1.SetEquals(groups2);
        }

        private static List<int> Copy(List<int> source)
        {
            List<int> result = new List<int>(source.Count);
            result.AddRange(source);
            return result;
        }

        public static bool TryEdgeCollapse(List<int>[] connectedVertices, int keepOrMove, int remove, Rat3Hybrid newDestination,
            List<int>[] connectedTriangles, IList<Rat3Hybrid> points, IList<TriWithGroupId> triangles, 
            bool suppressNormalCheck = false,
            Func<Dictionary<int, FullTriangle>, bool> validateCollapseCallback = null,
            bool allowBorderEdgeCollapse = false)
        {
            List<int> neighboursA = (connectedVertices[keepOrMove]);
            List<int> neighboursB = (connectedVertices[remove]);
            int[] scratch = EnsureListScratch(neighboursA.Count + neighboursB.Count);
            int intersectCount = CollectCommon(neighboursA, neighboursB, scratch);
            if (intersectCount > 2)
                return false;
            int intersectV0 = intersectCount > 0 ? scratch[0] : -1;
            int intersectV1 = intersectCount > 1 ? scratch[1] : -1;


            List<int> trisA = connectedTriangles[keepOrMove];
            List<int> trisB = connectedTriangles[remove];

            int triIntersectCount = CollectCommon(trisA, trisB, scratch);
            if (triIntersectCount > 2)
                return false; // Non-manifold edge.
            if (triIntersectCount == 0 || (triIntersectCount == 1 && !allowBorderEdgeCollapse))
                return false; // Disconnected or border edge.
            
            // Prevent collapsing edges between different groups (feature edges/group boundaries)
            // Optional check?
            //if (triangles[triIntersection[0]].GroupId != triangles[triIntersection[1]].GroupId)
            //    return false;

            List<int> pointAGroups = new List<int>();
            for (int i = 0; i < trisA.Count; ++i)
            {
                int groupId = triangles[trisA[i]].GroupId;
                if (!pointAGroups.Contains(groupId))
                    pointAGroups.Add(groupId);
            }
            List<int> pointBGroups = new List<int>();
            for (int i = 0; i < trisB.Count; ++i)
            {
                int groupId = triangles[trisB[i]].GroupId;
                if (!pointBGroups.Contains(groupId))
                    pointBGroups.Add(groupId);
            }
            bool fixA = pointAGroups.Count > 2;
            bool fixB = pointBGroups.Count > 2;
            if (fixA && fixB)
                return false;

            if (fixA && newDestination != points[keepOrMove])
                return false;
            if (fixB && newDestination != points[remove])
                return false;

            int triUnionCount = CollectUnion(trisA, trisB, scratch);
            if (!suppressNormalCheck)
            {
                //if (!CanCollapseEdge(a, b, newDestination, intersect, t, points, triangles))
                //    return false;
                int counter = 0;
                for (int i = 0; i < triUnionCount; ++i)
                {
                    TriWithGroupId tri = triangles[scratch[i]];
                    var normal = Rat3Hybrid.Cross(points[tri.B] - points[tri.A], points[tri.C] - points[tri.A]);
                    bool containsA = tri.Contains(keepOrMove);
                    bool containsB = tri.Contains(remove);

                    int c = 0; //Counts the number of tirangle corners that are either a, b or part of the set of all vertices connected to a AND b
                    if (intersectCount > 0 && tri.Contains(intersectV0))
                        ++c;
                    if (intersectCount > 1 && tri.Contains(intersectV1))
                        ++c;
                    if (containsA)
                        ++c;
                    if (containsB)
                        ++c;

                    //if (!(containsA && containsB))
                    //if (!intersect.Contains(t[i]))   
                    //if (check)
                    // Check that normals don't flip (dot product must be positive)
                    // This ensures the triangle orientation is preserved after collapse
                    if (c < 3)
                    {
                        if (containsA)
                        {
                            var n = Normal(points, tri, keepOrMove, newDestination);
                            var dot = Rat3Hybrid.Dot(n, normal);
                            // Reject if normal flips (dot <= 0) or triangle becomes degenerate (n is zero)
                            if (dot.Sign() <= 0)
                                return false;
                        }
                        else if (containsB)
                        {
                            var n = Normal(points, tri, remove, newDestination);
                            var dot = Rat3Hybrid.Dot(n, normal);
                            // Reject if normal flips (dot <= 0) or triangle becomes degenerate (n is zero)
                            if (dot.Sign() <= 0)
                                return false;
                        }
                        else
                            throw new Exception();
                    }
                    else
                        ++counter;
                }
                if (counter > 2)
                {
                    //    throw new Exception("More than 2 triangles sharing and edge???");
                }
            }

            // User-provided validation callback - build list of affected triangles in post-collapse form
            if (validateCollapseCallback != null)
            {
                Dictionary<int, FullTriangle> affectedTriangles = new Dictionary<int, FullTriangle>();
                
                for (int i = 0; i < triUnionCount; ++i)
                {
                    var triId = scratch[i];
                    TriWithGroupId tri = triangles[triId];
                    bool containsA = tri.Contains(keepOrMove);
                    bool containsB = tri.Contains(remove);
                    
                    // Skip triangles that will be deleted (contain both vertices)
                    if (containsA && containsB)
                        continue;
                    
                    // Build triangle in post-collapse form
                    var posA = points[tri.A];
                    var posB = points[tri.B];
                    var posC = points[tri.C];
                    
                    // Replace vertex 'remove' with 'keepOrMove' at newDestination
                    if (tri.A == remove)
                        posA = newDestination;
                    else if (tri.A == keepOrMove)
                        posA = newDestination;
                    
                    if (tri.B == remove)
                        posB = newDestination;
                    else if (tri.B == keepOrMove)
                        posB = newDestination;
                    
                    if (tri.C == remove)
                        posC = newDestination;
                    else if (tri.C == keepOrMove)
                        posC = newDestination;
                    
                    // Skip degenerate triangles (duplicate vertex positions)
                    //if (posA == posB || posA == posC || posB == posC)
                    //    continue;
                    
                    affectedTriangles.Add(triId, new FullTriangle { A = posA, B = posB, C = posC});
                }
                
                // Call user callback - if it returns false, reject the collapse
                if (!validateCollapseCallback(affectedTriangles))
                    return false;
            }

            //We will return true
            
            // FIXED: Update vertex position to the validated destination
            points[keepOrMove] = newDestination;

            neighboursA = Copy(neighboursA);
            neighboursB = Copy(neighboursB);


            for (int i = 0; i < neighboursA.Count; ++i)
            {
                int id = neighboursA[i];
                List<int> list = connectedVertices[id];
                if (!list.Remove(keepOrMove))
                {
                    throw new Exception();
                }
            }
            for (int i = 0; i < neighboursB.Count; ++i)
            {
                int id = neighboursB[i];
                List<int> list = connectedVertices[id];
                if (!list.Remove(remove))
                {
                    throw new Exception();
                }
            }
            connectedVertices[keepOrMove].Clear();
            connectedVertices[remove] = null;

            for (int i = 0; i < triUnionCount; ++i)
            {
                int id = scratch[i];
                TriWithGroupId tri = triangles[id];
                if (!connectedTriangles[tri.A].Remove(id))
                    throw new Exception();
                if (!connectedTriangles[tri.B].Remove(id))
                    throw new Exception();
                if (!connectedTriangles[tri.C].Remove(id))
                    throw new Exception();
            }
            connectedTriangles[keepOrMove].Clear();
            connectedTriangles[remove] = null;

            for (int i = 0; i < triUnionCount; ++i)
            {
                int id = scratch[i];
                TriWithGroupId tri = triangles[id];
                bool containsA = tri.Contains(keepOrMove);
                bool containsB = tri.Contains(remove);
                if (containsA && containsB)
                {
                    tri.A = -1;
                    tri.B = -1;
                    tri.C = -1;
                }
                else if (containsB)
                    tri.Replace(remove, keepOrMove);
                triangles[id] = tri;
            }
            for (int i = 0; i < triUnionCount; ++i)
            {
                int id = scratch[i];
                TriWithGroupId tri = triangles[id];
                if (tri.A < 0)
                    continue;

                if (!connectedVertices[tri.A].Contains(tri.B))
                    connectedVertices[tri.A].Add(tri.B);
                if (!connectedVertices[tri.A].Contains(tri.C))
                    connectedVertices[tri.A].Add(tri.C);

                if (!connectedVertices[tri.B].Contains(tri.A))
                    connectedVertices[tri.B].Add(tri.A);
                if (!connectedVertices[tri.B].Contains(tri.C))
                    connectedVertices[tri.B].Add(tri.C);

                if (!connectedVertices[tri.C].Contains(tri.A))
                    connectedVertices[tri.C].Add(tri.A);
                if (!connectedVertices[tri.C].Contains(tri.B))
                    connectedVertices[tri.C].Add(tri.B);

                connectedTriangles[tri.A].Add(id);
                connectedTriangles[tri.B].Add(id);
                connectedTriangles[tri.C].Add(id);
            }


            //If a triangle with the same 3 vertices is connected twice to a, then remove both of them
            for (int i = 0; i < triUnionCount; ++i)
            {
                int ii = scratch[i];
                TriWithGroupId triI = triangles[ii]; if (triI.A < 0) continue;
                for (int j = i + 1; j < triUnionCount; ++j)
                {
                    int jj = scratch[j];
                    TriWithGroupId triJ = triangles[jj]; if (triJ.A < 0) continue;
                    if (triJ.ContainsAll(triI.A, triI.B, triI.C))
                    {
                        if (!connectedTriangles[triI.A].Remove(ii)) throw new Exception();
                        if (!connectedTriangles[triI.B].Remove(ii)) throw new Exception();
                        if (!connectedTriangles[triI.C].Remove(ii)) throw new Exception();

                        if (!connectedTriangles[triJ.A].Remove(jj)) throw new Exception();
                        if (!connectedTriangles[triJ.B].Remove(jj)) throw new Exception();
                        if (!connectedTriangles[triJ.C].Remove(jj)) throw new Exception();

                        var deletedTriI = triI;
                        deletedTriI.A = -1;
                        deletedTriI.B = -1;
                        deletedTriI.C = -1;
                        triangles[ii] = deletedTriI;
                        
                        var deletedTriJ = triJ;
                        deletedTriJ.A = -1;
                        deletedTriJ.B = -1;
                        deletedTriJ.C = -1;
                        triangles[jj] = deletedTriJ;
                        break;
                    }
                }
            }

            return true;
        }

        private static Rat3Hybrid Normal(IList<Rat3Hybrid> points, TriWithGroupId tri, int id, Rat3Hybrid p)
        {
            var a = points[tri.A];
            var b = points[tri.B];
            var c = points[tri.C];
            if (tri.A == id)
                a = p;
            if (tri.B == id)
                b = p;
            if (tri.C == id)
                c = p;

            return Rat3Hybrid.Cross(b - a, c - a);
        }
    }
}
