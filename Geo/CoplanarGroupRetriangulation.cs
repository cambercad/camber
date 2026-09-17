using GeoCore;

namespace Geo
{


    public static class CoplanarGroupRetriangulation
    {
        /// <summary>
        /// Retriangulates planar groups to optimize the mesh by removing unnecessary vertices.
        /// </summary>
        /// <remarks>
        /// This method removes two types of points from planar groups:
        /// 
        /// 1. INTERNAL POINTS: All internal points (points not on the boundary) within a coplanar 
        ///    surface are removed. The planar region is retriangulated using only its boundary vertices.
        /// 
        /// 2. BOUNDARY POINTS: Boundary points are removed only when ALL of the following conditions are met:
        ///    a) The point connects two perfectly collinear boundary segments (checked using exact arithmetic)
        ///    b) Both collinear boundary segments are connected to the same two groups (one being the 
        ///       active planar patch group, the other being the adjacent group)
        ///    c) The adjacent group is also marked as planar (present in idsOfPlanarGroups)
        /// 
        /// Points are removed by excluding them from the retriangulation input; the position list remains unchanged.
        /// </remarks>
        /// <typeparam name="T">The vertex data type</typeparam>
        /// <param name="positions">The list of vertex positions</param>
        /// <param name="triangles">The list of triangles (modified in-place)</param>
        /// <param name="trianglesEx">The list of extended triangle data (modified in-place)</param>
        /// <param name="idsOfPlanarGroups">Set of group IDs that are marked as planar</param>
        /// <param name="removeCollinearBoundaryPoints">If true, removes selected collinear boundary points. Default is false to preserve watertight neighbours.</param>
        public static HashSet<int> RetriangulateCoplanar<T>(
            List<Rat3Hybrid> positions,
            List<Tri> triangles,
            List<MeshTriangle<T>> trianglesEx,
            HashSet<int> idsOfPlanarGroups,
            bool removeCollinearBoundaryPoints = false) where T : ITriangleVertex<T>
        {
            var retriangulatedGroupIds = new HashSet<int>();
            if (idsOfPlanarGroups == null || idsOfPlanarGroups.Count == 0)
                return retriangulatedGroupIds;

            TriangleAdjacency[] globalAdjacency = AdjacencyEx.BuildAdjacencyInformation(positions, triangles);
            var edgeToTriangles = AdjacencyEx.BuildEdgeToTrianglesMap(triangles);
            var nonManifoldEdges = AdjacencyEx.CollectNonManifoldEdges(edgeToTriangles);

            // Extract triangle groups
            var groupIdPerTriangle = new List<int>(trianglesEx.Count);
            for (int i = 0; i < trianglesEx.Count; i++)
            {
                groupIdPerTriangle.Add(trianglesEx[i].GroupId);
            }

            // Group triangles by their group ID
            var triangleIdsPerGroup = CoplanarGroupFusion.ExtractTriangleIdsPerGroup(groupIdPerTriangle);

            // Store new triangles and trianglesEx
            List<Tri>[] newTrianglesArr = new List<Tri>[triangleIdsPerGroup.Count];
            List<MeshTriangle<T>>[] newTrianglesExArr = new List<MeshTriangle<T>>[triangleIdsPerGroup.Count];

            var pairs = triangleIdsPerGroup.ToArray();
            int groupCount = pairs.Length;

            const int MinTrisForParallelGroup = 64;
            var isHeavyGroup = new bool[groupCount];
            var wasRetriangulatedByGroup = new bool[groupCount];
            int heavyGroupCount = 0;
            var heavyGroupIndices = new List<int>();
            for (int i = 0; i < groupCount; i++)
            {
                var pair = pairs[i];
                if (idsOfPlanarGroups.Contains(pair.Key) && pair.Value.Count >= MinTrisForParallelGroup)
                {
                    isHeavyGroup[i] = true;
                    heavyGroupIndices.Add(i);
                    ++heavyGroupCount;
                }
            }

            void ProcessGroup(int i)
            {
                var (groupTriangles, groupTrianglesEx, wasRetriangulated) = RetriangulateGroup(
                    pairs[i],
                    positions,
                    triangles,
                    trianglesEx,
                    idsOfPlanarGroups,
                    removeCollinearBoundaryPoints,
                    globalAdjacency,
                    groupIdPerTriangle,
                    edgeToTriangles,
                    nonManifoldEdges);
                newTrianglesArr[i] = groupTriangles;
                newTrianglesExArr[i] = groupTrianglesEx;
                wasRetriangulatedByGroup[i] = wasRetriangulated;
            }

            for (int i = 0; i < groupCount; i++)
            {
                if (isHeavyGroup[i])
                    continue;
                ProcessGroup(i);
            }

            ParallelEx.ForGroups(0, heavyGroupCount, k => ProcessGroup(heavyGroupIndices[k]));

            // Workers own distinct array entries; collect shared set membership
            // only after all workers finish, in stable group order.
            for (int i = 0; i < groupCount; i++)
                if (wasRetriangulatedByGroup[i])
                    retriangulatedGroupIds.Add(pairs[i].Key);

            // Replace the original lists with retriangulated ones (fixed group order)
            triangles.Clear();
            for (int i = 0; i < groupCount; i++)
            {
                if (newTrianglesArr[i] != null)
                    triangles.AddRange(newTrianglesArr[i]);
            }
            trianglesEx.Clear();
            for (int i = 0; i < groupCount; i++)
            {
                if (newTrianglesExArr[i] != null)
                    trianglesEx.AddRange(newTrianglesExArr[i]);
            }

            return retriangulatedGroupIds;
        }

        private static (List<Tri> triangles, List<MeshTriangle<T>> trianglesEx, bool wasRetriangulated) RetriangulateGroup<T>(
            KeyValuePair<int, List<int>> groupEntry,
            List<Rat3Hybrid> positions,
            List<Tri> triangles,
            List<MeshTriangle<T>> trianglesEx,
            HashSet<int> idsOfPlanarGroups,
            bool removeCollinearBoundaryPoints,
            TriangleAdjacency[] globalAdjacency,
            List<int> groupIdPerTriangle,
            Dictionary<(int, int), List<int>> edgeToTriangles,
            HashSet<(int, int)> nonManifoldEdges) where T : ITriangleVertex<T>
        {
            var newTriangles = new List<Tri>();
            var newTrianglesEx = new List<MeshTriangle<T>>();

            int groupId = groupEntry.Key;
            List<int> triIndices = groupEntry.Value;

            if (!idsOfPlanarGroups.Contains(groupId))
            {
                foreach (int triIndex in triIndices)
                {
                    newTriangles.Add(triangles[triIndex]);
                    newTrianglesEx.Add(trianglesEx[triIndex]);
                }
                return (newTriangles, newTrianglesEx, wasRetriangulated: false);
            }

            bool wasRetriangulated = false;
            var components = DisconnectedGroupSplit.SplitGroupIntoComponents(
                triangles, triIndices, groupId, globalAdjacency, groupIdPerTriangle, nonManifoldEdges);

            foreach (var component in components)
            {
                var retriangulated = RetriangulatePlanarGroup(
                    positions, triangles, trianglesEx, component, groupId, globalAdjacency, groupIdPerTriangle,
                    idsOfPlanarGroups, removeCollinearBoundaryPoints, edgeToTriangles, nonManifoldEdges);

#if DEBUG
                ValidateBoundaryPreserved(triangles, component, retriangulated.triangles, groupId);
#endif

                if (retriangulated.wasRetriangulated)
                    wasRetriangulated = true;

                newTriangles.AddRange(retriangulated.triangles);
                newTrianglesEx.AddRange(retriangulated.trianglesEx);
            }

            return (newTriangles, newTrianglesEx, wasRetriangulated);
        }

        private static (List<Tri> triangles, List<MeshTriangle<T>> trianglesEx, bool wasRetriangulated) RetriangulatePlanarGroup<T>(
            List<Rat3Hybrid> positions,
            List<Tri> triangles,
            List<MeshTriangle<T>> trianglesEx,
            List<int> triIndices,
            int groupId,
            TriangleAdjacency[] globalAdjacency,
            List<int> groupIdPerTriangle,
            HashSet<int> idsOfPlanarGroups,
            bool removeCollinearBoundaryPoints,
            Dictionary<(int, int), List<int>> edgeToTriangles,
            HashSet<(int, int)> nonManifoldEdges) where T : ITriangleVertex<T>
        {
            // Step 1: Extract boundary edges of the group with adjacency info
            var boundaryEdges = ExtractBoundaryEdges(triangles, triIndices);

            if (boundaryEdges.Count == 0)
                return CopyOriginalTriangles(triangles, trianglesEx, triIndices);

            // Step 1b: Determine which group each boundary edge is adjacent to
            var edgeAdjacentGroups = new Dictionary<(int, int), int>();
            foreach (var edge in boundaryEdges)
            {
                int adjacentGroup = FindAdjacentGroupForEdge(
                    edge, triIndices, triangles, globalAdjacency, groupIdPerTriangle, groupId, edgeToTriangles, nonManifoldEdges);
                var normalizedEdge = NormalizeEdge(edge.X, edge.Y);
                edgeAdjacentGroups[normalizedEdge] = adjacentGroup;
            }

            // Step 2: Project boundary vertices to a local 2D polygon domain.
            BuildProjectedBoundaryVertices(
                positions,
                boundaryEdges,
                out var projected2D,
                out var vertexIndexMap,
                out var reverseMap);

            // Step 5: Find the boundary polygon (assuming single closed loop for now)
            List<List<int>> borderAndHolePolygons = TraceBoundaryPolygon(
                boundaryEdges, vertexIndexMap, positions, projected2D, reverseMap, edgeAdjacentGroups,
                idsOfPlanarGroups, groupId, removeCollinearBoundaryPoints, nonManifoldEdges);

            if (borderAndHolePolygons == null || borderAndHolePolygons.Count == 0)
                return CopyOriginalTriangles(triangles, trianglesEx, triIndices);

#if DEBUG
            ValidateBoundaryPolygonsForEarClipping(projected2D, borderAndHolePolygons, groupId);
#endif

            List<Tri> triangulated2D;
            try
            {
                triangulated2D = Triangulator.TriangulatePolygon(projected2D, borderAndHolePolygons, delaunayPostProcess: true);
            }
            catch (InvalidOperationException)
            {
                return CopyOriginalTriangles(triangles, trianglesEx, triIndices);
            }

            // Step 7: Determine correct winding by comparing with original triangles
            bool shouldFlipWinding = ShouldFlipTriangleWinding(positions, triangles, triIndices, triangulated2D, reverseMap);

            // Step 8: Build a map from vertex index to vertex data
            // We need to collect all the vertex data from the original triangles
            var vertexDataMap = new Dictionary<int, T>();
            
            foreach (int triIndex in triIndices)
            {
                var tri = triangles[triIndex];
                var triEx = trianglesEx[triIndex];
                
                // Store vertex data for each vertex if not already stored
                if (!vertexDataMap.ContainsKey(tri.A))
                    vertexDataMap[tri.A] = triEx.V0;
                if (!vertexDataMap.ContainsKey(tri.B))
                    vertexDataMap[tri.B] = triEx.V1;
                if (!vertexDataMap.ContainsKey(tri.C))
                    vertexDataMap[tri.C] = triEx.V2;
            }

            // Step 9: Map back to 3D vertex indices
            var resultTriangles = new List<Tri>();
            var resultTrianglesEx = new List<MeshTriangle<T>>();

            foreach (var tri2D in triangulated2D)
            {
                // Map 2D indices back to 3D
                int v0 = reverseMap[tri2D.A];
                int v1 = reverseMap[tri2D.B];
                int v2 = reverseMap[tri2D.C];

                // Apply winding correction if needed
                var tri3D = shouldFlipWinding ? new Tri(v0, v2, v1) : new Tri(v0, v1, v2);
                resultTriangles.Add(tri3D);

                // Create triangle extended data using the mapped vertex data
                var triEx = new MeshTriangle<T>();
                triEx.GroupId = groupId;

                // Look up the correct vertex data for each vertex
                if (shouldFlipWinding)
                {
                    triEx.V0 = vertexDataMap[v0];
                    triEx.V1 = vertexDataMap[v2];
                    triEx.V2 = vertexDataMap[v1];
                }
                else
                {
                    triEx.V0 = vertexDataMap[v0];
                    triEx.V1 = vertexDataMap[v1];
                    triEx.V2 = vertexDataMap[v2];
                }

                resultTrianglesEx.Add(triEx);
            }

            return (resultTriangles, resultTrianglesEx, wasRetriangulated: true);
        }

        private static BigRationalHybrid SignedAreaTimesTwo(List<Rat2Hybrid> points, List<int> polygon)
        {
            BigRationalHybrid result = BigRationalHybrid.Zero;
            for (int i = 0; i < polygon.Count; i++)
            {
                var a = points[polygon[i]];
                var b = points[polygon[(i + 1) % polygon.Count]];
                result += a.X * b.Y - a.Y * b.X;
            }
            return result;
        }

        private static bool SegmentsIntersectOrTouch(List<Rat2Hybrid> points, int a0, int a1, int b0, int b1)
        {
            var a = points[a0];
            var b = points[a1];
            var c = points[b0];
            var d = points[b1];

            int o1 = Rat2Hybrid.Orient2DSign(a, b, c);
            int o2 = Rat2Hybrid.Orient2DSign(a, b, d);
            int o3 = Rat2Hybrid.Orient2DSign(c, d, a);
            int o4 = Rat2Hybrid.Orient2DSign(c, d, b);

            if (o1 == 0 && PointOnSegment(c, a, b))
                return true;
            if (o2 == 0 && PointOnSegment(d, a, b))
                return true;
            if (o3 == 0 && PointOnSegment(a, c, d))
                return true;
            if (o4 == 0 && PointOnSegment(b, c, d))
                return true;

            return o1 != o2 && o3 != o4;
        }

        private static bool PointOnSegment(Rat2Hybrid p, Rat2Hybrid a, Rat2Hybrid b)
        {
            return p.X.CompareTo(BigRationalHybrid.Min(a.X, b.X)) >= 0
                && p.X.CompareTo(BigRationalHybrid.Max(a.X, b.X)) <= 0
                && p.Y.CompareTo(BigRationalHybrid.Min(a.Y, b.Y)) >= 0
                && p.Y.CompareTo(BigRationalHybrid.Max(a.Y, b.Y)) <= 0;
        }

#if DEBUG
        private static void ValidateBoundaryPreserved(List<Tri> originalTriangles, List<int> originalTriIndices, List<Tri> newTriangles, int groupId)
        {
            var originalBoundary = ExtractBoundaryEdgeSet(originalTriangles, originalTriIndices);
            var newBoundary = ExtractBoundaryEdgeSet(newTriangles);

            if (!originalBoundary.SetEquals(newBoundary))
                throw new Exception($"Coplanar retriangulation changed the patch boundary. group={groupId}, originalEdges={originalBoundary.Count}, newEdges={newBoundary.Count}");
        }

        private static HashSet<(int, int)> ExtractBoundaryEdgeSet(List<Tri> triangles, List<int> triIndices)
        {
            var counts = new Dictionary<(int, int), int>();
            foreach (int triIndex in triIndices)
                AddTriangleEdges(counts, triangles[triIndex]);

            return counts.Where(kv => kv.Value == 1).Select(kv => kv.Key).ToHashSet();
        }

        private static HashSet<(int, int)> ExtractBoundaryEdgeSet(List<Tri> triangles)
        {
            var counts = new Dictionary<(int, int), int>();
            foreach (var tri in triangles)
                AddTriangleEdges(counts, tri);

            return counts.Where(kv => kv.Value == 1).Select(kv => kv.Key).ToHashSet();
        }

        private static void AddTriangleEdges(Dictionary<(int, int), int> counts, Tri tri)
        {
            AddBoundaryEdgeCount(counts, tri.A, tri.B);
            AddBoundaryEdgeCount(counts, tri.B, tri.C);
            AddBoundaryEdgeCount(counts, tri.C, tri.A);
        }

        private static void AddBoundaryEdgeCount(Dictionary<(int, int), int> counts, int a, int b)
        {
            var edge = NormalizeEdge(a, b);
            counts.TryGetValue(edge, out int count);
            counts[edge] = count + 1;
        }

        private static void ValidateBoundaryPolygonsForEarClipping(List<Rat2Hybrid> points, List<List<int>> polygons, int groupId)
        {
            for (int p = 0; p < polygons.Count; p++)
            {
                var polygon = polygons[p];
                if (polygon.Count < 3)
                    throw new Exception($"Coplanar retriangulation produced a polygon with fewer than 3 vertices. group={groupId}, polygon={p}");

                var used = new HashSet<int>();
                for (int i = 0; i < polygon.Count; i++)
                {
                    int index = polygon[i];
                    if (index < 0 || index >= points.Count)
                        throw new Exception($"Coplanar retriangulation produced an out-of-range polygon vertex. group={groupId}, polygon={p}, index={index}");
                    if (!used.Add(index))
                        throw new Exception($"Coplanar retriangulation produced a polygon with a repeated vertex. group={groupId}, polygon={p}, vertex={index}");
                }

                if (SignedAreaTimesTwo(points, polygon) == BigRationalHybrid.Zero)
                    throw new Exception($"Coplanar retriangulation produced a zero-area polygon. group={groupId}, polygon={p}");

                for (int i = 0; i < polygon.Count; i++)
                {
                    int iNext = (i + 1) % polygon.Count;
                    for (int j = i + 1; j < polygon.Count; j++)
                    {
                        int jNext = (j + 1) % polygon.Count;
                        if (iNext == j || jNext == i)
                            continue;

                        if (SegmentsIntersectOrTouch(points, polygon[i], polygon[iNext], polygon[j], polygon[jNext]))
                            throw new Exception($"Coplanar retriangulation produced a self-intersecting polygon. group={groupId}, polygon={p}, edges=({i},{iNext})/({j},{jNext})");
                    }
                }
            }
        }
#endif

        private static (List<Tri> triangles, List<MeshTriangle<T>> trianglesEx, bool wasRetriangulated) CopyOriginalTriangles<T>(
            List<Tri> triangles,
            List<MeshTriangle<T>> trianglesEx,
            List<int> triIndices) where T : ITriangleVertex<T>
        {
            var resultTriangles = new List<Tri>(triIndices.Count);
            var resultTrianglesEx = new List<MeshTriangle<T>>(triIndices.Count);

            foreach (int triIndex in triIndices)
            {
                resultTriangles.Add(triangles[triIndex]);
                resultTrianglesEx.Add(trianglesEx[triIndex]);
            }

            return (resultTriangles, resultTrianglesEx, wasRetriangulated: false);
        }

        /// <summary>
        /// Determines if the retriangulated triangles need their winding flipped
        /// by comparing normals via dot product
        /// </summary>
        private static bool ShouldFlipTriangleWinding(
            List<Rat3Hybrid> positions,
            List<Tri> originalTriangles,
            List<int> originalTriIndices,
            List<Tri> retriangulatedTriangles,
            List<int> reverseMap)
        {
            if (retriangulatedTriangles.Count == 0)
                return false;

            // Get a reference triangle from the original group
            var origTri = originalTriangles[originalTriIndices[0]];
            Rat3Hybrid p0_orig = positions[origTri.A];
            Rat3Hybrid p1_orig = positions[origTri.B];
            Rat3Hybrid p2_orig = positions[origTri.C];

            // Compute the normal of the original triangle
            var edge1_orig = p1_orig - p0_orig;
            var edge2_orig = p2_orig - p0_orig;
            var normalOrig = Rat3Hybrid.Cross(edge1_orig, edge2_orig);

            // Get a retriangulated triangle (map back to 3D indices)
            var retriTri = retriangulatedTriangles[0];
            Rat3Hybrid p0_retri = positions[reverseMap[retriTri.A]];
            Rat3Hybrid p1_retri = positions[reverseMap[retriTri.B]];
            Rat3Hybrid p2_retri = positions[reverseMap[retriTri.C]];

            // Compute the normal of the retriangulated triangle
            var edge1_retri = p1_retri - p0_retri;
            var edge2_retri = p2_retri - p0_retri;
            var normalRetri = Rat3Hybrid.Cross(edge1_retri, edge2_retri);

            // Compute dot product
            var dotProduct = Rat3Hybrid.Dot(normalOrig, normalRetri);

            // If dot product is negative, normals point in opposite directions
            // so we need to flip the winding
            return dotProduct < BigRationalHybrid.Zero;
        }

        /// <summary>
        /// Extracts directed boundary half-edges of a triangle patch.
        /// </summary>
        private static List<Int2> ExtractBoundaryEdges(List<Tri> triangles, List<int> triIndices)
        {
            var edgeOccurrences = new Dictionary<(int, int), List<Int2>>();

            foreach (int triIndex in triIndices)
            {
                var tri = triangles[triIndex];

                AddDirectedEdge(edgeOccurrences, tri.A, tri.B);
                AddDirectedEdge(edgeOccurrences, tri.B, tri.C);
                AddDirectedEdge(edgeOccurrences, tri.C, tri.A);
            }

            var boundaryEdges = new List<Int2>();
            foreach (var kvp in edgeOccurrences)
            {
                if (kvp.Value.Count == 1)
                    boundaryEdges.Add(kvp.Value[0]);
            }

            return boundaryEdges;
        }

        private static void AddDirectedEdge(Dictionary<(int, int), List<Int2>> edgeOccurrences, int start, int end)
        {
            var edge = NormalizeEdge(start, end);

            if (!edgeOccurrences.TryGetValue(edge, out var occurrences))
                edgeOccurrences[edge] = occurrences = new List<Int2>();

            occurrences.Add(new Int2(start, end));
        }

        /// <summary>
        /// Computes projection axes using the GetProjected method from Resolver
        /// Returns the two axes to use for 2D projection (matching Resolver.cs approach)
        /// </summary>
        internal static Int2 ComputeProjectionAxes(List<Rat3Hybrid> positions, List<int> vertexIndices)
        {
            if (vertexIndices.Count < 3)
                throw new Exception();

            bool success = false;
            // Use first three vertices to determine projection plane
            for(int i=0;i<vertexIndices.Count;++i)
            {
                Rat3Hybrid a = positions[vertexIndices[(i + 0) % vertexIndices.Count]];
                Rat3Hybrid b = positions[vertexIndices[(i + 1)% vertexIndices.Count]];
                Rat3Hybrid c = positions[vertexIndices[(i + 2)% vertexIndices.Count]];

                // Compute normal using cross product
                var ab = b - a;
                var ac = c - a;
                var n = Rat3Hybrid.Cross(ab, ac);

                if(n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0)
                {
                    continue;
                }
                success = true;

                // Find principal component (same logic as Resolver.GetProjected)
                bool xNeg = n.X < BigRationalHybrid.Zero;
                if (xNeg) n.X = -n.X;
                bool yNeg = n.Y < BigRationalHybrid.Zero;
                if (yNeg) n.Y = -n.Y;
                bool zNeg = n.Z < BigRationalHybrid.Zero;
                if (zNeg) n.Z = -n.Z;

                int x = -1;
                int y = -1;
                if (n.X >= n.Y && n.X >= n.Z)
                {
                    // Principal component is X, project onto YZ plane
                    x = 1;
                    y = 2;
                    if (xNeg)
                    {
                        int tmp = x;
                        x = y;
                        y = tmp;
                    }
                }
                else if (n.Y >= n.X && n.Y >= n.Z)
                {
                    // Principal component is Y, project onto XZ plane
                    x = 2;
                    y = 0;
                    if (yNeg)
                    {
                        int tmp = x;
                        x = y;
                        y = tmp;
                    }
                }
                else
                {
                    // Principal component is Z, project onto XY plane
                    x = 0;
                    y = 1;
                    if (zNeg)
                    {
                        int tmp = x;
                        x = y;
                        y = tmp;
                    }
                }
                return new Int2(x, y);
            }

            if (!success)
                throw new Exception();

            // Fallback to XY projection
            return new Int2(0, 1);
        }

        /// <summary>
        /// Projects a 3D point to 2D using the specified axes
        /// </summary>
        internal static Rat2Hybrid Project3DTo2D(Rat3Hybrid point, Int2 axes)
        {
            // Project by selecting the appropriate components
            return new Rat2Hybrid(point[axes.X], point[axes.Y]);
        }

        private static void BuildProjectedBoundaryVertices(
            List<Rat3Hybrid> positions,
            List<Int2> boundaryEdges,
            out List<Rat2Hybrid> projected2D,
            out Dictionary<int, int> vertexIndexMap,
            out List<int> reverseMap)
        {
            var boundaryVertices = new HashSet<int>();
            foreach (var edge in boundaryEdges)
            {
                boundaryVertices.Add(edge.X);
                boundaryVertices.Add(edge.Y);
            }

            var boundaryVertexList = boundaryVertices.ToList();
            var projectionAxes = ComputeProjectionAxes(positions, boundaryVertexList);

            projected2D = new List<Rat2Hybrid>();
            vertexIndexMap = new Dictionary<int, int>();
            reverseMap = new List<int>();
            var projectedPointToIndex = new Dictionary<Rat2Hybrid, int>();

            foreach (int vertexIndex in boundaryVertexList)
            {
                Rat2Hybrid projected = Project3DTo2D(positions[vertexIndex], projectionAxes);

                if (!projectedPointToIndex.TryGetValue(projected, out int projectedIndex))
                {
                    projectedIndex = projected2D.Count;
                    projectedPointToIndex.Add(projected, projectedIndex);
                    projected2D.Add(projected);
                    reverseMap.Add(vertexIndex);
                }

                vertexIndexMap[vertexIndex] = projectedIndex;
            }
        }

        private static List<List<int>> TraceBoundaryPolygon(
            List<Int2> boundaryEdges,
            Dictionary<int, int> vertexIndexMap,
            List<Rat3Hybrid> positions,
            List<Rat2Hybrid> projected2D,
            List<int> reverseMap,
            Dictionary<(int, int), int> edgeAdjacentGroups,
            HashSet<int> idsOfPlanarGroups,
            int groupId,
            bool removeCollinearBoundaryPoints,
            HashSet<(int, int)> nonManifoldEdges)
        {
            if (boundaryEdges.Count == 0)
                return new List<List<int>>();

            var edges2D = new List<Int2>();
            foreach (var edge in boundaryEdges)
            {
                int v0 = vertexIndexMap[edge.X];
                int v1 = vertexIndexMap[edge.Y];
                edges2D.Add(new Int2(v0, v1));
            }

            var loops = TraceDirectedBoundaryLoops(edges2D);
            if (loops == null)
                return null;

            // Conditionally filter out collinear boundary points that meet the conditions
            var filteredLoops = new List<List<int>>();
            foreach (var loop in loops)
            {
                var filteredLoop = removeCollinearBoundaryPoints
                    ? FilterCollinearBoundaryPoints(loop, positions, projected2D, reverseMap, edgeAdjacentGroups, idsOfPlanarGroups, groupId, nonManifoldEdges)
                    : loop;
                if (filteredLoop.Count < 3)
                    return null;
                filteredLoops.Add(filteredLoop);
            }

#if DEBUG
            if (!BoundaryPolygonsAreValid(projected2D, filteredLoops))
                return null;
#endif

            return filteredLoops;
        }

        private static bool BoundaryPolygonsAreValid(List<Rat2Hybrid> points, List<List<int>> loops)
        {
            for (int i = 0; i < loops.Count; i++)
            {
                if (!BoundaryLoopIsValid(points, loops[i]))
                    return false;

                for (int j = i + 1; j < loops.Count; j++)
                {
                    if (BoundaryLoopsTouchOrIntersect(points, loops[i], loops[j]))
                        return false;
                }
            }

            return true;
        }

        private static bool BoundaryLoopIsValid(List<Rat2Hybrid> points, List<int> loop)
        {
            if (loop.Count < 3)
                return false;
            if (SignedAreaTimesTwo(points, loop) == BigRationalHybrid.Zero)
                return false;

            var used = new HashSet<int>();
            foreach (int index in loop)
            {
                if (index < 0 || index >= points.Count || !used.Add(index))
                    return false;
            }

            for (int i = 0; i < loop.Count; i++)
            {
                int nextI = (i + 1) % loop.Count;
                for (int j = i + 1; j < loop.Count; j++)
                {
                    int nextJ = (j + 1) % loop.Count;
                    if (nextI == j || nextJ == i)
                        continue;

                    if (SegmentsIntersectOrTouch(points, loop[i], loop[nextI], loop[j], loop[nextJ]))
                        return false;
                }
            }

            return true;
        }

        private static bool BoundaryLoopsTouchOrIntersect(List<Rat2Hybrid> points, List<int> a, List<int> b)
        {
            var usedA = new HashSet<int>(a);
            foreach (int index in b)
            {
                if (usedA.Contains(index))
                    return true;
            }

            for (int i = 0; i < a.Count; i++)
            {
                int nextI = (i + 1) % a.Count;
                for (int j = 0; j < b.Count; j++)
                {
                    int nextJ = (j + 1) % b.Count;
                    if (SegmentsIntersectOrTouch(points, a[i], a[nextI], b[j], b[nextJ]))
                        return true;
                }
            }

            return false;
        }

        private static List<List<int>> TraceDirectedBoundaryLoops(List<Int2> edges)
        {
            var nextByStart = new Dictionary<int, int>();
            var incomingCount = new Dictionary<int, int>();

            foreach (var edge in edges)
            {
                if (edge.X == edge.Y)
                    return null;
                if (!nextByStart.TryAdd(edge.X, edge.Y))
                    return null;

                incomingCount.TryGetValue(edge.Y, out int count);
                incomingCount[edge.Y] = count + 1;
                if (incomingCount[edge.Y] > 1)
                    return null;
            }

            foreach (int start in nextByStart.Keys)
            {
                if (!incomingCount.ContainsKey(start))
                    return null;
            }

            var loops = new List<List<int>>();
            var visited = new HashSet<int>();

            foreach (int start in nextByStart.Keys)
            {
                if (visited.Contains(start))
                    continue;

                var loop = new List<int>();
                int current = start;

                while (true)
                {
                    if (visited.Contains(current))
                    {
                        if (current != start || loop.Count < 3)
                            return null;
                        loops.Add(loop);
                        break;
                    }

                    visited.Add(current);
                    loop.Add(current);

                    if (!nextByStart.TryGetValue(current, out int next))
                        return null;

                    current = next;
                }
            }

            return loops;
        }

        /// <summary>
        /// Filters out collinear boundary points that meet the removal conditions
        /// </summary>
        private static List<int> FilterCollinearBoundaryPoints(
            List<int> loop,
            List<Rat3Hybrid> positions,
            List<Rat2Hybrid> projected2D,
            List<int> reverseMap,
            Dictionary<(int, int), int> edgeAdjacentGroups,
            HashSet<int> idsOfPlanarGroups,
            int groupId,
            HashSet<(int, int)> nonManifoldEdges)
        {
            if (loop.Count < 3)
                return loop;

            var filteredLoop = new List<int>();

            for (int i = 0; i < loop.Count; i++)
            {
                int prevIdx = (i - 1 + loop.Count) % loop.Count;
                int currIdx = i;
                int nextIdx = (i + 1) % loop.Count;

                int prev2D = loop[prevIdx];
                int curr2D = loop[currIdx];
                int next2D = loop[nextIdx];

                // Map to 3D vertex indices
                int prev3D = reverseMap[prev2D];
                int curr3D = reverseMap[curr2D];
                int next3D = reverseMap[next2D];

                // Check if the three points are collinear in 2D (using projected coordinates)
                bool areCollinear = ArePointsCollinear(projected2D[prev2D], projected2D[curr2D], projected2D[next2D]);

                if (!areCollinear)
                {
                    // Not collinear, keep the point
                    filteredLoop.Add(curr2D);
                    continue;
                }

                // Condition 1: The point connects two collinear boundary segments (already checked)
                
                // Condition 2: Both edges are connected to the same two groups
                var edge1 = NormalizeEdge(prev3D, curr3D);
                var edge2 = NormalizeEdge(curr3D, next3D);

                if (nonManifoldEdges.Contains(edge1) || nonManifoldEdges.Contains(edge2))
                {
                    filteredLoop.Add(curr2D);
                    continue;
                }

                if (!edgeAdjacentGroups.TryGetValue(edge1, out int adjacentGroup1) ||
                    !edgeAdjacentGroups.TryGetValue(edge2, out int adjacentGroup2))
                {
                    // Can't determine adjacent groups, keep the point
                    filteredLoop.Add(curr2D);
                    continue;
                }

                if (adjacentGroup1 != adjacentGroup2)
                {
                    // Edges are connected to different groups, keep the point
                    filteredLoop.Add(curr2D);
                    continue;
                }

                // Condition 3: The other group is also marked as planar
                if (adjacentGroup1 == -1 || !idsOfPlanarGroups.Contains(adjacentGroup1))
                {
                    // Not a planar group (or no adjacent group), keep the point
                    filteredLoop.Add(curr2D);
                    continue;
                }

                // All conditions met, skip this point (don't add to filteredLoop)
            }

            return filteredLoop;
        }

        /// <summary>
        /// Checks if three points are collinear in 2D using cross product
        /// </summary>
        private static bool ArePointsCollinear(Rat2Hybrid p1, Rat2Hybrid p2, Rat2Hybrid p3)
        {
            // Calculate cross product of vectors (p2-p1) and (p3-p1)
            var v1 = p2 - p1;
            var v2 = p3 - p1;
            
            // Cross product in 2D: v1.x * v2.y - v1.y * v2.x
            var crossProduct = v1.X * v2.Y - v1.Y * v2.X;
            
            return crossProduct == BigRationalHybrid.Zero;
        }

        /// <summary>
        /// Finds the adjacent group for a boundary edge
        /// </summary>
        private static int FindAdjacentGroupForEdge(
            Int2 edge,
            List<int> triIndices,
            List<Tri> triangles,
            TriangleAdjacency[] globalAdjacency,
            List<int> groupIdPerTriangle,
            int currentGroupId,
            Dictionary<(int, int), List<int>> edgeToTriangles,
            HashSet<(int, int)> nonManifoldEdges)
        {
            var normalizedEdge = NormalizeEdge(edge.X, edge.Y);
            if (nonManifoldEdges.Contains(normalizedEdge))
                return -1;

            if (!edgeToTriangles.TryGetValue(normalizedEdge, out var edgeTriangles) || edgeTriangles.Count != 2)
                return -1;

            foreach (int triIndex in triIndices)
            {
                var tri = triangles[triIndex];
                if (!TriangleContainsEdge(tri, edge.X, edge.Y))
                    continue;

                int adjacentTriIndex = GetNeighbourAcrossEdge(tri, globalAdjacency[triIndex], edge.X, edge.Y);
                if (adjacentTriIndex >= 0 && groupIdPerTriangle[adjacentTriIndex] != currentGroupId)
                    return groupIdPerTriangle[adjacentTriIndex];

                int otherTriIndex = edgeTriangles[0] == triIndex ? edgeTriangles[1] : edgeTriangles[0];
                return groupIdPerTriangle[otherTriIndex] == currentGroupId ? -1 : groupIdPerTriangle[otherTriIndex];
            }
            
            return -1;
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

        /// <summary>
        /// Checks if a triangle contains an edge
        /// </summary>
        private static bool TriangleContainsEdge(Tri tri, int v1, int v2)
        {
            return EdgeMatches(tri.A, tri.B, v1, v2) ||
                   EdgeMatches(tri.B, tri.C, v1, v2) ||
                   EdgeMatches(tri.C, tri.A, v1, v2);
        }

        /// <summary>
        /// Checks if an edge matches (in either direction)
        /// </summary>
        private static bool EdgeMatches(int e1, int e2, int v1, int v2)
        {
            return (e1 == v1 && e2 == v2) || (e1 == v2 && e2 == v1);
        }

        /// <summary>
        /// Normalizes an edge to have the smaller vertex index first
        /// </summary>
        private static (int, int) NormalizeEdge(int v1, int v2)
        {
            return v1 < v2 ? (v1, v2) : (v2, v1);
        }
    }
}
