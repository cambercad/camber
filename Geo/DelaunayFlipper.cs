using GeoCore;

namespace Geo
{
    public static class DelaunayFlipper
    {

        /// <summary>
        /// Makes a triangulation Delaunay for coplanar 3D triangles by projecting to 2D
        /// Uses principal component projection
        /// Only processes groups specified in idsOfPlanarGroups
        /// </summary>
        public static void MakeDelaunay<T>(
            List<Rat3Hybrid> positions,
            List<Tri> triangles,
            List<MeshTriangle<T>> trianglesEx,
            HashSet<int> idsOfPlanarGroups,
            HashSet<int> skipGroupIds = null) where T : struct, ITriangleVertex<T>
        {
            if (triangles.Count == 0)
                return;

            // Step 1: Extract triangle groups
            var groupIdPerTriangle = new List<int>(trianglesEx.Count);
            for (int i = 0; i < trianglesEx.Count; i++)
            {
                groupIdPerTriangle.Add(trianglesEx[i].GroupId);
            }

            // Step 2: Group triangles by their group ID
            var triangleIdsPerGroup = CoplanarGroupFusion.ExtractTriangleIdsPerGroup(groupIdPerTriangle);

            // Step 3: Store new triangles and trianglesEx
            List<Tri>[] newTrianglesArr = new List<Tri>[triangleIdsPerGroup.Count];
            List<MeshTriangle<T>>[] newTrianglesExArr = new List<MeshTriangle<T>>[triangleIdsPerGroup.Count];

            var pairs = triangleIdsPerGroup.ToArray();

            // Step 4: Process each group (coarse-grained; avoid thread-pool overhead on few groups).
            ParallelEx.ForGroups(0, pairs.Length, i =>
            {
                var groupEntry = pairs[i];

                List<Tri> newTriangles = new List<Tri>();
                List<MeshTriangle<T>> newTrianglesEx = new List<MeshTriangle<T>>();

                int groupId = groupEntry.Key;
                List<int> triIndices = groupEntry.Value;

                // Only apply Delaunay to specified planar groups that were not already optimized during retriangulation.
                if (!idsOfPlanarGroups.Contains(groupId) || (skipGroupIds != null && skipGroupIds.Contains(groupId)))
                {
                    // Keep non-planar groups as-is
                    foreach (int triIndex in triIndices)
                    {
                        newTriangles.Add(triangles[triIndex]);
                        newTrianglesEx.Add(trianglesEx[triIndex]);
                    }
                    newTrianglesArr[i] = newTriangles;
                    newTrianglesExArr[i] = newTrianglesEx;
                    return;
                }

                // Apply Delaunay optimization to this planar group
                var optimized = MakeDelaunayForGroup(
                    positions, triangles, trianglesEx, triIndices, groupId);

                newTrianglesArr[i] = optimized.triangles;
                newTrianglesExArr[i] = optimized.trianglesEx;
            });

            // Step 5: Replace the original lists with optimized ones
            triangles.Clear();
            foreach (var v in newTrianglesArr)
                if (v != null)
                    triangles.AddRange(v);
            trianglesEx.Clear();
            foreach (var v in newTrianglesExArr)
                if (v != null)
                    trianglesEx.AddRange(v);
        }

        private static (List<Tri> triangles, List<MeshTriangle<T>> trianglesEx) MakeDelaunayForGroup<T>(
            List<Rat3Hybrid> positions,
            List<Tri> triangles,
            List<MeshTriangle<T>> trianglesEx,
            List<int> triIndices,
            int groupId) where T : struct, ITriangleVertex<T>
        {
            // Step 1: Collect all unique vertex indices used in this group
            var vertexIndices = new HashSet<int>();
            foreach (int triIndex in triIndices)
            {
                var tri = triangles[triIndex];
                vertexIndices.Add(tri.A);
                vertexIndices.Add(tri.B);
                vertexIndices.Add(tri.C);
            }
            var vertexList = vertexIndices.ToList();

            // Step 2: Compute projection axes using the first triangle's plane
            var projectionAxes = CoplanarGroupRetriangulation.ComputeProjectionAxes(positions, vertexList);

            // Step 3: Project all vertices to 2D
            var positions2D = new List<Rat2Hybrid>();
            var vertexIndexMap = new Dictionary<int, int>(); // 3D vertex index -> 2D point index
            var reverseMap = new List<int>(); // 2D point index -> 3D vertex index

            for (int i = 0; i < vertexList.Count; i++)
            {
                int vertexIndex = vertexList[i];
                Rat3Hybrid pos3D = positions[vertexIndex];
                Rat2Hybrid pos2D = CoplanarGroupRetriangulation.Project3DTo2D(pos3D, projectionAxes);

                positions2D.Add(pos2D);
                vertexIndexMap[vertexIndex] = i;
                reverseMap.Add(vertexIndex);
            }

            // Step 4: Map triangles to 2D indices and extract trianglesEx for this group
            var triangles2D = new List<Tri>();
            var trianglesEx2D = new List<MeshTriangle<T>>();
            
            foreach (int triIndex in triIndices)
            {
                var tri = triangles[triIndex];
                int a2D = vertexIndexMap[tri.A];
                int b2D = vertexIndexMap[tri.B];
                int c2D = vertexIndexMap[tri.C];
                triangles2D.Add(new Tri(a2D, b2D, c2D));
                trianglesEx2D.Add(trianglesEx[triIndex]);
            }

            // Step 5: Call the 2D Delaunay algorithm
            MakeDelaunay(positions2D, triangles2D, trianglesEx2D);

            // Step 6: Map the results back to 3D indices
            var resultTriangles = new List<Tri>();
            var resultTrianglesEx = new List<MeshTriangle<T>>();
            
            for (int i = 0; i < triangles2D.Count; i++)
            {
                var tri2D = triangles2D[i];
                int a3D = reverseMap[tri2D.A];
                int b3D = reverseMap[tri2D.B];
                int c3D = reverseMap[tri2D.C];
                resultTriangles.Add(new Tri(a3D, b3D, c3D));
                resultTrianglesEx.Add(trianglesEx2D[i]);
            }

            return (resultTriangles, resultTrianglesEx);
        }

        public static void MakeDelaunay<T>(
            List<Rat2Hybrid> positions,
            List<Tri> triangles,
            List<MeshTriangle<T>> trianglesEx) where T : struct, ITriangleVertex<T>
        {
            if (triangles.Count == 0)
                return;

            // Step 1: Build a map from vertexIndex to vertex data
            // Store one representative vertex data per vertex (first occurrence)
            var vertexDataMap = new Dictionary<int, T>();

            for (int i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                var triEx = trianglesEx[i];

                // Store vertex data - use first occurrence
                if (!vertexDataMap.ContainsKey(tri.A))
                    vertexDataMap[tri.A] = triEx.V0;
                if (!vertexDataMap.ContainsKey(tri.B))
                    vertexDataMap[tri.B] = triEx.V1;
                if (!vertexDataMap.ContainsKey(tri.C))
                    vertexDataMap[tri.C] = triEx.V2;
            }

            // Step 2: Preserve group ID (all triangles in the same coplanar group have the same ID)
            int groupId = trianglesEx.Count > 0 ? trianglesEx[0].GroupId : 0;

            // Step 3: Perform Delaunay optimization
            DelaunayOptimizer<Rat2HybridCircleArithmetic, Rat2Hybrid>.Optimize(positions, triangles);

            // Step 4: Reconstruct trianglesEx based on the updated triangles
            // For each vertex in the new triangles, look up its vertex data
            trianglesEx.Clear();

            for (int i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                var triEx = new MeshTriangle<T>();

                // Look up vertex data for each vertex
                triEx.V0 = vertexDataMap.ContainsKey(tri.A) ? vertexDataMap[tri.A] : default(T);
                triEx.V1 = vertexDataMap.ContainsKey(tri.B) ? vertexDataMap[tri.B] : default(T);
                triEx.V2 = vertexDataMap.ContainsKey(tri.C) ? vertexDataMap[tri.C] : default(T);

                // All triangles in the coplanar group have the same group ID
                triEx.GroupId = groupId;

                trianglesEx.Add(triEx);
            }
        }
    }
}
