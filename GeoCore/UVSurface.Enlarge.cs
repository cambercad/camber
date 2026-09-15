namespace GeoCore
{
    public partial class UVSurface
    {
        /// <summary>
        /// Tangentially enlarges a mesh along its boundary loops.
        /// Optionally uses angle bisector weighting and adaptive scaling.
        /// Recomputes angle-weighted normals only for new vertices.
        /// If uvCoordinates is provided, generates UV coordinates for new vertices by extrapolating from boundary.
        /// </summary>
        private static void TangentiallyEnlargeSurface(
            List<Vec3D> vertices,
            List<Rat3Hybrid> verticesPrecise,
            List<Vec3D> normals,
            List<Tri> triangles,
            double enlargementDistance,
            List<Vec2D> uvCoordinates = null,
            bool useAngleBisector = false,
            bool useAdaptiveScaling = false)
        {
#if DEBUG
            if (!MeshAnalysis.AreTrianglesConsistentlyOriented(verticesPrecise, triangles))
                throw new Exception();
#endif

            int originalVertexCount = vertices.Count;
            int originalTriangleCount = triangles.Count;

            // Extract boundary loops
            List<bool> closed;
            var boundaryLoops = Adjacency.ExtractBoundaryLoops(triangles, out closed, out var edges);

            // Build a map from boundary edges to their triangle info for winding detection
            var edgeToTriangleMap = new Dictionary<(int, int), TriangleEdge>();
            foreach (var edge in edges)
            {
                if (edge.IsOnBorder)
                {
                    var key = GetEdgeKey(edge.Start, edge.End);
                    edgeToTriangleMap[key] = edge;
                }
            }

            // --- Step 1: Create new rim vertices and triangles ---
            var newVertexIndicesGlobal = new List<int>();

            foreach (var loop in boundaryLoops)
            {
                int loopCount = loop.Count;
                var newVertexIndices = new int[loopCount];

                // Compute UV centroid if UV coordinates are provided
                Vec2D uvCenter = new Vec2D(0, 0);
                if (uvCoordinates != null)
                {
                    for (int i = 0; i < loopCount; i++)
                    {
                        uvCenter = uvCenter + uvCoordinates[loop[i]];
                    }
                    uvCenter = uvCenter / loopCount;
                }

                // Determine offset direction and loop orientation for this entire loop (only check once)
                bool loopOrientedOutward = DetermineLoopOrientation(vertices, triangles, edgeToTriangleMap, 
                                                                     loop, normals);

                for (int i = 0; i < loopCount; i++)
                {
                    int prev = loop[(i - 1 + loopCount) % loopCount];
                    int curr = loop[i];
                    int next = loop[(i + 1) % loopCount];

                    Vec3D vPrev = vertices[prev];
                    Vec3D vCurr = vertices[curr];
                    Vec3D vNext = vertices[next];
                    Vec3D nCurr = normals[curr].Normalized();

                    // Edge tangents
                    Vec3D t1 = (vCurr - vPrev).Normalized();
                    Vec3D t2 = (vNext - vCurr).Normalized();

                    // --- Option 2: Use true angle bisector weighting ---
                    Vec3D avgTangent;
                    if (useAngleBisector)
                    {
                        Vec3D bisector = ((-t1).Normalized() + t2.Normalized());
                        if (bisector.Length() < 1e-6)
                            bisector = Vec3DOps.Cross(nCurr, t1); // fallback if straight
                        avgTangent = bisector.Normalized();
                    }
                    else
                    {
                        avgTangent = (t1 + t2).Normalized();
                    }

                    // Compute offset direction perpendicular to tangent and normal
                    // Apply sign correction if loop orientation is reversed
                    Vec3D offsetDir = Vec3DOps.Cross(avgTangent, nCurr).Normalized();
                    if (loopOrientedOutward)
                        offsetDir = -offsetDir;

                    // --- Option 3: Adaptive scaling ---
                    double localScale = 1.0;
                    if (useAdaptiveScaling)
                    {
                        double len1 = (vCurr - vPrev).Length();
                        double len2 = (vNext - vCurr).Length();
                        double avgLen = (len1 + len2) * 0.5;
                        localScale = Math.Clamp(avgLen / enlargementDistance, 0.5, 2.0);
                    }

                    Vec3D offset = offsetDir * (enlargementDistance * localScale);
                    Vec3D newVertex = vCurr + offset;

                    vertices.Add(newVertex);
                    normals.Add(new Vec3D(0, 0, 0)); // temporary placeholder
                    newVertexIndices[i] = vertices.Count - 1;
                    newVertexIndicesGlobal.Add(vertices.Count - 1);

                    // Generate UV coordinate for new vertex if UV coordinates are provided
                    if (uvCoordinates != null)
                    {
                        Vec2D currUV = uvCoordinates[curr];
                        // Extrapolate UV outward from centroid
                        Vec2D direction = currUV - uvCenter;
                        Vec2D newUV = currUV + direction * 1.0; // Extend by same distance as from center to boundary
                        uvCoordinates.Add(newUV);
                    }
                }

                // Connect old and new boundary with rim triangles
                // Use loop orientation to determine consistent winding for all rim triangles
                for (int i = 0; i < loopCount; i++)
                {
                    int i0 = loop[i];
                    int i1 = loop[(i + 1) % loopCount];
                    int j0 = newVertexIndices[i];
                    int j1 = newVertexIndices[(i + 1) % loopCount];

                    // Use the loop orientation to determine triangle winding
                    // This ensures all rim triangles have consistent winding with the original surface
                    if (loopOrientedOutward)
                    {
                        // Loop traverses boundary in correct outward direction
                        triangles.Add(new Tri(i0, i1, j1));
                        triangles.Add(new Tri(i0, j1, j0));
                    }
                    else
                    {
                        // Loop traverses boundary in reversed direction, flip winding
                        triangles.Add(new Tri(i1, i0, j0));
                        triangles.Add(new Tri(i1, j0, j1));
                    }
                }
            }

            // --- Step 2: Recompute normals ONLY for new vertices ---
            RecomputeAngleWeightedNormalsForNewVertices(vertices, normals, triangles, originalVertexCount);

#if DEBUG
            if (!MeshAnalysis.AreTrianglesConsistentlyOriented(verticesPrecise, triangles))
                throw new Exception();
#endif
        }

        /// <summary>
        /// Recomputes angle-weighted normals only for vertices added after originalVertexCount.
        /// </summary>
        private static void RecomputeAngleWeightedNormalsForNewVertices(
            List<Vec3D> vertices,
            List<Vec3D> normals,
            List<Tri> triangles,
            int originalVertexCount)
        {
            // Accumulate angle-weighted normals
            var accum = new Vec3D[vertices.Count];

            foreach (var tri in triangles)
            {
                Vec3D v0 = vertices[tri.A];
                Vec3D v1 = vertices[tri.B];
                Vec3D v2 = vertices[tri.C];

                Vec3D e0 = v1 - v0;
                Vec3D e1 = v2 - v1;
                Vec3D e2 = v0 - v2;

                Vec3D n = Vec3DOps.Cross(e0, v2 - v0).Normalized();

                double a0 = AngleBetween(-e2, e0);
                double a1 = AngleBetween(-e0, e1);
                double a2 = AngleBetween(-e1, e2);

                accum[tri.A] += n * a0;
                accum[tri.B] += n * a1;
                accum[tri.C] += n * a2;
            }

            // Normalize only for new vertices
            for (int i = originalVertexCount; i < vertices.Count; i++)
            {
                normals[i] = accum[i].Normalized();
            }
        }

        private static double AngleBetween(Vec3D a, Vec3D b)
        {
            return Vec3DOps.Angle(a, b);
        }

        /// <summary>
        /// Creates a canonical edge key (always stores smaller index first).
        /// </summary>
        private static (int, int) GetEdgeKey(int v0, int v1)
        {
            return v0 < v1 ? (v0, v1) : (v1, v0);
        }
        
        /// <summary>
        /// Determines if a boundary loop is oriented outward relative to the surface.
        /// Checks a single edge to determine the loop's orientation.
        /// Returns: true if the loop is oriented correctly (outward), false if reversed (needs flipping).
        /// This result is used for both offset direction correction and rim triangle winding.
        /// </summary>
        private static bool DetermineLoopOrientation(
            List<Vec3D> vertices,
            List<Tri> triangles,
            Dictionary<(int, int), TriangleEdge> edgeToTriangleMap,
            List<int> loop,
            List<Vec3D> normals)
        {
            if (loop.Count < 2)
                return true; // Degenerate case, assume correct orientation
            
            // Check the first edge in the loop to determine orientation
            int vertexCurr = loop[0];
            int vertexNext = loop[1];
            
            // Get the boundary edge
            var edgeKey = GetEdgeKey(vertexCurr, vertexNext);
            
            if (!edgeToTriangleMap.TryGetValue(edgeKey, out var edge))
            {
                // Fallback: no triangle info available, assume correct orientation
                return true;
            }
            
            // Get the triangle that contains this boundary edge
            int triIndex = edge.NeighbourIndex1 != -1 ? edge.NeighbourIndex1 : edge.NeighbourIndex2;
            if (triIndex == -1)
            {
                // No valid triangle (shouldn't happen for proper boundaries)
                return true;
            }            
            
            Tri tri = triangles[triIndex];
            int local1 = tri.IndexOf(vertexCurr);
            int local2 = tri.IndexOf(vertexNext);
            if (local1 < 0 || local2 < 0)
                throw new Exception();

            if (local2 < local1)
                local2 += 3;

            return local2 - local1 != 1;
        }
    }
}
