using GeoCore;

namespace CSG
{
    public class Surface
    {
        public List<Tri> Triangles;
        public List<int> GroupIdPerTriangle;
        public List<Rat3Hybrid> PointsPrecise;

        public Surface() { }

        public Surface(List<Tri> triangles, List<Rat3Hybrid> pointsPrecise, List<int> groupIdPerTriangle)
        {
            Triangles = triangles;
            GroupIdPerTriangle = groupIdPerTriangle;
            PointsPrecise = pointsPrecise;
        }

        public Surface(List<Tri> triangles, List<Rat3Hybrid> pointsPrecise, int groupId = 0)
        {
            Triangles = triangles;
            PointsPrecise = pointsPrecise;

            GroupIdPerTriangle = new List<int>(triangles.Count);
            for (int i = 0; i < triangles.Count; i++)
                GroupIdPerTriangle.Add(groupId);
        }
    }

    public static class Splitter
    {
        /// <summary>
        /// Splits a Surface using trim curves (line strips on triangles).
        /// Each trim curve acts as a boundary that divides the surface into separate regions.
        /// 
        /// ASSUMPTIONS:
        /// - Trim curve points lie EXACTLY on their specified triangles (uses exact rational arithmetic)
        /// - If points don't lie exactly on triangles, triangulation will produce incorrect results
        /// - Surface triangles are non-degenerate (non-zero area)
        /// - Trim curves are well-formed (no problematic self-intersections within a single triangle)
        /// 
        /// BEHAVIOR:
        /// - Triangles intersected by trim curves are subdivided using constrained Delaunay triangulation
        /// - When a triangle is subdivided, ALL resulting triangles inherit the parent's group ID
        /// - Connected regions (not separated by trim curves) become separate output surfaces
        /// - Each output surface has independent, compact point indexing (points may be duplicated across surfaces)
        /// </summary>
        /// <param name="surface">The surface to split. Must have PointsPrecise and GroupIdPerTriangle populated.</param>
        /// <param name="trimCurves">List of trim curves, where each curve is a list of line segments on triangles.</param>
        /// <param name="throwOnInvalidTriangleIndex">If true, throws an exception when a trim segment references an invalid triangle index. If false, ignores such segments.</param>
        /// <param name="validateTrimCurves">If true, validates that trim curve points actually lie on their specified triangles (expensive). Throws ArgumentException if validation fails.</param>
        /// <returns>List of Surfaces, one for each connected region. GroupIdPerTriangle is preserved and inherited during subdivision.</returns>
        public static List<Surface> SplitUsingLineStrip(Surface surface, List<List<LineSegmentOnTriangle>> trimCurves, bool throwOnInvalidTriangleIndex = false, bool validateTrimCurves = false)
        {
            // Step 1: Initialize point creator with surface points
            NewPointCreator newPoints = new NewPointCreator();
            
            // Require precise points for exact arithmetic
            List<Rat3Hybrid> sourcePoints = surface.PointsPrecise;
            if (sourcePoints == null)
            {
                throw new ArgumentException("Surface.PointsPrecise must be provided for exact arithmetic splitting. " +
                    "The trim curves are known to lie exactly on the surface when using exact math.");
            }
            
            // Validate GroupIdPerTriangle
            if (surface.GroupIdPerTriangle == null || surface.GroupIdPerTriangle.Count != surface.Triangles.Count)
            {
                throw new ArgumentException("Surface.GroupIdPerTriangle must be provided and have the same count as Triangles.");
            }
            
            // Add all points to the point creator
            int[] pointMap = new int[sourcePoints.Count];
            for (int i = 0; i < sourcePoints.Count; i++)
            {
                bool isDuplicate;
                pointMap[i] = newPoints.GetIndex(sourcePoints[i], out isDuplicate);
            }
            
            newPoints.EndInitialize();
            
            // Step 2: Create ResolverTriangles for each triangle in the surface
            List<ResolverTriangle> resolverTris = new List<ResolverTriangle>(surface.Triangles.Count);
            for (int i = 0; i < surface.Triangles.Count; i++)
            {
                var tri = surface.Triangles[i];
                resolverTris.Add(new ResolverTriangle(
                    pointMap[tri.A],
                    pointMap[tri.B],
                    pointMap[tri.C],
                    i,
                    newPoints/*,
                    false, false, false*/));
            }
            
            // Step 3: Add trim curve segments to the appropriate ResolverTriangles
            foreach (var trimCurve in trimCurves)
            {
                foreach (var segment in trimCurve)
                {
                    // Validate triangle index
                    if (segment.TriangleId < 0 || segment.TriangleId >= surface.Triangles.Count)
                    {
                        if (throwOnInvalidTriangleIndex)
                        {
                            throw new ArgumentException($"Trim segment has invalid triangle index {segment.TriangleId}. Surface has {surface.Triangles.Count} triangles.");
                        }
                        else
                        {
                            // Ignore this segment
                            continue;
                        }
                    }
                    
                    var resolverTri = resolverTris[segment.TriangleId];
                    
                    // Optional validation: Check that segment points actually lie on the triangle
                    if (validateTrimCurves)
                    {
                        ValidateTrimSegmentOnTriangle(segment, resolverTri, newPoints, surface.Triangles[segment.TriangleId]);
                    }
                    
                    resolverTri.AddSegment(new PointPair(segment.PointStart, segment.PointEnd));
                }
            }
            
            // Step 4: Prepare for triangulation
            ResolverTriangle.ArrangeTrimSegments(resolverTris, newPoints);
            Dictionary<long, int> insertedSegments = new Dictionary<long, int>();
            for (int i = 0; i < resolverTris.Count; i++)
            {
                resolverTris[i].PrepareTriangulate(newPoints, insertedSegments);
            }
            
            // Step 5: Triangulate each ResolverTriangle
            var resultTriangles = new List<Tri>[resolverTris.Count];
            for (int i = 0; i < resolverTris.Count; i++)
            {
                var resolverTri = resolverTris[i];
                
                List<Tri> tris;
                if (!resolverTri.ContainsIntersections())
                {
                    // No subdivisions needed
                    tris = null; // Will be replaced with original triangle
                }
                else
                {
                    tris = resolverTri.Triangulate(newPoints);
                }
                resultTriangles[i] = tris!;
            }
            
            // Step 6: Collect all triangles into a single list
            var allTriangles = new List<Tri>();
            var allGroupIds = new List<int>(); // Group IDs for all triangles (inherited from parent)
            
            for (int i = 0; i < resultTriangles.Length; i++)
            {
                var tris = resultTriangles[i];
                var resolverTri = resolverTris[i];
                int originalTriIndex = resolverTri.Source; // Get the original triangle index
                int groupId = surface.GroupIdPerTriangle[originalTriIndex]; // Get group ID from original surface
                
                if (tris == null)
                {
                    // No subdivisions, use original triangle
                    allTriangles.Add(new Tri(resolverTri.A, resolverTri.B, resolverTri.C));
                    allGroupIds.Add(groupId);
                }
                else
                {
                    // Add subdivided triangles - all inherit the same group ID
                    foreach (var tri in tris)
                    {
                        allTriangles.Add(tri);
                        allGroupIds.Add(groupId);
                    }
                }
            }

            List<Rat3Hybrid> allPoints = newPoints.ExportPoints();

            // Step 7: Clusterize - group triangles that aren't separated by trim curves
            HashSet<long> boundaries = new HashSet<long>(insertedSegments.Keys);
            List<List<int>> clusters = Resolver.Clusterize(allPoints, boundaries, allTriangles);
            
            // Step 8: Create a separate Surface for each cluster
            List<Surface> result = new List<Surface>(clusters.Count);

         
            
            foreach (var cluster in clusters)
            {
                if (cluster.Count == 0)
                    continue;
                
                // Collect all unique point indices used in this cluster
                HashSet<int> usedPointIndices = new HashSet<int>();
                foreach (int triIndex in cluster)
                {
                    var tri = allTriangles[triIndex];
                    usedPointIndices.Add(tri.A);
                    usedPointIndices.Add(tri.B);
                    usedPointIndices.Add(tri.C);
                }
                
                // Create point mapping: old index -> new index
                Dictionary<int, int> pointRemapping = new Dictionary<int, int>();
                List<Rat3Hybrid> clusterPointsPrecise = new List<Rat3Hybrid>(usedPointIndices.Count);
                
                foreach (int oldIndex in usedPointIndices.OrderBy(x => x))
                {
                    int newIndex = clusterPointsPrecise.Count;
                    pointRemapping[oldIndex] = newIndex;
                    
                    var point = allPoints[oldIndex];
                    clusterPointsPrecise.Add(point);
                }
                
                // Remap triangles and group IDs
                List<Tri> clusterTriangles = new List<Tri>(cluster.Count);
                List<int> clusterGroupIds = new List<int>(cluster.Count);
                
                foreach (int triIndex in cluster)
                {
                    var tri = allTriangles[triIndex];
                    clusterTriangles.Add(new Tri(
                        pointRemapping[tri.A],
                        pointRemapping[tri.B],
                        pointRemapping[tri.C]));
                    
                    // Copy group ID (inherited from parent triangle)
                    clusterGroupIds.Add(allGroupIds[triIndex]);
                }
                
                // Create the Surface for this cluster
                Surface clusterSurface = new Surface
                {
                    Triangles = clusterTriangles,
                    GroupIdPerTriangle = clusterGroupIds,
                    PointsPrecise = clusterPointsPrecise
                };
                
                result.Add(clusterSurface);
            }
            
            return result;
        }
        
        /// <summary>
        /// Validates that a trim segment's points actually lie on the specified triangle.
        /// Uses exact rational arithmetic to verify the points are in the triangle's plane
        /// and within its boundaries.
        /// </summary>
        private static void ValidateTrimSegmentOnTriangle(LineSegmentOnTriangle segment, ResolverTriangle resolverTri, NewPointCreator newPoints, Tri originalTri)
        {
            // Get the triangle vertices
            var triA = newPoints.GetPoint(resolverTri.A);
            var triB = newPoints.GetPoint(resolverTri.B);
            var triC = newPoints.GetPoint(resolverTri.C);
            
            // Check both segment endpoints
            ValidatePointOnTriangle(segment.PointStart, triA, triB, triC, segment.TriangleId, "start");
            ValidatePointOnTriangle(segment.PointEnd, triA, triB, triC, segment.TriangleId, "end");
        }
        
        /// <summary>
        /// Validates that a point lies on a triangle using exact rational arithmetic.
        /// </summary>
        private static void ValidatePointOnTriangle(Rat3Hybrid point, Rat3Hybrid triA, Rat3Hybrid triB, Rat3Hybrid triC, int triangleId, string pointLabel)
        {
            // Create a plane from the triangle
            var normal = Rat3Hybrid.Cross(triB - triA, triC - triA);
            var plane = new Rat3HybridPlane(normal, triA);
            
            // Check if point is in the plane
            var distance = plane.SignedDistancePointPlane(point);
            if (distance != BigRationalHybrid.Zero)
            {
                throw new ArgumentException(
                    $"Trim segment {pointLabel} point is not in the plane of triangle {triangleId}. " +
                    $"Distance from plane: {distance.ToDouble()}. " +
                    $"Trim curves must lie exactly on their specified triangles.");
            }
            
            // Check if point is inside or on the boundary of the triangle
            var poly = new PlaneConvexPolygon(triA, triB, triC);
            bool isInside = poly.PointIsInsideOrOnBoundary(point, out bool isOnBoundary);
            
            if (!isInside)
            {
                throw new ArgumentException(
                    $"Trim segment {pointLabel} point is outside the bounds of triangle {triangleId}. " +
                    $"Point: ({point.X.ToDouble()}, {point.Y.ToDouble()}, {point.Z.ToDouble()}). " +
                    $"Trim curve points must lie within their specified triangles.");
            }
        }
    }
}
