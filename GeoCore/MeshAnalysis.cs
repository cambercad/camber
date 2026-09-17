using System.Collections.Generic;
using System.Diagnostics;

namespace GeoCore
{
    public static class DuplicatePointRemover
    {
        /// <summary>
        /// Creates a mapping for duplicate points using exact equality.
        /// Returns a dictionary that maps original point indices to canonical (first occurrence) indices.
        /// </summary>
        /// <param name="points">List of points to process</param>
        /// <returns>Dictionary mapping original index to canonical index</returns>
        public static Dictionary<int, int> DuplicateMap(IList<Vec3D> points)
        {
            var pointMap = new Dictionary<int, int>(); // maps original index to canonical index
            var pointHash = new Dictionary<(double, double, double), int>(); // maps point coordinates to first occurrence index
            
            // Build hash map for efficient duplicate detection using exact equality
            for (int i = 0; i < points.Count; i++)
            {
                var point = points[i];
                var pointKey = (point.X, point.Y, point.Z);
                
                if (pointHash.TryGetValue(pointKey, out int canonicalIndex))
                {
                    // This point is a duplicate of an existing point
                    pointMap[i] = canonicalIndex;
                }
                else
                {
                    // This is the first occurrence of this point
                    pointMap[i] = i;
                    pointHash[pointKey] = i;
                }
            }
            
            return pointMap;
        }

        /// <summary>Canonical indices from authoritative rational coordinates.</summary>
        public static Dictionary<int, int> DuplicateMap(IList<Rat3Hybrid> points)
        {
            var map = new Dictionary<int, int>(points.Count);
            var unique = new Dictionary<Rat3Hybrid, int>();
            for (int i = 0; i < points.Count; i++)
            {
                // Normalize a value copy for hashing; never mutate source geometry.
                var source = points[i];
                var point = new Rat3Hybrid(in source);
                point.Simplify();
                if (!unique.TryGetValue(point, out int canonical))
                    unique.Add(point, canonical = i);
                map.Add(i, canonical);
            }
            return map;
        }

        public static List<Tri> MapTriangles(IList<Tri> triangles, Dictionary<int, int> map)
        {
            List<Tri> result = new List<Tri>(triangles.Count);
            foreach (var tri in triangles)
            {
                int a = map[tri.A];
                int b = map[tri.B];
                int c = map[tri.C];
                // Optionally skip degenerate triangles (where two or more indices are the same)
                if (a == b || b == c || a == c)
                    continue;
                result.Add(new Tri(a, b, c));
            }
            return result;
        }
    }

    public static class MeshAnalysis
    {
        public static bool AreTrianglesConsistentlyOriented(IList<Tri> triangles)
        {
            if (triangles == null || triangles.Count == 0)
                return false;

            // Step 2: Build edge-to-triangles mapping to find adjacent triangles
            var edgeToTriangles = new Dictionary<(int, int), List<(int triangleIndex, bool isReversed)>>();

            for (int triIndex = 0; triIndex < triangles.Count; triIndex++)
            {
                var triangle = triangles[triIndex];

                // Map triangle vertices to canonical indices
                int a = triangle.A;
                int b = triangle.B;
                int c = triangle.C;

                // Skip degenerate triangles
                if (a == b || b == c || a == c)
                    continue;

                // Add the three directed edges of the triangle
                AddDirectedEdge(edgeToTriangles, a, b, triIndex, false);
                AddDirectedEdge(edgeToTriangles, b, c, triIndex, false);
                AddDirectedEdge(edgeToTriangles, c, a, triIndex, false);
            }

            // Step 3: Check orientation consistency for each edge shared by two triangles
            foreach (var kv in edgeToTriangles)
            {
                var edgeTriangles = kv.Value;
                if (edgeTriangles.Count == 2)
                {
                    // This edge is shared by exactly two triangles - check if they have opposite orientations
                    var tri1 = edgeTriangles[0];
                    var tri2 = edgeTriangles[1];

                    // For a consistently oriented mesh, adjacent triangles should traverse shared edges
                    // in opposite directions. If both traverse in the same direction, orientation is inconsistent.
                    if (tri1.isReversed == tri2.isReversed)
                    {
                        TraceOrientationConflict(kv.Key, tri1, tri2, triangles);
                        return false; // Inconsistent orientation detected
                    }
                }
                // Note: We don't check edges with count != 2 here, as that would be a topology issue
                // (boundary edges have count 1, non-manifold edges have count > 2)
            }

            return true; // All shared edges have consistent orientation
        }

        public static bool AreTrianglesConsistentlyOriented(List<Rat3Hybrid> points, List<Tri> triangles, bool allowTouch = false)
        {
            if (points == null || triangles == null || points.Count == 0 || triangles.Count == 0)
                return false;

            //// Step 1: Create a mapping for duplicate points using exact equality (same as IsWatertightMesh)
            //var pointMap = DuplicatePointRemover.DuplicateMap(points);

            //// Step 2: Build edge-to-triangles mapping to find adjacent triangles
            ////var edgeToTriangles = new Dictionary<(int, int), List<(int triangleIndex, bool isReversed)>>();
            
            //List<Tri> mappedTriangles = new List<Tri>(triangles.Count);
            //for (int triIndex = 0; triIndex < triangles.Count; triIndex++)
            //{
            //    var triangle = triangles[triIndex];
                
            //    // Map triangle vertices to canonical indices
            //    int a = pointMap[triangle.A];
            //    int b = pointMap[triangle.B];
            //    int c = pointMap[triangle.C];
                
            //    // Skip degenerate triangles
            //    if (a == b || b == c || a == c)
            //        continue;
                
            //    //// Add the three directed edges of the triangle
            //    //AddDirectedEdge(edgeToTriangles, a, b, triIndex, false);
            //    //AddDirectedEdge(edgeToTriangles, b, c, triIndex, false);
            //    //AddDirectedEdge(edgeToTriangles, c, a, triIndex, false);

            //    mappedTriangles.Add(new Tri(a,b,c));
            //}

            TriangleAdjacency[] adj;
            if (allowTouch)
                adj = AdjacencyEx.BuildAdjacencyInformation(points, triangles);
            else
                adj = Adjacency.BuildAdjacencyInformation(triangles);

            for(int i = 0; i< adj.Count();++i)
            {
                var a = adj[i];
                var tri = triangles[i];

                if (a.NeighbourAB >= 0)
                {
                    var ab = triangles[a.NeighbourAB];
                    if (IsEdgeForward(ab, tri.A, tri.B))
                        return false;
                }

                if (a.NeighbourBC >= 0)
                {
                    var bc = triangles[a.NeighbourBC];
                    if (IsEdgeForward(bc, tri.B, tri.C))
                        return false;
                }

                if (a.NeighbourCA >= 0)
                {
                    var ca = triangles[a.NeighbourCA];
                    if (IsEdgeForward(ca, tri.C, tri.A))
                        return false;
                }
            }
            return true;


            // Step 3: Check orientation consistency for each edge shared by two triangles
            //bool tracedViewerTuple = false;
            //foreach (var kv in edgeToTriangles)
            //{
            //    var edgeTriangles = kv.Value;
            //    if (edgeTriangles.Count == 2)
            //    {
            //        var tri1 = edgeTriangles[0];
            //        var tri2 = edgeTriangles[1];

            //        if (tri1.isReversed == tri2.isReversed)
            //        {
            //            var edge = kv.Key;
            //            if (edge.Item1 >= 0 && edge.Item1 < points.Count && edge.Item2 >= 0 && edge.Item2 < points.Count)
            //            {
            //                if (invalidEdges == null)
            //                    invalidEdges = new List<(Vec3D, Vec3D)>();
            //                invalidEdges.Add((points[edge.Item1], points[edge.Item2]));
            //            }

            //            //if (!tracedViewerTuple)
            //            //{
            //            //    TraceOrientationConflict(edge, tri1, tri2, triangles, points);
            //            //    tracedViewerTuple = true;
            //            //}
            //            //else
            //            //    TraceOrientationConflict(edge, tri1, tri2, triangles);
            //        }
            //    }
            //}

            //return invalidEdges == null;
        }

        private static bool IsEdgeForward(Tri ab, int a, int b)
        {
            var idA = ab.IndexOf(a);
            var idB = ab.IndexOf(b);
            if (idB < idA)
                idB += 3;
            var delta = idB - idA;
            return delta == 1;
        }

        /// <summary>
        /// Signed volume of the solid implied by a triangle mesh, using the origin-based tetrahedron formula
        /// V = (1/6) Σ (a · (b × c)) over triangles (a, b, c in winding order). Degenerate triangles are skipped.
        /// For a closed watertight mesh with outward-facing winding, this is positive.
        /// </summary>
        public static double ComputeSignedMeshVolume(IList<Vec3D> positions, IList<Tri> triangles)
        {
            if (positions == null || triangles == null || triangles.Count == 0)
                return 0;

            double vol6 = 0;
            for (int i = 0; i < triangles.Count; i++)
            {
                var t = triangles[i];
                int ia = t.A, ib = t.B, ic = t.C;
                if ((uint)ia >= (uint)positions.Count || (uint)ib >= (uint)positions.Count || (uint)ic >= (uint)positions.Count)
                    continue;
                if (ia == ib || ib == ic || ia == ic)
                    continue;
                var a = positions[ia];
                var b = positions[ib];
                var c = positions[ic];
                vol6 += Vec3DOps.Dot(a, Vec3DOps.Cross(b, c));
            }

            return vol6 / 6.0;
        }

        /// <summary>
        /// Exact signed volume in coordinate units cubed. Canonicalizing each
        /// term and partial sum prevents redundant denominator factors from
        /// growing with the number of triangles.
        /// </summary>
        public static BigRationalHybrid ComputeSignedMeshVolume(IList<Rat3Hybrid> positions, IList<Tri> triangles)
        {
            var sixVolume = new BigRationalHybrid(0);
            if (positions == null || triangles == null || positions.Count == 0 || triangles.Count == 0)
                return sixVolume;
            var origin = positions[0];
            foreach (var triangle in triangles)
            {
                var term = Rat3Hybrid.Dot(positions[triangle.A] - origin,
                    Rat3Hybrid.Cross(positions[triangle.B] - origin, positions[triangle.C] - origin));
                term.Simplify();
                sixVolume += term;
                sixVolume.Simplify();
            }
            var volume = sixVolume / new BigRationalHybrid(6);
            volume.Simplify();
            return volume;
        }

        public static bool IsWatertightMesh(IList<Vec3D> points, IList<Tri> triangles, bool allowTouch = false)
        {
            return IsWatertightMesh(points, triangles, out _, allowTouch);
        }
        public static bool IsWatertightMesh(IList<Vec3D> points, IList<Tri> triangles, out List<Vec3D> problematicEdges, bool allowTouch = false)
        {
            problematicEdges = new List<Vec3D>();
            if (points == null || triangles == null || points.Count == 0 || triangles.Count == 0)
                return false;
            var mapped = DuplicatePointRemover.MapTriangles(triangles, DuplicatePointRemover.DuplicateMap(points));
            foreach (var edge in InvalidIncidenceEdges(mapped, allowTouch))
            {
                problematicEdges.Add(points[edge.Item1]);
                problematicEdges.Add(points[edge.Item2]);
            }
            return problematicEdges.Count == 0;
        }

        /// <summary>
        /// Validate authoritative CAD topology without merging distinct rational
        /// vertices that happen to round to the same display-space double.
        /// </summary>
        public static bool IsWatertightMesh(IList<Rat3Hybrid> points, IList<Tri> triangles, bool allowTouch = false)
        {
            if (points == null || triangles == null || points.Count == 0 || triangles.Count == 0)
                return false;
            var mapped = DuplicatePointRemover.MapTriangles(triangles, DuplicatePointRemover.DuplicateMap(points));
            return IsWatertightMesh(mapped, allowTouch);
        }

        public static bool IsWatertightMesh(IList<Tri> triangles, bool allowTouch = false)
        {
            return triangles != null && triangles.Count != 0 && InvalidIncidenceEdges(triangles, allowTouch).Count == 0;
        }

        private static List<(int, int)> InvalidIncidenceEdges(IList<Tri> triangles, bool allowTouch)
        {
            var counts = new Dictionary<(int, int), int>();
            foreach (var triangle in triangles)
            {
                int a = triangle.A, b = triangle.B, c = triangle.C;
                if (a == b || b == c || a == c)
                    continue;
                AddEdge(counts, a, b);
                AddEdge(counts, b, c);
                AddEdge(counts, c, a);
            }
            var invalid = new List<(int, int)>();
            foreach (var edge in counts)
                if (allowTouch ? edge.Value % 2 != 0 : edge.Value != 2)
                    invalid.Add(edge.Key);
            return invalid;
        }

        public static bool ContainsDuplicatePoints(List<Rat3Hybrid> precisionPositions)
        {
            HashSet<Rat3Hybrid> set = new HashSet<Rat3Hybrid>();
            for (int i = 0; i < precisionPositions.Count; i++)
            {
                var copy = precisionPositions[i];
                copy.Simplify();

                if (!set.Add(copy))
                    return true;

                precisionPositions[i] = copy;
            }
            return false;
        }

        /// <summary>
        /// Logs the first detected orientation conflict (shared manifold edge traversed the same way by both triangles).
        /// <paramref name="edge"/> uses canonical vertex indices (min,max) as stored in the edge map.
        /// <paramref name="isReversed"/> per triangle: false = directed as (low→high), true = (high→low) along that canonical pair.
        /// </summary>
        private static void TraceOrientationConflict(
            (int, int) edge,
            (int triangleIndex, bool isReversed) tri1,
            (int triangleIndex, bool isReversed) tri2,
            IList<Tri> triangles)
        {
            Trace.WriteLine("MeshAnalysis: inconsistent triangle winding — two adjacent faces use the same direction along a shared edge (typical CSG/orientation bug upstream).");
            Trace.WriteLine($"  canonical edge vertex indices: ({edge.Item1}, {edge.Item2}); reversed flags (should differ): tri {tri1.triangleIndex} → {tri1.isReversed}, tri {tri2.triangleIndex} → {tri2.isReversed}");
            if (tri1.triangleIndex >= 0 && tri1.triangleIndex < triangles.Count
                && tri2.triangleIndex >= 0 && tri2.triangleIndex < triangles.Count)
            {
                var a = triangles[tri1.triangleIndex];
                var b = triangles[tri2.triangleIndex];
                Trace.WriteLine($"  tri {tri1.triangleIndex} corner indices (A,B,C): ({a.A},{a.B},{a.C})");
                Trace.WriteLine($"  tri {tri2.triangleIndex} corner indices (A,B,C): ({b.A},{b.B},{b.C})");
            }
        }

        private static void TraceOrientationConflict(
            (int, int) edge,
            (int triangleIndex, bool isReversed) tri1,
            (int triangleIndex, bool isReversed) tri2,
            IList<Tri> triangles,
            IList<Vec3D> points)
        {
            TraceOrientationConflict(edge, tri1, tri2, triangles);
            if (edge.Item1 >= 0 && edge.Item1 < points.Count && edge.Item2 >= 0 && edge.Item2 < points.Count)
            {
                var p0 = points[edge.Item1];
                var p1 = points[edge.Item2];
                Trace.WriteLine($"  edge segment in space (canonical vertex indices): ({p0.X:G9},{p0.Y:G9},{p0.Z:G9}) — ({p1.X:G9},{p1.Y:G9},{p1.Z:G9})");
                // GeoScriptViewer.DebugTracing.Write(object) draws line pairs + point markers for this tuple.
                Trace.Write(Tuple.Create(
                    "MeshAnalysis: winding conflict (shared edge)",
                    new Vec3D(1, 0.25, 1),
                    new List<Vec3D> { p0, p1 }));
            }
        }

        private static void AddDirectedEdge(Dictionary<(int, int), List<(int triangleIndex, bool isReversed)>> edgeToTriangles, 
                                           int v1, int v2, int triangleIndex, bool isReversed)
        {
            // Store directed edge as-is, but also store the canonical form for lookup
            var canonicalEdge = v1 < v2 ? (v1, v2) : (v2, v1);
            bool isReversedFromCanonical = (v1 > v2) != isReversed;
            
            if (!edgeToTriangles.ContainsKey(canonicalEdge))
                edgeToTriangles[canonicalEdge] = new List<(int, bool)>();
            
            edgeToTriangles[canonicalEdge].Add((triangleIndex, isReversedFromCanonical));
        }

        private static void AddEdge(Dictionary<(int, int), int> edgeCount, int v1, int v2)
        {
            // Ensure consistent edge representation (smaller index first)
            var edge = v1 < v2 ? (v1, v2) : (v2, v1);
            
            if (edgeCount.ContainsKey(edge))
                edgeCount[edge]++;
            else
                edgeCount[edge] = 1;
        }
    }
}
