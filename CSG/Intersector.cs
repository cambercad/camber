using GeoCore;

namespace CSG
{
    public struct LineSegmentOnTriangle
    {
        public Rat3Hybrid PointStart;
        public Rat3Hybrid PointEnd;
        public int TriangleId;

        public LineSegmentOnTriangle(Rat3Hybrid pointStart, Rat3Hybrid pointEnd, int triangleId)
        {
            PointStart = pointStart;
            PointEnd = pointEnd;
            TriangleId = triangleId;
        }
    }
    public struct LineSegmentOnTriangleEx
    {
        public Rat3Hybrid BarycentricCoordsStart;
        public Rat3Hybrid BarycentricCoordsEnd;
        public Rat3Hybrid PointStart;
        public Rat3Hybrid PointEnd;
        public int TriangleId;

        public LineSegmentOnTriangleEx(Rat3Hybrid barycentricCoordsStart, Rat3Hybrid barycentricCoordsEnd, Rat3Hybrid pointStart, Rat3Hybrid pointEnd, int triangleId)
        {
            BarycentricCoordsStart = barycentricCoordsStart;
            BarycentricCoordsEnd = barycentricCoordsEnd;
            PointStart = pointStart;
            PointEnd = pointEnd;
            TriangleId = triangleId;
        }
    }

    // Can give the intersection between meshes as line strips (without modifying the meshes)
    // Can give the intersection between meshes as line strips and resolve the meshes such that these intersection edges are present in both meshes
    public static class Intersector
    {
        public static List<Rat3Hybrid> TrimLineStrip(List<Rat3Hybrid> lineStrip, UVSurface surface, bool trimInNormalDirection = true)
        {
            return TrimLineStrip(lineStrip, surface.PointsPrecise, surface.Triangles, trimInNormalDirection);
        }

        // Trims the line strip against the triangles
        // If trimInNormalDirection is true (default), removes the part in the direction of triangle normals
        // If trimInNormalDirection is false, removes the part against the direction of triangle normals
        public static List<Rat3Hybrid> TrimLineStrip(List<Rat3Hybrid> lineStrip, List<Rat3Hybrid> pointsA, List<Tri> trianglesA, bool trimInNormalDirection = true)
        {
            if (lineStrip == null || lineStrip.Count < 2)
                return new List<Rat3Hybrid>();
            
            if (trianglesA == null || trianglesA.Count == 0)
                return new List<Rat3Hybrid>(lineStrip);


            // Find all intersections with triangles
            // Multiple triangles can intersect at the same point
            Dictionary<Rat3Hybrid, List<(int, int)>> intersections = new Dictionary<Rat3Hybrid, List<(int, int)>>();

            // Process each segment in the line strip
            for (int segIdx = 0; segIdx < lineStrip.Count - 1; segIdx++)
            {
                Rat3Hybrid segStart = lineStrip[segIdx];
                Rat3Hybrid segEnd = lineStrip[segIdx + 1];
                
                for (int triIdx = 0; triIdx < trianglesA.Count; triIdx++)
                {
                    Tri tri = trianglesA[triIdx];
                    Rat3Hybrid a = pointsA[tri.A];
                    Rat3Hybrid b = pointsA[tri.B];
                    Rat3Hybrid c = pointsA[tri.C];
                    
                    SegmentTriangleIntersectionType intersectionType = TriangleSegmentIntersector.SegmentIntersectsTriangle(
                        segStart, segEnd, a, b, c, 
                        out Rat3Hybrid intersection, 
                        out bool intersectionPointIsOnBoundary, 
                        out bool startIsOnTriangle, 
                        out bool endIsOnTriangle);
                    
                    if (intersectionType == SegmentTriangleIntersectionType.Intersect)
                    {
                        intersection.Simplify();

                        // Only count intersections that are strictly between segment endpoints
                        //if (!startIsOnTriangle && !endIsOnTriangle)
                        {
                            if (!intersections.ContainsKey(intersection))
                            {
                                intersections[intersection] = new List<(int, int)>();
                            }                            
                            intersections[intersection].Add((segIdx, triIdx));
                        }
                    }
                    else if (intersectionType == SegmentTriangleIntersectionType.Coplanar)
                    {

                    }
                }
            }

            // Check constraint: at most one intersection point (but multiple triangles at that point are OK)
            if (intersections.Count > 1)
            {
                throw new InvalidOperationException($"Segment has {intersections.Count} intersection points. Expected at most 1.");
            }

            // Trim the segment if there's an intersection
            if (intersections.Count == 1)
            {
                List<Rat3Hybrid> firstHalf = new List<Rat3Hybrid>();
                List<Rat3Hybrid> secondHalf = new List<Rat3Hybrid>();

                var f = intersections.First();
                var intersectionPoint = f.Key;
                var list = f.Value;
                var (segId, triIdx) = list[0];
                
                // Build first half: from start to intersection point
                for(int i = 0; i <= segId; ++i)
                {
                    if(firstHalf.Count == 0 || lineStrip[i] != firstHalf[firstHalf.Count - 1])
                        firstHalf.Add(lineStrip[i]);
                }
                if (firstHalf.Count == 0 || intersectionPoint != firstHalf[firstHalf.Count - 1])
                    firstHalf.Add(intersectionPoint);

                // Build second half: from intersection point to end
                secondHalf.Add(intersectionPoint);
                for (int i = segId + 1; i < lineStrip.Count; ++i)
                {
                    if (lineStrip[i] != secondHalf[secondHalf.Count - 1])
                        secondHalf.Add(lineStrip[i]);
                }

                // Decide which half to return based on triangle normal direction
                Tri tri = trianglesA[triIdx];
                Rat3Hybrid a = pointsA[tri.A];
                Rat3Hybrid b = pointsA[tri.B];
                Rat3Hybrid c = pointsA[tri.C];
                
                // Calculate triangle normal
                Rat3Hybrid normal = Rat3Hybrid.Cross(b - a, c - a);
                
                // Get direction of the line strip at intersection
                Rat3Hybrid segStart = lineStrip[segId];
                Rat3Hybrid segEnd = lineStrip[segId + 1];
                Rat3Hybrid segDirection = segEnd - segStart;
                
                // Determine which side of the intersection is in the normal direction
                int dotSign = Rat3Hybrid.DotSign(in segDirection, in normal);

                if (trimInNormalDirection)
                {
                    // Remove the part in normal direction (keep first half if dot > 0, second half if dot < 0)
                    if (dotSign > 0)
                        return firstHalf;
                    else
                        return secondHalf;
                }
                else
                {
                    // Remove the part against normal direction (keep second half if dot > 0, first half if dot < 0)
                    if (dotSign > 0)
                        return secondHalf;
                    else
                        return firstHalf;
                }
            }
            else
            {
                return new List<Rat3Hybrid>(lineStrip);
            }
        }

        // Only works if p is exactly on the plane defined by a,b,c
        public static Rat3Hybrid ComputeBarycentricCoordinates(Rat3Hybrid p, Rat3Hybrid a, Rat3Hybrid b, Rat3Hybrid c)
        {
            // Normal of the triangle (no normalization needed)
            var n = Rat3Hybrid.Cross(b - a, c - a);

            // Denominator (proportional to 2 * area of ABC)
            var denom = Rat3Hybrid.Dot(n, n);  

            // Numerators (signed sub-areas, same normal projection)
            var u = Rat3Hybrid.Dot(Rat3Hybrid.Cross(b - p, c - p), n);
            var v = Rat3Hybrid.Dot(Rat3Hybrid.Cross(c - p, a - p), n);
            var w = denom - u - v;

            var result = new Rat3Hybrid(u / denom, v / denom, w / denom);

#if DEBUG
            var debug = result.X * a + result.Y * b + result.Z * c;
            if (debug != p)
                throw new Exception();
#endif

            return result;
        }

        public static (List<LineSegmentOnTriangleEx> intersectionA, List<LineSegmentOnTriangleEx> intersectionB) 
            IntersectSurfaces(UVSurface a, UVSurface b, CoordinateConverter converter)
        {

            List<Rat3Hybrid> pointsA = a.PointsPrecise; 
            List<Rat3Hybrid> pointsB = b.PointsPrecise; 

            List<List<IntersectionSegmentEx>> intersectionStrips = new List<List<IntersectionSegmentEx>>();

            // Resolve can also be used to get the intersection line segments
            Resolver.Resolve(BooleanOp.NoOpIntersectionContourOnly, pointsA, a.Triangles, pointsB, b.Triangles,
                out var resultPoints, out var resultTriangles, out var sourceTriangleIndex, intersectionStrips: intersectionStrips);

            // Convert intersection strips to LineSegmentOnTriangle for both meshes
            List<LineSegmentOnTriangleEx> intersectionSegmentsA = new List<LineSegmentOnTriangleEx>();
            List<LineSegmentOnTriangleEx> intersectionSegmentsB = new List<LineSegmentOnTriangleEx>();

            foreach (var strip in intersectionStrips)
            {
                foreach (var seg in strip)
                {
                    // For mesh A
                    Tri triA = a.Triangles[seg.TriIdA];
                    Rat3Hybrid triA_A = pointsA[triA.A];
                    Rat3Hybrid triA_B = pointsA[triA.B];
                    Rat3Hybrid triA_C = pointsA[triA.C];

                    Rat3Hybrid baryStartA = ComputeBarycentricCoordinates(seg.StartPoint, triA_A, triA_B, triA_C);
                    Rat3Hybrid baryEndA = ComputeBarycentricCoordinates(seg.EndPoint, triA_A, triA_B, triA_C);

                    intersectionSegmentsA.Add(new LineSegmentOnTriangleEx(
                        baryStartA, baryEndA, seg.StartPoint, seg.EndPoint, seg.TriIdA));

                    // For mesh B
                    Tri triB = b.Triangles[seg.TriIdB];
                    Rat3Hybrid triB_A = pointsB[triB.A];
                    Rat3Hybrid triB_B = pointsB[triB.B];
                    Rat3Hybrid triB_C = pointsB[triB.C];

                    Rat3Hybrid baryStartB = ComputeBarycentricCoordinates(seg.StartPoint, triB_A, triB_B, triB_C);
                    Rat3Hybrid baryEndB = ComputeBarycentricCoordinates(seg.EndPoint, triB_A, triB_B, triB_C);

                    intersectionSegmentsB.Add(new LineSegmentOnTriangleEx(
                        baryStartB, baryEndB, seg.StartPoint, seg.EndPoint, seg.TriIdB));
                }
            }

            return (intersectionSegmentsA, intersectionSegmentsB);
        }
    }
}
