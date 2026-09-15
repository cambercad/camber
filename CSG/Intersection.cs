using System.Threading;
using GeoCore;

namespace CSG
{
    public enum SegmentTriangleIntersectionType
    {
        Intersect,
        Coplanar,
        NoIntersection
    }

    public static class TriangleSegmentIntersector
    {
        public static long AabbCullCount;
        public static long FullTestCount;
        //https://www.nas.nasa.gov/publications/software/docs/cart3d/pages/bool_intersection.html
        
        public static SegmentTriangleIntersectionType SegmentIntersectsTriangle(in Rat3Hybrid segmentStart, in Rat3Hybrid segmentEnd,
           in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c, out Rat3Hybrid intersection, out bool intersectionPointIsOnBoundary, out bool startIsOnTriangle, out bool endIsOnTriangle)
        {
            intersectionPointIsOnBoundary = false;
            startIsOnTriangle = false;
            endIsOnTriangle = false;

            if (SegmentTriangleAabbDisjoint(in segmentStart, in segmentEnd, in a, in b, in c))
            {
                Interlocked.Increment(ref AabbCullCount);
                intersection = default;
                return SegmentTriangleIntersectionType.NoIntersection;
            }
            Interlocked.Increment(ref FullTestCount);

            int ab = Orient3DSign(segmentStart, segmentEnd, a, b);
            int bc = Orient3DSign(segmentStart, segmentEnd, b, c);
            int ac = -Orient3DSign(segmentStart, segmentEnd, a, c);

            if ((ab >= 0 && bc >= 0 && ac >= 0) || (ab <= 0 && bc <= 0 && ac <= 0))
            {
                intersectionPointIsOnBoundary = ab == 0 || bc == 0 || ac == 0;

                int up = Orient3DSign(segmentStart, a, b, c);
                int down = Orient3DSign(segmentEnd, a, b, c);

                if (up == 0 && down == 0 /*ab == 0 && bc == 0 && ac == 0*/)
                {
                    //Actually all 5 values are zero in this case
                    //Coplanar
                    intersection = default;
                    return SegmentTriangleIntersectionType.Coplanar;
                }
                else if (up == 0)
                {
                    intersection = segmentStart;
                    startIsOnTriangle = true;
                    return SegmentTriangleIntersectionType.Intersect;
                }
                else if (down == 0)
                {
                    intersection = segmentEnd;
                    endIsOnTriangle = true;
                    return SegmentTriangleIntersectionType.Intersect;
                }
                else if (Math.Sign(up) != Math.Sign(down))
                {
                    intersection = PlaneLineIntersection(segmentStart, segmentEnd, a, b, c);
                    return SegmentTriangleIntersectionType.Intersect;
                }
            }

            intersection = default;
            return SegmentTriangleIntersectionType.NoIntersection;
        }

        public static Rat3Hybrid PlaneLineIntersection(in Rat3Hybrid segmentStart, in Rat3Hybrid segmentEnd, in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c)
        {
            // Compute the normal of the plane
            var normal = Rat3Hybrid.Cross(b - a, c - a);

            var dir = segmentEnd - segmentStart;

            // Compute the numerator and denominator for t
            var num = Rat3Hybrid.Dot(normal, a - segmentStart);
            var denom = Rat3Hybrid.Dot(normal, dir);

            // Compute t
            var t = num / denom;

            // Compute the intersection point
            return segmentStart + t * dir;
        }

        public static SegmentTriangleIntersectionType SegmentIntersectsPolygon(in Rat3Hybrid segmentStart, in Rat3Hybrid segmentEnd,
           IList<Rat3Hybrid> polygon, out bool intersectionPointIsOnBoundary, out bool startIsOnTriangle, out bool endIsOnTriangle)
        {
            int[] signs = new int[polygon.Count];
            for (int i = 0; i < polygon.Count; i++)
                signs[i] = Orient3DSign(segmentStart, segmentEnd, polygon[i], polygon[(i + 1) % polygon.Count]);

            intersectionPointIsOnBoundary = false;
            startIsOnTriangle = false;
            endIsOnTriangle = false;

            bool allLargerEqualZero = true;
            bool allSmallerEqualZero = true;
            bool anyZero = false;
            for (int i = 0; i < polygon.Count; i++)
            {
                allLargerEqualZero = allLargerEqualZero && (signs[i] >= 0);
                allSmallerEqualZero = allSmallerEqualZero && (signs[i] <= 0);
                anyZero = anyZero || (signs[i] == 0);
            }

            if (allLargerEqualZero || allSmallerEqualZero)
            {
                intersectionPointIsOnBoundary = anyZero;

                int up = Orient3DSign(segmentStart, polygon[0], polygon[1], polygon[2]);
                int down = Orient3DSign(segmentEnd, polygon[0], polygon[1], polygon[2]);

                if (up == 0 && down == 0 /*ab == 0 && bc == 0 && ac == 0*/)
                {
                    //Actually all 5 values are zero in this case
                    //Coplanar
                    startIsOnTriangle = true;
                    endIsOnTriangle = true;
                    return SegmentTriangleIntersectionType.Coplanar;
                }
                else if (up == 0)
                {
                    startIsOnTriangle = true;
                    return SegmentTriangleIntersectionType.Intersect;
                }
                else if (down == 0)
                {
                    endIsOnTriangle = true;
                    return SegmentTriangleIntersectionType.Intersect;
                }
                else if (Math.Sign(up) != Math.Sign(down))
                {
                    return SegmentTriangleIntersectionType.Intersect;
                }
            }

            return SegmentTriangleIntersectionType.NoIntersection;
        }

        public static int Orient3DSign(in Rat3Hybrid pa, in Rat3Hybrid pb, in Rat3Hybrid pc, in Rat3Hybrid pd) =>
            BigRationalHybrid.SignOfOrient3D(
                pa.X, pa.Y, pa.Z, pb.X, pb.Y, pb.Z, pc.X, pc.Y, pc.Z, pd.X, pd.Y, pd.Z);

        private static bool SegmentTriangleAabbDisjoint(
            in Rat3Hybrid segmentStart, in Rat3Hybrid segmentEnd,
            in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c)
        {
            if (AxisDisjoint(segmentStart.X, segmentEnd.X, a.X, b.X, c.X))
                return true;
            if (AxisDisjoint(segmentStart.Y, segmentEnd.Y, a.Y, b.Y, c.Y))
                return true;
            if (AxisDisjoint(segmentStart.Z, segmentEnd.Z, a.Z, b.Z, c.Z))
                return true;
            return false;
        }

        private static bool AxisDisjoint(
            in BigRationalHybrid seg0, in BigRationalHybrid seg1,
            in BigRationalHybrid t0, in BigRationalHybrid t1, in BigRationalHybrid t2) =>
            BigRationalHybrid.AxisSeparated(seg0, seg1, t0, t1, t2);
    }
}
