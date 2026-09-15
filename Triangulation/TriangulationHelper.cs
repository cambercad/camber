namespace GeoCore
{
    public static class TriangulationHelper<Arithmetic, Vec, Scalar> where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
    {
        public static Box2D BoundsForSegment(in Box2D p0, in Box2D p1)
        {
            var box = p0;
            box.Extend(p1);
            return box;
        }

        public static bool BboxesDisjoint(in Box2D a, in Box2D b) => !a.OverlapOrTouch(b);

        public static bool SegmentBboxesDisjoint(Arithmetic arithmetic, List<Vec> points, int a0, int a1, int b0, int b1)
        {
            Vec va0 = points[a0];
            Vec va1 = points[a1];
            Vec vb0 = points[b0];
            Vec vb1 = points[b1];

            Scalar minAx = arithmetic.Min(arithmetic.Get(va0, 0), arithmetic.Get(va1, 0));
            Scalar maxAx = arithmetic.Max(arithmetic.Get(va0, 0), arithmetic.Get(va1, 0));
            Scalar minAy = arithmetic.Min(arithmetic.Get(va0, 1), arithmetic.Get(va1, 1));
            Scalar maxAy = arithmetic.Max(arithmetic.Get(va0, 1), arithmetic.Get(va1, 1));

            Scalar minBx = arithmetic.Min(arithmetic.Get(vb0, 0), arithmetic.Get(vb1, 0));
            Scalar maxBx = arithmetic.Max(arithmetic.Get(vb0, 0), arithmetic.Get(vb1, 0));
            Scalar minBy = arithmetic.Min(arithmetic.Get(vb0, 1), arithmetic.Get(vb1, 1));
            Scalar maxBy = arithmetic.Max(arithmetic.Get(vb0, 1), arithmetic.Get(vb1, 1));

            if (arithmetic.Compare(maxAx, minBx) < 0 || arithmetic.Compare(maxBx, minAx) < 0)
                return true;
            if (arithmetic.Compare(maxAy, minBy) < 0 || arithmetic.Compare(maxBy, minAy) < 0)
                return true;
            return false;
        }

        public static bool SegmentsIntersect(Arithmetic arithmetic, List<Vec> points, int idA, int idB, Int2 constraint)
        {
            if (constraint.Contains(idA) && constraint.Contains(idB))
                return false; //Identical segments

            if (SegmentBboxesDisjoint(arithmetic, points, idA, idB, constraint.X, constraint.Y))
                return false;

            return SegmentsIntersectUnchecked(arithmetic, points, idA, idB, constraint);
        }

        /// <summary>Exact segment intersection after optional <see cref="Box2D"/> rejection.</summary>
        public static bool SegmentsIntersectUnchecked(Arithmetic arithmetic, List<Vec> points, int idA, int idB, Int2 constraint)
        {
            if (constraint.Contains(idA) && constraint.Contains(idB))
                return false; //Identical segments

            Vec pa = points[idA];
            Vec pb = points[idB];
            Vec px = points[constraint.X];
            Vec py = points[constraint.Y];

            var x = arithmetic.Orient2D(px, pa, pb);
            var y = arithmetic.Orient2D(py, pa, pb);

            if (x == 0 && y == 0)
            {
                //Colinear segments...
                return CollinearSegmentsOverlap(arithmetic, points, idA, idB, constraint);
            }

            if (constraint.Contains(idA) || constraint.Contains(idB))
                return false; //Segment and constraint are connected at an end point. Since they are not collinear, they don't intersect, just touch

            var a = arithmetic.Orient2D(pa, px, py);
            var b = arithmetic.Orient2D(pb, px, py);

            return Math.Sign(x) != Math.Sign(y) && Math.Sign(a) != Math.Sign(b);
        }

        public static bool CollinearSegmentsOverlap(Arithmetic arithmetic, List<Vec> points, int idA, int idB, Int2 constraint)
        {
            if (constraint.Contains(idA) && constraint.Contains(idB))
                return false; //Identical segments do not intersect

            var pa = points[idA];
            var pb = points[idB];
            var px = points[constraint.X];
            var py = points[constraint.Y];

            //Choose segment ab as reference and fint the principal direction                
            int id = -1;
            if (arithmetic.Compare(arithmetic.Get(pa, 0), arithmetic.Get(pb, 0)) == 0)
                id = 1; //Take y direction if x value of pa and pb is identical
            else
                id = 0;

            return IntervalOverlapNoTouch(arithmetic, arithmetic.Min(arithmetic.Get(pa, id), arithmetic.Get(pb, id)), arithmetic.Max(arithmetic.Get(pa, id), arithmetic.Get(pb, id)),
                arithmetic.Min(arithmetic.Get(px, id), arithmetic.Get(py, id)), arithmetic.Max(arithmetic.Get(px, id), arithmetic.Get(py, id)));
        }

        public static bool IntervalOverlapNoTouch(Arithmetic arithmetic, Scalar startA, Scalar endA, Scalar startB, Scalar endB)
        {
            return !(arithmetic.Compare(endA, startB) <= 0 || arithmetic.Compare(endB, startA) <= 0);
            //return !(endA <= startB || endB <= startA);
        }

    }
}
