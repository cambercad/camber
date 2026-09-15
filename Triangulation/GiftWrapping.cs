namespace GeoCore
{
    public class GiftWrappingInt2 : GiftWrapping<Int2Arithmetic, Int2, int> { }

    //Convex hull
    public class GiftWrapping<Arithmetic, Vec, Scalar> where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar> //where Vec : new()
    {
        private List<Vec> polygon;
        private IList<int> indexMap;
        private Arithmetic arithmetic = new Arithmetic();

        public void Initialize(List<Vec> points, IList<int> indexMap = null)
        {
            if (indexMap == null)
                this.polygon = points;
            else
            {
                this.polygon = new List<Vec>(indexMap.Count);
                for (int i = 0; i < indexMap.Count; ++i)
                    polygon.Add(points[indexMap[i]]);
                this.indexMap = indexMap;
            }
        }

        public static List<int> ConvexHull(List<Vec> points, IList<int> indexMap = null)
        {
            GiftWrapping<Arithmetic, Vec, Scalar> convex = new GiftWrapping<Arithmetic, Vec, Scalar>();
            convex.Initialize(points, indexMap);
            return convex.ConvexHull();
        }

        public List<int> ConvexHull()
        {
            Scalar minX = arithmetic.Get(polygon[0], 0);
            int first = 0;
            for (int i = 1; i < polygon.Count; ++i)
            {
                var p = arithmetic.Get(polygon[i], 0);
                int c = arithmetic.Compare(p, minX);
                if (c < 0)
                {
                    minX = p;
                    first = i;
                }
            }

            List<int> result = new List<int>();
            if (indexMap == null)
                result.Add(first);
            else
                result.Add(indexMap[first]);

            int previousStart = -1;
            int start = first;
            Vec currPoint = polygon[start];
            while (true)
            {
                int candidateId = -1;
                Vec candidatePoint = default;

                for (int i = 0; i < polygon.Count; ++i)
                {
                    if (i == start)
                        continue;

                    if(result.Count == 1)
                    {
                        candidateId = i;
                        candidatePoint = polygon[i];
                        break;
                    }
                    else if (arithmetic.Orient2D(currPoint, polygon[previousStart], polygon[i]) != 0) //This condition ensures that the colinearity handling in the next loop works
                    {
                        candidateId = i;
                        candidatePoint = polygon[i];
                        break;
                    }
                }
                if (candidateId < 0)
                    throw new Exception("Degenerate input - all points might lie on the same line");

                //List<int> collinear = new List<int>();
                for (int i = 0; i < polygon.Count; ++i)
                {
                    if (i == start || i == candidateId)
                        continue;

                    int sign = arithmetic.Orient2D(currPoint, candidatePoint, polygon[i]);
                    if (sign < 0)
                    {
                        candidateId = i;
                        candidatePoint = polygon[i];

                    }
                    else if (sign == 0)
                    {
                        //Keep the point that is closer to currPoint
                        if (IsCloserInSameDirection(currPoint, candidatePoint, polygon[i]))
                        {
                            candidateId = i;
                            candidatePoint = polygon[i];
                        }
                    }                    
                }
               
                if (candidateId == first)
                    break;

                previousStart = start;
                start = candidateId;
                currPoint = polygon[start];
                if (indexMap == null)
                    result.Add(start);
                else
                    result.Add(indexMap[start]);
            }

            return result;
        }

        private bool IsCloserInSameDirection(in Vec start, in Vec currentEnd, in Vec testPoint)
        {
            //Horizontal or not?
            bool deltaYIsZero = arithmetic.Compare(arithmetic.Get(start, 1), arithmetic.Get(currentEnd, 1)) == 0;
            int dirId = -1;
            if (deltaYIsZero)
            {
                dirId = 0;
                //int direction = arithmetic.Compare(arithmetic.Get(start, 0), arithmetic.Get(currentEnd, 0));
                //int compare = arithmetic.Compare(arithmetic.Get(currentEnd, 0), arithmetic.Get(testPoint, 0));
            }
            else
            {
                dirId = 1;
                //int yDirection = arithmetic.Compare(arithmetic.Get(start, 1), arithmetic.Get(currentEnd, 1));
            }

            int direction = arithmetic.Compare(arithmetic.Get(start, dirId), arithmetic.Get(currentEnd, dirId));
            if (direction == 0)
                throw new Exception("Puplicate points on convex hull?");

            int dir2 = arithmetic.Compare(arithmetic.Get(start, dirId), arithmetic.Get(testPoint, dirId));
            if (dir2 == 0)
                throw new Exception("Puplicate points on convex hull?");

            if (dir2 != direction)
                return false;          

            int cmp = arithmetic.Compare(arithmetic.Get(currentEnd, dirId), arithmetic.Get(testPoint, dirId));
            if(cmp==0)
                throw new Exception("Puplicate points on convex hull?");

            bool testPointIsFurtherAway = cmp == direction;

            return !testPointIsFurtherAway;
        }
    }
}
