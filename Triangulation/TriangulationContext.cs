using System.Collections.Generic;

namespace GeoCore
{
    /// <summary>
    /// Reusable triangulation buffers. Point bounds are built once per session and never change.
    /// Triangle AABBs for culling are merged from cached point bounds on demand (no parallel triangle cache).
    /// </summary>
    public class TriangulationContext<Arithmetic, Vec, Scalar> where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
    {
        private readonly Arithmetic arithmetic = new Arithmetic();

        private List<Vec> points;
        private readonly List<Box2D> pointBounds = new List<Box2D>();
        private readonly List<Tri> triangles = new List<Tri>();
        private readonly Dictionary<(int, int, int), int> orient2DCache = new Dictionary<(int, int, int), int>();

        internal List<Vec> Points => points;
        internal List<Box2D> PointBounds => pointBounds;
        internal List<Tri> Triangles => triangles;
        internal Arithmetic GetArithmetic() => arithmetic;

        public void BeginSession(List<Vec> sessionPoints)
        {
            points = sessionPoints;
            triangles.Clear();
            orient2DCache.Clear();

            int count = sessionPoints.Count;
            if (pointBounds.Count < count)
            {
                int grow = count - pointBounds.Count;
                for (int i = 0; i < grow; ++i)
                    pointBounds.Add(default);
            }
            else if (pointBounds.Count > count)
                pointBounds.RemoveRange(count, pointBounds.Count - count);

            for (int i = 0; i < count; ++i)
                pointBounds[i] = arithmetic.GetBounds(sessionPoints[i]);
        }

        public Box2D BoundsForTri(Tri t)
        {
            var box = pointBounds[t.A];
            box.Extend(pointBounds[t.B]);
            box.Extend(pointBounds[t.C]);
            return box;
        }

        public int Orient2D(int a, int b, int c)
        {
            int sign = 1;
            // Exact rational orientation changes sign on each swap. Share one
            // cache entry across all six permutations of the same three points.
            // Floating-point predicates retain their original evaluation order.
            if (typeof(Arithmetic) == typeof(Rat2HybridArithmetic))
            {
                if (a == b || b == c || a == c) return 0;
                if (a > b) { (a, b) = (b, a); sign = -sign; }
                if (b > c) { (b, c) = (c, b); sign = -sign; }
                if (a > b) { (a, b) = (b, a); sign = -sign; }
            }
            var key = (a, b, c);
            if (!orient2DCache.TryGetValue(key, out int result))
            {
                result = arithmetic.Orient2D(points[a], points[b], points[c]);
                orient2DCache.Add(key, result);
            }
            return sign * result;
        }

        public void SetTriangle(int i, Tri t) => triangles[i] = t;

        public void AddTriangle(Tri t) => triangles.Add(t);

        public void RemoveTriangleAt(int i)
        {
            int last = triangles.Count - 1;
            if (i != last)
                triangles[i] = triangles[last];
            triangles.RemoveAt(last);
        }
    }
}
