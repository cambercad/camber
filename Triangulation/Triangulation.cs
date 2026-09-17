using Geo;

namespace GeoCore
{
    public static class Triangulator
    {
        public static List<Tri> TriangulatePolygon(List<Int2> points, IList<int> borderPolygon = null, bool delaunayPostProcess = false)
        {
            TriangulationEarClipping<Int2Arithmetic, Int2, int> poly = new TriangulationEarClipping<Int2Arithmetic, Int2, int>();

            poly.Initialize(points, borderPolygon);

            List<Tri> triangles = new List<Tri>();
            poly.Triangulate(triangles);

            if (delaunayPostProcess)
                DelaunayOptimizer<Int2CircleArithmetic, Int2>.Optimize(points, triangles);

            return triangles;
        }

        public static List<Tri> TriangulatePolygon(List<Rat2Hybrid> points, IList<int> borderPolygon = null, bool delaunayPostProcess = false)
        {
            TriangulationEarClipping<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid> poly = new TriangulationEarClipping<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>();

            poly.Initialize(points, borderPolygon);

            List<Tri> triangles = new List<Tri>();
            poly.Triangulate(triangles);

            if (delaunayPostProcess)
                DelaunayOptimizer<Rat2HybridCircleArithmetic, Rat2Hybrid>.Optimize(points, triangles);

            return triangles;
        }

        public static List<Tri> TriangulatePolygon(List<Vec2D> points, IList<int> borderPolygon = null, bool delaunayPostProcess = false)
        {
            TriangulationEarClipping<Vec2DArithmeticNoPredicates, Vec2D, double> poly = new TriangulationEarClipping<Vec2DArithmeticNoPredicates, Vec2D, double>();

            poly.Initialize(points, borderPolygon);

            List<Tri> triangles = new List<Tri>();
            poly.Triangulate(triangles);

            if (delaunayPostProcess)
                DelaunayOptimizer<Vec2DCircleArithmeticNoPredicates, Vec2D>.Optimize(points, triangles);

            return triangles;
        }

        public static List<Tri> TriangulatePolygon(List<Vec2D> points, List<List<int>> borderAndHolePolygons, bool delaunayPostProcess = false)
        {
            var triangles = TriangulationWithHoles<Vec2DArithmeticNoPredicates, Vec2D, double>.Triangulate(points, borderAndHolePolygons);

            if (delaunayPostProcess)
                DelaunayOptimizer<Vec2DCircleArithmeticNoPredicates, Vec2D>.Optimize(points, triangles);

            return triangles;
        }
        public static List<Tri> TriangulatePolygon(List<Int2> points, List<List<int>> borderAndHolePolygons, bool delaunayPostProcess = false)
        {
            var triangles = TriangulationWithHoles<Int2Arithmetic, Int2, int>.Triangulate(points, borderAndHolePolygons);

            if (delaunayPostProcess)
                DelaunayOptimizer<Int2CircleArithmetic, Int2>.Optimize(points, triangles);

            return triangles;
        }

        public static List<Tri> TriangulatePolygon(List<Rat2Hybrid> points, List<List<int>> borderAndHolePolygons, bool delaunayPostProcess = false)
        {
            var triangles = TriangulationWithHoles<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>.Triangulate(points, borderAndHolePolygons);

            if (delaunayPostProcess)
                DelaunayOptimizer<Rat2HybridCircleArithmetic, Rat2Hybrid>.Optimize(points, triangles);

            return triangles;
        }

        public static List<Tri> TriangulatePolygon(List<List<Vec2D>> borderAndHolePolygons, bool delaunayPostProcess = false)
        {
            var triangles = TriangulationWithHoles<Vec2DArithmeticNoPredicates, Vec2D, double>.Triangulate(borderAndHolePolygons);

            if (delaunayPostProcess)
                DelaunayOptimizer<Vec2DCircleArithmeticNoPredicates, Vec2D>.Optimize(borderAndHolePolygons, triangles);

            return triangles;
        }
        public static List<Tri> TriangulatePolygon(List<List<Int2>> borderAndHolePolygons, bool delaunayPostProcess = false)
        {
            var triangles = TriangulationWithHoles<Int2Arithmetic, Int2, int>.Triangulate(borderAndHolePolygons);

            if (delaunayPostProcess)
                DelaunayOptimizer<Int2CircleArithmetic, Int2>.Optimize(borderAndHolePolygons, triangles);

            return triangles;
        }
        public static List<Tri> TriangulatePolygon(List<List<Rat2Hybrid>> borderAndHolePolygons, bool delaunayPostProcess = false)
        {
            var triangles = TriangulationWithHoles<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>.Triangulate(borderAndHolePolygons);

            if (delaunayPostProcess)
                DelaunayOptimizer<Rat2HybridCircleArithmetic, Rat2Hybrid>.Optimize(borderAndHolePolygons, triangles);

            return triangles;
        }

        public static List<Tri> TriangulatePolygon(List<Int2> points, List<int> borderPolygon, List<Int2> constraints, List<int> pointsToInsert = null, bool delaunayPostProcess = false)
        {
            var ctx = TriangulationWorkspaces.Int2Constrained.Value;
            ctx.BeginSession(points);
            var segments = TriangulateConstrained<Int2Arithmetic, Int2, int>.TriangulateWithConstraints(ctx, borderPolygon, constraints, pointsToInsert);

            if (delaunayPostProcess)
                DelaunayOptimizer<Int2CircleArithmetic, Int2>.Optimize(points, ctx.Triangles, segments);

            return new List<Tri>(ctx.Triangles);
        }

        public static List<Tri> TriangulatePolygon(List<Vec2D> points, List<int> borderPolygon, List<Int2> constraints, List<int> pointsToInsert = null, bool delaunayPostProcess = false)
        {
            var ctx = TriangulationWorkspaces.Vec2DConstrained.Value;
            ctx.BeginSession(points);
            var segments = TriangulateConstrained<Vec2DArithmeticNoPredicates, Vec2D, double>.TriangulateWithConstraints(ctx, borderPolygon, constraints, pointsToInsert);

            if (delaunayPostProcess)
                DelaunayOptimizer<Vec2DCircleArithmeticNoPredicates, Vec2D>.Optimize(points, ctx.Triangles, segments);

            return new List<Tri>(ctx.Triangles);
        }

        public static List<Tri> TriangulatePolygon(List<Rat2Hybrid> points, List<int> borderPolygon, List<Int2> constraints, List<int> pointsToInsert = null, bool delaunayPostProcess = false)
        {
            var ctx = TriangulationWorkspaces.Rat2HybridConstrained.Value;
            ctx.BeginSession(points);
            var segments = TriangulateConstrained<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>.TriangulateWithConstraints(ctx, borderPolygon, constraints, pointsToInsert);

            if (delaunayPostProcess)
                DelaunayOptimizer<Rat2HybridCircleArithmetic, Rat2Hybrid>.Optimize(points, ctx.Triangles, segments);

            return new List<Tri>(ctx.Triangles);
        }
    }

    internal static class TriangulationWorkspaces
    {
        // CSG Resolver triangulates coplanar patches in Parallel.For — one workspace per thread.
        internal static readonly ThreadLocal<TriangulationContext<Int2Arithmetic, Int2, int>> Int2Constrained =
            new(() => new TriangulationContext<Int2Arithmetic, Int2, int>());

        internal static readonly ThreadLocal<TriangulationContext<Vec2DArithmeticNoPredicates, Vec2D, double>> Vec2DConstrained =
            new(() => new TriangulationContext<Vec2DArithmeticNoPredicates, Vec2D, double>());

        internal static readonly ThreadLocal<TriangulationContext<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>> Rat2HybridConstrained =
            new(() => new TriangulationContext<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>());
    }
}
