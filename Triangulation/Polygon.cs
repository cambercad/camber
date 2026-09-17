namespace GeoCore
{
    public static class Polygon
    {
        public static bool IsPolygonCCW(List<Vec2D> polygon)
        {
            return PolygonOps<Vec2DArithmeticNoPredicates, Vec2D, double>.IsPolygonCCW(polygon);
        }

        public static PointInPolygonResult IsPointInPolygon(List<Vec2D> points, IList<int> polygon, Vec2D point)
        {
            return PolygonOps<Vec2DArithmeticNoPredicates, Vec2D, double>.IsPointInPolygon(points, polygon, point);
        }

        public static List<PolygonTree> BuildPolyTree(List<Vec2D> points, List<List<int>> polygons)
        {
            return PolygonOps<Vec2DArithmeticNoPredicates, Vec2D, double>.BuildPolyTree(points, polygons);
        }
    }

    public enum PointInPolygonResult
    {
        Inside,
        Outside,
        OnPolyBorder
    }

    public class PolygonTree
    {
        public int PolyId;
        public List<PolygonTree> Children; //A child polygon is fully contained inside its parent

        public PolygonTree(int poly)
        {
            PolyId = poly;
            Children = new List<PolygonTree>();
        }
    }


    public static class PolygonOps<Arithmetic, Vec, Scalar> where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
    {
        private static Arithmetic arithmetic = new Arithmetic();

        public static List<PolygonTree> BuildPolyTree(List<List<Vec>> polygons)
        {
            // Flatten all points into a single buffer and build index lists for each polygon
            List<Vec> flatPoints = new List<Vec>();
            List<List<int>> polys = new List<List<int>>();

            int index = 0;
            foreach (var poly in polygons)
            {
                List<int> indices = new List<int>();
                foreach (var pt in poly)
                {
                    flatPoints.Add(pt);
                    indices.Add(index++);
                }
                polys.Add(indices);
            }

            return PolygonOps<Arithmetic, Vec, Scalar>.BuildPolyTree(flatPoints, polys);
        }

        public static List<PolygonTree> BuildPolyTree(List<Vec> points,
            List<List<int>> polygons)
        {
            if (polygons.Count == 0) return new List<PolygonTree>();

            // Create polygon tree nodes for all polygons
            List<PolygonTree> trees = new List<PolygonTree>();
            for (int i = 0; i < polygons.Count; i++)
            {
                trees.Add(new PolygonTree(i));
            }

            // Build containment relationships
            for (int i = 0; i < trees.Count; i++)
            {
                for (int j = 0; j < trees.Count; j++)
                {
                    if (i == j) continue;

                    // Test if polygon j is inside polygon i (test only first point)
                    Vec testPoint = points[polygons[trees[j].PolyId][0]];
                    var result = IsPointInPolygon(points, polygons[trees[i].PolyId], testPoint);

                    if (result == PointInPolygonResult.Inside)
                    {
                        // Check if this is a direct containment (no intermediate parent)
                        bool isDirectChild = true;
                        for (int k = 0; k < trees.Count; k++)
                        {
                            if (k == i || k == j) continue;

                            var testResult1 = IsPointInPolygon(points, polygons[trees[k].PolyId], testPoint);
                            var testResult2 = IsPointInPolygon(points, polygons[trees[i].PolyId], points[polygons[trees[k].PolyId][0]]);

                            // If k contains j and i contains k, then i->j is not direct
                            if (testResult1 == PointInPolygonResult.Inside &&
                                testResult2 == PointInPolygonResult.Inside)
                            {
                                isDirectChild = false;
                                break;
                            }
                        }

                        if (isDirectChild)
                        {
                            trees[i].Children.Add(trees[j]);
                        }
                    }
                }
            }

            // Return only root polygons (those that are not children of any other polygon)
            List<PolygonTree> roots = new List<PolygonTree>();
            for (int i = 0; i < trees.Count; i++)
            {
                bool isRoot = true;
                for (int j = 0; j < trees.Count; j++)
                {
                    for (int k = 0; k < trees[j].Children.Count; k++)
                    {
                        if (trees[j].Children[k] == trees[i])
                        {
                            isRoot = false;
                            break;
                        }
                    }
                    if (!isRoot) break;
                }
                if (isRoot)
                {
                    roots.Add(trees[i]);
                }
            }

            for (int i = 0; i < roots.Count; ++i)
                OrientPolygonTreeRecursive(points, roots[i], polygons, true);

            return roots;
        }

        private static void OrientPolygonTreeRecursive(List<Vec> points, PolygonTree tree, List<List<int>> polygons, bool shouldBeCCW)
        {
            // Orient the current polygon
            var polygon = polygons[tree.PolyId];
            bool isCCW = IsPolygonCCW(points, polygon);

            if (isCCW != shouldBeCCW)
            {
                // Reverse the polygon orientation
                var reversedPolygon = new List<int>(polygon);
                reversedPolygon.Reverse();
                polygons[tree.PolyId] = reversedPolygon;
            }

            // Recursively orient children (holes should be CW, so opposite of parent)
            foreach (var child in tree.Children)
            {
                OrientPolygonTreeRecursive(points, child, polygons, !shouldBeCCW);
            }
        }


        public static bool IsPolygonCCW(List<Vec> polygon)
        {
            List<int> indices = new List<int>(polygon.Count);
            for (int i = 0; i < polygon.Count; ++i)
                indices.Add(i);

            return IsPolygonCCW(polygon, indices);
        }

        public static bool IsPolygonCCW(List<Vec> points, IList<int> polygon)
        {
            if (polygon == null || polygon.Count < 3)
                throw new ArgumentException("A polygon must have at least 3 vertices.");

            var reference = points[polygon[0]];
            BigRationalHybrid twiceArea = BigRationalHybrid.Zero;
            for (int i = 0; i < polygon.Count; ++i)
            {
                twiceArea += arithmetic.SignedArea2DTimesTwo(
                    points[polygon[i]],
                    points[polygon[(i + 1) % polygon.Count]],
                    reference);
            }

            int sign = BigRationalHybrid.Sign(twiceArea);
            if (sign == 0)
            {
                throw new InvalidOperationException(
                    $"Cannot orient degenerate polygon ({polygon.Count} vertices, zero signed area). " +
                    "Check for collinear or duplicate vertices in CSG cut or boundary trace.");
            }

            return sign > 0;
        }

        public static PointInPolygonResult IsPointInPolygon(List<Vec> points, IList<int> polygon, Vec point)
        {
            Scalar maxX = arithmetic.Get(points[polygon[0]], 0);
            Scalar maxY = arithmetic.Get(points[polygon[0]], 1);
            Scalar minX = arithmetic.Get(points[polygon[0]], 0);
            Scalar minY = arithmetic.Get(points[polygon[0]], 1);
            for (int i = 1; i < polygon.Count; ++i)
            {
                var p = arithmetic.Get(points[polygon[i]], 0);
                int c = arithmetic.Compare(p, maxX);
                if (c > 0)
                    maxX = p;
                c = arithmetic.Compare(p, minX);
                if (c < 0)
                    minX = p;

                p = arithmetic.Get(points[polygon[i]], 1);
                c = arithmetic.Compare(p, maxY);
                if (c > 0)
                    maxY = p;
                c = arithmetic.Compare(p, minY);
                if (c < 0)
                    minY = p;
            }

            {
                var pointX = arithmetic.Get(point, 0);
                int c = arithmetic.Compare(pointX, maxX);
                if (c > 0)
                    return PointInPolygonResult.Outside;
                c = arithmetic.Compare(pointX, minX);
                if (c < 0)
                    return PointInPolygonResult.Outside;

                var pointY = arithmetic.Get(point, 1);
                c = arithmetic.Compare(pointY, maxY);
                if (c > 0)
                    return PointInPolygonResult.Outside;
                c = arithmetic.Compare(pointY, minY);
                if (c < 0)
                    return PointInPolygonResult.Outside;
            }

            // Half-open horizontal crossings count a shared vertex once.
            // Exact orientation determines the crossing side without constructing
            // a finite ray endpoint or dividing rational coordinates.
            int winding = 0;
            var px = arithmetic.Get(point, 0);
            var py = arithmetic.Get(point, 1);
            for (int i = 0; i < polygon.Count; i++)
            {
                var a = points[polygon[i]];
                var b = points[polygon[(i + 1) % polygon.Count]];
                int ay = arithmetic.Compare(arithmetic.Get(a, 1), py);
                int by = arithmetic.Compare(arithmetic.Get(b, 1), py);
                if ((ay < 0 && by < 0) || (ay > 0 && by > 0))
                    continue;
                int side = arithmetic.Orient2D(a, b, point);
                if (side == 0)
                {
                    var ax = arithmetic.Get(a, 0);
                    var bx = arithmetic.Get(b, 0);
                    if (arithmetic.Compare(px, arithmetic.Min(ax, bx)) >= 0 &&
                        arithmetic.Compare(px, arithmetic.Max(ax, bx)) <= 0)
                        return PointInPolygonResult.OnPolyBorder;
                }
                if (ay <= 0 && by > 0 && side > 0)
                    winding++;
                else if (ay > 0 && by <= 0 && side < 0)
                    winding--;
            }
            return winding == 0 ? PointInPolygonResult.Outside : PointInPolygonResult.Inside;
        }
    }
}
