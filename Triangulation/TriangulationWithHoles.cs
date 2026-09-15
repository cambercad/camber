namespace GeoCore
{
    public class TriangulationWithHoles<Arithmetic, Vec, Scalar> where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
    {
        // Polygons are not allowed to intersect with each other (also no self intersections)
        // Figures out by itself which polygons are borders and which are holes
        public static List<Tri> Triangulate(List<Vec> points, List<List<int>> polygons)
        {
            List<PolygonTree> roots = PolygonOps<Arithmetic, Vec, Scalar>.BuildPolyTree(points, polygons);

            // Now triangulate each root polygon tree
            List<Tri> allTriangles = new List<Tri>();
            for(int i=0;i<roots.Count;++i)
            {
                var root = roots[i];
                var triangles = TriangulatePolygonTree(points, root, polygons);
                allTriangles.AddRange(triangles);
            }

            return allTriangles;
        }

        public static List<Tri> Triangulate(List<List<Vec>> polygons)
        {
            // Flatten all points into a single buffer and build index lists for each polygon
            List<Vec> flatBuffer = new List<Vec>();
            List<List<int>> polygonIndices = new List<List<int>>();

            int index = 0;
            foreach (var poly in polygons)
            {
                List<int> indices = new List<int>();
                foreach (var pt in poly)
                {
                    flatBuffer.Add(pt);
                    indices.Add(index++);
                }
                polygonIndices.Add(indices);
            }

            return Triangulate(flatBuffer, polygonIndices);
        }

        public static List<Tri> TriangulatePolygonTree(List<Vec> points, PolygonTree tree, List<List<int>> polygons)
        {
            // Extract holes from direct children
            List<List<int>> holes = new List<List<int>>();
            foreach (var child in tree.Children)
            {
                holes.Add(polygons[child.PolyId]);
            }

            // Triangulate the root polygon with its direct children as holes
            List<Tri> triangles = Triangulate(points, polygons[tree.PolyId], holes);

            // Recursively triangulate all child polygon trees
            foreach (var child in tree.Children)
            {
                foreach(var c in child.Children)
                {
                    var childTriangles = TriangulatePolygonTree(points, c, polygons);
                    triangles.AddRange(childTriangles);
                }
            }

            return triangles;
        }

        public static List<Tri> Triangulate(List<Vec> points, List<int> borderPolygon, List<List<int>> holes)
        {
            TriangulationEarClipping<Arithmetic, Vec, Scalar> triangulator = new TriangulationEarClipping<Arithmetic, Vec, Scalar>();
            triangulator.Initialize(points, borderPolygon, holes);

            List<Tri> result = new List<Tri>();
            triangulator.Triangulate(result);
            return result;
        }
    }
}
