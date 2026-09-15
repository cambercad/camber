using GeoCore;

namespace Triangulation
{
    public class TriangulationDebug
    {
        public static void WriteObj<Vec>(string filePath, List<Tri> removedTriangles, List<Vec> points)
        {
            using (var writer = new System.IO.StreamWriter(filePath))
            {
                // Write vertices
                for (int i = 0; i < points.Count; i++)
                {
                    var p = points[i];
                    writer.WriteLine($"v {p} 0");
                }

                // Write faces (triangles)
                foreach (var tri in removedTriangles)
                {
                    // OBJ file vertex indices start at 1
                    int a = tri.A + 1;
                    int b = tri.B + 1;
                    int c = tri.C + 1;
                    writer.WriteLine($"f {a} {b} {c}");
                }
            }
        }

        /// <summary>
        /// Writes vertices and each triangle edge as a separate <c>l i j</c> line (OBJ 1-based indices).
        /// Shared edges appear twice; same vertex transform as <see cref="WriteLineObj(string, List{Rat2Hybrid}, List{int}, List{Int2}, bool)"/> with <c>transformIntoUnitSquare == true</c>.
        /// </summary>
        public static void WriteLineObj(string fileName, List<Rat2Hybrid> bigRationalPoints, List<Tri> tris)
        {
            using var writer = new System.IO.StreamWriter(fileName);
            WriteLineObjVertices(writer, bigRationalPoints, transformIntoUnitSquare: true);
            if (tris == null)
                return;
            foreach (var tri in tris)
            {
                int a = tri.A + 1;
                int b = tri.B + 1;
                int c = tri.C + 1;
                writer.WriteLine($"l {a} {b}");
                writer.WriteLine($"l {b} {c}");
                writer.WriteLine($"l {c} {a}");
            }
        }

        /// <summary>
        /// Exports the border polygon (as a closed loop) and constraints as .obj file lines.
        /// Border polygon is exported as a closed polyline (with 'l' statement).
        /// Constraints are exported as lines ('l' statement).
        /// Vertices are exported at the top as 'v x y 0'.
        /// </summary>
        /// <param name="transformIntoUnitSquare">
        /// If true, vertex coordinates are scaled and translated with independent X/Y scale so the axis-aligned
        /// bounding box of <paramref name="bigRationalPoints"/> maps exactly to [0,1]×[0,1] (fills the square;
        /// aspect ratio may change).
        /// </param>
        public static void WriteLineObj(string fileName, List<Rat2Hybrid> bigRationalPoints, List<int> borderPolygon, List<Int2> constraints, bool transformIntoUnitSquare = true)
        {
            using (var writer = new System.IO.StreamWriter(fileName))
            {
                WriteLineObjVertices(writer, bigRationalPoints, transformIntoUnitSquare);

                // Write border polygon as a closed loop
                if (borderPolygon != null && borderPolygon.Count > 0)
                {
                    for (int i = 0; i < borderPolygon.Count; ++i)
                    {
                        var s = borderPolygon[i] + 1;
                        var e = borderPolygon[(i + 1) % borderPolygon.Count] + 1;
                        writer.WriteLine($"l {s} {e}");
                    }
                }

                // Write constraints as lines
                if (constraints != null)
                {
                    foreach (var c in constraints)
                    {
                        // .obj is 1-based, Int2.X and Int2.Y are 0-based
                        int v1 = c.X + 1;
                        int v2 = c.Y + 1;
                        writer.WriteLine($"l {v1} {v2}");
                    }
                }
            }
        }

        /// <summary>
        /// Same as <see cref="WriteLineObj(string, List{Rat2Hybrid}, List{int}, List{Int2}, bool)"/>, but writes multiple closed border loops.
        /// </summary>
        /// <param name="transformIntoUnitSquare">See the single-polygon overload.</param>
        public static void WriteLineObj(string fileName, List<Rat2Hybrid> bigRationalPoints, List<List<int>> borderPolygon, List<Int2> constraints, bool transformIntoUnitSquare = true)
        {
            using (var writer = new System.IO.StreamWriter(fileName))
            {
                WriteLineObjVertices(writer, bigRationalPoints, transformIntoUnitSquare);

                // Write border polygon as a closed loop
                if (borderPolygon != null && borderPolygon.Count > 0)
                {
                    foreach (var poly in borderPolygon)
                    {
                        for (int i = 0; i < poly.Count; ++i)
                        {
                            var s = poly[i] + 1;
                            var e = poly[(i + 1) % poly.Count] + 1;
                            writer.WriteLine($"l {s} {e}");
                        }
                    }
                }

                // Write constraints as lines
                if (constraints != null)
                {
                    foreach (var c in constraints)
                    {
                        // .obj is 1-based, Int2.X and Int2.Y are 0-based
                        int v1 = c.X + 1;
                        int v2 = c.Y + 1;
                        writer.WriteLine($"l {v1} {v2}");
                    }
                }
            }
        }

        private static void WriteLineObjVertices(System.IO.StreamWriter writer, List<Rat2Hybrid> bigRationalPoints, bool transformIntoUnitSquare)
        {
            if (bigRationalPoints == null)
                return;

            double minX = 0, minY = 0, maxX = 0, maxY = 0;
            if (transformIntoUnitSquare && bigRationalPoints.Count > 0)
                ComputeBoundingBoxRat2(bigRationalPoints, out minX, out minY, out maxX, out maxY);

            for (int i = 0; i < bigRationalPoints.Count; ++i)
            {
                var p = bigRationalPoints[i];
                double x = p.X.ToDouble();
                double y = p.Y.ToDouble();
                if (transformIntoUnitSquare)
                    ProjectToUnitSquare(x, y, minX, minY, maxX, maxY, out x, out y);
                writer.WriteLine($"v {x} {y} 0");
            }
        }

        private static void ComputeBoundingBoxRat2(List<Rat2Hybrid> points, out double minX, out double minY, out double maxX, out double maxY)
        {
            minX = maxX = points[0].X.ToDouble();
            minY = maxY = points[0].Y.ToDouble();
            for (int i = 1; i < points.Count; ++i)
            {
                var p = points[i];
                double x = p.X.ToDouble();
                double y = p.Y.ToDouble();
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
        }

        /// <summary>
        /// Non-uniform scale: bbox maps to [0,1]×[0,1] (min→0, max→1 on each axis) so the hull fills the square.
        /// Degenerate bbox (point or line) uses 0.5 on axes with zero extent.
        /// </summary>
        private static void ProjectToUnitSquare(double x, double y, double minX, double minY, double maxX, double maxY, out double ox, out double oy)
        {
            double dw = maxX - minX;
            double dh = maxY - minY;
            if (dw <= 0 && dh <= 0)
            {
                ox = oy = 0.5;
                return;
            }
            if (dw <= 0)
                ox = 0.5;
            else
                ox = (x - minX) / dw;
            if (dh <= 0)
                oy = 0.5;
            else
                oy = (y - minY) / dh;
        }


        public static bool PointExistsInTriangles(List<Tri> triangles, int id)
        {
            for (int i = 0; i < triangles.Count; ++i)
            {
                var tri = triangles[i];
                if (tri.Contains(id))
                    return true;
            }
            return false;
        }
        public static bool AllPointsExistsInTriangles(List<Tri> triangles, IEnumerable<int> set)
        {
            foreach (var v in set)
                if (!PointExistsInTriangles(triangles, v))
                    return false;

            return true;
        }

        public static void CheckForDuplicatePoints<Vec>(List<Vec> points)
        {
            HashSet<Vec> tmp = new HashSet<Vec>();
            for (int i = 0; i < points.Count; ++i)
                if (!tmp.Add(points[i]))
                    throw new Exception("Duplicate points detected");

            for (int i = 0; i < points.Count; ++i)
                for (int j = i + 1; j < points.Count; ++j)
                    if (points[i].ToString() == points[j].ToString())
                    {
                        bool bb = points[i].Equals(points[j]);
                        throw new Exception();
                    }
        }

        public static BigRationalHybrid ComputePolygonAreaTimesTwo<Arithmetic, Vec, Scalar>(Arithmetic arithmetic, List<Vec> points,List<int> polygon) where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
        {
            BigRationalHybrid sum = BigRationalHybrid.Zero;
            var reference = points[0];
            for (int i = 0; i < polygon.Count; ++i)
            {
                sum += arithmetic.SignedArea2DTimesTwo(points[polygon[i]],
                    points[polygon[(i + 1) % polygon.Count]], reference);
            }
            return sum;
        }
        public static BigRationalHybrid ComputePolygonAreaTimesTwo<Arithmetic, Vec, Scalar>(Arithmetic arithmetic, List<Vec> points,List<List<int>> polygon) where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
        {
            BigRationalHybrid sum = BigRationalHybrid.Zero;
            for (int i = 0; i < polygon.Count; ++i)
                sum += ComputePolygonAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, polygon[i]);
            return sum;
        }


        public static BigRationalHybrid ComputeAreaTimesTwo<Arithmetic, Vec, Scalar>(Arithmetic arithmetic, List<Vec> points, List<Tri> triangles) where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
        {
            BigRationalHybrid sum = BigRationalHybrid.Zero;
            int areaSign = 0;
            for (int i = 0; i < triangles.Count; ++i)
            {
                var tri = triangles[i];
                var area = arithmetic.SignedArea2DTimesTwo(points[tri.A], points[tri.B], points[tri.C]);
                var s = BigRationalHybrid.Sign(area);
                if (s == 0)
                    throw new Exception();
                if (i == 0)
                    areaSign = s;
                else if (areaSign != s)
                    throw new Exception();

                sum += area;
            }
            return sum;
        }

        private bool CheckAllPointsExit(IEnumerable<int> pts, List<Tri> triangles, bool throwOnError = true)
        {
            foreach (var v in pts)
            {
                bool success = false;
                for (int i = triangles.Count - 1; i >= 0; --i)
                {
                    if (triangles[i].Contains(v))
                    {
                        success = true;
                        break;
                    }
                }
                if (!success)
                {
                    if (throwOnError)
                        throw new Exception();
                    return false;
                }
            }
            return true;
        }
    }

    public class TriangulationDebug<Arithmetic, Vec, Scalar> where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
    {
        private List<Vec> points;
        private List<Tri> triangles = new List<Tri>();
        private Arithmetic arithmetic = new Arithmetic();

        public TriangulationDebug(List<Vec> points, List<Tri> triangles, Arithmetic arithmetic)
        {
            this.points = points;
            this.triangles = triangles;
            this.arithmetic = arithmetic;
        }



        public void ValidateInput(IList<int> borderPolygon, IList<Int2> constraints)
        {
            // Validate that the border polygon has no self-intersections
            ValidateBorderPolygonNoSelfIntersections(borderPolygon);

            // Validate that all constraint endpoints are inside or on the border polygon
            for (int i = 0; i < constraints.Count; ++i)
            {
                var c = constraints[i];
                ValidateConstraintEndpointInPolygon(borderPolygon, c.X, i);
                ValidateConstraintEndpointInPolygon(borderPolygon, c.Y, i);
            }

            // Validate that constraint segments don't cross border edges
            for (int i = 0; i < constraints.Count; ++i)
            {
                ValidateConstraintDoesNotCrossBorder(borderPolygon, constraints[i], i);
            }

            // Validate that constraints don't intersect illegally
            for (int i = 0; i < constraints.Count; ++i)
            {
                for (int j = i + 1; j < constraints.Count; ++j)
                {
                    ValidateConstraintPair(constraints[i], constraints[j], i, j);
                }
            }
        }

        private void ValidateBorderPolygonNoSelfIntersections(IList<int> borderPolygon)
        {
            if (borderPolygon.Count < 3)
                throw new Exception("Border polygon must have at least 3 vertices");

            // Check all pairs of edges for intersection
            for (int i = 0; i < borderPolygon.Count; ++i)
            {
                int nextI = (i + 1) % borderPolygon.Count;

                for (int j = i + 1; j < borderPolygon.Count; ++j)
                {
                    int nextJ = (j + 1) % borderPolygon.Count;

                    // Skip adjacent edges (they share a vertex by definition)
                    if (nextI == j || i == nextJ)
                        continue;

                    var edge1 = new Int2(borderPolygon[i], borderPolygon[nextI]);
                    var edge2 = new Int2(borderPolygon[j], borderPolygon[nextJ]);

                    if (BorderEdgesIntersect(edge1, edge2))
                        throw new Exception($"Border polygon has self-intersection: edge ({borderPolygon[i]}, {borderPolygon[nextI]}) intersects edge ({borderPolygon[j]}, {borderPolygon[nextJ]})");
                }
            }
        }

        private bool BorderEdgesIntersect(Int2 edge1, Int2 edge2)
        {
            // Edges should not share both endpoints (that would make them identical)
            if (edge1.X == edge2.X && edge1.Y == edge2.Y)
                return false;
            if (edge1.X == edge2.Y && edge1.Y == edge2.X)
                return false;

            var p1 = points[edge1.X];
            var p2 = points[edge1.Y];
            var p3 = points[edge2.X];
            var p4 = points[edge2.Y];

            // Check orientation of p3 and p4 with respect to line edge1
            var o1 = arithmetic.Orient2D(p3, p1, p2);
            var o2 = arithmetic.Orient2D(p4, p1, p2);

            // Check orientation of p1 and p2 with respect to line edge2
            var o3 = arithmetic.Orient2D(p1, p3, p4);
            var o4 = arithmetic.Orient2D(p2, p3, p4);

            // If all orientations are zero, segments are collinear
            if (o1 == 0 && o2 == 0 && o3 == 0 && o4 == 0)
            {
                // Collinear edges in a border polygon that overlap (not just touch) is a self-intersection
                return CollinearBorderEdgesOverlap(edge1, edge2);
            }

            // For border polygon edges, we need to check if they intersect at all
            // Two cases of intersection:
            // 1. Proper intersection (segments cross each other)
            // 2. One endpoint lies on the other segment (T-junction)

            // Check if endpoints of edge2 lie on edge1 (excluding edge1's endpoints)
            if (o1 == 0 && PointBetweenEndpoints(p3, p1, p2))
                return true;
            if (o2 == 0 && PointBetweenEndpoints(p4, p1, p2))
                return true;

            // Check if endpoints of edge1 lie on edge2 (excluding edge2's endpoints)
            if (o3 == 0 && PointBetweenEndpoints(p1, p3, p4))
                return true;
            if (o4 == 0 && PointBetweenEndpoints(p2, p3, p4))
                return true;

            // Check for proper intersection (segments cross)
            return Math.Sign(o1) != Math.Sign(o2) && Math.Sign(o3) != Math.Sign(o4);
        }

        private bool PointBetweenEndpoints(Vec point, Vec lineStart, Vec lineEnd)
        {
            // Assumes point is collinear with line
            // Check if point is strictly between lineStart and lineEnd
            int coordId;
            if (arithmetic.Compare(arithmetic.Get(lineStart, 0), arithmetic.Get(lineEnd, 0)) == 0)
                coordId = 1; // Use Y if X coordinates are identical
            else
                coordId = 0; // Use X otherwise

            var pCoord = arithmetic.Get(point, coordId);
            var startCoord = arithmetic.Get(lineStart, coordId);
            var endCoord = arithmetic.Get(lineEnd, coordId);

            var minCoord = arithmetic.Min(startCoord, endCoord);
            var maxCoord = arithmetic.Max(startCoord, endCoord);

            // Point must be strictly between (not at) the endpoints
            return arithmetic.Compare(pCoord, minCoord) > 0 && arithmetic.Compare(pCoord, maxCoord) < 0;
        }

        private bool CollinearBorderEdgesOverlap(Int2 edge1, Int2 edge2)
        {
            // For border polygon edges, any overlap beyond just touching at endpoints is invalid
            var p1 = points[edge1.X];
            var p2 = points[edge1.Y];
            var p3 = points[edge2.X];
            var p4 = points[edge2.Y];

            // Choose the principal direction
            int coordId;
            if (arithmetic.Compare(arithmetic.Get(p1, 0), arithmetic.Get(p2, 0)) == 0)
                coordId = 1; // Use Y if X coordinates are identical
            else
                coordId = 0; // Use X otherwise

            var min1 = arithmetic.Min(arithmetic.Get(p1, coordId), arithmetic.Get(p2, coordId));
            var max1 = arithmetic.Max(arithmetic.Get(p1, coordId), arithmetic.Get(p2, coordId));
            var min2 = arithmetic.Min(arithmetic.Get(p3, coordId), arithmetic.Get(p4, coordId));
            var max2 = arithmetic.Max(arithmetic.Get(p3, coordId), arithmetic.Get(p4, coordId));

            // Check if intervals overlap (not just touch)
            return TriangulationHelper<Arithmetic, Vec, Scalar>.IntervalOverlapNoTouch(arithmetic, min1, max1, min2, max2);
        }

        private void ValidateConstraintEndpointInPolygon(IList<int> borderPolygon, int pointId, int constraintId)
        {
            // Check if the point is on the border
            bool isOnBorder = false;
            for (int i = 0; i < borderPolygon.Count; ++i)
            {
                if (borderPolygon[i] == pointId)
                {
                    isOnBorder = true;
                    break;
                }
            }

            if (isOnBorder)
                return; // Points on border are allowed

            // Check if the point is strictly inside the polygon using PolygonOps
            var result = PolygonOps<Arithmetic, Vec, Scalar>.IsPointInPolygon(points, borderPolygon, points[pointId]);

            if (result == PointInPolygonResult.OnPolyBorder)
                return; // Points exactly on the border are allowed

            if (result != PointInPolygonResult.Inside)
                throw new Exception($"Constraint {constraintId} has an endpoint (point {pointId}) outside the border polygon");
        }

        private void ValidateConstraintDoesNotCrossBorder(IList<int> borderPolygon, Int2 constraint, int constraintId)
        {
            // Check that the constraint segment doesn't cross any border edges
            for (int i = 0; i < borderPolygon.Count; ++i)
            {
                int nextI = (i + 1) % borderPolygon.Count;
                var borderEdge = new Int2(borderPolygon[i], borderPolygon[nextI]);

                // Check if constraint crosses this border edge
                if (ConstraintCrossesBorderEdge(constraint, borderEdge))
                    throw new Exception($"Constraint {constraintId} (points {constraint.X}, {constraint.Y}) crosses border edge (points {borderPolygon[i]}, {borderPolygon[nextI]})");
            }
        }

        private bool ConstraintCrossesBorderEdge(Int2 constraint, Int2 borderEdge)
        {
            var p1 = points[constraint.X];
            var p2 = points[constraint.Y];
            var p3 = points[borderEdge.X];
            var p4 = points[borderEdge.Y];

            // Check orientation of border edge endpoints with respect to constraint line
            var o1 = arithmetic.Orient2D(p3, p1, p2);
            var o2 = arithmetic.Orient2D(p4, p1, p2);

            // Check orientation of constraint endpoints with respect to border edge line
            var o3 = arithmetic.Orient2D(p1, p3, p4);
            var o4 = arithmetic.Orient2D(p2, p3, p4);

            // If all orientations are zero, segments are collinear
            if (o1 == 0 && o2 == 0 && o3 == 0 && o4 == 0)
            {
                // Collinear case: constraint lies on the same line as border edge
                // This is VALID if:
                // 1. Constraint is fully contained within the border edge (constraint endpoints on or inside border edge)
                // 2. They only touch at endpoints
                // This is INVALID if:
                // - Constraint extends beyond the border edge in either direction

                // Check if both constraint endpoints lie within or at the border edge endpoints
                if (ConstraintFullyContainedInBorderEdge(constraint, borderEdge))
                    return false; // Valid: constraint lies on border

                // Otherwise, they overlap but constraint extends beyond border edge
                return true;
            }

            // If constraint and border edge share an endpoint, they don't cross (they touch)
            if (constraint.Contains(borderEdge.X) || constraint.Contains(borderEdge.Y))
                return false;

            // Segments cross if endpoints are on opposite sides of each other's lines
            var ret = /*Math.Sign(o1) != Math.Sign(o2) &&*/ o3 != 0 && o4 != 0 && Math.Sign(o3) != Math.Sign(o4);

            if (ret)
            {

            }

            return ret;
        }

        private bool ConstraintFullyContainedInBorderEdge(Int2 constraint, Int2 borderEdge)
        {
            // Check if both constraint endpoints lie on the border edge segment
            var p1 = points[constraint.X];
            var p2 = points[constraint.Y];
            var p3 = points[borderEdge.X];
            var p4 = points[borderEdge.Y];

            // Choose the principal direction
            int coordId;
            if (arithmetic.Compare(arithmetic.Get(p3, 0), arithmetic.Get(p4, 0)) == 0)
                coordId = 1; // Use Y if X coordinates of border edge are identical
            else
                coordId = 0; // Use X otherwise

            var minBorder = arithmetic.Min(arithmetic.Get(p3, coordId), arithmetic.Get(p4, coordId));
            var maxBorder = arithmetic.Max(arithmetic.Get(p3, coordId), arithmetic.Get(p4, coordId));
            var c1Coord = arithmetic.Get(p1, coordId);
            var c2Coord = arithmetic.Get(p2, coordId);

            // Both constraint endpoints must be within [min, max] of border edge
            bool c1Inside = arithmetic.Compare(c1Coord, minBorder) >= 0 && arithmetic.Compare(c1Coord, maxBorder) <= 0;
            bool c2Inside = arithmetic.Compare(c2Coord, minBorder) >= 0 && arithmetic.Compare(c2Coord, maxBorder) <= 0;

            return c1Inside && c2Inside;
        }

        private void ValidateConstraintPair(Int2 c1, Int2 c2, int id1, int id2)
        {
            // Constraints are allowed to share endpoints
            if (c1.Contains(c2.X) && c1.Contains(c2.Y))
                return; // Same constraint (shouldn't happen but handle gracefully)

            if (c1.Contains(c2.X) || c1.Contains(c2.Y))
                return; // Constraints share one endpoint - this is allowed

            // Check if c2 endpoints lie on c1 line
            ValidatePointNotOnConstraintLine(c1, c2.X, id1, id2, "X");
            ValidatePointNotOnConstraintLine(c1, c2.Y, id1, id2, "Y");

            // Check if c1 endpoints lie on c2 line
            ValidatePointNotOnConstraintLine(c2, c1.X, id2, id1, "X");
            ValidatePointNotOnConstraintLine(c2, c1.Y, id2, id1, "Y");

            // Check if constraints intersect (not at endpoints)
            if (ConstraintsIntersect(c1, c2))
                throw new Exception($"Constraints {id1} and {id2} intersect");
        }

        private void ValidatePointNotOnConstraintLine(Int2 constraint, int pointId, int constraintId, int otherConstraintId, string endpointName)
        {
            // If the point is an endpoint of the constraint, it's allowed
            if (constraint.Contains(pointId))
                return;

            var p = points[pointId];
            var a = points[constraint.X];
            var b = points[constraint.Y];

            // Use Orient2D to check if point is collinear with the constraint line
            var orient = arithmetic.Orient2D(p, a, b);
            if (orient != 0)
                return; // Point is not on the line

            // Point is collinear, now check if it lies between the constraint endpoints
            // Choose the principal direction (x or y)
            int coordId;
            if (arithmetic.Compare(arithmetic.Get(a, 0), arithmetic.Get(b, 0)) == 0)
                coordId = 1; // Use Y if X coordinates are identical
            else
                coordId = 0; // Use X otherwise

            var pCoord = arithmetic.Get(p, coordId);
            var aCoord = arithmetic.Get(a, coordId);
            var bCoord = arithmetic.Get(b, coordId);

            var minCoord = arithmetic.Min(aCoord, bCoord);
            var maxCoord = arithmetic.Max(aCoord, bCoord);

            // Check if point is strictly between the endpoints (not at the endpoints)
            if (arithmetic.Compare(pCoord, minCoord) > 0 && arithmetic.Compare(pCoord, maxCoord) < 0)
                throw new Exception($"Constraint {otherConstraintId} endpoint (point {pointId}) lies on constraint {constraintId} line");
        }

        private bool ConstraintsIntersect(Int2 c1, Int2 c2)
        {
            var p1 = points[c1.X];
            var p2 = points[c1.Y];
            var p3 = points[c2.X];
            var p4 = points[c2.Y];

            // Check orientation of p3 and p4 with respect to line c1
            var o1 = arithmetic.Orient2D(p3, p1, p2);
            var o2 = arithmetic.Orient2D(p4, p1, p2);

            // Check orientation of p1 and p2 with respect to line c2
            var o3 = arithmetic.Orient2D(p1, p3, p4);
            var o4 = arithmetic.Orient2D(p2, p3, p4);

            // If all orientations are zero, segments are collinear
            if (o1 == 0 && o2 == 0 && o3 == 0 && o4 == 0)
            {
                // Even if constraints share an endpoint, collinear overlap is invalid
                // Check if collinear segments overlap (using existing method)
                return TriangulationHelper<Arithmetic, Vec, Scalar>.CollinearSegmentsOverlap(arithmetic, points, c1.X, c1.Y, c2);
            }

            // If constraints share an endpoint (and not collinear), they don't intersect (just touch)
            if (c1.Contains(c2.X) || c1.Contains(c2.Y))
                return false;

            // Segments intersect if endpoints are on opposite sides of each other's lines
            return Math.Sign(o1) != Math.Sign(o2) && Math.Sign(o3) != Math.Sign(o4);
        }

    }
}
