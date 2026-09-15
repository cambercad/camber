using Triangulation;

namespace GeoCore
{
    public class TriangulateConstrained<Arithmetic, Vec, Scalar> where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
    {
        private TriangulationContext<Arithmetic, Vec, Scalar> ctx;

        protected TriangulateConstrained() { }

        private TriangulateConstrained(TriangulationContext<Arithmetic, Vec, Scalar> ctx)
        {
            this.ctx = ctx;

#if DEBUG
            TriangulationDebug.CheckForDuplicatePoints(ctx.Points);
#endif
        }

        public static void Triangulate(TriangulationContext<Arithmetic, Vec, Scalar> ctx, IList<int> borderPolygon, IList<Int2> constraints, List<int> pointsToInsert = null)
        {
            TriangulateConstrained<Arithmetic, Vec, Scalar> triangulator = new TriangulateConstrained<Arithmetic, Vec, Scalar>(ctx);
            triangulator.Triangulate(borderPolygon, constraints, pointsToInsert);
        }

        //Extract a method to insert constraints one by one - useful for debugging
        private void Triangulate(IList<int> borderPolygon, IList<Int2> constraints, List<int> pointsToInsert = null)
        {
            var points = ctx.Points;
            var triangles = ctx.Triangles;
            var arithmetic = ctx.GetArithmetic();

#if DEBUG
            bool orientation = PolygonOps<Arithmetic, Vec, Scalar>.IsPolygonCCW(points, borderPolygon);
            TriangulationDebug<Arithmetic, Vec, Scalar> validator = new TriangulationDebug<Arithmetic, Vec, Scalar>(points, triangles, arithmetic);
            validator.ValidateInput(borderPolygon, constraints);
#endif

            TriangulationEarClipping<Arithmetic, Vec, Scalar> poly = new TriangulationEarClipping<Arithmetic, Vec, Scalar>();
            poly.Initialize(points, borderPolygon);
            poly.TriangulateAppend(ctx);

#if DEBUG
            var referenceArea = TriangulationDebug.ComputeAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, triangles);
#endif

            //Insert the constraint points
            HashSet<int> pointsToInsertHashSet = new HashSet<int>();
            if(pointsToInsert!=null)
            {
                for (int i = 0; i < pointsToInsert.Count; i++)
                    pointsToInsertHashSet.Add(pointsToInsert[i]);
            }
            for (int i = 0; i < constraints.Count; ++i)
            {
                var c = constraints[i];
                pointsToInsertHashSet.Add(c.X);
                pointsToInsertHashSet.Add(c.Y);
            }
            for (int i = 0; i < borderPolygon.Count; ++i)
                pointsToInsertHashSet.Remove(borderPolygon[i]);

            foreach (int id in pointsToInsertHashSet)
            {
                InsertPoint(id);
#if DEBUG
                if (!TriangulationDebug.PointExistsInTriangles(triangles, id))
                    throw new Exception();
#endif
            }

            HashSet<int> vertexSet = new HashSet<int>();
            while (true) //Safety net because it is possible that enforcing one constraint removes another from the triangulation
            {
                int constraintImmediatelySatisfiedCount = 0;
                for (int i = 0; i < constraints.Count; ++i)
                {
                    Int2 c = constraints[i];
                    if (EdgeExists(c))
                    {
                        ++constraintImmediatelySatisfiedCount;
                        continue;
                    }

                    if (!PointExistsInTriangles(c.X))
                        InsertPoint(c.X);
                    if (!PointExistsInTriangles(c.Y))
                        InsertPoint(c.Y);

                    List<Tri> removedTriangles = RemoveIntersectingTriangles(c);
                    if (removedTriangles.Count == 0)
                        continue;

                    vertexSet.Clear();
                    for (int j = 0; j < removedTriangles.Count; ++j)
                    {
                        var tri = removedTriangles[j];
                        vertexSet.Add(tri.A);
                        vertexSet.Add(tri.B);
                        vertexSet.Add(tri.C);
                    }

#if DEBUG


                    if (!TriangulationDebug.PointExistsInTriangles(removedTriangles, c.X))
                        throw new Exception();
                    if (!TriangulationDebug.PointExistsInTriangles(removedTriangles, c.Y))
                        throw new Exception();
#endif

                    List<int> polygon = ExtractBoundaryPolygon(removedTriangles, out List<List<int>> holes);

#if DEBUG
                    var refRemovedArea = TriangulationDebug.ComputeAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, removedTriangles);
                    var polyArea = TriangulationDebug.ComputePolygonAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, polygon);
                    var holeArea = TriangulationDebug.ComputePolygonAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, holes); // Has negative sign
                    if (polyArea + holeArea != refRemovedArea)
                    {
                        //var dd = new List<List<int>>();
                        //dd.Add(polygon);
                        //dd.AddRange(holes);
                        //TriangulationDebug.WriteLineObj(@"C:\tmp\dbg\E" + (debugIndexer++) + ".obj", (List<Rat2Hybrid>)(object)points, dd, new List<Int2>());

                        throw new Exception();
                    }
#endif

                    List<int> polygonA, polygonB;
                    SplitPolygonByConstraint(c, polygon, out polygonA, out polygonB);

                    for (int j = 0; j < polygonA.Count; ++j)
                        vertexSet.Remove(polygonA[j]);
                    for (int j = 0; j < polygonB.Count; ++j)
                        vertexSet.Remove(polygonB[j]);

                    // Distribute the holes to polygonA and polygonB
                    List<List<int>> holesA = new List<List<int>>();
                    List<List<int>> holesB = new List<List<int>>();

                    DistributeHoles(polygonA, polygonB, holes, holesA, holesB);

                    if (polygonA.Count >= 3)
                    {
                        poly.Initialize(points, polygonA, holesA);
                        poly.TriangulateAppend(ctx);
                    }
                    if (polygonB.Count >= 3)
                    {
                        poly.Initialize(points, polygonB, holesB);
                        poly.TriangulateAppend(ctx);
                    }

#if DEBUG
                    BigRationalHybrid polyAreaA = TriangulationDebug.ComputePolygonAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, polygonA);
                    BigRationalHybrid polyAreaB = TriangulationDebug.ComputePolygonAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, polygonB);

                    var holeAreaA = TriangulationDebug.ComputePolygonAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, holesA); // Has negative sign
                    var holeAreaB = TriangulationDebug.ComputePolygonAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, holesB); // Has negative sign
                    var refRemovedAreaSplit = TriangulationDebug.ComputeAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, removedTriangles);
                    var totalArea = (polyAreaA + holeAreaA) + (polyAreaB + holeAreaB);
                    if (totalArea != refRemovedAreaSplit)
                        throw new Exception();
#endif



                    //Insert the points that remain in vertexSet
                    foreach (var v in vertexSet)
                        InsertPointIfNotPresent(v);

#if DEBUG
                    if (!TriangulationDebug.AllPointsExistsInTriangles(triangles, pointsToInsertHashSet))
                    {
                        TriangulationDebug.WriteLineObj(@"C:\tmp\dbg\abc.obj", (List<Rat2Hybrid>)(object)points, polygon, new List<Int2>() { c });
                        TriangulationDebug.WriteLineObj(@"C:\tmp\dbg\abc2.obj", (List<Rat2Hybrid>)(object)points, removedTriangles);

                        throw new Exception();
                    }


                    var area = TriangulationDebug.ComputeAreaTimesTwo<Arithmetic, Vec, Scalar>(arithmetic, points, triangles);
                    if (area != referenceArea)
                        throw new Exception();

                    var debug = CollecteIntersectingTriangles(c);
                    if (debug.Count != 0)
                        throw new Exception();
#endif
                }

                if (constraintImmediatelySatisfiedCount == constraints.Count)
                    break;
            }

#if DEBUG
            // Check if all constraints are satisfied
            HashSet<long> triEdges = new HashSet<long>();
            HashSet<int> triPoints = new HashSet<int>();
            for (int i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                triEdges.Add(Algorithms.Key(tri.A, tri.B));
                triEdges.Add(Algorithms.Key(tri.B, tri.C));
                triEdges.Add(Algorithms.Key(tri.C, tri.A));
                triPoints.Add(tri.A);
                triPoints.Add(tri.B);
                triPoints.Add(tri.C);
            }

            for (int i = 0; i < constraints.Count; ++i)
            {
                var c = constraints[i];
                if (c.X == c.Y)
                    continue;
                var k = Algorithms.Key(c.X, c.Y);
                if (!triEdges.Contains(k))
                    throw new Exception();
            }

            for (int i = 0; i < constraints.Count; ++i)
            {
                var c = constraints[i];
                if (c.X == c.Y)
                {
                    if (!triPoints.Contains(c.X))
                        throw new Exception();
                }
            }
#endif
        }

        private void DistributeHoles(List<int> polygonA, List<int> polygonB, List<List<int>> holes, List<List<int>> holesA, List<List<int>> holesB)
        {
            var points = ctx.Points;
            var arithmetic = ctx.GetArithmetic();
            for (int i = 0; i < holes.Count; ++i)
            {
                Vec testPoint = points[holes[i][0]];
                var resultA = PolygonOps<Arithmetic, Vec, Scalar>.IsPointInPolygon(points, polygonA, testPoint);
                if (resultA == PointInPolygonResult.Inside)
                {
                    holesA.Add(holes[i]);
                }
                else
                {
                    var resultB = PolygonOps<Arithmetic, Vec, Scalar>.IsPointInPolygon(points, polygonB, testPoint);
                    if (resultB != PointInPolygonResult.Inside)
                        throw new Exception("Hole is neither inside polygonA nor polygonB");
                    holesB.Add(holes[i]);
                }
            }
        }

        private bool PointExistsInTriangles(int id)
        {
            var triangles = ctx.Triangles;
            for (int i = 0; i < triangles.Count; ++i)
            {
                if (triangles[i].Contains(id))
                    return true;
            }
            return false;
        }

        private bool EdgeExists(Int2 c)
        {
            var triangles = ctx.Triangles;
            for (int i = 0; i < triangles.Count; ++i)
            {
                var tri = triangles[i];
                if (tri.Contains(c.X) && tri.Contains(c.Y))
                    return true;
            }
            return false;
        }

        private void SplitPolygonByConstraint(Int2 c, List<int> polygon, out List<int> polygonA, out List<int> polygonB)
        {
            EnsureVertexOnPolygonBoundary(polygon, c.X);
            EnsureVertexOnPolygonBoundary(polygon, c.Y);

            int idX = polygon.IndexOf(c.X);
            if (idX < 0)
                throw new InvalidOperationException($"Constraint vertex {c.X} could not be placed on the extracted polygon boundary.");

            int idY = polygon.IndexOf(c.Y);
            if (idY < 0)
                throw new InvalidOperationException($"Constraint vertex {c.Y} could not be placed on the extracted polygon boundary.");

            if (idX > idY)
                Algorithms.Swap(ref idX, ref idY);

            polygonA = new List<int>();
            polygonB = new List<int>();

            for (int i = 0; i < polygon.Count; ++i)
            {
                if (i == idX || i == idY)
                {
                    polygonA.Add(polygon[i]);
                    polygonB.Add(polygon[i]);
                    continue;
                }
                if (i > idX && i < idY)
                    polygonA.Add(polygon[i]);
                else
                    polygonB.Add(polygon[i]);
            }
        }

        private List<int> ExtractBoundaryPolygon(List<Tri> removedTriangles, out List<List<int>> holes)
        {
            var points = ctx.Points;
            TriangleEdge[] allTriEdges = Adjacency.BuildEdgeList(removedTriangles);

            List<TriangleEdge> borderEdges = new List<TriangleEdge>();
            Dictionary<int, List<int>> vertexToEdge = new Dictionary<int, List<int>>();
            for (int i = 0; i < allTriEdges.Length; ++i)
            {
                var e = allTriEdges[i];
                if (e.NeighbourIndex2 != -1)
                    continue;
                if (e.Reversed)
                    e.Reverse();

                borderEdges.Add(e);

                List<int> l;
                if (vertexToEdge.TryGetValue(e.Start, out l))
                    l.Add(e.End);
                else
                {
                    l = new List<int>();
                    l.Add(e.End);
                    vertexToEdge.Add(e.Start, l);
                }
            }

            bool[] edgeDone = new bool[borderEdges.Count];
            int edgeDoneCounter = 0;

            List<List<int>> allPolygons = new List<List<int>>();
            while (edgeDoneCounter < borderEdges.Count)
            {
                int seedEdgeIdx = -1;
                for (int i = 0; i < edgeDone.Length; ++i)
                {
                    if (!edgeDone[i])
                    {
                        seedEdgeIdx = i;
                        break;
                    }
                }

                TriangleEdge seedEdge = borderEdges[seedEdgeIdx];
                int seed = seedEdge.Start;
                List<int> currentPolygon = new List<int>();
                currentPolygon.Add(seed);
                allPolygons.Add(currentPolygon);

                int current = seed;
                int destination = seedEdge.End;
                MarkEdgeDone(borderEdges, edgeDone, current, destination, ref edgeDoneCounter);

                //while (destination != seed)
                while (true)
                {
                    currentPolygon.Add(destination);
                    int prev = current;
                    current = destination;

                    var destinations = vertexToEdge[current];
                    destination = -1;
                    if (destinations.Count == 1)
                    {
                        if (IsEdgeAvailable(borderEdges, edgeDone, current, destinations[0]))
                            destination = destinations[0];
                    }
                    else
                    {
                        Vec currPoint = points[current];
                        Vec prevPoint = points[prev];

                        for (int i = 0; i < destinations.Count; ++i)
                        {
                            int cand = destinations[i];
                            if (!IsEdgeAvailable(borderEdges, edgeDone, current, cand))
                                continue;
                            if (destination < 0 || IsCCwCloser(prevPoint, currPoint, points[destination], points[cand]))
                                destination = cand;
                        }
                    }
                    if (destination < 0)
                    {
                        if (seed != currentPolygon[currentPolygon.Count - 1])
                            throw new Exception();
                        currentPolygon.RemoveAt(currentPolygon.Count - 1);
                        break;
                    }

                    MarkEdgeDone(borderEdges, edgeDone, current, destination, ref edgeDoneCounter);
                }
            }

            holes = new List<List<int>>();
            int outermostPolyIndex = 0;
            if (allPolygons.Count > 1)
            {
                List<PolygonTree> trees = PolygonOps<Arithmetic, Vec, Scalar>.BuildPolyTree(points, allPolygons);
                if (trees.Count > 1)
                {
                    //TriangulationDebug.WriteLineObj(@"C:\tmp\dbg\D" + (debugIndexer++) + ".obj", (List<Rat2Hybrid>)(object)points, allPolygons, new List<Int2>());
                    throw new Exception("The tree can only have one root, otherwise something goes quite wrong");
                }

                var tree = trees[0];
                outermostPolyIndex = tree.PolyId;
                for (int i = 0; i < tree.Children.Count; ++i)
                {
                    var c = tree.Children[i];
                    if (c.Children.Count > 0)
                        throw new Exception("Nesting not supported");

                    var id = c.PolyId;
                    holes.Add(allPolygons[id]);
                }
            }

            return allPolygons[outermostPolyIndex];
        }

        private static void MarkEdgeDone(List<TriangleEdge> borderEdges, bool[] edgeDone, int start, int end, ref int edgeDoneCounter)
        {
            for (int i = 0; i < borderEdges.Count; ++i)
            {
                if (!edgeDone[i] && borderEdges[i].Start == start && borderEdges[i].End == end)
                {
                    edgeDone[i] = true;
                    ++edgeDoneCounter;
                    return;
                }
            }
        }

        private static bool IsEdgeAvailable(List<TriangleEdge> borderEdges, bool[] edgeDone, int start, int end)
        {
            for (int i = 0; i < borderEdges.Count; ++i)
            {
                if (borderEdges[i].Start == start && borderEdges[i].End == end)
                    return !edgeDone[i];
            }
            return false;
        }

        private bool IsCCwCloser(in Vec start, in Vec end, in Vec currentBest, in Vec candidate)
        {
            var arithmetic = ctx.GetArithmetic();
            int oBest = arithmetic.Orient2D(start, end, currentBest);
            int oCand = arithmetic.Orient2D(start, end, candidate);

            if (oBest != oCand)
                return oCand > oBest;

            return arithmetic.Orient2D(end, currentBest, candidate) > 0;
        }

        private List<Tri> RemoveIntersectingTriangles(Int2 c)
        {
            var triangles = ctx.Triangles;
            var constraintBox = ctx.PointBounds[c.X];
            constraintBox.Extend(ctx.PointBounds[c.Y]);

            List<Tri> intersectingTriangles = new List<Tri>();
            for (int i = triangles.Count - 1; i >= 0; --i)
            {
                if (TriangleViolatesConstraint(i, c, constraintBox))
                {
                    intersectingTriangles.Add(triangles[i]);
                    ctx.RemoveTriangleAt(i);
                }
            }
            return intersectingTriangles;
        }

        private List<Tri> CollecteIntersectingTriangles(Int2 c)
        {
            var triangles = ctx.Triangles;
            var constraintBox = ctx.PointBounds[c.X];
            constraintBox.Extend(ctx.PointBounds[c.Y]);

            List<Tri> intersectingTriangles = new List<Tri>();
            for (int i = triangles.Count - 1; i >= 0; --i)
            {
                if (TriangleViolatesConstraint(i, c, constraintBox))
                {
                    intersectingTriangles.Add(triangles[i]);
                }
            }
            return intersectingTriangles;
        }

        private bool TriangleViolatesConstraint(int triIndex, Int2 constraint, in Box2D constraintBox)
        {
            if (!BoundsOverlap(triIndex, constraintBox))
                return false; //Cheap early out to improve the performance

            var t = ctx.Triangles[triIndex];
            var points = ctx.Points;
            var arithmetic = ctx.GetArithmetic();

            if (TriangulationHelper<Arithmetic, Vec, Scalar>.SegmentsIntersect(arithmetic, points, t.A, t.B, constraint))
                return true;
            if (TriangulationHelper<Arithmetic, Vec, Scalar>.SegmentsIntersect(arithmetic, points, t.B, t.C, constraint))
                return true;
            if (TriangulationHelper<Arithmetic, Vec, Scalar>.SegmentsIntersect(arithmetic, points, t.C, t.A, constraint))
                return true;

            return false;
        }

        public bool IntervalOverlapOrTouch(Scalar startA, Scalar endA, Scalar startB, Scalar endB)
        {
            return !(ctx.GetArithmetic().Compare(endA, startB) < 0 || ctx.GetArithmetic().Compare(endB, startA) < 0);
        }

        private void InsertPoint(int id)
        {
            var triangles = ctx.Triangles;
            int count = triangles.Count;
            var pBox = ctx.PointBounds[id];
            bool pointInserted = false;
            for (int i = 0; i < count; ++i)
            {
                if (!BoundsOverlap(i, pBox))
                    continue; //Cheap early out to improve the performance

                if (PointInsideOrOnBoundaryExcludeCorners(triangles[i], id))
                {
                    InsertPoint(i, id);
                    pointInserted = true;
                }
            }
            if (!pointInserted)
                throw new Exception();
        }

        private bool InsertPointIfNotPresent(int id)
        {
            var triangles = ctx.Triangles;
            int count = triangles.Count;
            var pBox = ctx.PointBounds[id];
            bool pointInserted = false;
            for (int i = 0; i < count; ++i)
            {
                var tri = triangles[i];
                if (!BoundsOverlap(i, pBox))
                    continue; //Cheap early out to improve the performance

                if (tri.A == id || tri.B == id || tri.C == id)
                    return true;

                if (PointInsideOrOnBoundaryExcludeCorners(tri, id))
                {
                    InsertPoint(i, id);
                    pointInserted = true;
                }
            }

            if (!pointInserted)
                throw new Exception();

            return pointInserted;
        }

        private void InsertPoint(int triangleId, int pointId)
        {
            var points = ctx.Points;
            var triangles = ctx.Triangles;
            var p = points[pointId];
            var tri = triangles[triangleId];

#if DEBUG
            if (points[tri.A].Equals(p) || points[tri.B].Equals(p) || points[tri.C].Equals(p))
                throw new Exception();
#endif

            var ab = ctx.Orient2D(pointId, tri.A, tri.B);
            var bc = ctx.Orient2D(pointId, tri.B, tri.C);
            var ca = ctx.Orient2D(pointId, tri.C, tri.A);

            if (ab > 0 && bc > 0 && ca > 0)
                Flip1To3(triangleId, pointId);
            else if (ab == 0 && bc > 0 && ca > 0)
                Flip1To2(triangleId, pointId, 0);
            else if (bc == 0 && ab > 0 && ca > 0)
                Flip1To2(triangleId, pointId, 1);
            else if (ca == 0 && ab > 0 && bc > 0)
                Flip1To2(triangleId, pointId, 2);
            else
            {
#if DEBUG
                TriangulationDebug.CheckForDuplicatePoints(points);
#endif
                throw new Exception();
            }
        }

        private void Flip1To2(int triangleId, int pointId, int edgeId)
        {
            var tri = ctx.Triangles[triangleId];
            int splitA = tri.Get(edgeId);
            int splitB = tri.Get((edgeId + 1) % 3);
            int remaining = tri.A + tri.B + tri.C - splitA - splitB;
            ctx.SetTriangle(triangleId, new Tri(splitB, remaining, pointId));
            ctx.AddTriangle(new Tri(remaining, splitA, pointId));
        }

        private void Flip1To3(int triangleId, int pointId)
        {
            var tri = ctx.Triangles[triangleId];
            ctx.SetTriangle(triangleId, new Tri(tri.A, tri.B, pointId));
            ctx.AddTriangle(new Tri(tri.B, tri.C, pointId));
            ctx.AddTriangle(new Tri(tri.C, tri.A, pointId));
        }

        private bool BoundsOverlap(int triIndex, in Box2D queryBox)
        {
            return ctx.BoundsForTri(ctx.Triangles[triIndex]).OverlapOrTouch(queryBox);
        }

        private void EnsureVertexOnPolygonBoundary(List<int> polygon, int vertexId)
        {
            if (polygon.Contains(vertexId))
                return;

            for (int i = 0; i < polygon.Count; ++i)
            {
                int j = (i + 1) % polygon.Count;
                if (PointOnPolygonEdge(vertexId, polygon[i], polygon[j]))
                {
                    polygon.Insert(j, vertexId);
                    return;
                }
            }
        }

        private bool PointOnPolygonEdge(int vertexId, int segStartId, int segEndId)
        {
            var points = ctx.Points;
            var arithmetic = ctx.GetArithmetic();
            if (segStartId == vertexId || segEndId == vertexId)
                return false;

            var p = points[vertexId];
            var a = points[segStartId];
            var b = points[segEndId];
            if (ctx.Orient2D(segStartId, segEndId, vertexId) != 0)
                return false;

            return IntervalOverlapOrTouch(
                    arithmetic.Get(a, 0), arithmetic.Get(b, 0),
                    arithmetic.Get(p, 0), arithmetic.Get(p, 0))
                && IntervalOverlapOrTouch(
                    arithmetic.Get(a, 1), arithmetic.Get(b, 1),
                    arithmetic.Get(p, 1), arithmetic.Get(p, 1));
        }

        private bool PointInsideOrOnBoundaryExcludeCorners(Tri tri, int pointId)
        {
            if (tri.A == pointId || tri.B == pointId || tri.C == pointId)
                return false;

            return PointInsideOrOnBoundary(tri, pointId);
        }
        private bool PointInsideOrOnBoundary(Tri tri, int pointId)
        {
            var ab = ctx.Orient2D(pointId, tri.A, tri.B);
            var bc = ctx.Orient2D(pointId, tri.B, tri.C);
            var ca = ctx.Orient2D(pointId, tri.C, tri.A);

            return ab >= 0 && bc >= 0 && ca >= 0;
        }
    }
}
