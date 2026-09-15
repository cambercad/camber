namespace GeoCore
{
    //Polygon triangulation
    public class TriangulationEarClipping<Arithmetic, Vec, Scalar> where Arithmetic : struct, ITriangulationArithmetic<Vec, Scalar>
    {
        private List<Vec> polygon;
        private int count;
        private int[] offsets;
        private IList<int> indexMap;
        private Arithmetic arithmetic = new Arithmetic();
        private readonly Dictionary<(int, int, int), int> orient2DCache = new Dictionary<(int, int, int), int>();
        private readonly List<Box2D> bridgePointBounds = new List<Box2D>();
        private List<Vec> bridgePoints;
        private List<List<int>> bridgeHoles;

        private void ClearOrient2DCache() => orient2DCache.Clear();

        private int Orient2D(int a, int b, int c)
        {
            var key = (a, b, c);
            if (!orient2DCache.TryGetValue(key, out int result))
            {
                result = arithmetic.Orient2D(polygon[a], polygon[b], polygon[c]);
                orient2DCache.Add(key, result);
            }
            return result;
        }

        // All boundary segments must be part of closed loops
        // Holes are allows but MUST not contain other segments/polygons themselves (no nesting)
        // There is exactly one outer border containing all holes
        public void Initialize(List<Vec> points, List<int> outerBorder, List<List<int>> holes, bool skipOrientationCheck = false)
        {
            if (!skipOrientationCheck && !PolygonOps<Arithmetic, Vec, Scalar>.IsPolygonCCW(points, outerBorder))
                outerBorder.Reverse();

            if (holes.Count == 0)
            {
                Initialize(points, outerBorder);
                return;
            }

            outerBorder = new List<int>(outerBorder);
            holes = new List<List<int>>(holes);


            if (!skipOrientationCheck)
            {
                for (int i = 0; i < holes.Count; ++i)
                {
                    holes[i] = new List<int>(holes[i]);
                    var hole = holes[i];
                    if (PolygonOps<Arithmetic, Vec, Scalar>.IsPolygonCCW(points, hole))
                        hole.Reverse();
                }
            }

            List<(Vec, int, List<int>, int)> maxXPerHole = new List<(Vec, int, List<int>, int)>();
            for (int i = 0; i < holes.Count; ++i)
            {
                var hole = holes[i];


                Vec maxX = points[hole[0]];
                int maxXId = 0;
                for (int j = 1; j < hole.Count; ++j)
                {
                    Scalar x = arithmetic.Get(maxX, 0);
                    var c = points[hole[j]];
                    Scalar currentX = arithmetic.Get(c, 0);

                    int comp = arithmetic.Compare(currentX, x);
                    if(comp == 0)
                    {
                        // Break the tie by examining y
                        Scalar y = arithmetic.Get(maxX, 1);
                        Scalar currentY = arithmetic.Get(c, 1); 
                        int comp2 = arithmetic.Compare(currentY, y);
                        if (comp2 == 0)
                            throw new Exception("Duplicate points - might still be ok but needs implementation");

                        else if (comp2 == 1)
                        {
                            maxX = c;
                            maxXId = j;
                        }
                    }
                    else if(comp == 1)
                    {
                        maxX = c;
                        maxXId = j;
                    }
                }

                maxXPerHole.Add((maxX, maxXId, hole, i));
            }

            maxXPerHole.Sort(delegate ((Vec pt, int, List<int>, int) a, (Vec pt, int, List<int>, int) b)
            {
                int comp = arithmetic.Compare(arithmetic.Get(a.pt, 0), arithmetic.Get(b.pt, 0));
                if (comp != 0)
                    return -comp;
                int comp2 = arithmetic.Compare(arithmetic.Get(a.pt, 1), arithmetic.Get(b.pt, 1));
                return -comp2;
            });

            BuildBridgePointBounds(points);
            bridgeHoles = holes;
            var holePolygonBounds = new List<Box2D>(holes.Count);
            for (int h = 0; h < holes.Count; ++h)
                holePolygonBounds.Add(PolygonBounds(holes[h]));

            var originalOuterPointIndices = new HashSet<int>(outerBorder);
            for (int i = 0; i < maxXPerHole.Count; ++i)
            {
                var hole = maxXPerHole[i];
                int inPolygonId = FindVisiblePoint(hole.Item2, outerBorder, hole.Item3, hole.Item4, holePolygonBounds, originalOuterPointIndices);
                if (inPolygonId < 0)
                    throw new InvalidOperationException("No visible bridge found for polygon hole");

                var copy = outerBorder[inPolygonId];
                var shifted = CyclicShift(hole.Item3, hole.Item2);
                outerBorder.InsertRange(inPolygonId, shifted);
                outerBorder.Insert(inPolygonId + hole.Item3.Count, shifted[0]);
                outerBorder.Insert(inPolygonId, copy);
            }

            bridgePoints = null;
            bridgeHoles = null;
            //TriangulationDebug.WriteLineObj(@"C:\tmp\dbg\" + (indexer++)+".obj", (List<Rat2Hybrid>)(object)points, outerBorder, new List<Int2>());
            Initialize(points, outerBorder);
        }

        private void BuildBridgePointBounds(List<Vec> points)
        {
            bridgePoints = points;
            int n = points.Count;
            if (bridgePointBounds.Count < n)
            {
                int grow = n - bridgePointBounds.Count;
                for (int i = 0; i < grow; ++i)
                    bridgePointBounds.Add(default);
            }
            else if (bridgePointBounds.Count > n)
                bridgePointBounds.RemoveRange(n, bridgePointBounds.Count - n);

            for (int i = 0; i < n; ++i)
                bridgePointBounds[i] = arithmetic.GetBounds(points[i]);
        }

        private Box2D PolygonBounds(List<int> polygon)
        {
            var box = bridgePointBounds[polygon[0]];
            for (int i = 1; i < polygon.Count; ++i)
                box.Extend(bridgePointBounds[polygon[i]]);
            return box;
        }

        private List<int> CyclicShift(List<int> list, int shift)
        {
            while (shift < 0)
                shift += list.Count;

            List<int> result = new List<int>(list.Count);
            for (int i = 0; i < list.Count; ++i)
            {
                result.Add(list[(i + shift) % list.Count]);
            }
            return result;
        }

        private int FindVisiblePoint(int sourcePointId, List<int> outerBorder, List<int> hole, int holeIndex, List<Box2D> holePolygonBounds, HashSet<int> allowedOuterPointIndices)
        {
            int sourcePoint = hole[sourcePointId];
            Box2D outerBox = PolygonBounds(outerBorder);
            Box2D currentHoleBox = holePolygonBounds[holeIndex];

            for (int i = 0; i < outerBorder.Count; ++i)
            {
                int targetPoint = outerBorder[i];
                if (!allowedOuterPointIndices.Contains(targetPoint))
                    continue;

                Int2 segment = new Int2(sourcePoint, targetPoint);
                Box2D segmentBox = TriangulationHelper<Arithmetic, Vec, Scalar>.BoundsForSegment(
                    bridgePointBounds[sourcePoint], bridgePointBounds[targetPoint]);

                if (SegmentIsClear(segment, segmentBox, outerBorder, hole, holeIndex, holePolygonBounds, currentHoleBox, outerBox))
                    return i;
            }
            return -1;
        }

        private bool SegmentIsClear(Int2 segment, in Box2D segmentBox, List<int> outerBorder, List<int> hole,
            int currentHoleIndex, List<Box2D> holePolygonBounds, in Box2D currentHoleBox, in Box2D outerBox)
        {
            if (SegmentIntersectsPolygon(hole, segment, segmentBox, currentHoleBox))
                return false;
            for (int i = 0; i < bridgeHoles.Count; ++i)
            {
                if (i == currentHoleIndex)
                    continue;
                if (SegmentIntersectsPolygon(bridgeHoles[i], segment, segmentBox, holePolygonBounds[i]))
                    return false;
            }
            if (SegmentIntersectsPolygon(outerBorder, segment, segmentBox, outerBox))
                return false;
            return true;
        }

        private bool SegmentIntersectsPolygon(List<int> polygon, Int2 segment, in Box2D segmentBox, in Box2D polygonBox)
        {
            if (!segmentBox.OverlapOrTouch(polygonBox))
                return false;

            for (int i = 0; i < polygon.Count; ++i)
            {
                int start = polygon[i];
                int end = polygon[(i + 1) % polygon.Count];
                var edgeBox = TriangulationHelper<Arithmetic, Vec, Scalar>.BoundsForSegment(
                    bridgePointBounds[start], bridgePointBounds[end]);
                if (!segmentBox.OverlapOrTouch(edgeBox))
                    continue;

                if (TriangulationHelper<Arithmetic, Vec, Scalar>.SegmentsIntersectUnchecked(
                        arithmetic, bridgePoints, start, end, segment))
                    return true;
            }
            return false;
        }

        public void Initialize(List<Vec> points, IList<int> indexMap = null)
        {
#if DEBUG
            //Validate(points, indexMap);
#endif

            if (indexMap == null)
                this.polygon = points;
            else
            {
                this.polygon = new List<Vec>(indexMap.Count);
                for (int i = 0; i < indexMap.Count; ++i)
                    polygon.Add(points[indexMap[i]]);
                this.indexMap = indexMap;
            }
            count = polygon.Count;
            offsets = new int[count];
            for (int i = 0; i < count; ++i)
                offsets[i] = 1;
        }

        // No self intersection (touch or point on other segment counts as self intersection)
        private void Validate(List<Vec> points, IList<int> indexMap)
        {
            // Build the polygon vertex sequence (same logic as Initialize)
            IList<int> idx = indexMap ?? (IList<int>)Enumerable.Range(0, points.Count).ToList();
            int n = idx.Count;

            for (int i = 0; i < n; i++)
            {
                Vec p1 = points[idx[i]];
                Vec p2 = points[idx[(i + 1) % n]];

                for (int j = i + 2; j < n; j++)
                {
                    // Adjacent edges share exactly one endpoint → allowed to touch there
                    if (i == 0 && j == n - 1) continue;

                    Vec p3 = points[idx[j]];
                    Vec p4 = points[idx[(j + 1) % n]];

                    var o1 = arithmetic.Orient2D(p3, p1, p2);
                    var o2 = arithmetic.Orient2D(p4, p1, p2);
                    var o3 = arithmetic.Orient2D(p1, p3, p4);
                    var o4 = arithmetic.Orient2D(p2, p3, p4);

                    // Proper crossing
                    if (Math.Sign(o1) != Math.Sign(o2) && Math.Sign(o3) != Math.Sign(o4))
                        throw new Exception($"Polygon self-intersects: edge {i}-{i+1} crosses edge {j}-{(j+1)%n}");

                    // Any endpoint touching the interior of the other segment
                    if (o1 == 0 && PointStrictlyBetween(p3, p1, p2))
                        throw new Exception($"Polygon self-intersects: vertex {j} lies on edge {i}-{i+1}");
                    if (o2 == 0 && PointStrictlyBetween(p4, p1, p2))
                        throw new Exception($"Polygon self-intersects: vertex {(j+1)%n} lies on edge {i}-{i+1}");
                    if (o3 == 0 && PointStrictlyBetween(p1, p3, p4))
                        throw new Exception($"Polygon self-intersects: vertex {i} lies on edge {j}-{(j+1)%n}");
                    if (o4 == 0 && PointStrictlyBetween(p2, p3, p4))
                        throw new Exception($"Polygon self-intersects: vertex {i+1} lies on edge {j}-{(j+1)%n}");
                }
            }
        }

        private bool PointStrictlyBetween(Vec p, Vec a, Vec b)
        {
            // Assumes p is collinear with a–b. Pick the non-degenerate axis.
            int coord = arithmetic.Compare(arithmetic.Get(a, 0), arithmetic.Get(b, 0)) == 0 ? 1 : 0;
            var pv = arithmetic.Get(p, coord);
            var lo = arithmetic.Min(arithmetic.Get(a, coord), arithmetic.Get(b, coord));
            var hi = arithmetic.Max(arithmetic.Get(a, coord), arithmetic.Get(b, coord));
            return arithmetic.Compare(pv, lo) > 0 && arithmetic.Compare(pv, hi) < 0;
        }

        private int Next(int index)
        {
            if (offsets[index] == -1)
                throw new Exception();

            int result = (index + offsets[index]) % polygon.Count;
            if (result < 0)
                throw new Exception();
            return result;
        }

        private int NextNext(int index)
        {
            return Next(Next(index));
        }

        private void RemoveAt(int index, int previous)
        {
            if (offsets[index] == -1)
                throw new Exception();
            if (offsets[previous] == -1)
                throw new Exception();

            offsets[previous] += offsets[index];
            offsets[index] = -1;
            --count;
        }

        public void Triangulate(List<Tri> result)
        {
#if DEBUG
            var polyAreaTimesTwo = ComputePolygonAreaTimesTwo();
            var triAreaSum = BigRationalHybrid.Zero;
#endif

            ClearOrient2DCache();
            List<Tri> tmp = new List<Tri>();

            int index = 0;
            int skippedVertices = 0;
            while (count >= 3)
            {
                
                var a = polygon[index];
                var next = Next(index);
                var b = polygon[next];
                var nextnext = NextNext(index);
                var c = polygon[nextnext];
                int o = Orient2D(index, next, nextnext);

                if (count == 3 && o <= 0)
                    throw new Exception();

                if (o > 0 && IsValid(index, next, nextnext))
                {
                    if (indexMap == null)
                        tmp.Add(new Tri(index, next, nextnext));
                    else
                        tmp.Add(new Tri(indexMap[index], indexMap[next], indexMap[nextnext]));

#if DEBUG
                    triAreaSum += arithmetic.SignedArea2DTimesTwo(a, b, c);
#endif
                    RemoveAt(next, index);
                    index = nextnext;
                    skippedVertices = 0;
                }
                else
                {
                    index = next;
                    ++skippedVertices;
                    if (skippedVertices >= count)
                        throw new InvalidOperationException("No valid ear found for polygon");
                }
                
            }

            result.AddRange(tmp);

#if DEBUG
            if (typeof(Scalar) != typeof(double))
                if (polyAreaTimesTwo != triAreaSum)
                    throw new Exception();
#endif
        }

        public void TriangulateAppend(TriangulationContext<Arithmetic, Vec, Scalar> ctx)
        {
#if DEBUG
            var polyAreaTimesTwo = ComputePolygonAreaTimesTwo();
            var triAreaSum = BigRationalHybrid.Zero;
#endif

            ClearOrient2DCache();
            int index = 0;
            int skippedVertices = 0;
            while (count >= 3)
            {
                var a = polygon[index];
                var next = Next(index);
                var b = polygon[next];
                var nextnext = NextNext(index);
                var c = polygon[nextnext];
                int o = Orient2D(index, next, nextnext);

                if (count == 3 && o <= 0)
                    throw new Exception();

                if (o > 0 && IsValid(index, next, nextnext))
                {
                    if (indexMap == null)
                        ctx.AddTriangle(new Tri(index, next, nextnext));
                    else
                        ctx.AddTriangle(new Tri(indexMap[index], indexMap[next], indexMap[nextnext]));

#if DEBUG
                    triAreaSum += arithmetic.SignedArea2DTimesTwo(a, b, c);
#endif
                    RemoveAt(next, index);
                    index = nextnext;
                    skippedVertices = 0;
                }
                else
                {
                    index = next;
                    ++skippedVertices;
                    if (skippedVertices >= count)
                        throw new InvalidOperationException("No valid ear found for polygon");
                }
            }

#if DEBUG
            if (typeof(Scalar) != typeof(double))
                if (polyAreaTimesTwo != triAreaSum)
                    throw new Exception();
#endif
        }

        private BigRationalHybrid ComputePolygonAreaTimesTwo()
        {
            BigRationalHybrid sum = BigRationalHybrid.Zero;
            var reference = polygon[0];
            for(int i=0;i<polygon.Count;++i)
            {
                sum += arithmetic.SignedArea2DTimesTwo(polygon[i], polygon[(i + 1) % polygon.Count], reference);
            }
            return sum;
        }

        /// <summary>
        /// Validates whether a potential "ear" triangle is valid for clipping in the ear clipping algorithm.
        /// 
        /// An "ear" is a triangle formed by three consecutive vertices (index, next, nextnext) of the polygon
        /// that can be safely removed without creating an invalid triangulation.
        /// 
        /// ALGORITHM EXPLANATION:
        /// The ear clipping algorithm works by repeatedly finding and removing "ears" from a polygon.
        /// A valid ear must satisfy two conditions:
        /// 1. The triangle must be oriented counter-clockwise (CCW) - checked before calling IsValid
        /// 2. No other polygon vertices lie inside the ear triangle - checked by this method
        /// 
        /// WHAT THIS METHOD DOES:
        /// This method checks if any remaining polygon vertices (excluding the three vertices of the 
        /// candidate ear) lie inside the triangle formed by (index, next, nextnext).
        /// 
        /// It uses the Orient2D test to determine if a point is inside the triangle:
        /// - Orient2D(A, B, C) computes the signed area of triangle ABC (times 2)
        /// - Returns: positive if C is to the LEFT of vector AB (CCW turn)
        ///            negative if C is to the RIGHT of vector AB (CW turn)
        ///            zero if A, B, C are collinear
        /// 
        /// POINT-IN-TRIANGLE TEST:
        /// For a CCW-oriented triangle (index, next, nextnext), a point j is inside or on the boundary
        /// if it is on the left side (or collinear) with respect to ALL THREE edges:
        /// - Orient2D(index, next, j) >= 0 (j is left of or on edge index->next)
        /// - Orient2D(next, nextnext, j) >= 0 (j is left of or on edge next->nextnext)
        /// - Orient2D(nextnext, index, j) >= 0 (j is left of or on edge nextnext->index)
        /// 
        /// If all three conditions are satisfied, the point is inside or on the triangle boundary,
        /// making the ear invalid (we don't want other vertices inside the ear we're about to clip).
        /// 
        /// </summary>
        /// <param name="index">First vertex index of the candidate ear triangle</param>
        /// <param name="next">Second vertex index of the candidate ear triangle</param>
        /// <param name="nextnext">Third vertex index of the candidate ear triangle</param>
        /// <returns>True if the ear is valid (no vertices inside), false otherwise</returns>
        private bool VertexStrictlyOutsideEarAabb(Scalar minX, Scalar maxX, Scalar minY, Scalar maxY, int j)
        {
            var pj = polygon[j];
            Scalar jx = arithmetic.Get(pj, 0);
            if (arithmetic.Compare(jx, minX) < 0 || arithmetic.Compare(jx, maxX) > 0)
                return true;

            Scalar jy = arithmetic.Get(pj, 1);
            return arithmetic.Compare(jy, minY) < 0 || arithmetic.Compare(jy, maxY) > 0;
        }

        private bool IsValid(int index, int next, int nextnext)
        {
            var p0 = polygon[index];
            var p1 = polygon[next];
            var p2 = polygon[nextnext];

            Scalar minX = arithmetic.Min(arithmetic.Min(arithmetic.Get(p0, 0), arithmetic.Get(p1, 0)), arithmetic.Get(p2, 0));
            Scalar maxX = arithmetic.Max(arithmetic.Max(arithmetic.Get(p0, 0), arithmetic.Get(p1, 0)), arithmetic.Get(p2, 0));
            Scalar minY = arithmetic.Min(arithmetic.Min(arithmetic.Get(p0, 1), arithmetic.Get(p1, 1)), arithmetic.Get(p2, 1));
            Scalar maxY = arithmetic.Max(arithmetic.Max(arithmetic.Get(p0, 1), arithmetic.Get(p1, 1)), arithmetic.Get(p2, 1));

            int j = index;
            for (int i = 0; i < count; ++i)
            {
                // Skip the three vertices that form the ear triangle itself
                if (j == index || j == next || j == nextnext)
                {
                    j = Next(j);
                    continue;
                }

                if (VertexStrictlyOutsideEarAabb(minX, maxX, minY, maxY, j))
                {
                    j = Next(j);
                    continue;
                }

                // Perform a complete point-in-triangle test by checking the vertex j
                // against all three edges of the triangle
                int o1 = Orient2D(index, next, j);
                int o2 = Orient2D(next, nextnext, j);
                int o3 = Orient2D(nextnext, index, j);

                // If j is on the left side (or collinear) with all three edges,
                // then j is inside or on the boundary of the triangle, making this ear invalid
                // Don't use >=. We allow the other point to lie on the surface - this is actually not an invalid config - think
                // of a donut shaped triangle ring which has a narrow passage where it only connects at a single vertex
                if (o1 >= 0 && o2 >= 0 && o3 >= 0)
                {
                    int zeroCounter = 0;
                    if (o1 == 0) ++zeroCounter;
                    if (o2 == 0) ++zeroCounter;
                    if (o3 == 0) ++zeroCounter;
                    if (zeroCounter == 3)
                        throw new Exception();
                    //Zero counter == 2 could mean a vertex used by multiple triangles that don't share an edge
                    if(zeroCounter==2)
                    {

                    }
                    if (zeroCounter < 2)
                        return false;
                }

                j = Next(j);
            }
            // Sanity check: after iterating through all vertices, we should be back at the start
            if (j != index)
                throw new Exception();
            return true;
        }
    }
}
