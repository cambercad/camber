using GeoCore;

namespace CSG
{
    /// <summary>Per-boolean counters for the pair-intersection phase. Printed after each Resolve.</summary>
    public sealed class ResolvePairStats
    {
        public int TrisA;
        public int TrisB;
        public long BvhHitsAvB;
        public long BvhHitsBvA;
        public long AabbRejects;
        public long PairsProcessed;
        public long Orient3DFast;
        public long Orient3DSlow;
        public long Orient3DFilterHit;
        public long Orient3DFilterMiss;
        public long SegTriAabbCulls;
        public long SegTriFull;
        public long CoplanarHits;
        public long IntersectHits;
        public long ResolveAvBMs;
        public long ResolveBvAMs;

        public override string ToString()
        {
            return string.Format(
                "  ResolveDetail trisA={0} trisB={1} bvhAvB={2} bvhBvA={3} aabbReject={4} pairs={5} " +
                "segTriCull={6} segTriFull={7} intersect={8} coplanar={9} orientFast={10} orientSlow={11} " +
                "orientFilt={12} orientFiltMiss={13} AvB={14}ms BvA={15}ms",
                TrisA, TrisB, BvhHitsAvB, BvhHitsBvA, AabbRejects, PairsProcessed,
                SegTriAabbCulls, SegTriFull, IntersectHits, CoplanarHits, Orient3DFast, Orient3DSlow,
                Orient3DFilterHit, Orient3DFilterMiss, ResolveAvBMs, ResolveBvAMs);
        }
    }

    public static class Resolver
    {
        /// <summary>When true, each boolean prints a name line plus resolver timings and pair stats.</summary>
        public static bool LogBooleanOps;
        public static ResolvePairStats LastResolveStats;
        private static void AddIntersectionAvoidDuplicates(
            ref Rat3Hybrid ip0, ref Rat3Hybrid ip1, ref int ipCount, in Rat3Hybrid p)
        {
            if (ipCount >= 1 && ip0 == p)
                return;
            if (ipCount >= 2 && ip1 == p)
                return;
            if (ipCount == 0)
            {
                ip0 = p;
                ipCount = 1;
                return;
            }
            if (ipCount == 1)
            {
                ip1 = p;
                ipCount = 2;
                return;
            }
            throw new Exception();
        }

        private static bool OverlapOrTouch(in TriPoints t1, in TriPoints t2)
        {
            Rat3Hybrid min, max;
            min = max = t1.A;
            if (t1.B.X > max.X) max.X = t1.B.X; if (t1.B.Y > max.Y) max.Y = t1.B.Y; if (t1.B.Z > max.Z) max.Z = t1.B.Z;
            if (t1.B.X < min.X) min.X = t1.B.X; if (t1.B.Y < min.Y) min.Y = t1.B.Y; if (t1.B.Z < min.Z) min.Z = t1.B.Z;

            if (t1.C.X > max.X) max.X = t1.C.X; if (t1.C.Y > max.Y) max.Y = t1.C.Y; if (t1.C.Z > max.Z) max.Z = t1.C.Z;
            if (t1.C.X < min.X) min.X = t1.C.X; if (t1.C.Y < min.Y) min.Y = t1.C.Y; if (t1.C.Z < min.Z) min.Z = t1.C.Z;


            Rat3Hybrid boundsMin, boundsMax;
            boundsMin = boundsMax = t2.A;
            if (t2.B.X > boundsMax.X) boundsMax.X = t2.B.X; if (t2.B.Y > boundsMax.Y) boundsMax.Y = t2.B.Y; if (t2.B.Z > boundsMax.Z) boundsMax.Z = t2.B.Z;
            if (t2.B.X < boundsMin.X) boundsMin.X = t2.B.X; if (t2.B.Y < boundsMin.Y) boundsMin.Y = t2.B.Y; if (t2.B.Z < boundsMin.Z) boundsMin.Z = t2.B.Z;

            if (t2.C.X > boundsMax.X) boundsMax.X = t2.C.X; if (t2.C.Y > boundsMax.Y) boundsMax.Y = t2.C.Y; if (t2.C.Z > boundsMax.Z) boundsMax.Z = t2.C.Z;
            if (t2.C.X < boundsMin.X) boundsMin.X = t2.C.X; if (t2.C.Y < boundsMin.Y) boundsMin.Y = t2.C.Y; if (t2.C.Z < boundsMin.Z) boundsMin.Z = t2.C.Z;

            return !(max.X < boundsMin.X || min.X > boundsMax.X || max.Y < boundsMin.Y || min.Y > boundsMax.Y || max.Z < boundsMin.Z || min.Z > boundsMax.Z);
        }

        private static bool ProcessCoplanarSegmentVsTriangle(in Rat3Hybrid segmentStart, in Rat3Hybrid segmentEnd,
            PlaneConvexPolygon poly, /*ResolverTriangle triABC,*/ out Rat3LineSegment clipped)
        {
            //Trim the segment by the triangle's side planes
            //If the resulting segment is valid, add two points to the intersectionPoints

            Rat3LineSegment seg = new Rat3LineSegment(segmentStart, segmentEnd);

            IntersectionResult r = poly.GetIntersectionCoplanar(seg, out clipped);
            if (r == IntersectionResult.LineSegment)
            {
                //triABC.AddSegment(clipped);
                return true;
            }
            return false;
        }

        public static void ProcessTrianglePair(NewPointCreator points, ResolverTriangle tri1, ResolverTriangle tri2, int lockId = -1) =>
            ProcessTrianglePair(points, tri1, tri2, lockId, null);

        private static void ProcessTrianglePair(NewPointCreator points, ResolverTriangle tri1, ResolverTriangle tri2, int lockId, ResolvePairStats stats)
        {
            bool specialCoplanarTriangleTreatment = true;

            //Fetch the points
            TriPoints tr1;
            tr1.A = points.GetPoint(tri1.A);
            tr1.B = points.GetPoint(tri1.B);
            tr1.C = points.GetPoint(tri1.C);
            tr1.CoplanarEdgeInfo = tri1.CoplanarEdgeInfo;
            TriPoints tr2;
            tr2.A = points.GetPoint(tri2.A);
            tr2.B = points.GetPoint(tri2.B);
            tr2.C = points.GetPoint(tri2.C);
            tr2.CoplanarEdgeInfo = tri2.CoplanarEdgeInfo;

            //The bounds check is already done by the tree traversal
            if (!OverlapOrTouch(tr1, tr2)/*tri1.GetBounds(points).OverlapOrTouch(tri2.GetBounds(points))*/)
            {
                if (stats != null)
                    Interlocked.Increment(ref stats.AabbRejects);
                return;
            }
            if (stats != null)
                Interlocked.Increment(ref stats.PairsProcessed);

            Rat3Hybrid ip0 = default;
            Rat3Hybrid ip1 = default;
            int ipCount = 0;

            ProcessEdgesVsTriangle(tr1, tri2, tr2, lockId != 2, specialCoplanarTriangleTreatment, ref ip0, ref ip1, ref ipCount, stats);
            ProcessEdgesVsTriangle(tr2, tri1, tr1, lockId != 1, specialCoplanarTriangleTreatment, ref ip0, ref ip1, ref ipCount, stats);

            if(ipCount == 1)
            {
                //This point must lie on the boundary of both triangles OR (see below)
                var pt = ip0;
                var aa = tri1.CoplanarPlanePolygon.PointIsOnBoundary(pt, out _, out _, true);
                var bb = tri2.CoplanarPlanePolygon.PointIsOnBoundary(pt, out _, out _, true);

                // OR it is a corner of one triangle that is exaclty somewhere on the other triangle (including the boundaries) - single point touch
                bool ptIsCornerOfTri1 = pt == tr1.A || pt == tr1.B || pt == tr1.C;
                bool ptIsCornerOfTri2 = pt == tr2.A || pt == tr2.B || pt == tr2.C;

                bool ptOnTri1 = tri1.CoplanarPlanePolygon.PointIsInsideOrOnBoundary(pt, out _, true);
                bool ptOnTri2 = tri2.CoplanarPlanePolygon.PointIsInsideOrOnBoundary(pt, out _, true);

                bool case1Met = aa && bb;
                //bool case2Met = ptIsCornerOfTri1 && ptIsCornerOfTri2; //Redundant, covered by cases below
                bool case3Met = ptIsCornerOfTri1 && ptOnTri2;
                bool case4Met = ptIsCornerOfTri2 && ptOnTri1;

                if (!case1Met && !case3Met && !case4Met)
                    throw new Exception();

                //tri1.addPoint();
                //tri2.addPoint();
                if (lockId != 1)
                    tri1.AddPoint(pt);
                if (lockId != 2)
                    tri2.AddPoint(pt);
            }

            if (ipCount == 2)
            {
                if (lockId != 1)
                    tri1.AddSegment(new PointPair(ip0, ip1));
                if (lockId != 2)
                    tri2.AddSegment(new PointPair(ip0, ip1));
            }
        }

        private static void ProcessEdgesVsTriangle(
            in TriPoints edgeTri,
            ResolverTriangle resolverTri,
            in TriPoints triPointsFixed,
            bool write,
            bool specialCoplanarTriangleTreatment,
            ref Rat3Hybrid ip0,
            ref Rat3Hybrid ip1,
            ref int ipCount,
            ResolvePairStats stats)
        {
            for (int i = 0; i < 3; ++i)
            {
                Rat3Hybrid intersectionPoint;
                bool onBoundary, startIsOnTriangle, endIsOnTriangle;
                SegmentTriangleIntersectionType type = TriangleSegmentIntersector.SegmentIntersectsTriangle(
                    edgeTri[i], edgeTri[(i + 1) % 3],
                    triPointsFixed.A, triPointsFixed.B, triPointsFixed.C,
                    out intersectionPoint, out onBoundary, out startIsOnTriangle, out endIsOnTriangle, stats);
                if (type == SegmentTriangleIntersectionType.Coplanar)
                {
                    if (stats != null)
                        Interlocked.Increment(ref stats.CoplanarHits);
                    if (!write)
                        continue;
                    PlaneConvexPolygon poly2 = resolverTri.CoplanarPlanePolygon;

                    Rat3LineSegment seg;
                    bool hasSegment = ProcessCoplanarSegmentVsTriangle(edgeTri[i], edgeTri[(i + 1) % 3], poly2, out seg);
                    if (hasSegment)
                    {
                        if (specialCoplanarTriangleTreatment)
                        {
                            bool onCorner;
                            int sideIndex;
                            if (poly2.PointIsOnBoundary(seg.GetStartPoint(), out onCorner, out sideIndex))
                            {
                                if (!onCorner && !triPointsFixed.IsCoplanarEdge(sideIndex))
                                    resolverTri.AddSegment(new PointPair(seg.GetStartPoint(), seg.GetStartPoint()));
                            }
                            if (poly2.PointIsOnBoundary(seg.GetEndPoint(), out onCorner, out sideIndex))
                            {
                                if (!onCorner && !triPointsFixed.IsCoplanarEdge(sideIndex))
                                    resolverTri.AddSegment(new PointPair(seg.GetEndPoint(), seg.GetEndPoint()));
                            }
                        }
                        else
                            resolverTri.AddSegment(seg);
                    }
                }
                else if (type == SegmentTriangleIntersectionType.Intersect)
                {
                    if (stats != null)
                        Interlocked.Increment(ref stats.IntersectHits);
                    AddIntersectionAvoidDuplicates(ref ip0, ref ip1, ref ipCount, in intersectionPoint);
                }
            }
        }

        private static List<Tri> WithoutInvalidTriangles(List<Tri> triangles)
        {
            // Deferred adjacency queries still address the original source
            // indices. Compact into a new list only when invalid faces exist.
            return triangles.Any(t => t.A < 0)
                ? triangles.Where(t => t.A >= 0).ToList() : triangles;
        }

        private static Rat3Hybrid GetNormal(Tri triangle, NewPointCreator newPoints)
        {
            var a = newPoints.GetPoint(triangle.A);
            var b = newPoints.GetPoint(triangle.B);
            var c = newPoints.GetPoint(triangle.C);
            return Rat3Hybrid.Cross(b - a, c - a);
        }

        private static bool TrianglesAreCoplanar(int triId1, int triId2, List<Tri> triangles,
            NewPointCreator newPoints)
        {
            if (triId1 < 0 || triId2 < 0)
                return false;

            var left = GetNormal(triangles[triId1], newPoints);
            var right = GetNormal(triangles[triId2], newPoints);
            return Rat3Hybrid.Cross(left, right) == Rat3Hybrid.Zero;
        }

        private static int TrianglesAreCoplanarEncoded(int i, TriangleAdjacency adj, List<Tri> triangles, NewPointCreator newPoints)
        {
            int coplanarEdgeInfo = 0;
            if(TrianglesAreCoplanar(i, adj.NeighbourAB, triangles, newPoints))
                coplanarEdgeInfo |= 1;
            if (TrianglesAreCoplanar(i, adj.NeighbourBC, triangles, newPoints))
                coplanarEdgeInfo |= 2;
            if (TrianglesAreCoplanar(i, adj.NeighbourCA, triangles, newPoints))
                coplanarEdgeInfo |= 4;
            return coplanarEdgeInfo;
        }

        private static Tri FlipOrientation(Tri tri)
        {
            return new Tri(tri.A, tri.C, tri.B);
        }

        private static bool ProcessTriangle(Tri triIn, NewPointCreator newPoints, List<Tri> volumeTriangles, BVHNode[] volumeTree,
            Box3I volumeBounds, bool triangleFromMeshA, BoolSettings s, out Tri triOut, out InsideResult location)
        {
            triOut = new Tri(-1, -1, -1);
            Int3Intersector intersector = new Int3Intersector(newPoints, volumeTriangles);
            location = PointInMesh.PointInsideMesh(newPoints.GetPoint(triIn.A), newPoints.GetPoint(triIn.B), newPoints.GetPoint(triIn.C),
                /*volumePoints, volumeTriangles*/intersector, volumeTree, volumeBounds);

            //var location = PointInMesh.PointInsideMesh(newPoints.GetPlanePoint(triIn.A), newPoints.GetPlanePoint(triIn.B), newPoints.GetPlanePoint(triIn.C),
            //    volumePoints, volumeTriangles, volumeTree, volumeBounds);

            bool inside = location == InsideResult.Inside;
            if (triangleFromMeshA)
            {
                if (location == InsideResult.Inside || location == InsideResult.Outside)
                {
                    if (inside == s.keepInside1 || s.keepAll)
                    {
                        if (s.flipTriOrientation1)
                            triOut = FlipOrientation(triIn);
                        else
                            triOut = triIn;
                        return true;
                    }
                    else
                        return false;
                }
                else
                {
                    //Coplanar
                    bool identicalNormals = location == InsideResult.CoplanarSameNormal;
                    if (identicalNormals == s.keepCoplanarIfSameNormal || s.keepAll)
                    {
                        if (s.flipTriOrientation1)
                            triOut = FlipOrientation(triIn);
                        else
                            triOut = triIn;
                        return true;
                    }
                    else if (!identicalNormals == s.keepCoplanarIfOppositeNormal || s.keepAll)
                    {
                        if (s.flipTriOrientation1)
                            triOut = FlipOrientation(triIn);
                        else
                            triOut = triIn;
                        return true;
                    }
                    else
                        return false;
                }
            }
            else
            {
                if (location == InsideResult.Inside || location == InsideResult.Outside)
                {
                    if (inside == s.keepInside2 || s.keepAll)
                    {
                        if (s.flipTriOrientation2)
                            triOut = FlipOrientation(triIn);
                        else
                            triOut = triIn;
                        return true;
                    }
                    else
                        return false;
                }
                else
                {
                    //Keep only coplanar triangles of mesh A - otherwise there would be duplicate triangles
                    return false;
                }
            }
        }

        public static List<ResolverTriangle> ResolveDebug()
        {
            return Resolve(BooleanOp.Resolve, new List<Rat3Hybrid>() { new Rat3Hybrid(0,0,0), new Rat3Hybrid(1,0,0), new Rat3Hybrid(1,3,0) }, 
                new List<Tri>() { new Tri(0,1,2) },
                new List<Rat3Hybrid>() { new Rat3Hybrid(0, 0, 0), new Rat3Hybrid(3, 0, 0), new Rat3Hybrid(3, 1, 0) }, new List<Tri>() { new Tri(0, 1, 2) }, 
                out var x, out var y, out var z);
        }

        public static List<ResolverTriangle> Resolve(BooleanOp op, List<Rat3Hybrid> pointsA, List<Tri> trianglesA, List<Rat3Hybrid> pointsB, List<Tri> trianglesB,
            out List<Rat3Hybrid> resultPoints, out List<Tri> resultTriangles, out List<SourceTriangle> sourceTriangleIndex,
            List<int> map = null, List<List<IntersectionSegmentEx>> intersectionStrips = null,
            Action<BooleanFragments> classifiedFragments = null)
        {
            NewPointCreator newPoints = new NewPointCreator();
            int[] mapA = new int[pointsA.Count];
            for (int i = 0; i < pointsA.Count; ++i)
            {
                bool isDuplicate;
                mapA[i] = newPoints.GetIndex(pointsA[i], out isDuplicate);             

                if (map != null)
                    map.Add(mapA[i]);
            }

            int[] mapB = new int[pointsB.Count];
            for (int i = 0; i < pointsB.Count; ++i)
            {
                bool isDuplicate;
                mapB[i] = newPoints.GetIndex(pointsB[i], out isDuplicate);        

                if (map != null)
                    map.Add(mapB[i]);
            }

            newPoints.EndInitialize();

            var mappedTrianglesA = GetMapped(mapA, trianglesA);
            var mappedTrianglesB = GetMapped(mapB, trianglesB);

            var res = Resolve(op, newPoints, mappedTrianglesA, mappedTrianglesB, out resultPoints, out resultTriangles, 
                out sourceTriangleIndex, intersectionStrips, classifiedFragments);

            return res;
        }

        private static List<Tri> GetMapped(int[] map, List<Tri> input)
        {
            List<Tri> output = new List<Tri>(input.Count);
            for (int i = 0; i < input.Count; ++i)
            {
                var t = input[i];
                t = new Tri(map[t.A], map[t.B], map[t.C]);
                if (t.ContainsDuplicateIndex())
                    t = new Tri(-1, -1, -1);
                output.Add(t);
            }
            return output;
        }

        private static List<ResolverTriangle> Resolve(BooleanOp op, NewPointCreator newPoints, List<Tri> trianglesA, List<Tri> trianglesB,
            out List<Rat3Hybrid> resultPoints, out List<Tri> resultTrianglesOut, out List<SourceTriangle> sourceTriangleIndexOut,
            List<List<IntersectionSegmentEx>> intersectionStrips = null, Action<BooleanFragments> classifiedFragments = null)
        {
            Timing timing = new Timing();

            bool useAdj = true;

            TriangleAdjacency[] adjA = useAdj ? AdjacencyEx.BuildAdjacencyInformation(newPoints.GetPoints(), trianglesA) : null;
            TriangleAdjacency[] adjB = useAdj ? AdjacencyEx.BuildAdjacencyInformation(newPoints.GetPoints(), trianglesB) : null;

            var tim = timing.Start("ResolverTriangle");
            List<ResolverTriangle> resolverTris = new List<ResolverTriangle>(trianglesA.Count + trianglesB.Count);
            for (int i = 0; i < trianglesA.Count; ++i)
            {
                var t = trianglesA[i];
                if (t.A < 0)
                    continue;

                //var adj = useAdj ? adjA[i] : new TriangleAdjacency();
                resolverTris.Add(new ResolverTriangle(t.A, t.B, t.C, i, newPoints/*,
                    false, false, false*/
                    /*useAdj && TrianglesAreCoplanar(i, adj.NeighbourAB, trianglesA, newPoints),
                    useAdj && TrianglesAreCoplanar(i, adj.NeighbourBC, trianglesA, newPoints),
                    useAdj && TrianglesAreCoplanar(i, adj.NeighbourCA, trianglesA, newPoints)*/));
            }
            int numValidTrisA = resolverTris.Count;
            for (int i = 0; i < trianglesB.Count; ++i)
            {
                var t = trianglesB[i];
                if (t.A < 0)
                    continue;

                //var adj = useAdj ? adjB[i] : new TriangleAdjacency();
                resolverTris.Add(new ResolverTriangle(t.A, t.B, t.C, i, newPoints/*,
                    false, false, false*/
                    /*useAdj && TrianglesAreCoplanar(i, adj.NeighbourAB, trianglesB, newPoints),
                    useAdj && TrianglesAreCoplanar(i, adj.NeighbourBC, trianglesB, newPoints),
                    useAdj && TrianglesAreCoplanar(i, adj.NeighbourCA, trianglesB, newPoints)*/));
            }
            int numValidTrisB = resolverTris.Count - numValidTrisA;

            var coplanarTrianglesA = trianglesA;
            var coplanarTrianglesB = trianglesB;
            if (useAdj)
            {
                ParallelEx.For(0, resolverTris.Count, rId =>
                {
                    int i = resolverTris[rId].Source;
                   
                    if (rId < numValidTrisA)
                    {
                        var adj = adjA[i];
                        resolverTris[rId].SetCoplanarEdgeInfo(
                            delegate ()
                            {
                                return TrianglesAreCoplanarEncoded(i, adj, coplanarTrianglesA, newPoints);
                            }
                            //TrianglesAreCoplanar(i, adj.NeighbourAB, trianglesA, newPoints),
                            //TrianglesAreCoplanar(i, adj.NeighbourBC, trianglesA, newPoints),
                            //TrianglesAreCoplanar(i, adj.NeighbourCA, trianglesA, newPoints)
                            );
                    }
                    else
                    {
                        var adj = adjB[i];
                        resolverTris[rId].SetCoplanarEdgeInfo(
                            delegate()
                            {
                                return TrianglesAreCoplanarEncoded(i, adj, coplanarTrianglesB, newPoints);
                            }
                            //TrianglesAreCoplanar(i, adj.NeighbourAB, trianglesB, newPoints),
                            //TrianglesAreCoplanar(i, adj.NeighbourBC, trianglesB, newPoints),
                            //TrianglesAreCoplanar(i, adj.NeighbourCA, trianglesB, newPoints)
                            );
                    }
                });
            }

            tim.Stop();

            tim = timing.Start("RemoveInvalid");
            //Return the invalid triangles
            trianglesA = WithoutInvalidTriangles(trianglesA);
            trianglesB = WithoutInvalidTriangles(trianglesB);
            tim.Stop();

            if (resolverTris.Count != trianglesA.Count + trianglesB.Count)
                throw new Exception();

            BVHNode[] treeA = null;
            BVHNode[] treeB = null;
            Box3I boundingBoxA = Box3I.Empty();
            Box3I boundingBoxB = Box3I.Empty();
            tim = timing.Start("BuildTree");
         
            Box3F[] allBounds = new Box3F[resolverTris.Count];
            Box3I[] allBoundsExact = new Box3I[resolverTris.Count];
            ParallelEx.For(0, resolverTris.Count, i =>
            {
                var ib = resolverTris[i].GetBounds(newPoints);
                allBoundsExact[i] = ib;
                allBounds[i] = ib.GetBoundsF();
            });

            for (int i = 0; i < resolverTris.Count; ++i)
            {
                if (i < numValidTrisA)
                    boundingBoxA.Extend(allBoundsExact[i]);
                else
                    boundingBoxB.Extend(allBoundsExact[i]);
            }

            treeA = Tree.BuildTreeFast(delegate (int i)
            {
                return allBounds[i];
            },
            numValidTrisA);
            treeB = Tree.BuildTreeFast(delegate (int i)
            {
                return allBounds[i + numValidTrisA];
            },
            resolverTris.Count - numValidTrisA);
            tim.Stop();


            var pairStats = new ResolvePairStats { TrisA = numValidTrisA, TrisB = numValidTrisB };
            // One traversal: test each overlapping pair once and write both sides.
            // A is the outer index so each A triangle is written by only one worker;
            // B writes are serialized inside ResolverTriangle.AddSegment.
            tim = timing.Start("ResolveAvB");
            ParallelEx.ForLoadBalanced(0, numValidTrisA, i =>
            {
                TreeBoxOverlapEnumerator overlaps = new TreeBoxOverlapEnumerator(treeB, allBounds[i]);
                foreach (var j in overlaps)
                {
                    Interlocked.Increment(ref pairStats.BvhHitsAvB);
                    ProcessTrianglePair(newPoints, resolverTris[i], resolverTris[j + numValidTrisA], -1, pairStats);
                }
            });
            tim.Stop();
            pairStats.ResolveAvBMs = tim.MillisecondsEnd - tim.MillisecondsStart;
            pairStats.ResolveBvAMs = 0;

            //TODO: Don't just test a against b, test everything against everything - this might handle self-intersections automatically
            /*for (int i = 0; i < numValidTrisA; ++i)
            {
                for (int j = numValidTrisA; j < result.Count; ++j)
                {
                    ProcessTrianglePair(newPoints.Points, result[i], result[j]);
                }
            }*/

            BoolSettings boolSettings = new BoolSettings(op);

            tim = timing.Start("PrepareTriangulate");
            ResolverTriangle.ArrangeTrimSegments(resolverTris, newPoints);
            Dictionary<long, int> insertedSegmentsA = new Dictionary<long, int>();
            for (int i = 0; i < numValidTrisA; ++i)
                resolverTris[i].PrepareTriangulate(newPoints, insertedSegmentsA);
            Dictionary<long, int> insertedSegmentsB = new Dictionary<long, int>();
            for (int i = numValidTrisA; i < resolverTris.Count; ++i)
                resolverTris[i].PrepareTriangulate(newPoints, insertedSegmentsB);

       

            if (intersectionStrips != null)
            {
                foreach (var segment in insertedSegmentsB)
                {
                    int a, b;
                    Algorithms.DecomposeKey(segment.Key, out a, out b);
                    if (a == b)
                        continue; //Skip points - they are relevant for CSG but not for the intersection curve

                    int triAId;
                    if (!insertedSegmentsA.TryGetValue(segment.Key, out triAId))
                    {
                        throw new Exception();
                    }
                }

                List<IntersectionSegment> allInsertedSegments = new List<IntersectionSegment>(insertedSegmentsA.Count);
                foreach (var segment in insertedSegmentsA)
                {
                    int a, b;
                    Algorithms.DecomposeKey(segment.Key, out a, out b);
                    if (a == b)
                        continue; //Skip points - they are relevant for CSG but not for the intersection curve

                    int triBId;
                    if (!insertedSegmentsB.TryGetValue(segment.Key, out triBId))
                    {
                        throw new Exception("Edge not found in insertedSegmentsB");
                    }
                    if (triBId < 0)
                        throw new Exception();

                    allInsertedSegments.Add(new IntersectionSegment(a, b, segment.Value, triBId));
                }

                intersectionStrips.Clear();

                if (allInsertedSegments.Count > 0)
                {

                    tim.Stop();

                    tim = timing.Start("SegmentConnector");

                    var tmp = SegmentConnector.Connect(allInsertedSegments, 
                        delegate (IntersectionSegment seg) { return seg.Start; }, 
                        delegate (IntersectionSegment seg) { return seg.End; }, delegate(int a, int b) { return a == b; }, out List<bool> closed);

                    // Populate intersectionStrips from tmp
                    for (int i = 0; i < tmp.Count; i++)
                    {
                        List<IntersectionSegmentEx> strip = new List<IntersectionSegmentEx>();

                        for (int j = 0; j < tmp[i].Count; j++)
                        {
                            var segWithDir = tmp[i][j];
                            var seg = allInsertedSegments[Math.Abs(segWithDir)];

                            bool reversedDir = segWithDir < 0;

                            // Create IntersectionSegmentEx with point data
                            IntersectionSegmentEx segEx = new IntersectionSegmentEx
                            {
                                Start = reversedDir ? seg.End : seg.Start,
                                End = reversedDir ? seg.Start : seg.End,
                                TriIdA = seg.TriIdA,
                                TriIdB = seg.TriIdB,
                                StartPoint = newPoints.GetPoint(reversedDir ? seg.End : seg.Start),
                                EndPoint = newPoints.GetPoint(reversedDir ? seg.Start : seg.End)
                            };

                            if (strip.Count > 0)
                            {
                                var debug = strip[strip.Count - 1];
                                if (segEx.Start != debug.End)
                                    throw new Exception();
                                if (segEx.StartPoint != debug.EndPoint)
                                    throw new Exception();
                            }

                            strip.Add(segEx);
                        }

                        //// If the strip is closed, add the first segment at the end to close the loop
                        //if (closed[i] && strip.Count > 0)
                        //{
                        //    strip.Add(strip[0]);
                        //}

                        intersectionStrips.Add(strip);
                    }
                }
            }

            

            tim.Stop();

            resultPoints = newPoints.ExportPoints();

            if (op == BooleanOp.NoOpIntersectionContourOnly)
            {
                resultTrianglesOut = null;
                sourceTriangleIndexOut = null;
                return resolverTris;
            }


            tim = timing.Start("Triangulate");
            var resultTriangles = new List<Tri>[resolverTris.Count];
            var sourceTriangleIndex = new List<SourceTriangle>();

            // This additional array should improve the load balancing of Parallel.For
            List<int> trianglesToProcess = new List<int>(Math.Max(4, resolverTris.Count / 10));
            for(int i=0;i<resolverTris.Count;++i)
            {
                var resolverTri = resolverTris[i];
                if (resolverTri.ContainsIntersections())
                    trianglesToProcess.Add(i);
                else
                    resultTriangles[i] = null;
            }

            //for (int i = 0; i < resolverTris.Count; ++i)
            ParallelEx.For(0, trianglesToProcess.Count, local =>
            {
                int i = trianglesToProcess[local];
                // Indices here already passed ContainsIntersections() when building trianglesToProcess.
                resultTriangles[i] = resolverTris[i].Triangulate(newPoints);
            });

            int changeTriId = -1;
            var resultTrianglesJoined = new List<Tri>();
            for (int i = 0; i < resultTriangles.Length; ++i)
            {
                if (i == numValidTrisA)
                    changeTriId = resultTrianglesJoined.Count;

                int start = resultTrianglesJoined.Count;
                var tmp = resultTriangles[i];
                var t = resolverTris[i];
                if (tmp == null)
                {
                    resultTrianglesJoined.Add(new Tri(t.A, t.B, t.C));
                }
                else
                    resultTrianglesJoined.AddRange(tmp);

                for (int j = start; j < resultTrianglesJoined.Count; ++j)
                    sourceTriangleIndex.Add(new SourceTriangle(changeTriId == -1 ? MeshOrigin.MeshA : MeshOrigin.MeshB, t.Source));
            }


            // Preserve transient source topology only for callers requesting provenance.
            var classificationTriangles = classifiedFragments == null ? null : resultTrianglesJoined.ToArray();
            var classificationSources = classifiedFragments == null ? null : sourceTriangleIndex.ToArray();

            bool isTrimSurfaceOperation = op == BooleanOp.AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection || 
                                          op == BooleanOp.AAsSurfaceBAsTrimSurfaceKeepInTriNormalDirection ||
                                          op == BooleanOp.AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection ||
                                          op == BooleanOp.AAsVolumeBAsTrimSurfaceKeepInTriNormalDirection;

            List<List<int>> clustersA = null;
            List<List<int>> clustersB = null;
            // Regular booleans classify per triangle. Clusters are required for trim-surface
            // ops; in DEBUG they also feed ValidateCluster (inside/outside consistency canary).
            bool needClusters = isTrimSurfaceOperation;
#if DEBUG
            needClusters = true;
#endif
            if (needClusters && insertedSegmentsA.Count > 0)
            {
                tim.Stop();
                tim = timing.Start("Clusterize");
                var insertedSegments = insertedSegmentsA.Keys.ToHashSet();

                Task a = new Task(delegate ()
                {
                    clustersA = Clusterize(newPoints.GetPoints(), insertedSegments, resultTrianglesJoined, 0, changeTriId);
                });
                a.Start();
                Task b = new Task(delegate ()
                {
                    clustersB = Clusterize(newPoints.GetPoints(), insertedSegments, resultTrianglesJoined, changeTriId, resultTrianglesJoined.Count);
                });
                b.Start();
                a.Wait();
                b.Wait();
            }

            bool isTrimVolumeOperation = op == BooleanOp.AAsSurfaceBAsTrimVolumeKeepInside ||
                                         op == BooleanOp.AAsSurfaceBAsTrimVolumeKeepOutside;

            if (!isTrimSurfaceOperation)
            {
                InsideResult[] insideResults = new InsideResult[resultTrianglesJoined.Count];

                tim.Stop();
                tim = timing.Start("InsideOutside");
                ParallelEx.For(0, resultTrianglesJoined.Count, i =>
                //for (int i = 0; i < resultTriangles.Count; ++i)
                {
                    var tri = resultTrianglesJoined[i];
                    bool b;
                    InsideResult r;
                    if (i < changeTriId)
                        b = ProcessTriangle(tri, newPoints, trianglesB, treeB, boundingBoxB, false, boolSettings, out tri, out r);
                    else
                    {
                        if (isTrimVolumeOperation)
                        {
                            b = false;
                            r = InsideResult.Unknown;
                        }
                        else
                            b = ProcessTriangle(tri, newPoints, trianglesA, treeA, boundingBoxA, true, boolSettings, out tri, out r);
                    }

                    if (!b)
                    {
                        var tmp = sourceTriangleIndex[i];
                        tmp.SourceTriangleIndex = -1;
                        sourceTriangleIndex[i] = tmp;
                    }
                    resultTrianglesJoined[i] = tri;
                    insideResults[i] = r;
                });



                if (clustersA != null)
                {
                    ValidateCluster(clustersA, insideResults);
                }
                if (clustersB != null)
                {
                    ValidateCluster(clustersB, insideResults);
                }
            }
            else // isTrimSurfaceOperation
            {
                // Trim surface code here
                tim.Stop();
                tim = timing.Start("TrimSurfaceOperation");
                
                // Determine operation parameters from enum
                bool keepInNormalDirection = op == BooleanOp.AAsSurfaceBAsTrimSurfaceKeepInTriNormalDirection ||
                                             op == BooleanOp.AAsVolumeBAsTrimSurfaceKeepInTriNormalDirection;
                bool meshAIsVolume = op == BooleanOp.AAsVolumeBAsTrimSurfaceKeepInTriNormalDirection ||
                                     op == BooleanOp.AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection;
                bool detectPartialCuts = true;// meshAIsVolume; // Enable partial cut detection for volumes
                
                if (clustersA == null || clustersA.Count == 0)
                {
                    // No clusters to process - no intersection occurred
                    // Keep all triangles from meshA
                    tim.Stop();
                }
                else
                {
                    var insertedSegments = insertedSegmentsA.Keys.ToHashSet();
                    
                    // Classify each cluster in meshA
                    bool[] clusterInNormalDirection = new bool[clustersA.Count];
                    var untouchedCavities = new List<List<int>>();
                    
                    for (int i = 0; i < clustersA.Count; i++)
                    {
                        var cluster = clustersA[i];
                        
                        // A closed inner shell can remain disconnected from the trim
                        // boundary. Its material owner is determined after the cut
                        // outer shell and cap have been selected.
                        bool touchesCut = cluster.Any(index => {
                            var t = resultTrianglesJoined[index];
                            return insertedSegments.Contains(Algorithms.Key(t.A,t.B)) ||
                                insertedSegments.Contains(Algorithms.Key(t.B,t.C)) ||
                                insertedSegments.Contains(Algorithms.Key(t.C,t.A));
                        });
                        if (!touchesCut)
                        {
                            clusterInNormalDirection[i] = keepInNormalDirection;
                            if (meshAIsVolume && ShellOrientation(cluster,resultTrianglesJoined,newPoints) < 0)
                                untouchedCavities.Add(cluster);
                            continue;
                        }

                        // Detect partial cuts if enabled
                        if (detectPartialCuts)
                        {
                            bool? isPartialCut = DetectPartialCut(
                                cluster,
                                resultTrianglesJoined,
                                insertedSegments,
                                insertedSegmentsB,
                                trianglesB,
                                newPoints,
                                resolverTris.GetRange(numValidTrisA, numValidTrisB));
                            
                            if (isPartialCut == true)
                            {
                                throw new Exception("Partial cut detected in cluster " + i + 
                                    ". Surface B does not completely cut through surface A.");
                            }
                            if (isPartialCut == null)
                            {
                                clusterInNormalDirection[i] = ClassifyCoincidentCluster(cluster,
                                    resultTrianglesJoined,insertedSegments,insertedSegmentsB,newPoints,
                                    resolverTris.GetRange(numValidTrisA,numValidTrisB));
                                continue;
                            }
                        }
                        
                        // Classify the cluster
                        clusterInNormalDirection[i] = ClassifyClusterInNormalDirection(
                            cluster,
                            resultTrianglesJoined,
                            insertedSegments,
                            insertedSegmentsB,
                            newPoints,
                            resolverTris.GetRange(numValidTrisA, numValidTrisB));
                    }
                    
                    // Mark triangles for removal based on classification
                    for (int i = 0; i < clustersA.Count; i++)
                    {
                        var cluster = clustersA[i];
                        bool inNormalDir = clusterInNormalDirection[i];
                        bool shouldKeep = (keepInNormalDirection && inNormalDir) || 
                                         (!keepInNormalDirection && !inNormalDir);
                        
                        if (!shouldKeep)
                        {
                            // Mark all triangles in this cluster for removal
                            foreach (int triIndex in cluster)
                            {
                                var tmp = sourceTriangleIndex[triIndex];
                                tmp.SourceTriangleIndex = -1;
                                sourceTriangleIndex[triIndex] = tmp;
                            }
                        }
                    }
                    
                    // If meshA is a surface (not a volume), remove all triangles from meshB
                    if (!meshAIsVolume)
                    {
                        // Mark all triangles from meshB for removal
                        for (int i = changeTriId; i < resultTrianglesJoined.Count; i++)
                        {
                            var tmp = sourceTriangleIndex[i];
                            tmp.SourceTriangleIndex = -1;
                            sourceTriangleIndex[i] = tmp;
                        }
                    }
                    else
                    {
                        // meshA is a volume - keep triangles from meshB to fill the cut
                        // Rule: Keep only triangles from B that face against the normal direction of adjacent triangles from A
                        // Additionally, flip B triangle winding if we're keeping the part in B-normal direction
                        if (clustersB != null && clustersB.Count > 0)
                        {
                            for (int i = 0; i < clustersB.Count; i++)
                            {
                                var clusterB = clustersB[i];
                                
                                // Classify B against the closed source volume, including exact coincident faces.
                                var probe=resultTrianglesJoined[clusterB[0]];
                                var location=PointInMesh.PointInsideMesh(newPoints.GetPoint(probe.A),
                                    newPoints.GetPoint(probe.B),newPoints.GetPoint(probe.C),
                                    new Int3Intersector(newPoints,trianglesA),treeA,boundingBoxA);
                                // Coincident A faces own the boundary once. The A
                                // normal determines whether its material side is kept.
                                bool shouldKeepCluster = (location is InsideResult.Inside or InsideResult.Outside) &&
                                    (location==InsideResult.Outside)==keepInNormalDirection;

                                if (!shouldKeepCluster)
                                {
                                    // Mark all triangles in this cluster for removal
                                    foreach (int triIndex in clusterB)
                                    {
                                        var tmp = sourceTriangleIndex[triIndex];
                                        tmp.SourceTriangleIndex = -1;
                                        sourceTriangleIndex[triIndex] = tmp;
                                    }
                                }
                                else
                                {
                                    // Keep this cluster, but check if we need to flip triangle orientation
                                    // If we're keeping the part in B-normal direction, flip the B triangles
                                    if (keepInNormalDirection)
                                    {
                                        foreach (int triIndex in clusterB)
                                        {
                                            //resultTrianglesJoined[triIndex] = FlipOrientation(resultTrianglesJoined[triIndex]);
                                            resultTrianglesJoined[triIndex] = resultTrianglesJoined[triIndex];
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    if (untouchedCavities.Count > 0)
                        ClassifyUntouchedCavities(untouchedCavities,resultTrianglesJoined,
                            sourceTriangleIndex,newPoints);
                    tim.Stop();
                }
            }


            classifiedFragments?.Invoke(new BooleanFragments(classificationTriangles, classificationSources,
                sourceTriangleIndex.Select(source => source.SourceTriangleIndex >= 0).ToArray()));
            RemoveMarkedTriangles(resultTrianglesJoined, sourceTriangleIndex, 0);
            tim.Stop();

            resultTrianglesOut = resultTrianglesJoined;
            sourceTriangleIndexOut = sourceTriangleIndex;

            //File.WriteAllText(@"C:\tmp\profile.txt", timing.ToString());
            if (LogBooleanOps)
            {
                Console.WriteLine(timing.ToString());
                Console.WriteLine(pairStats.ToString());
            }

            LastResolveStats = pairStats;
            return resolverTris;
        }

        private static int ShellOrientation(List<int> cluster,List<Tri> triangles,NewPointCreator points)
        {
            // First crossing from the exterior identifies shell orientation;
            // summing an exact mass property would accumulate unrelated rational
            // denominators over the entire mesh merely to obtain this sign.
            var minX=points.GetPoint(triangles[cluster[0]].A).X;
            Tri target=default;bool found=false;
            foreach(var index in cluster)
            {
                var t=triangles[index];
                var a=points.GetPoint(t.A);var b=points.GetPoint(t.B);var c=points.GetPoint(t.C);
                if(a.X<minX)minX=a.X;if(b.X<minX)minX=b.X;if(c.X<minX)minX=c.X;
                if(!found && Rat3Hybrid.Cross(b-a,c-a).X.Sign()!=0){target=t;found=true;}
            }
            if(!found)throw new InvalidOperationException("A volume shell has no face transverse to the X axis.");
            var ta=points.GetPoint(target.A);var tb=points.GetPoint(target.B);var tc=points.GetPoint(target.C);
            for(int sample=1;;sample++)
            {
                // This parabola of positive barycentric weights stays strictly
                // inside the target triangle. Each projected mesh edge can meet
                // it at most twice, so finite exact degeneracies are exhausted.
                var k=new BigRationalHybrid(sample);var k2=k*k;
                var end=(ta+tb*k+tc*k2)/(BigRationalHybrid.One+k+k2);
                end.Simplify();
                var start=new Rat3Hybrid(minX-BigRationalHybrid.One,end.Y,end.Z);
                BigRationalHybrid nearest=default;int orientation=0;bool boundary=false;bool coplanar=false;
                foreach(var index in cluster)
                {
                    var t=triangles[index];var a=points.GetPoint(t.A);var b=points.GetPoint(t.B);var c=points.GetPoint(t.C);
                    var hit=TriangleSegmentIntersector.SegmentIntersectsTriangle(start,end,a,b,c,
                        out var point,out bool onBoundary,out _,out _);
                    if(hit==SegmentTriangleIntersectionType.Coplanar){coplanar=true;continue;}
                    if(hit!=SegmentTriangleIntersectionType.Intersect)continue;
                    if(orientation==0 || point.X<nearest)
                    {nearest=point.X;orientation=-Rat3Hybrid.Cross(b-a,c-a).X.Sign();boundary=onBoundary;}
                    else if(point.X==nearest)boundary=true;
                }
                if(orientation!=0 && !boundary && !coplanar)return orientation;
            }
        }

        private static void ClassifyUntouchedCavities(List<List<int>> cavities,List<Tri> triangles,
            List<SourceTriangle> sources,NewPointCreator points)
        {
            var deferred=cavities.SelectMany(c => c).ToHashSet();
            var retained=triangles.Where((t,i) => sources[i].SourceTriangleIndex >= 0 && !deferred.Contains(i)).ToList();
            // Disjoint retained solids are separate material regions. Test their
            // union, not the parity of overlapping outer enclosures with their
            // still-deferred internal cavity surfaces missing.
            var components=Clusterize(points.GetPoints(),new HashSet<long>(),retained,0,retained.Count);
            var regions=new List<(List<Tri> mesh,BVHNode[] tree,Box3I bounds)>();
            foreach(var component in components)
            {
                if(ShellOrientation(component,retained,points) <= 0) continue;
                var mesh=component.Select(i => retained[i]).ToList();
                var bounds=Box3I.Empty();
                var boxes=new Box3F[mesh.Count];
                for(int i=0;i<mesh.Count;i++)
                {
                    var t=mesh[i];var box=points.GetPoint(t.A).GetBox();
                    box.Extend(points.GetPoint(t.B).GetBox());box.Extend(points.GetPoint(t.C).GetBox());
                    bounds.Extend(box);boxes[i]=box.GetBoundsF();
                }
                regions.Add((mesh,Tree.BuildTreeFast(i => boxes[i],mesh.Count),bounds));
            }
            foreach(var cavity in cavities)
            {
                var t=triangles[cavity[0]];
                bool contained=regions.Any(region => PointInMesh.PointInsideMesh(points.GetPoint(t.A),
                    points.GetPoint(t.B),points.GetPoint(t.C),new Int3Intersector(points,region.mesh),
                    region.tree,region.bounds)==InsideResult.Inside);
                if(!contained)
                    foreach(var i in cavity){var source=sources[i];source.SourceTriangleIndex=-1;sources[i]=source;}
            }
        }

        private static Rat3Hybrid ComputeTriangleCentroid(Tri triangle, NewPointCreator newPoints)
        {
            var a = newPoints.GetPoint(triangle.A);
            var b = newPoints.GetPoint(triangle.B);
            var c = newPoints.GetPoint(triangle.C);
            return (a + b + c) / new BigRationalHybrid(3);
        }

        private static int IsPointInNormalDirection(Rat3Hybrid centroid, Tri triangleB, NewPointCreator newPoints)
        {
            var a = newPoints.GetPoint(triangleB.A);
            var b = newPoints.GetPoint(triangleB.B);
            var c = newPoints.GetPoint(triangleB.C);
            
            // Compute normal of triangle B
            var normal = Rat3Hybrid.Cross(b - a, c - a);
            
            // Check if centroid is on positive side of plane defined by triangleB
            var toPoint = centroid - a;
            return Rat3Hybrid.DotSign(in toPoint, in normal);

            //return dotProduct.Sign() > 0;
        }

        private static bool ClassifyClusterInNormalDirection(
            List<int> cluster, 
            List<Tri> resultTrianglesJoined,
            HashSet<long> insertedSegments,
            Dictionary<long, int> insertedSegmentsOtherMesh,
            NewPointCreator newPoints,
            List<ResolverTriangle> resolverTrisOtherMesh)
        {
            // Find a triangle in the cluster that is adjacent to an inserted edge
            foreach (int triIndex in cluster)
            {
                Tri tri = resultTrianglesJoined[triIndex];
                
                // Check all three edges of this triangle
                long[] edges = new long[3];
                edges[0] = Algorithms.Key(tri.A, tri.B);
                edges[1] = Algorithms.Key(tri.B, tri.C);
                edges[2] = Algorithms.Key(tri.C, tri.A);
                
                foreach (long edgeKey in edges)
                {
                    if (insertedSegments.Contains(edgeKey))
                    {
                        // Found an inserted edge, find corresponding triangle from other mesh
                        int triOtherSource; // This is the Source field, not a direct index
                        if (insertedSegmentsOtherMesh.TryGetValue(edgeKey, out triOtherSource))
                        {
                            // Find the ResolverTriangle with matching Source
                            ResolverTriangle resolverTriOther = null;
                            foreach (var rt in resolverTrisOtherMesh)
                            {
                                if (rt.Source == triOtherSource)
                                {
                                    resolverTriOther = rt;
                                    break;
                                }
                            }
                            
                            if (resolverTriOther == null)
                                throw new Exception($"ResolverTriangle with Source={triOtherSource} not found in other mesh");
                            
                            // Get the original triangle from the other mesh (before subdivision)
                            Tri triangleOther = new Tri(resolverTriOther.A, resolverTriOther.B, resolverTriOther.C);
                            
                            var centroid = ComputeTriangleCentroid(tri, newPoints);
                            
                            int result = IsPointInNormalDirection(centroid, triangleOther, newPoints);

                            if (result == 0)
                                continue; // Skip coplanar

                            return result > 0;
                        }
                        else
                        {
                            throw new Exception("Inserted edge not found in insertedSegmentsOtherMesh");
                        }
                    }
                }
            }
            
            throw new Exception("No triangle adjacent to an inserted edge found in cluster");
        }

        private static bool ClassifyCoincidentCluster(List<int> cluster,List<Tri> triangles,
            HashSet<long> cuts,Dictionary<long,int> otherCuts,NewPointCreator points,List<ResolverTriangle> others)
        {
            foreach(var index in cluster)
            {
                var t=triangles[index];
                foreach(var edge in new[]{Algorithms.Key(t.A,t.B),Algorithms.Key(t.B,t.C),Algorithms.Key(t.C,t.A)})
                {
                    if(!cuts.Contains(edge) || !otherCuts.TryGetValue(edge,out var source))continue;
                    var other=others.FirstOrDefault(r => r.Source==source) ??
                        throw new InvalidOperationException($"Trim contact source triangle {source} was not found.");
                    var a=points.GetPoint(other.A);var normal=Rat3Hybrid.Cross(points.GetPoint(other.B)-a,points.GetPoint(other.C)-a);
                    bool planar=cluster.All(i => {
                        var q=triangles[i];
                        return Rat3Hybrid.Dot(points.GetPoint(q.A)-a,normal).Sign()==0 &&
                            Rat3Hybrid.Dot(points.GetPoint(q.B)-a,normal).Sign()==0 &&
                            Rat3Hybrid.Dot(points.GetPoint(q.C)-a,normal).Sign()==0;
                    });
                    if(!planar)continue;
                    var own=Rat3Hybrid.Cross(points.GetPoint(t.B)-points.GetPoint(t.A),points.GetPoint(t.C)-points.GetPoint(t.A));
                    return Rat3Hybrid.Dot(own,normal).Sign()<0;
                }
            }
            throw new InvalidOperationException("Trim contact has no transverse side and is not a coincident planar region.");
        }

        // null means all boundary evidence is coplanar; the caller must assign
        // coincident-face ownership, rather than treating this as an uncut region.
        private static bool? DetectPartialCut(
            List<int> cluster,
            List<Tri> resultTrianglesJoined,
            HashSet<long> insertedSegments,
            Dictionary<long, int> insertedSegmentsOtherMesh,
            List<Tri> trianglesOtherMesh,
            NewPointCreator newPoints,
            List<ResolverTriangle> resolverTrisOtherMesh)
        {
            // Check all triangles adjacent to inserted edges in the cluster
            // If they give inconsistent results, it's a partial cut
            int? firstResult = null;
            
            foreach (int triIndex in cluster)
            {
                Tri tri = resultTrianglesJoined[triIndex];
                
                // Check all three edges of this triangle
                long[] edges = new long[3];
                edges[0] = Algorithms.Key(tri.A, tri.B);
                edges[1] = Algorithms.Key(tri.B, tri.C);
                edges[2] = Algorithms.Key(tri.C, tri.A);
                
                foreach (long edgeKey in edges)
                {
                    if (insertedSegments.Contains(edgeKey))
                    {
                        // Found an inserted edge, check if in normal direction
                        int triOtherSource; // This is the Source field, not a direct index
                        if (insertedSegmentsOtherMesh.TryGetValue(edgeKey, out triOtherSource))
                        {
                            // Find the ResolverTriangle with matching Source
                            ResolverTriangle resolverTriOther = null;
                            foreach (var rt in resolverTrisOtherMesh)
                            {
                                if (rt.Source == triOtherSource)
                                {
                                    resolverTriOther = rt;
                                    break;
                                }
                            }
                            
                            if (resolverTriOther == null)
                                throw new Exception($"ResolverTriangle with Source={triOtherSource} not found in other mesh");
                            
                            Tri triangleOther = new Tri(resolverTriOther.A, resolverTriOther.B, resolverTriOther.C);
                            
                            var centroid = ComputeTriangleCentroid(tri, newPoints);
                            int inNormalDir = IsPointInNormalDirection(centroid, triangleOther, newPoints);

                            if (inNormalDir != 0) // Exclude coplanar
                            {
                                if (firstResult == null)
                                {
                                    firstResult = inNormalDir;
                                }
                                else if (firstResult.Value != inNormalDir)
                                {
                                    // Inconsistency detected - partial cut
                                    return true;
                                }
                            }
                            else
                            {

                            }
                        }
                    }
                }
            }

            if (firstResult == null)
                return null;
            
            return false; // No inconsistency found
        }

        private static void ValidateCluster(List<List<int>> clusters, InsideResult[] insideResults)
        {
            //All triangles in a cluster must have the same insideResult
            for (int i = 0; i < clusters.Count; ++i)
            {
                var c = clusters[i];
                List<InsideResult> tmp = new List<InsideResult>(c.Count);

                for (int j = 0; j < c.Count; ++j)
                {
                    tmp.Add(insideResults[c[j]]);
                }


                var reference = tmp[0];
                for (int j = 1; j < tmp.Count; ++j)
                {
                    if (tmp[j] != reference)
                        throw new Exception();
                }
            }
        }

        public static List<List<int>> Clusterize(List<Rat3Hybrid> points, HashSet<long> insertedSegmentsAsBoundaries, List<Tri> resultTrianglesJoined, 
            int triStartInclusive, int triEndExclusive)
        {
            // Extract the triangles in the specified range
            List<Tri> tris = new List<Tri>(triEndExclusive - triStartInclusive);
            for (int i = triStartInclusive; i < triEndExclusive; i++)
            {
                tris.Add(resultTrianglesJoined[i]);
            }

            // Clusterize the subset of triangles
            List<List<int>> clusters = Clusterize(points, insertedSegmentsAsBoundaries, tris);
            
            // Offset the result indices back to the original triangle list indices
            for (int i = 0; i < clusters.Count; i++)
            {
                for (int j = 0; j < clusters[i].Count; j++)
                {
                    clusters[i][j] += triStartInclusive;
                }
            }
            
            return clusters;
        }

        public static List<List<int>> Clusterize(List<Rat3Hybrid> points, HashSet<long> insertedSegmentsAsBoundaries, List<Tri> resultTrianglesJoined)
        {
            //Use flood fill but cannot go over insertedSegmentsAsBoundaries
            
            // Build adjacency information for all triangles
            TriangleAdjacency[] adjacency = AdjacencyEx.BuildAdjacencyInformation(points, resultTrianglesJoined);
            
            // Track which triangles have been visited
            bool[] visited = new bool[resultTrianglesJoined.Count];
            
            // Result clusters
            List<List<int>> clusters = new List<List<int>>();
            
            // Process each triangle
            for (int i = 0; i < resultTrianglesJoined.Count; i++)
            {
                if (visited[i])
                    continue;
                    
                // Start a new cluster with BFS flood fill
                List<int> cluster = new List<int>();
                Queue<int> queue = new Queue<int>();
                
                queue.Enqueue(i);
                visited[i] = true;
                
                while (queue.Count > 0)
                {
                    int currentTriIndex = queue.Dequeue();
                    cluster.Add(currentTriIndex);
                    
                    Tri currentTri = resultTrianglesJoined[currentTriIndex];
                    TriangleAdjacency adj = adjacency[currentTriIndex];
                    
                    // Check all three neighbors
                    // Neighbor across edge AB
                    if (adj.NeighbourAB >= 0 && !visited[adj.NeighbourAB])
                    {
                        long edgeKey = Algorithms.Key(currentTri.A, currentTri.B);
                        if (!insertedSegmentsAsBoundaries.Contains(edgeKey))
                        {
                            visited[adj.NeighbourAB] = true;
                            queue.Enqueue(adj.NeighbourAB);
                        }
                    }
                    
                    // Neighbor across edge BC
                    if (adj.NeighbourBC >= 0 && !visited[adj.NeighbourBC])
                    {
                        long edgeKey = Algorithms.Key(currentTri.B, currentTri.C);
                        if (!insertedSegmentsAsBoundaries.Contains(edgeKey))
                        {
                            visited[adj.NeighbourBC] = true;
                            queue.Enqueue(adj.NeighbourBC);
                        }
                    }
                    
                    // Neighbor across edge CA
                    if (adj.NeighbourCA >= 0 && !visited[adj.NeighbourCA])
                    {
                        long edgeKey = Algorithms.Key(currentTri.C, currentTri.A);
                        if (!insertedSegmentsAsBoundaries.Contains(edgeKey))
                        {
                            visited[adj.NeighbourCA] = true;
                            queue.Enqueue(adj.NeighbourCA);
                        }
                    }
                }
                
                clusters.Add(cluster);
            }
            
            return clusters;
        }

        private static void RemoveMarkedTriangles(List<Tri> resultTriangles, List<SourceTriangle> sourceTriangleIndex, int start)
        {
            int indexer = start;
            for (int i = start; i < resultTriangles.Count; ++i)
            {
                if (sourceTriangleIndex[i].SourceTriangleIndex >= 0)
                {
                    resultTriangles[indexer] = resultTriangles[i];
                    sourceTriangleIndex[indexer++] = sourceTriangleIndex[i];
                }
            }
            if (indexer < resultTriangles.Count)
            {
                resultTriangles.RemoveRange(indexer, resultTriangles.Count - indexer);
                sourceTriangleIndex.RemoveRange(indexer, sourceTriangleIndex.Count - indexer);
            }
        }
    }
}
