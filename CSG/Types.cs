using GeoCore;
using System;
using System.Threading;
using Triangulation;

namespace CSG
{

    //public enum PlaneSegmentCondition
    //{
    //    Segment,
    //    Point,
    //    Invalid
    //}

    public struct IntersectionSegment
    {
        public int Start;
        public int End;
        public int TriIdA;
        public int TriIdB;

        public IntersectionSegment(int start, int end, int triIdA, int triIdB)
        {
            Start = start;
            End = end;
            TriIdA = triIdA;
            TriIdB = triIdB;
        }
    }

    public struct IntersectionSegmentEx
    {
        public int Start;
        public int End;
        public int TriIdA;
        public int TriIdB;
        public Rat3Hybrid StartPoint;
        public Rat3Hybrid EndPoint;

        public override string ToString()
        {
            return Start.ToString()+" "+End.ToString()+"   "+StartPoint.ToString()+" "+EndPoint.ToString();
        }
    }

        public enum MeshOrigin
    {
        MeshA,
        MeshB
    }

    public struct SourceTriangle
    {
        public MeshOrigin MeshOrigin;
        public int SourceTriangleIndex;

        public SourceTriangle(MeshOrigin meshOrigin, int sourceTriangleIndex)
        {
            MeshOrigin = meshOrigin;
            SourceTriangleIndex = sourceTriangleIndex;
        }

        public override string ToString()
        {
            return MeshOrigin.ToString() + " " + SourceTriangleIndex.ToString();
        }
    }

    public enum BooleanOp
    {
        Union,
        Difference,
        Intersect,
        Resolve,
        NoOpIntersectionContourOnly,
        AAsSurfaceBAsTrimSurfaceKeepInTriNormalDirection,
        AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection,
        AAsVolumeBAsTrimSurfaceKeepInTriNormalDirection,
        AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection,

        AAsSurfaceBAsTrimVolumeKeepInside,
        AAsSurfaceBAsTrimVolumeKeepOutside,
    }

    public struct BoolSettings
    {
        public bool keepAll;
        public bool keepInside1;
        public bool keepInside2;
        public bool flipTriOrientation1;
        public bool flipTriOrientation2;
        public bool keepCoplanarIfSameNormal;
        public bool keepCoplanarIfOppositeNormal;

        public BoolSettings(BooleanOp op = BooleanOp.Difference) : this()
        {

            switch (op)
            {
                case BooleanOp.Union:
                    keepInside1 = false;
                    keepInside2 = false;
                    flipTriOrientation1 = false;
                    flipTriOrientation2 = false;
                    keepCoplanarIfSameNormal = true;
                    keepCoplanarIfOppositeNormal = false;
                    break;
                case BooleanOp.Intersect:
                    keepInside1 = true;
                    keepInside2 = true;
                    flipTriOrientation1 = false;
                    flipTriOrientation2 = false;
                    keepCoplanarIfSameNormal = true;
                    keepCoplanarIfOppositeNormal = false;
                    break;
                case BooleanOp.Difference:
                    //keepInside1 = false;
                    //keepInside2 = true;
                    //flipTriOrientation1 = false;
                    //flipTriOrientation2 = true;
                    //keepCoplanarIfSameNormal = true;
                    //keepCoplanarIfOppositeNormal = false;
                    keepAll = false;
                    keepInside1 = true;
                    keepInside2 = false;
                    flipTriOrientation1 = true;
                    flipTriOrientation2 = false;
                    keepCoplanarIfSameNormal = false; // true;
                    keepCoplanarIfOppositeNormal = true; // false;
                    break;
                case BooleanOp.Resolve:
                    keepAll = true;
                    break;
                case BooleanOp.AAsSurfaceBAsTrimVolumeKeepInside:
                    keepInside2 = true;  // Keep triangles from A that are inside B
                    break;
                case BooleanOp.AAsSurfaceBAsTrimVolumeKeepOutside:
                    keepInside2 = false; // Keep triangles from A that are outside B
                    break;
            }
        }
    }

    public struct LazeIntEvaluator
    {
        private int Value;
        private bool IsEvaluated;
        private Func<int> Evaluator;

        public LazeIntEvaluator(Func<int> evaluator) 
        {
            Evaluator = evaluator;
            Value = 0;
            IsEvaluated = false;
        }

        public int GetValue()
        {
            if(!IsEvaluated)
            {
                Value = Evaluator();
                IsEvaluated = true;
            }
            return Value;
        }
    }

    public struct TriPoints
    {
        public Rat3Hybrid A;
        public Rat3Hybrid B;
        public Rat3Hybrid C;
        public LazeIntEvaluator CoplanarEdgeInfo;

        public Rat3Hybrid this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0:
                        return A;
                    case 1:
                        return B;
                    case 2:
                        return C;
                }
                return default;
            }
        }

        public bool IsCoplanarEdge(int i)
        {
            return (CoplanarEdgeInfo.GetValue() & (1 << i)) != 0;
        }

        public bool IsPointOnBoundary(Rat3Hybrid pt)
        {
            PlaneConvexPolygon helper = new PlaneConvexPolygon(A, B, C);
            return helper.PointIsOnBoundary(pt, out _, out _, true);
        }
        public bool IsPointInsideOrOnBoundary(Rat3Hybrid pt)
        {
            PlaneConvexPolygon helper = new PlaneConvexPolygon(A, B, C);
            return helper.PointIsInsideOrOnBoundary(pt, out _, true);
        }
    }

    //public struct PointWithIndex
    //{
    //    public Rat3Hybrid A;
    //    public int IndexA;

    //    public PointWithIndex(Rat3Hybrid a) 
    //    {
    //        A = a;
    //        IndexA = -1;
    //    }
    //    public void UpdateIndex(NewPointCreator newPoints)
    //    {
    //        IndexA = newPoints.GetIndex(A);
    //    }
    //}

    public struct PointPair
    {
        public Rat3Hybrid A;
        public Rat3Hybrid B;
        public int IndexA;
        public int IndexB;

        public PointPair(Rat3Hybrid a, Rat3Hybrid b)
        {
            A = a;
            B = b;
            IndexA = -1;
            IndexB = -1;
        }
        public void UpdateIndices(NewPointCreator newPoints)
        {
            IndexA = newPoints.GetIndex(A);
            IndexB = newPoints.GetIndex(B);
        }
    }

    public struct Rat3LineSegmentOnTri
    {
        public Rat3Hybrid A;
        public Rat3Hybrid B;
        public int TriangleIdA;
        public int TriangleIdB;
    }

    public struct Rat3LineSegment
    {
        public Rat3Hybrid A;
        public Rat3Hybrid B;

        public Rat3LineSegment(Rat3Hybrid a, Rat3Hybrid b)
        {
            A = a;
            B = b;
        }

        public Rat3Hybrid GetStartPoint()
        {
            return A;
        }
        public Rat3Hybrid GetEndPoint()
        {
            return B;
        }

        //Trims away everything on positive side of teh plane
        public Rat3LineSegment Trim(in Rat3HybridPlane plane, out bool resultIsInvalid)
        {
            int signA = plane.SignedDistancePointPlaneSign(A, out bool haveDistA, out BigRationalHybrid distA);
            int signB = plane.SignedDistancePointPlaneSign(B, out bool haveDistB, out BigRationalHybrid distB);
            if (signA > 0 && signB > 0)
            {
                resultIsInvalid = true;
                return new Rat3LineSegment();
            }
            if (signA <= 0 && signB <= 0)
            {
                resultIsInvalid = false;
                return this;
            }
            if (signA == 0 && signB > 0)
            {
                resultIsInvalid = false;
                return new Rat3LineSegment(A, A);
            }
            if (signB == 0 && signA > 0)
            {
                resultIsInvalid = false;
                return new Rat3LineSegment(B, B);
            }

            if (signA == 0 || signB == 0)
                throw new Exception("The zero distance cases should already be handled at this point");

            if (!haveDistA)
                distA = plane.SignedDistancePointPlane(A);
            if (!haveDistB)
                distB = plane.SignedDistancePointPlane(B);

            distA = BigRationalHybrid.Abs(distA);
            distB = BigRationalHybrid.Abs(distB);
            distA.Simplify();
            distB.Simplify();

            var intersectionPoint = (distA * B + distB * A) / (distA + distB);
            // A clipped endpoint is reused by subsequent planes. Reduce its
            // exact representation here to prevent multiplicative growth.
            intersectionPoint.Simplify();

            if (signA > 0 && signB < 0)
            {
                resultIsInvalid = false;
                return new Rat3LineSegment(intersectionPoint, B);
            }
            if (signB > 0 && signA < 0)
            {
                resultIsInvalid = false;
                return new Rat3LineSegment(A, intersectionPoint);
            }

            throw new Exception("This line should never be reached");
        }
    }

    public class ResolverTriangle
    {
        public int A;
        public int B;
        public int C;
        public int Source;

        private readonly NewPointCreator _pointCreator;
        private readonly Lazy<PlaneConvexPolygon> _coplanarPlanePolygon;

        public List<PointPair> insertedSegments; // Can contain points (seg.Start = seg.End)

        public LazeIntEvaluator CoplanarEdgeInfo; //bit 0 is AB, bit 1 is BC, bit 2 is CA

        public ResolverTriangle(int a, int b, int c, int i, NewPointCreator newPoints/*, 
            bool abCoplanar, bool bcCoplanar, bool caCoplanar*/)
        {
            A = a;
            B = b;
            C = c;

            Source = i;

            _pointCreator = newPoints ?? throw new ArgumentNullException(nameof(newPoints));
            _coplanarPlanePolygon = new Lazy<PlaneConvexPolygon>(CreateCoplanarPlanePolygon, LazyThreadSafetyMode.ExecutionAndPublication);

            //CoplanarEdgeInfo = 0;
            /*if (abCoplanar) CoplanarEdgeInfo |= 1;
            if (bcCoplanar) CoplanarEdgeInfo |= 2;
            if (caCoplanar) CoplanarEdgeInfo |= 4;*/
        }

        private PlaneConvexPolygon CreateCoplanarPlanePolygon() =>
            new PlaneConvexPolygon(_pointCreator.GetPoint(A), _pointCreator.GetPoint(B), _pointCreator.GetPoint(C));

        /// <summary>
        /// Thread-safe lazy coplanar helper for this triangle (vertices from the constructor <c>NewPointCreator</c> at A,B,C).
        /// Valid while those coordinates are unchanged (e.g. boolean overlap phase).
        /// </summary>
        public PlaneConvexPolygon CoplanarPlanePolygon => _coplanarPlanePolygon.Value;

        //public void SetCoplanarEdgeInfo(bool abCoplanar, bool bcCoplanar, bool caCoplanar)
        //{
        //    CoplanarEdgeInfo = 0;
        //    if (abCoplanar) CoplanarEdgeInfo |= 1;
        //    if (bcCoplanar) CoplanarEdgeInfo |= 2;
        //    if (caCoplanar) CoplanarEdgeInfo |= 4;
        //}

        public void SetCoplanarEdgeInfo(Func<int> coplanarEdgeInfo)
        {
            CoplanarEdgeInfo = new(coplanarEdgeInfo);
        }

        public Box3I GetBounds(NewPointCreator points)
        {
            // Point enclosures are monotone in each coordinate: enclosing
            // first and taking integer extrema gives the same box as finding
            // rational extrema first. Avoid twelve rational comparisons (and
            // their potentially very large cross-products) per triangle.
            var box = points.GetPoint(A).GetBox();
            box.Extend(points.GetPoint(B).GetBox());
            box.Extend(points.GetPoint(C).GetBox());

            return box;
        }

        public bool AddSegment(in Rat3LineSegment s)
        {
            return AddSegment(new PointPair(s.A, s.B));
        }

        // Constrained triangulation needs a planar graph: every crossing and
        // overlapping endpoint must be an explicit vertex. Arrange before
        // registering boundaries, so clustering uses the resulting subedges.
        private void CollectTrimSegmentCuts(NewPointCreator points, Dictionary<long, List<Rat3Hybrid>> sharedCuts)
        {
            var segments = insertedSegments;
            if (segments == null || segments.Count < 2) return;
            var a = points.GetPoint(A);
            var b = points.GetPoint(B);
            var c = points.GetPoint(C);
            if (!CoplanarTriangleProjectionAxes.TryProjectNonDegenerate(
                    in a, in b, in c, out _, out _, out _, out _, out int x, out int y))
                return;

            var bounds = segments.Select(segment => (
                minX:segment.A[x]<segment.B[x]?segment.A[x]:segment.B[x],
                maxX:segment.A[x]>segment.B[x]?segment.A[x]:segment.B[x],
                minY:segment.A[y]<segment.B[y]?segment.A[y]:segment.B[y],
                maxY:segment.A[y]>segment.B[y]?segment.A[y]:segment.B[y])).ToArray();
            var cuts = new List<BigRationalHybrid>[segments.Count];
            for (int i = 0; i < segments.Count; i++)
                cuts[i] = new List<BigRationalHybrid> { new(0), new(1) };

            BigRationalHybrid Cross(Rat3Hybrid u, Rat3Hybrid v) => u[x] * v[y] - u[y] * v[x];
            void AddEndpoint(int index, Rat3Hybrid point)
            {
                var segment = segments[index];
                var direction = segment.B - segment.A;
                if (direction == new Rat3Hybrid(0, 0, 0) || Cross(point - segment.A, direction).Sign() != 0) return;
                int axis = direction[x].Sign() != 0 ? x : y;
                var parameter = (point[axis] - segment.A[axis]) / direction[axis];
                if (parameter >= new BigRationalHybrid(0) && parameter <= new BigRationalHybrid(1))
                    cuts[index].Add(parameter);
            }

            for (int i = 0; i < segments.Count; i++)
            {
                var first = segments[i];
                var direction = first.B - first.A;
                for (int j = i + 1; j < segments.Count; j++)
                {
                    if (bounds[i].maxX<bounds[j].minX || bounds[j].maxX<bounds[i].minX ||
                        bounds[i].maxY<bounds[j].minY || bounds[j].maxY<bounds[i].minY) continue;
                    var second = segments[j];
                    var otherDirection = second.B - second.A;
                    var denominator = Cross(direction, otherDirection);
                    if (denominator.Sign() == 0)
                    {
                        AddEndpoint(i, second.A);
                        AddEndpoint(i, second.B);
                        AddEndpoint(j, first.A);
                        AddEndpoint(j, first.B);
                        continue;
                    }
                    var delta = second.A - first.A;
                    var u = Cross(delta, otherDirection) / denominator;
                    var v = Cross(delta, direction) / denominator;
                    if (u < new BigRationalHybrid(0) || u > new BigRationalHybrid(1) ||
                        v < new BigRationalHybrid(0) || v > new BigRationalHybrid(1)) continue;
                    cuts[i].Add(u);
                    cuts[j].Add(v);
                }
            }

            for (int i = 0; i < segments.Count; i++)
            {
                var segment = segments[i];
                int start = points.GetIndex(segment.A, out _);
                int end = points.GetIndex(segment.B, out _);
                long key = Algorithms.Key(start,end);
                if (!sharedCuts.TryGetValue(key,out var list))
                    sharedCuts.Add(key,list=new List<Rat3Hybrid>());
                foreach (var parameter in cuts[i])
                    list.Add(segment.A+(segment.B-segment.A)*parameter);
            }
        }

        internal static void ArrangeTrimSegments(IEnumerable<ResolverTriangle> triangles, NewPointCreator points)
        {
            var sharedCuts = new Dictionary<long,List<Rat3Hybrid>>();
            foreach (var triangle in triangles)
                triangle.CollectTrimSegmentCuts(points,sharedCuts);
            // Both intersecting meshes must use identical subedges. A cut found
            // on either triangle is propagated to every copy of that segment.
            foreach (var triangle in triangles)
            {
                var segments=triangle.insertedSegments;
                if (segments==null) continue;
                triangle.insertedSegments=null;
                foreach(var segment in segments)
                {
                    var direction=segment.B-segment.A;
                    long key=Algorithms.Key(points.GetIndex(segment.A,out _),points.GetIndex(segment.B,out _));
                    if (direction==new Rat3Hybrid(0,0,0)||!sharedCuts.TryGetValue(key,out var locations))
                    {
                        triangle.AddSegment(segment);
                        continue;
                    }
                    int axis=direction.X.Sign()!=0?0:direction.Y.Sign()!=0?1:2;
                    var parameters=locations.Select(point=>(point[axis]-segment.A[axis])/direction[axis]).ToList();
                    parameters.Sort((left,right)=>left.CompareTo(right));
                    for(int i=1;i<parameters.Count;++i)
                    {
                        if(parameters[i]==parameters[i-1])continue;
                        triangle.AddSegment(new PointPair(segment.A+direction*parameters[i-1],segment.A+direction*parameters[i]));
                    }
                }
            }
        }

        public void PrepareTriangulate(NewPointCreator newPointCreator, Dictionary<long, int> splitSegments = null)
        {
            if (insertedSegments != null)
            {
                for (int i = 0; i < insertedSegments.Count; ++i)
                {
                    PointPair seg = insertedSegments[i];
                    seg.UpdateIndices(newPointCreator);
                    insertedSegments[i] = seg;

                    if (splitSegments != null)
                    {
                        var key = Algorithms.Key(seg.IndexA, seg.IndexB);
                        // TODO: Verify: Probably duplicate segments in the same mesh can exist if the segment lies exactly on a triangle edge and each triangle edge is shared between 2 triangles
                        if (!splitSegments.ContainsKey(key))
                            splitSegments.Add(key, Source);
                    }
                }
            }
        }

        public bool ContainsIntersections()
        {
            return !(insertedSegments == null || insertedSegments.Count == 0);
        }

        private static Int2 GetProjected(in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c) =>
            CoplanarTriangleProjectionAxes.GetPrincipalPlaneAxisIndices(in a, in b, in c);

        //https://ti.inf.ethz.ch/ew/courses/Geo17/lecture/gca17-6.pdf
        public List<Tri> Triangulate(NewPointCreator points)
        {
            if (!ContainsIntersections())
                return new List<Tri>() { new Tri(A, B, C) };

            //var points = newPointCreator.Points;

            // Project onto a coordinate plane with non-degenerate 2D area (try all three if needed).
            var a = points.GetPoint(A);
            var b = points.GetPoint(B);
            var c = points.GetPoint(C);
            if (!CoplanarTriangleProjectionAxes.TryProjectNonDegenerate(
                    in a, in b, in c,
                    out Rat2Hybrid projA, out Rat2Hybrid projB, out Rat2Hybrid projC,
                    out _, out int x, out int y))
            {
                return new List<Tri> { new Tri(A, B, C) };
            }

            List<Rat2Hybrid> bigRationalPoints = new List<Rat2Hybrid>
            {
                projA,
                projB,
                projC
            };

            List<Int2> constraints = new List<Int2>(insertedSegments.Count);

            List<int> expansionMap = new List<int>() { A, B, C };

            Dictionary<int, int> compactionMap = new Dictionary<int, int>();
            compactionMap.Add(A, 0);
            compactionMap.Add(B, 1);
            compactionMap.Add(C, 2);

            for (int i = 0; i < insertedSegments.Count; ++i)
            {
                var seg = insertedSegments[i];
                //seg.UpdateIndices(newPointCreator);
                if (!compactionMap.ContainsKey(seg.IndexA))
                {
                    compactionMap.Add(seg.IndexA, bigRationalPoints.Count);
                    expansionMap.Add(seg.IndexA);
                    var p = seg.A;
                    bigRationalPoints.Add(new Rat2Hybrid(p[x], p[y]));
                }
                if (!compactionMap.ContainsKey(seg.IndexB))
                {
                    compactionMap.Add(seg.IndexB, bigRationalPoints.Count);
                    expansionMap.Add(seg.IndexB);
                    var p = seg.B;
                    bigRationalPoints.Add(new Rat2Hybrid(p[x], p[y]));
                }
                constraints.Add(new Int2(compactionMap[seg.IndexA], compactionMap[seg.IndexB]));
            }

            if (compactionMap.Count != expansionMap.Count)
                throw new InvalidOperationException("CSG triangle triangulation: constraint point compaction map size mismatch.");

            List<int> borderPolygon = new List<int>() { 0, 1, 2 };

#if DEBUG
            CSGDebug.AnalyzeConstraints(bigRationalPoints, constraints);

            bool export = false;
            if (export)
            {
                CSGDebug.WriteLineObj(@"C:\tmp\debug.obj", bigRationalPoints, borderPolygon, constraints);
                CSGDebug.WriteLineObjRational(@"C:\tmp\debug_rational.obj", bigRationalPoints, borderPolygon, constraints);
            }
#endif

            //List<Tri> triangles = TriangulateConstrained<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>.Triangulate(bigRationalPoints, borderPolygon, constraints);
            // Constrained triangulation is enough for CSG; Delaunay flips are quality-only and
            // run exact InCircle on fat rationals.
            List<Tri> triangles = Triangulator.TriangulatePolygon(bigRationalPoints, borderPolygon, constraints, null, false);

            // Find all end points of constraints that lie exactly on the border polygon and insert these points
            // If start and end point of one constraint are collinear with the border polygon edge, then the constraint can be deleted
            // All the constraint segments that remain should be connected to strips using the HashSegmentConnector
            // All strips should either be a closed loop or run from polygon border to polygon border
            // The strips divide the polygon into multiple regions - each region can be triangulated using standard earclipping, see EarClippingWithHoles
            // Loops can be nested and they lie inside a region - they need speical attention
            for (int i = 0; i < triangles.Count; ++i)
            {
                var tri = triangles[i];
                tri.A = expansionMap[tri.A];
                tri.B = expansionMap[tri.B];
                tri.C = expansionMap[tri.C];
                triangles[i] = tri;
            }
            return triangles;
        }

      

        public bool AddPoint(Rat3Hybrid p)
        {
            return AddSegment(new PointPair(p, p));
        }

        public bool AddSegment(PointPair segment)
        {
            // Dual-write Resolve can add to the same B triangle from many A workers.
            lock (this)
            {
            if (insertedSegments == null)
                insertedSegments = new List<PointPair>();

            segment.A.Simplify();
            segment.B.Simplify();

            //Check if the segment already exists
            for (int i = 0; i < insertedSegments.Count; ++i)
            {
                var s = insertedSegments[i];
                if (s.A == segment.A && s.B == segment.B)
                {
#if DEBUG
                    var tmp = segment;// new PlanePointPair(a, b, newPoints);
                    if (s.IndexA != tmp.IndexA || s.IndexB != tmp.IndexB)
                        throw new Exception();
#endif
                    return false;
                }
                if (s.A == segment.B && s.B == segment.A)
                {
#if DEBUG
                    var tmp = segment;// new PlanePointPair(a, b, newPoints);
                    if (s.IndexA != tmp.IndexB || s.IndexB != tmp.IndexA)
                        throw new Exception();
#endif
                    return false;
                }
            }

            var pair = segment;// new PlanePointPair(a, b, newPoints);

#if DEBUG
            //if (a == newPoints.GetPlanePoint(A) && pair.IndexA != A)
            //    throw new Exception();
            //if (a == newPoints.GetPlanePoint(B) && pair.IndexA != B)
            //    throw new Exception();
            //if (a == newPoints.GetPlanePoint(C) && pair.IndexA != C)
            //    throw new Exception();


            //if (b == newPoints.GetPlanePoint(A) && pair.IndexB != A)
            //    throw new Exception();
            //if (b == newPoints.GetPlanePoint(B) && pair.IndexB != B)
            //    throw new Exception();
            //if (b == newPoints.GetPlanePoint(C) && pair.IndexB != C)
            //    throw new Exception();


            //if (!PointInTriangle(a))
            //    throw new Exception();

            //if (!PointInTriangle(b))
            //    throw new Exception();
#endif

            insertedSegments.Add(pair);

            //AvoidPointDuplicates();

            return true;
            }
        }

        public bool ContainsEdge(int a, int b)
        {
            if (a == b)
                throw new Exception();

            return Contains(a) && Contains(b);
        }
        public bool Contains(int id)
        {
            return A == id || B == id || C == id;
        }

    }

    public enum IntersectionResult
    {
        None,
        Point, //Non coplanar line intersects with triangle - intersection point can be exactly on boundary
        LineSegment, //Coplanar line segment partially or fully inside triangle
        LineSegmentZeroLength //Coplanar line segment touches triangle
    }

    public struct Rat3HybridPlane
    {
        public Rat3Hybrid Normal;
        public BigRationalHybrid PlaneD;

        public Rat3HybridPlane(in Rat3Hybrid normal, in Rat3Hybrid pointOnPlane)
        {
            // Own the coefficients before reducing: caller vectors can be
            // shared by concurrent triangle intersection workers.
            Normal = new Rat3Hybrid(normal);
            Normal.Simplify();
            PlaneD = -Rat3Hybrid.Dot(Normal, pointOnPlane);
            PlaneD.Simplify();
        }

        public BigRationalHybrid SignedDistancePointPlane(in Rat3Hybrid p)
        {
            return p.X * Normal.X + p.Y * Normal.Y + p.Z * Normal.Z + PlaneD;
        }

        /// <summary>Exact sign of <see cref="SignedDistancePointPlane"/>; cheaper when all components use the long fast path.</summary>
        public int SignedDistancePointPlaneSign(in Rat3Hybrid p)
        {
            return BigRationalHybrid.SignOfPlaneDistance(
                in p.X, in Normal.X, in p.Y, in Normal.Y, in p.Z, in Normal.Z, in PlaneD, out _, out _);
        }

        /// <inheritdoc cref="SignedDistancePointPlaneSign(in Rat3Hybrid)"/>
        /// <param name="rationalDistanceWasComputed">True iff <paramref name="rationalDistance"/> is the exact signed distance (hybrid sum).</param>
        public int SignedDistancePointPlaneSign(in Rat3Hybrid p, out bool rationalDistanceWasComputed, out BigRationalHybrid rationalDistance)
        {
            return BigRationalHybrid.SignOfPlaneDistance(
                in p.X, in Normal.X, in p.Y, in Normal.Y, in p.Z, in Normal.Z, in PlaneD,
                out rationalDistanceWasComputed, out rationalDistance);
        }
    }

    /// <summary>
    /// Picks two world axes to orthographically project a coplanar triangle: drop the coordinate where |plane normal| is largest
    /// (stable conditioning). Same rule as <see cref="ResolverTriangle.Triangulate"/> projection.
    /// </summary>
    internal static class CoplanarTriangleProjectionAxes
    {
        public static Int2 GetPrincipalPlaneAxisIndices(in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c)
        {
            var ab = b - a;
            var ac = c - a;
            var n = Rat3Hybrid.Cross(ab, ac);
            bool xNeg = n.X < BigRationalHybrid.Zero;
            if (xNeg) n.X = -n.X;
            bool yNeg = n.Y < BigRationalHybrid.Zero;
            if (yNeg) n.Y = -n.Y;
            bool zNeg = n.Z < BigRationalHybrid.Zero;
            if (zNeg) n.Z = -n.Z;

            int x;
            int y;
            if (n.X >= n.Y && n.X >= n.Z)
            {
                x = 1;
                y = 2;
                if (xNeg)
                    Algorithms.Swap(ref x, ref y);
            }
            else if (n.Y >= n.X && n.Y >= n.Z)
            {
                x = 2;
                y = 0;
                if (yNeg)
                    Algorithms.Swap(ref x, ref y);
            }
            else
            {
                x = 0;
                y = 1;
                if (zNeg)
                    Algorithms.Swap(ref x, ref y);
            }

            return new Int2(x, y);
        }

        private static bool UnorderedAxisPairMatches(in Int2 primary, int ax1, int ax2) =>
            (primary.X == ax1 && primary.Y == ax2) || (primary.X == ax2 && primary.Y == ax1);

        /// <summary>Tries principal axes first, then the other coordinate planes until projected area is non-degenerate.</summary>
        public static bool TryProjectNonDegenerate(in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c,
            out Rat2Hybrid outA2, out Rat2Hybrid outB2, out Rat2Hybrid outC2, out int refSign, out int outAx1, out int outAx2)
        {
            Int2 primary = GetPrincipalPlaneAxisIndices(in a, in b, in c);
            (int ax1, int ax2)[] cand = new (int ax1, int ax2)[3];
            cand[0] = (primary.X, primary.Y);
            int n = 1;
            (int u, int v)[] all = { (1, 2), (0, 2), (0, 1) };
            for (int i = 0; i < 3 && n < 3; i++)
            {
                if (UnorderedAxisPairMatches(in primary, all[i].u, all[i].v))
                    continue;
                cand[n++] = (all[i].u, all[i].v);
            }

            outA2 = default;
            outB2 = default;
            outC2 = default;
            refSign = 0;
            outAx1 = 0;
            outAx2 = 1;

            for (int i = 0; i < n; i++)
            {
                var t = cand[i];
                outAx1 = t.ax1;
                outAx2 = t.ax2;
                outA2 = new Rat2Hybrid(a[t.ax1], a[t.ax2]);
                outB2 = new Rat2Hybrid(b[t.ax1], b[t.ax2]);
                outC2 = new Rat2Hybrid(c[t.ax1], c[t.ax2]);
                refSign = Rat2Hybrid.Orient2DSign(in outA2, in outB2, in outC2);
                if (refSign != 0)
                    return true;
            }

            return false;
        }
    }

    public class PlaneConvexPolygon
    {
        public Rat3Hybrid A;
        public Rat3Hybrid B;
        public Rat3Hybrid C;

        private Rat3HybridPlane triPlane;
        private Rat3HybridPlane ab;
        private Rat3HybridPlane bc;
        private Rat3HybridPlane ca;

        private Rat2Hybrid a2;
        private Rat2Hybrid b2;
        private Rat2Hybrid c2;
        private Rat2Hybrid edgeAB2;
        private Rat2Hybrid edgeBC2;
        private Rat2Hybrid edgeCA2;
        private int orient2dRefSign;
        private int projAxis1;
        private int projAxis2;

        public PlaneConvexPolygon(Rat3Hybrid a, Rat3Hybrid b, Rat3Hybrid c)
        {
            A = a;
            B = b;
            C = c;


            var normal = Rat3Hybrid.Cross(b - a, c - a);
            triPlane = new Rat3HybridPlane(normal, a);
            ab = new Rat3HybridPlane(Rat3Hybrid.Cross(normal, b - a), a);
            bc = new Rat3HybridPlane(Rat3Hybrid.Cross(normal, c - b), b);
            ca = new Rat3HybridPlane(Rat3Hybrid.Cross(normal, a - c), c);


            var center = (a + b + c) / new BigRationalHybrid(3);
            var signAB = ab.SignedDistancePointPlaneSign(center);
            var signBC = bc.SignedDistancePointPlaneSign(center);
            var signCA = ca.SignedDistancePointPlaneSign(center);

            if (signAB != signBC || signAB != signCA || signBC != signCA)
                throw new Exception();

            if (signAB == 0 || signBC == 0 || signCA == 0)
                throw new Exception();

            if (signAB > 0)
            {
                ab.Normal = -ab.Normal;
                ab.PlaneD = -ab.PlaneD;
            }
            if (signBC > 0)
            {
                bc.Normal = -bc.Normal;
                bc.PlaneD = -bc.PlaneD;
            }
            if (signCA > 0)
            {
                ca.Normal = -ca.Normal;
                ca.PlaneD = -ca.PlaneD;
            }

            InitPlanar2DProjection(a, b, c);
        }

        private static void InitPlanar2DProjectionForTriangle(in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c,
            out Rat2Hybrid outA2, out Rat2Hybrid outB2, out Rat2Hybrid outC2, out int refSign, out int outAx1, out int outAx2)
        {
            if (!CoplanarTriangleProjectionAxes.TryProjectNonDegenerate(in a, in b, in c, out outA2, out outB2, out outC2, out refSign,
                    out outAx1, out outAx2))
                throw new Exception("Triangle projects degenerately on all coordinate planes.");
        }

        private static Rat2Hybrid Project2(in Rat3Hybrid p, int ax1, int ax2) =>
            new Rat2Hybrid(p[ax1], p[ax2]);

        private void InitPlanar2DProjection(in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c)
        {
            InitPlanar2DProjectionForTriangle(in a, in b, in c, out a2, out b2, out c2, out orient2dRefSign, out projAxis1, out projAxis2);
            edgeAB2 = b2 - a2;
            edgeBC2 = c2 - b2;
            edgeCA2 = a2 - c2;
        }

        public IntersectionResult GetIntersectionCoplanar(in Rat3LineSegment seg, out Rat3LineSegment segment)
        {
            segment = default;
            Rat3LineSegment clipped = seg; // new PlaneSegment(seg.l0, seg.l1);


            bool resultIsInvalid;
            clipped = clipped.Trim(ab, out resultIsInvalid);
            if (resultIsInvalid)
                return IntersectionResult.None;

            clipped = clipped.Trim(bc, out resultIsInvalid);
            if (resultIsInvalid)
                return IntersectionResult.None;

            clipped = clipped.Trim(ca, out resultIsInvalid);
            if (resultIsInvalid)
                return IntersectionResult.None;

            //PlaneSegmentCondition condition = clipped.GetCondition();
            if (clipped.GetStartPoint() != clipped.GetEndPoint())
            {
                segment = clipped;

#if DEBUG
                if (!PointIsInsideOrOnBoundary(clipped.GetStartPoint(), out _, false))
                    throw new Exception();

                if (!PointIsInsideOrOnBoundary(clipped.GetEndPoint(), out _, false))
                    throw new Exception();
#endif

                return IntersectionResult.LineSegment;
            }
            else //if (condition == PlaneSegmentCondition.Point)
            {
                //intersection = PlaneArithmetic.IntersectionPoint(clipped.l0, clipped.l1, clipped.r0);
                segment = clipped;
                return IntersectionResult.LineSegmentZeroLength; //TODO: Is it good to emit a point here?
            }
        }

        public bool PointIsOnBoundary(in Rat3Hybrid x, out bool isOnCorner, out int boundaryIndex, bool performInPlaneCheck = false)
        {
            bool isOnBoundary;
            bool insideOrOnBoundary = PointIsInsideOrOnBoundary(x, out isOnCorner, out isOnBoundary, out boundaryIndex, performInPlaneCheck);
            return insideOrOnBoundary && isOnBoundary;
        }

        public bool PointIsInsideOrOnBoundary(in Rat3Hybrid x, out bool isOnBoundary, bool performInPlaneCheck = false)
        {
            return PointIsInsideOrOnBoundary(x, out _, out isOnBoundary, out _, performInPlaneCheck);
        }

        public bool PointIsInsideOrOnBoundary2(in Rat3Hybrid x, out bool isOnBoundary, bool performInPlaneCheck = false)
        {
            return PointIsInsideOrOnBoundary2(x, out _, out isOnBoundary, out _, performInPlaneCheck);
        }

        public bool PointIsInsideOrOnBoundary(in Rat3Hybrid x, out bool isOnCorner, out bool isOnBoundary, out int boundaryIndex,
            bool performInPlaneCheck = false)
        {
            if (performInPlaneCheck)
            {
                var d = triPlane.SignedDistancePointPlane(x);
                if (d != BigRationalHybrid.Zero)
                    throw new Exception();
            }

            Rat2Hybrid p2 = Project2(x, projAxis1, projAxis2);
            Rat2Hybrid va = p2 - a2;
            var signAB = Rat2Hybrid.CrossSign(in edgeAB2, in va);
            if (signAB != 0 && signAB != orient2dRefSign)
            {
                boundaryIndex = -1;
                isOnCorner = false;
                isOnBoundary = false;
                return false;
            }

            Rat2Hybrid vpb = va - edgeAB2;
            var signBC = Rat2Hybrid.CrossSign(in edgeBC2, in vpb);
            if (signBC != 0 && signBC != orient2dRefSign)
            {
                boundaryIndex = -1;
                isOnCorner = false;
                isOnBoundary = false;
                return false;
            }

            Rat2Hybrid vpc = vpb - edgeBC2;
            var signCA = Rat2Hybrid.CrossSign(in edgeCA2, in vpc);
            if (signCA != 0 && signCA != orient2dRefSign)
            {
                boundaryIndex = -1;
                isOnCorner = false;
                isOnBoundary = false;
                return false;
            }

            boundaryIndex = -1;

            int zeroCount = 0;
            if (signAB == 0)
            {
                ++zeroCount;
                boundaryIndex = 0;
            }
            if (signBC == 0)
            {
                ++zeroCount;
                boundaryIndex = 1;
            }
            if (signCA == 0)
            {
                ++zeroCount;
                boundaryIndex = 2;
            }

            if (zeroCount > 2)
                throw new Exception();
            isOnCorner = zeroCount == 2;

            bool pointIsInsideOrOnBoundary = EqualOrOneIsZero(signAB, orient2dRefSign) && EqualOrOneIsZero(signBC, orient2dRefSign) &&
                EqualOrOneIsZero(signCA, orient2dRefSign);

            isOnBoundary = pointIsInsideOrOnBoundary && zeroCount > 0;

            return pointIsInsideOrOnBoundary;
        }

        /// <summary>3D half-space test (same predicate as pre-2D implementation); kept for reference and parity checks.</summary>
        public bool PointIsInsideOrOnBoundary2(in Rat3Hybrid x, out bool isOnCorner, out bool isOnBoundary, out int boundaryIndex,
            bool performInPlaneCheck = false)
        {
            if (performInPlaneCheck)
            {
                var d = triPlane.SignedDistancePointPlane(x);
                if (d != BigRationalHybrid.Zero)
                    throw new Exception();
            }

            var signAB = ab.SignedDistancePointPlaneSign(x);
            var signBC = bc.SignedDistancePointPlaneSign(x);
            var signCA = ca.SignedDistancePointPlaneSign(x);

            boundaryIndex = -1;

            int zeroCount = 0;
            if (signAB == 0)
            {
                ++zeroCount;
                boundaryIndex = 0;
            }
            if (signBC == 0)
            {
                ++zeroCount;
                boundaryIndex = 1;
            }
            if (signCA == 0)
            {
                ++zeroCount;
                boundaryIndex = 2;
            }

            if (zeroCount > 2)
                throw new Exception();
            isOnCorner = zeroCount == 2;

            bool pointIsInsideOrOnBoundary = EqualOrOneIsZero(signAB, signBC) && EqualOrOneIsZero(signAB, signCA) && EqualOrOneIsZero(signBC, signCA);

            isOnBoundary = pointIsInsideOrOnBoundary && zeroCount > 0;

            return pointIsInsideOrOnBoundary;
        }

        private static bool EqualOrOneIsZero(int lhs, int rhs)
        {
            return lhs == 0 || rhs == 0 || lhs == rhs;
        }
    }
}
