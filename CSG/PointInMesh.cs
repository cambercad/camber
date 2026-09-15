using GeoCore;

namespace CSG
{
    public enum InsideResult
    {
        Inside,
        //OnSurface,
        CoplanarSameNormal,
        CoplanarOppositeNormal,
        Outside,
        Unknown
    }


    public enum IntersectionCountInfo
    {
        StartOnSurface,
        EndOnSurface,
        Normal,
        InvalidConfiguration
    }

    public interface IIntersector
    {
        Rat3Hybrid TriangleNormal(int triangleIndex);

        SegmentTriangleIntersectionType SegmentIntersectsTriangle(in Rat3Hybrid segmentStart, in Int3 segmentEnd, int triangleIndex,
            out bool intersectionIsOnTriangleBoundary, out bool startIsOnTriangle, out bool endIsOnTriangle);

        SegmentTriangleIntersectionType SegmentIntersectsTriangle(in Int3 segmentStart, in Int3 segmentEnd, int triangleIndex,
            out bool intersectionIsOnTriangleBoundary, out bool startIsOnTriangle, out bool endIsOnTriangle);
    }

    public struct Int3Intersector : IIntersector
    {
        private NewPointCreator meshPoints;
        private List<Tri> meshTriangles;

        public Int3Intersector(NewPointCreator meshPoints, List<Tri> meshTriangles)
        {
            this.meshPoints = meshPoints;
            this.meshTriangles = meshTriangles;
        }

        public SegmentTriangleIntersectionType SegmentIntersectsTriangle(in Rat3Hybrid segmentStart, in Int3 segmentEnd, int triangleIndex,
            out bool intersectionIsOnTriangleBoundary, out bool startIsOnTriangle, out bool endIsOnTriangle)
        {
            var tri = meshTriangles[triangleIndex];
            var a = meshPoints.GetPoint(tri.A);
            var b = meshPoints.GetPoint(tri.B);
            var c = meshPoints.GetPoint(tri.C);

            SegmentTriangleIntersectionType res = TriangleSegmentIntersector.SegmentIntersectsTriangle(segmentStart,
                new Rat3Hybrid(segmentEnd.X, segmentEnd.Y, segmentEnd.Z), a, b, c, out _,
                out intersectionIsOnTriangleBoundary, out startIsOnTriangle, out endIsOnTriangle);
            return res;
        }

        public SegmentTriangleIntersectionType SegmentIntersectsTriangle(in Int3 segmentStart, in Int3 segmentEnd, int triangleIndex,
           out bool intersectionIsOnTriangleBoundary, out bool startIsOnTriangle, out bool endIsOnTriangle)
        {
            return this.SegmentIntersectsTriangle(new Rat3Hybrid(segmentStart.X, segmentStart.Y, segmentStart.Z), segmentEnd, triangleIndex,
                out intersectionIsOnTriangleBoundary, out startIsOnTriangle, out endIsOnTriangle);
        }

        public Rat3Hybrid TriangleNormal(int triangleIndex)
        {
            var tri = meshTriangles[triangleIndex];
            var a = meshPoints.GetPoint(tri.A);
            var b = meshPoints.GetPoint(tri.B);
            var c = meshPoints.GetPoint(tri.C);
            var delta1 = b - a;
            var delta2 = c - a;
            return Rat3Hybrid.Cross(new Rat3Hybrid(delta1.X, delta1.Y, delta1.Z), new Rat3Hybrid(delta2.X, delta2.Y, delta2.Z));
        }
    }

    public class PointInMesh
    {
        public static InsideResult PointInsideMesh<Intersec>(in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c, in Intersec intersector /*List<Int3> points, List<Triangle> triangles*/, BVHNode[] tree, in Box3I trianglesBoundingBox)
            where Intersec : struct, IIntersector
        {
            Box3I testBox = a.GetBox();
            testBox.Extend(b.GetBox());
            testBox.Extend(c.GetBox());
            if (!trianglesBoundingBox.OverlapOrTouch(testBox))
            {
                return InsideResult.Outside;
            }

            Int3 end;
            //end = new Int3(trianglesBoundingBox.Min.X - 1, trianglesBoundingBox.Min.Y - 1, trianglesBoundingBox.Min.Z - 1);
            //Int3 e = end;
            //var test = PointInsideMesh(end, a, b, c, points, triangles, triIndexOffset, trianglesBoundingBox);

            Rat3Hybrid p = /*new Rat(1, 3) **/ (a + b + c) / new BigRationalHybrid(3);
            Int3 pInt = p.ToInt();

            int id = 0;
            end = new Int3(trianglesBoundingBox.Min.X - 1, pInt.Y, pInt.Z);
            int min = Math.Abs(pInt.X - trianglesBoundingBox.Min.X);
            int m = Math.Abs(pInt.X - trianglesBoundingBox.Max.X);
            if (m < min)
            {
                end = new Int3(trianglesBoundingBox.Max.X + 1, pInt.Y, pInt.Z);
                min = m;
                id = 1;
            }
            m = Math.Abs(pInt.Y - trianglesBoundingBox.Min.Y);
            if (m < min)
            {
                end = new Int3(pInt.X, trianglesBoundingBox.Min.Y - 1, pInt.Z);
                min = m;
                id = 2;
            }
            m = Math.Abs(pInt.Y - trianglesBoundingBox.Max.Y);
            if (m < min)
            {
                end = new Int3(pInt.X, trianglesBoundingBox.Max.Y + 1, pInt.Z);
                min = m;
                id = 3;
            }
            m = Math.Abs(pInt.Z - trianglesBoundingBox.Min.Z);
            if (m < min)
            {
                end = new Int3(pInt.X, pInt.Y, trianglesBoundingBox.Min.Z - 1);
                min = m;
                id = 4;
            }
            m = Math.Abs(pInt.Z - trianglesBoundingBox.Max.Z);
            if (m < min)
            {
                end = new Int3(pInt.X, pInt.Y, trianglesBoundingBox.Max.Z + 1);
                min = m;
                id = 5;
            }
            var result = PointInsideMesh(end, a, b, c, intersector /*points, triangles*/, tree);

            if (trianglesBoundingBox.OverlapOrTouch(new Box3I(end, end)))
                throw new Exception();

            //if (test != result)
            //{
            //    LineGeometry lg = new LineGeometry(new Double3(0, 1, 0), 2);
            //    lg.AddLine(debug.Convert(pInt), debug.Convert(e));

            //    System.Diagnostics.Trace.Write(lg);

            //    lg = new LineGeometry(new Double3(1, 0, 0), 2);
            //    lg.AddLine(debug.Convert(pInt), debug.Convert(end));

            //    System.Diagnostics.Trace.Write(lg);
            //}

            return result;
        }

        public static InsideResult PointInsideMesh<Intersec>(in Int3 end, in Rat3Hybrid a, in Rat3Hybrid b, in Rat3Hybrid c, in Intersec intersector /*List<Int3> points, List<Triangle> triangles*/, BVHNode[] tree)
                where Intersec : struct, IIntersector
        {
            Rat3Hybrid p = /*new Rat(1, 3) **/ (a + b + c) / new BigRationalHybrid(3);
            Box3I pBox = p.GetBox();
            int sum = 0;
            bool repeat = true;
            int counter = 0;
            Random r = new Random(0);
            while (repeat)
            {
                repeat = false;

                Int3 offset = new Int3(r.Next() % (counter + 1) + 1, r.Next() % (counter + 1) + 1, r.Next() % (counter + 1) + 1);
                Int3 pInt = p.ToInt();
                pInt.X -= offset.X;
                pInt.Y -= offset.Y;
                pInt.Z -= offset.Z;

                int c1 = 0;

                {
                    Box3I intersectionBox = new Box3I(new Int3(int.MaxValue), new Int3(int.MinValue));
                    intersectionBox.Extend(pInt /*end*/);
                    intersectionBox.Extend(pBox);
                    intersectionBox.Enlarge(1);

                    TreeBoxOverlapEnumerator triangleIndices = new TreeBoxOverlapEnumerator(tree, intersectionBox.GetBoundsF());
                    //var triangleIdList = triangleIndices.GetList();

                    int coplanarTriangleIndex = -1;
                    var result = IntersectionCountInfo.Normal;
                    //if (triangleIdList.Count > 0)
                    result = PointInMeshTester<Intersec>.CountIntersections(p, pBox, /*end*/pInt, intersector/*points, triangles*/, triangleIndices, out c1, out coplanarTriangleIndex);
                    if (result == IntersectionCountInfo.StartOnSurface)
                    {
                        //Compare the normals of triangle formed by a, b, c and the triangle with index coplanarTriangleIndex
                        //var tri = triangles[coplanarTriangleIndex];
                        var nProbe = Rat3Hybrid.Cross(b - a, c - a);
                        var nTri = intersector.TriangleNormal(coplanarTriangleIndex);
                        int sign = Rat3Hybrid.DotSign(in nProbe, in nTri);

                        if (sign > 0)
                            return InsideResult.CoplanarSameNormal;
                        else
                            return InsideResult.CoplanarOppositeNormal;
                    }
                    if (result == IntersectionCountInfo.EndOnSurface)
                        repeat = true;
                    if (result == IntersectionCountInfo.InvalidConfiguration)
                        repeat = true;

                    sum = c1;
                }

                if (!repeat)
                {
                    Box3I intersectionBox = new Box3I(new Int3(int.MaxValue), new Int3(int.MinValue));
                    intersectionBox.Extend(pInt);
                    intersectionBox.Extend(end);
                    intersectionBox.Enlarge(1);

                    TreeBoxOverlapEnumerator triangleIndices = new TreeBoxOverlapEnumerator(tree, intersectionBox.GetBoundsF());
                    //var triangleIdList = triangleIndices.GetList();

                    int c2;
                    int coplanarTriangleIndex;
                    var result2 = PointInMeshTester<Intersec>.CountIntersections(pInt, new Box3I(pInt, pInt), end, intersector/*points, triangles*/, triangleIndices, out c2, out coplanarTriangleIndex);
                    if (result2 == IntersectionCountInfo.StartOnSurface)
                        throw new Exception("result == IntersectionCountInfo.EndOnSurface should have thrown above!");
                    if (result2 == IntersectionCountInfo.EndOnSurface)
                        throw new Exception("Invalid position of end");
                    if (result2 == IntersectionCountInfo.InvalidConfiguration)
                        repeat = true;

                    sum = c1 + c2;
                }

                ++counter;
            }


            return sum % 2 == 0 ? InsideResult.Outside : InsideResult.Inside;
        }
    }

    public class PointInMeshTester<Intersec> where Intersec : struct, IIntersector
    {
        public static IntersectionCountInfo CountIntersections(in Rat3Hybrid segmentStart, in Box3I segmentStartBox, in Int3 segmentEnd, in Intersec intersector, IEnumerable<int> triangleIndices,
            out int rawIntersectionCounter, out int coplanarTriangleIndex)
        {
            Box3I pBox = segmentStartBox;

            Box3I segmentBox = pBox;
            segmentBox.Extend(segmentEnd);

            coplanarTriangleIndex = -1;

            rawIntersectionCounter = 0;
            //for (int indexer = 0; indexer < triangleIndices.Count; ++indexer)
            foreach (int i in triangleIndices)
            {
                //int i = triangleIndices[indexer];
                /*Triangle t = triangles[i];
                if (t.A < 0)
                    continue;*/

                /*Box3I box = new Box3I(points[t.A], points[t.B], points[t.C]);
                if (!box.OverlapOrTouch(segmentBox))
                    continue;*/

                bool intersectionIsOnTriangleBoundary, startIsOnTriangle, endIsOnTriangle;
                SegmentTriangleIntersectionType r = intersector.SegmentIntersectsTriangle(segmentStart, segmentEnd, /*points[t.A], points[t.B], points[t.C]*/i, out intersectionIsOnTriangleBoundary, out startIsOnTriangle, out endIsOnTriangle);
                switch (r)
                {
                    case SegmentTriangleIntersectionType.Coplanar:
                        return IntersectionCountInfo.InvalidConfiguration;
                    case SegmentTriangleIntersectionType.NoIntersection:
                        break;
                    case SegmentTriangleIntersectionType.Intersect:
                        if (startIsOnTriangle)
                        {
                            coplanarTriangleIndex = i;
                            return IntersectionCountInfo.StartOnSurface;
                        }
                        if (endIsOnTriangle)
                        {
                            coplanarTriangleIndex = i;
                            return IntersectionCountInfo.EndOnSurface;
                        }
                        if (intersectionIsOnTriangleBoundary)
                            return IntersectionCountInfo.InvalidConfiguration;
                        ++rawIntersectionCounter;
                        break;
                }
            }

            return IntersectionCountInfo.Normal;
        }
        public static IntersectionCountInfo CountIntersections(in Int3 segmentStart, in Box3I segmentStartBox, in Int3 segmentEnd, in Intersec intersector, IEnumerable<int> triangleIndices,
            out int rawIntersectionCounter, out int coplanarTriangleIndex)
        {
            Box3I pBox = segmentStartBox;

            Box3I segmentBox = pBox;
            segmentBox.Extend(segmentEnd);

            coplanarTriangleIndex = -1;

            rawIntersectionCounter = 0;
            //for (int indexer = 0; indexer < triangleIndices.Count; ++indexer)
            foreach (int i in triangleIndices)
            {
                //int i = triangleIndices[indexer];
                /*Triangle t = triangles[i];
                if (t.A < 0)
                    continue;*/

                /*Box3I box = new Box3I(points[t.A], points[t.B], points[t.C]);
                if (!box.OverlapOrTouch(segmentBox))
                    continue;*/

                bool intersectionIsOnTriangleBoundary, startIsOnTriangle, endIsOnTriangle;
                SegmentTriangleIntersectionType r = intersector.SegmentIntersectsTriangle(segmentStart, segmentEnd, /*points[t.A], points[t.B], points[t.C]*/i, out intersectionIsOnTriangleBoundary, out startIsOnTriangle, out endIsOnTriangle);
                switch (r)
                {
                    case SegmentTriangleIntersectionType.Coplanar:
                        return IntersectionCountInfo.InvalidConfiguration;
                    case SegmentTriangleIntersectionType.NoIntersection:
                        break;
                    case SegmentTriangleIntersectionType.Intersect:
                        if (startIsOnTriangle)
                        {
                            coplanarTriangleIndex = i;
                            return IntersectionCountInfo.StartOnSurface;
                        }
                        if (endIsOnTriangle)
                        {
                            coplanarTriangleIndex = i;
                            return IntersectionCountInfo.EndOnSurface;
                        }
                        if (intersectionIsOnTriangleBoundary)
                            return IntersectionCountInfo.InvalidConfiguration;
                        ++rawIntersectionCounter;
                        break;
                }
            }

            return IntersectionCountInfo.Normal;
        }
    }
}
