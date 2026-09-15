using GeoCore;

namespace NURBS
{
    public struct BSurfaceInfo
    {
        public bool StrU0;
        public bool StrUn;
        public bool StrV0;
        public bool StrVn;
        public bool FlatU;
        public bool FlatV;
        public double MinU;
        public double MaxU;
        public double MinV;
        public double MaxV;

        public int Index;
        public int ParentIndex;
        public bool IsLeave;


        public override string ToString()
        {
            return "L: " + IsLeave + "   Par: " + ParentIndex + "   MinU: " + MinU.ToString() + ",   MaxU: " + MaxU.ToString() + ",   MinV: " + MinV.ToString() + ",   MaxV: " + MaxV.ToString() + ",   FlatU: " + FlatU.ToString() + ",   FlatV: " + FlatV.ToString();
        }
    }

    public class AdaptiveSurfaceSplitter
    {
        public static List<Vec3D> Tessellate(BSplineCurve curve, out List<double> parameters, double start = 0, double end = 1, double tol = 5e-2, double eps = 1e-12)
        {
            if (start != 0 || end != 1)
                curve = curve.ExtractRange(start, end);

            parameters = new List<double>();
            Tessellate(curve, 0, 1, tol, eps, parameters);

            int l = parameters.Count;
            List<Vec3D> points = new List<Vec3D>(l + 1);
            for (int i = 0; i < l; ++i)
                points.Add(curve.EvaluateUniform(parameters[i]));

            points.Add(curve.EvaluateUniform(1));

            return points;
        }
        private static void Tessellate(BSplineCurve curve, double min, double max, double tol, double eps, List<double> uPoints)
        {
            bool flat = IsCurveStraight(curve, tol, eps);
            if (flat)
            {
                lock (uPoints)
                {
                    uPoints.Add(min);
                }
            }
            else
            {
                double center = 0.5 * (min + max);
                BSplineCurve lower, upper;
                curve.Split(0.5, out lower, out upper, false);
                Tessellate(lower, min, center, tol, eps, uPoints);
                Tessellate(upper, center, max, tol, eps, uPoints);
            }
        }
        public static bool IsCurveStraight(BSplineCurve n, double tol, double eps)
        {
            //Special case: lines are automatically straight
            if (n.Degree == 1)
                return true;

            int last = n.Degree;
            Vec3D e0 = GetControlPoint(0, n);

            //Form an initial line to test the other points against (skipping degenerate lines)
            Vec3D vec = default(Vec3D);
            double linelen = 0;
            for (int i = last; i > 0; --i)
            {
                Vec3D cp = GetControlPoint(i, n);
                vec = cp - e0;

                linelen = vec.Length();
                if (linelen > eps)
                    break;
            }

            if (linelen > eps)
            {
                vec.Normalize();
                double tol2 = tol * tol;
                for (int i = 1; i <= last; i++)
                {
                    Vec3D cp = GetControlPoint(i, n);
                    double s;
                    double dist = GeometricAlgorithms.SquaredDistancePointLine(cp, e0, vec, out s);

                    if (dist > tol2)
                        return false;
                }
            }
            return true;
        }
        private static Vec3D GetControlPoint(int id, BSplineCurve n)
        {
            Vec4D v = n.ControlPoints[id];
            double invW = 1.0 / v.W;
            return new Vec3D(v.X * invW, v.Y * invW, v.Z * invW);
        }







        //public unsafe long Key(double x, double y)
        //{
        //    float fx = (float)x;
        //    float fy = (float)y;

        //    int* ptr = (int*)&fx;
        //    int ix = *ptr;
        //    ptr = (int*)&fy;
        //    int iy = *ptr;

        //    return (((long)ix) << 32) | ((long)iy);
        //}

        //private class Double2ComparerXFirst : IComparer<int> { }
        //private class Double2ComparerYFirst { }
        public static int[] BuildHorizontalQuickAccessIndexer(List<Vec2D> points, out Dictionary<double, Int2> rowStartIndices)
        {
            int l = points.Count;
            int[] map = new int[l];
            for (int i = 0; i < l; ++i)
                map[i] = i;

            Array.Sort(map, delegate (int a, int b)
            {
                Vec2D pA = points[a];
                Vec2D pB = points[b];

                int c = pA.Y.CompareTo(pB.Y);
                if (c == 0)
                    return pA.X.CompareTo(pB.X);
                else
                    return c;
            });
            //ParallelMergeSort.Sort(map, yFirstComparer);

            rowStartIndices = new Dictionary<double, Int2>();
            Vec2D prev = points[map[0]];
            //rowStartIndices.Add(prev.Y, 0);
            Int2 item = new Int2(0, -1);
            for (int i = 1; i < l; ++i)
            {
                Vec2D p = points[map[i]];
                if (p.Y != prev.Y)
                {
                    //rowStartIndices.Add(p.Y, i);
                    item.Y = i - 1;
                    rowStartIndices.Add(prev.Y, item);
                    item.X = i;
                    item.Y = -1;
                    prev = p;
                }
            }
            item.Y = l - 1;
            rowStartIndices.Add(prev.Y, item);

            return map;
        }

        public static int[] BuildVerticalQuickAccessIndexer(List<Vec2D> points, out Dictionary<double, Int2> colStartIndices)
        {
            int l = points.Count;
            int[] map = new int[l];
            for (int i = 0; i < l; ++i)
                map[i] = i;

            Array.Sort(map, delegate (int a, int b)
            {
                Vec2D pA = points[a];
                Vec2D pB = points[b];

                int c = pA.X.CompareTo(pB.X);
                if (c == 0)
                    return pA.Y.CompareTo(pB.Y);
                else
                    return c;
            });

            colStartIndices = new Dictionary<double, Int2>();
            Vec2D prev = points[map[0]];
            //colStartIndices.Add(prev.X, 0);
            Int2 item = new Int2(0, -1);
            for (int i = 1; i < l; ++i)
            {
                Vec2D p = points[map[i]];
                if (p.X != prev.X)
                {
                    //colStartIndices.Add(p.X, i);
                    item.Y = i - 1;
                    colStartIndices.Add(prev.X, item);
                    item.X = i;
                    item.Y = -1;
                    prev = p;
                }
            }
            item.Y = l - 1;
            colStartIndices.Add(prev.X, item);

            return map;
        }

        public static List<Tri>[] TriangulateNoBorder(IList<BSurfaceInfo> patches, List<Vec2D> points, double tol = 1e-8, double eps = 1e-12)
        {
            //Build a kD-Tree from the patch-rectangles

            //Add border points to patch rectangles

            //Then triangulate each patch rectangle taking care of border constraints
            //Use knowledge that each border-loop is closed
            List<int>[] polygons = ExtractPolygons(patches, points, tol, eps);
            var tris = BoundaryClipper.TriangulateDefaultBorder(points, polygons);


            return tris;
        }

        public static List<Tri>[] TriangulateWithBorder2(IList<BSurfaceInfo> patches, List<Vec2D> points, List<List<int>> borderLoops, double tol = 1e-8, double eps = 1e-12)
        {
            //Build a kD-Tree from the patch-rectangles

            //Add border points to patch rectangles

            //Then triangulate each patch rectangle taking care of border constraints
            //Use knowledge that each border-loop is closed
            List<int>[] polygons = ExtractPolygons(patches, points, tol, eps);
            var tris = BoundaryClipper.ClipOutsideBoundary(points, polygons, borderLoops, tol, eps);


            return tris;
        }

        public static List<Tri>[] TriangulateWithBorder(IList<BSurfaceInfo> patches, List<Vec2D> points,
            List<List<int>> borderLoops, double tol = 1e-8, double eps = 1e-12)
        {
            //Compute the BoxTree2D using the information stored in the patches

            List<int>[] polygons = ExtractPolygons(patches, points, tol, eps);

            List<BoxNode2D> tree = new List<BoxNode2D>(patches.Count);
            int l = patches.Count;
            int indexer = 0;
            for (int i = 0; i < l; ++i)
            {
                BSurfaceInfo info = patches[i];
                //if (polygons[indexer] == null)
                //{

                //}

                BoxNode2D box = new BoxNode2D()
                {
                    MinX = info.MinU - eps,
                    MaxX = info.MaxU + eps,
                    MinY = info.MinV - eps,
                    MaxY = info.MaxV + eps,
                    IndexB = -1
                };

                if (info.IsLeave)
                {
                    box.IndexA = indexer;
                    ++indexer;
                }
                else
                    box.IndexA = -1;

                tree.Add(box);
            }
            //Establish the connections
            for (int i = 1; i < l; ++i)
            {
                BSurfaceInfo info = patches[i];
                BoxNode2D box = tree[info.ParentIndex];
                if (box.IndexA < 0)
                    box.IndexA = i;
                else if (box.IndexB < 0)
                    box.IndexB = i;
                else
                    throw new Exception();
                tree[info.ParentIndex] = box;
            }

            var tris = BoundaryClipper.ClipOutsideBoundary(points, polygons, tree, borderLoops, tol, eps);


            return tris;
        }



        //TODO: This is more or less the samce code like the one in the method below (not good, duplicate code)
        public static List<int>[] ExtractPolygons(IList<BSurfaceInfo> patches, List<Vec2D> points, double tol = 1e-8, double eps = 1e-12)
        {
            Dictionary<double, Int2> rowStartIndices = null;
            int[] horizontalMap = null;
            Task t1 = new Task(delegate ()
            {
                horizontalMap = BuildHorizontalQuickAccessIndexer(points, out rowStartIndices); //TODO: Replace rowStartIndices by binary search?
            });
            t1.Start();
            Dictionary<double, Int2> colStartIndices = null;
            int[] verticalMap = null;
            Task t2 = new Task(delegate ()
            {
                verticalMap = BuildVerticalQuickAccessIndexer(points, out colStartIndices);
            });
            t2.Start();

            t1.Wait();
            t2.Wait();



            int counter = 0;
            int l = patches.Count;
            int[] map = new int[l];
            for (int i = 0; i < l; ++i)
            {
                BSurfaceInfo info = patches[i];
                if (info.IsLeave)
                {
                    map[counter] = i;
                    ++counter;
                }
            }


            List<int>[] result = new List<int>[counter];
            //TODO: Use parallel tasks, this is PERFECTLY parallelizable!
            //for (int i = 0; i < l; ++i)
            Parallel.For(0, counter, j =>
            {
                int i = map[j];
                BSurfaceInfo info = patches[i];
                result[j] = ExtractBorderPointIndices(info, points, horizontalMap, rowStartIndices, verticalMap, colStartIndices);
            });
            return result;
        }

        //points should not contain duplicates!
        public static List<Tri> TriangulateIgnoringBorder(IList<BSurfaceInfo> patches, List<Vec2D> points, double tol = 1e-8, double eps = 1e-12)
        {
            List<Tri> result = new List<Tri>();

            Dictionary<double, Int2> rowStartIndices = null;
            int[] horizontalMap = null;
            Task t1 = new Task(delegate ()
            {
                horizontalMap = BuildHorizontalQuickAccessIndexer(points, out rowStartIndices); //TODO: Replace rowStartIndices by binary search?
            });
            t1.Start();
            Dictionary<double, Int2> colStartIndices = null;
            int[] verticalMap = null;
            Task t2 = new Task(delegate ()
            {
                verticalMap = BuildVerticalQuickAccessIndexer(points, out colStartIndices);
            });
            t2.Start();

            t1.Wait();
            t2.Wait();


            int l = patches.Count;
            //TODO: Use parallel tasks, this is PERFECTLY parallelizable!
            //for (int i = 0; i < l; ++i)
            Parallel.For(0, l, i =>
            {
                BSurfaceInfo info = patches[i];
                List<int> borderPoints = ExtractBorderPointIndices(info, points, horizontalMap, rowStartIndices, verticalMap, colStartIndices);

                List<Tri> tris = Triangulator.TriangulatePolygon(points, borderPoints/*, tol, eps*/);

#if DEBUG

                //for (int j = 0; j < tris.Count; ++j)
                //{
                //    Tri tri = tris[j];
                //    if (tri.A == tri.B || tri.B == tri.C || tri.C == tri.A)
                //    {

                //    }
                //}

                //                //Debug
                //                for (int j = 0; j < tris.Count; ++j)
                //                {
                //                    Tri tri = tris[j];
                //                    Vec2D a = points[tri.A];
                //                    Vec2D b = points[tri.B];
                //                    Vec2D c = points[tri.C];

                //                    b = b - a;
                //                    c = c - a;
                //                    double area = b.X * c.Y - b.Y * c.X;
                //                    if (area < 1e-12)
                //                    {

                //                    }
                //                }
#endif

                lock (result)
                {
                    result.AddRange(tris);
                }
            });
            return result;
        }

        //        public static List<Tri> Triangulate(IList<BSurfaceInfo> patches, List<Vec2D> points, List<List<int>> borderLoops, double tol = 1e-8, double eps = 1e-12, double largeTolerance = 1e-6)
        //        {
        //            List<Tri> trisNoBorder = TriangulateIgnoringBorder(patches, points, tol, eps);

        //#if DEBUG
        //            //Debug
        //            int l = trisNoBorder.Count;
        //            for (int i = 0; i < l; ++i)
        //            {
        //                Tri tri = trisNoBorder[i];
        //                Vec2D a = points[tri.A];
        //                Vec2D b = points[tri.B];
        //                Vec2D c = points[tri.C];

        //                b = b - a;
        //                c = c - a;
        //                double area = b.X * c.Y - b.Y * c.X;
        //                if (area < 1e-12)
        //                {

        //                }
        //            }
        //#endif

        //            return BoundaryClipper.ClipOutsideBoundary(points, trisNoBorder, borderLoops, tol, eps, largeTolerance, false);
        //        }

        //public static List<Tri> ClipOutsideBoundary(List<Vec2D> points, List<Tri> triangles, List<List<int>> borderLoops, double tol = 1e-8, double eps = 1e-12)
        //{
        //    throw new NotImplementedException();
        //}

        private static void ReverseRange<T>(List<T> list, int start, int end)
        {
            int half = (end - start) >> 1;

            int l = end - 1;

            for (int i = 0; i < half; ++i)
            {
                T tmp = list[start + i];
                list[start + i] = list[l - i];
                list[l - i] = tmp;
            }
        }



        private static int BinarySearchX(int lower, int upper, int[] map, List<Vec2D> points, double x)
        {
            double lo = points[map[lower]].X;
            double up = points[map[upper]].X;

            if (lo == x)
                return lower;
            if (up == x)
                return upper;

            while (true)
            {
                if (upper == lower)
                    return lower;

                int center = (lower + upper) >> 1;
                double c = points[map[center]].X;

                if (c == x)
                    return center;

                if (c < x)
                {
                    lower = center;
                    lo = c;
                }
                else
                {
                    upper = center;
                    up = c;
                }
            }
        }

        private static int BinarySearchY(int lower, int upper, int[] map, List<Vec2D> points, double y)
        {
            double lo = points[map[lower]].Y;
            double up = points[map[upper]].Y;

            if (lo == y)
                return lower;
            if (up == y)
                return upper;

            while (true)
            {
                if (upper == lower)
                    return lower;

                int center = (lower + upper) >> 1;
                double c = points[map[center]].Y;

                if (c == y)
                    return center;

                if (c < y)
                {
                    lower = center;
                    lo = c;
                }
                else
                {
                    upper = center;
                    up = c;
                }
            }
        }


        //points should not contain duplicates!
        public static List<int> ExtractBorderPointIndices(BSurfaceInfo info, List<Vec2D> points,
            int[] horizontalMap, Dictionary<double, Int2> rowStartIndices, int[] verticalMap, Dictionary<double, Int2> colStartIndices)
        {
            int l = points.Count;

            //TODO: Recycle the four following lines!
            //Dictionary<double, int> rowStartIndices;
            //int[] horizontalMap = BuildHorizontalQuickAccessIndexer(points, out rowStartIndices); //TODO: Replace rowStartIndices by binary search?
            //Dictionary<double, int> colStartIndices;
            //int[] verticalMap = BuildVerticalQuickAccessIndexer(points, out colStartIndices);

            Int2 rowStartTop = rowStartIndices[info.MinV];
            Int2 rowStartBot = rowStartIndices[info.MaxV];
            Int2 colStartLeft = colStartIndices[info.MinU];
            Int2 colStartRight = colStartIndices[info.MaxU];

            int startTop = BinarySearchX(rowStartTop.X, rowStartTop.Y, horizontalMap, points, info.MinU);
            int endTop = BinarySearchX(startTop, rowStartTop.Y, horizontalMap, points, info.MaxU);

            int startBot = BinarySearchX(rowStartBot.X, rowStartBot.Y, horizontalMap, points, info.MinU);
            int endBot = BinarySearchX(startBot, rowStartBot.Y, horizontalMap, points, info.MaxU);


            int startLeft = BinarySearchY(colStartLeft.X, colStartLeft.Y, verticalMap, points, info.MinV);
            int endLeft = BinarySearchY(startLeft, colStartLeft.Y, verticalMap, points, info.MaxV);

            int startRight = BinarySearchY(colStartRight.X, colStartRight.Y, verticalMap, points, info.MinV);
            int endRight = BinarySearchY(startRight, colStartRight.Y, verticalMap, points, info.MaxV);


            List<int> result = new List<int>(endTop - startTop + endBot - startBot + endLeft - startLeft + endRight - startRight);
            for (int i = startTop; i < endTop; ++i)
                result.Add(horizontalMap[i]);

            for (int i = startRight; i < endRight; ++i)
                result.Add(verticalMap[i]);

            for (int i = endBot; i > startBot; --i)
                result.Add(horizontalMap[i]);

            for (int i = endLeft; i > startLeft; --i)
                result.Add(verticalMap[i]);

            return result;
        }





        ////points should not contain duplicates!
        //public static List<int> ExtractBorderPointIndices(BSurfaceInfo info, List<Vec2D> points,
        //    int[] horizontalMap, Dictionary<double, Int2> rowStartIndices, int[] verticalMap, Dictionary<double, Int2> colStartIndices)
        //{
        //    int l = points.Count;

        //    //TODO: Recycle the four following lines!
        //    //Dictionary<double, int> rowStartIndices;
        //    //int[] horizontalMap = BuildHorizontalQuickAccessIndexer(points, out rowStartIndices); //TODO: Replace rowStartIndices by binary search?
        //    //Dictionary<double, int> colStartIndices;
        //    //int[] verticalMap = BuildVerticalQuickAccessIndexer(points, out colStartIndices);

        //    int rowStartTop = rowStartIndices[info.MinV].X;
        //    int rowStartBottom = rowStartIndices[info.MaxV].X;
        //    int colStartLeft = colStartIndices[info.MinU].X;
        //    int colStartRight = colStartIndices[info.MaxU].X;

        //    List<int> result = new List<int>(16); //Just an estimate for the size
        //    for (int i = rowStartTop; i < l; ++i)
        //    {
        //        int j = horizontalMap[i];
        //        Vec2D p = points[j];

        //        if (p.Y != info.MinV || p.X > info.MaxU) //binary search?
        //            break;

        //        if (p.X >= info.MinU /*&& p.X < info.MaxU*/)
        //            result.Add(j);
        //    }

        //    for (int i = colStartRight; i < l; ++i)
        //    {
        //        int j = verticalMap[i];
        //        Vec2D p = points[j];

        //        if (p.Y >= info.MaxV) //binary search?
        //            break;

        //        if (p.Y > info.MinV /*&& p.Y < info.MaxV*/)
        //            result.Add(j);
        //    }

        //    //List<int> buffer = new List<int>();
        //    int start = result.Count;
        //    for (int i = rowStartBottom; i < l; ++i)
        //    {
        //        int j = horizontalMap[i];
        //        Vec2D p = points[j];

        //        if (p.Y != info.MaxV || p.X > info.MaxU)
        //            break;

        //        if (p.X >= info.MinU)
        //            result.Add(j);
        //    }
        //    //buffer.Reverse();
        //    //result.AddRange(buffer);
        //    ReverseRange(result, start, result.Count);

        //    //buffer.Clear();
        //    start = result.Count;
        //    for (int i = colStartLeft; i < l; ++i)
        //    {
        //        int j = verticalMap[i];
        //        Vec2D p = points[j];

        //        if (p.Y >= info.MaxV)
        //            break;

        //        if (p.Y > info.MinV)
        //            result.Add(j);
        //    }
        //    //buffer.Reverse();
        //    //result.AddRange(buffer);
        //    ReverseRange(result, start, result.Count);

        //    return result;
        //}




        public enum NurbsCorner
        {
            U0V0,
            U0V1,
            U1V0,
            U1V1
        }

        public struct PosNorUV
        {
            //public Vec3D Position;
            //public Vec3D Normal;
            public readonly Vec2D UV;
            private readonly NurbsSurfaceData surface;
            private readonly NurbsCorner corner;

            //private Vec3D debug;

            public PosNorUV(NurbsSurfaceData surface, Vec2D uv, NurbsCorner corner/*, Vec3D debug*/)
            {
                this.surface = surface;
                this.corner = corner;
                UV = uv;
                //this.debug = debug;
            }

            public Vec3D GetPoint()
            {
                switch (corner)
                {
                    case NurbsCorner.U0V0:
                        return surface.GetPointMinUMinV();
                    case NurbsCorner.U0V1:
                        return surface.GetPointMinUMaxV();
                    case NurbsCorner.U1V0:
                        return surface.GetPointMaxUMinV();
                    default:
                    case NurbsCorner.U1V1:
                        return surface.GetPointMaxUMaxV();
                }
            }

            public Vec3D GetNormal()
            {
                switch (corner)
                {
                    case NurbsCorner.U0V0:
                        return surface.GetNormalMinUMinV();
                    case NurbsCorner.U0V1:
                        return surface.GetNormalMinUMaxV();
                    case NurbsCorner.U1V0:
                        return surface.GetNormalMaxUMinV();
                    default:
                    case NurbsCorner.U1V1:
                        return surface.GetNormalMaxUMaxV();
                }
            }

            public Vec3D GetPointAndNormal(out Vec3D normal)
            {
                switch (corner)
                {
                    case NurbsCorner.U0V0:
                        return surface.GetPointAndNormalMinUMinV(out normal);
                    case NurbsCorner.U0V1:
                        return surface.GetPointAndNormalMinUMaxV(out normal);
                    case NurbsCorner.U1V0:
                        return surface.GetPointAndNormalMaxUMinV(out normal);
                    default:
                    case NurbsCorner.U1V1:
                        return surface.GetPointAndNormalMaxUMaxV(out normal);
                }
            }
        }


        private static List<PosNorUV> BSurfaceInsertionPoints(NurbsSurfaceData surface, double approxTolerance = 5e-2, double eps = 1e-12)
        {
            List<BSurfaceInfo> surfaceInfos;
            return BSurfaceInsertionPoints(surface, out surfaceInfos, approxTolerance, eps);
        }


        private static int Compare(PosNorUV a, PosNorUV b) { return a.UV.X == b.UV.X ? a.UV.Y.CompareTo(b.UV.Y) : a.UV.X.CompareTo(b.UV.X); }

        private unsafe static ulong ToLongMask(double d)
        {
            float f = (float)d;
            float* fPtr = &f;
            uint* iPtr = (uint*)fPtr;
            return (ulong)*iPtr;
        }


        private static List<PosNorUV> BSurfaceInsertionPoints(NurbsSurfaceData surface, out List<BSurfaceInfo> surfaceInfos,
            double approxTolerance = 5e-2, double eps = 1e-12, double maxSpanU = 1.0, double maxSpanV = 1.0)
        {
            surfaceInfos = new List<BSurfaceInfo>();
            List<PosNorUV> uvPoints = new List<PosNorUV>();

            //if (minNumSplitsUBase2Exponent == 0 && minNumSplitsVBase2Exponent == 0)
            //{
            BSurfaceInfo info = new BSurfaceInfo()
            {
                FlatU = false,
                FlatV = false,
                StrU0 = false,
                StrV0 = false,
                StrUn = false,
                StrVn = false,
                MaxU = 1,
                MinU = 0,
                MaxV = 1,
                MinV = 0,
                ParentIndex = -1,
            };
            List<Task> tasks = new List<Task>();
            /*List<BSurfaceInfo>*/
            BSurfaceInsertionPoints(surface, info, 0, 3/*5*/, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV);
            for (int i = 0; i < tasks.Count; ++i)
                tasks[i].Wait();
            //}
            //else
            //{
            //    throw new Exception();
            //    //int numStepsU = 1 << minNumSplitsUBase2Exponent; // Power(2, minNumSplitsUBase2Exponent);
            //    //int numStepsV = 1 << minNumSplitsVBase2Exponent; // Power(2, minNumSplitsVBase2Exponent);

            //    //for (int i = 0; i < numStepsU; ++i)
            //    //{
            //    //    double uStart = (double)i / numStepsU;
            //    //    double uEnd = (double)(i + 1) / numStepsU;
            //    //    for (int j = 0; j < numStepsV; ++j)
            //    //    {
            //    //        double vStart = (double)j / numStepsV;
            //    //        double vEnd = (double)(j + 1) / numStepsV;



            //    //        List<Task> tasks = new List<Task>();
            //    //        /*List<BSurfaceInfo>*/
            //    //        BSurfaceInsertionPoints(surface, info, 0, -1, approxTolerance, eps, uvPoints, surfaceInfos, tasks);
            //    //        for (int k = 0; k < tasks.Count; ++k)
            //    //            tasks[i].Wait();
            //    //    }
            //    //}
            //}




            //if(uvPoints.Count > 4)
            //{
            //    //Debug
            //}
            //Now sort and remove duplicates
            //List<Vec2D> duplicateFreeUVPoints = uvPoints.Distinct().ToList();//ParallelEnumerable.Distinct()
            List<PosNorUV> duplicateFreeUVPoints = new List<PosNorUV>(uvPoints.Count);
            //uvPoints.Sort(Compare/*delegate (PosNorUV a, PosNorUV b) { return a.UV.X == b.UV.X ? a.UV.Y.CompareTo(b.UV.Y) : a.UV.X.CompareTo(b.UV.X); }*/);



            //long[] buffer = new long[uvPoints.Count];
            //for (int i = 0; i < buffer.Length; ++i)
            //{
            //    PosNorUV p = uvPoints[i];
            //    buffer[i] = ((long)(1.0 / p.UV.X + 0.5) << 32) | (long)(1.0 / p.UV.Y + 0.5);
            //}
            //int[] indexer = new int[uvPoints.Count];
            //for (int i = 0; i < buffer.Length; ++i)
            //    indexer[i] = i;

            //Array.Sort(buffer, indexer);

            ulong[] buffer = new ulong[uvPoints.Count];
            for (int i = 0; i < buffer.Length; ++i)
            {
                PosNorUV p = uvPoints[i];
                buffer[i] = (ToLongMask(p.UV.X) << 32) | ToLongMask(p.UV.Y);
            }
            int[] indexer = new int[uvPoints.Count];
            for (int i = 0; i < buffer.Length; ++i)
                indexer[i] = i;

            Array.Sort(buffer, indexer);



            //DualPivotQuickSort.Sort(uvPoints, Compare); //Does not seem to work properly...
            PosNorUV prev = default(PosNorUV);
            if (uvPoints.Count > 0)
            {
                prev = uvPoints[/*0*/indexer[0]];
                duplicateFreeUVPoints.Add(prev);
            }
            int l = uvPoints.Count;
            for (int i = 1; i < l; ++i)
            {
                PosNorUV p = uvPoints[/*i*/indexer[i]];
                if (p.UV.X != prev.UV.X || p.UV.Y != prev.UV.Y)
                {
                    duplicateFreeUVPoints.Add(p);
                    prev = p;
                }
            }


            //List<Vec2D> debug = uvPoints.AsParallel().Distinct().ToList();






            //for (int i = 0; i < duplicateFreeUVPoints.Count; ++i)
            //{
            //    Vec2D a = duplicateFreeUVPoints[i];
            //    for (int j = i+1; j < duplicateFreeUVPoints.Count; ++j)
            //    {
            //        Vec2D b = duplicateFreeUVPoints[j];
            //        double dx = b.X - a.X;
            //        double dy = b.Y - a.Y;
            //        if(dx*dx+dy*dy<1e-16)
            //        {

            //        }
            //    }
            //}

            return duplicateFreeUVPoints;
        }

        //private Vec3D GetPt(int i, int crvInd, bool dirflag) { dirflag ?  }
        //GPU Gems IV Page 313
        //dirflag == true  ->  test in U direction
        //dirflag == false  ->  test in V direction
        //surface.Nu
        //public static bool IsCurveStraightGPUGems(BSurface n, double approxTolerance, int crvInd, bool dirflag, double eps)
        //{
        //    Func<int, Vec3D> GETPT = delegate(int id)
        //    {
        //        //SPole v = dirflag ? n.GetPole(crvInd, id) : n.GetPole(id, crvInd);
        //        SPole v = dirflag ? n.GetPole(id, crvInd) : n.GetPole(crvInd, id);
        //        double invW = 1.0 / v[3];
        //        return new Vec3D(v[0] * invW, v[1] * invW, v[2] * invW);
        //    };

        //    Vec3D p, prod;
        //    Vec3D vec = default(Vec3D);
        //    Vec3D cp, e0;
        //    int i, last;
        //    double linelen, dist;

        //    //Special case: lines are automatically straight
        //    if ((dirflag ? n.Nu : n.Nv) == 2)
        //        return true;

        //    last = (dirflag ? n.Nu : n.Nv) - 1;
        //    e0 = GETPT(0);

        //    //Form an initial line to test the other points against (skipping degenerate lines)
        //    linelen = 0;
        //    for (i = last; i > 0 && linelen < eps; i--)
        //    {
        //        cp = GETPT(i);
        //        vec = cp - e0;

        //        linelen = vec.Length;
        //    }

        //    vec = vec / linelen;
        //    if (linelen > eps)
        //    {
        //        for (i = 1; i <= last; i++)
        //        {
        //            //The cross product of the vector defining the intial line with ghe vector
        //            //of the current point gives the distance to the line
        //            cp = GETPT(i);
        //            p = cp - e0;
        //            prod = Vec3DOps.Cross(p, vec);
        //            dist = prod.Length;

        //            if (dist > approxTolerance)
        //                return false;
        //        }
        //    }
        //    return true;
        //}

        private static Vec3D GetControlPoint(int id, NurbsSurfaceData n, int crvInd, bool uDirection)
        {
            Vec4D v = uDirection ? n.Poles[id][crvInd] : n.Poles[crvInd][id];
            double invW = v.W == 1.0 ? 1.0 : 1.0 / v.W;
            return new Vec3D() { X = v.X * invW, Y = v.Y * invW, Z = v.Z * invW };
        }

        private static Vec3D GetControlPointU(int id, NurbsSurfaceData n, int crvInd)
        {
            Vec4D v = n.Poles[id][crvInd];
            //double invW = v.W == 1.0 ? 1.0 : 1.0 / v.W;
            //return new Vec3D() { X = v.X * invW, Y = v.Y * invW, Z = v.Z * invW };
            if (v.W == 1.0)
                return new Vec3D(v[0], v[1], v[2]);
            else
            {
                double invW = 1.0 / v[3];
                return new Vec3D(v[0] * invW, v[1] * invW, v[2] * invW);
            }
        }

        private static Vec3D GetControlPointV(int id, NurbsSurfaceData n, int crvInd)
        {
            Vec4D v = n.Poles[crvInd][id];
            //double invW = v.W == 1.0 ? 1.0 : 1.0 / v.W;
            //return new Vec3D() { X = v.X * invW, Y = v.Y * invW, Z = v.Z * invW };
            if (v.W == 1.0)
                return new Vec3D(v[0], v[1], v[2]);
            else
            {
                double invW = 1.0 / v[3];
                return new Vec3D(v[0] * invW, v[1] * invW, v[2] * invW);
            }
        }

        private static Vec3D GetControlPoint(Vec4D v)
        {
            //double invW = v.W == 1.0 ? 1.0 : 1.0 / v[3];
            //return new Vec3D(v[0] * invW, v[1] * invW, v[2] * invW);
            if (v.W == 1.0)
                return new Vec3D(v[0], v[1], v[2]);
            else
            {
                double invW = 1.0 / v[3];
                return new Vec3D(v[0] * invW, v[1] * invW, v[2] * invW);
            }
        }

        private static bool IsVIsoCurveStraight(NurbsSurfaceData n, double approxTolerance, int vIsoValue, double eps)
        {
            //If v is constant then the variable direction is u
            return IsCurveStraight(n, approxTolerance, vIsoValue, true, eps);
        }

        private static bool IsUIsoCurveStraight(NurbsSurfaceData n, double approxTolerance, int uIsoValue, double eps)
        {
            //If u is constant then the variable direction is v
            return IsCurveStraight(n, approxTolerance, uIsoValue, false, eps);
        }

        private static bool IsCurveStraight(NurbsSurfaceData n, double approxTolerance, int crvInd, bool uDirection, double eps)
        {
            //Special case: lines are automatically straight
            if ((uDirection ? n.Nu : n.Nv) == 2)
                return true;

            int last = (uDirection ? n.Nu : n.Nv) - 1;
            Vec3D e0 = GetControlPoint(0, n, crvInd, uDirection);

            //Form an initial line to test the other points against (skipping degenerate lines)
            Vec3D vec = default(Vec3D);
            double linelen = 0;
            double eps2 = eps * eps;
            for (int i = last; i > 0; --i)
            {
                Vec3D cp = GetControlPoint(i, n, crvInd, uDirection);
                vec = cp - e0;

                linelen = vec.LengthSquared();
                if (linelen > eps2)
                    break;
            }

            if (linelen > eps2)
            {
                //vec.Normalize();
                double tol2 = approxTolerance * approxTolerance;
                for (int i = 1; i <= last; i++)
                {
                    Vec3D cp = GetControlPoint(i, n, crvInd, uDirection);
                    double dist = GeometricAlgorithms.DistancePointLineSquared(cp, e0, vec);

                    if (dist > tol2)
                        return false;
                }
            }
            return true;
        }



        ////GPU Gems IV Page 314
        //public static bool TestFlatGPUGems(NurbsSurfaceData surface, ref BSurfaceInfo n, double approxTolerance, double eps)
        //{
        //    Func<Vec4D, Vec3D> ScreenProject = delegate(Vec4D v)
        //    {
        //        double invW = 1.0 / v[3];
        //        return new Vec3D(v[0] * invW, v[1] * invW, v[2] * invW);
        //    };

        //    Func<NurbsSurfaceData, int> maxU = delegate(NurbsSurfaceData s)
        //    {
        //        return s.Nu - 1;
        //    };
        //    Func<NurbsSurfaceData, int> maxV = delegate(NurbsSurfaceData s)
        //    {
        //        return s.Nv - 1;
        //    };

        //    //double maxU = s.Nu - 1;
        //    //double maxV = 

        //    int i;
        //    bool straight;
        //    Vec3D cp00, cp0n, cpn0, cpnn, planeEqn;
        //    double dist, d;

        //    //Check edge straightness
        //    if (!n.StrU0)
        //        n.StrU0 = IsCurveStraight(surface, approxTolerance, 0, false, eps);
        //    if (!n.StrUn)
        //        n.StrUn = IsCurveStraight(surface, approxTolerance, maxU(surface), false, eps);
        //    if (!n.StrV0)
        //        n.StrV0 = IsCurveStraight(surface, approxTolerance, 0, true, eps);
        //    if (!n.StrVn)
        //        n.StrVn = IsCurveStraight(surface, approxTolerance, maxV(surface), true, eps);

        //    //Test to make sure control points are straight in U and V
        //    straight = true;
        //    if (!n.FlatU && n.StrV0 && n.StrVn)
        //        for (i = 1; i < maxV(surface) && (straight = IsCurveStraight(surface, approxTolerance, i, true, eps)); i++) ;

        //    if (straight && n.StrV0 && n.StrVn)
        //        n.FlatU = true;

        //    //Page 315
        //    straight = true;
        //    if (!n.FlatV && n.StrU0 && n.StrUn)
        //        for (i = 1; i < maxU(surface) && (straight = IsCurveStraight(surface, approxTolerance, i, false, eps)); i++) ;

        //    if (straight && n.StrU0 && n.StrUn)
        //        n.FlatV = true;

        //    if (!n.FlatV || !n.FlatU)
        //        return false;

        //    //The surface can pass the above tests but still be twisted
        //    cp00 = ScreenProject(surface.Poles[0][0]);
        //    cp0n = ScreenProject(surface.Poles[0][maxV(surface)]);
        //    cpn0 = ScreenProject(surface.Poles[maxU(surface)][0]);
        //    cpnn = ScreenProject(surface.Poles[maxU(surface)][maxV(surface)]);

        //    cp0n = cp0n - cp00; //Make edges into vectors
        //    cpn0 = cpn0 - cpn0;

        //    //Compute the plane equation from two adjacent sides, and measure the distance from the far point
        //    //to the plane. If it's larger than tolerance, the surface is twisted

        //    planeEqn = Vec3DOps.Cross(cpn0, cp0n);
        //    planeEqn.Normalize();

        //    d = Vec3DOps.Dot(planeEqn, cp00);
        //    dist = Math.Abs(Vec3DOps.Dot(planeEqn, cpnn) - d);

        //    if (dist > approxTolerance) // Surface is twisted)
        //        return false;
        //    else
        //        return true;
        //}

        private static bool TestFlat(NurbsSurfaceData surface, ref BSurfaceInfo n, double approxTolerance, double eps, double maxSpanU, double maxSpanV)
        {
            if (n.MaxU - n.MinU > maxSpanU || n.MaxV - n.MinV > maxSpanV)
                return false;

            int maxU = surface.Nu - 1;
            int maxV = surface.Nv - 1;

            //Check edge straightness
            if (!n.StrU0)
                n.StrU0 = IsUIsoCurveStraight(surface, approxTolerance, 0, eps);
            if (!n.StrUn)
                n.StrUn = IsUIsoCurveStraight(surface, approxTolerance, maxU, eps);
            if (!n.StrV0)
                n.StrV0 = IsVIsoCurveStraight(surface, approxTolerance, 0, eps);
            if (!n.StrVn)
                n.StrVn = IsVIsoCurveStraight(surface, approxTolerance, maxV, eps);

            //Test to make sure control points are straight in U and V
            bool straight = true;
            if (!n.FlatU && n.StrV0 && n.StrVn)
                for (int i = 1; i < maxV; i++)
                {
                    straight = IsVIsoCurveStraight(surface, approxTolerance, i, eps);
                    if (!straight)
                        break;
                }

            if (straight && n.StrV0 && n.StrVn)
                n.FlatU = true;

            //Page 315
            straight = true;
            if (!n.FlatV && n.StrU0 && n.StrUn)
                for (int i = 1; i < maxU; i++)
                {
                    straight = IsUIsoCurveStraight(surface, approxTolerance, i, eps);
                    if (!straight)
                        break;
                }

            if (straight && n.StrU0 && n.StrUn)
                n.FlatV = true;

            if (!n.FlatV || !n.FlatU)
                return false;

            //The surface can pass the above tests but still be twisted
            Vec3D a = GetControlPoint(surface.Poles[0][0]);
            Vec3D b = GetControlPoint(surface.Poles[maxU][0]);
            Vec3D c = GetControlPoint(surface.Poles[0][maxV]);
            Vec3D d = GetControlPoint(surface.Poles[maxU][maxV]);

            if (DistanceToPlane(d, a, b, c) > approxTolerance) // Surface is twisted)
                return false;
            if (DistanceToPlane(c, a, b, d) > approxTolerance) // Surface is twisted)
                return false;
            if (DistanceToPlane(a, b, c, d) > approxTolerance) // Surface is twisted)
                return false;
            if (DistanceToPlane(b, a, c, d) > approxTolerance) // Surface is twisted)
                return false;

            return true;
        }

        private static double DistanceToPlane(Vec3D p, Vec3D pointOnPlaneA, Vec3D pointOnPlaneB, Vec3D pointOnPlaneC)
        {
            //Vec3D ab = pointOnPlaneB - pointOnPlaneA;
            //Vec3D ac = pointOnPlaneC - pointOnPlaneA;
            double abX = pointOnPlaneB.X - pointOnPlaneA.X; double abY = pointOnPlaneB.Y - pointOnPlaneA.Y; double abZ = pointOnPlaneB.Z - pointOnPlaneA.Z;
            double acX = pointOnPlaneC.X - pointOnPlaneA.X; double acY = pointOnPlaneC.Y - pointOnPlaneA.Y; double acZ = pointOnPlaneC.Z - pointOnPlaneA.Z;
            //Vec3D normal = Vec3DOps.Cross(ab, ac);
            double normalX = abY * acZ - abZ * acY;
            double normalY = abZ * acX - abX * acZ;
            double normalZ = abX * acY - abY * acX;
            //normal.Normalize();
            double s = 1.0 / Math.Sqrt(normalX * normalX + normalY * normalY + normalZ * normalZ);
            normalX *= s;
            normalY *= s;
            normalZ *= s;
            double d = pointOnPlaneA.X * normalX + pointOnPlaneA.Y * normalY + pointOnPlaneA.Z * normalZ;// Vec3DOps.Dot(pointOnPlaneA, normal);
            double dist = Math.Abs(/*Vec3DOps.Dot(normal, p)*/normalX * p.X + normalY * p.Y + normalZ * p.Z - d);
            return dist;
        }



        //private static void BSurfaceInsertionPointsTask(NurbsSurfaceData surface, BSurfaceInfo info, int depth, int splitDepth, double approxTolerance, double eps,
        //    List<PosNorUV> uvPoints, List<BSurfaceInfo> surfaceInfos, List<Task> tasks, double maxSpanU, double maxSpanV)
        //{
        //    if (splitDepth == depth)
        //    {
        //        Task t = new Task(delegate ()
        //        {
        //            BSurfaceInsertionPoints(surface, info, depth, -1, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV);
        //        });
        //        t.Start();
        //        lock (tasks)
        //        {
        //            tasks.Add(t);
        //        }
        //    }
        //    else
        //    {
        //        BSurfaceInsertionPoints(surface, info, depth, splitDepth, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV);
        //    }
        //}

        private static void BSurfaceInsertionPoints(NurbsSurfaceData surface, BSurfaceInfo info, int depth, int splitDepth, double approxTolerance, double eps,
            List<PosNorUV> uvPoints, List<BSurfaceInfo> surfaceInfos, List<Task> tasks, double maxSpanU, double maxSpanV)
        {
            Stack<SplitterStackItem> stack = new Stack<SplitterStackItem>();
            stack.Push(new SplitterStackItem(surface, info, depth));


            while (stack.Count > 0)
            {
                var current = stack.Pop();
                surface = current.NurbsData; //TODO: Not parallelizable...
                info = current.SurfaceInfo;
                depth = current.Depth;

                /*if (splitDepth == depth)
                {
                    Task t = new Task(delegate ()
                    {
                        BSurfaceInsertionPoints(surface, info, depth, -1, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV);
                    });
                    t.Start();
                    lock (tasks)
                    {
                        tasks.Add(t);
                    }
                    continue;
                }*/



                bool flat = TestFlat(surface, ref info, approxTolerance, eps, maxSpanU, maxSpanV);
                bool rangleTooSmall = info.MaxU - info.MinU < 1e-3 || info.MaxV - info.MinV < 1e-3;//This line is for safety if the surface ist "strange"
                                                                                                   //bool rangleTooSmall = info.MaxU - info.MinU < 1e-4 || info.MaxV - info.MinV < 1e-4;//This line is for safety if the surface ist "strange"
                if (flat || rangleTooSmall)
                {
                    lock (uvPoints)
                    {
                        uvPoints.Add(new PosNorUV(surface, new Vec2D(info.MinU, info.MinV), NurbsCorner.U0V0/*, surface.GetPointMinUMinV()*/));
                        uvPoints.Add(new PosNorUV(surface, new Vec2D(info.MinU, info.MaxV), NurbsCorner.U0V1/*, surface.GetPointMinUMaxV()*/));
                        uvPoints.Add(new PosNorUV(surface, new Vec2D(info.MaxU, info.MinV), NurbsCorner.U1V0/*, surface.GetPointMaxUMinV()*/));
                        uvPoints.Add(new PosNorUV(surface, new Vec2D(info.MaxU, info.MaxV), NurbsCorner.U1V1/*, surface.GetPointMaxUMaxV()*/));
                    }
                    lock (surfaceInfos)
                    {
                        info.ParentIndex = info.Index;
                        info.Index = surfaceInfos.Count;
                        info.IsLeave = true;
                        surfaceInfos.Add(info);
                    }
                }
                else
                {
                    lock (surfaceInfos)
                    {
                        info.ParentIndex = info.Index;
                        info.Index = surfaceInfos.Count;
                        info.IsLeave = false;
                        surfaceInfos.Add(info);
                    }

                    //Split and start recursion call  
                    bool splitU = true;
                    if (info.FlatU)
                    {
                        splitU = false;
                    }
                    else if (info.FlatV)
                    {
                        splitU = true;
                    }
                    else if (info.MaxV - info.MinV > info.MaxU - info.MinU)
                    {
                        splitU = false;
                    }
                    else
                    {
                        splitU = true;
                    }

                    BSurfaceInfo l, r;
                    NurbsSurfaceData lower, upper;
                    if (splitU)
                    {
                        //SplitU(surface, info, depth, splitDepth, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV);
                        double centerU = 0.5 * (info.MinU + info.MaxU);

                        l = info;
                        l.MinU = info.MinU; l.MaxU = centerU; l.MinV = info.MinV; l.MaxV = info.MaxV;

                        r = info;
                        r.MinU = centerU; r.MaxU = info.MaxU; r.MinV = info.MinV; r.MaxV = info.MaxV;

                        surface.SplitU(0.5, out lower, out upper);

                        //BSurfaceInsertionPointsTask(lower, l, depth + 1, splitDepth, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.A*/);
                        //BSurfaceInsertionPointsTask(upper, r, depth + 1, splitDepth, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.B*/);
                    }
                    else
                    {
                        //SplitV(surface, info, depth, splitDepth, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV);
                        double centerV = 0.5 * (info.MinV + info.MaxV);

                        l = info;
                        l.MinU = info.MinU; l.MaxU = info.MaxU; l.MinV = info.MinV; l.MaxV = centerV;

                        r = info;
                        r.MinU = info.MinU; r.MaxU = info.MaxU; r.MinV = centerV; r.MaxV = info.MaxV;

                        surface.SplitV(0.5, out lower, out upper);

                        //BSurfaceInsertionPointsTask(lower, l, depth + 1, splitDepth, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.A*/);
                        //BSurfaceInsertionPointsTask(upper, r, depth + 1, splitDepth, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.B*/);
                    }


                    //BSurfaceInsertionPointsTask(lower, l, depth + 1, splitDepth, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.A*/);
                    //BSurfaceInsertionPointsTask(upper, r, depth + 1, splitDepth, approxTolerance, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.B*/);

                    stack.Push(new SplitterStackItem(upper, r, depth + 1));
                    stack.Push(new SplitterStackItem(lower, l, depth + 1));
                }
            }
        }

        private struct SplitterStackItem
        {
            public NurbsSurfaceData NurbsData;
            public BSurfaceInfo SurfaceInfo;
            public int Depth;

            public SplitterStackItem(NurbsSurfaceData nurbsData, BSurfaceInfo surfaceInfo, int depth)
            {
                NurbsData = nurbsData;
                SurfaceInfo = surfaceInfo;
                Depth = depth;
            }
        }


        //private static void SplitU(NurbsSurfaceData surface, BSurfaceInfo info, int depth, int splitDepth, double tol, double eps,
        //    List<PosNorUV> uvPoints, List<BSurfaceInfo> surfaceInfos, List<Task> tasks, double maxSpanU, double maxSpanV/*, BinaryNode node*/)
        //{
        //    double centerU = 0.5 * (info.MinU + info.MaxU);

        //    BSurfaceInfo l = info;
        //    l.MinU = info.MinU; l.MaxU = centerU; l.MinV = info.MinV; l.MaxV = info.MaxV;

        //    BSurfaceInfo r = info;
        //    r.MinU = centerU; r.MaxU = info.MaxU; r.MinV = info.MinV; r.MaxV = info.MaxV;

        //    //node.A = new BinaryNode(new Vec2D(l.MinU, l.MinV), new Vec2D(l.MaxU, l.MaxV));
        //    //node.B = new BinaryNode(new Vec2D(r.MinU, r.MinV), new Vec2D(r.MaxU, r.MaxV));            

        //    NurbsSurfaceData lower, upper;
        //    surface.SplitU(0.5, out lower, out upper);

        //    //#if DEBUG
        //    //            Vec3D debug = upper.GetPointMinUMinV();
        //    //            Vec3D debug2 = ToV3(surfaceDebug.GetPoint(r.MinU, r.MinV));
        //    //            if ((debug - debug2).LengthSquared > 1e-8)
        //    //            {

        //    //            }
        //    //#endif

        //    BSurfaceInsertionPointsTask(lower, l, depth + 1, splitDepth, tol, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.A*/);
        //    BSurfaceInsertionPointsTask(upper, r, depth + 1, splitDepth, tol, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.B*/);
        //}

        //private static void SplitV(NurbsSurfaceData surface, BSurfaceInfo info, int depth, int splitDepth, double tol, double eps,
        //    List<PosNorUV> uvPoints, List<BSurfaceInfo> surfaceInfos, List<Task> tasks, double maxSpanU, double maxSpanV/*, BinaryNode node*/)
        //{
        //    double centerV = 0.5 * (info.MinV + info.MaxV);

        //    BSurfaceInfo l = info;
        //    l.MinU = info.MinU; l.MaxU = info.MaxU; l.MinV = info.MinV; l.MaxV = centerV;

        //    BSurfaceInfo r = info;
        //    r.MinU = info.MinU; r.MaxU = info.MaxU; r.MinV = centerV; r.MaxV = info.MaxV;

        //    //node.A = new BinaryNode(new Vec2D(l.MinU, l.MinV), new Vec2D(l.MaxU, l.MaxV));
        //    //node.B = new BinaryNode(new Vec2D(r.MinU, r.MinV), new Vec2D(r.MaxU, r.MaxV));

        //    NurbsSurfaceData lower, upper;
        //    surface.SplitV(0.5, out lower, out upper);

        //    //#if DEBUG
        //    //            Vec3D debug = upper.GetPointMinUMinV();
        //    //            Vec3D debug2 = ToV3(surfaceDebug.GetPoint(r.MinU, r.MinV));
        //    //            if ((debug - debug2).LengthSquared > 1e-8)
        //    //            {

        //    //            }
        //    //#endif



        //    BSurfaceInsertionPointsTask(lower, l, depth + 1, splitDepth, tol, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.A*/);
        //    BSurfaceInsertionPointsTask(upper, r, depth + 1, splitDepth, tol, eps, uvPoints, surfaceInfos, tasks, maxSpanU, maxSpanV/*, node.B*/);
        //}


        //private static class DeBoor
        //{
        //    public static int KnotIndex(double w, int degree, double[] knots)
        //    {
        //        //if (w < 0 || w > 1)
        //        //    throw new Exception("Parameter is not in the valid range");

        //        int k = knots.Length - 1;
        //        while (k >= degree)
        //        {
        //            if (knots[k] <= w)
        //                break;
        //            --k;
        //        }
        //        k = Math.Max(degree, Math.Min(k, knots.Length - degree - 2));
        //        return k;
        //    }

        //    public static Vec3D Evaluate(int degree, double parameter, Vec3D[] controlPoints, double[] knots)
        //    {
        //        return Evaluate(degree, KnotIndex(parameter, degree, knots), parameter, controlPoints, knots);
        //    }

        //    public unsafe static Vec3D Evaluate(int p, int k, double w, Vec3D[] controlPoints, double[] knots)
        //    {
        //        Vec3D* buffer = stackalloc Vec3D[p + 1];

        //        int offset = k - p;
        //        for (int i = offset; i <= k; ++i)
        //            buffer[i - offset] = controlPoints[i];

        //        return Evaluate(p, k, w, buffer, offset, knots);
        //    }

        //    public unsafe static Vec3D Evaluate(int p, int k, double w, Vec3D* buffer, int offset, double[] knots)
        //    {
        //        for (int r = 1; r <= p; ++r)
        //        {
        //            for (int i = k; i >= k - p + r; --i)
        //            {
        //                double a = (w - knots[i]) / (knots[i + p - r + 1] - knots[i]);
        //                buffer[i - offset] = (1 - a) * buffer[i - offset - 1] + a * buffer[i - offset];
        //            }
        //        }
        //        return buffer[k - offset];
        //    }


        //    public static Vec3D Project(Vec4D v)
        //    {
        //        if (v.W != 0)
        //        {
        //            double s = 1.0 / v.W;
        //            return new Vec3D(v.X * s, v.Y * s, v.Z * s);
        //        }
        //        else
        //            return new Vec3D(v.X, v.Y, v.Z);
        //    }

        //    /*public static Vector3d Project(Vector4d v)
        //    {
        //        if (v.W > -1e-16 && v.W > 1e-16)
        //            throw new DivideByZeroException();
        //        double s = 1.0 / v.W;
        //        return new Vector3d(v.X * s, v.Y * s, v.Z * s);
        //    }*/

        //    public unsafe static Vec3D Evaluate(int degree, double parameter, Vec4D[] controlPoints, double[] knots)
        //    {
        //        return Project(EvaluateRational(degree, parameter, controlPoints, knots));
        //    }

        //    public unsafe static Vec4D EvaluateRational(int degree, double parameter, Vec4D[] controlPoints, double[] knots)
        //    {
        //        int k = KnotIndex(parameter, degree, knots);
        //        Vec4D* buffer = stackalloc Vec4D[degree + 1];
        //        return EvaluateRational(degree, k, parameter, controlPoints, knots, buffer);
        //    }

        //    public unsafe static Vec4D EvaluateRational(int p, int k, double w, Vec4D[] controlPoints, double[] knots, Vec4D* buffer)
        //    {
        //        int offset = k - p;
        //        for (int i = offset; i <= k; ++i)
        //            buffer[i - offset] = controlPoints[i];

        //        return EvaluateRational(p, k, w, buffer, offset, knots);
        //    }

        //    public unsafe static Vec4D EvaluateRational(int p, int k, double w, Vec4D* buffer, int offset, double[] knots)
        //    {
        //        for (int r = 1; r <= p; ++r)
        //        {
        //            for (int i = k; i >= k - p + r; --i)
        //            {
        //                double a = (w - knots[i]) / (knots[i + p - r + 1] - knots[i]);
        //                buffer[i - offset] = (1 - a) * buffer[i - offset - 1] + a * buffer[i - offset];
        //            }
        //        }
        //        return buffer[k - offset];
        //    }
        //}

        private sealed class NurbsCurveData
        {
            public readonly int Degree; //public int Degree { get { return _degree; } } //p        
            public readonly Vec4D[] ControlPoints; //public Vec4D[] ControlPoints { get { return _controlPoints; } } //d
            public readonly double[] Knots; //public double[] Knots { get { return _knots; } }


            public NurbsCurveData(int degree, Vec4D[] controlPoints, double[] knots)
            {
                Degree = degree;
                ControlPoints = controlPoints;
                Knots = knots;
            }

            ////http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/de-Boor.html
            //public Vec3D Evaluate(double u)
            //{
            //    return DeBoor.Project(EvaluateH(u));
            //    //Vector3d res = DeBoor.Project(v);

            //    //return DeBoor2.EvaluateRational(_degree, u, _controlPoints, _knots);
            //}

            //private Vec4D EvaluateH(double u)
            //{
            //    return DeBoor.EvaluateRational(Degree, u, ControlPoints, Knots);
            //}

            public void Split(double param, out NurbsCurveData lower, out NurbsCurveData upper, double eps = 1e-14)
            {
                Split(param, Degree, Knots, ControlPoints, out lower, out upper, eps);
            }

            public static void Split(double param, int degree, double[] knots, Vec4D[] controlPoints,
                out NurbsCurveData lower, out NurbsCurveData upper, double eps = 1e-14)
            {
                int existingKnotMultiplicity;
                int k = GetKnotInsertionIndex(param, knots, out existingKnotMultiplicity, eps); //existingKnotMultiplicity = 0;
                NurbsCurveData buffer = InsertKnot(degree, knots, controlPoints, param, k, degree - existingKnotMultiplicity /*+ 1*/);


                int order = degree + 1;
                int numInnerPointsLower = k - order + 1 /*- existingKnotMultiplicity*/;
                double[] lowerKnots = new double[2 * order + numInnerPointsLower];
                for (int i = 0; i < order; ++i)
                    lowerKnots[i] = 0;
                double s = 1.0 / param;
                for (int i = 0; i < numInnerPointsLower; ++i)
                    lowerKnots[i + order] = knots[i + order] * s;
                for (int i = order + numInnerPointsLower; i < lowerKnots.Length; ++i)
                    lowerKnots[i] = 1;

                Vec4D[] lowerCP = new Vec4D[numInnerPointsLower + order];
                for (int i = 0; i < lowerCP.Length; ++i)
                    lowerCP[i] = buffer.ControlPoints[i];


                int numInnerPointsUpper = (knots.Length - k - 1) - order - existingKnotMultiplicity;
                double[] upperKnots = new double[2 * order + numInnerPointsUpper];
                for (int i = 0; i < order; ++i)
                    upperKnots[i] = 0;
                int indexOffset = order + numInnerPointsLower + existingKnotMultiplicity;
                s = 1.0 / (1 - param);
                for (int i = 0; i < numInnerPointsUpper; ++i)
                    upperKnots[i + order] = (knots[i + indexOffset] - param) * s;
                for (int i = order + numInnerPointsUpper; i < upperKnots.Length; ++i)
                    upperKnots[i] = 1;

                Vec4D[] upperCP = new Vec4D[numInnerPointsUpper + order];
                indexOffset = lowerCP.Length - 1;
                for (int i = 0; i < upperCP.Length; ++i)
                    upperCP[i] = buffer.ControlPoints[i + indexOffset /*lowerCP.Length - 1 /*- existingKnotMultiplicity*/];


                lower = new NurbsCurveData(degree, lowerCP, lowerKnots);
                upper = new NurbsCurveData(degree, upperCP, upperKnots);

                //Vec3D debug2 = lower.Evaluate(1);
                //Vec3D debug3 = upper.Evaluate(0);

            }

            public NurbsCurveData InsertKnot(double knotLocation, int knotMultiplicity = 1, bool init = true)
            {
                int k = GetKnotInsertionIndex(knotLocation, Knots);
                return InsertKnot(Degree, Knots, ControlPoints, knotLocation, k, knotMultiplicity);
            }

            private static NurbsCurveData InsertKnot(int degree, double[] knots, Vec4D[] controlPoints,
                double knotLocation, int k, int knotMultiplicity = 1)
            {
                int numKnots = knots.Length;
                double[] newKnots = new double[numKnots + knotMultiplicity];
                for (int i = 0; i <= k; ++i)
                    newKnots[i] = knots[i];
                for (int i = 0; i < knotMultiplicity; ++i)
                    newKnots[k + i + 1] = knotLocation;
                for (int i = k + 1; i < numKnots; ++i)
                    newKnots[i + knotMultiplicity] = knots[i];

                int numControlPoints = controlPoints.Length;
                Vec4D[] newControlPoints = new Vec4D[numControlPoints + knotMultiplicity];
                for (int i = 0; i <= k; ++i)
                    newControlPoints[i] = controlPoints[i];
                for (int i = k + 1; i < numControlPoints; ++i)
                    newControlPoints[i + knotMultiplicity] = controlPoints[i];

                for (int i = 0; i < knotMultiplicity; ++i)
                {
                    //Store the point at the bottom to get space to store the first result of the following loop
                    newControlPoints[k + knotMultiplicity - i] = newControlPoints[k];
                    int lower = k - degree + i;
                    int indexOffset = degree + (knotMultiplicity - i);
                    for (int j = k; j > lower; --j)
                    {
                        double alpha = (knotLocation - newKnots[j]) / (newKnots[j + indexOffset] - newKnots[j]);
                        if (alpha < 0)
                            alpha = 0;
                        if (alpha > 1)
                            alpha = 1;
                        //if (double.IsNaN(alpha))
                        //    throw new Exception();
                        newControlPoints[j] = (1 - alpha) * newControlPoints[j - 1] + alpha * newControlPoints[j];
                    }
                }

                return new NurbsCurveData(degree, newControlPoints, newKnots);
            }

            private static int GetKnotInsertionIndex(double knotLocation, double[] knots)
            {
                int existingKnotMultiplicity;
                return GetKnotInsertionIndex(knotLocation, knots, out existingKnotMultiplicity);
            }

            private static int GetKnotInsertionIndex(double knotLocation, double[] knots, out int existingKnotMultiplicity, double eps = 1e-14)
            {
                existingKnotMultiplicity = 0;
                int k = -1;
                int numKnots = knots.Length;
                for (int i = 1; i < numKnots; ++i)
                //for (int i = numKnots - 1; i > 0; --i)
                {
                    double lower = knots[i - 1] - eps;
                    double upper = knots[i] + eps;
                    if (knotLocation >= lower && knotLocation <= upper)
                    {
                        k = i - 1;

                        for (int j = 0; j < numKnots; ++j)
                        {
                            if (Math.Abs(knots[j] - knotLocation) <= eps)
                                ++existingKnotMultiplicity;
                        }

                        break;
                    }
                }
                if (k < 0)
                    throw new Exception();

                return k;
            }
        }

        public sealed class NurbsSurfaceData
        {
            public readonly int DegreeU;
            public readonly int DegreeV;
            public readonly int Nu;
            public readonly int Nv;
            public readonly Vec4D[][] Poles; //public Vec4D[][] ControlPoints { get { return _controlPoints; } } // = new List<List<Vector3d>>();//The inner lists are in u-Direction
            public readonly double[] Knotu;// = new List<Knot>();
            public readonly double[] Knotv;// = new List<Knot>();


            public NurbsSurfaceData(int degreeU, int degreeV, Vec4D[][] controlPoints, double[] knotsU, double[] knotsV)
            {
                DegreeU = degreeU;
                DegreeV = degreeV;
                Nu = controlPoints.Length;
                Nv = controlPoints[0].Length;
                Poles = controlPoints;
                Knotu = knotsU;
                Knotv = knotsV;
            }



            public Vec3D GetNormalMinUMinV()
            {
                Vec3D corner = ToPoint(Poles[0][0]);
                Vec3D a = ToPoint(Poles[0][1]);
                Vec3D b = ToPoint(Poles[1][0]);
                return Vec3DOps.Cross(b - corner, a - corner);//.Normalized();
            }

            public Vec3D GetNormalMinUMaxV()
            {
                Vec3D corner = ToPoint(Poles[0][Nv - 1]);
                Vec3D a = ToPoint(Poles[0][Nv - 2]);
                Vec3D b = ToPoint(Poles[1][Nv - 1]);
                return Vec3DOps.Cross(a - corner, b - corner);//.Normalized();
            }

            public Vec3D GetNormalMaxUMinV()
            {
                Vec3D corner = ToPoint(Poles[Nu - 1][0]);
                Vec3D a = ToPoint(Poles[Nu - 1][1]);
                Vec3D b = ToPoint(Poles[Nu - 2][0]);
                return Vec3DOps.Cross(a - corner, b - corner);//.Normalized();
            }

            public Vec3D GetNormalMaxUMaxV()
            {
                Vec3D corner = ToPoint(Poles[Nu - 1][Nv - 1]);
                Vec3D a = ToPoint(Poles[Nu - 1][Nv - 2]);
                Vec3D b = ToPoint(Poles[Nu - 2][Nv - 1]);
                return Vec3DOps.Cross(b - corner, a - corner);//.Normalized();
            }


            public Vec3D GetPointAndNormalMinUMinV(out Vec3D normal)
            {
                Vec3D corner = ToPoint(Poles[0][0]);
                Vec3D a = ToPoint(Poles[0][1]);
                Vec3D b = ToPoint(Poles[1][0]);
                normal = Vec3DOps.Cross(b - corner, a - corner);//.Normalized();
                return corner;
            }

            public Vec3D GetPointAndNormalMinUMaxV(out Vec3D normal)
            {
                Vec3D corner = ToPoint(Poles[0][Nv - 1]);
                Vec3D a = ToPoint(Poles[0][Nv - 2]);
                Vec3D b = ToPoint(Poles[1][Nv - 1]);
                normal = Vec3DOps.Cross(a - corner, b - corner);//.Normalized();
                return corner;
            }

            public Vec3D GetPointAndNormalMaxUMinV(out Vec3D normal)
            {
                Vec3D corner = ToPoint(Poles[Nu - 1][0]);
                Vec3D a = ToPoint(Poles[Nu - 1][1]);
                Vec3D b = ToPoint(Poles[Nu - 2][0]);
                normal = Vec3DOps.Cross(a - corner, b - corner);//.Normalized();
                return corner;
            }

            public Vec3D GetPointAndNormalMaxUMaxV(out Vec3D normal)
            {
                Vec3D corner = ToPoint(Poles[Nu - 1][Nv - 1]);
                Vec3D a = ToPoint(Poles[Nu - 1][Nv - 2]);
                Vec3D b = ToPoint(Poles[Nu - 2][Nv - 1]);
                normal = Vec3DOps.Cross(b - corner, a - corner);//.Normalized();
                return corner;
            }

            public Vec3D GetPointMinUMinV()
            {
                return ToPoint(Poles[0][0]);
            }

            public Vec3D GetPointMinUMaxV()
            {
                return ToPoint(Poles[0][Nv - 1]);
            }

            public Vec3D GetPointMaxUMinV()
            {
                return ToPoint(Poles[Nu - 1][0]);
            }

            public Vec3D GetPointMaxUMaxV()
            {
                return ToPoint(Poles[Nu - 1][Nv - 1]);
            }

            public static Vec3D ToPoint(Vec4D pole)
            {
                if (pole.W != 1 && pole.W != 0)
                {
                    double s = 1.0 / pole.W;
                    pole.X *= s;
                    pole.Y *= s;
                    pole.Z *= s;
                }
                return new Vec3D() { X = pole.X, Y = pole.Y, Z = pole.Z };
            }



            public void SplitV(double vIsoValue, out NurbsSurfaceData lower, out NurbsSurfaceData upper)
            {
                int l = Poles.Length;
                Vec4D[][] lowerPoints = new Vec4D[l][];
                Vec4D[][] upperPoints = new Vec4D[l][];
                NurbsCurveData low = null;
                NurbsCurveData up = null;
                for (int i = 0; i < l; ++i)
                {
                    NurbsCurveData.Split(vIsoValue, DegreeV, Knotv, Poles[i], out low, out up);
                    /*int m = low.ControlPoints.Length;
                    lowerPoints[i] = new Vector4d[m]; Array.Copy(low.ControlPoints, lowerPoints, m);
                    m = up.ControlPoints.Length;
                    upperPoints[i] = new Vector4d[m]; Array.Copy(up.ControlPoints, upperPoints, m);*/
                    lowerPoints[i] = low.ControlPoints;
                    upperPoints[i] = up.ControlPoints;
                }

                //l = Knotu.Length;
                //double[] lowerKnotsU = new double[l]; Array.Copy(Knotu, lowerKnotsU, l);
                //double[] upperKnotsU = new double[l]; Array.Copy(Knotu, upperKnotsU, l);
                lower = new NurbsSurfaceData(DegreeU, DegreeV, lowerPoints, Knotu/*lowerKnotsU*/, low.Knots);
                upper = new NurbsSurfaceData(DegreeU, DegreeV, upperPoints, Knotu/*upperKnotsU*/, up.Knots);
            }

            public void SplitU(double uIsoValue, out NurbsSurfaceData lower, out NurbsSurfaceData upper)
            {
                int l = Poles.Length;
                int m = Poles[0].Length;

                Vec4D[] buffer = new Vec4D[l];
                NurbsCurveData low, up;
                for (int j = 0; j < l; ++j)
                    buffer[j] = Poles[j][0];
                NurbsCurveData.Split(uIsoValue, DegreeU, Knotu, buffer, out low, out up);
                int lc = low.ControlPoints.Length;
                Vec4D[][] lowerPoints = new Vec4D[lc][];
                for (int i = 0; i < lc; ++i)
                    lowerPoints[i] = new Vec4D[m];

                int uc = up.ControlPoints.Length;
                Vec4D[][] upperPoints = new Vec4D[uc][];
                for (int i = 0; i < uc; ++i)
                    upperPoints[i] = new Vec4D[m];

                //double[] lowerKnotsU = low.Knots; // new double[low.Knots.Length]; Array.Copy(low.Knots, lowerKnotsU, lowerKnotsU.Length);
                //double[] upperKnotsU = new double[up.Knots.Length]; Array.Copy(up.Knots, upperKnotsU, upperKnotsU.Length);

                for (int j = 0; j < lc; ++j)
                    lowerPoints[j][0] = low.ControlPoints[j];
                for (int j = 0; j < uc; ++j)
                    upperPoints[j][0] = up.ControlPoints[j];

                for (int i = 1; i < m; ++i)
                {
                    for (int j = 0; j < l; ++j)
                        buffer[j] = Poles[j][i];
                    NurbsCurveData.Split(uIsoValue, DegreeU, Knotu, buffer, out low, out up);

                    for (int j = 0; j < lc; ++j)
                        lowerPoints[j][i] = low.ControlPoints[j];
                    for (int j = 0; j < uc; ++j)
                        upperPoints[j][i] = up.ControlPoints[j];
                }

                //l = Knotv.Length;
                //double[] lowerKnotsV = new double[l]; Array.Copy(Knotv, lowerKnotsV, l);
                //double[] upperKnotsV = new double[l]; Array.Copy(Knotv, upperKnotsV, l);
                lower = new NurbsSurfaceData(DegreeU, DegreeV, lowerPoints, low.Knots, Knotv/*lowerKnotsV*/);
                upper = new NurbsSurfaceData(DegreeU, DegreeV, upperPoints, up.Knots, Knotv/*upperKnotsV*/);
            }
        }


        public static TriangulatedGeometryWithBorder TriangulateAdaptive(BSplineSurface surface, List<List<Vec2D>> uvBorderLoops = null, double maxDeviation = 0.1,
           double minimalBoundaryPointDistance = 1e-2, double tol = 1e-8, double eps = 1e-12,
           double largeTolerance = 1e-6, double maxBorderPointUVDistanceForVisualizationOnly = 0.01, bool noNormals = false, double maxSpanU = 1.0, double maxSpanV = 1.0)
        {
            List<BSurfaceInfo> patches;
            List<Tri>[] triBuffer;
            return TriangulateAdaptive(surface, out patches, out triBuffer, uvBorderLoops, maxDeviation, minimalBoundaryPointDistance,
                tol, eps, largeTolerance, maxBorderPointUVDistanceForVisualizationOnly, noNormals, maxSpanU, maxSpanV);
        }




        //private static bool CheckPoint(int j, int k, BBoundary boundary, Func<double, double, Vec3D> evalPoint,
        //    Vec3D prev, Vec3D current, double param, double minimalBoundaryPointDistance = 1e-2)
        //{
        //    double uCenter = (1 - param) * boundary[j, k - 1][0] + param * boundary[j, k][0];
        //    double vCenter = (1 - param) * boundary[j, k - 1][1] + param * boundary[j, k][1];
        //    Vec3D center = evalPoint(uCenter, vCenter);

        //    double dx = current[0] - center[0];
        //    double dy = current[1] - center[1];
        //    double dz = current[2] - center[2];
        //    double delta1 = dx * dx + dy * dy + dz * dz;
        //    dx = prev[0] - center[0];
        //    dy = prev[1] - center[1];
        //    dz = prev[2] - center[2];
        //    double delta2 = dx * dx + dy * dy + dz * dz;
        //    double d2 = minimalBoundaryPointDistance * minimalBoundaryPointDistance;
        //    return delta1 > d2 || delta2 > d2;
        //}


        ////Be careful: This adds elemnts to the list points
        //public static List<List<int>> CollectBorderLoops(BBoundary boundary, List<Vec2D> points, Func<double, double, Vec3D> evalPoint, double minimalBoundaryPointDistance = 1e-2)
        //{
        //    int numParts = boundary.GetNumberOfParts();
        //    //List<Vec2D> points = new List<Vec2D>(); ;
        //    List<List<int>> borderLoops = new List<List<int>>(numParts);
        //    double d2 = minimalBoundaryPointDistance * minimalBoundaryPointDistance;
        //    for (int j = 0; j < numParts; ++j)
        //    {
        //        int numGroups = boundary.GetNumberOfGroups(j);
        //        List<int> borderLoop = new List<int>(numGroups - 1);
        //        Vec3D prev = evalPoint(boundary[j, 0][0], boundary[j, 0][1]);
        //        for (int k = 1; k < numGroups; ++k) //Start at 1 since first and last point are the same
        //        {
        //            SVectorDouble point = boundary[j, k];
        //            double u = point[0];
        //            double v = point[1];
        //            Vec3D current = evalPoint(u, v);
        //            double dx = current[0] - prev[0];
        //            double dy = current[1] - prev[1];
        //            double dz = current[2] - prev[2];
        //            double delta = dx * dx + dy * dy + dz * dz;
        //            if (delta > d2)
        //            {
        //                borderLoop.Add(points.Count);
        //                points.Add(new Vec2D(u, v));
        //                prev = current;
        //            }
        //            else
        //            {
        //                if (CheckPoint(j, k, boundary, evalPoint, prev, current, 0.5, minimalBoundaryPointDistance))
        //                {
        //                    borderLoop.Add(points.Count);
        //                    points.Add(new Vec2D(u, v));
        //                    prev = current;
        //                }
        //                else
        //                {

        //                }
        //            }
        //        }
        //        borderLoops.Add(borderLoop);
        //    }
        //    return borderLoops;
        //}






        //#if DEBUG
        //        private static BSurface surfaceDebug;
        //#endif
        public static TriangulatedGeometryWithBorder TriangulateAdaptive(BSplineSurface surface, out List<BSurfaceInfo> patches, out List<Tri>[] triBuffer, List<List<Vec2D>> uvBorderLoops = null, double maxDeviation = 0.1,
            double minimalBoundaryPointDistance = 1e-2, double tol = 1e-8, double eps = 1e-12,
            double largeTolerance = 1e-6, double maxBorderPointUVDistanceForVisualizationOnly = 0.01, bool noNormals = false, double maxSpanU = 1.0, double maxSpanV = 1.0)
        {
            //#if DEBUG
            //            surfaceDebug = surface;
            //#endif
            //Vec2D[] pts = BSurfaceInsertionPoints(surface, maxDeviation);
            //return BTriangulator.Triangulate(surface, pts, minimalBoundaryPointDistance, tol, eps);

            List<double> times = new List<double>();
            System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

            double[] knotsU = surface.KnotsU; // ToArray(surface.Knotu);
            double[] knotsV = surface.KnotsV; // ToArray(surface.Knotv);
            int degreeU = surface.DegreeU; // surface.Ku - 1;
            int degreeV = surface.DegreeV; // surface.Kv - 1;
            Vec4D[][] points = surface.ControlPoints; // ToV4(surface.GetPoles());

            NurbsSurfaceData s = new NurbsSurfaceData(degreeU, degreeV, points, knotsU, knotsV);

            sw.Start();
            //Trying out the new stuff here:
            //List<BSurfaceInfo> patches;
            List<PosNorUV> ptsEx = BSurfaceInsertionPoints(s, out patches, maxDeviation, eps, maxSpanU, maxSpanV);

            List<Vec2D> pts = new List<Vec2D>(ptsEx.Count);
            for (int i = 0; i < ptsEx.Count; ++i)
                pts.Add(ptsEx[i].UV);

            times.Add(sw.ElapsedMilliseconds);




            Func<double, double, Vec3D> evalPoint = delegate (double u, double v) { return surface.Evaluate(u, v); /*Convert(surface.GetPoint(u, v));*/ };


            //List<Tri>[] triBuffer;
            //List<List<int>> borderLoops = BTriangulator.CollectBorderLoops(surface.Boundary, pts, evalPoint, minimalBoundaryPointDistance);
            List<List<int>> borderLoops;
            if (uvBorderLoops == null)
            {
                borderLoops = new List<List<int>>();
                triBuffer = TriangulateNoBorder(patches, pts, tol, eps);
            }
            else
            {
                borderLoops = new List<List<int>>(uvBorderLoops.Count);
                for (int i = 0; i < uvBorderLoops.Count; ++i)
                {
                    var loop = uvBorderLoops[i];
                    List<int> list = new List<int>(loop.Count);
                    for (int j = 0; j < loop.Count; ++j)
                    {
                        list.Add(pts.Count);
                        pts.Add(loop[j]);
                    }
                }


                //List<Tri> triangles = TriangulateIgnoringBorder(patches, pts, tol, eps);
                times.Add(sw.ElapsedMilliseconds - times[times.Count - 1]);
                //Be careful: the following command potentially introduces duplicate points and this is not allowed for TriangulateIgnoringBorder
                //List<List<int>> borderLoops = BTriangulator.CollectBorderLoops(surface.Boundary, pts, evalPoint, minimalBoundaryPointDistance);
                //if (!surface.BoundaryIsDefault)
                //{
                //    triangles = BoundaryClipper.ClipOutsideBoundary(pts, triangles, borderLoops, tol/*boundaryClipperTolerance*/, eps, largeTolerance, false);
                //}
                triBuffer = TriangulateWithBorder(patches, pts, borderLoops, tol, eps);
                times.Add(sw.ElapsedMilliseconds - times[times.Count - 1]);
            }



            //List<Tri> triangles = BoundaryClipper.Flatten(triBuffer);





            //Remove unused points (those that lie outside bundary wont'be used anymore since triangles out there were removed by BoundaryClipper)
            //This does not remove potential point duplicates but there should not be many of those (but there probaly are some)
            int l = pts.Count;
            bool[] pointIsUsed = new bool[l];
            int[] compressorMap = new int[l];
            //for (int i = 0; i < l; ++i)
            //    pointIsUsed[i] = -1;
            //int numTris = triangles.Count;
            for (int j = 0; j < triBuffer.Length; ++j)
            {
                var buffer = triBuffer[j];
                if (buffer == null)
                    continue;
                //int numTris = buffer.Count;
                for (int i = 0; i < buffer.Count/*numTris*/; ++i)
                {
                    Tri tri = buffer[i];

                    if (tri.A == tri.B || tri.A == tri.C || tri.B == tri.C)
                    {
                        buffer[i] = buffer[buffer.Count - 1];
                        buffer.RemoveAt(buffer.Count - 1);
                        --i;
                    }
                    else
                    {
                        pointIsUsed[tri.A] = true;
                        pointIsUsed[tri.B] = true;
                        pointIsUsed[tri.C] = true;
                    }
                }
            }
            int indexer = 0;
            for (int i = 0; i < l; ++i)
            {
                compressorMap[i] = indexer;
                if (pointIsUsed[i])
                {
                    ++indexer;
                }
            }
            //Map the triangles
            for (int j = 0; j < triBuffer.Length; ++j)
            {
                var buffer = triBuffer[j];
                if (buffer == null)
                    continue;
                int numTris = buffer.Count;
                for (int i = 0; i < numTris; ++i)
                {
                    Tri tri = buffer[i];
                    tri.A = compressorMap[tri.A];
                    tri.B = compressorMap[tri.B];
                    tri.C = compressorMap[tri.C];
                    buffer[i] = tri;
                }
            }


            ////TODO: Next line potentially introduces duplicates causing problems
            //List<List<int>> borderLoops = BTriangulator.CollectBorderLoops(surface.Boundary, pts, evalPoint, minimalBoundaryPointDistance);
            //List<Tri> triangles = null;
            //if (surface.BoundaryIsDefault)
            //    triangles = TriangulateIgnoringBorder(patches, pts, tol, eps);
            //else
            //    triangles = Triangulate(patches, pts, borderLoops, tol, eps);






            List<Vec3D> points3D = new List<Vec3D>(/*pts.Count*/indexer);
            Reserve(points3D, indexer);
            List<Vec3D> normals3D;
            if (noNormals)
                normals3D = null;
            else
            {
                normals3D = new List<Vec3D>(/*pts.Count */ indexer);
                Reserve(normals3D, indexer);
            }
            List<Vec2D> uv = new List<Vec2D>(/*pts.Count */ indexer);
            Reserve(uv, indexer);
            //for (int i = 0; i < pts.Count; ++i)


            //for (int i = 0; i < ptsEx.Count; ++i)
            //{
            //    if (ptsEx[i].UV != pts[i])
            //    {

            //    }
            //}

            bool nonRational = true; // false && !surface.Rational;
            var poles = surface.ControlPoints; // surface.GetPoles();
            int a = poles.Length;
            int b = a > 0 ? poles[0].Length : 0;
            for (int i = 0; i < a; ++i)
                for (int j = 0; j < b; ++j)
                {
                    var pole = poles[i][j];
                    if (pole.W != 1)
                    {
                        nonRational = false;
                        break;
                    }
                }

            if (noNormals)
            {
                //Parallel.For(0, pts.Count, i =>
                int numPoints = pts.Count;
                for (int i = 0; i < numPoints; ++i)
                {
                    if (pointIsUsed[i])
                    {
                        Vec2D v;
                        Vec3D point;
                        if (i < ptsEx.Count && nonRational)
                        {
                            var info = ptsEx[i];
                            v = info.UV;
                            point = info.GetPoint();
                        }
                        else
                        {
                            v = pts[i];
                            point = surface.Evaluate(v.X, v.Y);
                        }
                        points3D[compressorMap[i]] = point;
                        uv[compressorMap[i]] = v;
                    }
                }//);
            }
            else
            {
                //Parallel.For(0, pts.Count, i =>
                int numPoints = pts.Count;
                for (int i = 0; i < numPoints; ++i)
                {
                    if (pointIsUsed[i])
                    {
                        Vec2D v;
                        Vec3D point, normal;
                        if (i < ptsEx.Count && nonRational)
                        {
                            var info = ptsEx[i];
                            v = info.UV;
                            point = info.GetPointAndNormal(out normal);

                            double l2 = normal.LengthSquared();
                            if (l2 < 1e-12)
                            {
                                /*SVectorDouble3D n;
                                point = ToV3(surface.GetPointAndNormal(v.X, v.Y, out n));
                                normal = ToV3(n);*/
                                point = surface.EvaluatePointAndNormal(v.X, v.Y, out normal);
                                if (normal.LengthSquared() >= 1e-12)
                                    normal.Normalize();
                            }
                            else
                                normal.Normalize();

                            //#if DEBUG
                            //                            //debug                          
                            //                            SVectorDouble3D n;
                            //                            SVectorDouble3D p = surface.GetPointAndNormal(v.X, v.Y, out n);
                            //                            var p2 = BTriangulator.Convert(p);
                            //                            var n2 = BTriangulator.Convert(n);

                            //                            var v2 = pts[i];
                            //                            if ((point - p2).LengthSquared > 1e-8)
                            //                            {

                            //                            }
                            //                            else
                            //                            {

                            //                            }
                            //                            if ((normal - n2).LengthSquared > 1e-8)
                            //                            {

                            //                            }
                            //#endif
                        }
                        else
                        {
                            v = pts[i];
                            /*SVectorDouble3D n;
                            SVectorDouble3D p = surface.GetPointAndNormal(v.X, v.Y, out n);
                            point = Convert(p);
                            normal = Convert(n);
                            normal.Normalize();*/
                            point = surface.EvaluatePointAndNormal(v.X, v.Y, out normal);
                            if (normal.LengthSquared() >= 1e-12)
                                normal.Normalize();

                            //#if DEBUG
                            //                            //debug
                            //                            if (i < ptsEx.Count)
                            //                            {
                            //                                var info = ptsEx[i];
                            //                                Vec3D n2;
                            //                                var p2 = info.GetPointAndNormal(out n2);


                            //                                //point = p2;
                            //                                //normal = n2;


                            //                                if ((info.UV - v).LengthSquared > 1e-8)
                            //                                {

                            //                                }

                            //                                if ((point - p2).LengthSquared > 1e-8)
                            //                                {

                            //                                }
                            //                                else
                            //                                {

                            //                                }
                            //                                if ((normal - n2).LengthSquared > 1e-8)
                            //                                {

                            //                                }
                            //                            }
                            //#endif
                        }
                        points3D[compressorMap[i]] = point;
                        normals3D[compressorMap[i]] = normal;
                        uv[compressorMap[i]] = v;
                    }
                }//);
            }

            List<List<Vec3D>> borderLoops3D = new List<List<Vec3D>>(borderLoops.Count);
            for (int i = 0; i < borderLoops.Count; ++i)
            {
                List<int> indices = borderLoops[i];
                if (indices.Count > 0)
                {
                    List<Vec3D> loop = new List<Vec3D>(indices.Count);
                    for (int k = 0; k < indices.Count; ++k)
                    {
                        Vec2D current = pts[indices[k]];
                        Vec2D next = pts[indices[(k + 1) % indices.Count]];

                        loop.Add(evalPoint(current.X, current.Y));

                        double d = (next - current).Length();
                        int numPoints = (int)(d / maxBorderPointUVDistanceForVisualizationOnly + 1);
                        for (int j = 1; j < numPoints; ++j)
                        {
                            double u = (double)j / numPoints;
                            Vec2D p = (1 - u) * current + u * next;
                            loop.Add(evalPoint(p.X, p.Y));
                        }
                    }


                    borderLoops3D.Add(loop);
                }
            }

            //MakeTriangleOrientationConsistent(triBuffer, points3D);
            AlignNormalDirectionToTriangleOrientation(triBuffer, points3D, normals3D);


            List<Tri> triangles = BoundaryClipper.Flatten(triBuffer);




            triangles = RemoveZeroAreaTriangles(triangles, points3D);

            TriangulatedGeometryWithBorder g = new TriangulatedGeometryWithBorder(triangles, points3D, normals3D, uv, borderLoops3D);


            times.Add(sw.ElapsedMilliseconds - times[times.Count - 1]);
            sw.Stop();

            return g;
            //return BTriangulator.BuildGeometry(pts, triangles, /*new List<List<int>>()*/borderLoops, surface.Name,
            //    evalPoint,
            //    delegate(double u, double v)
            //    {
            //        Vec3D n = BTriangulator.Convert(surface.GetNormal(u, v));
            //        n.Normalize();
            //        return n;
            //    });

            //return BTriangulator.Triangulate(surface, pts, minimalBoundaryPointDistance, tol, eps);
        }





        public static void AlignNormalDirectionToTriangleOrientation(IList<List<Tri>> triangles, IList<Vec3D> points, IList<Vec3D> normals)
        {
            if (triangles == null)
                return;
            foreach (var batch in triangles)
                AlignNormalDirectionToTriangleOrientation(batch, points, normals);
        }

        public static void AlignNormalDirectionToTriangleOrientation(IList<Tri> triangles, IList<Vec3D> points, IList<Vec3D> normals)
        {
            if (triangles == null || points == null || normals == null)
                return;

            const double minAreaSq = 1e-24;
            for (int i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                if (tri.A >= points.Count || tri.B >= points.Count || tri.C >= points.Count)
                    continue;

                var a = points[tri.A];
                var b = points[tri.B];
                var c = points[tri.C];
                var faceNormal = Vec3DOps.Cross(b - a, c - a);
                if (faceNormal.LengthSquared() < minAreaSq)
                    continue;

                faceNormal = faceNormal.Normalized();
                FlipNormalIfNeeded(tri.A, faceNormal, normals);
                FlipNormalIfNeeded(tri.B, faceNormal, normals);
                FlipNormalIfNeeded(tri.C, faceNormal, normals);
            }
        }

        private static void FlipNormalIfNeeded(int index, Vec3D faceNormal, IList<Vec3D> normals)
        {
            if (index < 0 || index >= normals.Count)
                return;
            var n = normals[index];
            if (n.LengthSquared() < 1e-30)
                return;
            if (Vec3DOps.Dot(n.Normalized(), faceNormal) < 0)
                normals[index] = n * -1;
        }

        private static List<Tri> RemoveZeroAreaTriangles(IList<Tri> triangles, IList<Vec3D> points)
        {
            const double minAreaSq = 1e-24;
            var result = new List<Tri>(triangles.Count);
            foreach (var tri in triangles)
            {
                if (tri.A >= points.Count || tri.B >= points.Count || tri.C >= points.Count)
                    continue;
                var a = points[tri.A];
                var b = points[tri.B];
                var c = points[tri.C];
                if (Vec3DOps.Cross(b - a, c - a).LengthSquared() >= minAreaSq)
                    result.Add(tri);
            }
            return result;
        }

        private static void Reserve<T>(List<T> list, int numElements)
        {
            for (int i = list.Count; i < numElements; ++i)
                list.Add(default(T));
        }
    }


    //public class NurbsSubdivider
    //{
    //    private static Vec3D GetControlPoint(int id, BSplineSurface n, int crvInd, bool uDirection)
    //    {
    //        Vec4D v = uDirection ? n.ControlPoints[id][crvInd] : n.ControlPoints[crvInd][id];
    //        double invW = 1.0 / v.W;
    //        return new Vec3D(v.X * invW, v.Y * invW, v.Z * invW);
    //    }

    //    private static Vec3D GetControlPoint(Vec4D v)
    //    {
    //        double invW = 1.0 / v.W;
    //        return new Vec3D(v.X * invW, v.Y * invW, v.Z * invW);
    //    }

    //    public static bool IsVIsoCurveStraight(BSplineSurface n, double approxTolerance, int vIsoValue, double eps)
    //    {
    //        //If v is constant then the variable direction is u
    //        return IsCurveStraight(n, approxTolerance, vIsoValue, true, eps);
    //    }

    //    public static bool IsUIsoCurveStraight(BSplineSurface n, double approxTolerance, int uIsoValue, double eps)
    //    {
    //        //If u is constant then the variable direction is v
    //        return IsCurveStraight(n, approxTolerance, uIsoValue, false, eps);
    //    }

    //    /*public static bool IsCurveStraight(BSplineCurve n, double approxTolerance, double eps)
    //    {
    //        //Special case: lines are automatically straight
    //        if (n.Degree == 1)
    //            return true;

    //        int last = n.Degree;
    //        Vector3d e0 = GetControlPoint(n.ControlPoints[0]);

    //        //Form an initial line to test the other points against (skipping degenerate lines)
    //        Vector3d vec = default(Vector3d);
    //        double linelen = 0;
    //        for (int i = last; i > 0; --i)
    //        {
    //            Vector3d cp = GetControlPoint(n.ControlPoints[i]);
    //            vec = cp - e0;

    //            linelen = vec.Length;
    //            if (linelen > eps)
    //                break;
    //        }

    //        if (linelen > eps)
    //        {
    //            vec.Normalize();
    //            double tol2 = approxTolerance * approxTolerance;
    //            for (int i = 1; i <= last; i++)
    //            {
    //                Vector3d cp = GetControlPoint(n.ControlPoints[i]);
    //                double s;
    //                double dist = GeometricAlgorithms.SquaredDistancePointLine(cp, e0, vec, out s);

    //                if (dist > tol2)
    //                    return false;
    //            }
    //        }
    //        return true;
    //    }*/

    //    public static bool IsCurveStraight(BSplineSurface n, double approxTolerance, int crvInd, bool uDirection, double eps)
    //    {
    //        //Special case: lines are automatically straight
    //        if ((uDirection ? n.DegreeU : n.DegreeV) == 1)
    //            return true;

    //        int last = (uDirection ? n.DegreeU : n.DegreeV);
    //        Vec3D e0 = GetControlPoint(0, n, crvInd, uDirection);

    //        //Form an initial line to test the other points against (skipping degenerate lines)
    //        Vec3D vec = default(Vec3D);
    //        double linelen = 0;
    //        for (int i = last; i > 0; --i)
    //        {
    //            Vec3D cp = GetControlPoint(i, n, crvInd, uDirection);
    //            vec = cp - e0;

    //            linelen = vec.Length;
    //            if (linelen > eps)
    //                break;
    //        }

    //        if (linelen > eps)
    //        {
    //            vec.Normalize();
    //            double tol2 = approxTolerance * approxTolerance;
    //            for (int i = 1; i <= last; i++)
    //            {
    //                Vec3D cp = GetControlPoint(i, n, crvInd, uDirection);
    //                double s;
    //                double dist = Algebra3D.GeometricAlgorithms.SquaredDistancePointLine(cp, e0, vec, out s);

    //                if (dist > tol2)
    //                    return false;
    //            }
    //        }
    //        return true;
    //    }

    //    private static double DistanceToPlane(Vec3D p, Vec3D pointOnPlaneA, Vec3D pointOnPlaneB, Vec3D pointOnPlaneC)
    //    {
    //        Vec3D ab = pointOnPlaneB - pointOnPlaneA;
    //        Vec3D ac = pointOnPlaneC - pointOnPlaneA;
    //        Vec3D normal = Vec3DOps.Cross(ab, ac);
    //        double l = normal.LengthSquared();
    //        if (l > 1e-24)
    //            normal = normal / Math.Sqrt(l);
    //        else
    //            return 0;
    //        double d = Vec3DOps.Dot(pointOnPlaneA, normal);
    //        return Math.Abs(Vec3DOps.Dot(normal, p) - d);
    //    }

    //    private struct BSurfaceInfo
    //    {
    //        public bool StrU0;
    //        public bool StrUn;
    //        public bool StrV0;
    //        public bool StrVn;
    //        public bool FlatU;
    //        public bool FlatV;
    //        public double MinU;
    //        public double MaxU;
    //        public double MinV;
    //        public double MaxV;
    //    }

    //    private static bool TestFlat(BSplineSurface surface, ref BSurfaceInfo n, double tol, double eps)
    //    {
    //        int maxU = surface.DegreeU;// surface.Nu - 1;
    //        int maxV = surface.DegreeV;// surface.Nv - 1;

    //        //Check edge straightness
    //        if (!n.StrU0)
    //            n.StrU0 = IsUIsoCurveStraight(surface, tol, 0, eps);
    //        if (!n.StrUn)
    //            n.StrUn = IsUIsoCurveStraight(surface, tol, maxU, eps);
    //        if (!n.StrV0)
    //            n.StrV0 = IsVIsoCurveStraight(surface, tol, 0, eps);
    //        if (!n.StrVn)
    //            n.StrVn = IsVIsoCurveStraight(surface, tol, maxV, eps);

    //        //Test to make sure control points are straight in U and V
    //        bool straight = true;
    //        if (!n.FlatU && n.StrV0 && n.StrVn)
    //            for (int i = 1; i < maxV; i++)
    //            {
    //                straight = IsVIsoCurveStraight(surface, tol, i, eps);
    //                if (!straight)
    //                    break;
    //            }

    //        if (straight && n.StrV0 && n.StrVn)
    //            n.FlatU = true;

    //        //Page 315
    //        straight = true;
    //        if (!n.FlatV && n.StrU0 && n.StrUn)
    //            for (int i = 1; i < maxU; i++)
    //            {
    //                straight = IsUIsoCurveStraight(surface, tol, i, eps);
    //                if (!straight)
    //                    break;
    //            }

    //        if (straight && n.StrU0 && n.StrUn)
    //            n.FlatV = true;

    //        if (!n.FlatV || !n.FlatU)
    //            return false;

    //        //The surface can pass the above tests but still be twisted
    //        Vec3D a = GetControlPoint(surface.ControlPoints[0][0]);
    //        Vec3D b = GetControlPoint(surface.ControlPoints[maxU][0]);
    //        Vec3D c = GetControlPoint(surface.ControlPoints[0][maxV]);
    //        Vec3D d = GetControlPoint(surface.ControlPoints[maxU][maxV]);

    //        if (DistanceToPlane(d, a, b, c) > tol) // Surface is twisted)
    //            return false;
    //        if (DistanceToPlane(c, a, b, d) > tol) // Surface is twisted)
    //            return false;
    //        if (DistanceToPlane(a, b, c, d) > tol) // Surface is twisted)
    //            return false;
    //        if (DistanceToPlane(b, a, c, d) > tol) // Surface is twisted)
    //            return false;

    //        return true;
    //    }

    //    public static List<Vec2D> BSurfaceInsertionPoints(BSplineSurface surface, out BinaryNode root, double approxTolerance = 5e-2, double eps = 1e-12)
    //    {
    //        List<Vec2D> uvPoints = new List<Vec2D>();
    //        BSurfaceInfo info = new BSurfaceInfo()
    //        {
    //            FlatU = false,
    //            FlatV = false,
    //            StrU0 = false,
    //            StrV0 = false,
    //            StrUn = false,
    //            StrVn = false,
    //            MaxU = 1,
    //            MinU = 0,
    //            MaxV = 1,
    //            MinV = 0
    //        };
    //        root = new BinaryNode(new Vec2D(0, 0), new Vec2D(1, 1));
    //        BSplineSurfaceInsertionPoints(surface, info, approxTolerance, eps, uvPoints, root);

    //        //Debug
    //        Vec2D[] duplicateFreeUVPoints = uvPoints.Distinct().ToArray();
    //        if (uvPoints.Count != duplicateFreeUVPoints.Length)
    //            throw new Exception();

    //        return /*duplicateFreeUVPoints*/uvPoints;
    //    }

    //    private static void SplitU(BSplineSurface surface, BSurfaceInfo info, double tol, double eps, List<Vec2D> uvPoints, BinaryNode node)
    //    {
    //        double centerU = 0.5 * (info.MinU + info.MaxU);

    //        BSurfaceInfo l = info;
    //        l.MinU = info.MinU; l.MaxU = centerU; l.MinV = info.MinV; l.MaxV = info.MaxV;

    //        BSurfaceInfo r = info;
    //        r.MinU = centerU; r.MaxU = info.MaxU; r.MinV = info.MinV; r.MaxV = info.MaxV;

    //        node.A = new BinaryNode(new Vec2D(l.MinU, l.MinV), new Vec2D(l.MaxU, l.MaxV));
    //        node.B = new BinaryNode(new Vec2D(r.MinU, r.MinV), new Vec2D(r.MaxU, r.MaxV));

    //        BSplineSurface lower, upper;
    //        surface.SplitU(0.5, out lower, out upper, false);
    //        BSplineSurfaceInsertionPoints(lower, l, tol, eps, uvPoints, node.A);
    //        BSplineSurfaceInsertionPoints(upper, r, tol, eps, uvPoints, node.B);
    //    }

    //    private static void SplitV(BSplineSurface surface, BSurfaceInfo info, double tol, double eps, List<Vec2D> uvPoints, BinaryNode node)
    //    {
    //        double centerV = 0.5 * (info.MinV + info.MaxV);

    //        BSurfaceInfo l = info;
    //        l.MinU = info.MinU; l.MaxU = info.MaxU; l.MinV = info.MinV; l.MaxV = centerV;

    //        BSurfaceInfo r = info;
    //        r.MinU = info.MinU; r.MaxU = info.MaxU; r.MinV = centerV; r.MaxV = info.MaxV;

    //        node.A = new BinaryNode(new Vec2D(l.MinU, l.MinV), new Vec2D(l.MaxU, l.MaxV));
    //        node.B = new BinaryNode(new Vec2D(r.MinU, r.MinV), new Vec2D(r.MaxU, r.MaxV));

    //        BSplineSurface lower, upper;
    //        surface.SplitV(0.5, out lower, out upper, false);
    //        BSplineSurfaceInsertionPoints(lower, l, tol, eps, uvPoints, node.A);
    //        BSplineSurfaceInsertionPoints(upper, r, tol, eps, uvPoints, node.B);
    //    }

    //    //public List<Vector2d> ComputeInsertionPoints(BSplineSurface surface, double tol = 5e-2, double eps = 1e-12)
    //    //{
    //    //    List<Vector2d> uvPoint = new List<Vector2d>();
    //    //    BSurfaceInsertionPoints(surface, 
    //    //}

    //    //private static void BSplineCurveInsertionPoints(BSplineCurve curve, double tol, double eps, List<double> uPoints)
    //    //{
    //    //    bool straight = IsCurveStraight(curve, tol, eps);
    //    //    if (straight)
    //    //    {

    //    //    }
    //    //    else
    //    //    {

    //    //    }
    //    //}

    //    private static void BSplineSurfaceInsertionPoints(BSplineSurface surface, BSurfaceInfo info, double tol, double eps, List<Vec2D> uvPoints, BinaryNode node)
    //    {
    //        bool flat = TestFlat(surface, ref info, tol, eps);
    //        if (flat)
    //        {
    //            lock (uvPoints)
    //            {
    //                uvPoints.Add(new Vec2D(info.MinU, info.MinV));
    //                if (info.MaxV == 1)
    //                    uvPoints.Add(new Vec2D(info.MinU, info.MaxV));
    //                if (info.MaxU == 1)
    //                    uvPoints.Add(new Vec2D(info.MaxU, info.MinV));
    //                if (info.MaxU == 1 && info.MaxV == 1)
    //                    uvPoints.Add(new Vec2D(info.MaxU, info.MaxV));
    //            }
    //        }
    //        else
    //        {
    //            //Split and start recursion call                
    //            if (info.FlatU)
    //            {
    //                SplitV(surface, info, tol, eps, uvPoints, node);
    //            }
    //            else if (info.FlatV)
    //            {
    //                //Split only in u-Direction
    //                SplitU(surface, info, tol, eps, uvPoints, node);
    //            }
    //            else if (info.MaxV - info.MinV > info.MaxU - info.MinU)
    //            {
    //                SplitV(surface, info, tol, eps, uvPoints, node);
    //            }
    //            else
    //            {
    //                SplitU(surface, info, tol, eps, uvPoints, node);
    //            }
    //            /*for (int i = 0; i < tasks.Count; ++i)
    //            {
    //                Task t = tasks[i];
    //                if (t != null)
    //                    t.Wait();
    //            }*/
    //        }
    //    }

    //    public static List<Vec3D> Tessellate(BSplineCurve curve, double start = 0, double end = 1, double tol = 5e-2, double eps = 1e-12)
    //    {
    //        List<double> parameters;
    //        return Tessellate(curve, out parameters, start, end, tol, eps);
    //    }

    //    public static List<Vec3D> Tessellate(BSplineCurve curve, out List<double> parameters, double start = 0, double end = 1, double tol = 5e-2, double eps = 1e-12)
    //    {
    //        if (start != 0 || end != 1)
    //            curve = curve.ExtractRange(start, end);

    //        parameters = new List<double>();
    //        Tessellate(curve, 0, 1, tol, eps, parameters);

    //        int l = parameters.Count;
    //        List<Vec3D> points = new List<Vec3D>(l + 1);
    //        for (int i = 0; i < l; ++i)
    //            points.Add(curve.Evaluate(parameters[i]));

    //        points.Add(curve.Evaluate(1));

    //        return points;
    //    }

    //    //public static List<Vector3d> Tessellate2DBorder(BSplineCurve curve,  Func<double, double, Vector3d> surfaceEval, double start = 0, double end = 1, double tol = 5e-2, double eps = 1e-12)
    //    //{
    //    //    if (start != 0 || end != 1)
    //    //        curve = curve.ExtractRange(start, end);

    //    //    List<double> uPoints = new List<double>();
    //    //    Tessellate2DBorder(curve, surfaceEval, 0, 1, tol, eps, uPoints);

    //    //    int l = uPoints.Count;
    //    //    List<Vector3d> points = new List<Vector3d>(l + 1);
    //    //    for (int i = 0; i < l; ++i)
    //    //        points.Add(curve.Evaluate(uPoints[i]));

    //    //    points.Add(curve.Evaluate(1));

    //    //    return points;
    //    //}

    //    private static Vec3D GetControlPoint(int id, BSplineCurve n)
    //    {
    //        Vec4D v = n.ControlPoints[id];
    //        double invW = 1.0 / v.W;
    //        return new Vec3D(v.X * invW, v.Y * invW, v.Z * invW);
    //    }

    //    public static bool IsCurveStraight(BSplineCurve n, double tol, double eps)
    //    {
    //        //Special case: lines are automatically straight
    //        if (n.Degree == 1)
    //            return true;

    //        int last = n.Degree;
    //        Vec3D e0 = GetControlPoint(0, n);

    //        //Form an initial line to test the other points against (skipping degenerate lines)
    //        Vec3D vec = default(Vec3D);
    //        double linelen = 0;
    //        for (int i = last; i > 0; --i)
    //        {
    //            Vec3D cp = GetControlPoint(i, n);
    //            vec = cp - e0;

    //            linelen = vec.Length;
    //            if (linelen > eps)
    //                break;
    //        }

    //        if (linelen > eps)
    //        {
    //            vec.Normalize();
    //            double tol2 = tol * tol;
    //            for (int i = 1; i <= last; i++)
    //            {
    //                Vec3D cp = GetControlPoint(i, n);
    //                double s;
    //                double dist = Algebra3D.GeometricAlgorithms.SquaredDistancePointLine(cp, e0, vec, out s);

    //                if (dist > tol2)
    //                    return false;
    //            }
    //        }
    //        return true;
    //    }

    //    //TODO: Not sure if this works correctly...
    //    public static bool IsCurveStraight(BSplineCurve n, Func<double, double, Vec3D> surfaceEval, double tol, double eps)
    //    {
    //        //Special case: lines are automatically straight !!!Does not work when curve lies on a surface!!!
    //        //if (n.Degree == 1)
    //        //    return true;

    //        int last = n.Degree;
    //        Vec3D e0 = GetControlPoint(0, n);

    //        //Form an initial line to test the other points against (skipping degenerate lines)
    //        Vec3D vec = default(Vec3D);
    //        double linelen = 0;
    //        for (int i = last; i > 0; --i)
    //        {
    //            Vec3D cp = GetControlPoint(i, n);
    //            vec = cp - e0;

    //            linelen = vec.Length;
    //            if (linelen > eps)
    //                break;
    //        }

    //        if (linelen > eps)
    //        {
    //            //vec.Normalize();
    //            Vec3D e1 = e0 + vec;
    //            double tol2 = tol * tol;
    //            for (int i = 1; i <= last; i++)
    //            {
    //                Vec3D cp = GetControlPoint(i, n);
    //                double s;
    //                double dist = Algebra3D.GeometricAlgorithms.SquaredDistancePointLine(surfaceEval(cp.X, cp.Y), surfaceEval(e0.X, e0.Y), surfaceEval(e1.X, e1.Y) - surfaceEval(e0.X, e0.Y), out s);

    //                if (dist > tol2)
    //                    return false;
    //            }
    //        }
    //        return true;
    //    }


    //    private static void Tessellate(BSplineCurve curve, double min, double max, double tol, double eps, List<double> uPoints)
    //    {
    //        bool flat = IsCurveStraight(curve, tol, eps);
    //        if (flat)
    //        {
    //            lock (uPoints)
    //            {
    //                uPoints.Add(min);
    //            }
    //        }
    //        else
    //        {
    //            double center = 0.5 * (min + max);
    //            BSplineCurve lower, upper;
    //            curve.Split(0.5, out lower, out upper, false);
    //            Tessellate(lower, min, center, tol, eps, uPoints);
    //            Tessellate(upper, center, max, tol, eps, uPoints);
    //        }
    //    }


    //    private static void Tessellate2DBorder(BSplineCurve curve, Func<double, double, Vec3D> surfaceEval, double min, double max, double tol, double eps, List<double> uPoints)
    //    {
    //        bool flat = IsCurveStraight(curve, surfaceEval, tol, eps);
    //        if (flat)
    //        {
    //            lock (uPoints)
    //            {
    //                uPoints.Add(min);
    //            }
    //        }
    //        else
    //        {
    //            double center = 0.5 * (min + max);
    //            BSplineCurve lower, upper;
    //            curve.Split(0.5, out lower, out upper, false);
    //            Tessellate2DBorder(lower, surfaceEval, min, center, tol, eps, uPoints);
    //            Tessellate2DBorder(upper, surfaceEval, center, max, tol, eps, uPoints);
    //        }
    //    }
    //}

    //public class NurbsTessellator
    //{
    //    ////Assumes the borderLoops to be 2D - just ignore z-coordinates since all of them are zero or meaningless
    //    //public static List<List<Vector2d>> TessellateBorderLoopsUV(List<CurveStrip> borderLoops, BSplineSurface surface, double maxDeviation = 0.1, double eps = 1e-12)
    //    //{
    //    //    //if (borderLoops.Count == 0)
    //    //    //    return DefaultBorderLoop();

    //    //    //Func<double, double, Vector3d> surfaceEval = delegate(double x, double y)
    //    //    //{
    //    //    //    return sur
    //    //    //};

    //    //    //TODO: Make sure the curveStrips are actually closed
    //    //    List<List<Vector2d>> borderPoints = new List<List<Vector2d>>(borderLoops.Count);
    //    //    for (int i = 0; i < borderLoops.Count; ++i)
    //    //    {
    //    //        CurveStrip loop = borderLoops[i];
    //    //        List<Vector2d> points = new List<Vector2d>();
    //    //        for (int j = 0; j < loop.Count; ++j)
    //    //        {
    //    //            BSplineCurve curve = loop[j];
    //    //            List<Vector3d> pts = NurbsSubdivider.Tessellate2DBorder(curve, surface.Evaluate, tol: maxDeviation, eps: eps);
    //    //            for (int k = 1; k < pts.Count; ++k)
    //    //            {
    //    //                Vector3d p = pts[k];
    //    //                points.Add(new Vector2d(p.X, p.Y));
    //    //            }
    //    //        }
    //    //        borderPoints.Add(points);
    //    //    }
    //    //    return borderPoints;
    //    //}

    //    //Assumes the borderLoops to be 2D - just ignore z-coordinates since all of them are zero or meaningless
    //    public static List<List<Vec2D>> TessellateBorderLoopsUV(List<CurveStrip> borderLoops, double maxDeviation = 0.1, double eps = 1e-12)
    //    {
    //        //if (borderLoops.Count == 0)
    //        //    return DefaultBorderLoop();

    //        //Func<double, double, Vector3d> surfaceEval = delegate(double x, double y)
    //        //{
    //        //    return sur
    //        //};

    //        //TODO: Make sure the curveStrips are actually closed
    //        List<List<Vec2D>> borderPoints = new List<List<Vec2D>>(borderLoops.Count);
    //        for (int i = 0; i < borderLoops.Count; ++i)
    //        {
    //            CurveStrip loop = borderLoops[i];
    //            List<Vec2D> points = new List<Vec2D>();
    //            for (int j = 0; j < loop.Count; ++j)
    //            {
    //                BSplineCurve curve = loop[j];
    //                List<Vec3D> pts = NurbsSubdivider.Tessellate(curve, tol: maxDeviation, eps: eps);
    //                for (int k = 1; k < pts.Count; ++k)
    //                {
    //                    Vec3D p = pts[k];
    //                    points.Add(new Vec2D(p.X, p.Y));
    //                }
    //            }
    //            borderPoints.Add(points);
    //        }
    //        return borderPoints;
    //    }

    //    public static TriangulatedGeometry Triangulate(BSplineSurfaceBounded surface, double maxDeviation = 0.1, double minimalBoundaryPointDistance = 1e-6, double tol = 1e-8, double eps = 1e-12)
    //    {
    //        BinaryNode root;
    //        IList<Vec2D> pts = NurbsSubdivider.BSurfaceInsertionPoints(surface, out root, maxDeviation,  eps);

    //        List<List<Vec2D>> borderLoops = TessellateBorderLoopsUV(surface.BorderLoops, /*surface,*/ maxDeviation, eps);            
    //        List<List<Vec2D>> adjustedLoops = TreeCutter.Cut(borderLoops, root);
    //        return Triangulate(surface, pts, adjustedLoops, minimalBoundaryPointDistance, tol, eps);
    //    }

    //    public static TriangulatedGeometry Triangulate(BSplineSurfaceBounded surface, double maxDeviationBorder = 0.1, int numPointsU = -1, int numPointV = -1, double minimalBoundaryPointDistance = 1e-6, double tol = 1e-8, double eps = 1e-12)
    //    {
    //        Vec2D[] pts = RectangularPointGrid(numPointsU, numPointV);

    //        throw new NotImplementedException("Implement TreeCutter.CutGrid");
    //        //List<List<Vector2d>> borderLoops = TessellateBorderLoopsUV(surface.BorderLoops, /*surface,*/ maxDeviation, eps);
    //        //List<List<Vector2d>> adjustedLoops = TreeCutter.Cut(borderLoops, root);
    //        //return Triangulate(surface, pts, adjustedLoops, minimalBoundaryPointDistance, tol, eps);
    //    }

    //    public static TriangulatedGeometry Triangulate(BSplineSurfaceBounded surface, IList<Vec2D> pointsToInsert, IList<List<Vec2D>> boundaryLoops, double minimalBoundaryPointDistance = 1e-6, double tol = 1e-8, double eps = 1e-12)
    //    {
    //        return Triangulate(boundaryLoops, //TessellateBorderLoops(surface.BorderLoops, tol, eps),
    //            surface.Evaluate, //delegate(double x, double y) { return surface.Evaluate(x, y); },
    //            surface.EvaluateNormal, //delegate(double x, double y) { return surface.EvaluateNormal(x, y); },
    //            pointsToInsert, minimalBoundaryPointDistance, /*surface.Name*/"name", tol, eps);
    //    }

    //    //TODO: Specify u min and max as well as v min and max
    //    public static Vec2D[] RectangularPointGrid(int numPointsU = -1, int numPointV = -1)
    //    {
    //        Vec2D[] pts;
    //        if (numPointsU > 0 && numPointV > 0)
    //        {
    //            double scalingU = 1.0 / numPointsU;
    //            double scalingV = 1.0 / numPointV;
    //            pts = new Vec2D[numPointsU * numPointV];
    //            for (int v = 0; v < numPointV; ++v)
    //                for (int u = 0; u < numPointsU; ++u)
    //                    pts[v * numPointsU + u] = new Vec2D((u + 0.5) * scalingU, (v + 0.5) * scalingV);
    //        }
    //        else
    //            pts = new Vec2D[0];

    //        return pts;
    //    }

    //    public static TriangulatedGeometry Triangulate(IList<List<Vec2D>> boundaryLoops, Func<double, double, Vec3D> evalPoint, Func<double, double, Vec3D> evalNormal,
    //       double minimalBoundaryPointDistance = 1e-6, string name = "name", int numPointsU = -1, int numPointV = -1, double tol = 1e-8, double eps = 1e-12)
    //    {
    //        Vec2D[] pts = RectangularPointGrid(numPointsU, numPointV);
    //        return Triangulate(boundaryLoops, evalPoint, evalNormal, pts, minimalBoundaryPointDistance, name, tol, eps);
    //    }

    //    public static TriangulatedGeometry Triangulate(IList<List<Vec2D>> boundaryLoops, Func<double, double, Vec3D> evalPoint, Func<double, double, Vec3D> evalNormal,
    //        IList<Vec2D> pts, double minimalBoundaryPointDistance = 1e-6, string name = "name", double tol = 1e-8, double eps = 1e-12)
    //    {
    //        int numLoops = boundaryLoops.Count;
    //        List<Vec2D> points = new List<Vec2D>(); ;
    //        List<List<int>> borderLoops = new List<List<int>>(numLoops);
    //        for (int i = 0; i < numLoops; ++i)
    //        {
    //            List<Vec2D> loop = boundaryLoops[i];
    //            int numPoints = loop.Count;
    //            List<int> borderLoop = new List<int>(numPoints);
    //            Vec2D uv = loop[0];
    //            //Vector3d prev = evalPoint(uv.X, uv.Y);
    //            Vec2D prev = uv;
    //            borderLoop.Add(points.Count);
    //            points.Add(uv);
    //            for (int j = 1; j < numPoints; ++j) //Start at 1 since first and last point are the same
    //            {
    //                uv = loop[j];
    //                //Vector3d current = evalPoint(uv.X, uv.Y);
    //                //Vector3d current = evalPoint(uv.X, uv.Y);
    //                /*double dx = current.X - prev.X;
    //                double dy = current.Y - prev.Y;
    //                double dz = current.Z - prev.Z;*/
    //                double dx = uv.X - prev.X;
    //                double dy = uv.Y - prev.Y;
    //                if (dx * dx + dy * dy /*+ dz * dz*/ > minimalBoundaryPointDistance * minimalBoundaryPointDistance)
    //                {
    //                    borderLoop.Add(points.Count);
    //                    points.Add(uv);
    //                    prev = uv/*current*/;
    //                }
    //            }
    //            borderLoops.Add(borderLoop);
    //        }

    //        //List<Tri> triangles = ConstrainedTriangulator.TriangulateEarClipping(points, borderLoops, tol, eps, 50, 50, 0.005, 0.01);// Triangulate(borderLoops, 100, 100, 0.01, out uvPoints);
    //        List<Tri> triangles = ConstrainedTriangulator.TriangulateEarClipping(points, borderLoops, tol, eps, pts);// Triangulate(borderLoops, 100, 100, 0.01, out uvPoints);


    //        //if (visualize)
    //        //{
    //        //    lock (pointBuffer2D)
    //        //    {
    //        //        pointBuffer2D.Add(points);
    //        //    }
    //        //    lock (triangleBuffer)
    //        //    {
    //        //        triangleBuffer.Add(triangles);
    //        //    }
    //        //}

    //        List<Vec3D> pos = new List<Vec3D>(points.Count);
    //        List<Vec3D> nor = new List<Vec3D>(points.Count);
    //        //Channel<Vec2D> tex = new Channel<Vec2D>(points.Count);
    //        for (int j = 0; j < points.Count; ++j)
    //        {
    //            Vec2D v = points[j];
    //            pos.Add(evalPoint(v.X, v.Y));
    //            Vec3D n = evalNormal(v.X, v.Y);
    //            n.Normalize();
    //            nor.Add(n);
    //            //tex.Add(v);
    //        }


    //        /*List<List<SVectorDouble3D>> borderLoops3D = new List<List<SVectorDouble3D>>(borderLoops.Count);
    //        for (int j = 0; j < borderLoops.Count; ++j)
    //        {
    //            List<int> indices = borderLoops[j];
    //            List<SVectorDouble3D> loop = new List<SVectorDouble3D>(indices.Count);
    //            for (int k = 0; k < indices.Count; ++k)
    //            {
    //                Vector2d v = points[indices[k]];
    //                loop.Add(evalPoint(v.X, v.Y));
    //            }
    //            if (loop.Count > 0)
    //                borderLoops3D.Add(loop);
    //        }*/

    //        TriangulatedGeometry g = new TriangulatedGeometry(triangles, pos, nor, points);
    //        return g;
    //    }
    //}
}
