using GeoCore;

namespace NURBS
{
    /// <summary>
    /// Computation of 2D polygon operations including polygon offset and boolean operations (union, difference, intersect, xor)
    /// Most of the algorithms are based on the great Clipper library
    /// </summary>
    public class PolygonManipulator
    {
        public static bool Intersect(List<Vec2D> polygonSubject, List<Vec2D> polygonClip, out List<List<Vec2D>> result, bool aClosed = true, bool bClosed = true)
        {
            return Operate(ClipType.ctIntersection, new List<List<Vec2D>>() { polygonSubject }, new List<List<Vec2D>>() { polygonClip }, out result, null, aClosed, bClosed);
        }

        public static bool Difference(List<Vec2D> polygonSubject, List<Vec2D> polygonClip, out List<List<Vec2D>> result, bool aClosed = true, bool bClosed = true)
        {
            return Operate(ClipType.ctDifference, new List<List<Vec2D>>() { polygonSubject }, new List<List<Vec2D>>() { polygonClip }, out result, null, aClosed, bClosed);
        }

        public static bool Union(List<Vec2D> polygonSubject, List<Vec2D> polygonClip, out List<List<Vec2D>> result, bool aClosed = true, bool bClosed = true)
        {
            return Operate(ClipType.ctUnion, new List<List<Vec2D>>() { polygonSubject }, new List<List<Vec2D>>() { polygonClip }, out result, null, aClosed, bClosed);
        }

        public static bool Xor(List<Vec2D> polygonSubject, List<Vec2D> polygonClip, out List<List<Vec2D>> result, bool aClosed = true, bool bClosed = true)
        {
            return Operate(ClipType.ctXor, new List<List<Vec2D>>() { polygonSubject }, new List<List<Vec2D>>() { polygonClip }, out result, null, aClosed, bClosed);
        }

        public static List<List<Vec2D>> SimplifyPolygon(List<Vec2D> polygon, List<List<int>> sourcePointIndices = null, PolyFillType fillType = PolyFillType.pftEvenOdd)
        {
            Vec2D shift;
            double scaling = Scaling(out shift, polygon);
            double backScaling = 1.0 / (High * scaling);

            List<IntPoint> poly = Convert(polygon, scaling, shift);
            //List<List<IntPoint>> solution = Clipper.SimplifyPolygon(poly, fillType);


            List<List<IntPoint>> solution = new List<List<IntPoint>>();
            Clipper c = new Clipper();
            /*c.ZFillFunction = delegate (IntPoint bot1, IntPoint top1,
                IntPoint bot2, IntPoint top2, ref IntPoint pt)
            {
                pt.Z1 = (ushort)bot1.Z;
                pt.Z2 = (ushort)top1.Z;
                pt.Z3 = (ushort)bot2.Z;
                pt.Z4 = (ushort)top2.Z;
            };*/
            c.StrictlySimple = true;
            c.AddPath(poly, PolyType.ptSubject, true);
            c.Execute(ClipType.ctUnion, solution, fillType, fillType);



            var result = new List<List<Vec2D>>(solution.Count);
            for (int i = 0; i < solution.Count; ++i)
            {
                List<IntPoint> source = solution[i];
                List<Vec2D> list = new List<Vec2D>(source.Count);
                for (int j = 0; j < source.Count; ++j)
                {
                    IntPoint p = source[j];
                    Vec2D d = new Vec2D(p.X * backScaling - shift.X, p.Y * backScaling - shift.Y);
                    list.Add(d);
                    if (double.IsNaN(d.X) || double.IsNaN(d.Y))
                    {

                    }
                }
                result.Add(list);
            }



            if (sourcePointIndices != null)
            {
                for (int i = 0; i < solution.Count; ++i)
                {
                    List<IntPoint> source = solution[i];
                    List<int> list = new List<int>(source.Count);
                    for (int j = 0; j < source.Count; ++j)
                    {
                        IntPoint p = source[j];
                        list.Add((int)p.Z - 1);
                        //list.Add(p.Z);
                    }
                    sourcePointIndices.Add(list);
                }
            }

            return result;
        }


        public static bool Operate(ClipType ct, List<List<Vec2D>> polygonSubject, List<List<Vec2D>> polygonClip, out List<List<Vec2D>> result,
            List<List<int>> sourcePointIndices = null, bool aClosed = true, bool bClosed = true, PolyFillType subjFillType = PolyFillType.pftEvenOdd, PolyFillType clipFillType = PolyFillType.pftEvenOdd)
        {
            Vec2D shift;
            double scaling = Scaling(out shift, polygonSubject, polygonClip);
            double backScaling = 1.0 / (High * scaling);

            Clipper c = new Clipper();
            int indexOffset = 0;
            for (int i = 0; i < polygonSubject.Count; ++i)
            {
                var poly = polygonSubject[i];
                List<IntPoint> a = Convert(poly, scaling, shift, zIndexOffset: indexOffset);
                c.AddPath(a, PolyType.ptSubject, aClosed);
                indexOffset += poly.Count;
            }

            for (int i = 0; i < polygonClip.Count; ++i)
            {
                var poly = polygonClip[i];
                List<IntPoint> b = Convert(poly, scaling, shift, zIndexOffset: indexOffset);
                c.AddPath(b, PolyType.ptClip, bClosed);
                indexOffset += poly.Count;
            }

            ISolution solution;
            if (aClosed)
                solution = new PathsSolution(subjFillType, clipFillType);
            else
                solution = new PolyTtreeSolution(subjFillType, clipFillType);
            bool success = solution.Execute(c, ct);



            result = new List<List<Vec2D>>(solution.SolutionCount);
            for (int i = 0; i < solution.SolutionCount; ++i)
            {
                List<IntPoint> source = solution[i];
                List<Vec2D> list = new List<Vec2D>(source.Count);
                for (int j = 0; j < source.Count; ++j)
                {
                    IntPoint p = source[j];
                    Vec2D d = new Vec2D(p.X * backScaling - shift.X, p.Y * backScaling - shift.Y);
                    list.Add(d);
                    if (double.IsNaN(d.X) || double.IsNaN(d.Y))
                    {

                    }
                }
                result.Add(list);
            }

            if (sourcePointIndices != null)
            {
                for (int i = 0; i < solution.SolutionCount; ++i)
                {
                    List<IntPoint> source = solution[i];
                    List<int> list = new List<int>(source.Count);
                    for (int j = 0; j < source.Count; ++j)
                    {
                        IntPoint p = source[j];
                        list.Add((int)p.Z - 1);
                    }
                    sourcePointIndices.Add(list);
                }
            }

            return success;
        }


        private interface ISolution
        {
            bool Execute(Clipper c, ClipType ct);
            int SolutionCount { get; }
            List<IntPoint> this[int i] { get; }
        }

        private class PathsSolution : ISolution
        {
            private List<List<IntPoint>> solution;
            private PolyFillType subjFillType, clipFillType;


            public PathsSolution(PolyFillType subjFillType = PolyFillType.pftEvenOdd, PolyFillType clipFillType = PolyFillType.pftEvenOdd)
            {
                this.subjFillType = subjFillType;
                this.clipFillType = clipFillType;
            }

            public bool Execute(Clipper c, ClipType ct)
            {
                solution = new List<List<IntPoint>>();
                return c.Execute(ct, solution, subjFillType, clipFillType);
            }

            public List<IntPoint> this[int i] { get { return solution[i]; } }

            public int SolutionCount { get { return solution.Count; } }
        }

        private class PolyTtreeSolution : ISolution
        {
            private PolyTree solution;
            private PolyFillType subjFillType, clipFillType;


            public PolyTtreeSolution(PolyFillType subjFillType = PolyFillType.pftEvenOdd, PolyFillType clipFillType = PolyFillType.pftEvenOdd)
            {
                this.subjFillType = subjFillType;
                this.clipFillType = clipFillType;
            }

            public bool Execute(Clipper c, ClipType ct)
            {
                solution = new PolyTree();
                return c.Execute(ct, solution, subjFillType, clipFillType);
            }

            public List<IntPoint> this[int i] { get { return solution.m_AllPolys[i].Contour; } }

            public int SolutionCount { get { return solution.m_AllPolys.Count; } }
        }

        public static double Scaling(out Vec2D shift, params IList<List<Vec2D>>[] polygons)
        {
            Vec2D min, max;

            MinMax(out min, out max, polygons);

            double normalize = 1.0 / Math.Max(max.X - min.X, max.Y - min.Y);
            shift = -0.5 * (min + max);

            return normalize;
        }

        private static void MinMax(out Vec2D min, out Vec2D max, params IList<List<Vec2D>>[] polygons)
        {
            min = new Vec2D(double.MaxValue, double.MaxValue);
            max = new Vec2D(double.MinValue, double.MinValue);

            for (int i = 0; i < polygons.Length; ++i)
            {
                var list = polygons[i];
                for (int k = 0; k < list.Count; ++k)
                {
                    IList<Vec2D> polygon = list[k];
                    int l = polygon.Count;
                    for (int j = 0; j < l; ++j)
                    {
                        Vec2D p = polygon[j];

                        if (p.X < min.X) min.X = p.X; if (p.Y < min.Y) min.Y = p.Y;
                        if (p.X > max.X) max.X = p.X; if (p.Y > max.Y) max.Y = p.Y;
                    }
                }
            }
        }

        public static double Scaling(out Vec2D shift, params IList<Vec2D>[] polygons)
        {
            Vec2D min, max;

            MinMax(out min, out max, polygons);

            double normalize = 1.0 / Math.Max(max.X - min.X, max.Y - min.Y);
            shift = -0.5 * (min + max);

            return normalize;
        }

        private static void MinMax(out Vec2D min, out Vec2D max, params IList<Vec2D>[] polygons)
        {
            min = new Vec2D(double.MaxValue, double.MaxValue);
            max = new Vec2D(double.MinValue, double.MinValue);

            for (int i = 0; i < polygons.Length; ++i)
            {
                IList<Vec2D> polygon = polygons[i];
                int l = polygon.Count;
                for (int j = 0; j < l; ++j)
                {
                    Vec2D p = polygon[j];

                    if (p.X < min.X) min.X = p.X; if (p.Y < min.Y) min.Y = p.Y;
                    if (p.X > max.X) max.X = p.X; if (p.Y > max.Y) max.Y = p.Y;
                }
            }
        }

        public static List<IntPoint> Convert(List<Vec2D> points, out double backScaling, out Vec2D backShift, int count = -1, int zIndexOffset = 0)
        {
            Vec2D shift;
            double scaling = Scaling(out shift, points);

            backScaling = 1.0 / (High * scaling);
            backShift = -shift;

            return Convert(points, scaling, shift);
        }

        public static List<Vec2D> Convert(List<IntPoint> points, double backScaling, Vec2D backShift)
        {
            int l = points.Count;
            List<Vec2D> result = new List<Vec2D>(l);
            for (int i = 0; i < l; ++i)
            {
                var p = points[i];
                result.Add(new Vec2D(backScaling * (p.X - backShift.X), backScaling * (p.Y - backShift.Y)));
            }
            return result;
        }

        public const long High = long.MaxValue >> 16;

        public static double WorldDistanceToClipperUnits(double worldDistance, params IList<Vec2D>[] polygons)
        {
            double scaling = Scaling(out _, polygons);
            return worldDistance * High * scaling;
        }

        public static List<IntPoint> Convert(List<Vec2D> polygon, double scaling, Vec2D shift, int count = -1, int zIndexOffset = 0)
        {
            int l = polygon.Count;
            if (count >= 0)
                l = count;
            List<IntPoint> path = new List<IntPoint>(l);

            zIndexOffset += 1; //We always want an offset of at least 1 to distinguis points that were inserted by clipper (which have by default index 0) from the others

            //long high = long.MaxValue >> 16; //>> 24
            scaling = High * scaling;
            for (int i = 0; i < l; ++i)
            {
                Vec2D v = polygon[i];
                path.Add(new IntPoint((v.X + shift.X) * scaling, (v.Y + shift.Y) * scaling, i + zIndexOffset)); //Use one based indices because new points will get a zero z by default
            }
            //backScaling = 1.0 / scaling;
            return path;
        }


        public static List<List<Vec2D>> Offset(List<Vec2D> polygons, double offset, JoinType jt = JoinType.jtSquare, EndType endType = EndType.etClosedPolygon, double arcTolerance = 100000000, double miterLimit = 2)
        {
            return Offset(polygons, offset, null, jt, endType, arcTolerance, miterLimit);
        }
        public static List<List<Vec2D>> Offset(List<Vec2D> polygon, double offset, List<List<int>> indices, JoinType jt = JoinType.jtSquare, EndType endType = EndType.etClosedPolygon, double arcTolerance = 100000000, double miterLimit = 2)
        {
            return Offset(new List<List<Vec2D>>() { polygon }, offset, indices, jt, endType, arcTolerance, miterLimit);
        }

        public static List<List<Vec2D>> Offset(List<List<Vec2D>> polygons, double offset, JoinType jt = JoinType.jtSquare, EndType endType = EndType.etClosedPolygon, double arcTolerance = 100000000, double miterLimit = 2)
        {
            return Offset(polygons, offset, null, jt, endType, arcTolerance, miterLimit);
        }

        public static List<List<Vec2D>> Offset(List<List<Vec2D>> polygons, double offset, List<List<int>> indices, List<EndType> endTypes, JoinType jt = JoinType.jtSquare, double arcTolerance = 100000000, double miterLimit = 2)
        {
            PolygonOffsetCalculator poc = new PolygonOffsetCalculator(polygons, endTypes, jt, arcTolerance, miterLimit);
            return poc.Offset(offset, indices);
        }

        public static List<List<Vec2D>> Offset(List<List<Vec2D>> polygons, double offset, List<List<int>> indices, JoinType jt = JoinType.jtSquare, EndType endType = EndType.etClosedPolygon, double arcTolerance = 100000000, double miterLimit = 2)
        {
            PolygonOffsetCalculator poc = new PolygonOffsetCalculator(polygons, jt, endType, arcTolerance, miterLimit);
            return poc.Offset(offset, indices);

            //if (polygon.Count < 3)
            //    return new List<List<Vec2D>>();

            //Vec2D first = polygon[0];
            //Vec2D last = polygon[polygon.Count - 1];
            //bool firstEqualLastPoint = (first - last).LengthSquared < 1e-12;
            //if (firstEqualLastPoint)
            //{
            //    polygon.RemoveAt(polygon.Count - 1);
            //    if (polygon.Count < 3)
            //        return new List<List<Vec2D>>();
            //}

            //Vec2D shift;
            //double scaling = Scaling(out shift, polygon);
            //double backScaling;
            //List<IntPoint> path = Convert(polygon, scaling, shift, out backScaling);

            //List<List<IntPoint>> offsetPath = null;

            //ClipperOffset co = new ClipperOffset();
            //co.ArcTolerance = arcTolerance;
            //co.AddPath(path, /*JoinType.jtSquare*/jt, endType/*EndType.etClosedPolygon*/);

            //offsetPath = new List<List<IntPoint>>();
            //co.Execute(ref offsetPath, offset * (High * scaling));


            //if (offsetPath.Count == 0)
            //    return new List<List<Vec2D>>();

            ////if (offsetPath.Count != 1)
            ////{
            ////    int max = 0;
            ////    int maxIndex = -1;
            ////    for (int i = 0; i < offsetPath.Count; ++i)
            ////    {
            ////        if (offsetPath[i].Count > max)
            ////        {
            ////            maxIndex = i;
            ////            max = offsetPath[i].Count;
            ////        }
            ////    }
            ////    List<IntPoint> tmp = offsetPath[maxIndex];
            ////    offsetPath.Clear();
            ////    offsetPath.Add(tmp);
            ////    //throw new NotSupportedException("The offsetting-process created more than one polygon and it's not defined which one to take");
            ////}
            ////List<IntPoint> p = offsetPath[0];
            //List<List<Vec2D>> result = new List<List<Vec2D>>(offsetPath.Count);
            //for (int i = 0; i < offsetPath.Count; ++i)
            //{
            //    List<IntPoint> p = offsetPath[i];
            //    List<Vec2D> list = new List<Vec2D>(p.Count);
            //    for (int j = 0; j < p.Count; ++j)
            //    {
            //        IntPoint v = p[j];
            //        list.Add(new Vec2D(v.X * backScaling - shift.X, v.Y * backScaling - shift.Y));
            //    }
            //    result.Add(list);
            //}

            //if (indices != null)
            //{
            //    for (int i = 0; i < offsetPath.Count; ++i)
            //    {
            //        List<IntPoint> p = offsetPath[i];
            //        List<int> list = new List<int>(p.Count);
            //        for (int j = 0; j < p.Count; ++j)
            //        {
            //            IntPoint v = p[j];
            //            list.Add((int)(v.Z - 1));
            //        }
            //        indices.Add(list);
            //    }
            //}

            //if (firstEqualLastPoint)
            //{
            //    for (int i = 0; i < result.Count; ++i)
            //    {
            //        if (result[i].Count > 0)
            //            result[i].Add(result[i][0]);
            //        if (indices != null && indices[i].Count > 0)
            //            indices[i].Add(indices[i][0]);
            //    }
            //}

            //return result;
        }



        #region Batched

        private struct IntersectionPoint
        {
            public Vec2D Point;
            public int ClosedPolyIndex;
            public int ClipStripIndex;
            public bool StepInsidePolygon;
            public bool StepIntoClipArea { get { return StepInsidePolygon; } } //Assumes both polygons are oriented clock-wise, no holes supported at the moment

            public IntersectionPoint(Vec2D point, int closedPolyIndex, int clipStripIndex, bool stepInsidePolygon)
            {
                Point = point;
                ClosedPolyIndex = closedPolyIndex;
                ClipStripIndex = clipStripIndex;
                StepInsidePolygon = stepInsidePolygon;
            }
        }

        private struct Box
        {
            public IntPoint Min;
            public IntPoint Max;

            public Box(IntPoint min, IntPoint max)
            {
                Min = min;
                Max = max;
            }

            public static bool Overlap(Box a, Box b)
            {
                return !(a.Max.X < b.Min.X || b.Max.X < a.Min.X || a.Max.Y < b.Min.Y || b.Max.Y < a.Min.Y);
            }
        }

        private class Batch
        {
            private List<List<IntPoint>> polygons = new List<List<IntPoint>>();
            private List<Box> boxes = new List<Box>();

            public int Count { get { return polygons.Count; } }
            public List<List<IntPoint>> Polygons { get { return polygons; } }

            public bool Add(List<IntPoint> poly, Box box)
            {
                for (int i = 0; i < boxes.Count; ++i)
                {
                    if (Box.Overlap(boxes[i], box))
                        return false;
                }
                boxes.Add(box);
                polygons.Add(poly);
                return true;
            }
        }

        private static Box CreateBoundingBox(List<IntPoint> poly)
        {
            Box b = new Box();
            b.Max = poly[0];
            b.Min = b.Max;
            for (int i = 1; i < poly.Count; ++i)
            {
                IntPoint p = poly[i];
                if (p.X < b.Min.X) b.Min.X = p.X; if (p.X > b.Max.X) b.Max.X = p.X;
                if (p.Y < b.Min.Y) b.Min.Y = p.Y; if (p.Y > b.Max.Y) b.Max.Y = p.Y;
            }
            return b;
        }

        public static bool OperateBatched(ClipType ct, List<List<IntPoint>> polygonSubject, List<List<IntPoint>> polygonClip,
            out List<PolyNode>[] result, bool aClosed = true, bool bClosed = true, int batchSize = 128)
        {
            Box[] boxes = new Box[polygonClip.Count];
            for (int i = 0; i < polygonClip.Count; ++i)
                boxes[i] = CreateBoundingBox(polygonClip[i]);

            List<Batch> batches = new List<Batch>();
            List<int> todo = new List<int>(polygonClip.Count);
            for (int i = 0; i < polygonClip.Count; ++i)
                todo.Add(i);
            List<int> next = new List<int>(polygonClip.Count);
            while (todo.Count > 0)
            {
                next.Clear();
                Batch b = new Batch();
                for (int i = 0; i < todo.Count; ++i)
                {
                    int id = todo[i];
                    var poly = polygonClip[id];
                    if (!b.Add(poly, boxes[id]))
                    {
                        next.Add(id);
                    }
                    else
                    {
                        for (int j = 0; j < poly.Count; ++j)
                        {
                            IntPoint p = poly[j];
                            p.UpperZ = id;
                            poly[j] = p;
                        }
                    }

                    if (b.Count >= batchSize)
                    {
                        for (int j = i + 1; j < todo.Count; ++j)
                            next.Add(todo[j]);
                        break;
                    }
                }
                batches.Add(b);

                var tmp = todo;
                todo = next;
                next = tmp;
            }

            //result = new List<List<IntPoint>>();
            List<PolyNode>[] buffer = new List<PolyNode>[polygonClip.Count/*batches.Count*/];
            for (int i = 0; i < polygonClip.Count; ++i)
                buffer[i] = new List<PolyNode>();

            Parallel.For(0, batches.Count, i =>
            {
                Clipper c = new Clipper(Clipper.ioPreserveCollinear);
                c.ZFillFunction = delegate (IntPoint bot1, IntPoint top1,
                    IntPoint bot2, IntPoint top2, ref IntPoint pt)
                {
                    if (bot1.UpperZ != 0) pt.UpperZ = bot1.UpperZ;
                    else if (bot2.UpperZ != 0) pt.UpperZ = bot2.UpperZ;
                    else if (top1.UpperZ != 0) pt.UpperZ = top1.UpperZ;
                    else pt.UpperZ = top2.UpperZ;
                };

                if (!c.AddPaths(polygonSubject, PolyType.ptSubject, aClosed))
                    throw new Exception("Polygon clipping could not add subjects");
                if (!c.AddPaths(batches[i].Polygons, PolyType.ptClip, bClosed))
                    throw new Exception("Polygon clipping could not add clip objects");

                //var r = new List<List<IntPoint>>();
                PolyTree tree = new PolyTree();
                bool success = c.Execute(ct, tree);
                for (int j = 0; j < tree.m_Childs.Count; ++j)
                {
                    PolyNode list = tree.m_Childs[j];
                    if (list.m_polygon.Count > 0)
                    {
                        int id = list.m_polygon[0].UpperZ;
                        if (id == 0)
                        {
                            for (int k = 1; k < list.m_polygon.Count; ++k)
                            {
                                if (list.m_polygon[k].UpperZ != 0)
                                {
                                    id = list.m_polygon[k].UpperZ;
                                    break;
                                }
                            }
                        }


                        lock (buffer[id])
                            buffer[id].Add(list);
                    }
                }
            });

            result = buffer;
            return true;
        }

        #endregion
    }

    public class PolygonOffsetCalculator
    {
        private readonly List<List<Vec2D>> polygons;
        private readonly Vec2D shift;
        //private readonly bool firstEqualLastPoint;
        private readonly ClipperOffset co;
        private readonly double scaling;
        private readonly double backScaling;


        public PolygonOffsetCalculator(List<Vec2D> polygon, JoinType jt = JoinType.jtSquare,
            EndType endType = EndType.etClosedPolygon, double arcTolerance = 100000000, double miterLimit = 2)
            : this(new List<List<Vec2D>>() { polygon }, jt, endType, arcTolerance, miterLimit)
        { }

        public PolygonOffsetCalculator(List<List<Vec2D>> polygons, JoinType jt = JoinType.jtSquare,
            EndType endType = EndType.etClosedPolygon, double arcTolerance = 100000000, double miterLimit = 2)
            : this(polygons, BuildEndTypeList(endType, polygons.Count), jt, arcTolerance, miterLimit)
        { }

        private static List<EndType> BuildEndTypeList(EndType endType, int count)
        {
            List<EndType> endTypes = new List<EndType>(count);
            for (int i = 0; i < count; ++i)
                endTypes.Add(endType);
            return endTypes;
        }

        public PolygonOffsetCalculator(List<List<Vec2D>> polygons, List<EndType> endTypes, JoinType jt = JoinType.jtSquare,
             double arcTolerance = 100000000, double miterLimit = 2)
        {
            this.polygons = polygons;

            scaling = PolygonManipulator.Scaling(out shift, polygons);
            backScaling = 1.0 / (PolygonManipulator.High * scaling);


            co = new ClipperOffset(miterLimit, arcTolerance);
            int indexOffset = 0;
            for (int i = 0; i < polygons.Count; ++i)
            {
                var polygon = polygons[i];
                if (polygon.Count < 3)
                    throw new Exception();

                List<IntPoint> path = PolygonManipulator.Convert(polygon, scaling, shift, zIndexOffset: indexOffset);
                co.AddPath(path, /*JoinType.jtSquare*/jt, endTypes[i]/*EndType.etClosedPolygon*/);
                indexOffset += polygon.Count;
            }
        }

        public List<List<Vec2D>> Offset(double offset)
        {
            return Offset(offset, null);
        }

        public List<List<Vec2D>> Offset(double offset, List<List<int>> indices)
        {
            List<List<IntPoint>> offsetPath = new List<List<IntPoint>>();
            co.Execute(ref offsetPath, offset * (PolygonManipulator.High * scaling));


            if (offsetPath.Count == 0)
                return new List<List<Vec2D>>();

            //if (offsetPath.Count != 1)
            //{
            //    int max = 0;
            //    int maxIndex = -1;
            //    for (int i = 0; i < offsetPath.Count; ++i)
            //    {
            //        if (offsetPath[i].Count > max)
            //        {
            //            maxIndex = i;
            //            max = offsetPath[i].Count;
            //        }
            //    }
            //    List<IntPoint> tmp = offsetPath[maxIndex];
            //    offsetPath.Clear();
            //    offsetPath.Add(tmp);
            //    //throw new NotSupportedException("The offsetting-process created more than one polygon and it's not defined which one to take");
            //}
            //List<IntPoint> p = offsetPath[0];
            List<List<Vec2D>> result = new List<List<Vec2D>>(offsetPath.Count);
            for (int i = 0; i < offsetPath.Count; ++i)
            {
                List<IntPoint> p = offsetPath[i];
                List<Vec2D> list = new List<Vec2D>(p.Count);
                for (int j = 0; j < p.Count; ++j)
                {
                    IntPoint v = p[j];
                    list.Add(new Vec2D(v.X * backScaling - shift.X, v.Y * backScaling - shift.Y));
                }
                result.Add(list);
            }

            if (indices != null)
            {
                for (int i = 0; i < offsetPath.Count; ++i)
                {
                    List<IntPoint> p = offsetPath[i];
                    List<int> list = new List<int>(p.Count);
                    for (int j = 0; j < p.Count; ++j)
                    {
                        IntPoint v = p[j];
                        list.Add((int)(v.Z - 1));
                    }
                    indices.Add(list);
                }
            }

            /*if (firstEqualLastPoint)
            {
                for (int i = 0; i < result.Count; ++i)
                {
                    if (result[i].Count > 0)
                        result[i].Add(result[i][0]);
                    if (indices != null && indices[i].Count > 0)
                        indices[i].Add(indices[i][0]);
                }
            }*/

            return result;
        }

        ////One based indices to distinguish existing points from new points (new points have index zero)
        //public List<List<Vec2D>> OffsetNoIndexPostProcessing(double offset, List<List<long>> indices)
        //{
        //    List<List<IntPoint>> offsetPath = new List<List<IntPoint>>();
        //    co.Execute(ref offsetPath, offset * (PolygonManipulator.High * scaling));


        //    if (offsetPath.Count == 0)
        //        return new List<List<Vec2D>>();

        //    //if (offsetPath.Count != 1)
        //    //{
        //    //    int max = 0;
        //    //    int maxIndex = -1;
        //    //    for (int i = 0; i < offsetPath.Count; ++i)
        //    //    {
        //    //        if (offsetPath[i].Count > max)
        //    //        {
        //    //            maxIndex = i;
        //    //            max = offsetPath[i].Count;
        //    //        }
        //    //    }
        //    //    List<IntPoint> tmp = offsetPath[maxIndex];
        //    //    offsetPath.Clear();
        //    //    offsetPath.Add(tmp);
        //    //    //throw new NotSupportedException("The offsetting-process created more than one polygon and it's not defined which one to take");
        //    //}
        //    //List<IntPoint> p = offsetPath[0];
        //    List<List<Vec2D>> result = new List<List<Vec2D>>(offsetPath.Count);
        //    for (int i = 0; i < offsetPath.Count; ++i)
        //    {
        //        List<IntPoint> p = offsetPath[i];
        //        List<Vec2D> list = new List<Vec2D>(p.Count);
        //        for (int j = 0; j < p.Count; ++j)
        //        {
        //            IntPoint v = p[j];
        //            list.Add(new Vec2D(v.X * backScaling - shift.X, v.Y * backScaling - shift.Y));
        //        }
        //        result.Add(list);
        //    }

        //    if (indices != null)
        //    {
        //        for (int i = 0; i < offsetPath.Count; ++i)
        //        {
        //            List<IntPoint> p = offsetPath[i];
        //            List<long> list = new List<long>(p.Count);
        //            for (int j = 0; j < p.Count; ++j)
        //            {
        //                IntPoint v = p[j];
        //                list.Add(v.Z);
        //            }
        //            indices.Add(list);
        //        }
        //    }

        //    if (firstEqualLastPoint)
        //    {
        //        for (int i = 0; i < result.Count; ++i)
        //        {
        //            if (result[i].Count > 0)
        //                result[i].Add(result[i][0]);
        //            if (indices != null && indices[i].Count > 0)
        //                indices[i].Add(indices[i][0]);
        //        }
        //    }

        //    return result;
        //}
    }
}
