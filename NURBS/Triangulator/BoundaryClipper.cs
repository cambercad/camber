using System.Numerics;
using GeoCore;

namespace NURBS
{
    public struct IntPointArithmetic : ITriangulationArithmetic<IntPoint, long>
    {
        public int Compare(in long a, in long b) { return a.CompareTo(b); }
        public long Get(in IntPoint v, int componentIndex) { return v.Get(componentIndex); }
        public void Set(ref IntPoint v, int componentIndex, long value) { v.Get(componentIndex) = value; }
        public long Max(in long a, in long b) { return Math.Max(a, b); }
        public long Min(in long a, in long b) { return Math.Min(a, b); }
        public int Orient2D(in IntPoint pa, in IntPoint pb, in IntPoint pc)
        {
            return Int128.Sign(SignedArea2DTimesTwoInt128(pa, pb, pc));
        }

        public BigRationalHybrid SignedArea2DTimesTwo(in IntPoint pa, in IntPoint pb, in IntPoint pc)
        {
            Int128 s = SignedArea2DTimesTwoInt128(pa, pb, pc);
            return new BigRationalHybrid((BigInteger)s, BigInteger.One);
        }

        private static Int128 SignedArea2DTimesTwoInt128(in IntPoint pa, in IntPoint pb, in IntPoint pc)
        {
            long acx = pa.X - pc.X;
            long bcx = pb.X - pc.X;
            long acy = pa.Y - pc.Y;
            long bcy = pb.Y - pc.Y;
            return (Int128)acx * (Int128)bcy - (Int128)acy * (Int128)bcx;
        }

        public Box2D GetBounds(in IntPoint v)
        {
            return new Box2D(new Vec2D(v.X, v.Y), new Vec2D(v.X, v.Y));
        }
    }

    public class Node
    {
        public int ID { get; private set; }
        public List<int> Indices { get; private set; }
        public List<Node> TopNodes { get; private set; } //The nodes that contain the polygon represented by this node
        public List<Node> SubNodes { get; private set; } //The nodes contained in the polygon represented by this node

        public Node(List<int> indices, int id)
        {
            Indices = indices;
            ID = id;
            TopNodes = new List<Node>();
            SubNodes = new List<Node>();
        }

        public bool ContainsPoint(List<Vec2D> points, Vec2D p) { return Polygon.IsPointInPolygon(points, Indices, p) == PointInPolygonResult.Inside; }

        public void AddTopNode(Node n) { TopNodes.Add(n); }

        public void AddSubNode(Node n) { SubNodes.Add(n); }

        public double MaxX(List<Vec2D> points) { int index; return MaxX(Indices, points, out index).X; }

        public static Vec2D MaxX(List<int> Indices, List<Vec2D> points, out int index)
        {
            index = -1;
            Vec2D max = new Vec2D(double.MinValue, 0);
            for (int i = 0; i < Indices.Count; ++i)
            {
                Vec2D p = points[Indices[i]];
                if (p.X > max.X)
                {
                    max = p;
                    index = i;
                }
            }
            return max;
        }

        public static List<Node> BuildVertexLoopTree(List<Vec2D> points, List<List<int>> polygons)
        {
            List<Node> nodes = new List<Node>(polygons.Count);
            for (int i = 0; i < polygons.Count; ++i)
            {
                List<int> polygon = polygons[i];
                nodes.Add(new Node(polygon, i));
            }

            for (int i = 0; i < nodes.Count; ++i)
            {
                Node n = nodes[i];
                for (int j = 0; j < nodes.Count; ++j)
                {
                    if (i != j)
                    {
                        Node n2 = nodes[j];
                        if (n.ContainsPoint(points, points[n2.Indices[0]]))
                        {
                            //n.AddSubNode(n2);
                            n2.AddTopNode(n);
                        }
                    }
                }
            }

            List<Node> rootNodes = new List<Node>();
            for (int i = 0; i < nodes.Count; ++i)
            {
                Node n = nodes[i];
                if (n.TopNodes.Count == 0)
                    rootNodes.Add(n);
                else
                {
                    double minX = double.MaxValue;
                    int index = -1;
                    for (int j = 0; j < n.TopNodes.Count; ++j)
                    {
                        Node tn = n.TopNodes[j];
                        double max = tn.MaxX(points);
                        if (max < minX)
                        {
                            minX = max;
                            index = j;
                        }
                    }

                    n.TopNodes[index].AddSubNode(n);
                }
            }

            return rootNodes;
        }
    }
    public static class BoundaryClipper
    {
        public static void MinMax(IList<Vec2D> points, IList<int> indices, out Vec2D min, out Vec2D max)
        {
            min = new Vec2D(double.MaxValue, double.MaxValue);
            max = new Vec2D(double.MinValue, double.MinValue);
            int l = indices.Count;
            for (int i = 0; i < l; ++i)
            {
                Vec2D p = points[indices[i]];
                if (p.X < min.X) min.X = p.X; if (p.Y < min.Y) min.Y = p.Y;
                if (p.X > max.X) max.X = p.X; if (p.Y > max.Y) max.Y = p.Y;
            }
        }

        private static void BufferPoint(List<BoxNode2D> tree, List<IntPoint>[] additionalBorderPointBuffer, Vec2D p, IntPoint ip, double enlargement = 1e-6)
        {
            BoxNode2D pointBox = new BoxNode2D(p.X - enlargement, p.Y - enlargement, p.X + enlargement, p.Y + enlargement, -1);
            BoxTree2D.Traverse(tree, pointBox, delegate (BoxNode2D a, BoxNode2D b, int id)
            {
                List<IntPoint> buffer = additionalBorderPointBuffer[a.ID];
                if (buffer != null)
                    buffer.Add(ip);
                else
                    additionalBorderPointBuffer[a.ID] = new List<IntPoint>() { ip };

                return false;
            },
            0);
        }

        private static int CheckPointExists(List<BoxNode2D> tree, List<IntPoint>[] additionalBorderPointBuffer, Vec2D p, IntPoint ip, double enlargement = 1e-6)
        {
            int index = -1;
            BoxNode2D pointBox = new BoxNode2D(p.X - enlargement, p.Y - enlargement, p.X + enlargement, p.Y + enlargement, -1);
            BoxTree2D.Traverse(tree, pointBox, delegate (BoxNode2D a, BoxNode2D b, int id)
            {
                List<IntPoint> buffer = additionalBorderPointBuffer[a.ID];
                if (buffer != null)
                {
                    //There might already be another point with the same location
                    for (int i = 0; i < buffer.Count; ++i)
                    {
                        IntPoint candidate = buffer[i];
                        // if (Math.Abs(candidate.X - ip.X) <= 10 && Math.Abs(candidate.Y - ip.Y) <= 10)
                        if (candidate.X == ip.X && candidate.Y == ip.Y)
                        {
                            index = candidate.LowerZ; // (int)candidate.Z;
                            return true;
                        }
                    }
                }

                return false;
            },
            0);
            return index;
        }

        public static List<Tri>[] TriangulateDefaultBorder(List<Vec2D> points, IList<List<int>> polygons)
        {
            // Preserve distinct surface parameters, including adjacent floating-point
            // values near knots. A fixed integer grid can collapse valid UV leaves.
            var pts = new List<Rat2Hybrid>(points.Count);
            foreach (var point in points)
            {
                BigRational x = point.X, y = point.Y;
                pts.Add(new Rat2Hybrid(
                    new BigRationalHybrid(x.Numerator, x.Denominator),
                    new BigRationalHybrid(y.Numerator, y.Denominator)));
            }

            List<Tri>[] buffer = new List<Tri>[polygons.Count];
            Parallel.For(0, polygons.Count, i =>
            {
                List<int> polygon = polygons[i];
                buffer[i] = Triangulator.TriangulatePolygon(pts, polygon);
            });

            return buffer;
        }

        public static List<Tri> TriangulatePolygon(List<IntPoint> points, IList<int> borderPolygon = null)
        {
            TriangulationEarClipping<IntPointArithmetic, IntPoint, long> poly = new TriangulationEarClipping<IntPointArithmetic, IntPoint, long>();

            poly.Initialize(points, borderPolygon);

            List<Tri> triangles = new List<Tri>();
            poly.Triangulate(triangles);
            return triangles;
        }

        public static List<Tri> TriangulatePolygonWithHoles(List<IntPoint> points, List<List<int>> borderPolygon = null)
        {
            return TriangulationWithHoles<IntPointArithmetic, IntPoint, long>.Triangulate(points, borderPolygon);
        }

        public static List<T> Flatten<T>(IList<List<T>> listOfLists)
        {
            int l = listOfLists.Count;
            int counter = 0;
            for (int i = 0; i < l; ++i)
            {
                if (listOfLists[i] != null)
                    counter += listOfLists[i].Count;
            }

            List<T> result = new List<T>(l);
            for (int i = 0; i < l; ++i)
            {
                if (listOfLists[i] != null)
                    result.AddRange(listOfLists[i]);
            }
            return result;
        }

        public static List<Tri>[] ClipOutsideBoundary(List<Vec2D> points, IList<List<int>> polygons, List<List<int>> borderLoops, double tol = 1e-8, double eps = 1e-12, double largeTolerance = 1e-4, bool removeZeroAreaTriangles = false)
        {
            int numPoly = polygons.Count;
            BoxNode2D[] polyBoxes = new BoxNode2D[numPoly];
            for (int i = 0; i < numPoly; ++i)
            {
                List<int> poly = polygons[i];
                Vec2D min, max;
                MinMax(points, poly, out min, out max);
                polyBoxes[i] = new BoxNode2D(min.X, min.Y, max.X, max.Y, i/*, 2 * tol*/);
            }

            List<BoxNode2D> tree = BoxTree2DParallel.BuildTree(polyBoxes); //This enlarges all boxes a bit

            return ClipOutsideBoundary(points, polygons, tree, borderLoops, tol, eps, largeTolerance, removeZeroAreaTriangles);
        }

        public static List<Tri>[] ClipOutsideBoundary(List<Vec2D> points, IList<List<int>> polygons, List<BoxNode2D> tree,
            List<List<int>> borderLoops, double tol = 1e-8, double eps = 1e-12, double largeTolerance = 1e-4, bool removeZeroAreaTriangles = false)
        {
            int numPoly = polygons.Count;

            List<IntPoint>[] additionalBorderPointBuffer = new List<IntPoint>[numPoly];
            for (int i = 0; i < additionalBorderPointBuffer.Length; ++i)
                additionalBorderPointBuffer[i] = new List<IntPoint>();

            bool[] candidates = new bool[numPoly];
            //for (int i = 0; i < borderLoops.Count; ++i)
            Parallel.For(0, borderLoops.Count, delegate (int i)
            {
                List<int> loop = borderLoops[i];
                int l = loop.Count;
                for (int j = 0; j < l; ++j)
                {
                    int start = loop[j];
                    int end = loop[(j + 1) % l];

                    Vec2D s = points[start];
                    Vec2D e = points[end];
                    Vec2D d = e - s;
                    BoxNode2D lineBox = new BoxNode2D(s, e, j);
                    BoxTree2D.Traverse(tree, lineBox, delegate (BoxNode2D a, BoxNode2D b, int index)
                    {
                        if (a.IntersectsLineSegment(s, e))
                            candidates[a.ID] = true;
                        return false;
                    },
                    0);
                }
            });



            int numPoints = points.Count;
            List<IntPoint> pts = new List<IntPoint>(numPoints);
            long scaling = long.MaxValue >> 16;
            double backScaling = 1.0 / scaling;
            for (int i = 0; i < numPoints; ++i)
            {
                Vec2D p = points[i];
                IntPoint ip = new IntPoint((p.X - 0.5) * scaling, (p.Y - 0.5) * scaling, i + 1);
                //BufferPoint(tree, additionalBorderPointBuffer, p, ip);
                pts.Add(ip);
            }
            for (int i = 0; i < polygons.Count; ++i)
            {
                List<int> poly = polygons[i];
                for (int j = 0; j < poly.Count; ++j)
                    additionalBorderPointBuffer[i].Add(pts[poly[j]]);
            }


            List<Node> rootNodes = Node.BuildVertexLoopTree(points, borderLoops);
            if (rootNodes.Count == 0) throw new Exception("Something went wrong...");

            List<List<IntPoint>> borderTest = new List<List<IntPoint>>();
            for (int i = 0; i < borderLoops.Count; ++i)
            {
                //Determine even/odd index of loop
                //bool odd = IsOdd(rootNodes, i); //If it is odd, then the loop is a hole inside another loop

                List<int> loop = borderLoops[i];
                List<IntPoint> buffer = new List<IntPoint>(loop.Count);
                for (int j = 0; j < loop.Count; ++j)
                {
                    buffer.Add(pts[loop[j]]);
                }
                borderTest.Add(buffer);
            }

            //ConcurrentList<Tri> splittedTriangles = new ConcurrentList<Tri>();
            //for (int i = 0; i < numPoly; ++i)
            List<Tri>[] triBuffer = new List<Tri>[numPoly];

            int numClipPolys = 0;
            for (int i = 0; i < numPoly; ++i)
                if (candidates[i])
                    ++numClipPolys;
            List<List<IntPoint>> clipPolygons = new List<List<IntPoint>>(numClipPolys);
            for (int i = 0; i < numClipPolys; ++i)
                clipPolygons.Add(null);
            //int indexer = -1;

            int[] expander = new int[numClipPolys];
            Dictionary<int, int> compressor = new Dictionary<int, int>(numClipPolys);
            numClipPolys = 0;
            for (int i = 0; i < numPoly; ++i)
            {
                if (candidates[i])
                {
                    compressor.Add(i, numClipPolys);
                    expander[numClipPolys] = i;
                    ++numClipPolys;
                }
            }


            Parallel.For(0, numPoly, delegate (int i)
            {
                List<int> poly = polygons[i];
                if (candidates[i])
                {
                    // LIMITATION: Nested boundary loops (holes inside holes) are not supported.
                    List<IntPoint> po = new List<IntPoint>(poly.Count);
                    for (int j = 0; j < poly.Count; ++j)
                        po.Add(pts[poly[j]]);

                    clipPolygons[compressor[i]] = po;
                    //clipPolygons[Interlocked.Increment(ref indexer)] = po;
                }
                else
                {
                    /*List<Tri> buffer = TriangulatorIntPoint.TriangulateEarClippingDefault(pts, poly);
                    Tri tri = buffer[0];

                    if (!IsPointInClipArea((1.0 / 3.0) * (points[tri.A] + points[tri.B] + points[tri.C]), rootNodes, points))
                    {
                        triBuffer[i] = buffer;
                    }
                    else
                        triBuffer[i] = null;*/



                    if (!IsPointInClipArea(points[poly[0]], rootNodes, points))
                    {
                        //List<Tri> buffer = TriangulatorIntPoint.TriangulateEarClippingDefault(pts, poly);
                        List<Tri> buffer = TriangulatePolygon(pts, poly);
                        triBuffer[i] = buffer;
                    }
                    else
                        triBuffer[i] = null;
                }
            });

            List<PolyNode>[] r;
            PolygonManipulator.OperateBatched(ClipType.ctIntersection, borderTest, clipPolygons, out r);

            //Result r can contain nested loops!
            //a
            //Use polygon hole pyramid and subject IntPoint's z indices (negative z-values)

            Parallel.For(0, r.Length, j =>
            {
                List<PolyNode> nodes = r[j];

                List<Tri> buffer = null;

                for (int m = 0; m < nodes.Count; ++m)
                {
                    PolyNode node = nodes[m];
                    List<List<IntPoint>> loops = new List<List<IntPoint>>();
                    //List<IntPoint> loop = node.m_polygon;// loops[m];
                    loops.Add(node.m_polygon);
                    for (int i = 0; i < node.ChildCount; ++i)
                    {
                        loops.Add(node.m_Childs[i].m_polygon);
                        if (node.m_Childs[i].m_Childs.Count != 0)
                            throw new Exception("Triangulatro does not yet support holes in holes");
                    }
                    //if (node.ChildCount != 0)
                    //{
                    //    System.Diagnostics.Debug.Write("Holes: " + node.ChildCount);

                    //    //TODO
                    //}

                    List<List<int>> polys = new List<List<int>>();
                    for (int n = 0; n < loops.Count; ++n)
                    {
                        List<IntPoint> loop = loops[n];
                        List<int> polygon = new List<int>(loop.Count);

                        int l = points.Count;
                        for (int k = 0; k < loop.Count; ++k)
                        {
                            IntPoint p = loop[k];
                            int z = p.LowerZ - 1;// (int)p.Z - 1;
                            if (z >= 0)
                                polygon.Add(z);
                            else
                            {
                                lock (points)
                                {
                                    Vec2D d = new Vec2D(p.X * backScaling + 0.5, p.Y * backScaling + 0.5);
                                    int id = CheckPointExists(tree, additionalBorderPointBuffer, d, p);

                                    if (id > 0)
                                    {
                                        id = id - 1;
                                        polygon.Add(id);
                                    }
                                    else
                                    {
                                        id = points.Count;


                                        p.LowerZ = id + 1;
                                        BufferPoint(tree, additionalBorderPointBuffer, d, p);


                                        points.Add(d);
                                        polygon.Add(id);

                                        pts.Add(p);
                                    }
                                }
                            }
                        }

                        polys.Add(polygon);
                    }

                    List<Tri> tmp;
                    if (polys.Count == 1)
                        tmp = TriangulatePolygon(pts, polys[0]);
                    else
                    {
                        //List<int> main = polys[0];
                        //polys.RemoveAt(0);
                        //tmp = TriangulatorIntPoint.TriangulateEarClipping(pts, main, polys);
                        tmp = TriangulatePolygonWithHoles(pts, polys);
                    }

                    //var tmp = TriangulatorIntPoint.TriangulateEarClippingDefault(pts, polygon);
                    if (buffer == null)
                        buffer = tmp;
                    else
                        buffer.AddRange(tmp);
                }


                triBuffer[expander[j]] = buffer;
            });


            return triBuffer; // splittedTriangles.ToList();
        }

        public static bool IsPointInClipArea(Vec2D point, List<Node> rootNodes, List<Vec2D> points)
        {
            if (rootNodes.Count == 0) return true; //No border -> clip everything?
            for (int i = 0; i < rootNodes.Count; ++i)
            {
                if (!IsPointInClipArea(point, rootNodes[i], points))
                    return false;
            }
            return true;
        }

        public static bool IsPointInClipArea(Vec2D point, Node node, List<Vec2D> points)
        {
            if (node.ContainsPoint(points, point))
            {
                for (int j = 0; j < node.SubNodes.Count; ++j)
                {
                    Node hole = node.SubNodes[j];
                    if (hole.ContainsPoint(points, point))
                    {
                        if (hole.SubNodes.Count == 0)
                            return true;
                        else
                        {
                            for (int k = 0; k < hole.SubNodes.Count; ++k)
                            {
                                if (!IsPointInClipArea(point, hole.SubNodes[k], points))
                                    return false;
                            }
                        }
                    }
                }
                return false;
            }
            return true;
        }
    }
}
