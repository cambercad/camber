//using GeoCore;
//using System;
//using System.Collections.Generic;

//namespace Curves
//{
//    public static class CurveConnector
//    {
//        public static bool IsSolidBody(List<bool> c)
//        {
//            int j = 0;
//            for (int i = 0; i < c.Count; ++i)
//            {
//                if (c[i])
//                    ++j;
//                else
//                    --j;
//            }

//            bool closed;
//            if (j == c.Count) closed = true;
//            //else if (j == -c.Count) closed = false;
//            //else throw new Exception("Extrude would lead to mixed body (sheets and solids). Mixed bodies are not supported.");
//            else closed = false;
//            return closed;
//        }

//        //public static List<List<int>> ConnectCurves(IList<Curve2D> data, double eps, out bool closed)
//        //{
//        //    List<bool> c;
//        //    List<List<int>> r = ConnectCurves(data, eps, out c);

//        //    int j = 0;
//        //    for (int i = 0; i < c.Count; ++i)
//        //    {
//        //        if (c[i])
//        //            ++j;
//        //        else
//        //            --j;
//        //    }

//        //    if (j == c.Count) closed = true;
//        //    //else if (j == -c.Count) closed = false;
//        //    //else throw new Exception("Extrude would lead to mixed body (sheets and solids). Mixed bodies are not supported.");
//        //    else closed = false;

//        //    return r;
//        //}

//        public static List<List<int>> ConnectCurves(IList<Curve2D> data, double eps, out List<bool> closed)
//        {
//            List<List<int>> list = new List<List<int>>();
//            List<int> v = new List<int>();
//            list.Add(v);

//            Curve2D[] curves = new Curve2D[data.Count];
//            for (int i = 0; i < data.Count; ++i)
//                curves[i] = data[i];

//            Curve2D c = curves[0];
//            ValuePair<Vec2D, Vec2D> s = new ValuePair<Vec2D, Vec2D>(c.StartPosition, c.EndPosition);
//            v.Add(0);
//            curves[0] = null;

//            int counter = 1;
//            while (true)
//            {
//                if (counter == curves.Length)
//                {
//                    closed = new List<bool>(list.Count);
//                    for (int i = 0; i < list.Count; ++i)
//                    {
//                        v = list[i];
//                        Vec2D start = GetStart(data, v[0]);
//                        Vec2D end = GetEnd(data, v[v.Count - 1]);

//                        closed.Add(AreEqual(start, end, eps));
//                    }

//                    return list;
//                }


//                bool success = false;
//                for (int i = 0; i < curves.Length; ++i)
//                {
//                    c = curves[i];
//                    if (c == null)
//                        continue;

//                    bool match = false;
//                    if (Math.Abs(v[0]) != i)
//                    {
//                        //Try connecting at start of curve-strip
//                        int j = v[0];
//                        Curve2D d = data[Math.Abs(j)];
//                        Vec2D start = j >= 0 ? d.StartPosition : d.EndPosition;
//                        if (AreEqual(start, c.EndPosition, eps))
//                        {
//                            v.Insert(0, i);
//                            match = true;
//                        }
//                        else if (AreEqual(start, c.StartPosition, eps))
//                        {
//                            v.Insert(0, -i);
//                            match = true;
//                        }
//                    }
//                    if (!match && Math.Abs(v[v.Count - 1]) != i)
//                    {
//                        //Try connecting at end of curve-strip
//                        int j = v[v.Count - 1];
//                        Curve2D d = data[Math.Abs(j)];
//                        Vec2D end = j >= 0 ? d.EndPosition : d.StartPosition;
//                        if (AreEqual(end, c.StartPosition, eps))
//                        {
//                            v.Add(i);
//                            match = true;
//                        }
//                        else if (AreEqual(end, c.EndPosition, eps))
//                        {
//                            v.Add(-i);
//                            match = true;
//                        }
//                    }

//                    if (match)
//                    {
//                        success = true;
//                        curves[i] = null;
//                        ++counter;
//                    }
//                }
//                if (!success)
//                {
//                    v = new List<int>();
//                    list.Add(v);
//                    for (int i = 0; i < curves.Length; ++i)
//                    {
//                        if (curves[i] != null)
//                        {
//                            v.Add(i);
//                            curves[i] = null;
//                            ++counter;
//                            break;
//                        }
//                    }
//                }
//            }
//        }

//        public static Vec2D GetStart(IList<Curve2D> curves, int id)
//        {
//            if (id < 0)
//                return curves[-id].EndPosition;
//            else
//                return curves[id].StartPosition;
//        }

//        public static Vec2D GetEnd(IList<Curve2D> curves, int id)
//        {
//            if (id < 0)
//                return curves[-id].StartPosition;
//            else
//                return curves[id].EndPosition;
//        }

//        private static bool AreEqual(Vec2D a, Vec2D b, double eps)
//        {
//            double dx = a.X - b.X;
//            double dy = a.Y - b.Y;
//            return dx * dx + dy * dy < eps * eps;
//        }
//    }
//}
