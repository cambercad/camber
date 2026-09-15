namespace GeoCore
{
    public interface ISegment<T>
    {
        T Item1 { get; }
        T Item2 { get; }
    }
    public class SegmentConnector
    {
        public static List<List<int>> ConnectAndResolve(IList<Int2> segments)
        {
            List<List<int>> intListList = new List<List<int>>();
            if (segments.Count == 0)
                return intListList;
            List<int> intList = new List<int>();
            Int2 segment1 = segments[0];
            intList.Add(segment1.X);
            intList.Add(segment1.Y);
            bool[] flagArray = new bool[segments.Count];
            for (int index = 0; index < flagArray.Length; ++index)
                flagArray[index] = false;
            flagArray[0] = true;
            int count = segments.Count;
            int num = 0;
            while (true)
            {
                bool flag = false;
                for (int index = 0; index < count; ++index)
                {
                    Int2 segment2 = segments[index];
                    if (!flagArray[index])
                    {
                        switch (AreConnected(segment2.X, segment2.Y, intList[0], intList[1]))
                        {
                            case 0:
                                intList.Insert(0, segment2.Y);
                                ++num;
                                flagArray[index] = true;
                                flag = true;
                                continue;
                            case 1:
                                intList.Insert(0, segment2.X);
                                ++num;
                                flagArray[index] = true;
                                flag = true;
                                continue;
                            default:
                                switch (AreConnected(segment2.X, segment2.Y, intList[intList.Count - 1], intList[intList.Count - 2]))
                                {
                                    case 0:
                                        intList.Add(segment2.Y);
                                        ++num;
                                        flagArray[index] = true;
                                        flag = true;
                                        continue;
                                    case 1:
                                        intList.Add(segment2.X);
                                        ++num;
                                        flagArray[index] = true;
                                        flag = true;
                                        continue;
                                    default:
                                        continue;
                                }
                        }
                    }
                }
                if (!flag)
                {
                    intListList.Add(intList);
                    intList = new List<int>();
                    for (int index = 0; index < segments.Count; ++index)
                    {
                        if (!flagArray[index])
                        {
                            intList.Add(segments[index].X);
                            intList.Add(segments[index].Y);
                            flagArray[index] = true;
                            break;
                        }
                    }
                    if (intList.Count == 0)
                        break;
                }
            }
            return intListList;
        }

        public static List<List<int>> Connect(IList<Int2> data, out List<bool> closed)
        {
            return Connect(data, delegate (Int2 arg) { return arg.X; }, delegate (Int2 arg) { return arg.Y; }, delegate (int a, int b) { return a == b; }, out closed);
        }

        public static List<List<T>> Connect<T, U>(List<U> segments, double eps, Func<T, T, double, bool> comparer, double largeTolerance = 0.1)
            where U : ISegment<T>
        {
            if (segments.Count == 0)
                return new List<List<T>>();

            List<List<T>> buffer = new List<List<T>>();
            if (segments.Count == 0)
                return buffer;

            List<T> v = new List<T>();

            U s = segments[0];


            v.Add(s.Item1);
            v.Add(s.Item2);
            bool[] done = new bool[segments.Count];
            for (int i = 0; i < done.Length; ++i)
                done[i] = false;
            done[0] = true;

            int l = segments.Count;
            int counter = 0;
            while (/*counter < l*/true)
            {
                bool success = false;

                for (int j = 0; j < l; ++j)
                {
                    s = segments[j];

                    if (done[j])
                        continue;

                    //double d = -1;
                    int res = AreConnected(s.Item1, s.Item2, v[0], v[1], eps, comparer);
                    if (res == 0)
                    {
                        v.Insert(0, s.Item2);
                        ++counter;
                        done[j] = true;
                        success = true;
                        continue;
                        //++a;
                    }
                    else if (res == 1)
                    {
                        v.Insert(0, s.Item1);
                        ++counter;
                        done[j] = true;
                        success = true;
                        continue;
                        //++a;
                    }


                    res = AreConnected(s.Item1, s.Item2, v[v.Count - 1], v[v.Count - 2], eps, comparer);
                    if (res == 0)
                    {
                        v.Add(s.Item2);
                        ++counter;
                        done[j] = true;
                        success = true;
                        continue;
                        //++a;
                    }
                    else if (res == 1)
                    {
                        v.Add(s.Item1);
                        ++counter;
                        done[j] = true;
                        success = true;
                        continue;
                        //++a;
                    }

                }

                if (!success)
                {
                    buffer.Add(v);
                    v = new List<T>();

                    for (int i = 0; i < segments.Count; ++i)
                    {
                        if (!done[i]/* != null*/)
                        {
                            v.Add(segments[i].Item1);
                            v.Add(segments[i].Item2);
                            done[i] = true;
                            break;
                        }
                    }
                    if (v.Count == 0)
                    {
                        //Remove duplicates and connect again
                        RemoveDuplicates(buffer, eps, comparer);

                        return Connect(buffer, largeTolerance, comparer);
                    }
                }
            }

            //return v;
        }

        private static List<List<T>> Connect<T>(List<List<T>> segments, double eps, Func<T, T, double, bool> comparer/*, out List<Tuple<Double3, Double3>> remainingSegments*/)
        {
            if (segments.Count == 0)
                return new List<List<T>>();
            if (segments.Count == 1)
                return segments;

            List<List<T>> buffer = new List<List<T>>();

            List<T> v = new List<T>();
            v.AddRange(segments[0]);
            segments[0] = null;

            int l = segments.Count;
            int counter = 0;
            while (true)
            {
                bool success = false;
                for (int j = 0; j < l; ++j)
                {
                    List<T> s = segments[j];

                    if (s == null)
                        continue;

                    int res = AreConnected(s[0], s[s.Count - 1], v[0], v[1], eps, comparer);
                    if (res == 0)
                    {
                        s = s.GetRange(1, s.Count - 1);
                        s.Reverse();
                        v.InsertRange(0, s);
                        ++counter;
                        segments[j] = null;
                        success = true;
                        continue;
                        //++a;
                    }
                    else if (res == 1)
                    {
                        s = s.GetRange(0, s.Count - 1);
                        v.InsertRange(0, s);
                        ++counter;
                        segments[j] = null;
                        success = true;
                        continue;
                        //++a;
                    }


                    res = AreConnected(s[0], s[s.Count - 1], v[v.Count - 1], v[v.Count - 2], eps, comparer);
                    if (res == 0)
                    {
                        s = s.GetRange(1, s.Count - 1);
                        v.AddRange(s);
                        ++counter;
                        segments[j] = null;
                        success = true;
                        continue;
                        //++a;
                    }
                    else if (res == 1)
                    {
                        s = s.GetRange(0, s.Count - 1);
                        s.Reverse();
                        v.AddRange(s);
                        ++counter;
                        segments[j] = null;
                        success = true;
                        continue;
                        //++a;
                    }

                }


                if (!success)
                {
                    buffer.Add(v);
                    v = new List<T>();
                    for (int i = 0; i < segments.Count; ++i)
                    {
                        if (segments[i] != null)
                        {
                            v.AddRange(segments[i]);
                            segments[i] = null;
                            break;
                        }
                    }
                    if (v.Count == 0)
                        return buffer;
                }
            }
        }

        private static void RemoveDuplicates<T>(List<List<T>> input, double eps, Func<T, T, double, bool> comparer)
        {
            List<int> delete = new List<int>();
            for (int i = 0; i < input.Count; ++i)
            {
                for (int j = i + 1; j < input.Count; ++j)
                {
                    if (AreEqual(input[i], input[j], eps, comparer))
                        delete.Add(i); //delete.Add(j);
                }
            }
            //input.Sort();
            for (int i = delete.Count - 1; i >= 0; --i)
                input.RemoveAt(delete[i]);
        }

        private static bool AreEqual<T>(List<T> a, List<T> b, double eps, Func<T, T, double, bool> comparer)
        {
            if (a.Count != b.Count)
                return false;

            for (int i = 0; i < a.Count; ++i)
                if (!comparer(a[i], b[i], eps)/*(a[i] - b[i]).LengthSquared > eps * eps*/)
                    return false;
            return true;
        }

        private static int AreConnected<T>(T startCandidate, T endCandidate, T connector, T last, double eps, Func<T, T, double, bool> comparer)
        {
            if (comparer(startCandidate, connector, eps) && !comparer(endCandidate, last, eps))
                return 0;
            if (comparer(endCandidate, connector, eps) && !comparer(startCandidate, last, eps))
                return 1;
            return -1;
        }

        private static int AreConnected(int startCandidate, int endCandidate, int connector, int last)
        {
            if (startCandidate == connector && endCandidate != last)
                return 0;
            if (endCandidate == connector && startCandidate != last)
                return 1;
            return -1;
        }

        /*private static bool AreEqual(Double3 a, Double3 b, double eps)
        {
            return Math.Abs(a.X - b.X) < eps && Math.Abs(a.Y - b.Y) < eps && Math.Abs(a.Z - b.Z) < eps;
        }*/

        public static U GetStart<T, U>(IList<T> curves, int id, Func<T, U> getStart, Func<T, U> getEnd)
        {
            if (id < 0)
                return getEnd(curves[-id]);
            else
                return getStart(curves[id]);
        }
        public static U GetEnd<T, U>(IList<T> curves, int id, Func<T, U> getStart, Func<T, U> getEnd)
        {
            //if (id >= 0)
            //    return getEnd(curves[id]);
            //else
            //    return getStart(curves[-id]);

            if (id < 0)
                return getStart(curves[-id]);
            else
                return getEnd(curves[id]);
        }
        private static int Abs(int v) { if (v < 0) return -v; else return v; }
        public static List<List<int>> Connect<T, U>(IList<T> data, Func<T, U> getStart, Func<T, U> getEnd,
            Func<U, U, bool> areEqual, out List<bool> closed, bool removeDuplicates = false)
        {
            if (data.Count == 0)
            {
                closed = new List<bool>();
                return new List<List<int>>();
            }

            List<List<int>> list = new List<List<int>>();
            List<int> v = new List<int>();
            list.Add(v);

            bool[] done = new bool[data.Count];
            for (int i = 0; i < data.Count; ++i)
                done[i] = false;

            int counter = 1;
            if (removeDuplicates)
            {
                int l = data.Count;
                for (int i = 0; i < l; ++i)
                {
                    T a = data[i];
                    U startA = getStart(a);
                    U endA = getEnd(a);
                    for (int j = i + 1; j < l; ++j)
                    {
                        if (done[j])
                            continue;

                        T b = data[j];
                        U startB = getStart(b);
                        U endB = getEnd(b);
                        if (areEqual(startA, startB) && areEqual(endA, endB))
                        {
                            done[j] = true;
                            ++counter;
                        }
                        else if (areEqual(startA, endB) && areEqual(endA, startB))
                        {
                            done[j] = true;
                            ++counter;
                        }
                    }
                }
            }

            T c = data[0];
            ValuePair<U, U> s = new ValuePair<U, U>(getStart(c), getEnd(c));
            v.Add(0);
            done[0] = true;

            //Func<T, T, bool> segmentsEqual = delegate (T a, T b)
            //{
            //    U startA = getStart(a);
            //    U endA = getEnd(a);
            //    U startB = getStart(b);
            //    U endB = getEnd(b);
            //    if (areEqual(startA, startB) && areEqual(endA, endB))
            //        return true;
            //    if (areEqual(startA, endB) && areEqual(endA, startB))
            //        return true;
            //    return false;
            //};


            while (true)
            {
                if (counter == data.Count)
                {
                    closed = new List<bool>(list.Count);
                    for (int i = 0; i < list.Count; ++i)
                    {
                        v = list[i];
                        U start = GetStart(data, v[0], getStart, getEnd);
                        U end = GetEnd(data, v[v.Count - 1], getStart, getEnd);

                        closed.Add(areEqual(start, end));
                    }

                    return list;
                }


                bool success = false;
                for (int i = 1; i < data.Count; ++i)
                {
                    if (done[i])
                        continue;
                    c = data[i];

                    bool match = false;
                    if (Abs(v[0]) != i)
                    {
                        //Try connecting at start of curve-strip
                        int j = v[0];
                        T d = data[Abs(j)];

                        /*if (removeDuplicates && segmentsEqual(c, d))
                        {
                            match = true;
                        }
                        else
                        {*/
                        U start = j >= 0 ? getStart(d) : getEnd(d);
                        if (areEqual(start, getEnd(c)))
                        {
                            v.Insert(0, i);
                            match = true;
                        }
                        else if (areEqual(start, getStart(c)))
                        {
                            v.Insert(0, -i);
                            match = true;
                        }
                        //}
                    }
                    if (!match && Abs(v[v.Count - 1]) != i)
                    {
                        //Try connecting at end of curve-strip
                        int j = v[v.Count - 1];
                        T d = data[Abs(j)];

                        /*if (removeDuplicates && segmentsEqual(c, d))
                        {
                            match = true;
                        }
                        else
                        {*/
                        U end = j >= 0 ? getEnd(d) : getStart(d);
                        if (areEqual(end, getStart(c)))
                        {
                            v.Add(i);
                            match = true;
                        }
                        else if (areEqual(end, getEnd(c)))
                        {
                            v.Add(-i);
                            match = true;
                        }
                        //}
                    }

                    if (match)
                    {
                        success = true;
                        done[i] = true;
                        ++counter;
                    }
                }
                if (!success)
                {
                    v = new List<int>();
                    list.Add(v);
                    for (int i = 0; i < data.Count; ++i)
                    {
                        if (!done[i])
                        {
                            v.Add(i);
                            done[i] = true;
                            ++counter;
                            break;
                        }
                    }
                }
            }
        }


        public static List<List<U>> ConnectAndResolve<T, U>(IList<T> data, Func<T, U> getStart, Func<T, U> getEnd, bool removeDuplicates = false) where U : IComparable<U>
        {
            List<bool> closed;
            return Resolve(data, getStart, getEnd, Connect(data, getStart, getEnd, delegate (U a, U b) { return a.CompareTo(b) == 0; }, out closed, removeDuplicates), closed);
        }

        public static List<List<U>> ConnectAndResolve<T, U>(IList<T> data, Func<T, U> getStart, Func<T, U> getEnd, out List<bool> closed, bool removeDuplicates = false) where U : IComparable<U>
        {
            return Resolve(data, getStart, getEnd, Connect(data, getStart, getEnd, delegate (U a, U b) { return a.CompareTo(b) == 0; }, out closed, removeDuplicates), closed);
        }

        public static List<List<U>> ConnectAndResolve<T, U>(IList<T> data, Func<T, U> getStart, Func<T, U> getEnd, Func<U, U, bool> areEqual, bool removeDuplicates = false)
        {
            List<bool> closed;
            return Resolve(data, getStart, getEnd, Connect(data, getStart, getEnd, areEqual, out closed, removeDuplicates), closed);
        }


        public static List<List<U>> ConnectAndResolve<T, U>(IList<T> data, Func<T, U> getStart, Func<T, U> getEnd, Func<U, U, bool> areEqual, out List<bool> closed, bool removeDuplicates = false)
        {
            return Resolve(data, getStart, getEnd, Connect(data, getStart, getEnd, areEqual, out closed, removeDuplicates), closed);
        }


        public static List<List<U>> Resolve<T, U>(IList<T> data, Func<T, U> getStart, Func<T, U> getEnd, List<List<int>> indices, List<bool> closed)
        {
            List<List<U>> result = new List<List<U>>();
            for (int i = 0; i < indices.Count; ++i)
            {
                var list = indices[i];
                List<U> r = new List<U>(list.Count + 1);

                int id = list[0];

                if (!closed[i])
                {
                    r.Add(GetStart(data, id, getStart, getEnd));
                    r.Add(GetEnd(data, id, getStart, getEnd));
                }
                else
                {
                    r.Add(GetEnd(data, id, getStart, getEnd));
                }


                for (int j = 1; j < list.Count; ++j)
                {
                    id = list[j];
                    r.Add(GetEnd(data, id, getStart, getEnd));
                }
                result.Add(r);
            }
            return result;
        }


        public static List<List<U>> Resolve<U>(List<List<U>> data, List<List<int>> indices, List<bool> closed)
        {
            List<List<U>> result = new List<List<U>>();
            for (int i = 0; i < indices.Count; ++i)
            {
                var list = indices[i];
                List<U> r = new List<U>(list.Count + 1);

                int id = list[0];

                if (!closed[i])
                {
                    if (id < 0)
                    {
                        //r.Add(getEnd(data[-id]));
                        //r.Add(getStart(data[-id]));
                        var d = data[-id];
                        for (int j = d.Count - 1; j >= 0; --j)
                            r.Add(d[j]);
                    }
                    else
                    {
                        //r.Add(getStart(data[id]));
                        //r.Add(getEnd(data[id]));
                        var d = data[id];
                        for (int j = 0; j < d.Count; ++j)
                            r.Add(d[j]);
                    }
                }
                else
                {
                    if (id < 0)
                    {
                        //r.Add(getEnd(data[-id]));

                        var d = data[-id];
                        for (int j = d.Count - 1; j > 0; --j)
                            r.Add(d[j]);
                    }
                    else
                    {
                        //r.Add(getStart(data[id]));

                        var d = data[id];
                        for (int j = 0; j < d.Count - 1; ++j)
                            r.Add(d[j]);
                    }
                }


                for (int j = 1; j < list.Count; ++j)
                {
                    id = list[j];
                    if (id < 0)
                    {
                        //r.Add(getStart(data[-id]));
                        var d = data[-id];
                        for (int k = d.Count - 2; k >= 0; --k)
                            r.Add(d[k]);
                    }
                    else
                    {
                        //r.Add(getEnd(data[id]));
                        var d = data[id];
                        for (int k = 1; k < d.Count; ++k)
                            r.Add(d[k]);
                    }
                }
                result.Add(r);
            }
            return result;
        }

        //public static List<List<int>> Connect(IList<Int2> segments)
        //{
        //    if (segments.Count == 0)
        //        return new List<List<int>>();

        //    List<List<int>> buffer = new List<List<int>>();
        //    if (segments.Count == 0)
        //        return buffer;

        //    List<int> v = new List<int>();

        //    Int2 s = segments[0];


        //    v.Add(s.X);
        //    v.Add(s.Y);
        //    bool[] done = new bool[segments.Count];
        //    for (int i = 0; i < done.Length; ++i)
        //        done[i] = false;
        //    done[0] = true;

        //    int l = segments.Count;
        //    int counter = 0;
        //    while (/*counter < l*/true)
        //    {
        //        bool success = false;

        //        for (int j = 0; j < l; ++j)
        //        {
        //            s = segments[j];

        //            if (done[j])
        //                continue;

        //            //double d = -1;
        //            int res = AreConnected(s.X, s.Y, v[0], v[1]);
        //            if (res == 0)
        //            {
        //                v.Insert(0, s.Y);
        //                ++counter;
        //                done[j] = true;
        //                success = true;
        //                continue;
        //                //++a;
        //            }
        //            else if (res == 1)
        //            {
        //                v.Insert(0, s.X);
        //                ++counter;
        //                done[j] = true;
        //                success = true;
        //                continue;
        //                //++a;
        //            }


        //            res = AreConnected(s.X, s.Y, v[v.Count - 1], v[v.Count - 2]);
        //            if (res == 0)
        //            {
        //                v.Add(s.Y);
        //                ++counter;
        //                done[j] = true;
        //                success = true;
        //                continue;
        //                //++a;
        //            }
        //            else if (res == 1)
        //            {
        //                v.Add(s.X);
        //                ++counter;
        //                done[j] = true;
        //                success = true;
        //                continue;
        //                //++a;
        //            }

        //        }

        //        if (!success)
        //        {
        //            buffer.Add(v);
        //            v = new List<int>();

        //            for (int i = 0; i < segments.Count; ++i)
        //            {
        //                if (!done[i]/* != null*/)
        //                {
        //                    v.Add(segments[i].X);
        //                    v.Add(segments[i].Y);
        //                    done[i] = true;
        //                    break;
        //                }
        //            }
        //            if (v.Count == 0)
        //            {
        //                //Remove duplicates and connect again
        //                //RemoveDuplicates(buffer, eps, comparer);

        //                return buffer; // Connect(buffer, largeTolerance, comparer);
        //            }
        //        }
        //    }

        //    //return v;
        //}

    }
}
