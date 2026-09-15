namespace GeoCore
{
    public struct IdWithDir<T>
    {
        public T Segment;
        public int Index;
        public bool ReversedDir;
    }

    //Requires good GetHasCode and Equals implementation on template type U
    public static class HashSegmentConnector
    {
        private class ListEx<U> : List<U>
        {
            public bool Processed = false;
            public U Point;

            public ListEx(U point) { Point = point; }
        }

        public static List<List<U>> ConnectAndResolve<T, U>(IList<T> segments, Func<T, U> getStart, Func<T, U> getEnd, out List<bool> closed)
        {
            if (segments.Count == 0)
                throw new Exception();
            if (segments.Count == 1)
            {
                closed = new List<bool>() { false };
                U start = getStart(segments[0]);
                U end = getEnd(segments[0]);
                if (start.Equals(end))
                    throw new Exception();
                return new List<List<U>>() { new List<U>() { start, end } };
            }

            Dictionary<U, ListEx<U>> hashLinkMap = new Dictionary<U, ListEx<U>>((int)(segments.Count * 1.1));
            for (int i = 0; i < segments.Count; ++i)
            {
                var s = segments[i];
                if (getStart(s).Equals(getEnd(s)))
                    continue;

                U hashStart = getStart(s);
                U hashEnd = getEnd(s);

                ListEx<U> list;
                if (hashLinkMap.TryGetValue(hashStart, out list))
                {
                    bool isDuplicate = false;
                    for (int j = 0; j < list.Count; ++j)
                        if (list[j].Equals(hashEnd))
                        {
                            isDuplicate = true;
                            break;
                        }
                    if (!isDuplicate)
                        list.Add(hashEnd/*, s.End*/);
                }
                else
                    hashLinkMap.Add(hashStart, new ListEx<U>(getStart(s)) { hashEnd/*, s.End*/ });


                if (hashLinkMap.TryGetValue(hashEnd, out list))
                {
                    bool isDuplicate = false;
                    for (int j = 0; j < list.Count; ++j)
                        if (list[j].Equals(hashStart))
                        {
                            isDuplicate = true;
                            break;
                        }
                    if (!isDuplicate)
                        list.Add(hashStart/*, s.Start*/);
                }
                else
                    hashLinkMap.Add(hashEnd, new ListEx<U>(getEnd(s)) { hashStart/*, s.Start*/ });
            }

            closed = new List<bool>();
            List<List<U>> strips = new List<List<U>>();
            foreach (var v in hashLinkMap)
            {
                if (!v.Value.Processed)
                {
                    bool c;
                    strips.Add(Process(v.Key, v.Value, hashLinkMap, out c));
                    closed.Add(c);
                }
            }
            return strips;
        }

        private static List<U> Process<U>(U startHash, ListEx<U> baseList, Dictionary<U, ListEx<U>> hashLinkMap, out bool closed)
        {
            List<U> stripA = new List<U>();
            closed = false;

            stripA.Add(baseList.Point);
            baseList.Processed = true;
            if (baseList.Count > 0)
            {
                Stack<U> stack = new Stack<U>();
                stack.Push(baseList[0]);
                U lastValid = default(U);
                while (stack.Count > 0)
                {
                    var target = stack.Pop();
                    var list = hashLinkMap[target];
                    if (!list.Processed)
                    {
                        lastValid = target;

                        list.Processed = true;
                        stripA.Add(list.Point);

                        for (int i = 0; i < list.Count; ++i)
                            stack.Push(list[i]);
                    }
                }

                if (stripA.Count > 2)
                {
                    var lastList = hashLinkMap[lastValid];
                    for (int i = 0; i < lastList.Count; ++i)
                        if (lastList[i].Equals(startHash))
                        {
                            closed = true;
                            break;
                        }
                }
            }


            List<U> stripB = new List<U>();
            if (baseList.Count > 1)
            {
                Stack<U> stack = new Stack<U>();
                stack.Push(baseList[1]);
                while (stack.Count > 0)
                {
                    var target = stack.Pop();
                    var list = hashLinkMap[target];
                    if (!list.Processed)
                    {
                        list.Processed = true;
                        stripB.Add(list.Point);

                        for (int i = 0; i < list.Count; ++i)
                            stack.Push(list[i]);
                    }
                }
            }

            //stripA.Reverse();
            int l = stripA.Count >> 1;
            int upper = stripA.Count - 1;
            for (int i = 0; i < l; ++i)
            {
                U tmp = stripA[i];
                stripA[i] = stripA[upper - i];
                stripA[upper - i] = tmp;
            }

            for (int i = 0; i < stripB.Count; ++i)
                stripA.Add(stripB[i]);


            return stripA;
        }

    }
}