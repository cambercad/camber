namespace GeoSolver.Sparse
{
    /// <summary>
    /// Quotient-graph minimum degree (George–Liu) on A+A'. C-style linked
    /// adjacency, degree buckets, supervariables, aggressive absorption.
    /// </summary>
    public static class AmdOrdering
    {
        public static int[] Compute(CscMatrix a)
        {
            if (a == null)
                throw new ArgumentNullException(nameof(a));
            if (a.RowCount != a.ColumnCount)
                throw new ArgumentException("AMD requires a square matrix pattern.");
            int n = a.ColumnCount;
            int[] perm = n == 0 ? Array.Empty<int>() : new int[n];
            if (n > 0)
                new Work().Order(n, a.ColumnPointers, a.RowIndices, perm);
            return perm;
        }

        internal sealed class Work
        {
            private int[] _head;
            private int[] _to;
            private int[] _next;
            private int _ncell;
            private int _free;

            private int[] _state; // 0 live var, 1 element, 2 dead
            private int[] _weight;
            private int[] _degree;
            private int[] _memberHead;
            private int[] _memberNext;
            private int[] _eltHead;
            private int[] _evTo;
            private int[] _evNext;
            private int _evN;
            private int[] _degHead;
            private int[] _degNext;
            private int[] _degPrev;
            private int[] _mark;
            private int[] _clique;
            private int[] _oldElt;
            private int[] _hash;
            private int[] _order;
            private int[] _csrAp;
            private int[] _csrAi;
            private int[] _buf;
            private int _stamp;
            private int _minDeg;
            private int _n;

            public void Order(int n, int[] ap, int[] ai, int[] perm)
            {
                Ensure(n, ap[n] * 4 + n * 8 + 32);
                _n = n;
                _stamp = 1;
                BuildGraph(n, ap, ai);

                for (int i = 0; i <= n; i++)
                    _degHead[i] = -1;
                _minDeg = n;
                for (int i = 0; i < n; i++)
                {
                    _state[i] = 0;
                    _weight[i] = 1;
                    _memberHead[i] = i;
                    _memberNext[i] = -1;
                    _eltHead[i] = -1;
                    int d = 0;
                    for (int e = _head[i]; e != -1; e = _next[e])
                        d++;
                    _degree[i] = d;
                    BucketLink(i, d);
                    if (d < _minDeg)
                        _minDeg = d;
                }

                int nextPerm = 0;
                while (nextPerm < n)
                {
                    int p = ExtractMin();
                    if (p < 0)
                    {
                        p = -1;
                        for (int i = 0; i < n; i++)
                        {
                            if (_state[i] == 0)
                            {
                                p = i;
                                break;
                            }
                        }
                        if (p < 0)
                            throw new InvalidOperationException("AMD exhausted the graph before permuting all nodes.");
                    }

                    _stamp = Buf.NextStamp(_mark, n, _stamp);
                    _mark[p] = _stamp;
                    int nClique = 0;
                    int nOld = 0;
                    for (int e = _head[p]; e != -1; e = _next[e])
                    {
                        int v = _to[e];
                        if (v == p || _mark[v] == _stamp)
                            continue;
                        if (_state[v] == 1)
                        {
                            _oldElt[nOld++] = v;
                            for (int c = _eltHead[v]; c != -1; c = _evNext[c])
                            {
                                int u = _evTo[c];
                                if (_state[u] == 0 && _mark[u] != _stamp)
                                {
                                    _mark[u] = _stamp;
                                    _clique[nClique++] = u;
                                }
                            }
                        }
                        else if (_state[v] == 0)
                        {
                            _mark[v] = _stamp;
                            _clique[nClique++] = v;
                        }
                    }

                    int mem = _memberHead[p];
                    while (mem != -1)
                    {
                        perm[nextPerm++] = mem;
                        mem = _memberNext[mem];
                    }
                    BucketUnlink(p);
                    _state[p] = 2;

                    _stamp = Buf.NextStamp(_mark, n, _stamp);
                    for (int t = 0; t < nClique; t++)
                        _mark[_clique[t]] = _stamp;
                    _mark[p] = _stamp;
                    for (int t = 0; t < nOld; t++)
                    {
                        int el = _oldElt[t];
                        if (_state[el] != 1)
                            continue;
                        bool subset = true;
                        for (int c = _eltHead[el]; c != -1; c = _evNext[c])
                        {
                            int u = _evTo[c];
                            if (_state[u] == 0 && _mark[u] != _stamp)
                            {
                                subset = false;
                                break;
                            }
                        }
                        if (subset)
                            _state[el] = 2;
                    }

                    _eltHead[p] = -1;
                    if (nClique > 0)
                    {
                        _state[p] = 1;
                        int eh = -1;
                        for (int t = 0; t < nClique; t++)
                            eh = PushElt(eh, _clique[t]);
                        _eltHead[p] = eh;
                    }

                    int cliqueStamp = Buf.NextStamp(_mark, n, _stamp);
                    _stamp = cliqueStamp;
                    for (int t = 0; t < nClique; t++)
                        _mark[_clique[t]] = cliqueStamp;

                    for (int t = 0; t < nClique; t++)
                    {
                        int i = _clique[t];
                        CompactAdj(i, p, cliqueStamp);
                        AddEdge(i, p);
                    }

                    MergeSupervars(nClique);
                    for (int t = 0; t < nClique; t++)
                    {
                        int i = _clique[t];
                        if (_state[i] != 0)
                            continue;
                        int d = ExternalDegree(i);
                        BucketUnlink(i);
                        _degree[i] = d;
                        BucketLink(i, d);
                        if (d < _minDeg)
                            _minDeg = d;
                    }
                }
            }

            private void Ensure(int n, int cellCap)
            {
                Buf.I(ref _head, n);
                Buf.I(ref _state, n);
                Buf.I(ref _weight, n);
                Buf.I(ref _degree, n);
                Buf.I(ref _memberHead, n);
                Buf.I(ref _memberNext, n);
                Buf.I(ref _eltHead, n);
                Buf.I(ref _evTo, cellCap);
                Buf.I(ref _evNext, cellCap);
                _evN = 0;
                Buf.I(ref _degHead, n + 1);
                Buf.I(ref _degNext, n);
                Buf.I(ref _degPrev, n);
                Buf.I(ref _mark, n);
                Buf.I(ref _clique, n);
                Buf.I(ref _oldElt, n);
                Buf.I(ref _hash, n);
                Buf.I(ref _order, n);
                Buf.I(ref _csrAp, n + 1);
                Buf.I(ref _to, cellCap);
                Buf.I(ref _next, cellCap);
            }

            private void BuildGraph(int n, int[] ap, int[] ai)
            {
                Buf.Zero(_csrAp, n + 1);
                int nnz = ap[n];
                for (int j = 0; j < n; j++)
                {
                    int end = ap[j + 1];
                    for (int p = ap[j]; p < end; p++)
                    {
                        int i = ai[p];
                        if (i == j || (uint)i >= (uint)n)
                            continue;
                        _csrAp[i + 1]++;
                        _csrAp[j + 1]++;
                    }
                }
                for (int i = 0; i < n; i++)
                    _csrAp[i + 1] += _csrAp[i];
                int cap = _csrAp[n];
                Buf.I(ref _csrAi, cap == 0 ? 1 : cap);
                Buf.I(ref _buf, n);
                Array.Copy(_csrAp, _buf, n);
                for (int j = 0; j < n; j++)
                {
                    int end = ap[j + 1];
                    for (int p = ap[j]; p < end; p++)
                    {
                        int i = ai[p];
                        if (i == j || (uint)i >= (uint)n)
                            continue;
                        _csrAi[_buf[i]++] = j;
                        _csrAi[_buf[j]++] = i;
                    }
                }

                _ncell = 0;
                _free = -1;
                for (int i = 0; i < n; i++)
                    _head[i] = -1;

                for (int i = 0; i < n; i++)
                {
                    int lo = _csrAp[i];
                    int hi = Buf.SortUnique(_csrAi, lo, _csrAp[i + 1]);
                    for (int p = lo; p < hi; p++)
                    {
                        int j = _csrAi[p];
                        if (j != i)
                            AddEdge(i, j);
                    }
                }
            }

            private int PushElt(int head, int varId)
            {
                if (_evN >= _evTo.Length)
                {
                    int cap = Buf.GrowCap(_evTo.Length, _evN + 1);
                    Array.Resize(ref _evTo, cap);
                    Array.Resize(ref _evNext, cap);
                }
                int c = _evN++;
                _evTo[c] = varId;
                _evNext[c] = head;
                return c;
            }

            private int NewCell(int to)
            {
                int e;
                if (_free >= 0)
                {
                    e = _free;
                    _free = _next[e];
                }
                else
                {
                    if (_ncell >= _to.Length)
                    {
                        int cap = Buf.GrowCap(_to.Length, _ncell + 1);
                        Array.Resize(ref _to, cap);
                        Array.Resize(ref _next, cap);
                    }
                    e = _ncell++;
                }
                _to[e] = to;
                return e;
            }

            private void FreeCell(int e)
            {
                _next[e] = _free;
                _free = e;
            }

            private void AddEdge(int from, int to)
            {
                int e = NewCell(to);
                _next[e] = _head[from];
                _head[from] = e;
            }

            private void CompactAdj(int i, int eliminated, int cliqueStamp)
            {
                int prev = -1;
                int e = _head[i];
                while (e != -1)
                {
                    int nxt = _next[e];
                    int v = _to[e];
                    bool drop = v == i || v == eliminated;
                    if (!drop)
                    {
                        if (_state[v] == 2)
                            drop = true;
                        else if (_state[v] == 0 && _mark[v] == cliqueStamp)
                            drop = true;
                    }
                    if (drop)
                    {
                        if (prev < 0)
                            _head[i] = nxt;
                        else
                            _next[prev] = nxt;
                        FreeCell(e);
                    }
                    else
                        prev = e;
                    e = nxt;
                }
            }

            private void MergeSupervars(int nClique)
            {
                if (nClique < 2)
                    return;
                for (int t = 0; t < nClique; t++)
                {
                    int i = _clique[t];
                    _order[t] = t;
                    int h = 0;
                    int len = 0;
                    for (int e = _head[i]; e != -1; e = _next[e])
                    {
                        len++;
                        h = unchecked(h + (_to[e] + 1) * -1640531535);
                    }
                    _hash[t] = h + len * 97;
                }
                Array.Sort(_hash, _order, 0, nClique);

                int start = 0;
                while (start < nClique)
                {
                    int h = _hash[start];
                    int end = start + 1;
                    while (end < nClique && _hash[end] == h)
                        end++;
                    for (int u = start; u < end; u++)
                    {
                        int iu = _clique[_order[u]];
                        if (_state[iu] != 0)
                            continue;
                        for (int v = u + 1; v < end; v++)
                        {
                            int iv = _clique[_order[v]];
                            if (_state[iv] != 0)
                                continue;
                            if (!SameAdj(iu, iv))
                                continue;
                            _weight[iu] += _weight[iv];
                            int tail = _memberHead[iu];
                            while (_memberNext[tail] != -1)
                                tail = _memberNext[tail];
                            _memberNext[tail] = _memberHead[iv];
                            BucketUnlink(iv);
                            _state[iv] = 2;
                        }
                    }
                    start = end;
                }
            }

            private bool SameAdj(int a, int b)
            {
                _stamp = Buf.NextStamp(_mark, _n, _stamp);
                int na = 0;
                for (int e = _head[a]; e != -1; e = _next[e])
                {
                    _mark[_to[e]] = _stamp;
                    na++;
                }
                int nb = 0;
                for (int e = _head[b]; e != -1; e = _next[e])
                {
                    if (_mark[_to[e]] != _stamp)
                        return false;
                    nb++;
                }
                return na == nb;
            }

            private int ExternalDegree(int i)
            {
                _stamp = Buf.NextStamp(_mark, _n, _stamp);
                int d = 0;
                for (int e = _head[i]; e != -1; e = _next[e])
                {
                    int v = _to[e];
                    if (v == i)
                        continue;
                    if (_state[v] == 1)
                    {
                        for (int c = _eltHead[v]; c != -1; c = _evNext[c])
                        {
                            int u = _evTo[c];
                            if (u != i && _state[u] == 0 && _mark[u] != _stamp)
                            {
                                _mark[u] = _stamp;
                                d += _weight[u];
                            }
                        }
                    }
                    else if (_state[v] == 0 && _mark[v] != _stamp)
                    {
                        _mark[v] = _stamp;
                        d += _weight[v];
                    }
                }
                int cap = _n - _weight[i];
                if (d > cap)
                    d = cap;
                if (d < 0)
                    d = 0;
                return d;
            }

            private void BucketLink(int i, int d)
            {
                if (d < 0)
                    d = 0;
                if (d > _n)
                    d = _n;
                _degree[i] = d;
                int h = _degHead[d];
                _degNext[i] = h;
                _degPrev[i] = -1;
                if (h >= 0)
                    _degPrev[h] = i;
                _degHead[d] = i;
            }

            private void BucketUnlink(int i)
            {
                int d = _degree[i];
                if (d < 0)
                    return;
                int nxt = _degNext[i];
                int prv = _degPrev[i];
                if (prv >= 0)
                    _degNext[prv] = nxt;
                else if (d <= _n && _degHead[d] == i)
                    _degHead[d] = nxt;
                if (nxt >= 0)
                    _degPrev[nxt] = prv;
                _degree[i] = -1;
            }

            private int ExtractMin()
            {
                int n = _n;
                while (_minDeg <= n && _degHead[_minDeg] < 0)
                    _minDeg++;
                if (_minDeg > n)
                    return -1;
                while (_minDeg <= n)
                {
                    int i = _degHead[_minDeg];
                    while (i >= 0)
                    {
                        int nxt = _degNext[i];
                        if (_state[i] == 0)
                        {
                            BucketUnlink(i);
                            return i;
                        }
                        BucketUnlink(i);
                        i = nxt;
                    }
                    _minDeg++;
                }
                return -1;
            }
        }
    }
}
