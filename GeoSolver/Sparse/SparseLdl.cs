namespace GeoSolver.Sparse
{
    /// <summary>
    /// Reusable supernodal LDLᵀ. Analyze the pattern once, then NumericFactor
    /// into the same buffers while the sparsity stays fixed (Newton loop).
    /// </summary>
    public sealed class SparseLdl
    {
        public const double PivotEpsilon = 1e-30;

        private readonly AmdOrdering.Work _amd = new AmdOrdering.Work();

        private int _n;
        private int _srcNnz;
        private int[] _srcCp;
        private int[] _srcRi;

        private int[] _perm;
        private int[] _iperm;
        private int[] _amdPerm;
        private int[] _post;
        private int[] _parent;
        private int[] _ancestor;
        private int[] _childHead;
        private int[] _childNext;
        private int[] _stackNode;
        private int[] _stackChild;

        private int[] _pCp;
        private int[] _pRi;
        private double[] _pVx;
        private int _pNnz;

        private int[] _lCp;
        private int[] _lRi;
        private int[] _spa;
        private int[] _mark;
        private int _stamp;

        private int _nsuper;
        private int[] _superCol;
        private int[] _width;
        private int[] _height;
        private int[] _rowPtr;
        private int[] _rows;
        private int[] _valPtr;
        private int[] _col2super;
        private int[] _cPtr;
        private int[] _cIdx;

        private double[] _d;
        private double[] _lx;
        private double[] _work;
        private double[] _panel;
        private int[] _map;
        private int[] _ovLoc;
        private int[] _ovRel;

        public int Dimension => _n;
        public int SuperNodeCount => _nsuper;
        public int NonZerosL => _valPtr == null ? 0 : _valPtr[_nsuper];

        public static bool TryFactor(CscMatrix a, out SparseLdl factor)
        {
            factor = new SparseLdl();
            if (!factor.Factorize(a))
            {
                factor = null;
                return false;
            }
            return true;
        }

        public static SparseLdl Factor(CscMatrix a)
        {
            var f = new SparseLdl();
            if (!f.Factorize(a))
                throw new InvalidOperationException("Zero or tiny pivot in sparse LDL.");
            return f;
        }

        public static bool TrySolve(CscMatrix a, double[] b, double[] x)
        {
            var f = new SparseLdl();
            if (!f.Factorize(a))
                return false;
            f.Solve(b, x);
            return true;
        }

        /// <summary>Analyze if the pattern changed, then numeric factor.</summary>
        public bool Factorize(CscMatrix a)
        {
            if (a == null || a.RowCount != a.ColumnCount)
                return false;
            if (!SamePattern(a) && !Analyze(a))
                return false;
            return NumericFactor(a);
        }

        public bool Analyze(CscMatrix a)
        {
            if (a == null || a.RowCount != a.ColumnCount)
                return false;
            int n = a.ColumnCount;
            int[] ap = a.ColumnPointers;
            int[] ai = a.RowIndices;
            int nnz = ap[n];
            _n = n;
            _srcNnz = nnz;
            Buf.I(ref _srcCp, n + 1);
            Buf.I(ref _srcRi, Math.Max(nnz, 1));
            Array.Copy(ap, _srcCp, n + 1);
            Array.Copy(ai, _srcRi, nnz);
            if (n == 0)
            {
                _nsuper = 0;
                Buf.I(ref _superCol, 1);
                _superCol[0] = 0;
                Buf.I(ref _valPtr, 1);
                _valPtr[0] = 0;
                Buf.D(ref _work, 1);
                return true;
            }

            Buf.I(ref _amdPerm, n);
            Buf.I(ref _perm, n);
            Buf.I(ref _iperm, n);
            Buf.I(ref _parent, n);
            Buf.I(ref _ancestor, n);
            Buf.I(ref _post, n);
            Buf.I(ref _childHead, n);
            Buf.I(ref _childNext, n);
            Buf.I(ref _stackNode, n);
            Buf.I(ref _stackChild, n);
            _amd.Order(n, ap, ai, _amdPerm);
            EtreeMapped(n, ap, ai, _amdPerm, _iperm, _parent, _ancestor);
            Postorder(_parent, n, _post, _childHead, _childNext, _stackNode, _stackChild);
            for (int k = 0; k < n; k++)
                _perm[k] = _amdPerm[_post[k]];
            for (int k = 0; k < n; k++)
                _iperm[_perm[k]] = k;

            PermutePattern(n, ap, ai, a.Values, _perm, _iperm, true);
            Etree(_n, _pCp, _pRi, _parent, _ancestor);
            SymbolicL(n, _pCp, _pRi, _parent);
            BuildSupernodes(n);
            BuildContributors();

            Buf.D(ref _d, n);
            Buf.D(ref _work, n);
            Buf.I(ref _map, n);
            Buf.Fill(_map, n, -1);
            Buf.I(ref _ovLoc, n);
            Buf.I(ref _ovRel, n);

            int maxPanel = 1;
            for (int s = 0; s < _nsuper; s++)
            {
                int sz = _height[s] * _width[s];
                if (sz > maxPanel)
                    maxPanel = sz;
            }
            Buf.D(ref _panel, maxPanel);
            Buf.D(ref _lx, _valPtr[_nsuper] == 0 ? 1 : _valPtr[_nsuper]);
            return true;
        }

        public bool NumericFactor(CscMatrix a)
        {
            int n = _n;
            if (n == 0)
                return true;
            if (a.ColumnCount != n)
                return false;

            PermuteValues(a.ColumnPointers, a.RowIndices, a.Values);

            double[] panel = _panel;
            double[] lx = _lx;
            double[] d = _d;
            int[] map = _map;
            int[] rows = _rows;
            int[] ap = _pCp;
            int[] ai = _pRi;
            double[] ax = _pVx;

            for (int s = 0; s < _nsuper; s++)
            {
                int m = _height[s];
                int w = _width[s];
                int c0 = _superCol[s];
                int rp = _rowPtr[s];
                int psz = m * w;
                Array.Clear(panel, 0, psz);
                for (int r = 0; r < m; r++)
                    map[rows[rp + r]] = r;

                for (int j = 0; j < w; j++)
                {
                    int col = c0 + j;
                    int end = ap[col + 1];
                    for (int p = ap[col]; p < end; p++)
                    {
                        int rel = map[ai[p]];
                        if (rel >= 0)
                            panel[rel + j * m] += ax[p];
                    }
                }
                int c0i = _cPtr[s];
                int c1i = _cPtr[s + 1];
                for (int t = c0i; t < c1i; t++)
                    ApplyUpdate(_cIdx[t], panel, m, w);

                if (!FactorPanel(panel, m, w, d, c0))
                    return false;

                Array.Copy(panel, 0, lx, _valPtr[s], psz);
                for (int r = 0; r < m; r++)
                    map[rows[rp + r]] = -1;
            }
            return true;
        }

        public void Solve(double[] b, double[] x)
        {
            int n = _n;
            double[] y = _work;
            int[] perm = _perm;
            for (int i = 0; i < n; i++)
                y[i] = b[perm[i]];

            for (int s = 0; s < _nsuper; s++)
                ForwardSuper(s, y);

            double[] d = _d;
            for (int i = 0; i < n; i++)
            {
                double di = d[i];
                if (Math.Abs(di) < PivotEpsilon)
                    throw new InvalidOperationException("Zero diagonal in LDL solve.");
                y[i] /= di;
            }

            for (int s = _nsuper - 1; s >= 0; s--)
                BackwardSuper(s, y);

            int[] iperm = _iperm;
            for (int i = 0; i < n; i++)
                x[i] = y[iperm[i]];
        }

        private bool SamePattern(CscMatrix a)
        {
            int n = a.ColumnCount;
            if (n != _n || a.ColumnPointers[n] != _srcNnz)
                return false;
            int[] cp = a.ColumnPointers;
            int[] ri = a.RowIndices;
            int[] scp = _srcCp;
            int[] sri = _srcRi;
            if (scp == null || sri == null)
                return false;
            for (int i = 0; i <= n; i++)
            {
                if (cp[i] != scp[i])
                    return false;
            }
            int nnz = _srcNnz;
            for (int p = 0; p < nnz; p++)
            {
                if (ri[p] != sri[p])
                    return false;
            }
            return true;
        }

        private void PermutePattern(int n, int[] ap, int[] ai, double[] ax, int[] perm, int[] iperm, bool withValues)
        {
            int nnz = ap[n];
            Buf.I(ref _pCp, n + 1);
            Buf.I(ref _pRi, nnz == 0 ? 1 : nnz);
            Buf.D(ref _pVx, nnz == 0 ? 1 : nnz);
            int w = 0;
            _pCp[0] = 0;
            for (int j = 0; j < n; j++)
            {
                int oj = perm[j];
                int end = ap[oj + 1];
                for (int p = ap[oj]; p < end; p++)
                {
                    int oi = ai[p];
                    _pRi[w] = (uint)oi < (uint)n ? iperm[oi] : oi;
                    _pVx[w] = withValues ? ax[p] : 0.0;
                    w++;
                }
                w = Buf.SortSumUnique(_pRi, _pVx, _pCp[j], w);
                _pCp[j + 1] = w;
            }
            _pNnz = w;
        }

        private void PermuteValues(int[] ap, int[] ai, double[] ax)
        {
            int n = _n;
            int[] perm = _perm;
            int[] iperm = _iperm;
            int[] map = _map;
            Array.Clear(_pVx, 0, _pNnz);
            for (int j = 0; j < n; j++)
            {
                int lo = _pCp[j];
                int hi = _pCp[j + 1];
                for (int p = lo; p < hi; p++)
                    map[_pRi[p]] = p;
                int oj = perm[j];
                int end = ap[oj + 1];
                for (int p = ap[oj]; p < end; p++)
                {
                    int oi = ai[p];
                    if ((uint)oi >= (uint)n)
                        continue;
                    int dest = map[iperm[oi]];
                    if (dest >= 0)
                        _pVx[dest] += ax[p];
                }
                for (int p = lo; p < hi; p++)
                    map[_pRi[p]] = -1;
            }
        }

        private static void EtreeMapped(int n, int[] ap, int[] ai, int[] amd, int[] iperm, int[] parent, int[] ancestor)
        {
            for (int i = 0; i < n; i++)
                iperm[amd[i]] = i;
            for (int i = 0; i < n; i++)
            {
                parent[i] = -1;
                ancestor[i] = -1;
            }
            for (int j = 0; j < n; j++)
            {
                int oj = amd[j];
                int end = ap[oj + 1];
                for (int p = ap[oj]; p < end; p++)
                {
                    int oi = ai[p];
                    if ((uint)oi >= (uint)n)
                        continue;
                    int i = iperm[oi];
                    if (i >= j)
                        continue;
                    for (int t = i; t != -1 && t < j; )
                    {
                        int nxt = ancestor[t];
                        ancestor[t] = j;
                        if (nxt == -1)
                        {
                            parent[t] = j;
                            break;
                        }
                        t = nxt;
                    }
                }
            }
        }

        private static void Etree(int n, int[] ap, int[] ai, int[] parent, int[] ancestor)
        {
            for (int i = 0; i < n; i++)
            {
                parent[i] = -1;
                ancestor[i] = -1;
            }
            for (int j = 0; j < n; j++)
            {
                int end = ap[j + 1];
                for (int p = ap[j]; p < end; p++)
                {
                    int i = ai[p];
                    if (i >= j)
                        continue;
                    for (int t = i; t != -1 && t < j; )
                    {
                        int nxt = ancestor[t];
                        ancestor[t] = j;
                        if (nxt == -1)
                        {
                            parent[t] = j;
                            break;
                        }
                        t = nxt;
                    }
                }
            }
        }

        private static void Postorder(int[] parent, int n, int[] post, int[] childHead, int[] childNext, int[] stackNode, int[] stackChild)
        {
            for (int i = 0; i < n; i++)
                childHead[i] = -1;
            for (int i = 0; i < n; i++)
            {
                int p = parent[i];
                if (p < 0)
                    continue;
                childNext[i] = childHead[p];
                childHead[p] = i;
            }
            int k = 0;
            int sp = 0;
            for (int root = 0; root < n; root++)
            {
                if (parent[root] >= 0)
                    continue;
                stackNode[sp] = root;
                stackChild[sp] = childHead[root];
                sp++;
                while (sp > 0)
                {
                    int node = stackNode[sp - 1];
                    int ch = stackChild[sp - 1];
                    if (ch >= 0)
                    {
                        stackChild[sp - 1] = childNext[ch];
                        stackNode[sp] = ch;
                        stackChild[sp] = childHead[ch];
                        sp++;
                    }
                    else
                    {
                        post[k++] = node;
                        sp--;
                    }
                }
            }
        }

        private void SymbolicL(int n, int[] ap, int[] ai, int[] parent)
        {
            for (int i = 0; i < n; i++)
                _childHead[i] = -1;
            for (int i = 0; i < n; i++)
            {
                int p = parent[i];
                if (p < 0)
                    continue;
                _childNext[i] = _childHead[p];
                _childHead[p] = i;
            }

            Buf.I(ref _lCp, n + 1);
            Buf.I(ref _spa, n);
            Buf.I(ref _mark, n);
            _stamp = 1;
            _lCp[0] = 0;
            int nnz = 0;
            Buf.I(ref _lRi, n);

            for (int j = 0; j < n; j++)
            {
                _stamp = Buf.NextStamp(_mark, n, _stamp);
                int colNnz = 0;
                _mark[j] = _stamp;
                _spa[colNnz++] = j;

                int end = ap[j + 1];
                for (int p = ap[j]; p < end; p++)
                {
                    int i = ai[p];
                    if (i > j && _mark[i] != _stamp)
                    {
                        _mark[i] = _stamp;
                        _spa[colNnz++] = i;
                    }
                }
                for (int ch = _childHead[j]; ch >= 0; ch = _childNext[ch])
                {
                    int c0 = _lCp[ch];
                    int c1 = _lCp[ch + 1];
                    for (int p = c0; p < c1; p++)
                    {
                        int i = _lRi[p];
                        if (i > j && _mark[i] != _stamp)
                        {
                            _mark[i] = _stamp;
                            _spa[colNnz++] = i;
                        }
                    }
                }

                int dest = nnz;
                Buf.I(ref _lRi, dest + colNnz);
                Array.Copy(_spa, 0, _lRi, dest, colNnz);
                Array.Sort(_lRi, dest, colNnz);
                nnz += colNnz;
                _lCp[j + 1] = nnz;
            }
        }

        private void BuildSupernodes(int n)
        {
            Buf.I(ref _superCol, n + 1);
            int ns = 0;
            int j = 0;
            while (j < n)
            {
                _superCol[ns++] = j;
                j++;
                while (j < n && parentChain(j) && Nested(j - 1, j))
                    j++;
            }
            _nsuper = ns;
            _superCol[ns] = n;
            Buf.I(ref _width, ns);
            Buf.I(ref _height, ns);
            Buf.I(ref _rowPtr, ns + 1);
            Buf.I(ref _col2super, n);
            int nnzRows = 0;
            _rowPtr[0] = 0;
            for (int s = 0; s < ns; s++)
            {
                int c0 = _superCol[s];
                int c1 = _superCol[s + 1];
                _width[s] = c1 - c0;
                _height[s] = _lCp[c0 + 1] - _lCp[c0];
                nnzRows += _height[s];
                _rowPtr[s + 1] = nnzRows;
                for (int c = c0; c < c1; c++)
                    _col2super[c] = s;
            }
            Buf.I(ref _rows, nnzRows == 0 ? 1 : nnzRows);
            Buf.I(ref _valPtr, ns + 1);
            _valPtr[0] = 0;
            for (int s = 0; s < ns; s++)
            {
                Array.Copy(_lRi, _lCp[_superCol[s]], _rows, _rowPtr[s], _height[s]);
                _valPtr[s + 1] = _valPtr[s] + _height[s] * _width[s];
            }

            bool parentChain(int col)
            {
                return _parent[col - 1] == col;
            }
        }

        private bool Nested(int jPrev, int j)
        {
            int lenP = _lCp[jPrev + 1] - _lCp[jPrev];
            int lenC = _lCp[j + 1] - _lCp[j];
            if (lenP != lenC + 1)
                return false;
            int p0 = _lCp[jPrev] + 1;
            int c0 = _lCp[j];
            for (int t = 0; t < lenC; t++)
            {
                if (_lRi[p0 + t] != _lRi[c0 + t])
                    return false;
            }
            return true;
        }

        private void BuildContributors()
        {
            int ns = _nsuper;
            Buf.I(ref _cPtr, ns + 1);
            Buf.Zero(_cPtr, ns + 1);
            for (int d = 0; d < ns; d++)
            {
                int w = _width[d];
                int end = _rowPtr[d + 1];
                int last = -1;
                for (int p = _rowPtr[d] + w; p < end; p++)
                {
                    int s = _col2super[_rows[p]];
                    if (s != last)
                    {
                        _cPtr[s + 1]++;
                        last = s;
                    }
                }
            }
            for (int s = 0; s < ns; s++)
                _cPtr[s + 1] += _cPtr[s];
            int nidx = _cPtr[ns];
            Buf.I(ref _cIdx, nidx == 0 ? 1 : nidx);
            Buf.I(ref _spa, ns);
            Array.Copy(_cPtr, _spa, ns);
            for (int d = 0; d < ns; d++)
            {
                int w = _width[d];
                int end = _rowPtr[d + 1];
                int last = -1;
                for (int p = _rowPtr[d] + w; p < end; p++)
                {
                    int s = _col2super[_rows[p]];
                    if (s == last)
                        continue;
                    last = s;
                    _cIdx[_spa[s]++] = d;
                }
            }
        }

        private void ForwardSuper(int s, double[] y)
        {
            int m = _height[s];
            int w = _width[s];
            int c0 = _superCol[s];
            int rp = _rowPtr[s];
            int vp = _valPtr[s];
            double[] lx = _lx;
            int[] rows = _rows;
            for (int k = 0; k < w; k++)
            {
                double yk = y[c0 + k];
                int colBase = vp + k * m;
                for (int i = k + 1; i < m; i++)
                    y[rows[rp + i]] -= lx[colBase + i] * yk;
            }
        }

        private void BackwardSuper(int s, double[] y)
        {
            int m = _height[s];
            int w = _width[s];
            int c0 = _superCol[s];
            int rp = _rowPtr[s];
            int vp = _valPtr[s];
            double[] lx = _lx;
            int[] rows = _rows;
            for (int k = w - 1; k >= 0; k--)
            {
                double sum = y[c0 + k];
                int colBase = vp + k * m;
                for (int i = k + 1; i < m; i++)
                    sum -= lx[colBase + i] * y[rows[rp + i]];
                y[c0 + k] = sum;
            }
        }

        private unsafe bool FactorPanel(double[] panel, int m, int w, double[] d, int col0)
        {
            fixed (double* p = panel)
            {
                for (int k = 0; k < w; k++)
                {
                    double* colk = p + k * m;
                    double dk = colk[k];
                    if (Math.Abs(dk) < PivotEpsilon || double.IsNaN(dk) || double.IsInfinity(dk))
                        return false;
                    d[col0 + k] = dk;
                    double inv = 1.0 / dk;
                    for (int i = k + 1; i < m; i++)
                        colk[i] *= inv;
                    for (int j = k + 1; j < w; j++)
                    {
                        double t = dk * colk[j];
                        if (t == 0.0)
                            continue;
                        double* colj = p + j * m;
                        for (int i = j; i < m; i++)
                            colj[i] -= colk[i] * t;
                    }
                }
            }
            return true;
        }

        private void ApplyUpdate(int d, double[] panel, int m, int w)
        {
            int md = _height[d];
            int wd = _width[d];
            int rp = _rowPtr[d];
            int vp = _valPtr[d];
            int c0d = _superCol[d];
            int[] map = _map;
            int[] rows = _rows;
            int[] ovLoc = _ovLoc;
            int[] ovRel = _ovRel;
            double[] lx = _lx;
            double[] diag = _d;
            int nov = 0;
            for (int i = 0; i < md; i++)
            {
                int rel = map[rows[rp + i]];
                if (rel < 0)
                    continue;
                ovLoc[nov] = i;
                ovRel[nov] = rel;
                nov++;
            }
            if (nov == 0)
                return;

            for (int k = 0; k < wd; k++)
            {
                double dk = diag[c0d + k];
                if (dk == 0.0)
                    continue;
                int colBase = vp + k * md;
                for (int a = 0; a < nov; a++)
                {
                    int colRel = ovRel[a];
                    if (colRel >= w)
                        continue;
                    double t = dk * lx[colBase + ovLoc[a]];
                    if (t == 0.0)
                        continue;
                    int destCol = colRel * m;
                    for (int b = 0; b < nov; b++)
                        panel[destCol + ovRel[b]] -= lx[colBase + ovLoc[b]] * t;
                }
            }
        }
    }
}
