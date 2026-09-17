namespace GeoSolver.Sparse
{
    /// <summary>
    /// Grow-only Newton linear workspace: Jacobian CSC, Gramian, LDL analyze-once.
    /// </summary>
    internal sealed class SparseNewtonWorkspace
    {
        private readonly SparseLdl _ldl = new SparseLdl();

        private int[] _jCp;
        private int[] _jRi;
        private double[] _jVx;
        private int _jRows;
        private int _jCols;
        private int _jNnz;
        private CscMatrix _j;

        private int[] _atCp;
        private int[] _atRi;
        private double[] _atVx;
        private int[] _atNext;

        private int[] _gCp;
        private int[] _gRi;
        private double[] _gVx;
        private int _gN;
        private int _gNnz;
        private CscMatrix _g;

        private int[] _colNnz;
        private int[] _cursor;
        private int[] _mark;
        private int[] _pat;
        private int[] _gMap;
        private double[] _acc;
        private int _stamp;

        private double[] _vbuf;
        private int[] _cbuf;

        private int[] _lastJCp;
        private int[] _lastJRi;
        private int _lastJRows;
        private int _lastJCols;
        private int _lastJNnz;
        private bool _hasPattern;

        public bool TryStep(
            IEquationContainer equations,
            double[] columnScale,
            double[] b,
            double[] result,
            double[] scratchEq,
            double[] scratchParam)
        {
            int m = equations.NumEquations;
            int n = equations.NumParameters;
            if (!BuildJacobian(equations, columnScale, m, n))
                return false;

            bool under = m < n;
            int gdim = under ? m : n;
            bool patternChanged = !ReuseGramianPattern(gdim);

            if (under)
            {
                if (patternChanged && !FormAAt(true))
                    return false;
                if (!patternChanged && !FormAAt(false))
                    return false;
                WrapG(m);
                AddTraceDamping(m);
                Array.Copy(b, scratchEq, m);
                if (!_ldl.Factorize(_g))
                    return false;
                _ldl.Solve(scratchEq, scratchEq);
                _j.TransposeMultiply(scratchEq, result);
            }
            else
            {
                if (patternChanged && !FormAtA(true))
                    return false;
                if (!patternChanged && !FormAtA(false))
                    return false;
                WrapG(n);
                AddTraceDamping(n);
                _j.TransposeMultiply(b, scratchParam);
                if (!_ldl.Factorize(_g))
                    return false;
                _ldl.Solve(scratchParam, result);
            }

            for (int c = 0; c < n; c++)
                result[c] *= columnScale[c];
            return true;
        }

        private bool BuildJacobian(IEquationContainer equations, double[] columnScale, int m, int n)
        {
            Buf.I(ref _colNnz, n);
            Buf.Zero(_colNnz, n);
            int vcap = _vbuf == null ? Math.Max(8, n) : _vbuf.Length;
            Buf.D(ref _vbuf, vcap);
            Buf.I(ref _cbuf, vcap);

            for (int r = 0; r < m; r++)
            {
                int nnzRow = equations.NumNonZerosInRow(r);
                if (nnzRow + 1 > _vbuf.Length)
                {
                    Buf.D(ref _vbuf, nnzRow + 8);
                    Buf.I(ref _cbuf, nnzRow + 8);
                }
                int idx = 0;
                equations.EvaluateJacobianRow(r, ref idx, _vbuf, _cbuf);
                for (int k = 0; k < idx; k++)
                {
                    int c = _cbuf[k];
                    if ((uint)c < (uint)n)
                        _colNnz[c]++;
                }
            }

            Buf.I(ref _jCp, n + 1);
            _jCp[0] = 0;
            for (int c = 0; c < n; c++)
                _jCp[c + 1] = _jCp[c] + _colNnz[c];
            int nnz = _jCp[n];
            Buf.I(ref _jRi, nnz == 0 ? 1 : nnz);
            Buf.D(ref _jVx, nnz == 0 ? 1 : nnz);
            Buf.I(ref _cursor, n);
            Array.Copy(_jCp, _cursor, n);

            for (int r = 0; r < m; r++)
            {
                int idx = 0;
                equations.EvaluateJacobianRow(r, ref idx, _vbuf, _cbuf);
                for (int k = 0; k < idx; k++)
                {
                    int c = _cbuf[k];
                    if ((uint)c >= (uint)n)
                        continue;
                    int p = _cursor[c]++;
                    _jRi[p] = r;
                    _jVx[p] = _vbuf[k] * columnScale[c];
                }
            }

            for (int c = 0; c < n; c++)
                _cursor[c] = Buf.SortSumUnique(_jRi, _jVx, _jCp[c], _jCp[c + 1]);
            int w = 0;
            for (int c = 0; c < n; c++)
            {
                int lo = _jCp[c];
                int hi = _cursor[c];
                int len = hi - lo;
                if (lo != w)
                {
                    for (int t = 0; t < len; t++)
                    {
                        _jRi[w + t] = _jRi[lo + t];
                        _jVx[w + t] = _jVx[lo + t];
                    }
                }
                _jCp[c] = w;
                w += len;
            }
            _jCp[n] = w;
            _jRows = m;
            _jCols = n;
            _jNnz = w;
            if (_j == null || _j.RowCount != m || _j.ColumnCount != n
                || !ReferenceEquals(_j.ColumnPointers, _jCp)
                || !ReferenceEquals(_j.RowIndices, _jRi)
                || !ReferenceEquals(_j.Values, _jVx))
                _j = new CscMatrix(m, n, _jCp, _jRi, _jVx);
            return true;
        }

        private bool ReuseGramianPattern(int gdim)
        {
            bool same = _hasPattern
                && gdim == _gN
                && _jRows == _lastJRows
                && _jCols == _lastJCols
                && _jNnz == _lastJNnz;
            if (same)
            {
                for (int c = 0; c <= _jCols; c++)
                {
                    if (_jCp[c] != _lastJCp[c])
                    {
                        same = false;
                        break;
                    }
                }
            }
            if (same)
            {
                for (int p = 0; p < _jNnz; p++)
                {
                    if (_jRi[p] != _lastJRi[p])
                    {
                        same = false;
                        break;
                    }
                }
            }
            if (!same)
            {
                Buf.I(ref _lastJCp, _jCols + 1);
                Buf.I(ref _lastJRi, Math.Max(_jNnz, 1));
                Array.Copy(_jCp, _lastJCp, _jCols + 1);
                Array.Copy(_jRi, _lastJRi, _jNnz);
                _lastJRows = _jRows;
                _lastJCols = _jCols;
                _lastJNnz = _jNnz;
            }
            return same;
        }

        private void WrapG(int n)
        {
            _gN = n;
            if (_g == null || _g.RowCount != n
                || !ReferenceEquals(_g.ColumnPointers, _gCp)
                || !ReferenceEquals(_g.RowIndices, _gRi)
                || !ReferenceEquals(_g.Values, _gVx))
                _g = new CscMatrix(n, n, _gCp, _gRi, _gVx);
        }

        private bool FormAtA(bool symbolic)
        {
            int m = _jRows;
            int n = _jCols;
            if (!TransposeJ())
                return false;
            Buf.I(ref _mark, n);
            Buf.D(ref _acc, n);
            Buf.I(ref _pat, n);
            Buf.I(ref _gMap, n);
            if (symbolic)
                return AtASymbolic(m, n);
            return AtANumeric(n);
        }

        private bool FormAAt(bool symbolic)
        {
            int m = _jRows;
            int n = _jCols;
            if (!TransposeJ())
                return false;
            Buf.I(ref _mark, m);
            Buf.D(ref _acc, m);
            Buf.I(ref _pat, m);
            Buf.I(ref _gMap, m);
            if (symbolic)
                return AAtSymbolic(m, n);
            return AAtNumeric(m);
        }

        private bool TransposeJ()
        {
            int m = _jRows;
            int n = _jCols;
            int nnz = _jNnz;
            Buf.I(ref _atCp, m + 1);
            Buf.Zero(_atCp, m + 1);
            for (int p = 0; p < nnz; p++)
                _atCp[_jRi[p] + 1]++;
            for (int i = 0; i < m; i++)
                _atCp[i + 1] += _atCp[i];
            Buf.I(ref _atRi, nnz == 0 ? 1 : nnz);
            Buf.D(ref _atVx, nnz == 0 ? 1 : nnz);
            Buf.I(ref _atNext, m);
            Array.Copy(_atCp, _atNext, m);
            for (int j = 0; j < n; j++)
            {
                int end = _jCp[j + 1];
                for (int p = _jCp[j]; p < end; p++)
                {
                    int i = _jRi[p];
                    int d = _atNext[i]++;
                    _atRi[d] = j;
                    _atVx[d] = _jVx[p];
                }
            }
            return true;
        }

        private bool AtASymbolic(int m, int n)
        {
            Buf.I(ref _gCp, n + 1);
            _gCp[0] = 0;
            _stamp = 1;
            int nnz = 0;
            Buf.I(ref _gRi, Math.Max(n, 1));
            Buf.D(ref _gVx, Math.Max(n, 1));
            for (int j = 0; j < n; j++)
            {
                _stamp = Buf.NextStamp(_mark, n, _stamp);
                int colNnz = 0;
                int aend = _jCp[j + 1];
                for (int p = _jCp[j]; p < aend; p++)
                {
                    int row = _jRi[p];
                    double aij = _jVx[p];
                    int rend = _atCp[row + 1];
                    for (int q = _atCp[row]; q < rend; q++)
                    {
                        int k = _atRi[q];
                        if (_mark[k] != _stamp)
                        {
                            _mark[k] = _stamp;
                            _acc[k] = aij * _atVx[q];
                            _pat[colNnz++] = k;
                        }
                        else
                            _acc[k] += aij * _atVx[q];
                    }
                }
                // A free parameter can have an empty Jacobian column. Keep
                // its diagonal in the symbolic pattern for regularization.
                if (_mark[j] != _stamp)
                {
                    _mark[j] = _stamp;
                    _acc[j] = 0;
                    _pat[colNnz++] = j;
                }
                Array.Sort(_pat, 0, colNnz);
                Buf.I(ref _gRi, nnz + colNnz);
                Buf.D(ref _gVx, nnz + colNnz);
                for (int t = 0; t < colNnz; t++)
                {
                    int i = _pat[t];
                    _gRi[nnz] = i;
                    _gVx[nnz] = _acc[i];
                    nnz++;
                }
                _gCp[j + 1] = nnz;
            }
            _gNnz = nnz;
            _gN = n;
            _hasPattern = true;
            return true;
        }

        private void AddTraceDamping(int n)
        {
            double trace = 0;
            for (int column = 0; column < n; column++)
                for (int p = _gCp[column]; p < _gCp[column + 1]; p++)
                    if (_gRi[p] == column) trace += _gVx[p];
            double damping = NewtonSolver.TraceDamping(trace, n);
            for (int column = 0; column < n; column++)
                for (int p = _gCp[column]; p < _gCp[column + 1]; p++)
                    if (_gRi[p] == column) _gVx[p] += damping;
        }

        private bool AtANumeric(int n)
        {
            Array.Clear(_gVx, 0, _gNnz);
            Buf.Fill(_gMap, n, -1);
            for (int j = 0; j < n; j++)
            {
                int lo = _gCp[j];
                int hi = _gCp[j + 1];
                for (int p = lo; p < hi; p++)
                    _gMap[_gRi[p]] = p;
                int aend = _jCp[j + 1];
                for (int p = _jCp[j]; p < aend; p++)
                {
                    int row = _jRi[p];
                    double aij = _jVx[p];
                    int rend = _atCp[row + 1];
                    for (int q = _atCp[row]; q < rend; q++)
                    {
                        int dest = _gMap[_atRi[q]];
                        if (dest >= 0)
                            _gVx[dest] += aij * _atVx[q];
                    }
                }
                for (int p = lo; p < hi; p++)
                    _gMap[_gRi[p]] = -1;
            }
            return true;
        }

        private bool AAtSymbolic(int m, int n)
        {
            Buf.I(ref _gCp, m + 1);
            _gCp[0] = 0;
            _stamp = 1;
            int nnz = 0;
            Buf.I(ref _gRi, Math.Max(m, 1));
            Buf.D(ref _gVx, Math.Max(m, 1));
            for (int i = 0; i < m; i++)
            {
                _stamp = Buf.NextStamp(_mark, m, _stamp);
                int colNnz = 0;
                int rend = _atCp[i + 1];
                for (int q = _atCp[i]; q < rend; q++)
                {
                    int col = _atRi[q];
                    double air = _atVx[q];
                    int aend = _jCp[col + 1];
                    for (int p = _jCp[col]; p < aend; p++)
                    {
                        int k = _jRi[p];
                        if (_mark[k] != _stamp)
                        {
                            _mark[k] = _stamp;
                            _acc[k] = air * _jVx[p];
                            _pat[colNnz++] = k;
                        }
                        else
                            _acc[k] += air * _jVx[p];
                    }
                }
                // A satisfied constant equation can have an empty Jacobian
                // row. Its dual-Gramian diagonal still needs regularization.
                if (_mark[i] != _stamp)
                {
                    _mark[i] = _stamp;
                    _acc[i] = 0;
                    _pat[colNnz++] = i;
                }
                Array.Sort(_pat, 0, colNnz);
                Buf.I(ref _gRi, nnz + colNnz);
                Buf.D(ref _gVx, nnz + colNnz);
                for (int t = 0; t < colNnz; t++)
                {
                    int row = _pat[t];
                    _gRi[nnz] = row;
                    _gVx[nnz] = _acc[row];
                    nnz++;
                }
                _gCp[i + 1] = nnz;
            }
            _gNnz = nnz;
            _gN = m;
            _hasPattern = true;
            return true;
        }

        private bool AAtNumeric(int m)
        {
            Array.Clear(_gVx, 0, _gNnz);
            Buf.Fill(_gMap, m, -1);
            for (int i = 0; i < m; i++)
            {
                int lo = _gCp[i];
                int hi = _gCp[i + 1];
                for (int p = lo; p < hi; p++)
                    _gMap[_gRi[p]] = p;
                int rend = _atCp[i + 1];
                for (int q = _atCp[i]; q < rend; q++)
                {
                    int col = _atRi[q];
                    double air = _atVx[q];
                    int aend = _jCp[col + 1];
                    for (int p = _jCp[col]; p < aend; p++)
                    {
                        int dest = _gMap[_jRi[p]];
                        if (dest >= 0)
                            _gVx[dest] += air * _jVx[p];
                    }
                }
                for (int p = lo; p < hi; p++)
                    _gMap[_gRi[p]] = -1;
            }
            return true;
        }
    }
}
