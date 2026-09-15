namespace GeoSolver.Sparse
{
    /// <summary>
    /// Compressed sparse column matrix (0-based). Columns store row indices in ascending order.
    /// </summary>
    public sealed class CscMatrix
    {
        public CscMatrix(int rowCount, int columnCount, int[] columnPointers, int[] rowIndices, double[] values)
        {
            if (rowCount < 0 || columnCount < 0)
                throw new ArgumentOutOfRangeException();
            if (columnPointers == null || columnPointers.Length < columnCount + 1)
                throw new ArgumentException("ColumnPointers must have length ColumnCount + 1.");
            if (rowIndices == null || values == null)
                throw new ArgumentNullException();
            int nnz = columnPointers[columnCount];
            if (columnPointers[0] != 0 || nnz < 0)
                throw new ArgumentException("CSC pointers do not match index array length.");
            if (rowIndices.Length < nnz || values.Length < nnz)
                throw new ArgumentException("RowIndices and Values must cover all nonzeros.");

            RowCount = rowCount;
            ColumnCount = columnCount;
            ColumnPointers = columnPointers;
            RowIndices = rowIndices;
            Values = values;
        }

        public int RowCount { get; }
        public int ColumnCount { get; }
        public int[] ColumnPointers { get; }
        public int[] RowIndices { get; }
        public double[] Values { get; }
        public int NonZeros => ColumnPointers[ColumnCount];

        public static CscMatrix FromTriplets(int rowCount, int columnCount, List<int> rows, List<int> cols, List<double> vals)
        {
            if (rows.Count != cols.Count || rows.Count != vals.Count)
                throw new ArgumentException("Triplet arrays must have equal length.");

            int nnzIn = rows.Count;
            var order = new int[nnzIn];
            for (int i = 0; i < nnzIn; i++)
                order[i] = i;
            Array.Sort(order, (a, b) =>
            {
                int cmp = cols[a].CompareTo(cols[b]);
                return cmp != 0 ? cmp : rows[a].CompareTo(rows[b]);
            });

            var cp = new int[columnCount + 1];
            var ri = new List<int>(nnzIn);
            var vx = new List<double>(nnzIn);
            int col = 0;
            int lastRow = -1;
            int lastCol = -1;
            for (int t = 0; t < nnzIn; t++)
            {
                int s = order[t];
                int c = cols[s];
                int r = rows[s];
                double v = vals[s];
                if ((uint)c >= (uint)columnCount || (uint)r >= (uint)rowCount)
                    throw new ArgumentOutOfRangeException();
                while (col < c)
                {
                    cp[col + 1] = ri.Count;
                    col++;
                    lastRow = -1;
                }
                if (c == lastCol && r == lastRow)
                {
                    vx[vx.Count - 1] += v;
                    continue;
                }
                if (v == 0.0)
                    continue;
                ri.Add(r);
                vx.Add(v);
                lastRow = r;
                lastCol = c;
            }
            while (col < columnCount)
            {
                cp[col + 1] = ri.Count;
                col++;
            }
            cp[columnCount] = ri.Count;
            return new CscMatrix(rowCount, columnCount, cp, ri.ToArray(), vx.ToArray());
        }

        public CscMatrix Transpose()
        {
            int m = RowCount;
            int n = ColumnCount;
            int nnz = NonZeros;
            var tcp = new int[m + 1];
            for (int p = 0; p < nnz; p++)
                tcp[RowIndices[p] + 1]++;
            for (int i = 0; i < m; i++)
                tcp[i + 1] += tcp[i];

            var tri = new int[nnz];
            var tv = new double[nnz];
            var next = new int[m];
            Array.Copy(tcp, next, m);
            for (int j = 0; j < n; j++)
            {
                int end = ColumnPointers[j + 1];
                for (int p = ColumnPointers[j]; p < end; p++)
                {
                    int i = RowIndices[p];
                    int d = next[i]++;
                    tri[d] = j;
                    tv[d] = Values[p];
                }
            }
            return new CscMatrix(n, m, tcp, tri, tv);
        }

        public void Multiply(double[] x, double[] y)
        {
            int m = RowCount;
            int n = ColumnCount;
            Array.Clear(y, 0, m);
            for (int j = 0; j < n; j++)
            {
                double xj = x[j];
                if (xj == 0.0)
                    continue;
                int end = ColumnPointers[j + 1];
                for (int p = ColumnPointers[j]; p < end; p++)
                    y[RowIndices[p]] += Values[p] * xj;
            }
        }

        public void TransposeMultiply(double[] x, double[] y)
        {
            int n = ColumnCount;
            for (int j = 0; j < n; j++)
            {
                double sum = 0;
                int end = ColumnPointers[j + 1];
                for (int p = ColumnPointers[j]; p < end; p++)
                    sum += Values[p] * x[RowIndices[p]];
                y[j] = sum;
            }
        }

        /// <summary>Gramian G = AᵀA (n×n, structurally symmetric, both triangles stored).</summary>
        public CscMatrix TransposeMultiplySelf()
        {
            return FormGramian(transposeLeft: true);
        }

        /// <summary>G = AAᵀ (m×m, structurally symmetric, both triangles stored).</summary>
        public CscMatrix MultiplyTransposeSelf()
        {
            return FormGramian(transposeLeft: false);
        }

        /// <summary>
        /// Gustavson sparse × sparse. Result is this × other.
        /// </summary>
        public CscMatrix Multiply(CscMatrix other)
        {
            if (ColumnCount != other.RowCount)
                throw new ArgumentException("Inner dimensions must agree.");

            int m = RowCount;
            int n = other.ColumnCount;
            int kdim = ColumnCount;
            int[] bp = other.ColumnPointers;
            int[] bi = other.RowIndices;
            double[] bx = other.Values;

            var mark = new int[m];
            var acc = new double[m];
            var pattern = new int[m];
            int stamp = 1;

            var cp = new int[n + 1];
            var riList = new List<int>(NonZeros + other.NonZeros);
            var vxList = new List<double>(NonZeros + other.NonZeros);

            for (int j = 0; j < n; j++)
            {
                if (stamp == int.MaxValue)
                {
                    Array.Clear(mark, 0, m);
                    stamp = 1;
                }
                int nnzCol = 0;
                int bend = bp[j + 1];
                for (int t = bp[j]; t < bend; t++)
                {
                    int k = bi[t];
                    if ((uint)k >= (uint)kdim)
                        continue;
                    double bkj = bx[t];
                    int aend = ColumnPointers[k + 1];
                    for (int p = ColumnPointers[k]; p < aend; p++)
                    {
                        int i = RowIndices[p];
                        if (mark[i] != stamp)
                        {
                            mark[i] = stamp;
                            acc[i] = Values[p] * bkj;
                            pattern[nnzCol++] = i;
                        }
                        else
                            acc[i] += Values[p] * bkj;
                    }
                }

                Array.Sort(pattern, 0, nnzCol);
                for (int t = 0; t < nnzCol; t++)
                {
                    int i = pattern[t];
                    double v = acc[i];
                    if (v != 0.0)
                    {
                        riList.Add(i);
                        vxList.Add(v);
                    }
                }
                cp[j + 1] = riList.Count;
                stamp++;
            }

            return new CscMatrix(m, n, cp, riList.ToArray(), vxList.ToArray());
        }

        private CscMatrix FormGramian(bool transposeLeft)
        {
            // AᵀA: columns of A are the "factors". AAᵀ: rows of A, via CSC outer products of each column.
            if (transposeLeft)
                return FormAtA();
            return FormAAt();
        }

        private CscMatrix FormAtA()
        {
            int n = ColumnCount;
            CscMatrix at = Transpose();
            int[] rp = at.ColumnPointers;
            int[] rj = at.RowIndices;
            double[] rx = at.Values;

            var mark = new int[n];
            var acc = new double[n];
            var pattern = new int[n];
            int stamp = 1;
            var cp = new int[n + 1];
            var riList = new List<int>(Math.Max(NonZeros, n));
            var vxList = new List<double>(Math.Max(NonZeros, n));

            for (int j = 0; j < n; j++)
            {
                if (stamp == int.MaxValue)
                {
                    Array.Clear(mark, 0, n);
                    stamp = 1;
                }
                int nnzCol = 0;
                int aend = ColumnPointers[j + 1];
                for (int p = ColumnPointers[j]; p < aend; p++)
                {
                    int row = RowIndices[p];
                    double aij = Values[p];
                    int rend = rp[row + 1];
                    for (int q = rp[row]; q < rend; q++)
                    {
                        int k = rj[q];
                        if (mark[k] != stamp)
                        {
                            mark[k] = stamp;
                            acc[k] = aij * rx[q];
                            pattern[nnzCol++] = k;
                        }
                        else
                            acc[k] += aij * rx[q];
                    }
                }

                Array.Sort(pattern, 0, nnzCol);
                for (int t = 0; t < nnzCol; t++)
                {
                    int i = pattern[t];
                    double v = acc[i];
                    if (v != 0.0)
                    {
                        riList.Add(i);
                        vxList.Add(v);
                    }
                }
                cp[j + 1] = riList.Count;
                stamp++;
            }

            return new CscMatrix(n, n, cp, riList.ToArray(), vxList.ToArray());
        }

        private CscMatrix FormAAt()
        {
            int m = RowCount;
            int n = ColumnCount;
            var mark = new int[m];
            var acc = new double[m];
            var pattern = new int[m];
            int stamp = 1;
            var cp = new int[m + 1];
            var riList = new List<int>(Math.Max(NonZeros, m));
            var vxList = new List<double>(Math.Max(NonZeros, m));

            // Column i of AAᵀ = A * (row i of A)ᵀ. Walk CSC by building CSR once.
            CscMatrix csrLike = Transpose();
            int[] rp = csrLike.ColumnPointers;
            int[] rj = csrLike.RowIndices;
            double[] rx = csrLike.Values;

            for (int i = 0; i < m; i++)
            {
                if (stamp == int.MaxValue)
                {
                    Array.Clear(mark, 0, m);
                    stamp = 1;
                }
                int nnzCol = 0;
                int rend = rp[i + 1];
                for (int q = rp[i]; q < rend; q++)
                {
                    int col = rj[q];
                    double air = rx[q];
                    int aend = ColumnPointers[col + 1];
                    for (int p = ColumnPointers[col]; p < aend; p++)
                    {
                        int k = RowIndices[p];
                        if (mark[k] != stamp)
                        {
                            mark[k] = stamp;
                            acc[k] = air * Values[p];
                            pattern[nnzCol++] = k;
                        }
                        else
                            acc[k] += air * Values[p];
                    }
                }

                Array.Sort(pattern, 0, nnzCol);
                for (int t = 0; t < nnzCol; t++)
                {
                    int row = pattern[t];
                    double v = acc[row];
                    if (v != 0.0)
                    {
                        riList.Add(row);
                        vxList.Add(v);
                    }
                }
                cp[i + 1] = riList.Count;
                stamp++;
            }

            return new CscMatrix(m, m, cp, riList.ToArray(), vxList.ToArray());
        }

    }
}
