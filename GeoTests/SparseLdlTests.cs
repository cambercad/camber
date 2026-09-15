using GeoSolver;
using GeoSolver.Sparse;

namespace GeoTests;

public class SparseLdlTests
{
    [Fact]
    public void DenseSpd_MatchesExactSolution()
    {
        CscMatrix a = CscMatrix.FromTriplets(2, 2,
            new List<int> { 0, 1, 0, 1 },
            new List<int> { 0, 0, 1, 1 },
            new List<double> { 4, 1, 1, 3 });
        double[] b = { 5, 4 };
        double[] x = new double[2];
        Assert.True(SparseLdl.TrySolve(a, b, x));
        Assert.Equal(1.0, x[0], 10);
        Assert.Equal(1.0, x[1], 10);
    }

    [Fact]
    public void RepeatedNumericFactor_ReusesPatternAndUpdatesValues()
    {
        CscMatrix a = CscMatrix.FromTriplets(3, 3,
            new List<int> { 0, 1, 0, 1, 2, 1, 2 },
            new List<int> { 0, 0, 1, 1, 1, 2, 2 },
            new List<double> { 4, 1, 1, 4, 1, 1, 3 });
        var factor = new SparseLdl();
        double[] b = { 1, 2, 3 };
        double[] x = new double[3];

        Assert.True(factor.Factorize(a));
        factor.Solve(b, x);
        Assert.True(RelativeResidual(a, x, b) < 1e-12);

        a.Values[0] = 5;
        a.Values[3] = 6;
        a.Values[6] = 4;
        Assert.True(factor.Factorize(a));
        factor.Solve(b, x);
        Assert.True(RelativeResidual(a, x, b) < 1e-12);

        factor.Factorize(a);
        factor.Solve(b, x);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 20; i++)
        {
            Assert.True(factor.Factorize(a));
            factor.Solve(b, x);
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(0, allocated);
    }

    [Fact]
    public void TridiagonalLaplacian_ResidualIsSmall()
    {
        int n = 40;
        var rows = new List<int>();
        var cols = new List<int>();
        var vals = new List<double>();
        for (int i = 0; i < n; i++)
        {
            Add(rows, cols, vals, i, i, 2.0);
            if (i + 1 < n)
            {
                Add(rows, cols, vals, i, i + 1, -1.0);
                Add(rows, cols, vals, i + 1, i, -1.0);
            }
        }

        CscMatrix a = CscMatrix.FromTriplets(n, n, rows, cols, vals);
        double[] xTrue = new double[n];
        double[] b = new double[n];
        for (int i = 0; i < n; i++)
            xTrue[i] = 0.1 * (i + 1);
        a.Multiply(xTrue, b);

        double[] x = new double[n];
        Assert.True(SparseLdl.TrySolve(a, b, x));
        Assert.True(RelativeResidual(a, x, b) < 1e-10);
    }

    [Fact]
    public void GridLaplacian_ResidualIsSmall()
    {
        const int m = 24;
        int n = m * m;
        var rows = new List<int>();
        var cols = new List<int>();
        var vals = new List<double>();
            for (int gy = 0; gy < m; gy++)
            {
                for (int gx = 0; gx < m; gx++)
                {
                    int i = gy * m + gx;
                    double diag = 4.0;
                    TryGridEdge(rows, cols, vals, m, gx, gy, 1, 0, ref diag);
                    TryGridEdge(rows, cols, vals, m, gx, gy, -1, 0, ref diag);
                    TryGridEdge(rows, cols, vals, m, gx, gy, 0, 1, ref diag);
                    TryGridEdge(rows, cols, vals, m, gx, gy, 0, -1, ref diag);
                    Add(rows, cols, vals, i, i, diag);
                }
            }

        CscMatrix a = CscMatrix.FromTriplets(n, n, rows, cols, vals);
        double[] xTrue = new double[n];
        double[] b = new double[n];
        for (int i = 0; i < n; i++)
            xTrue[i] = Math.Sin(0.03 * i);
        a.Multiply(xTrue, b);

        double[] x = new double[n];
        Assert.True(SparseLdl.TrySolve(a, b, x));
        Assert.True(RelativeResidual(a, x, b) < 1e-9);
    }

    [Fact]
    public void RandomGramian_MatchesDenseSolve()
    {
        const int m = 30;
        const int n = 18;
        var rng = new Random(7);
        var rows = new List<int>();
        var cols = new List<int>();
        var vals = new List<double>();
        for (int j = 0; j < n; j++)
        {
            int nnz = 3 + rng.Next(3);
            var used = new HashSet<int>();
            for (int k = 0; k < nnz; k++)
            {
                int i = rng.Next(m);
                if (!used.Add(i))
                    continue;
                Add(rows, cols, vals, i, j, rng.NextDouble() * 2 - 1);
            }
        }

        CscMatrix jac = CscMatrix.FromTriplets(m, n, rows, cols, vals);
        CscMatrix g = jac.TransposeMultiplySelf();
        double[] xTrue = new double[n];
        for (int i = 0; i < n; i++)
            xTrue[i] = rng.NextDouble();
        double[] b = new double[n];
        g.Multiply(xTrue, b);

        double[] x = new double[n];
        Assert.True(SparseLdl.TrySolve(g, b, x));
        Assert.True(RelativeResidual(g, x, b) < 1e-8);

        double[] denseX = new double[n];
        double[,] dense = ToDense(g);
        Assert.True(NewtonSolver.SolveLinearSystem(denseX, dense, (double[])b.Clone(), n));
        for (int i = 0; i < n; i++)
            Assert.Equal(denseX[i], x[i], 6);
    }

    [Fact]
    public void JacobianAAt_UnderdeterminedMinNormPath()
    {
        var rows = new List<int> { 0, 0, 1, 1 };
        var cols = new List<int> { 0, 1, 1, 2 };
        var vals = new List<double> { 1, 1, 1, 1 };
        CscMatrix j = CscMatrix.FromTriplets(2, 3, rows, cols, vals);
        CscMatrix jjt = j.MultiplyTransposeSelf();
        double[] rhs = { 3, 2 };
        double[] y = new double[2];
        Assert.True(SparseLdl.TrySolve(jjt, rhs, y));
        double[] x = new double[3];
        j.TransposeMultiply(y, x);
        double[] jx = new double[2];
        j.Multiply(x, jx);
        Assert.Equal(3.0, jx[0], 8);
        Assert.Equal(2.0, jx[1], 8);
    }

    [Fact]
    public void AmdPermutation_IsValidBijection()
    {
        int n = 12;
        var rows = new List<int>();
        var cols = new List<int>();
        var vals = new List<double>();
        for (int i = 0; i < n; i++)
        {
            Add(rows, cols, vals, i, i, 4);
            Add(rows, cols, vals, i, (i + 1) % n, -1);
            Add(rows, cols, vals, (i + 1) % n, i, -1);
            Add(rows, cols, vals, i, (i + 3) % n, -1);
            Add(rows, cols, vals, (i + 3) % n, i, -1);
        }
        CscMatrix a = CscMatrix.FromTriplets(n, n, rows, cols, vals);
        int[] p = AmdOrdering.Compute(a);
        Assert.Equal(n, p.Length);
        var seen = new bool[n];
        for (int i = 0; i < n; i++)
        {
            Assert.InRange(p[i], 0, n - 1);
            Assert.False(seen[p[i]]);
            seen[p[i]] = true;
        }
    }

    [Fact]
    public void SparseNewton_TwoByTwo_AgreesWithDense()
    {
        Param xs = new Param(1.0);
        Param ys = new Param(2.0);
        var sparseEqs = new List<Expr>
        {
            Expr.Parameter(xs) + Expr.Parameter(ys) - Expr.Constant(5),
            Expr.Parameter(xs) - Expr.Parameter(ys) - Expr.Constant(1),
        };
        SolveResult sparseResult = NewtonSolver.NewtonSolveDetailed(
            new EquationContainer(sparseEqs, applySketchReductions: false), forceSparse: true);
        Assert.True(sparseResult.Converged, sparseResult.Message);
        Assert.Equal(3.0, xs.Value, 5);
        Assert.Equal(2.0, ys.Value, 5);
    }

    [Fact]
    public void SparseNewton_ChainOfOffsets_Converges()
    {
        const int n = 40;
        var ps = new Param[n];
        var eqs = new List<Expr>();
        for (int i = 0; i < n; i++)
            ps[i] = new Param(i * 0.1);
        eqs.Add(Expr.Parameter(ps[0]) - Expr.Constant(0));
        for (int i = 0; i < n - 1; i++)
            eqs.Add(Expr.Parameter(ps[i + 1]) - Expr.Parameter(ps[i]) - Expr.Constant(1));

        SolveResult result = NewtonSolver.NewtonSolveDetailed(
            new EquationContainer(eqs, applySketchReductions: false), forceSparse: true);
        Assert.True(result.Converged, result.Message);
        for (int i = 0; i < n; i++)
            Assert.Equal(i, ps[i].Value, 6);
    }

    private static void TryGridEdge(List<int> rows, List<int> cols, List<double> vals, int m, int x, int y, int dx, int dy, ref double diag)
    {
        int nx = x + dx;
        int ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= m || ny >= m)
            return;
        int i = y * m + x;
        int j = ny * m + nx;
        Add(rows, cols, vals, i, j, -1.0);
    }

    private static void Add(List<int> rows, List<int> cols, List<double> vals, int r, int c, double v)
    {
        rows.Add(r);
        cols.Add(c);
        vals.Add(v);
    }

    private static double RelativeResidual(CscMatrix a, double[] x, double[] b)
    {
        var ax = new double[a.RowCount];
        a.Multiply(x, ax);
        double num = 0;
        double den = 0;
        for (int i = 0; i < b.Length; i++)
        {
            double r = ax[i] - b[i];
            num += r * r;
            den += b[i] * b[i];
        }
        if (den == 0.0)
            return Math.Sqrt(num);
        return Math.Sqrt(num / den);
    }

    private static double[,] ToDense(CscMatrix a)
    {
        int n = a.ColumnCount;
        var d = new double[n, n];
        for (int j = 0; j < n; j++)
        {
            int end = a.ColumnPointers[j + 1];
            for (int p = a.ColumnPointers[j]; p < end; p++)
                d[a.RowIndices[p], j] = a.Values[p];
        }
        return d;
    }
}
