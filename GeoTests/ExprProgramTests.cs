using System.Diagnostics;
using GeoSolver;

namespace GeoTests;

public class ExprProgramTests
{
    [Fact]
    public void Bytecode_MatchesTreeEvaluate_OnArithmeticAndTrig()
    {
        var x = Expr.Parameter(0.3, "x");
        var y = Expr.Parameter(-1.1, "y");
        Expr e = (Expr.Sin(x) * Expr.Cos(y) + Expr.Sqrt(x * x + Expr.Constant(0.25))) / (y + Expr.Constant(2.0))
                 - Expr.Pow2(x - y) + Expr.Abs(y) + (-x);

        AssertNear(e.Evaluate(), ExprProgram.From(e).Evaluate());
        x.Value.Value = 1.7;
        y.Value.Value = 0.4;
        AssertNear(e.Evaluate(), ExprProgram.From(e).Evaluate());
    }

    [Fact]
    public void Compile_UsesBytecode_AndTracksLiveParams()
    {
        var p = Expr.Parameter(2.0, "p");
        Expr e = p * p + Expr.Constant(1.0);
        Func<double> f = e.Compile();
        AssertNear(5.0, f());
        p.Value.Value = 3.0;
        AssertNear(10.0, f());
    }

    [Fact]
    public void Benchmark_Tree_Vs_Bytecode()
    {
        var x = Expr.Parameter(0.25, "x");
        var y = Expr.Parameter(1.5, "y");
        var z = Expr.Parameter(-0.75, "z");
        Expr e = Expr.Pow2(x - y) + Expr.Pow2(y - z) + Expr.Sin(x * y) * Expr.Cos(z)
                 + Expr.Sqrt(x * x + y * y + Expr.Constant(1e-6))
                 - (x * y + y * z) / (z * z + Expr.Constant(0.5));

        ExprProgram prog = ExprProgram.From(e);
        AssertNear(e.Evaluate(), prog.Evaluate());

        const int warmup = 20000;
        const int iters = 400000;
        const int trials = 7;
        Warm(e, prog, warmup);

        double tree = MedianNs(trials, iters, i =>
        {
            x.Value.Value = 0.25 + (i & 31) * 1e-4;
            return e.Evaluate();
        });
        double bc = MedianNs(trials, iters, i =>
        {
            x.Value.Value = 0.25 + (i & 31) * 1e-4;
            return prog.Evaluate();
        });

        Console.WriteLine(
            $"median ns/eval x{iters} x{trials} trials: tree={tree:F1}  bytecode={bc:F1}  ops={prog.InstructionCount}");
        Assert.True(tree > 0 && bc > 0);
    }

    private static void Warm(Expr e, ExprProgram prog, int n)
    {
        double acc = 0;
        for (int i = 0; i < n; i++)
        {
            acc += e.Evaluate();
            acc += prog.Evaluate();
        }
        if (acc == double.NegativeInfinity)
            throw new InvalidOperationException();
    }

    private static double MedianNs(int trials, int iters, Func<int, double> eval)
    {
        var samples = new double[trials];
        for (int t = 0; t < trials; t++)
        {
            var sw = Stopwatch.StartNew();
            double acc = 0;
            for (int i = 0; i < iters; i++)
                acc += eval(i);
            sw.Stop();
            if (double.IsNaN(acc))
                throw new InvalidOperationException("NaN accumulator");
            samples[t] = sw.ElapsedTicks * (1e9 / Stopwatch.Frequency) / iters;
        }
        Array.Sort(samples);
        return samples[trials / 2];
    }

    private static void AssertNear(double expected, double actual)
    {
        Assert.True(Math.Abs(expected - actual) < 1e-12,
            $"expected {expected}, got {actual}");
    }
}
