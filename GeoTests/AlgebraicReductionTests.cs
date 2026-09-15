using GeoSolver;

namespace GeoTests;

public class AlgebraicReductionTests
{
    [Fact]
    public void FoldConstants_DoesNotMutateSharedPrimalTree()
    {
        Param p = new Param(1.0);
        Expr omega = Expr.Parameter(p);
        Expr invN = Expr.Constant(1.0) / Expr.Sqrt(omega * omega + Expr.Constant(1.0));
        double before = invN.Evaluate();
        Expr deriv = invN.PartialDerivative(p);
        double ad = deriv.Evaluate();
        double after = invN.Evaluate();
        Assert.Equal(before, after, 12);

        const double h = 1e-6;
        p.Value = 1.0 + h;
        double plus = invN.Evaluate();
        p.Value = 1.0 - h;
        double minus = invN.Evaluate();
        p.Value = 1.0;
        double fd = (plus - minus) / (2.0 * h);
        Assert.InRange(Math.Abs(ad - fd), 0, 1e-6);

        Expr folded = invN.FoldConstants();
        Assert.Equal(ExprType.Divide, folded.Type);
        Assert.Equal(before, folded.Evaluate(), 12);
    }

    [Fact]
    public void Substitute_MergesIdenticalParameters()
    {
        Param a = new Param(3);
        Param b = new Param(8);
        var eqs = new List<Expr>
        {
            Expr.Parameter(a) - Expr.Parameter(b),
            (Expr.Parameter(a) + Expr.Parameter(b)) - Expr.Constant(10)
        };
        var container = new EquationContainer(eqs, applySketchReductions: true);
        Assert.True(container.NumParameters <= 1);
        NewtonSolver.NewtonSolve(container);
        container.UpdateSubstituedParameters();
        Assert.InRange(a.Value, 4.9, 5.1);
        Assert.InRange(b.Value, 4.9, 5.1);
    }

    [Fact]
    public void Substitute_PinsParameterToConstant_IncludingLengthScale()
    {
        Param x = new Param(12);
        Expr scaled = (Expr.Parameter(x) - Expr.Constant(4.0)) * Expr.Constant(0.025);
        var container = new EquationContainer(new List<Expr> { scaled }, applySketchReductions: true);
        Assert.Equal(0, container.NumParameters);
        Assert.InRange(x.Value, 3.999, 4.001);
    }

    [Fact]
    public void Substitute_AffineOffset_MergesParameters()
    {
        Param a = new Param(0);
        Param b = new Param(0);
        // a - b - 40 = 0  =>  a = b + 40
        var eqs = new List<Expr>
        {
            (Expr.Parameter(a) - Expr.Parameter(b)) - Expr.Constant(40)
        };
        var container = new EquationContainer(eqs, applySketchReductions: true);
        Assert.Equal(0, container.NumParameters);
        container.UpdateSubstituedParameters();
        Assert.InRange(a.Value - b.Value, 39.999, 40.001);
    }

    [Fact]
    public void Substitute_PinScaledParameter()
    {
        Param x = new Param(0);
        Expr eq = Expr.Parameter(x) * Expr.Constant(2.0) - Expr.Constant(10.0);
        var container = new EquationContainer(new List<Expr> { eq }, applySketchReductions: true);
        Assert.Equal(0, container.NumParameters);
        Assert.InRange(x.Value, 4.999, 5.001);
    }

    [Fact]
    public void Substitute_KeepsFrozenParameter()
    {
        Param frozen = new Param(2);
        frozen.Frozen = true;
        Param free = new Param(9);
        var eqs = new List<Expr>
        {
            Expr.Parameter(free) - Expr.Parameter(frozen)
        };
        var container = new EquationContainer(eqs, applySketchReductions: true);
        container.UpdateSubstituedParameters();
        Assert.Equal(0, container.NumParameters);
        Assert.InRange(free.Value, 1.999, 2.001);
        Assert.InRange(frozen.Value, 1.999, 2.001);
    }
}
