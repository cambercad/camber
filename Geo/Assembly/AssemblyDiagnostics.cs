using GeoSolver;

namespace Geo;

/// <summary>A snapshot of one mate's normalized equation residuals at the current pose.</summary>
public sealed class AssemblyMateResidual
{
    internal AssemblyMateResidual(int index, AssemblyMateRecord mate, double[] residuals, double tolerance)
    {
        Index=index; Kind=mate.Kind.ToString(); Label=mate.Label;
        Entities=Array.AsReadOnly(mate.Entities.ToArray());
        Residuals=Array.AsReadOnly(residuals); Tolerance=tolerance;
        MaxResidual=residuals.Length==0 ? 0 : residuals.Max(value=>double.IsFinite(value) ? Math.Abs(value) : double.PositiveInfinity);
    }
    public int Index { get; }
    public string Kind { get; }
    public string Label { get; }
    public IReadOnlyList<string> Entities { get; }
    public IReadOnlyList<double> Residuals { get; }
    public double MaxResidual { get; }
    public double Tolerance { get; }
    public bool Satisfied => MaxResidual<=Tolerance;
}

public partial class Assembly
{
    private readonly List<(AssemblyMateRecord Mate,IBaseEquation[] Equations)> _mateEquations=new();
    public double CharacteristicLength => _solver.CharacteristicLength;

    /// <summary>
    /// Evaluate each mate's actual solver equations without solving or moving parts.
    /// Residuals use the solver's dimensionless normalization, not model units.
    /// Unsatisfied mates locate error; they do not prove which constraints cause a conflict.
    /// </summary>
    public IReadOnlyList<AssemblyMateResidual> GetMateResiduals()
    {
        var result=new List<AssemblyMateResidual>(_mateEquations.Count);
        for(int index=0;index<_mateEquations.Count;index++)
        {
            var (mate,constraints)=_mateEquations[index];
            var equations=new List<Expr>();
            bool contact=false;
            foreach(var constraint in constraints)
            {
                constraint.GenerateEquations(equations,_solver.CharacteristicLength);
                contact |= constraint.Any(parameter=>parameter.ScaleKind==ParamScaleKind.ContactLambda);
            }
            result.Add(new AssemblyMateResidual(index,mate,equations.Select(expression=>expression.Evaluate()).ToArray(),
                NewtonSolver.ResidualTolerance(contact)));
        }
        return result.AsReadOnly();
    }
}
