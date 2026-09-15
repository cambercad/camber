namespace GeoSolver
{
    /// <summary>Outcome of a multi-parameter Newton / least-squares constraint solve.</summary>
    public readonly struct SolveResult
    {
        public SolveResult(bool converged, double sumOfSquaredErrors, int numParameters, int numEquations, string message = null)
        {
            Converged = converged;
            SumOfSquaredErrors = sumOfSquaredErrors;
            NumParameters = numParameters;
            NumEquations = numEquations;
            Message = message;
        }

        public bool Converged { get; }
        public double SumOfSquaredErrors { get; }
        public int NumParameters { get; }
        public int NumEquations { get; }
        public string Message { get; }
    }

    /// <summary>
    /// Column-scaling hint for free Newton parameters.
    /// Contact multipliers use <see cref="ContactLambda"/>; everything else stays <see cref="Default"/>.
    /// </summary>
    public enum ParamScaleKind
    {
        Default = 0,
        ContactLambda = 1,
        Orientation = 2,
    }
}
