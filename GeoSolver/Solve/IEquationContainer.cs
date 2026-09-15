namespace GeoSolver
{

    public interface IEquationContainer
    {
        int NumParameters { get; }
        int NumEquations { get; }
        double Evaluate(int i);
        double EvaluatJacobian(int i, int j);
        Param GetParameter(int i);

        int NumNonZerosInRow(int i);
        void EvaluateJacobianRow(int rowIndex, ref int indexer, IList<double> values, IList<int> columnIndices);
    }
}