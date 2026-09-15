using GeoSolver.Sparse;

namespace GeoSolver
{
    public class NewtonSolver
    {
        public const double LENGTH_EPS = 1e-6;
        public const double CONVERGE_TOLERANCE = LENGTH_EPS / 1e2;
        public const int SparseParameterThreshold = 80;

        public static bool NewtonSolve(Expr e, Param p, double eps = 1e-12)
        {
            double f = e.Evaluate();
            if (f < eps && f > -eps)
                return true;

            for (int i = 0; i < 80; i++)
            {
                f = e.Evaluate();
                double df = e.EvaluatePartialDerivative(p);
                if (Math.Abs(df) < eps)
                    return Math.Abs(f) < CONVERGE_TOLERANCE;

                double delta = f / df;
                p.Value = p.Value - delta;
                if (Math.Abs(delta) < CONVERGE_TOLERANCE)
                    return true;
            }
            return Math.Abs(e.Evaluate()) < CONVERGE_TOLERANCE;
        }

        public static bool NewtonSolve(IEquationContainer equations, HashSet<Param> draggedParams = null)
        {
            return NewtonSolveDetailed(equations, draggedParams).Converged;
        }

        /// <param name="allowSoftOverconstrained">
        /// When false (default for sketches), more equations than parameters refuses the step.
        /// Kinematics and contact pass true so consistent redundant loops use Gauss–Newton.
        /// </param>
        public static SolveResult NewtonSolveDetailed(
            IEquationContainer equations,
            HashSet<Param> draggedParams = null,
            HashSet<Param> nonNegativeParams = null,
            double lengthScale = 1.0,
            bool? forceSparse = null,
            bool allowSoftOverconstrained = false,
            int maxNewtonIterations = 0)
        {
            int numEquations = equations.NumEquations;
            int numParameters = equations.NumParameters;
            if (numEquations == 0)
                return new SolveResult(true, 0, numParameters, 0);

            // Regularized Fischer–Burmeister residuals settle near ~sqrt(ε)≈1e-5, not LENGTH_EPS/100.
            // Soft overconstrained kinematics keep a small consistent-redundancy floor.
            double residualTol;
            if (nonNegativeParams != null && nonNegativeParams.Count > 0)
                residualTol = Math.Max(CONVERGE_TOLERANCE, 5e-5);
            else if (allowSoftOverconstrained)
                residualTol = LENGTH_EPS;
            else
                residualTol = CONVERGE_TOLERANCE;

            double[] b = new double[Math.Max(numEquations, 1)];
            FillResidual(equations, b);

            if (numParameters == 0)
            {
                bool ok = IsConverged(b, numEquations, residualTol);
                return new SolveResult(ok, SumSquares(b, numEquations), 0, numEquations,
                    ok ? null : "No free parameters with residual equations remaining.");
            }

            if (IsConverged(b, numEquations, residualTol))
                return new SolveResult(true, SumSquares(b, numEquations), numParameters, numEquations);

            if (numEquations > numParameters && !allowSoftOverconstrained)
                return new SolveResult(false, SumSquares(b, numEquations), numParameters, numEquations, "Overconstrained");

            int maxIter = maxNewtonIterations > 0 ? maxNewtonIterations : 400;
            bool useSparse = forceSparse ?? (numParameters >= SparseParameterThreshold);
            return useSparse
                ? RunSparse(equations, b, draggedParams, nonNegativeParams, lengthScale, residualTol, maxIter)
                : RunDense(equations, b, draggedParams, nonNegativeParams, lengthScale, residualTol, maxIter);
        }

        public static bool SparseNewtonSolve(IEquationContainer equations, HashSet<Param> dragged = null)
        {
            return NewtonSolveDetailed(equations, dragged, forceSparse: true).Converged;
        }

        private static SolveResult RunDense(
            IEquationContainer equations,
            double[] b,
            HashSet<Param> draggedParams,
            HashSet<Param> nonNegativeParams,
            double lengthScale,
            double residualTol,
            int maxIter)
        {
            int numEquations = equations.NumEquations;
            int numParameters = equations.NumParameters;
            double[] result = new double[numParameters];
            double[,] jacobian = new double[numEquations, numParameters];
            double[] columnScale = BuildColumnScales(equations, draggedParams, lengthScale);
            double[] scratch = new double[Math.Max(numEquations, numParameters)];
            double[,] work = null;
            double[] oldValues = new double[numParameters];

            int iter = 0;
            bool converged = false;
            string message = null;

            do
            {
                for (int i = 0; i < numEquations; i++)
                    for (int j = 0; j < numParameters; j++)
                        jacobian[i, j] = equations.EvaluatJacobian(i, j);

                ApplyColumnScalingInPlace(jacobian, columnScale, numEquations, numParameters);

                if (!SolveNewtonStep(result, jacobian, b, numEquations, numParameters, columnScale, ref work, scratch))
                {
                    message = "Dense linear solve failed.";
                    break;
                }

                if (!BacktrackingLineSearch(equations, b, result, oldValues, nonNegativeParams, residualTol, out _))
                    return new SolveResult(false, SumSquares(b, numEquations), numParameters, numEquations, "NaN residual during line search.");

                converged = IsConverged(b, numEquations, residualTol);
            } while (iter++ < maxIter && !converged);

            return new SolveResult(converged, SumSquares(b, numEquations), numParameters, numEquations,
                converged ? null : message ?? "Max iterations reached.");
        }

        private static SolveResult RunSparse(
            IEquationContainer equations,
            double[] b,
            HashSet<Param> draggedParams,
            HashSet<Param> nonNegativeParams,
            double lengthScale,
            double residualTol,
            int maxIter)
        {
            int numEquations = equations.NumEquations;
            int numParameters = equations.NumParameters;
            double[] result = new double[numParameters];
            double[] columnScale = BuildColumnScales(equations, draggedParams, lengthScale);
            double[] scratchEq = new double[numEquations];
            double[] scratchParam = new double[numParameters];
            double[] oldValues = new double[numParameters];

            var ws = new SparseNewtonWorkspace();
            int iter = 0;
            bool converged = false;
            string message = null;

            do
            {
                if (!ws.TryStep(equations, columnScale, b, result, scratchEq, scratchParam))
                {
                    message = "Sparse linear solve failed.";
                    break;
                }

                if (!BacktrackingLineSearch(equations, b, result, oldValues, nonNegativeParams, residualTol, out _))
                    return new SolveResult(false, SumSquares(b, numEquations), numParameters, numEquations, "NaN residual during line search.");

                converged = IsConverged(b, numEquations, residualTol);
            } while (iter++ < maxIter && !converged);

            return new SolveResult(converged, SumSquares(b, numEquations), numParameters, numEquations,
                converged ? null : message ?? "Max iterations reached.");
        }

        /// <summary>
        /// Underdetermined: min-norm via JJᵀ. Square or overconstrained: Gauss–Newton (JᵀJ) Δ = Jᵀ b with light damping.
        /// </summary>
        private static bool SolveNewtonStep(
            double[] result,
            double[,] jScaled,
            double[] b,
            int numEquations,
            int numParameters,
            double[] columnScale,
            ref double[,] work,
            double[] scratch)
        {
            // Min-norm JJᵀ only when underdetermined. Square systems (including
            // rank-deficient assembly mates) use Gauss–Newton: damped JJᵀ on a
            // singular square Jacobian stalls with a residual floor.
            if (numEquations < numParameters)
            {
                if (work == null || work.GetLength(0) != numEquations || work.GetLength(1) != numEquations)
                    work = new double[numEquations, numEquations];

                for (int r = 0; r < numEquations; r++)
                {
                    for (int c = 0; c < numEquations; c++)
                    {
                        double sum = 0;
                        for (int i = 0; i < numParameters; i++)
                            sum += jScaled[r, i] * jScaled[c, i];
                        work[r, c] = sum;
                    }
                }

                double[] rhs = new double[numEquations];
                Array.Copy(b, rhs, numEquations);
                if (SolveLinearSystem(scratch, work, rhs, numEquations))
                {
                    for (int c = 0; c < numParameters; c++)
                    {
                        double sum = 0;
                        for (int i = 0; i < numEquations; i++)
                            sum += jScaled[i, c] * scratch[i];
                        result[c] = sum * columnScale[c];
                    }
                    return true;
                }

                return false;
            }

            if (work == null || work.GetLength(0) != numParameters || work.GetLength(1) != numParameters)
                work = new double[numParameters, numParameters];

            for (int r = 0; r < numParameters; r++)
            {
                for (int c = 0; c < numParameters; c++)
                {
                    double sum = 0;
                    for (int i = 0; i < numEquations; i++)
                        sum += jScaled[i, r] * jScaled[i, c];
                    work[r, c] = sum;
                }
            }

            AddTraceDamping(work, numParameters);

            for (int c = 0; c < numParameters; c++)
            {
                double sum = 0;
                for (int i = 0; i < numEquations; i++)
                    sum += jScaled[i, c] * b[i];
                scratch[c] = sum;
            }

            double[] rhsParam = new double[numParameters];
            Array.Copy(scratch, rhsParam, numParameters);
            if (!SolveLinearSystem(result, work, rhsParam, numParameters))
                return false;
            UnscaleStep(result, columnScale, numParameters);
            return true;
        }

        private static void AddTraceDamping(double[,] work, int n)
        {
            double trace = 0;
            for (int r = 0; r < n; r++)
                trace += work[r, r];
            double damp = 1e-8 * (1.0 + Math.Abs(trace) / Math.Max(1, n));
            for (int r = 0; r < n; r++)
                work[r, r] += damp;
        }

        internal static double[] BuildColumnScales(
            IEquationContainer equations,
            HashSet<Param> draggedParams,
            double lengthScale)
        {
            double L = lengthScale > 1e-12 ? lengthScale : 1.0;
            int n = equations.NumParameters;
            double[] scale = new double[n];
            for (int c = 0; c < n; c++)
            {
                Param p = equations.GetParameter(c);
                double s = 1.0;
                if (p.ScaleKind == ParamScaleKind.ContactLambda)
                    s = 1.0 / L;
                if (draggedParams != null && draggedParams.Contains(p))
                    s *= 1.0 / 20.0;
                scale[c] = s;
            }
            return scale;
        }

        private static void ApplyColumnScalingInPlace(double[,] jacobian, double[] columnScale, int numEquations, int numParameters)
        {
            for (int c = 0; c < numParameters; c++)
            {
                double s = columnScale[c];
                if (s == 1.0)
                    continue;
                for (int r = 0; r < numEquations; r++)
                    jacobian[r, c] *= s;
            }
        }

        private static void UnscaleStep(double[] result, double[] columnScale, int numParameters)
        {
            for (int c = 0; c < numParameters; c++)
                result[c] *= columnScale[c];
        }

        private static void FillResidual(IEquationContainer equations, double[] b)
        {
            for (int i = 0; i < equations.NumEquations; i++)
                b[i] = equations.Evaluate(i);
        }

        private static bool BacktrackingLineSearch(
            IEquationContainer equations,
            double[] b,
            double[] step,
            double[] oldValues,
            HashSet<Param> nonNegativeParams,
            double residualTol,
            out double newError)
        {
            int numParameters = equations.NumParameters;
            int numEquations = equations.NumEquations;
            double currentError = SumSquares(b, numEquations);
            for (int i = 0; i < numParameters; i++)
                oldValues[i] = equations.GetParameter(i).Value;

            double alpha = 1.0;
            newError = currentError;

            for (int lsIter = 0; lsIter < 10; lsIter++)
            {
                bool hasNaN = false;
                for (int i = 0; i < numParameters; i++)
                {
                    equations.GetParameter(i).Value = oldValues[i] - alpha * step[i];
                    if (double.IsNaN(equations.GetParameter(i).Value))
                    {
                        hasNaN = true;
                        break;
                    }
                }

                if (hasNaN)
                {
                    alpha *= 0.5;
                    continue;
                }

                ClampNonNegative(equations, nonNegativeParams);

                newError = 0;
                bool nanResidual = false;
                for (int i = 0; i < numEquations; i++)
                {
                    b[i] = equations.Evaluate(i);
                    if (double.IsNaN(b[i]))
                    {
                        nanResidual = true;
                        newError = double.MaxValue;
                        break;
                    }
                    newError += b[i] * b[i];
                }

                if (nanResidual)
                {
                    alpha *= 0.5;
                    if (alpha < 1e-8)
                        return false;
                    continue;
                }

                if (newError < currentError * 1.1 || newError < residualTol * residualTol * numEquations)
                    return true;

                alpha *= 0.5;
                if (alpha < 1e-8)
                {
                    for (int i = 0; i < numParameters; i++)
                        equations.GetParameter(i).Value = oldValues[i] - alpha * step[i];
                    ClampNonNegative(equations, nonNegativeParams);
                    for (int i = 0; i < numEquations; i++)
                        b[i] = equations.Evaluate(i);
                    newError = SumSquares(b, numEquations);
                    return IsFinite(b, numEquations);
                }
            }

            return true;
        }

        private static void ClampNonNegative(IEquationContainer equations, HashSet<Param> nonNegativeParams)
        {
            if (nonNegativeParams == null || nonNegativeParams.Count == 0)
                return;
            for (int i = 0; i < equations.NumParameters; i++)
            {
                Param p = equations.GetParameter(i);
                if (nonNegativeParams.Contains(p) && p.Value < 0)
                    p.Value = 0;
            }
        }

        private static bool IsConverged(double[] b, int n, double tol)
        {
            for (int i = 0; i < n; i++)
            {
                if (double.IsNaN(b[i]) || Math.Abs(b[i]) > tol)
                    return false;
            }
            return true;
        }

        private static bool IsFinite(double[] b, int n)
        {
            for (int i = 0; i < n; i++)
            {
                if (double.IsNaN(b[i]))
                    return false;
            }
            return true;
        }

        private static double SumSquares(double[] b, int n)
        {
            double s = 0;
            for (int i = 0; i < n; i++)
                s += b[i] * b[i];
            return s;
        }

        public static bool SolveLinearSystem(double[] result, double[,] A, double[] b, int n)
        {
            int imax = -1;
            for (int i = 0; i < n; i++)
            {
                double max = 0;
                for (int ip = i; ip < n; ip++)
                {
                    if (Math.Abs(A[ip, i]) > max)
                    {
                        imax = ip;
                        max = Math.Abs(A[ip, i]);
                    }
                }
                if (Math.Abs(max) < 1e-25)
                    return false;

                double tmp;
                for (int jp = 0; jp < n; jp++)
                {
                    tmp = A[i, jp];
                    A[i, jp] = A[imax, jp];
                    A[imax, jp] = tmp;
                }
                tmp = b[i];
                b[i] = b[imax];
                b[imax] = tmp;

                for (int ip = i + 1; ip < n; ip++)
                {
                    double temp = A[ip, i] / A[i, i];
                    for (int jp = i; jp < n; jp++)
                        A[ip, jp] -= temp * A[i, jp];
                    b[ip] -= temp * b[i];
                }
            }

            for (int i = n - 1; i >= 0; i--)
            {
                if (Math.Abs(A[i, i]) < 1e-25)
                    return false;

                double temp = b[i];
                for (int j = n - 1; j > i; j--)
                    temp -= result[j] * A[i, j];
                result[i] = temp / A[i, i];
            }

            return true;
        }
    }
}
