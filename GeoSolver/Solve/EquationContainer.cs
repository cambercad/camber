namespace GeoSolver
{

    public class EquationContainer : IEquationContainer
    {
        public int NumParameters { get { return _params.Length; } }
        public int NumEquations { get { return _equations.Length; ; } }

        private Expr[] _equations;
        private ExprProgram[] _equationCode;
        private Param[] _params;
        private int[][] _jacobianColumns;
        private HashSet<int>[] _jacobianColumnSet;
        private List<DependentParam> _dependentParams;

        public bool FullyDefined { get { return _equations.Length == _params.Length; } }

        private static Expr[] Substitute(Expr[] equations, out List<DependentParam> equalityMap, HashSet<Param> dragged = null)
        {
            equalityMap = new List<DependentParam>();
            List<Expr> newEquations = new List<Expr>();
            newEquations.AddRange(equations);

            bool progressed = true;
            while (progressed)
            {
                progressed = false;
                for (int i = newEquations.Count - 1; i >= 0; --i)
                {
                    Expr e = newEquations[i];
                    Param keep;
                    Param drop;
                    double scale;
                    double offset;
                    bool isPin;
                    if (!TryLinearReduction(e, dragged, out keep, out drop, out scale, out offset, out isPin))
                        continue;

                    newEquations.RemoveAt(i);
                    progressed = true;

                    if (isPin)
                    {
                        drop.Value = offset;
                        for (int j = 0; j < newEquations.Count; ++j)
                            newEquations[j] = newEquations[j].FreezeParamAsConstant(drop).FoldConstants();
                    }
                    else
                    {
                        Expr replacement = (Expr.Parameter(keep) * Expr.Constant(scale) + Expr.Constant(offset)).FoldConstants();
                        for (int j = 0; j < newEquations.Count; ++j)
                            newEquations[j] = newEquations[j].ReplaceParam(drop, replacement).FoldConstants();
                        equalityMap.Add(new DependentParam(keep, drop, scale, offset));
                    }
                    break;
                }
            }

            equalityMap.Reverse();
            return newEquations.ToArray();
        }

        /// <summary>
        /// Sketch-style exact reductions on linear residuals:
        /// one free parameter pins to a constant; two free parameters merge with an affine map
        /// drop = scale * keep + offset (covers p − q and p − q − c).
        /// </summary>
        internal static bool TryLinearReduction(
            Expr e,
            HashSet<Param> dragged,
            out Param keep,
            out Param drop,
            out double scale,
            out double offset,
            out bool isPin)
        {
            keep = null;
            drop = null;
            scale = 1.0;
            offset = 0.0;
            isPin = false;

            Dictionary<Param, double> coeffs = new Dictionary<Param, double>();
            double constant = 0.0;
            if (!TryCollectLinear(e, 1.0, coeffs, ref constant))
                return false;

            List<KeyValuePair<Param, double>> terms = new List<KeyValuePair<Param, double>>();
            foreach (var kv in coeffs)
            {
                if (Math.Abs(kv.Value) > 1e-14)
                    terms.Add(kv);
            }

            if (terms.Count == 1)
            {
                Param p = terms[0].Key;
                double a = terms[0].Value;
                if (Math.Abs(a) < 1e-12)
                    return false;
                if (p.Frozen)
                    return false;
                if (dragged != null && dragged.Contains(p))
                    return false;
                drop = p;
                keep = p;
                offset = -constant / a;
                isPin = true;
                return true;
            }

            if (terms.Count != 2)
                return false;

            Param p0 = terms[0].Key;
            Param p1 = terms[1].Key;
            double a0 = terms[0].Value;
            double a1 = terms[1].Value;
            if (p0 == p1 || (p0.Frozen && p1.Frozen))
                return false;
            if (dragged != null && dragged.Contains(p0) && dragged.Contains(p1))
                return false;

            bool drop0;
            if (p1.Frozen && !p0.Frozen)
                drop0 = true;
            else if (p0.Frozen && !p1.Frozen)
                drop0 = false;
            else if (dragged != null && dragged.Contains(p0) && !dragged.Contains(p1))
                drop0 = false;
            else if (dragged != null && dragged.Contains(p1) && !dragged.Contains(p0))
                drop0 = true;
            else
                drop0 = Math.Abs(a0) >= Math.Abs(a1);

            if (drop0)
            {
                if (Math.Abs(a0) < 1e-12)
                    return false;
                drop = p0;
                keep = p1;
                scale = -a1 / a0;
                offset = -constant / a0;
            }
            else
            {
                if (Math.Abs(a1) < 1e-12)
                    return false;
                drop = p1;
                keep = p0;
                scale = -a0 / a1;
                offset = -constant / a1;
            }

            if (drop.Frozen)
                return false;
            if (dragged != null && dragged.Contains(drop))
                return false;
            return true;
        }

        /// <summary>Kept for tests that still name the old entry point.</summary>
        internal static bool TryAlgebraicReduction(
            Expr e,
            HashSet<Param> dragged,
            out Param keep,
            out Param drop,
            out double pinValue,
            out bool isPin)
        {
            double scale;
            double offset;
            bool ok = TryLinearReduction(e, dragged, out keep, out drop, out scale, out offset, out isPin);
            pinValue = offset;
            return ok && (isPin || (Math.Abs(scale - 1.0) < 1e-12 && Math.Abs(offset) < 1e-12));
        }

        private static bool TryCollectLinear(Expr e, double scale, Dictionary<Param, double> coeffs, ref double constant)
        {
            switch (e.Type)
            {
                case ExprType.Constant:
                    constant += scale * e.Value.Value;
                    return true;
                case ExprType.Parameter:
                    if (e.Value.Frozen)
                    {
                        constant += scale * e.Value.Value;
                        return true;
                    }
                    AddCoeff(coeffs, e.Value, scale);
                    return true;
                case ExprType.Negate:
                    return TryCollectLinear(e.A, -scale, coeffs, ref constant);
                case ExprType.Add:
                    return TryCollectLinear(e.A, scale, coeffs, ref constant)
                        && TryCollectLinear(e.B, scale, coeffs, ref constant);
                case ExprType.Subtract:
                    return TryCollectLinear(e.A, scale, coeffs, ref constant)
                        && TryCollectLinear(e.B, -scale, coeffs, ref constant);
                case ExprType.Multiply:
                    if (e.A.Type == ExprType.Constant)
                        return TryCollectLinear(e.B, scale * e.A.Value.Value, coeffs, ref constant);
                    if (e.B.Type == ExprType.Constant)
                        return TryCollectLinear(e.A, scale * e.B.Value.Value, coeffs, ref constant);
                    return false;
                case ExprType.Divide:
                    if (e.B.Type == ExprType.Constant && e.B.Value.Value != 0.0)
                        return TryCollectLinear(e.A, scale / e.B.Value.Value, coeffs, ref constant);
                    return false;
                default:
                    return false;
            }
        }

        private static void AddCoeff(Dictionary<Param, double> coeffs, Param p, double scale)
        {
            double prev;
            if (coeffs.TryGetValue(p, out prev))
                coeffs[p] = prev + scale;
            else
                coeffs.Add(p, scale);
        }

        public void UpdateSubstituedParameters()
        {
            if (_dependentParams == null)
                return;
            for (int i = 0; i < _dependentParams.Count; ++i)
                _dependentParams[i].Update();
        }

        private static Expr[] SolveSingleParamEquations(Expr[] equations)
        {
            List<Expr> newEquations = new List<Expr>();
            newEquations.AddRange(equations);
            bool success = true;
            while (success)
            {
                success = false;
                for (int i = newEquations.Count - 1; i >= 0; --i)
                {
                    Expr e = newEquations[i];
                    HashSet<Param> set = e.GetContainedParams();
                    if (set.Count == 0)
                    {
                        // Drop satisfied constants. Unsatisfied constants stay so SumOfSquaredErrors
                        // still reflects inconsistency; Newton handles nParams==0 without throwing.
                        if (Math.Abs(e.Evaluate()) < NewtonSolver.CONVERGE_TOLERANCE)
                            newEquations.RemoveAt(i);
                        continue;
                    }
                    if (set.Count == 1)
                    {
                        //Only one unknown, can solve it without need of other equations
                        Param p = set.First();
                        bool s = NewtonSolver.NewtonSolve(e, p);
                        if (s)
                        {
                            newEquations.RemoveAt(i);

                            for (int j = 0; j < newEquations.Count; ++j)
                                newEquations[j] = newEquations[j].FreezeParamAsConstant(p).FoldConstants();
                            success = true;
                        }
                        // Leave a stubborn one-parameter row for dense Newton.
                    }
                }
            }
            return newEquations.ToArray();
        }

        public EquationContainer(IList<Expr> equations, HashSet<Param> dragged = null, bool applySketchReductions = true)
        {
            _equations = new Expr[equations.Count];
            for (int i = 0; i < equations.Count; ++i)
                _equations[i] = new Expr(equations[i]).FoldConstants();

            if (applySketchReductions)
            {
                _equations = Substitute(_equations, out _dependentParams, dragged);
                _equations = SolveSingleParamEquations(_equations);
            }
            else
            {
                _dependentParams = new List<DependentParam>();
            }


            Dictionary<Param, int> buffer = new Dictionary<Param, int>();
            for (int i = 0; i < _equations.Length; ++i)
            {
                Expr c = _equations[i];
                foreach (Param p in c)
                {
                    if (p.Frozen)
                        continue;
                    if (!buffer.ContainsKey(p))
                        buffer.Add(p, buffer.Count);
                }
            }

            _params = new Param[buffer.Count];
            foreach (var v in buffer)
                _params[v.Value] = v.Key;

            _jacobianColumns = new int[_equations.Length][];
            _jacobianColumnSet = new HashSet<int>[_equations.Length];
            for (int i = 0; i < _equations.Length; ++i)
            {
                var paramSet = _equations[i].GetContainedParams();
                var cols = new List<int>(paramSet.Count);
                foreach (Param p in paramSet)
                {
                    int j;
                    if (buffer.TryGetValue(p, out j))
                        cols.Add(j);
                }
                _jacobianColumns[i] = cols.ToArray();
                _jacobianColumnSet[i] = new HashSet<int>(cols);
            }

            _equationCode = new ExprProgram[_equations.Length];
            for (int i = 0; i < _equations.Length; i++)
                _equationCode[i] = ExprProgram.From(_equations[i]);
        }

        public double Evaluate(int equationIndex)
        {
            return _equationCode[equationIndex].Evaluate();
        }

        public double EvaluatJacobian(int equationIndex, int paramIndex)
        {
            if (!_jacobianColumnSet[equationIndex].Contains(paramIndex))
                return 0.0;
            return _equations[equationIndex].EvaluatePartialDerivative(_params[paramIndex]);
        }

        public Param GetParameter(int paramIndex)
        {
            return _params[paramIndex];
        }

        public double SumOfSquaredErrors()
        {
            double error = 0;
            for (int i = 0; i < _equationCode.Length; ++i)
            {
                double e = _equationCode[i].Evaluate();
                error += e * e;
            }
            return error;
        }

        public int NumNonZerosInRow(int i)
        {
            return _jacobianColumns[i].Length;
        }

        public void EvaluateJacobianRow(int rowIndex, ref int indexer, IList<double> values, IList<int> columnIndices)
        {
            int[] cols = _jacobianColumns[rowIndex];
            Expr e = _equations[rowIndex];
            for (int k = 0; k < cols.Length; k++)
            {
                int c = cols[k];
                values[indexer] = e.EvaluatePartialDerivative(_params[c]);
                columnIndices[indexer] = c;
                ++indexer;
            }
        }
    }
}