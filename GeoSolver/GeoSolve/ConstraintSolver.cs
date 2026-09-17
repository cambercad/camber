namespace GeoSolver
{
    public class ConstraintSolver
    {       
        private static int GetNodeIndex(Param[] parameters, Param p)
        {
            for (int i = 0; i < parameters.Length; ++i)
            {
                if (parameters[i] == p)
                    return i;
            }
            return -1;
        }

        private static bool MakeEqual(List<int> map, Param a, Param b, Param[] parameters)
        {
            int pIndex = GetNodeIndex(parameters, a);
            int nIndex = GetNodeIndex(parameters, b);

            if (pIndex == -1 || nIndex == -1)
                return false;

            double v = 0.5 * (a.Value + b.Value);
            a.Value = v;
            b.Value = v;

            while (map[pIndex] >= 0)
                pIndex = map[pIndex];

            while (map[nIndex] >= 0)
                nIndex = map[nIndex];

            int index = map.Count;
            map[pIndex] = index;
            map[nIndex] = index;
            map.Add(-1);

            return true;
        }

        public static IList<List<Param>> BuildEqualityMap(ref Param[] parameters, ref IList<IBaseEquation> constraints)
        {
            List<int> map = new List<int>();
            for (int i = 0; i < parameters.Length; ++i)
                map.Add(-1);

            List<IBaseEquation> remainingConstraints = new List<IBaseEquation>();
            for (int i = 0; i < constraints.Count; ++i)
            {
                IBaseEquation constraint = constraints[i];
                PointOnPoint2d c = constraint as PointOnPoint2d;
                bool success = false;
                if (c != null)
                {
                    success = true;
                    CVec2D a = c.Point1 as CVec2D;
                    CVec2D b = c.Point2 as CVec2D;
                    if (a != null && b != null && a.Ex.Type == ExprType.Parameter && a.Ey.Type == ExprType.Parameter)
                    {
                        success = success && MakeEqual(map, a.Ex.Value, b.Ex.Value, parameters);
                        success = success && MakeEqual(map, a.Ey.Value, b.Ey.Value, parameters);
                    }
                    else
                        success = false;
                }

                Horizontal2d h = constraint as Horizontal2d;
                if (h != null)
                {
                    success = MakeEqual(map, h.Line.CStart.Ey.Value, h.Line.CEnd.Ey.Value, parameters);
                }

                Vertical2d v = constraint as Vertical2d;
                if (v != null)
                {
                    success = MakeEqual(map, v.Line.CStart.Ex.Value, v.Line.CEnd.Ex.Value, parameters);
                }

                if (!success)
                    remainingConstraints.Add(constraint);
            }

            constraints = remainingConstraints;

            int indexer = 0;
            for (int i = 0; i < map.Count; ++i)
            {
                if (map[i] == -1)
                    map[i] = --indexer;
            }

            for (int i = 0; i < parameters.Length; ++i)
            {
                int index = map[i];
                while (index >= 0)
                    index = map[index];

                map[i] = -index - 1;
            }

            List<List<Param>> equalityMap = new List<List<Param>>();
            for (int i = 0; i < parameters.Length; ++i)
            {
                int j = map[i];
                while (j >= equalityMap.Count)
                    equalityMap.Add(new List<Param>());
                equalityMap[j].Add(parameters[i]);
            }

            Param[] remainingParameters = new Param[equalityMap.Count];
            for (int i = 0; i < equalityMap.Count; ++i)
                remainingParameters[i] = equalityMap[i][0];
            parameters = remainingParameters;

            return equalityMap;
        }

        public static double Solve(IList<IBaseEquation> cons, HashSet<Param> draggedParams = null, double scaling = 1)
        {
            return SolveDetailed(cons, draggedParams, scaling).SumOfSquaredErrors;
        }

        public static double Solve(IList<IBaseEquation> cons, HashSet<Param> draggedParams, double scaling, out int numParams, out int numConstraints)
        {
            SolveResult result = SolveDetailed(cons, draggedParams, scaling);
            numParams = result.NumParameters;
            numConstraints = result.NumEquations;
            return result.SumOfSquaredErrors;
        }

        public static SolveResult SolveDetailed(
            IList<IBaseEquation> cons,
            HashSet<Param> draggedParams = null,
            double scaling = 1,
            bool allowSoftOverconstrained = false,
            bool applySketchReductions = true,
            int maxNewtonIterations = 0)
        {
            List<Expr> equations = new List<Expr>();
            HashSet<Param> nonNegative = null;

            for (int i = 0; i < cons.Count; ++i)
            {
                IBaseEquation c = cons[i];
                if (c is ContactHalfSpace3d contact)
                {
                    if (nonNegative == null)
                        nonNegative = new HashSet<Param>();
                    nonNegative.Add(contact.Lambda);
                }
                if (c is TangentCircularCircular2d tangent &&
                    tangent.TryGenerateJoinedArcEquation(cons, equations))
                    continue;
                c.GenerateEquations(equations, scaling);
            }

            EquationContainer e = new EquationContainer(equations, draggedParams, applySketchReductions);
            bool softOver = allowSoftOverconstrained || (nonNegative != null && nonNegative.Count > 0);
            SolveResult result = NewtonSolver.NewtonSolveDetailed(
                e, draggedParams, nonNegative, scaling, null, softOver, maxNewtonIterations);
            e.UpdateSubstituedParameters();

            // Final clamp / report SSE from live residuals
            if (nonNegative != null)
            {
                foreach (Param p in nonNegative)
                {
                    if (p.Value < 0)
                        p.Value = 0;
                }
            }

            return new SolveResult(result.Converged, e.SumOfSquaredErrors(), e.NumParameters, e.NumEquations, result.Message);
        }
    }
}
