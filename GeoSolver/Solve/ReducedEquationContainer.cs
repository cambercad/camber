namespace GeoSolver
{
    public class ReducedEquationContainer : IEquationContainer
    {
        private EquationContainer _container;
        private bool[] _paramEnabled;
        private int _numParameters;
        private int[] _paramMap;

        public ReducedEquationContainer(EquationContainer container)
        {
            _container = container;
            _numParameters = container.NumParameters;

            _paramEnabled = new bool[_numParameters];
            _paramMap = new int[_numParameters];
            for (int i = 0; i < _numParameters; ++i)
            {
                _paramEnabled[i] = true;
                _paramMap[i] = i;
            }
        }

        public int NumParameters { get { return _numParameters; } }

        public int NumEquations { get { return _container.NumEquations; } }

        public double Evaluate(int i) { return _container.Evaluate(i); }

        public double EvaluatJacobian(int i, int j)
        {
            return _container.EvaluatJacobian(i, _paramMap[j]);
        }

        public Param GetParameter(int i)
        {
            return _container.GetParameter(_paramMap[i]);
        }

        public bool this[int index]
        {
            get { return _paramEnabled[index]; }
            set
            {
                if (_paramEnabled[index] != value)
                {
                    _paramEnabled[index] = value;
                    UpdateParamMap();
                }
            }
        }

        public int OriginalNumParameters { get { return _container.NumParameters; } }
        public void UpdateParamMap()
        {
            int indexer = 0;
            for (int i = 0; i < _paramMap.Length; ++i)
            {
                bool b = _paramEnabled[i];
                if (_paramEnabled[i])
                {
                    _paramMap[indexer] = i;
                    ++indexer;
                }
            }
            _numParameters = indexer;
        }

        public int NumNonZerosInRow(int i)
        {
            throw new System.NotImplementedException();
        }

        public void EvaluateJacobianRow(int rowIndex, ref int indexer, IList<double> values, IList<int> columnIndices)
        {
            throw new System.NotImplementedException();
        }
    }
}