namespace GeoSolver
{
    public class DependentParam
    {
        private Param _source; //independent
        private Param _dependent;


        private double _scale;
        private double _offset;

        public DependentParam(Param source, Param dependent)
            : this(source, dependent, 1.0, 0.0)
        {
        }

        public DependentParam(Param source, Param dependent, double scale, double offset)
        {
            _source = source;
            _dependent = dependent;
            _scale = scale;
            _offset = offset;
        }

        public void Update()
        {
            _dependent.Value = _scale * _source.Value + _offset;
        }
    }
}