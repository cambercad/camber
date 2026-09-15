using System.Collections.Generic;
using GeoCore;


namespace NURBS
{
    
    public class BSplineCurveBounded : BSplineCurve
    {
        private double _start; public double Start { get { return _start; } set { _start = value; } }
        private double _end; public double End { get { return _end; } set { _end = value; } }


        public BSplineCurveBounded(int degree, List<Vec3D> controlPoints, List<double> knots, bool closedCurve = false, double start = 0, double end = 1)
            : base(degree, controlPoints, knots, closedCurve)
        {
            _start = start;
            _end = end;
        }
        public BSplineCurveBounded(int degree, List<Vec3D> controlPoints, List<double> weights, List<double> knots, bool closedCurve = false, double start = 0, double end = 1)
            : base(degree, controlPoints, weights, knots, closedCurve)
        {
            _start = start;
            _end = end;
        }
        public BSplineCurveBounded(int degree, Vec3D[] controlPoints, double[] knots, bool closedCurve = false, double start = 0, double end = 1)
            : base(degree, controlPoints, knots, closedCurve)
        {
            _start = start;
            _end = end;
        }

        public BSplineCurveBounded(int degree, Vec3D[] controlPoints, double[] weights, double[] knots, bool closedCurve = false, double start = 0, double end = 1)
            : base(degree, controlPoints, weights, knots, closedCurve)
        {
            _start = start;
            _end = end;
        }

        public BSplineCurveBounded(int degree, Vec4D[] controlPoints, double[] knots, bool closedCurve = false, double start = 0, double end = 1)
            : base(degree, controlPoints, knots, closedCurve)
        {
            _start = start;
            _end = end;
        }

        //public override void Split(double param, out BSplineCurve lower, out BSplineCurve upper, bool init = true)
        //{
        //    BSplineCurve c = ExtractRange(_start, _end, false);
        //    //Split((param - _start) / (_end - _start), c.Degree, c.Knots, c.ControlPoints, c.ClosedCurve, out lower, out upper, init);
        //    c.SplitInternal(param, out lower, out upper, init);
        //    //BSplineCurve curve = this;
        //    //if (_start != 0)
        //    //{
        //    //    ExtractRange();
        //    //    BSplineCurve.Split(_start, _degree, _knots, _controlPoints, _closedCurve, out lower, out upper, false);
        //    //    curve = upper;
        //    //}
        //    //if (_end != 1)
        //    //{
        //    //    double split = 
        //    //}
        //}

        //public override Vector3d Evaluate(double u)
        //{
        //    return base.Evaluate(_start + u * (_end - _start));
        //}

        //protected override Vector4d EvaluateH(double u)
        //{
        //    return base.EvaluateH(_start + u * (_end - _start));
        //}

        public BSplineCurve GetNativeParamRangeCurve()
        {
            BSplineCurve c = ExtractRange(_start, _end, false);
            return c;
        }
    }
}
