//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using DX11Core;

//namespace NURBS
//{
//    public class BSplineDerivative
//    {
//        private BSplineCurve _func;
//        private BSplineCurve _deriv1;
//        private BSplineCurve _deriv2;


//        public BSplineDerivative(BSplineCurve func, BSplineCurve deriv1, BSplineCurve deriv2)
//        {
//            _func = func;
//            _deriv1 = deriv1;
//            _deriv2 = deriv2;
//        }

//        public Vector3d EvaluateDU(double u)
//        {
//            Vector4d f = _func.EvaluateH(u);
//            Vector4d d1 = _deriv1.EvaluateH(u);

//            return XYZ(d1) / f.W - XYZ(f) * d1.W / (f.W * f.W);
//        }

//        public Vector3d EvaluateDDU(double u)
//        {
//            Vector4d f = _func.EvaluateH(u);
//            Vector4d d1 = _deriv1.EvaluateH(u);
//            Vector4d d2 = _deriv2.EvaluateH(u);

//            double fw2 = f.W * f.W;

//            return ((XYZ(d2) / f.W - XYZ(f) * (d2.W / fw2)) - ((XYZ(d1) / fw2 - XYZ(f) * (d1.W / (fw2 * f.W))) * 2 * d1.W));
//            //return (((XYZ(d2) * f.W - XYZ(f) * d2.W) * f.W * f.W) - ((XYZ(d1) * f.W - XYZ(f) * d1.W) * 2 * f.W * d1.W)) / (f.W * f.W * f.W * f.W);
//        }

//        //public Vector3d EvaluateDDU(double u)
//        //{
//        //    Vector4d f = _func.EvaluateH(u);
//        //    Vector4d d1 = _deriv1.EvaluateH(u);
//        //    Vector4d d2 = _deriv2.EvaluateH(u);

//        //    return (XYZ(d2) * f.W * f.W - 2 * XYZ(d1) * f.W * d1.W + XYZ(f) * (2 * d1.W * d1.W - f.W * d2.W)) / (f.W * f.W * f.W);
//        //}

//        private Vector3d XYZ(Vector4d v) { return new Vector3d(v.X, v.Y, v.Z); }
//    }
//}
