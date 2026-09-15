using GeoCore;

namespace NURBS
{
    public class BSplineSurfaceOfRevolution : BSplineSurfaceBounded
    {
        protected Vec3D _pointOnAxis;
        protected Vec3D _axis;
        protected Vec3D _zeroAngleDirection;
        //protected Vector3d _y;

        protected BSplineCurve _radiusCurve;


        public BSplineSurfaceOfRevolution(Vec3D pointOnAxis, Vec3D axis, BSplineCurveBounded radiusCurve, Vec3D zeroAngleDirection)
            : this(pointOnAxis, axis, radiusCurve.GetNativeParamRangeCurve(), zeroAngleDirection)
        {        }

        public BSplineSurfaceOfRevolution(Vec3D pointOnAxis, Vec3D axis, BSplineCurve radiusCurve, Vec3D zeroAngleDirection)
            : base(radiusCurve.Degree, 2, RevolutionControlPoints(4, pointOnAxis, axis, radiusCurve.ControlPoints, zeroAngleDirection), radiusCurve.Knots, BSplineCircle.CircleKnots(4))
        {
            _pointOnAxis = pointOnAxis;
            _axis = axis;
            _zeroAngleDirection = zeroAngleDirection;
            //_y = 0;
            _radiusCurve = radiusCurve;
        }

        public static Vec4D[][] RevolutionControlPoints(int numCirclePoints, Vec3D center, Vec3D axis, Vec4D[] controlPoints, Vec3D centerToStart)
        {
            Vec4D[][] points = new Vec4D[controlPoints.Length][];

            for (int i = 0; i < controlPoints.Length; ++i)
            {
                Vec4D cp = controlPoints[i];
                Vec3D v = DeBoor.Project(cp);
                Vec3D c = GeometricAlgorithms.ProjectPointOntoLine(v, center, axis);
                double r = (c - v).Length();
                
                Vec4D[] pts = BSplineCircle.CircleControlPoints(numCirclePoints, c, axis, r, centerToStart);
                for (int j = 0; j < pts.Length; ++j)
                    pts[j] = cp.W * pts[j];
                points[i] = pts;
            }

            return points;
        }

        /*public static double[][] RevolutionWeights(int numCirclePoints, double[] axialWeights)
        {
            double[][] weights = new double[axialWeights.Length][];

            for (int i = 0; i < axialWeights.Length; ++i)
            {
                double w = axialWeights[i];
                double[] circle = BSplineCircle.CircleWeights(numCirclePoints);
                for (int j = 0; j < circle.Length; ++j)
                    circle[j] *= w;

                weights[i] = circle;
            }

            return weights;
        }*/
    }
}
