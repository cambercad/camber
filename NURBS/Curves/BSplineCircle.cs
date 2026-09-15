using GeoCore;

using System;

namespace NURBS
{
    //http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/NURBS/RB-circles.html
    
    public class BSplineCircle : BSplineCurveBounded
    {
        protected Vec3D _center;
        protected Vec3D _axis;
        protected Vec3D _centerToStart;
        protected Vec3D _y;

        protected double _radius;


        public BSplineCircle(Vec3D center, Vec3D axis, double radius, Vec3D centerToStart, double start = 0, double end = 1)
            : base(2, CircleControlPoints(4, center, axis, radius, centerToStart), CircleKnots(4), false, start, end)
        {
            _center = center;
            _radius = radius;

            axis.Normalize();
            _axis = axis;
            _y = Vec3DOps.Cross(axis, centerToStart);
            _y.Normalize();
            _centerToStart = Vec3DOps.Cross(_y, axis);
            _centerToStart.Normalize();
        }

        //http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/NURBS/RB-circles.html
        /*public static double[] CircleWeights(int numPoints)
        {
            double w = Math.Sin((numPoints - 2) * Math.PI / numPoints * 0.5);

            double[] weights = new double[2 * numPoints + 1];
            for (int i = 0; i < weights.Length; ++i)
                if (i % 2 == 0)
                    weights[i] = 1;
                else
                    weights[i] = w;

            return weights;
        }*/

        public static Vec4D[] CircleControlPoints(int numPoints, Vec3D center, Vec3D axis, double radius, Vec3D centerToStart)
        {
            Vec3D y = Vec3DOps.Cross(axis, centerToStart);
            y.Normalize();
            Vec3D x = Vec3DOps.Cross(y, axis);
            x.Normalize();

            Vec4D[] points = new Vec4D[2 * numPoints + 1];
            for (int i = 0; i < numPoints; ++i)
            {
                double angle = (Math.PI * 2 * i) / numPoints;
                points[2 * i] = new Vec4D(center + radius * (Math.Cos(angle) * x + Math.Sin(angle) * y), 1);
            }

            double innerAngleSum = (numPoints - 2) * Math.PI;
            double alpha = 0.5 * innerAngleSum / numPoints;

            double d = radius * Math.Sin(alpha);
            double l = Math.Sqrt(radius * radius - d * d);

            double weight = Math.Sin((numPoints - 2) * Math.PI / numPoints * 0.5);

            double offset = l / Math.Tan(alpha);
            for (int i = 0; i < numPoints; ++i)
            {
                double angle = (Math.PI * 2 * (i + 0.5)) / numPoints;
                points[2 * i + 1] = new Vec4D(weight * (center + (d + offset) * (Math.Cos(angle) * x + Math.Sin(angle) * y)), weight);
            }

            points[points.Length - 1] = points[0];

            //Form2.Draw(points, 1000, 1000, 20);

            return points;
        }

        public static double[] CircleKnots(int numPoints)
        {
            double[] knots = new double[6 + 2 * (numPoints - 1)];
            knots[0] = 0;
            knots[1] = 0;
            knots[2] = 0;

            for (int i = 1; i < numPoints; ++i)
            {
                double v = (double)i / numPoints;
                knots[2 * (i - 1) + 3] = v;
                knots[2 * (i - 1) + 3 + 1] = v;
            }

            knots[knots.Length - 3] = 1.0;
            knots[knots.Length - 2] = 1.0;
            knots[knots.Length - 1] = 1.0;

            return knots;
        }
    }
}
