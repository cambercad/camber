using GeoCore;

namespace Curves
{
    //Starts on x axis
    public class Circle3D : Curve3D
    {
        private Vec3D _center;
        private Vec3D _x;
        private Vec3D _y;
        private double _radius;

        public Vec3D Center { get { return _center; } }
        public double Radius { get { return _radius; } }
        public Vec3D Normal { get { return Vec3DOps.Cross(_x, _y).Normalized(); } }

        protected Circle3D(string name = null) : base(name) { }

        public Circle3D(Vec3D center, Vec3D x, Vec3D y, double radius, string name = null) : base(name)
        {
            _center = center;
            _x = x.Normalized() * radius;
            _y = y.Normalized() * radius;
            _radius = radius;
        }

        public Circle3D(Vec3D center, Vec3D x, Vec3D y, string name = null) : base(name)
        {
            _center = center;
            _radius = x.Length();
            _x = x;
            _y = y.Normalized() * _radius;
        }

        public Circle3D(Circle3D c) : base(c.Name)
        {
            _center = c._center;
            _x = c._x;
            _y = c._y;
            _radius = c._radius;
        }

        public override CurveVertex3D Evaluate(double uniform)
        {
            double angle = 2 * Math.PI * uniform;
            double cos = Math.Cos(angle);
            double sin = Math.Sin(angle);

            Vec3D position = _center + cos * _x + sin * _y;
            Vec3D tangent = (-sin * _x + cos * _y).Normalized();
            Vec3D up = Vec3DOps.Cross(tangent, Normal);

            return new CurveVertex3D(position, tangent, up, uniform);
        }

        public override List<CurveVertex3D> Tessellate(double tolerance)
        {
            int numSteps = NumberOfCirclePoints(tolerance, _radius);
            List<CurveVertex3D> points = new List<CurveVertex3D>(numSteps);
            
            double scaling = 1.0 / (numSteps - 1);
            for (int i = 0; i < numSteps; ++i)
                points.Add(Evaluate(i * scaling));

            return points;
        }

        public static int NumberOfCirclePoints(double maxDeviation, double radius, int minNumSegments = 2, double angle = 2 * Math.PI)
        {
            double maxAngleStep = 2 * Math.Acos(1 - maxDeviation / radius);
            int numSteps = Math.Max(minNumSegments, (int)(angle / maxAngleStep) + 1);
            return numSteps;
        }

        public Curve3D GetCopy() { return new Circle3D(this); }

        /// <summary>
        /// Returns defining points: [center, zeroAnglePoint, quarterPoint]
        /// The zero angle point is at angle 0 (along the X axis of the circle).
        /// The quarter point is at angle 90° (along the Y axis of the circle).
        /// These three points define the center, radius, and plane orientation.
        /// </summary>
        public List<Vec3D> ToPoints()
        {
            // center, point at angle 0, point at angle 90°
            return new List<Vec3D> { _center, _center + _x, _center + _y };
        }

        /// <summary>
        /// Creates a Circle3D from defining points: [center, zeroAnglePoint, quarterPoint]
        /// The radius is derived from the distance between center and zeroAnglePoint.
        /// The plane is defined by the center, zeroAnglePoint (X direction), and quarterPoint (Y direction).
        /// </summary>
        public static Circle3D FromPoints(List<Vec3D> points, string name = null)
        {
            if (points == null || points.Count < 3)
                throw new ArgumentException("Circle3D requires 3 points: [center, zeroAnglePoint, quarterPoint]");
            Vec3D center = points[0];
            Vec3D x = points[1] - center;
            Vec3D y = points[2] - center;
            return new Circle3D(center, x, y, name);
        }
    }
}
