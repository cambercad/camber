using GeoCore;

namespace Curves
{
    /// <summary>
    /// A 3D arc (partial circle) defined by center, radius, start angle, and sweep angle.
    /// Starts on the x-axis direction at startAngle and sweeps sweepAngle radians.
    /// </summary>
    public class Arc3D : Curve3D
    {
        private Vec3D _center;
        private Vec3D _x;
        private Vec3D _y;
        private double _radius;
        private double _startAngle;
        private double _sweepAngle;

        public Vec3D Center { get { return _center; } }
        public double Radius { get { return _radius; } }
        public Vec3D Normal { get { return Vec3DOps.Cross(_x, _y).Normalized(); } }
        public double StartAngle { get { return _startAngle; } }
        public double SweepAngle { get { return _sweepAngle; } }

        protected Arc3D(string name = null) : base(name) { }

        /// <summary>
        /// Creates an arc in 3D space.
        /// </summary>
        /// <param name="center">Center of the arc</param>
        /// <param name="x">X-axis direction of the arc plane (will be normalized)</param>
        /// <param name="y">Y-axis direction of the arc plane (will be normalized)</param>
        /// <param name="radius">Radius of the arc</param>
        /// <param name="startAngle">Starting angle in radians (0 = along x-axis)</param>
        /// <param name="sweepAngle">Sweep angle in radians (positive = CCW when viewed from normal direction)</param>
        /// <param name="name">Optional name for the curve</param>
        public Arc3D(Vec3D center, Vec3D x, Vec3D y, double radius, double startAngle, double sweepAngle, string name = null) : base(name)
        {
            _center = center;
            _x = x.Normalized() * radius;
            _y = y.Normalized() * radius;
            _radius = radius;
            _startAngle = startAngle;
            _sweepAngle = sweepAngle;
        }

        public Arc3D(Arc3D a) : base(a.Name)
        {
            _center = a._center;
            _x = a._x;
            _y = a._y;
            _radius = a._radius;
            _startAngle = a._startAngle;
            _sweepAngle = a._sweepAngle;
        }

        /// <summary>
        /// Creates an arc from three points: start, a point on the arc, and end.
        /// The arc passes through all three points in order.
        /// </summary>
        /// <param name="start">Start point of the arc</param>
        /// <param name="pointOnArc">A point on the arc between start and end</param>
        /// <param name="end">End point of the arc</param>
        /// <param name="name">Optional name for the curve</param>
        public Arc3D(Vec3D start, Vec3D pointOnArc, Vec3D end, string name = null) : base(name)
        {
            // Compute vectors between points
            Vec3D ab = pointOnArc - start;
            Vec3D bc = end - pointOnArc;
            Vec3D ca = start - end;

            // Compute the normal of the plane containing the three points
            Vec3D normal = Vec3DOps.Cross(ab, bc);
            if (normal.LengthSquared() < 1e-20)
                throw new ArgumentException("The three points are collinear and do not define an arc");
            normal = normal.Normalized();

            // Compute circumcenter using barycentric coordinates
            double a = bc.LengthSquared();
            double b = ca.LengthSquared();
            double c = ab.LengthSquared();

            double u = a * (b + c - a);
            double v = b * (c + a - b);
            double w = c * (a + b - c);
            double sum = u + v + w;

            _center = (u * start + v * pointOnArc + w * end) / sum;
            _radius = (start - _center).Length();

            // Set up the local coordinate system
            // _x points from center to start, so startAngle = 0
            Vec3D xDir = (start - _center).Normalized();
            Vec3D yDir = Vec3DOps.Cross(normal, xDir).Normalized();

            _x = xDir * _radius;
            _y = yDir * _radius;
            _startAngle = 0;

            // Compute angle to end point
            Vec3D toEnd = end - _center;
            double endX = Vec3DOps.Dot(toEnd, xDir);
            double endY = Vec3DOps.Dot(toEnd, yDir);
            double endAngle = Math.Atan2(endY, endX);

            // Compute angle to pointOnArc
            Vec3D toMid = pointOnArc - _center;
            double midX = Vec3DOps.Dot(toMid, xDir);
            double midY = Vec3DOps.Dot(toMid, yDir);
            double midAngle = Math.Atan2(midY, midX);

            // Determine sweep direction based on whether pointOnArc is on the short or long arc
            // Normalize angles to [0, 2*PI)
            if (endAngle < 0) endAngle += 2.0 * Math.PI;
            if (midAngle < 0) midAngle += 2.0 * Math.PI;

            // Check if midAngle is between 0 and endAngle (CCW direction)
            bool midOnCCW = (endAngle > 0) ? (midAngle > 0 && midAngle < endAngle) : false;
            
            if (midOnCCW)
            {
                // CCW sweep from start to end
                _sweepAngle = endAngle;
            }
            else
            {
                // CW sweep (negative) or going the long way CCW
                if (endAngle > 0)
                    _sweepAngle = endAngle - 2.0 * Math.PI;
                else
                    _sweepAngle = endAngle;
            }
        }

        public override CurveVertex3D Evaluate(double uniform)
        {
            double angle = _startAngle + _sweepAngle * uniform;
            double cos = Math.Cos(angle);
            double sin = Math.Sin(angle);

            Vec3D position = _center + cos * _x + sin * _y;
            
            // Tangent direction (derivative of position with respect to angle)
            // If sweepAngle > 0, tangent is in CCW direction
            Vec3D tangentDir = -sin * _x + cos * _y;
            if (_sweepAngle < 0)
                tangentDir = -tangentDir;
            Vec3D tangent = tangentDir.Normalized();
            
            Vec3D up = Vec3DOps.Cross(tangent, Normal);

            return new CurveVertex3D(position, tangent, up, uniform);
        }

        public override List<CurveVertex3D> Tessellate(double tolerance)
        {
            int numSteps = Circle3D.NumberOfCirclePoints(tolerance, _radius, 2, Math.Abs(_sweepAngle));
            List<CurveVertex3D> points = new List<CurveVertex3D>(numSteps);
            
            double scaling = 1.0 / (numSteps - 1);
            for (int i = 0; i < numSteps; ++i)
                points.Add(Evaluate(i * scaling));

            return points;
        }

        public Curve3D GetCopy() { return new Arc3D(this); }

        /// <summary>
        /// Returns defining points: [start, midpoint, end]
        /// </summary>
        public List<Vec3D> ToPoints()
        {
            return new List<Vec3D> { Start, Evaluate(0.5).Origin, End };
        }

        /// <summary>
        /// Creates an Arc3D from defining points: [start, midpoint, end]
        /// </summary>
        public static Arc3D FromPoints(List<Vec3D> points, string name = null)
        {
            if (points == null || points.Count < 3)
                throw new ArgumentException("Arc3D requires 3 points: [start, midpoint, end]");
            return new Arc3D(points[0], points[1], points[2], name);
        }
    }
}

