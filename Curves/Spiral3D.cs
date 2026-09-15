using GeoCore;

namespace Curves
{
    /// <summary>
    /// Archimedean spiral in 3D: radius grows linearly from <see cref="StartRadius"/> to <see cref="EndRadius"/>
    /// while sweeping <see cref="NumRevolutions"/> in the plane of X and Y, with optional advancement along
    /// <see cref="ZAxis"/> (set <see cref="ZAdvancementPerRevolution"/> to 0 for a planar spiral).
    /// Starts on the X axis at <see cref="StartRadius"/>.
    /// </summary>
    public class Spiral3D : Curve3D
    {
        private Vec3D _center;
        private Vec3D _x;
        private Vec3D _y;
        private double _startRadius;
        private double _endRadius;
        private double _zAdvancementPerRevolution;
        private double _numRevolutions;
        private double _angleSign;

        public Vec3D Center { get { return _center; } }
        public double StartRadius { get { return _startRadius; } }
        public double EndRadius { get { return _endRadius; } }
        public double ZAdvancementPerRevolution { get { return _zAdvancementPerRevolution; } }
        public double NumRevolutions { get { return _numRevolutions; } }
        public Vec3D ZAxis { get { return Vec3DOps.Cross(_x, _y).Normalized(); } }

        public Vec3D X { get { return _x; } }
        public Vec3D Y { get { return _y; } }

        protected Spiral3D(string name = null) : base(name) { }

        public Spiral3D(
            Vec3D center,
            Vec3D x,
            Vec3D y,
            double startRadius,
            double endRadius,
            double zAdvancementPerRevolution,
            double numRevolutions,
            bool rightHanded = true,
            string name = null)
            : base(name)
        {
            _center = center;
            _x = x.Normalized();
            _y = y.Normalized();
            _startRadius = startRadius;
            _endRadius = endRadius;
            _zAdvancementPerRevolution = zAdvancementPerRevolution;
            _numRevolutions = numRevolutions;
            _angleSign = rightHanded ? 1.0 : -1.0;
        }

        public Spiral3D(Spiral3D s) : base(s.Name)
        {
            _center = s._center;
            _x = s._x;
            _y = s._y;
            _startRadius = s._startRadius;
            _endRadius = s._endRadius;
            _zAdvancementPerRevolution = s._zAdvancementPerRevolution;
            _numRevolutions = s._numRevolutions;
            _angleSign = s._angleSign;
        }

        public override CurveVertex3D Evaluate(double uniform)
        {
            double angle = 2 * Math.PI * uniform * _numRevolutions * _angleSign;
            double cos = Math.Cos(angle);
            double sin = Math.Sin(angle);
            double radius = _startRadius + uniform * (_endRadius - _startRadius);
            double zOffset = uniform * _zAdvancementPerRevolution * _numRevolutions;

            Vec3D radial = cos * _x + sin * _y;
            Vec3D position = _center + radial * radius + zOffset * ZAxis;

            double radiusDelta = _endRadius - _startRadius;
            double angularVelocity = 2 * Math.PI * _numRevolutions;
            Vec3D tangential = -sin * _x + cos * _y;
            Vec3D tangent = (
                radiusDelta * radial
                + radius * angularVelocity * tangential
                + _zAdvancementPerRevolution * _numRevolutions * ZAxis).Normalized();
            Vec3D up = Vec3DOps.Cross(tangent, ZAxis).Normalized();

            return new CurveVertex3D(position, tangent, up, uniform);
        }

        public override List<CurveVertex3D> Tessellate(double tolerance)
        {
            double totalAngle = 2 * Math.PI * _numRevolutions;
            double maxRadius = Math.Max(_startRadius, _endRadius);
            int numStepsAngle = Circle3D.NumberOfCirclePoints(tolerance, maxRadius, 2, totalAngle);
            int numStepsRadial = Math.Max(2, (int)Math.Ceiling(Math.Abs(_endRadius - _startRadius) / tolerance) + 1);
            int numSteps = Math.Max(numStepsAngle, numStepsRadial);

            List<CurveVertex3D> points = new List<CurveVertex3D>(numSteps);

            double scaling = 1.0 / (numSteps - 1);
            for (int i = 0; i < numSteps; ++i)
                points.Add(Evaluate(i * scaling));

            return points;
        }

        public Curve3D GetCopy() { return new Spiral3D(this); }
    }
}
