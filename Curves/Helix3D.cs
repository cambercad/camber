using GeoCore;

namespace Curves
{
    //Starts on x axis
    public class Helix3D : Curve3D
    {
        private Vec3D _center;
        private Vec3D _x;
        private Vec3D _y;
        private double _zAdvancementPerRevolution;
        private double _numRevolutions;
        private double _radius;
        private double _angleSign;

        public Vec3D Center { get { return _center; } }
        public double Radius { get { return _radius; } }
        public double ZAdvancementPerRevolution { get { return _zAdvancementPerRevolution; } }
        public double NumRevolutions { get { return _numRevolutions; } }
        public Vec3D ZAxis { get { return Vec3DOps.Cross(_x, _y).Normalized(); } }

        public Vec3D X { get { return _x; } }
        public Vec3D Y { get { return _y; } }

        protected Helix3D(string name = null) : base(name) { }

        public Helix3D(Vec3D center, Vec3D x, Vec3D y, double radius, double zAdvancementPerRevolution, double numRevolutions, bool rightHanded = true, string name = null)
            : base(name)
        {
            _center = center;
            _x = x.Normalized();
            _y = y.Normalized();
            _radius = radius;
            _zAdvancementPerRevolution = zAdvancementPerRevolution;
            _numRevolutions = numRevolutions;

            _angleSign = rightHanded ? 1.0 : -1.0;
        }

        //public Helix3D(Vec3D center, Vec3D x, Vec3D y, double zAdvancementPerRevolution, double numRevolutions)
        //{
        //    _center = center;
        //    _radius = x.Length();
        //    _x = x;
        //    _y = y.Normalized() * _radius;
        //    _zAdvancementPerRevolution = zAdvancementPerRevolution;
        //    _numRevolutions = numRevolutions;
        //}

        public Helix3D(Helix3D h) : base(h.Name)
        {
            _center = h._center;
            _x = h._x;
            _y = h._y;
            _radius = h._radius;
            _zAdvancementPerRevolution = h._zAdvancementPerRevolution;
            _numRevolutions = h._numRevolutions;
        }

        public override CurveVertex3D Evaluate(double uniform)
        {
            double angle = 2 * Math.PI * uniform * _numRevolutions * _angleSign;
            double cos = Math.Cos(angle);
            double sin = Math.Sin(angle);
            double zOffset = uniform * _zAdvancementPerRevolution * _numRevolutions;

            // Position: circular motion in XY plane at radius, plus linear Z advancement
            Vec3D position = _center + (cos * _x + sin * _y) * _radius + zOffset * ZAxis;
            
            // Derivative of position with respect to uniform parameter
            // d(angle)/d(uniform) = 2π * numRevolutions
            double angularVelocity = 2 * Math.PI * _numRevolutions;
            // Tangent: derivative of circular motion (scaled by radius) plus Z advancement rate
            Vec3D tangent = ((-sin * _x + cos * _y) * _radius * angularVelocity + _zAdvancementPerRevolution * _numRevolutions * ZAxis).Normalized();
            Vec3D up = Vec3DOps.Cross(tangent, ZAxis).Normalized();

            return new CurveVertex3D(position, tangent, up, uniform);
        }

        public override List<CurveVertex3D> Tessellate(double tolerance)
        {
            double totalAngle = 2 * Math.PI * _numRevolutions;
            int numSteps = Circle3D.NumberOfCirclePoints(tolerance, _radius, 2, totalAngle);
            List<CurveVertex3D> points = new List<CurveVertex3D>(numSteps);
            
            double scaling = 1.0 / (numSteps - 1);
            for (int i = 0; i < numSteps; ++i)
                points.Add(Evaluate(i * scaling));

            return points;
        }

        public Curve3D GetCopy() { return new Helix3D(this); }
    }
}
