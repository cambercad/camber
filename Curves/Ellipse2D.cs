#nullable disable

using GeoCore;

namespace Curves
{
    public class Ellipse2D : Curve2D
    {
        private Vec2D _center;
        private Vec2D _majorAxis;
        private double _minorAxisLength;
        private double _startAngle;
        private double _endAngle;

        public Vec2D Center { get { return _center; } }
        public Vec2D MajorAxis { get { return _majorAxis; } }
        public double MajorAxisLength { get { return _majorAxis.Length(); } }
        public double MinorAxisLength { get { return _minorAxisLength; } }
        public double StartAngle { get { return _startAngle; } }
        public double EndAngle { get { return _endAngle; } }
        public bool IsFullEllipse { get { return Math.Abs(_endAngle - _startAngle - 2.0 * Math.PI) < 1e-10; } }

        protected Ellipse2D() { }

        /// <summary>
        /// Creates a full ellipse
        /// </summary>
        public Ellipse2D(Vec2D center, Vec2D majorAxis, double minorAxisLength)
            : this(center, majorAxis, minorAxisLength, 0, 2.0 * Math.PI)
        {
        }

        /// <summary>
        /// Creates an elliptical arc
        /// </summary>
        public Ellipse2D(Vec2D center, Vec2D majorAxis, double minorAxisLength, double startAngle, double endAngle)
        {
            _center = center;
            _majorAxis = majorAxis;
            _minorAxisLength = minorAxisLength;
            _startAngle = startAngle;
            _endAngle = endAngle;
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
        {
            double angle = _startAngle + uniform * (_endAngle - _startAngle);
            Vec2D point = EvaluateAtAngle(angle);
            Vec2D tangent = EvaluateTangentAtAngle(angle);
            
            if (tangent.LengthSquared() < 1e-10)
                tangent = new Vec2D(1, 0);
            else
                tangent.Normalize();

            Vec2D normal = new Vec2D(-tangent.Y, tangent.X);
            return new CurveVertex2D(point, normal, uniform);
        }

        private Vec2D EvaluateAtAngle(double angle)
        {
            double majorAxisLen = _majorAxis.Length();
            if (majorAxisLen < 1e-10)
                return _center;

            Vec2D majorDir = _majorAxis / majorAxisLen;
            Vec2D minorDir = new Vec2D(-majorDir.Y, majorDir.X);

            double cosAngle = Math.Cos(angle);
            double sinAngle = Math.Sin(angle);

            Vec2D point = _center + majorAxisLen * cosAngle * majorDir + _minorAxisLength * sinAngle * minorDir;
            return point;
        }

        private Vec2D EvaluateTangentAtAngle(double angle)
        {
            double majorAxisLen = _majorAxis.Length();
            if (majorAxisLen < 1e-10)
                return new Vec2D(1, 0);

            Vec2D majorDir = _majorAxis / majorAxisLen;
            Vec2D minorDir = new Vec2D(-majorDir.Y, majorDir.X);

            double cosAngle = Math.Cos(angle);
            double sinAngle = Math.Sin(angle);

            Vec2D tangent = -majorAxisLen * sinAngle * majorDir + _minorAxisLength * cosAngle * minorDir;
            return tangent;
        }

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
        {
            double angleRange = _endAngle - _startAngle;
            double maxRadius = Math.Max(MajorAxisLength, _minorAxisLength);
            
            // Use similar logic as circles for point count
            int numPoints = Math.Max(8, (int)(Math.Abs(angleRange) / (2 * Math.Acos(1 - maxDeviation / maxRadius))) + 1);
            return Tessellate(numPoints);
        }

        public override List<CurveVertex2D> Tessellate(int numPoints)
        {
            List<CurveVertex2D> result = new List<CurveVertex2D>(numPoints);
            double step = 1.0 / (numPoints - 1);

            for (int i = 0; i < numPoints; i++)
            {
                double uniform = i * step;
                result.Add(EvaluateVertex(uniform));
            }

            return result;
        }

        public override double Length()
        {
            // Ramanujan's approximation for ellipse perimeter (or numerical integration for arcs)
            if (IsFullEllipse)
            {
                double a = MajorAxisLength;
                double b = _minorAxisLength;
                double h = Math.Pow((a - b) / (a + b), 2);
                double circumference = Math.PI * (a + b) * (1 + 3 * h / (10 + Math.Sqrt(4 - 3 * h)));
                return circumference;
            }
            else
            {
                // For arcs, use numerical approximation
                int samples = 100;
                double length = 0;
                double step = 1.0 / (samples - 1);
                Vec2D prev = EvaluateVertex(0).Position;

                for (int i = 1; i < samples; i++)
                {
                    Vec2D curr = EvaluateVertex(i * step).Position;
                    length += (curr - prev).Length();
                    prev = curr;
                }

                return length;
            }
        }

        public override Curve2D GetCopy()
        {
            return new Ellipse2D(_center, _majorAxis, _minorAxisLength, _startAngle, _endAngle);
        }

        public override Curve2D Reverse()
        {
            return new Ellipse2D(_center, _majorAxis, _minorAxisLength, _endAngle, _startAngle);
        }

        public override List<Vec2D> ToReferencePoints()
        {
            return new List<Vec2D>() { _center, _center + _majorAxis };
        }

        public override void UpdateFromReferencePoints(List<Vec2D> referencePoints)
        {
            if (referencePoints.Count >= 2)
            {
                _center = referencePoints[0];
                _majorAxis = referencePoints[1] - referencePoints[0];
            }
        }
    }
}

