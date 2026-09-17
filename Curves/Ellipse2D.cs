#nullable disable

using GeoCore;

namespace Curves
{
    public class Ellipse2D : Curve2D
    {
        protected Vec2D _center;
        protected Vec2D _majorAxis;
        protected double _minorAxisLength;
        private double _startAngle;
        private double _endAngle;

        public Vec2D Center { get { return _center; } }
        public Vec2D MajorAxis { get { return _majorAxis; } }
        public double MajorAxisLength { get { return _majorAxis.Length(); } }
        public double MinorAxisLength { get { return _minorAxisLength; } }
        public double StartAngle { get { return _startAngle; } }
        public double EndAngle { get { return _endAngle; } }
        public bool IsFullEllipse { get { return Math.Abs(Math.Abs(_endAngle - _startAngle) - 2.0 * Math.PI) < 1e-10; } }

        protected Ellipse2D() { }

        /// <summary>
        /// Creates a full ellipse
        /// </summary>
        public Ellipse2D(Vec2D center, Vec2D majorAxis, double minorAxisLength, CurveFlags flags = CurveFlags.None)
            : this(center, majorAxis, minorAxisLength, 0, 2.0 * Math.PI, flags)
        {
        }

        /// <summary>
        /// Creates an elliptical arc
        /// </summary>
        public Ellipse2D(Vec2D center, Vec2D majorAxis, double minorAxisLength, double startAngle, double endAngle, CurveFlags flags = CurveFlags.None)
        {
            if (!double.IsFinite(center.X) || !double.IsFinite(center.Y) ||
                !double.IsFinite(majorAxis.Length()) || majorAxis.Length() <= 0 ||
                !double.IsFinite(minorAxisLength) || minorAxisLength <= 0 ||
                !double.IsFinite(startAngle) || !double.IsFinite(endAngle) || startAngle == endAngle ||
                Math.Abs(endAngle-startAngle) > 2*Math.PI + 1e-12)
                throw new ArgumentOutOfRangeException(nameof(minorAxisLength), "Ellipse axes must be positive and finite, and sweep must be nonzero and at most one turn.");
            Flags = flags;
            _center = center;
            _majorAxis = majorAxis;
            _minorAxisLength = minorAxisLength;
            _startAngle = startAngle;
            _endAngle = endAngle;
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
        {
            double angle = _startAngle + (IsFullEllipse && uniform == 1 ? 0 : uniform) * (_endAngle - _startAngle);
            Vec2D point = EvaluateAtAngle(angle);
            Vec2D tangent = EvaluateTangentAtAngle(angle) * Math.Sign(_endAngle-_startAngle);
            
            if (tangent.LengthSquared() == 0)
                tangent = new Vec2D(1, 0);
            else
                tangent.Normalize();

            Vec2D normal = new Vec2D(tangent.Y, -tangent.X);
            return new CurveVertex2D(point, normal, uniform);
        }

        private Vec2D EvaluateAtAngle(double angle)
        {
            double majorAxisLen = _majorAxis.Length();
            if (majorAxisLen == 0)
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
            if (majorAxisLen == 0)
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
            if (!double.IsFinite(maxDeviation) || maxDeviation <= 0)
                throw new ArgumentOutOfRangeException(nameof(maxDeviation));
            double maxRadius = Math.Max(MajorAxisLength, _minorAxisLength);
            // Linear scaling of a unit circle bounds chord error by maxRadius.
            double step = 4 * Math.Asin(Math.Sqrt(Math.Min(maxDeviation/maxRadius, 2) / 2));
            double count = Math.Ceiling(Math.Abs(_endAngle-_startAngle) / step);
            if (!double.IsFinite(count) || count > int.MaxValue-5)
                throw new ArgumentOutOfRangeException(nameof(maxDeviation), "Requested ellipse tessellation exceeds the supported point count.");
            int segments = Math.Max(4,(int)count);
            if (IsFullEllipse)
                segments = (segments + 3) / 4 * 4;
            return Tessellate(segments + 1);
        }

        public override List<CurveVertex2D> Tessellate(int numPoints)
        {
            if (numPoints < 2)
                throw new ArgumentOutOfRangeException(nameof(numPoints));
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
            return new Ellipse2D(_center, _majorAxis, _minorAxisLength, _startAngle, _endAngle, Flags) { Name = Name };
        }

        public override Curve2D Reverse()
        {
            return new Ellipse2D(_center, _majorAxis, _minorAxisLength, _endAngle, _startAngle, Flags) { Name = Name };
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

