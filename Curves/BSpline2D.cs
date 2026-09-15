#nullable disable

using GeoCore;

namespace Curves
{
    public class BSpline2D : Curve2D
    {
        private Vec2D[] _controlPoints;
        private double[] _knots;
        private double[] _weights;
        private int _degree;
        private double _uMin;
        private double _uMax;

        public Vec2D[] ControlPoints { get { return _controlPoints; } }
        public double[] Knots { get { return _knots; } }
        public double[] Weights { get { return _weights; } }
        public int Degree { get { return _degree; } }
        public bool IsRational { get { return _weights != null; } }

        protected BSpline2D() 
        {
            _controlPoints = Array.Empty<Vec2D>();
            _knots = Array.Empty<double>();
        }

        /// <summary>
        /// Creates a B-spline curve (non-rational)
        /// </summary>
        public BSpline2D(Vec2D[] controlPoints, double[] knots, int degree)
        {
            _controlPoints = controlPoints;
            _knots = knots;
            _degree = degree;
            _weights = null;
            ValidateAndInitialize();
        }

        /// <summary>
        /// Creates a NURBS curve (rational B-spline)
        /// </summary>
        public BSpline2D(Vec2D[] controlPoints, double[] knots, int degree, double[] weights)
        {
            _controlPoints = controlPoints;
            _knots = knots;
            _degree = degree;
            _weights = weights;
            ValidateAndInitialize();
        }

        private void ValidateAndInitialize()
        {
            if (_controlPoints.Length == 0)
                throw new ArgumentException("Must have at least one control point");
            
            if (_knots.Length != _controlPoints.Length + _degree + 1)
                throw new ArgumentException("Invalid knot vector size");

            if (_weights != null && _weights.Length != _controlPoints.Length)
                throw new ArgumentException("Weights array must match control points");

            _uMin = _knots[_degree];
            _uMax = _knots[_knots.Length - _degree - 1];
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
        {
            double u = _uMin + uniform * (_uMax - _uMin);
            Vec2D point = EvaluatePoint(u);
            Vec2D tangent = EvaluateTangent(u);
            
            if (tangent.LengthSquared() < 1e-10)
                tangent = new Vec2D(1, 0);
            else
                tangent.Normalize();

            Vec2D normal = new Vec2D(-tangent.Y, tangent.X);
            return new CurveVertex2D(point, normal, uniform);
        }

        private Vec2D EvaluatePoint(double u)
        {
            // Clamp to valid range
            if (u <= _uMin) u = _uMin;
            if (u >= _uMax) u = _uMax;

            int n = _controlPoints.Length - 1;
            Vec2D point = new Vec2D(0, 0);

            if (IsRational)
            {
                double weightSum = 0;
                for (int i = 0; i <= n; i++)
                {
                    double basis = BasisFunction(i, _degree, u);
                    double w = _weights[i];
                    point.X += basis * w * _controlPoints[i].X;
                    point.Y += basis * w * _controlPoints[i].Y;
                    weightSum += basis * w;
                }
                if (weightSum > 1e-10)
                {
                    point.X /= weightSum;
                    point.Y /= weightSum;
                }
            }
            else
            {
                for (int i = 0; i <= n; i++)
                {
                    double basis = BasisFunction(i, _degree, u);
                    point.X += basis * _controlPoints[i].X;
                    point.Y += basis * _controlPoints[i].Y;
                }
            }

            return point;
        }

        private Vec2D EvaluateTangent(double u)
        {
            if (_degree == 0)
                return new Vec2D(1, 0);

            // Clamp to valid range
            if (u <= _uMin) u = _uMin + 1e-8;
            if (u >= _uMax) u = _uMax - 1e-8;

            int n = _controlPoints.Length - 1;
            Vec2D tangent = new Vec2D(0, 0);

            for (int i = 0; i <= n; i++)
            {
                double basisDeriv = BasisDerivative(i, _degree, u);
                tangent.X += basisDeriv * _controlPoints[i].X;
                tangent.Y += basisDeriv * _controlPoints[i].Y;
            }

            return tangent;
        }

        // Cox-de Boor recursion formula for B-spline basis functions
        private double BasisFunction(int i, int p, double u)
        {
            if (p == 0)
            {
                // Handle the special case for the last knot span
                if (i == _controlPoints.Length - 1 && Math.Abs(u - _uMax) < 1e-10)
                    return 1.0;
                return (u >= _knots[i] && u < _knots[i + 1]) ? 1.0 : 0.0;
            }

            double left = 0.0;
            double denomLeft = _knots[i + p] - _knots[i];
            if (Math.Abs(denomLeft) > 1e-10)
            {
                left = (u - _knots[i]) / denomLeft * BasisFunction(i, p - 1, u);
            }

            double right = 0.0;
            double denomRight = _knots[i + p + 1] - _knots[i + 1];
            if (Math.Abs(denomRight) > 1e-10)
            {
                right = (_knots[i + p + 1] - u) / denomRight * BasisFunction(i + 1, p - 1, u);
            }

            return left + right;
        }

        private double BasisDerivative(int i, int p, double u)
        {
            if (p == 0)
                return 0.0;

            double left = 0.0;
            double denomLeft = _knots[i + p] - _knots[i];
            if (Math.Abs(denomLeft) > 1e-10)
            {
                left = p / denomLeft * BasisFunction(i, p - 1, u);
            }

            double right = 0.0;
            double denomRight = _knots[i + p + 1] - _knots[i + 1];
            if (Math.Abs(denomRight) > 1e-10)
            {
                right = p / denomRight * BasisFunction(i + 1, p - 1, u);
            }

            return left - right;
        }

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
        {
            // Simple adaptive sampling based on control point count
            int numPoints = Math.Max(50, _controlPoints.Length * 10);
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
            // Approximate length by sampling
            int samples = Math.Max(100, _controlPoints.Length * 20);
            double length = 0;
            double step = 1.0 / (samples - 1);
            Vec2D prev = EvaluatePoint(_uMin);

            for (int i = 1; i < samples; i++)
            {
                double u = _uMin + i * step * (_uMax - _uMin);
                Vec2D curr = EvaluatePoint(u);
                length += (curr - prev).Length();
                prev = curr;
            }

            return length;
        }

        public override Curve2D GetCopy()
        {
            Vec2D[] cpCopy = new Vec2D[_controlPoints.Length];
            Array.Copy(_controlPoints, cpCopy, _controlPoints.Length);

            double[] knotsCopy = new double[_knots.Length];
            Array.Copy(_knots, knotsCopy, _knots.Length);

            if (IsRational)
            {
                double[] weightsCopy = new double[_weights.Length];
                Array.Copy(_weights, weightsCopy, _weights.Length);
                return new BSpline2D(cpCopy, knotsCopy, _degree, weightsCopy);
            }

            return new BSpline2D(cpCopy, knotsCopy, _degree);
        }

        public override Curve2D Reverse()
        {
            // Reverse control points
            Vec2D[] cpReversed = new Vec2D[_controlPoints.Length];
            for (int i = 0; i < _controlPoints.Length; i++)
            {
                cpReversed[i] = _controlPoints[_controlPoints.Length - 1 - i];
            }

            // Reverse and mirror knot vector
            double[] knotsReversed = new double[_knots.Length];
            double knotSpan = _knots[_knots.Length - 1] - _knots[0];
            for (int i = 0; i < _knots.Length; i++)
            {
                knotsReversed[i] = knotSpan - (_knots[_knots.Length - 1 - i] - _knots[0]);
            }

            if (IsRational)
            {
                // Reverse weights
                double[] weightsReversed = new double[_weights.Length];
                for (int i = 0; i < _weights.Length; i++)
                {
                    weightsReversed[i] = _weights[_weights.Length - 1 - i];
                }
                return new BSpline2D(cpReversed, knotsReversed, _degree, weightsReversed);
            }

            return new BSpline2D(cpReversed, knotsReversed, _degree);
        }
    }
}

