using GeoCore;


namespace Curves
{
    public class Bezier2D : Curve2D
    {
        private Vec2D[] _controlPoints;
        public Vec2D[] ControlPoints { get { return _controlPoints; } }


        protected Bezier2D() { }
        public Bezier2D(params Vec2D[] controlPoints)
        {
            _controlPoints = controlPoints;
        }

        public override double Length()
        {
            return GaussIntegrator.GetIntegrator(2 * (_controlPoints.Length - 1)).Integrate(0, 1, Speed);
        }

        public double Speed(double uniform)
        {
            Vec2D tangent = BezierTessellator.EvaluateBezierDerivativeDeCasteljau(_controlPoints, uniform);
            return Math.Sqrt(tangent.X * tangent.X + tangent.Y * tangent.Y);
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
        {
            Vec2D tangent = BezierTessellator.EvaluateBezierDerivativeDeCasteljau(_controlPoints, uniform).Normalized();
            Vec2D n = new Vec2D(tangent.Y, -tangent.X);
            return new CurveVertex2D(BezierTessellator.EvaluateBezierDeCasteljau(_controlPoints, uniform), n, uniform);
        }

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
        {
            List<double> uValues;
            List<Vec2D> pts = BezierTessellator.Tessellate(maxDeviation, _controlPoints, out uValues);
            List<CurveVertex2D> result = new List<CurveVertex2D>(pts.Count);
            for (int i = 0; i < pts.Count; ++i)
            {
                double u = uValues[i];
                Vec2D tangent = BezierTessellator.EvaluateBezierDerivativeDeCasteljau(_controlPoints, u).Normalized();
                Vec2D n = new Vec2D(tangent.Y, -tangent.X);
                result.Add(new CurveVertex2D(pts[i], n, u));
            }
            return result;
        }
        public override List<CurveVertex2D> Tessellate(int numPoints)
        {           
            double s = 1.0/(numPoints-1);
            List<CurveVertex2D> result = new List<CurveVertex2D>(numPoints);
            for (int i = 0; i < numPoints; ++i)
            {
                double u = i * s;
                Vec2D tangent = BezierTessellator.EvaluateBezierDerivativeDeCasteljau(_controlPoints, u).Normalized();
                Vec2D n = new Vec2D(tangent.Y, -tangent.X);
                result.Add(new CurveVertex2D(BezierTessellator.EvaluateBezierDeCasteljau(_controlPoints, u), n, u));
            }
            //result[result.Count - 1].Position = EndPosition;
            return result;
        }

        public override Curve2D GetCopy()
        {
            Vec2D[] copy = new Vec2D[_controlPoints.Length];
            Array.Copy(_controlPoints, copy, _controlPoints.Length);
            return new Bezier2D(copy);
        }

        public override Curve2D Reverse()
        {
            Vec2D[] reversed = new Vec2D[_controlPoints.Length];
            for (int i = 0; i < _controlPoints.Length; i++)
            {
                reversed[i] = _controlPoints[_controlPoints.Length - 1 - i];
            }
            return new Bezier2D(reversed);
        }
    }
}