using GeoCore;
using NURBS;

namespace Geo.NurbsConstruction
{
    /// <summary>
    /// Maps normalized arc-length parameter s ∈ [0,1] to the B-spline knot parameter.
    /// </summary>
    public sealed class ArcLengthMapper
    {
        private readonly BSplineCurve _curve;

        public ArcLengthMapper(BSplineCurve curve)
        {
            _curve = curve;
        }

        public double KnotFromNormalizedArcLength(double s)
        {
            s = Math.Clamp(s, 0, 1);
            if (s <= 0)
                return 0;
            if (s >= 1)
                return 1;
            double targetLength = s * _curve.TotalArcLength;
            return _curve.GetCurveParameter(targetLength);
        }

        public static bool IsApproximatelyLinear(BSplineCurve curve, double tolerance = 1e-6)
        {
            if (curve.Degree != 1)
                return false;
            var a = curve.EvaluateUniform(0);
            var b = curve.EvaluateUniform(1);
            double midKnot = curve.GetCurveParameter(0.5 * curve.TotalArcLength);
            var mid = curve.EvaluateUniform(midKnot);
            var linearMid = 0.5 * (a + b);
            return Vec3DOps.Length(mid - linearMid) < tolerance;
        }
    }
}
