using Curves;
using GeoCore;
using NURBS;

namespace Geo.NurbsConstruction
{
    public static class Curve3DToBSpline
    {
        public static BSplineCurve ToBSplineCurve(Curve3D curve)
        {
            return curve switch
            {
                Line3D line => LineToBSpline(line),
                Arc3D arc => ArcToBSpline(arc),
                Circle3D circle => CircleToBSpline(circle),
                CubicHermiteSpline3D hermite => HermiteToBSpline(hermite),
                _ => PolylineFallback(curve)
            };
        }

        public static BSplineCurve LineToBSpline(Line3D line)
        {
            return new BSplineCurve(1,
                new[] { line.Start, line.End },
                new[] { 0.0, 0.0, 1.0, 1.0 },
                closedCurve: false);
        }

        public static BSplineCurve ArcToBSpline(Arc3D arc)
        {
            var normal = arc.SweepAngle < 0 ? -arc.Normal : arc.Normal;
            var startDir = arc.Evaluate(0).Origin - arc.Center;
            double endParam = Math.Abs(arc.SweepAngle) / (2 * Math.PI);
            if (endParam < 1e-15)
                return LineToBSpline(new Line3D(arc.Evaluate(0).Origin, arc.Evaluate(1).Origin));
            if (endParam > 1.0)
                endParam = 1.0;
            var circle = new BSplineCircle(arc.Center, normal, arc.Radius, startDir, start: 0, end: endParam);
            return circle.GetNativeParamRangeCurve();
        }

        public static BSplineCurve CircleToBSpline(Circle3D circle)
        {
            var startDir = circle.Evaluate(0).Origin - circle.Center;
            var nurbsCircle = new BSplineCircle(circle.Center, circle.Normal, circle.Radius, startDir);
            return nurbsCircle.GetNativeParamRangeCurve();
        }

        public static BSplineCurve HermiteToBSpline(CubicHermiteSpline3D hermite)
        {
            var points = hermite.Points;
            var segments = new List<BSplineCurve>();
            for (int i = 0; i < points.Count - 1; i++)
            {
                var p0 = points[i];
                var p1 = points[i + 1];
                segments.Add(CubicHermiteSpanBspline.ToBSplineCurve(p0, hermite.Tangents[i], p1, hermite.Tangents[i + 1]));
            }
            return NurbsSurfaceFactory.ConcatenateCompatible(segments);
        }

        public static BSplineCurve PolylineFallback(Curve3D curve)
        {
            var samples = curve.Tessellate(0.01);
            var points = samples.Select(v => v.Origin).ToArray();
            int degree = Math.Min(3, points.Length - 1);
            return new BSplineCurve(degree, points, BSplineCurve.UniformKnotVector(degree, points.Length), false);
        }
    }
}
