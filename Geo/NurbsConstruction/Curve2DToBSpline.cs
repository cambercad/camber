using Curves;
using GeoCore;
using NURBS;

namespace Geo.NurbsConstruction
{
    public static class Curve2DToBSpline
    {
        public static BSplineCurve ToBSplineCurve(Curve2D curve, CoordinateSystem frame)
        {
            return curve switch
            {
                Line2D line => LineToBSpline(line, frame),
                Arc2D arc => ArcToBSpline(arc, frame),
                Circle2D circle => CircleToBSpline(circle, frame),
                CubicHermiteSpline2D hermite => HermiteToBSpline(hermite, frame),
                Bezier2D bezier => BezierToBSpline(bezier, frame),
                Ellipse2D ellipse => EllipseToBSpline(ellipse, frame),
                BSpline2D spline => BSpline2DToBSpline(spline, frame),
                _ => PolylineFallback(curve, frame)
            };
        }

        public static BSplineCurve LineToBSpline(Line2D line, CoordinateSystem frame)
        {
            var start = frame.PointTo3D(line.StartPosition);
            var end = frame.PointTo3D(line.EndPosition);
            return new BSplineCurve(1,
                new[] { start, end },
                new[] { 0.0, 0.0, 1.0, 1.0 },
                closedCurve: false);
        }

        public static BSplineCurve ArcToBSpline(Arc2D arc, CoordinateSystem frame)
        {
            var center = frame.PointTo3D(arc.Center);
            // Arc2D stores a signed span (CW fillets have AngleEnd < AngleStart).
            // Wrapping a negative sweep into (0, 2π] skins a complementary pipe.
            double sweep = arc.AngleEnd - arc.AngleStart;
            var axis = sweep < 0 ? -frame.Z : frame.Z;
            var startDir = frame.PointTo3D(arc.StartPosition) - center;
            double endParam = Math.Abs(sweep) / (2 * Math.PI);
            if (endParam < 1e-15)
                return LineToBSpline(new Line2D(arc.StartPosition, arc.EndPosition), frame);
            if (endParam > 1.0)
                endParam = 1.0;
            var circle = new BSplineCircle(center, axis, arc.Radius, startDir, start: 0, end: endParam);
            return circle.GetNativeParamRangeCurve();
        }

        public static BSplineCurve CircleToBSpline(Circle2D circle, CoordinateSystem frame)
        {
            var center = frame.PointTo3D(circle.Center);
            var axis = frame.Z;
            var startDir = frame.PointTo3D(circle.StartPosition) - center;
            var nurbsCircle = new BSplineCircle(center, axis, circle.Radius, startDir);
            return nurbsCircle.GetNativeParamRangeCurve();
        }

        public static BSplineCurve BezierToBSpline(Bezier2D bezier, CoordinateSystem frame)
        {
            var cps = bezier.ControlPoints.Select(p => frame.PointTo3D(p)).ToArray();
            return new BezierCurve(cps);
        }

        public static BSplineCurve EllipseToBSpline(Ellipse2D ellipse, CoordinateSystem frame)
        {
            int samples = ellipse.IsFullEllipse ? 32 : 24;
            var points = new Vec3D[samples];
            for (int i = 0; i < samples; i++)
            {
                double u = samples == 1 ? 0 : (double)i / (samples - (ellipse.IsFullEllipse ? 0 : 1));
                if (ellipse.IsFullEllipse && i == samples - 1)
                    u = 0;
                points[i] = frame.PointTo3D(ellipse.EvaluateVertex(u).Position);
            }
            return ChordLengthSpline(points, ellipse.IsFullEllipse);
        }

        public static BSplineCurve BSpline2DToBSpline(BSpline2D spline, CoordinateSystem frame)
        {
            var points = spline.ControlPoints.Select(p => frame.PointTo3D(p)).ToArray();
            if (spline.IsRational)
            {
                var weighted = new Vec4D[points.Length];
                for (int i = 0; i < points.Length; i++)
                {
                    double w = spline.Weights[i];
                    weighted[i] = new Vec4D(points[i].X * w, points[i].Y * w, points[i].Z * w, w);
                }
                return new BSplineCurve(spline.Degree, weighted, spline.Knots, false);
            }
            return new BSplineCurve(spline.Degree, points, spline.Knots, false);
        }

        public static BSplineCurve HermiteToBSpline(CubicHermiteSpline2D hermite, CoordinateSystem frame)
        {
            var points = hermite.Points;
            if (points.Count < 2)
                throw new ArgumentException("Hermite spline needs at least two points.");

            var segments = new List<BSplineCurve>();
            for (int i = 0; i < points.Count - 1; i++)
            {
                var p0 = frame.PointTo3D(points[i]);
                var p1 = frame.PointTo3D(points[i + 1]);
                var m0 = frame.DirectionTo3D(hermite.Tangents[i]);
                var m1 = frame.DirectionTo3D(hermite.Tangents[i + 1]);
                segments.Add(CubicHermiteSpanBspline.ToBSplineCurve(p0, m0, p1, m1));
            }

            return NurbsSurfaceFactory.ConcatenateCompatible(segments);
        }

        public static BSplineCurve PolylineFallback(Curve2D curve, CoordinateSystem frame)
        {
            var samples = curve.Tessellate(64);
            var points = samples.Select(v => frame.PointTo3D(v.Position)).ToArray();
            return ChordLengthSpline(points);
        }

        public static BSplineCurve FromPolylineEndpoints(IReadOnlyList<Vec2D> vertices, CoordinateSystem frame)
        {
            if (vertices.Count < 2)
                throw new ArgumentException("Need at least two points.");
            var start = frame.PointTo3D(vertices[0]);
            var end = frame.PointTo3D(vertices[vertices.Count - 1]);
            return new BSplineCurve(1, new[] { start, end }, new[] { 0.0, 0.0, 1.0, 1.0 }, closedCurve: false);
        }

        private static BSplineCurve ChordLengthSpline(Vec3D[] points, bool closed = false)
        {
            if (points.Length < 2)
                throw new ArgumentException("Need at least two points.");

            int degree = Math.Min(3, points.Length - 1);
            var knots = BSplineCurve.UniformKnotVector(degree, points.Length);
            return new BSplineCurve(degree, points, knots, closed);
        }
    }
}
