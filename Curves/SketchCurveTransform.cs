using Curves.Base;
using GeoCore;

namespace Curves
{
    public static class SketchCurveTransform
    {
        public static Curve2D TransformCurve(Curve2D source, RigidTransform2D transform, BaseCurveFactory factory)
            => TransformCurve(source, transform.Map, factory);

        public static Curve2D TransformCurve(Curve2D source, Func<Vec2D, Vec2D> map, BaseCurveFactory factory)
        {
            if (source == null)
                throw new ArgumentNullException(nameof(source));
            if (map == null)
                throw new ArgumentNullException(nameof(map));
            if (factory == null)
                throw new ArgumentNullException(nameof(factory));

            if (source is Line2D line)
            {
                return factory.CreateLine2D(
                    map(line.StartPosition),
                    map(line.EndPosition),
                    source.Flags);
            }

            if (source is Circle2D circle)
            {
                return factory.CreateCircle2D(
                    map(circle.Center),
                    circle.Radius,
                    source.Flags);
            }

            if (source is Arc2D arc)
            {
                var pts = arc.ToPoints();
                return factory.CreateArc2D(
                    map(pts[0]),
                    map(pts[1]),
                    map(pts[2]),
                    source.Flags);
            }

            if (source is Bezier2D bezier)
            {
                var controlPoints = bezier.ControlPoints.Select(map).ToArray();
                var copy = new Bezier2D(controlPoints);
                copy.Flags = source.Flags;
                return copy;
            }

            if (source is Ellipse2D ellipse)
            {
                var center = map(ellipse.Center);
                var major = map(ellipse.Center + ellipse.MajorAxis) - center;
                var minor = map(ellipse.Center + new Vec2D(-ellipse.MajorAxis.Y,ellipse.MajorAxis.X)) - center;
                double sign = major.X*minor.Y-major.Y*minor.X < 0 ? -1 : 1;
                return factory.CreateEllipse2D(center,major,ellipse.MinorAxisLength,
                    sign*ellipse.StartAngle,sign*ellipse.EndAngle,source.Flags);
            }

            if (source is CubicHermiteSpline2D)
            {
                var copy = source.GetCopy();
                var refPts = source.ToReferencePoints();
                for (int i = 0; i < refPts.Count; i++)
                    refPts[i] = map(refPts[i]);
                copy.UpdateFromReferencePoints(refPts);
                copy.Flags = source.Flags;
                return copy;
            }

            return TransformViaTessellation(source, map);
        }

        private static Curve2D TransformViaTessellation(Curve2D source, Func<Vec2D, Vec2D> map)
        {
            var vertices = source.Tessellate(24);
            if (vertices.Count < 2)
                throw new NotSupportedException($"Cannot transform curve type {source.GetType().Name}: insufficient tessellation points.");

            if (vertices.Count == 2)
            {
                return new Line2D(
                    map(vertices[0].Position),
                    map(vertices[1].Position),
                    source.Flags);
            }

            var sampled = new SampledCurve(vertices.Count);
            for (int i = 0; i < vertices.Count; i++)
                sampled.Add(map(vertices[i].Position), vertices[i].Tangent);

            sampled.Flags = source.Flags;
            return sampled;
        }

        public static List<List<Curve2D>> TransformContours(
            IReadOnlyList<IReadOnlyList<Curve2D>> contours,
            Func<Vec2D, Vec2D> map,
            BaseCurveFactory factory)
        {
            var result = new List<List<Curve2D>>();
            if (contours == null)
                return result;

            foreach (var contour in contours)
            {
                var transformed = new List<Curve2D>();
                foreach (var curve in contour)
                {
                    var copy = TransformCurve(curve, map, factory);
                    copy.Flags = curve.Flags;
                    transformed.Add(copy);
                }

                if (transformed.Count > 0)
                    result.Add(transformed);
            }

            return result;
        }
    }
}
