using Geo.BRep;
using GeoCore;
using NURBS;

namespace Geo.Export
{
    internal enum ExportCurveKind
    {
        Line,
        Circle,
        Nurbs
    }

    internal sealed class ClassifiedTrim
    {
        public ExportCurveKind Kind;
        public Vec3D P0;
        public Vec3D P1;
        public Vec2D Uv0;
        public Vec2D Uv1;
        public Vec3D CircleCenter;
        public Vec3D CircleAxis;
        public Vec3D CircleRefDir;
        public double CircleRadius;
        public BSplineCurve Nurbs;
    }

    internal static class ExportTrimSegment
    {
        public static List<ClassifiedTrim> ClassifyLoop(BRepTrimLoop loop)
        {
            var result = new List<ClassifiedTrim>();
            foreach (var segment in BRepTrimCurves.SegmentsFromLoopWithUv(loop, mergeCollinear: true, fitArcs: true))
            {
                var classified = Classify(segment);
                if ((classified.P1 - classified.P0).Length() < BRepTrimCurves.MinSegmentLength &&
                    classified.Kind != ExportCurveKind.Circle)
                    continue;
                result.Add(classified);
            }
            return result;
        }

        public static ClassifiedTrim Classify(BRepTrimSegment segment)
        {
            var p0 = segment.Curve.EvaluateUniform(0);
            var p1 = segment.Curve.EvaluateUniform(1);
            var mid = segment.Curve.EvaluateUniform(0.5);
            var item = new ClassifiedTrim
            {
                P0 = p0,
                P1 = p1,
                Uv0 = segment.Uv0,
                Uv1 = segment.Uv1,
                Nurbs = segment.Curve,
                Kind = ExportCurveKind.Nurbs
            };

            if (TryAsLine(p0, mid, p1))
            {
                item.Kind = ExportCurveKind.Line;
                return item;
            }

            // Only keep a CIRCLE when the 3D curve is already a circular NURBS (arc-fit).
            // Any three non-collinear points define a circle, so fitting helix chords would
            // invent 3D edges that do not lie on a twisted extrude / involute flank.
            if (IsCircularNurbs(segment.Curve) &&
                TryAsCircle(p0, mid, p1, out var center, out var axis, out var radius))
            {
                item.Kind = ExportCurveKind.Circle;
                item.CircleCenter = center;
                item.CircleAxis = axis;
                item.CircleRadius = radius;
                var refDir = p0 - center;
                if (refDir.LengthSquared() < 1e-16)
                    refDir = mid - center;
                item.CircleRefDir = Orthonormalize(axis, refDir);
                return item;
            }

            return item;
        }

        private static bool IsCircularNurbs(BSplineCurve curve)
        {
            if (curve == null || curve.Degree != 2 || curve.ControlPoints == null || curve.ControlPoints.Length < 3)
                return false;
            for (int i = 0; i < curve.ControlPoints.Length; i++)
            {
                if (Math.Abs(curve.ControlPoints[i].W - 1.0) > 1e-6)
                    return true;
            }
            return false;
        }

        private static bool TryAsLine(Vec3D p0, Vec3D mid, Vec3D p1)
        {
            var d = p1 - p0;
            double len = d.Length();
            if (len < 1e-12)
                return false;
            d *= 1.0 / len;
            var toMid = mid - p0;
            var perp = toMid - d * Vec3DOps.Dot(toMid, d);
            return perp.Length() / len < BRepTrimCurves.CollinearAngleTolerance;
        }

        private static bool TryAsCircle(Vec3D p0, Vec3D mid, Vec3D p1, out Vec3D center, out Vec3D axis, out double radius)
        {
            center = default;
            axis = default;
            radius = 0;
            try
            {
                var arc = new Curves.Arc3D(p0, mid, p1);
                center = arc.Center;
                axis = arc.Normal;
                radius = arc.Radius;
                return radius > 1e-12;
            }
            catch
            {
                return false;
            }
        }

        private static Vec3D Orthonormalize(Vec3D axis, Vec3D refDir)
        {
            var r = refDir - axis * Vec3DOps.Dot(refDir, axis);
            if (r.LengthSquared() < 1e-16)
            {
                var hint = Math.Abs(axis.X) < 0.9 ? new Vec3D(1, 0, 0) : new Vec3D(0, 1, 0);
                r = Vec3DOps.Cross(axis, hint);
            }
            return r.Normalized();
        }
    }
}
