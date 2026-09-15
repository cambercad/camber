using System;
using System.Collections.Generic;
using GeoCore;

namespace Curves
{
        /// <summary>
        /// Piecewise C¹ cubic Hermite spline in 2D through a sequence of points (sketch coordinates).
        /// Default tangents (no optional directions): <b>natural cubic</b> — C² continuity at interior knots
        /// and zero second derivative at the ends (tridiagonal solve). That is the usual “natural spline”
        /// end condition and avoids the flat chord look at the terminals.
        /// Optional <see cref="StartTangent"/> / <see cref="EndTangent"/> set end derivative <em>direction</em>
        /// (magnitude from the adjacent chord) and use Catmull–Rom interiors.
        /// The per-knot constructor takes one nullable direction per point.
        /// Global parameter u ∈ [0,1] splits evenly across spans (same as <see cref="CubicHermiteSpline3D"/>).
        /// Adaptive tessellation uses a per-span clamped cubic <see cref="NURBS.BSplineCurve"/> in the z = 0 plane
        /// (<see cref="NURBS.BSplineCurve.Tessellate(double)"/>), taking x/y from the 3D evaluator.
        /// </summary>
        public sealed class CubicHermiteSpline2D : Curve2D
        {
        private Vec2D[] _points;
        private Vec2D[] _tangents;
        /// <summary>When non-null, per-knot direction hints (magnitude rescaled internally). Null entries fall back to chord/cardinal defaults for that knot.</summary>
        private readonly Vec2D?[] _knotTangentDirections;

        public IReadOnlyList<Vec2D> Points => _points;
        public IReadOnlyList<Vec2D> Tangents => _tangents;

        public Vec2D? StartTangent { get; init; }
        public Vec2D? EndTangent { get; init; }

        public CubicHermiteSpline2D(
            IReadOnlyList<Vec2D> points,
            Vec2D? startTangent = null,
            Vec2D? endTangent = null,
            CurveFlags flags = CurveFlags.None)
        {
            if (points == null || points.Count < 2)
                throw new ArgumentException("CubicHermiteSpline2D requires at least two points.", nameof(points));

            _points = new Vec2D[points.Count];
            for (int i = 0; i < points.Count; i++)
                _points[i] = points[i];

            StartTangent = startTangent;
            EndTangent = endTangent;
            base.Flags = flags;
            _knotTangentDirections = null;
            _tangents = BuildTangents(_points, startTangent, endTangent, null);
        }

        /// <summary>
        /// Hermite spline with optional tangent direction per knot. Non-null entries set that knot’s derivative direction;
        /// magnitude is chosen from adjacent chord lengths (ends: single chord; interior: half-sum of incoming and outgoing).
        /// Null entries use the same defaults as the two-end constructor (cardinal average inside, chord at ends).
        /// </summary>
        public CubicHermiteSpline2D(
            IReadOnlyList<Vec2D> points,
            IReadOnlyList<Vec2D?> knotTangentDirections,
            CurveFlags flags = CurveFlags.None)
        {
            if (points == null || points.Count < 2)
                throw new ArgumentException("CubicHermiteSpline2D requires at least two points.", nameof(points));
            if (knotTangentDirections == null || knotTangentDirections.Count != points.Count)
                throw new ArgumentException("knotTangentDirections must have the same length as points.", nameof(knotTangentDirections));

            _points = new Vec2D[points.Count];
            for (int i = 0; i < points.Count; i++)
                _points[i] = points[i];

            _knotTangentDirections = new Vec2D?[points.Count];
            for (int i = 0; i < points.Count; i++)
                _knotTangentDirections[i] = knotTangentDirections[i];

            StartTangent = null;
            EndTangent = null;
            base.Flags = flags;
            _tangents = BuildTangents(_points, null, null, _knotTangentDirections);
        }

        private CubicHermiteSpline2D(CubicHermiteSpline2D other)
        {
            _points = (Vec2D[])other._points.Clone();
            StartTangent = other.StartTangent;
            EndTangent = other.EndTangent;
            _knotTangentDirections = other._knotTangentDirections != null
                ? (Vec2D?[])other._knotTangentDirections.Clone()
                : null;
            base.Flags = other.Flags;
            Name = other.Name;
            _tangents = _knotTangentDirections != null
                ? BuildTangents(_points, null, null, _knotTangentDirections)
                : BuildTangents(_points, StartTangent, EndTangent, null);
        }

        private static Vec2D[] BuildTangents(Vec2D[] p, Vec2D? startTangent, Vec2D? endTangent, Vec2D?[] knotTangentDirections)
        {
            int n = p.Length;
            var m = new Vec2D[n];
            if (knotTangentDirections != null)
            {
                // Mix: unspecified knots fall back to a full natural solve, then override specified entries.
                Vec2D[] natural = BuildNaturalTangents(p);
                for (int i = 0; i < n; i++)
                {
                    if (knotTangentDirections[i].HasValue)
                    {
                        double refLen = KnotReferenceTangentLength(p, i);
                        m[i] = HermiteEndDerivativeFromDirection(knotTangentDirections[i].Value, refLen);
                    }
                    else
                        m[i] = natural[i];
                }

                return m;
            }

            // Fully automatic: natural cubic (zero curvature at ends, C² at interiors).
            if (!startTangent.HasValue && !endTangent.HasValue)
                return BuildNaturalTangents(p);

            // Explicit end directions + Catmull–Rom interiors.
            for (int i = 1; i < n - 1; i++)
                m[i] = 0.5 * (p[i + 1] - p[i - 1]);

            Vec2D chordStart = p[1] - p[0];
            m[0] = startTangent.HasValue
                ? HermiteEndDerivativeFromDirection(startTangent.Value, chordStart)
                : BuildNaturalTangents(p)[0];

            Vec2D chordEnd = p[n - 1] - p[n - 2];
            m[n - 1] = endTangent.HasValue
                ? HermiteEndDerivativeFromDirection(endTangent.Value, chordEnd)
                : BuildNaturalTangents(p)[n - 1];
            return m;
        }

        /// <summary>
        /// Natural cubic Hermite tangents (uniform local parameter s∈[0,1] per span):
        /// 2 m0 + m1 = 3 (p1−p0),
        /// m_{i−1} + 4 m_i + m_{i+1} = 3 (p_{i+1}−p_{i−1}) (interior C²),
        /// m_{n−2} + 2 m_{n−1} = 3 (p_{n−1}−p_{n−2}).
        /// </summary>
        private static Vec2D[] BuildNaturalTangents(Vec2D[] p)
        {
            int n = p.Length;
            if (n == 2)
            {
                Vec2D chord = p[1] - p[0];
                return new[] { chord, chord };
            }

            // Tridiagonal: a[i]*m[i-1] + b[i]*m[i] + c[i]*m[i+1] = d[i]
            var a = new double[n];
            var b = new double[n];
            var c = new double[n];
            var d = new Vec2D[n];

            b[0] = 2;
            c[0] = 1;
            d[0] = 3 * (p[1] - p[0]);

            for (int i = 1; i < n - 1; i++)
            {
                a[i] = 1;
                b[i] = 4;
                c[i] = 1;
                d[i] = 3 * (p[i + 1] - p[i - 1]);
            }

            a[n - 1] = 1;
            b[n - 1] = 2;
            d[n - 1] = 3 * (p[n - 1] - p[n - 2]);

            // Forward sweep
            for (int i = 1; i < n; i++)
            {
                double w = a[i] / b[i - 1];
                b[i] -= w * c[i - 1];
                d[i] -= w * d[i - 1];
            }

            var m = new Vec2D[n];
            m[n - 1] = d[n - 1] / b[n - 1];
            for (int i = n - 2; i >= 0; i--)
                m[i] = (d[i] - c[i] * m[i + 1]) / b[i];
            return m;
        }

        private static double KnotReferenceTangentLength(Vec2D[] p, int i)
        {
            int n = p.Length;
            if (i == 0)
                return (p[1] - p[0]).Length();
            if (i == n - 1)
                return (p[n - 1] - p[n - 2]).Length();
            return 0.5 * ((p[i] - p[i - 1]).Length() + (p[i + 1] - p[i]).Length());
        }

        /// <summary>Unit direction of <paramref name="direction"/> times length of <paramref name="referenceChord"/> (fallback if degenerate).</summary>
        private static Vec2D HermiteEndDerivativeFromDirection(Vec2D direction, Vec2D referenceChord)
        {
            double refLen = referenceChord.Length();
            double dirLen = direction.Length();
            if (refLen < 1e-18 || dirLen < 1e-18)
                return referenceChord;
            return direction * (refLen / dirLen);
        }

        private static Vec2D HermiteEndDerivativeFromDirection(Vec2D direction, double referenceLength)
        {
            double dirLen = direction.Length();
            if (referenceLength < 1e-18 || dirLen < 1e-18)
                return new Vec2D(0, 0);
            return direction * (referenceLength / dirLen);
        }

        public override double Length()
        {
            int spans = _points.Length - 1;
            int order = Math.Clamp(4 * spans, 4, 32);
            return GaussIntegrator.GetIntegrator(order).Integrate(0, 1, Speed);
        }

        private double Speed(double uniform)
        {
            EvaluateDpdu(uniform, out _, out Vec2D dpdu);
            return dpdu.Length();
        }

        private void EvaluateDpdu(double uniform, out Vec2D pos, out Vec2D dpdu)
        {
            uniform = Math.Clamp(uniform, 0, 1);
            int n = _points.Length;
            int spanCount = n - 1;
            double fu = uniform * spanCount;
            int seg = (int)Math.Floor(fu);
            if (seg >= spanCount)
                seg = spanCount - 1;
            double s = fu - seg;
            if (s < 0) s = 0;
            if (s > 1) s = 1;

            Vec2D p0 = _points[seg];
            Vec2D p1 = _points[seg + 1];
            Vec2D m0 = _tangents[seg];
            Vec2D m1 = _tangents[seg + 1];

            Hermite(s, p0, m0, p1, m1, out pos, out Vec2D dpds);
            dpdu = dpds * spanCount;
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
        {
            EvaluateDpdu(uniform, out Vec2D pos, out Vec2D dpdu);
            Vec2D tangent = dpdu.Normalized();
            Vec2D normal = new Vec2D(tangent.Y, -tangent.X);
            return new CurveVertex2D(pos, normal, uniform);
        }

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
        {
            double tol = maxDeviation > 0 ? maxDeviation : 1e-6;
            int spans = _points.Length - 1;
            var result = new List<CurveVertex2D>();
            for (int seg = 0; seg < spans; seg++)
            {
                var bspline = CubicHermiteSpanBspline.ToBSplineCurve(
                    _points[seg], _tangents[seg], _points[seg + 1], _tangents[seg + 1]);
                _ = bspline.Tessellate(tol, out List<double> localParams);
                int nPts = localParams.Count + 1;
                int start = seg > 0 ? 1 : 0;
                for (int j = start; j < nPts; j++)
                {
                    double localU = j < localParams.Count ? localParams[j] : 1.0;
                    double globalU = (seg + localU) / spans;
                    result.Add(EvaluateVertex(Math.Clamp(globalU, 0, 1)));
                }
            }

            return result;
        }

        public override List<CurveVertex2D> Tessellate(int pointCount)
        {
            if (pointCount < 2)
                throw new ArgumentOutOfRangeException(nameof(pointCount));
            double s = 1.0 / (pointCount - 1);
            var result = new List<CurveVertex2D>(pointCount);
            for (int i = 0; i < pointCount; i++)
                result.Add(EvaluateVertex(i * s));
            return result;
        }

        private static void Hermite(double s, Vec2D p0, Vec2D m0, Vec2D p1, Vec2D m1, out Vec2D p, out Vec2D dpds)
        {
            double s2 = s * s;
            double s3 = s2 * s;
            double h00 = 2 * s3 - 3 * s2 + 1;
            double h10 = s3 - 2 * s2 + s;
            double h01 = -2 * s3 + 3 * s2;
            double h11 = s3 - s2;
            p = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;

            double dh00 = 6 * s2 - 6 * s;
            double dh10 = 3 * s2 - 4 * s + 1;
            double dh01 = -6 * s2 + 6 * s;
            double dh11 = 3 * s2 - 2 * s;
            dpds = dh00 * p0 + dh10 * m0 + dh01 * p1 + dh11 * m1;
        }

        public override List<Vec2D> ToReferencePoints()
        {
            return new List<Vec2D>(_points);
        }

        public override void UpdateFromReferencePoints(List<Vec2D> referencePoints)
        {
            if (referencePoints == null || referencePoints.Count != _points.Length)
                throw new ArgumentException("Reference point count must match knot count.", nameof(referencePoints));
            for (int i = 0; i < _points.Length; i++)
                _points[i] = referencePoints[i];
            RebuildTangents();
        }

        /// <summary>
        /// Replaces through-knots (may grow/shrink). Keeps optional end-tangent directions;
        /// per-knot direction arrays only allow the same knot count.
        /// </summary>
        public void SetThroughPoints(IReadOnlyList<Vec2D> points)
        {
            if (points == null || points.Count < 2)
                throw new ArgumentException("CubicHermiteSpline2D requires at least two points.", nameof(points));
            if (_knotTangentDirections != null && _knotTangentDirections.Length != points.Count)
                throw new InvalidOperationException("Cannot change knot count when per-knot tangent directions were specified.");

            _points = new Vec2D[points.Count];
            for (int i = 0; i < points.Count; i++)
                _points[i] = points[i];
            RebuildTangents();
        }

        private void RebuildTangents()
        {
            _tangents = _knotTangentDirections != null
                ? BuildTangents(_points, null, null, _knotTangentDirections)
                : BuildTangents(_points, StartTangent, EndTangent, null);
        }

        public override Curve2D GetCopy() => new CubicHermiteSpline2D(this);

        public override Curve2D Reverse()
        {
            int n = _points.Length;
            var revPts = new Vec2D[n];
            for (int i = 0; i < n; i++)
                revPts[i] = _points[n - 1 - i];

            if (_knotTangentDirections != null)
            {
                var revDirs = new Vec2D?[n];
                for (int i = 0; i < n; i++)
                {
                    var od = _knotTangentDirections[n - 1 - i];
                    revDirs[i] = od.HasValue ? -od.Value : (Vec2D?)null;
                }

                var ck = new CubicHermiteSpline2D(revPts, revDirs, Flags);
                ck.Name = Name;
                return ck;
            }

            Vec2D? newStart = EndTangent.HasValue ? -EndTangent.Value : (Vec2D?)null;
            Vec2D? newEnd = StartTangent.HasValue ? -StartTangent.Value : (Vec2D?)null;
            var c = new CubicHermiteSpline2D(revPts, newStart, newEnd, Flags);
            c.Name = Name;
            return c;
        }
    }
}
