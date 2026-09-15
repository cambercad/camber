using System;
using System.Collections.Generic;
using GeoCore;

namespace Curves
{
    /// <summary>
    /// Piecewise C¹ cubic Hermite spline in 3D through a sequence of points.
    /// The two-end constructor: optional <see cref="StartTangent"/> / <see cref="EndTangent"/> set the <em>direction</em> of the
    /// first/last knot derivatives w.r.t. segment parameter s ∈ [0,1]; magnitude is replaced by adjacent chord length.
    /// The per-knot constructor takes one nullable direction per point (null = default cardinal/chord for that knot); magnitudes are rescaled
    /// from adjacent chord lengths (interior: half-sum of neighbor chords).
    /// Global <see cref="Curve3D.Evaluate"/> parameter u ∈ [0,1] allocates equal length in parameter space per span.
    /// Adaptive tessellation uses a per-span clamped cubic <see cref="NURBS.BSplineCurve"/> (Bézier-equivalent) and
    /// <see cref="NURBS.BSplineCurve.Tessellate(double)"/> (same max-deviation logic as other NURBS curves).
    /// </summary>
    public sealed class CubicHermiteSpline3D : Curve3D
    {
        private readonly Vec3D[] _points;
        private readonly Vec3D[] _tangents;
        private readonly Vec3D?[] _knotTangentDirections;

        public IReadOnlyList<Vec3D> Points => _points;
        public IReadOnlyList<Vec3D> Tangents => _tangents;
        /// <summary>Derivative w.r.t. segment parameter s on the first span, or null for one-sided chord.</summary>
        public Vec3D? StartTangent { get; }
        /// <summary>Derivative w.r.t. segment parameter s on the last span at the end, or null for one-sided chord.</summary>
        public Vec3D? EndTangent { get; }

        public CubicHermiteSpline3D(
            IReadOnlyList<Vec3D> points,
            Vec3D? startTangent = null,
            Vec3D? endTangent = null,
            string name = null)
            : base(name)
        {
            if (points == null || points.Count < 2)
                throw new ArgumentException("CubicHermiteSpline3D requires at least two points.", nameof(points));

            _points = new Vec3D[points.Count];
            for (int i = 0; i < points.Count; i++)
                _points[i] = points[i];

            StartTangent = startTangent;
            EndTangent = endTangent;
            _knotTangentDirections = null;
            _tangents = BuildTangents(_points, startTangent, endTangent, null);
        }

        /// <summary>
        /// Hermite spline with optional tangent direction per knot. Non-null entries set that knot’s derivative direction;
        /// magnitude is chosen from adjacent chord lengths (ends: single chord; interior: half-sum of incoming and outgoing).
        /// </summary>
        public CubicHermiteSpline3D(
            IReadOnlyList<Vec3D> points,
            IReadOnlyList<Vec3D?> knotTangentDirections,
            string name = null)
            : base(name)
        {
            if (points == null || points.Count < 2)
                throw new ArgumentException("CubicHermiteSpline3D requires at least two points.", nameof(points));
            if (knotTangentDirections == null || knotTangentDirections.Count != points.Count)
                throw new ArgumentException("knotTangentDirections must have the same length as points.", nameof(knotTangentDirections));

            _points = new Vec3D[points.Count];
            for (int i = 0; i < points.Count; i++)
                _points[i] = points[i];

            _knotTangentDirections = new Vec3D?[points.Count];
            for (int i = 0; i < points.Count; i++)
                _knotTangentDirections[i] = knotTangentDirections[i];

            StartTangent = null;
            EndTangent = null;
            _tangents = BuildTangents(_points, null, null, _knotTangentDirections);
        }

        public CubicHermiteSpline3D(CubicHermiteSpline3D other) : base(other.Name)
        {
            _points = (Vec3D[])other._points.Clone();
            StartTangent = other.StartTangent;
            EndTangent = other.EndTangent;
            _knotTangentDirections = other._knotTangentDirections != null
                ? (Vec3D?[])other._knotTangentDirections.Clone()
                : null;
            _tangents = _knotTangentDirections != null
                ? BuildTangents(_points, null, null, _knotTangentDirections)
                : BuildTangents(_points, StartTangent, EndTangent, null);
        }

        private static Vec3D[] BuildTangents(Vec3D[] p, Vec3D? startTangent, Vec3D? endTangent, Vec3D?[] knotTangentDirections)
        {
            int n = p.Length;
            var m = new Vec3D[n];
            if (knotTangentDirections != null)
            {
                for (int i = 0; i < n; i++)
                {
                    if (knotTangentDirections[i].HasValue)
                    {
                        double refLen = KnotReferenceTangentLength(p, i);
                        m[i] = HermiteEndDerivativeFromDirection(knotTangentDirections[i].Value, refLen);
                    }
                    else if (i == 0)
                        m[i] = p[1] - p[0];
                    else if (i == n - 1)
                        m[i] = p[n - 1] - p[n - 2];
                    else
                        m[i] = 0.5 * (p[i + 1] - p[i - 1]);
                }

                return m;
            }

            for (int i = 1; i < n - 1; i++)
                m[i] = 0.5 * (p[i + 1] - p[i - 1]);

            Vec3D chordStart = p[1] - p[0];
            m[0] = startTangent.HasValue
                ? HermiteEndDerivativeFromDirection(startTangent.Value, chordStart)
                : chordStart;

            Vec3D chordEnd = p[n - 1] - p[n - 2];
            m[n - 1] = endTangent.HasValue
                ? HermiteEndDerivativeFromDirection(endTangent.Value, chordEnd)
                : chordEnd;
            return m;
        }

        private static double KnotReferenceTangentLength(Vec3D[] p, int i)
        {
            int n = p.Length;
            if (i == 0)
                return (p[1] - p[0]).Length();
            if (i == n - 1)
                return (p[n - 1] - p[n - 2]).Length();
            return 0.5 * ((p[i] - p[i - 1]).Length() + (p[i + 1] - p[i]).Length());
        }

        private static Vec3D HermiteEndDerivativeFromDirection(Vec3D direction, Vec3D referenceChord)
        {
            double refLen = referenceChord.Length();
            double dirLen = direction.Length();
            if (refLen < 1e-18 || dirLen < 1e-18)
                return referenceChord;
            return direction * (refLen / dirLen);
        }

        private static Vec3D HermiteEndDerivativeFromDirection(Vec3D direction, double referenceLength)
        {
            double dirLen = direction.Length();
            if (referenceLength < 1e-18 || dirLen < 1e-18)
                return new Vec3D(0, 0, 0);
            return direction * (referenceLength / dirLen);
        }

        /// <summary>Position and unit tangent at global parameter u ∈ [0,1].</summary>
        public override CurveVertex3D Evaluate(double uniform)
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

            Vec3D p0 = _points[seg];
            Vec3D p1 = _points[seg + 1];
            Vec3D m0 = _tangents[seg];
            Vec3D m1 = _tangents[seg + 1];

            Hermite(s, p0, m0, p1, m1, out Vec3D pos, out Vec3D dds);
            Vec3D tangent = dds.Normalized();
            Vec3D up = StablePerpendicular(tangent);
            return new CurveVertex3D(pos, tangent, up, uniform);
        }

        public override List<CurveVertex3D> Tessellate(double maxDeviation)
        {
            double tol = maxDeviation > 0 ? maxDeviation : 1e-6;
            int spans = _points.Length - 1;
            var result = new List<CurveVertex3D>();
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
                    result.Add(Evaluate(Math.Clamp(globalU, 0, 1)));
                }
            }

            return result;
        }

        private static void Hermite(double s, Vec3D p0, Vec3D m0, Vec3D p1, Vec3D m1, out Vec3D p, out Vec3D dpds)
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

        private static Vec3D StablePerpendicular(Vec3D tangent)
        {
            Vec3D t = tangent.Normalized();
            Vec3D refAxis = Math.Abs(t.X) < 0.9 ? new Vec3D(1, 0, 0) : new Vec3D(0, 1, 0);
            Vec3D up = Vec3DOps.Cross(t, refAxis);
            double len = up.Length();
            if (len < 1e-14)
                up = Vec3DOps.Cross(t, new Vec3D(0, 0, 1));
            else
                up = up / len;
            return up;
        }

        public CubicHermiteSpline3D GetCopy() => new CubicHermiteSpline3D(this);
    }
}
