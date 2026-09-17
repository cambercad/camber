using Curves;
using GeoCore;
using NURBS;

namespace Geo.NurbsConstruction
{
    public static class NurbsSurfaceFactory
    {
        public static BSplineLinearExtrudeSurface LinearExtrude(BSplineCurve profile, Vec3D extrudeDirection, double extrudeLength)
        {
            return new BSplineLinearExtrudeSurface(profile, extrudeDirection.Normalized(), extrudeLength);
        }

        public static BSplineSurfaceOfRevolution Revolve(
            Vec3D pointOnAxis, Vec3D axis, BSplineCurve profileCurve, Vec3D zeroAngleDirection)
        {
            return new BSplineSurfaceOfRevolution(pointOnAxis, axis.Normalized(), profileCurve, zeroAngleDirection.Normalized());
        }

        public static BSplinePlane PlanarCap(CoordinateSystem frame, Vec2D uvMin, Vec2D uvMax, double capOffsetAlongNormal)
        {
            var origin = frame.PointFromCoordSysToWorld(new Vec3D(
                0.5 * (uvMin.X + uvMax.X),
                0.5 * (uvMin.Y + uvMax.Y),
                capOffsetAlongNormal));

            var halfExtentX = 0.5 * (uvMax.X - uvMin.X) * frame.X;
            var halfExtentY = 0.5 * (uvMax.Y - uvMin.Y) * frame.Y;
            var normal = frame.Z;

            return new BSplinePlane(origin, normal, halfExtentX, halfExtentY);
        }

        public static INurbsSurface PlanarCapWithMeshUv(CoordinateSystem frame, Vec2D uvMin, Vec2D uvMax, double capOffsetAlongNormal)
        {
            double spanX = uvMax.X - uvMin.X;
            double spanY = uvMax.Y - uvMin.Y;
            double scale = 1.0 / Math.Max(spanX, spanY);
            var plane = PlanarCap(frame, uvMin, uvMax, capOffsetAlongNormal);
            return new CapUvMappedSurface(plane, uvMin, uvMax, scale);
        }

        public static ArcLengthMapper CreateArcLengthMapperIfNeeded(BSplineCurve curve)
        {
            if (curve != null && !ArcLengthMapper.IsApproximatelyLinear(curve))
                return new ArcLengthMapper(curve);
            return null;
        }

        public static INurbsSurface WrapForMeshUv(
            BSplineSurface inner,
            BSplineCurve profileCurveForArcLengthU = null,
            ParametricRange? range = null,
            BSplineCurve railCurveForArcLengthV = null,
            bool swapMeshUv = false)
        {
            return WrapForMeshUv(
                inner,
                profileCurveForArcLengthU,
                range,
                CreateArcLengthMapperIfNeeded(railCurveForArcLengthV),
                swapMeshUv);
        }

        public static INurbsSurface WrapForMeshUv(
            BSplineSurface inner,
            BSplineCurve profileCurveForArcLengthU,
            ParametricRange? range,
            ArcLengthMapper sharedRailMapper,
            bool swapMeshUv = false)
        {
            ArcLengthMapper uMapper = CreateArcLengthMapperIfNeeded(profileCurveForArcLengthU);
            return new MeshUvMappedSurface(inner, uMapper, range ?? ParametricRange.UnitSquare, sharedRailMapper, swapMeshUv);
        }

        public static INurbsSurface WrapLinearExtrudeForMeshUv(
            BSplineLinearExtrudeSurface inner,
            BSplineCurve profileCurveForArcLengthU = null,
            ParametricRange? range = null)
        {
            return WrapForMeshUv(inner, profileCurveForArcLengthU, range, swapMeshUv: true);
        }

        public static BSplineSurface TensorProductFromVCurves(int degreeU, IList<BSplineCurve> vCurves)
        {
            if (vCurves == null || vCurves.Count == 0)
                throw new ArgumentException("At least one v-curve is required.");
            return new BSplineSurface(degreeU, vCurves);
        }

        /// <summary>
        /// Twisted extrude / helical sweep of an exact profile NURBS: each profile control point
        /// traces a helix approximated with a compact cubic in V. U degree, knots, and weights
        /// come from the profile — not from mesh vertices.
        /// </summary>
        public static BSplineSurface SweepProfileAlongFrames(
            BSplineCurve profile,
            CoordinateSystem sourceFrame,
            IReadOnlyList<CoordinateSystem> frames)
        {
            if (profile == null)
                throw new ArgumentNullException(nameof(profile));
            if (frames == null || frames.Count < 2)
                throw new ArgumentException("Sweep requires at least two guide frames.");

            int nu = profile.ControlPoints.Length;
            int nv = frames.Count;
            int degreeV = Math.Min(3, nv - 1);
            var knotsV = BSplineCurve.UniformKnotVector(degreeV, nv);
            var uWeights = new double[nu];
            var vCurves = new List<BSplineCurve>(nu);

            for (int i = 0; i < nu; i++)
            {
                var h = profile.ControlPoints[i];
                double w = Math.Abs(h.W) < 1e-15 ? 1.0 : h.W;
                var world = new Vec3D(h.X / w, h.Y / w, h.Z / w);
                var local = sourceFrame.PointFromWorldToCoordSys(world);
                var rail = new Vec3D[nv];
                for (int j = 0; j < nv; j++)
                    rail[j] = frames[j].PointFromCoordSysToWorld(new Vec3D(local.X, local.Y, 0));
                uWeights[i] = w;
                vCurves.Add(new BSplineCurve(degreeV, rail, knotsV, false));
            }

            return new BSplineSurface(profile.Degree, vCurves, uWeights, (double[])profile.Knots.Clone());
        }

        public static BSplineSurface Transform(BSplineSurface surface, in Mat4D transform)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));
            int nu = surface.NumControlPointsU;
            int nv = surface.NumControlPointsV;
            var net = new Vec3D[nu][];
            var weights = new double[nu][];
            for (int i = 0; i < nu; i++)
            {
                net[i] = new Vec3D[nv];
                weights[i] = new double[nv];
                for (int j = 0; j < nv; j++)
                {
                    var h = surface.ControlPoints[i][j];
                    double w = Math.Abs(h.W) < 1e-15 ? 1.0 : h.W;
                    var p = new Vec3D(h.X / w, h.Y / w, h.Z / w);
                    net[i][j] = transform.TransformPoint(p);
                    weights[i][j] = w;
                }
            }

            return new BSplineSurface(
                surface.DegreeU, surface.DegreeV, net, weights,
                (double[])surface.KnotsU.Clone(), (double[])surface.KnotsV.Clone());
        }

        public static BSplineSurface SweepSegmentAlongFrames(IReadOnlyList<Vec2D> segmentPolyline, IReadOnlyList<CoordinateSystem> frames)
        {
            if (frames.Count < 2)
                throw new ArgumentException("Sweep requires at least two guide frames.");
            if (segmentPolyline == null || segmentPolyline.Count < 2)
                throw new ArgumentException("Profile segment needs at least two points.");

            int numU = segmentPolyline.Count;
            var vCurves = new List<BSplineCurve>(numU);
            for (int j = 0; j < numU; j++)
            {
                var profilePoint = segmentPolyline[j];
                var railPoints = new Vec3D[frames.Count];
                for (int i = 0; i < frames.Count; i++)
                    railPoints[i] = frames[i].PointTo3D(profilePoint);
                int degree = Math.Min(3, frames.Count - 1);
                vCurves.Add(new BSplineCurve(degree, railPoints, BSplineCurve.UniformKnotVector(degree, frames.Count), false));
            }

            int degreeU = Math.Min(3, numU - 1);
            return TensorProductFromVCurves(degreeU, vCurves);
        }

        public static BSplineCurve BuildRailCurveFromFrames(IReadOnlyList<CoordinateSystem> frames)
        {
            var points = frames.Select(f => f.Origin).ToArray();
            int degree = Math.Min(3, points.Length - 1);
            return new BSplineCurve(degree, points, BSplineCurve.UniformKnotVector(degree, points.Length), false);
        }

        public static BSplineSurface LoftFromProfileSamples(IReadOnlyList<IReadOnlyList<Vec3D>> profiles, LoftStyle style)
        {
            if (profiles == null || profiles.Count < 2)
                throw new ArgumentException("Loft requires at least two profiles.");
            int m = profiles[0].Count;
            for (int p = 1; p < profiles.Count; p++)
            {
                if (profiles[p].Count != m)
                    throw new ArgumentException("All loft profiles must have the same sample count.");
            }

            var vCurves = new List<BSplineCurve>(m);
            for (int k = 0; k < m; k++)
            {
                var column = new Vec3D[profiles.Count];
                for (int p = 0; p < profiles.Count; p++)
                    column[p] = profiles[p][k];
                vCurves.Add(BuildLoftVCurve(column, style));
            }

            int degreeU = Math.Min(3, m - 1);
            return TensorProductFromVCurves(degreeU, vCurves);
        }

        /// <summary>
        /// Skins compatible profile NURBS (same U degree, knots, and control-point count).
        /// U comes from the authored curves; V is ruled or Hermite through corresponding CPs.
        /// </summary>
        public static BSplineSurface LoftFromProfileCurves(IReadOnlyList<BSplineCurve> profiles, LoftStyle style)
        {
            if (profiles == null || profiles.Count < 2)
                throw new ArgumentException("Loft requires at least two profiles.");
            int nu = profiles[0].ControlPoints.Length;
            int degreeU = profiles[0].Degree;
            if (nu < 2)
                throw new ArgumentException("Each loft profile needs at least two control points.");
            for (int p = 1; p < profiles.Count; p++)
            {
                if (profiles[p].Degree != degreeU || profiles[p].ControlPoints.Length != nu)
                    throw new ArgumentException("Loft profiles must share U degree and control-point count.");
            }

            var uWeights = new double[nu];
            var vCurves = new List<BSplineCurve>(nu);
            for (int i = 0; i < nu; i++)
            {
                var h0 = profiles[0].ControlPoints[i];
                uWeights[i] = Math.Abs(h0.W) < 1e-15 ? 1.0 : h0.W;
                var column = new Vec3D[profiles.Count];
                for (int p = 0; p < profiles.Count; p++)
                {
                    var h = profiles[p].ControlPoints[i];
                    double w = Math.Abs(h.W) < 1e-15 ? 1.0 : h.W;
                    column[p] = new Vec3D(h.X / w, h.Y / w, h.Z / w);
                }
                vCurves.Add(BuildLoftVCurve(column, style));
            }

            return new BSplineSurface(degreeU, vCurves, uWeights, (double[])profiles[0].Knots.Clone());
        }

        /// <summary>
        /// C0 join of clamped curves, elevating their degrees without changing geometry.
        /// Joint CPs of later segments are dropped.
        /// </summary>
        public static BSplineCurve ConcatenateCompatible(IReadOnlyList<BSplineCurve> segments)
        {
            if (segments == null || segments.Count == 0)
                throw new ArgumentException("Need at least one segment.");
            if (segments.Count == 1)
                return segments[0];

            int degree = segments.Max(segment => segment.Degree);
            var cps = new List<Vec4D>();
            var knots = new List<double>();
            for (int k = 0; k <= degree; k++)
                knots.Add(0);

            int n = segments.Count;
            for (int s = 0; s < n; s++)
            {
                var seg = ElevateDegree(segments[s], degree);
                int start = s == 0 ? 0 : 1;
                for (int i = start; i < seg.ControlPoints.Length; i++)
                    cps.Add(seg.ControlPoints[i]);

                double a = (double)s / n;
                double b = (double)(s + 1) / n;
                int interiorCount = seg.ControlPoints.Length - degree - 1;
                int firstInterior = degree + 1;
                for (int i = 0; i < interiorCount; i++)
                {
                    double u = seg.Knots[firstInterior + i];
                    knots.Add(a + (b - a) * u);
                }
                if (s < n - 1)
                {
                    for (int m = 0; m < degree; m++)
                        knots.Add(b);
                }
            }

            for (int k = 0; k <= degree; k++)
                knots.Add(1);

            return new BSplineCurve(degree, cps.ToArray(), knots.ToArray(), false);
        }

        // Split into homogeneous Bezier spans with existing knot insertion, then
        // use the Bernstein degree-elevation identity. No curve fitting/sampling.
        internal static BSplineCurve ElevateDegree(BSplineCurve curve, int degree)
        {
            if (curve.Degree == degree)
                return curve;
            int p = curve.Degree;
            var breaks = curve.Knots.Distinct().OrderBy(k => k).ToArray();
            var refined = curve;
            foreach (double knot in breaks.Skip(1).SkipLast(1))
            {
                int multiplicity = curve.Knots.Count(k => k == knot);
                if (multiplicity > p)
                    throw new ArgumentException("Cannot concatenate a discontinuous curve.");
                if (multiplicity < p)
                    refined = refined.InsertKnot(knot, p - multiplicity);
            }
            var points = new List<Vec4D>();
            var knots = new List<double>();
            for (int span = 0; span < breaks.Length - 1; span++)
            {
                var control = refined.ControlPoints.Skip(span * p).Take(p + 1).ToArray();
                for (int d = p; d < degree; d++)
                {
                    var elevated = new Vec4D[d + 2];
                    elevated[0] = control[0];
                    elevated[d + 1] = control[d];
                    for (int i = 1; i <= d; i++)
                    {
                        double alpha = (double)i / (d + 1);
                        elevated[i] = control[i - 1] * alpha + control[i] * (1 - alpha);
                    }
                    control = elevated;
                }
                points.AddRange(span == 0 ? control : control.Skip(1));
                knots.AddRange(Enumerable.Repeat(breaks[span], span == 0 ? degree + 1 : degree));
            }
            knots.AddRange(Enumerable.Repeat(breaks[^1], degree + 1));
            return new BSplineCurve(degree, points.ToArray(), knots.ToArray(), curve.ClosedCurve);
        }

        private static BSplineCurve BuildLoftVCurve(Vec3D[] column, LoftStyle style)
        {
            if (style == LoftStyle.Hermite || style == LoftStyle.SmoothCatmullRom)
                return BuildHermiteVCurve(column);
            int degree = Math.Min(1, column.Length - 1);
            return new BSplineCurve(degree, column, BSplineCurve.UniformKnotVector(degree, column.Length), false);
        }

        internal static BSplineCurve BuildHermiteVCurve(Vec3D[] points, Vec3D? startTangent = null, Vec3D? endTangent = null, bool forceCubic = false)
        {
            int pCount = points.Length;
            if (pCount == 2 && !forceCubic && startTangent == null && endTangent == null)
                return new BSplineCurve(1, points, new[] { 0.0, 0.0, 1.0, 1.0 }, false);

            var segments = new List<BSplineCurve>();
            for (int p = 0; p < pCount - 1; p++)
            {
                Vec3D m0 = p == 0
                    ? points[1] - points[0]
                    : 0.5 * (points[p + 1] - points[p - 1]);
                Vec3D m1 = p == pCount - 2
                    ? points[pCount - 1] - points[pCount - 2]
                    : 0.5 * (points[p + 2] - points[p]);
                if (p == 0 && startTangent.HasValue) m0 = startTangent.Value / (pCount - 1);
                if (p == pCount - 2 && endTangent.HasValue) m1 = endTangent.Value / (pCount - 1);
                segments.Add(CubicHermiteSpanBspline.ToBSplineCurve(points[p], m0, points[p + 1], m1));
            }

            return ConcatenateCompatible(segments);
        }

        public static void ProfileBoundingBox(IEnumerable<Vec2D> points, out Vec2D min, out Vec2D max)
        {
            min = new Vec2D(double.MaxValue, double.MaxValue);
            max = new Vec2D(double.MinValue, double.MinValue);
            foreach (var p in points)
            {
                if (p.X < min.X) min.X = p.X;
                if (p.Y < min.Y) min.Y = p.Y;
                if (p.X > max.X) max.X = p.X;
                if (p.Y > max.Y) max.Y = p.Y;
            }
        }
    }
}
