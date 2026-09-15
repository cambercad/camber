using GeoCore;

namespace Curves
{
    /// <summary>
    /// Circle-involute sampling and ISO 53-style spur teeth. Flanks are natural
    /// cubic Hermite splines through involute knots spaced to the given max-deviation
    /// (same idea as NACA 4-digit airfoils).
    /// </summary>
    public static class InvoluteGear2D
    {
        public static double RadiusAt(double baseRadius, double t)
        {
            return baseRadius * Math.Sqrt(1.0 + t * t);
        }

        public static double RollAngleAtRadius(double baseRadius, double radius)
        {
            if (radius <= baseRadius)
                return 0.0;
            double t2 = (radius / baseRadius) * (radius / baseRadius) - 1.0;
            return Math.Sqrt(t2);
        }

        /// <summary>
        /// Circle involute of <paramref name="baseRadius"/>. Unroll angle <paramref name="t"/> ≥ 0.
        /// <paramref name="mirror"/> reflects across the local x-axis (the other tooth flank).
        /// </summary>
        public static Vec2D Position(Vec2D center, double baseRadius, double t, double rotation, bool mirror = false)
        {
            double c = Math.Cos(t);
            double s = Math.Sin(t);
            double x = baseRadius * (c + t * s);
            double y = baseRadius * (s - t * c);
            if (mirror)
                y = -y;
            return Rotate(center, new Vec2D(x, y), rotation);
        }

        public static CubicHermiteSpline2D FitSpline(
            Vec2D center,
            double baseRadius,
            double tStart,
            double tEnd,
            double rotation,
            double maxDeviation,
            CurveFlags flags = CurveFlags.None,
            bool mirror = false)
        {
            if (baseRadius <= 1e-15)
                throw new ArgumentOutOfRangeException(nameof(baseRadius));
            if (maxDeviation <= 0)
                maxDeviation = 0.01;

            double tMin = Math.Min(tStart, tEnd);
            double tMax = Math.Max(tStart, tEnd);
            double tmaxAbs = Math.Max(Math.Abs(tStart), Math.Abs(tEnd));
            if (tmaxAbs < 0.1)
                tmaxAbs = 0.1;
            // Involute curvature κ = 1/(rb |t|); equal-t chord sagitta ≈ rb |t| Δt² / 8.
            double dt = Math.Sqrt(Math.Max(1e-12, 8.0 * maxDeviation / (baseRadius * tmaxAbs)));
            int n = Math.Max(8, (int)Math.Ceiling(Math.Abs(tEnd - tStart) / dt) + 1);

            CubicHermiteSpline2D spline = BuildEqualTSpline(
                center, baseRadius, tStart, tEnd, rotation, n, flags, mirror);
            for (int pass = 0; pass < 8; pass++)
            {
                if (MaxSplineDeviation(spline, center, baseRadius, tMin, tMax, rotation, mirror) <= maxDeviation)
                    return spline;
                n = Math.Min(512, n * 2);
                spline = BuildEqualTSpline(center, baseRadius, tStart, tEnd, rotation, n, flags, mirror);
            }
            return spline;
        }

        static CubicHermiteSpline2D BuildEqualTSpline(
            Vec2D center, double baseRadius, double tStart, double tEnd, double rotation, int n,
            CurveFlags flags, bool mirror)
        {
            var points = new List<Vec2D>(n);
            double step = (tEnd - tStart) / (n - 1);
            for (int i = 0; i < n; i++)
                points.Add(Position(center, baseRadius, tStart + i * step, rotation, mirror));
            return new CubicHermiteSpline2D(points, null, null, flags);
        }

        static double MaxSplineDeviation(
            CubicHermiteSpline2D spline, Vec2D center, double rb, double tMin, double tMax,
            double rotation, bool mirror)
        {
            int samples = Math.Max(24, 4 * (spline.Points.Count - 1));
            double worst = 0;
            for (int i = 0; i <= samples; i++)
            {
                Vec2D p = spline.EvaluateVertex(i / (double)samples).Position;
                worst = Math.Max(worst, DistanceToInvolute(center, rb, tMin, tMax, rotation, p, mirror));
            }
            return worst;
        }

        static double DistanceToInvolute(
            Vec2D center, double rb, double tMin, double tMax, double rotation, Vec2D q, bool mirror)
        {
            double r = (q - center).Length();
            double t = RollAngleAtRadius(rb, r);
            t = Math.Clamp(t, tMin, tMax);
            for (int k = 0; k < 12; k++)
            {
                Vec2D p = Position(center, rb, t, rotation, mirror);
                Vec2D dp = Tangent(rb, t, rotation, mirror);
                Vec2D ddp = Acceleration(rb, t, rotation, mirror);
                Vec2D w = p - q;
                double f = w.X * dp.X + w.Y * dp.Y;
                double df = dp.LengthSquared() + w.X * ddp.X + w.Y * ddp.Y;
                if (Math.Abs(df) < 1e-30)
                    break;
                t = Math.Clamp(t - f / df, tMin, tMax);
            }
            return (Position(center, rb, t, rotation, mirror) - q).Length();
        }

        static Vec2D Tangent(double baseRadius, double t, double rotation, bool mirror)
        {
            double x = baseRadius * t * Math.Cos(t);
            double y = baseRadius * t * Math.Sin(t);
            if (mirror)
                y = -y;
            return Rotate(new Vec2D(0, 0), new Vec2D(x, y), rotation);
        }

        static Vec2D Acceleration(double baseRadius, double t, double rotation, bool mirror)
        {
            double c = Math.Cos(t);
            double s = Math.Sin(t);
            double x = baseRadius * (c - t * s);
            double y = baseRadius * (s + t * c);
            if (mirror)
                y = -y;
            return Rotate(new Vec2D(0, 0), new Vec2D(x, y), rotation);
        }

        public static List<Curve2D> CreateGear(
            Vec2D center,
            double module,
            int teeth,
            double pressureAngleRadians,
            double addendumFactor,
            double dedendumFactor,
            double maxDeviation)
        {
            if (module <= 0)
                throw new ArgumentOutOfRangeException(nameof(module));
            if (teeth < 6)
                throw new ArgumentOutOfRangeException(nameof(teeth), "Need at least 6 teeth.");

            var curves = new List<Curve2D>();
            double step = 2.0 * Math.PI / teeth;
            for (int i = 0; i < teeth; i++)
            {
                curves.AddRange(CreateTooth(
                    center, module, teeth, pressureAngleRadians, addendumFactor, dedendumFactor,
                    maxDeviation, i * step));
            }
            return curves;
        }

        public static List<Curve2D> CreateTooth(
            Vec2D center,
            double module,
            int teeth,
            double pressureAngleRadians,
            double addendumFactor,
            double dedendumFactor,
            double maxDeviation,
            double toothRotation)
        {
            if (module <= 0)
                throw new ArgumentOutOfRangeException(nameof(module));
            if (teeth < 6)
                throw new ArgumentOutOfRangeException(nameof(teeth), "Need at least 6 teeth.");

            double rp = 0.5 * module * teeth;
            double rb = rp * Math.Cos(pressureAngleRadians);
            if (rb <= 1e-12)
                throw new ArgumentException("Base radius vanished; pressure angle too large.");
            double ra = rp + addendumFactor * module;
            double rf = Math.Max(0.15 * module, rp - dedendumFactor * module);
            double tTip = RollAngleAtRadius(rb, ra);
            double tFoot = RollAngleAtRadius(rb, Math.Max(rb * 1.002, rf));
            if (tTip <= tFoot + 1e-8)
                throw new ArgumentException("Addendum is inside the form diameter.");

            // Same layout as Geo.Samples.CogWheel: one evolvent, mirror across the
            // tooth axis, circular tip and root. Flanks are Hermite fits (not SampledCurve).
            // Base circle stays rp·cos(α) so the pitch-circle pressure angle is ISO-ish,
            // unlike CogWheel which unrolls the root circle.
            double halfThick = Math.PI / (2.0 * teeth);
            double pitchPolar = PolarOfLocal(Math.Tan(pressureAngleRadians));
            double rotMinus = toothRotation - halfThick - pitchPolar;
            Vec2D axis = new Vec2D(Math.Cos(toothRotation), Math.Sin(toothRotation));

            var rising = FitSpline(center, rb, tFoot, tTip, rotMinus, maxDeviation);
            var falling = MirrorHermite(rising, center, axis);
            Vec2D tipA = rising.EndPosition;
            Vec2D tipB = falling.StartPosition;
            Vec2D footA = rising.StartPosition;
            Vec2D footB = falling.EndPosition;

            double sector = Math.PI / teeth;
            double aLo = toothRotation - sector;
            double aHi = toothRotation + sector;
            double fillet = 0.38 * module;

            var curves = new List<Curve2D>();
            AppendRootAndFillet(curves, center, rf, aLo, footA, -1.0, fillet);
            curves.Add(rising);
            curves.Add(new Arc2D(tipA, Polar(center, ra, Angle(center, tipA) + 0.5 * WrapDelta(Angle(center, tipA), Angle(center, tipB))), tipB));
            curves.Add(falling);
            AppendFilletAndRoot(curves, center, rf, footB, aHi, 1.0, fillet);
            return curves;
        }

        /// <summary>
        /// Inner hole of an internal ring gear (filled region includes the origin).
        /// This is an ordinary external spur with addendum/dedendum swapped: hole
        /// tips sit at the internal root (rp + dedendum) and hole roots at the
        /// internal tip (rp − addendum). Subtract the extrusion from a disc.
        /// The cut-outs are then the same involute family as a mating pinion.
        /// </summary>
        public static List<Curve2D> CreateInternalGear(
            Vec2D center,
            double module,
            int teeth,
            double pressureAngleRadians,
            double addendumFactor,
            double dedendumFactor,
            double maxDeviation)
        {
            EnsureInternalMesh(module, teeth, pressureAngleRadians, addendumFactor);
            return CreateGear(
                center, module, teeth, pressureAngleRadians,
                dedendumFactor, addendumFactor, maxDeviation);
        }

        public static List<Curve2D> CreateInternalTooth(
            Vec2D center,
            double module,
            int teeth,
            double pressureAngleRadians,
            double addendumFactor,
            double dedendumFactor,
            double maxDeviation,
            double toothRotation)
        {
            EnsureInternalMesh(module, teeth, pressureAngleRadians, addendumFactor);
            return CreateTooth(
                center, module, teeth, pressureAngleRadians,
                dedendumFactor, addendumFactor, maxDeviation, toothRotation);
        }

        static void EnsureInternalMesh(
            double module, int teeth, double pressureAngleRadians, double addendumFactor)
        {
            if (module <= 0)
                throw new ArgumentOutOfRangeException(nameof(module));
            if (teeth < 18)
                throw new ArgumentOutOfRangeException(nameof(teeth), "Internal gears need at least 18 teeth.");
            double rp = 0.5 * module * teeth;
            double rb = rp * Math.Cos(pressureAngleRadians);
            if (rb <= 1e-12)
                throw new ArgumentException("Base radius vanished; pressure angle too large.");
            double ra = rp - addendumFactor * module;
            if (ra <= rb * 1.01)
                throw new ArgumentException("Internal tip is inside the base circle; use more teeth or less addendum.");
        }

        static CubicHermiteSpline2D MirrorHermite(CubicHermiteSpline2D src, Vec2D origin, Vec2D axis)
        {
            var pts = new List<Vec2D>(src.Points.Count);
            for (int i = src.Points.Count - 1; i >= 0; i--)
            {
                Vec2D l = GeometricAlgorithms.ProjectPointOntoLine(src.Points[i], origin, axis);
                pts.Add(l + (l - src.Points[i]));
            }
            return new CubicHermiteSpline2D(pts, null, null, src.Flags);
        }

        static void AppendRootAndFillet(
            List<Curve2D> curves, Vec2D center, double rf, double aGap, Vec2D foot, double gapSign, double fillet)
        {
            Vec2D g;
            Curve2D join = MakeFillet(center, rf, foot, gapSign, fillet, out g);
            double aG = Angle(center, g);
            Vec2D sector = Polar(center, rf, aGap);
            if ((g - sector).LengthSquared() > 1e-16)
                curves.Add(new Arc2D(sector, ArcMid(center, rf, aGap, aG), g));
            if (join != null)
                curves.Add(join);
        }

        static void AppendFilletAndRoot(
            List<Curve2D> curves, Vec2D center, double rf, Vec2D foot, double aGap, double gapSign, double fillet)
        {
            Vec2D g;
            Curve2D join = MakeFillet(center, rf, foot, gapSign, fillet, out g);
            if (join != null)
                curves.Add(join.Reverse());
            double aG = Angle(center, g);
            Vec2D sector = Polar(center, rf, aGap);
            if ((sector - g).LengthSquared() > 1e-16)
                curves.Add(new Arc2D(g, ArcMid(center, rf, aG, aGap), sector));
        }

        static Curve2D MakeFillet(
            Vec2D center, double rf, Vec2D foot, double gapSign, double fillet, out Vec2D rootPoint)
        {
            double aFoot = Angle(center, foot);
            rootPoint = Polar(center, rf, aFoot);
            double lift = (foot - center).Length() - rf;
            if (lift <= 1e-8)
                return null;

            double rho = Math.Min(fillet, 0.9 * lift);
            Vec2D c1;
            Vec2D c2;
            if (rho > 1e-8 &&
                GeometricAlgorithms.CircleCircleIntersection(center, rf + rho, foot, rho, out c1, out c2))
            {
                Vec2D radial = foot - center;
                Vec2D towardGap = gapSign * new Vec2D(-radial.Y, radial.X);
                Vec2D c = ((c1 - foot).X * towardGap.X + (c1 - foot).Y * towardGap.Y) >=
                          ((c2 - foot).X * towardGap.X + (c2 - foot).Y * towardGap.Y)
                    ? c1
                    : c2;
                double aC = Angle(center, c);
                rootPoint = Polar(center, rf, aC);
                Vec2D mid = c + rho * ((0.5 * (rootPoint + foot) - c).Normalized());
                if ((rootPoint - foot).LengthSquared() > 1e-16 && (mid - rootPoint).LengthSquared() > 1e-18)
                    return new Arc2D(rootPoint, mid, foot);
            }

            if ((rootPoint - foot).LengthSquared() <= 1e-16)
                return null;
            return new Line2D(rootPoint, foot);
        }

        static Vec2D Rotate(Vec2D center, Vec2D local, double rotation)
        {
            double c = Math.Cos(rotation);
            double s = Math.Sin(rotation);
            return center + new Vec2D(c * local.X - s * local.Y, s * local.X + c * local.Y);
        }

        static Vec2D Polar(Vec2D center, double r, double a)
        {
            return center + new Vec2D(r * Math.Cos(a), r * Math.Sin(a));
        }

        static double Angle(Vec2D center, Vec2D p)
        {
            return Math.Atan2(p.Y - center.Y, p.X - center.X);
        }

        static double PolarOfLocal(double t)
        {
            double c = Math.Cos(t);
            double s = Math.Sin(t);
            return Math.Atan2(s - t * c, c + t * s);
        }

        static double WrapDelta(double a0, double a1)
        {
            double d = a1 - a0;
            while (d < 0)
                d += 2.0 * Math.PI;
            while (d > 2.0 * Math.PI)
                d -= 2.0 * Math.PI;
            return d;
        }

        static Vec2D ArcMid(Vec2D center, double r, double a0, double a1)
        {
            double d = a1 - a0;
            while (d < 0)
                d += 2.0 * Math.PI;
            while (d > 2.0 * Math.PI)
                d -= 2.0 * Math.PI;
            return Polar(center, r, a0 + 0.5 * d);
        }
    }
}
