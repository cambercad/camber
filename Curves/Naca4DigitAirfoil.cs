using System;
using System.Collections.Generic;
using GeoCore;

namespace Curves
{
    /// <summary>
    /// NACA 4-digit airfoil definition: maximum camber (first digit, % chord), position of max camber
    /// (second digit, tenths of chord), and maximum thickness (last two digits, % chord).
    /// Geometry follows the standard thickness + mean-line construction (e.g. Abbott & von Doenhoff).
    /// </summary>
    public readonly struct Naca4DigitSpec : IEquatable<Naca4DigitSpec>
    {
        /// <summary>Maximum camber as fraction of chord (e.g. 0.02 for digit 2).</summary>
        public double MaxCamber { get; init; }
        /// <summary>Chordwise position of maximum camber, 0–1 (e.g. 0.4 for digit 4).</summary>
        public double MaxCamberPosition { get; init; }
        /// <summary>Maximum thickness as fraction of chord (e.g. 0.12 for digits 12).</summary>
        public double MaxThickness { get; init; }

        public Naca4DigitSpec(double maxCamber, double maxCamberPosition, double maxThickness)
        {
            MaxCamber = maxCamber;
            MaxCamberPosition = maxCamberPosition;
            MaxThickness = maxThickness;
        }

        /// <summary>
        /// Parse integer 0–9999 as four digits (leading zeros implied), e.g. 12 → 0012, 2412 → 2412.
        /// </summary>
        public static Naca4DigitSpec FromCode(int code)
        {
            if (code < 0 || code > 9999)
                throw new ArgumentOutOfRangeException(nameof(code), "NACA code must be in 0..9999.");
            int d0 = (code / 1000) % 10;
            int d1 = (code / 100) % 10;
            int d2 = (code / 10) % 10;
            int d3 = code % 10;
            double m = d0 / 100.0;
            double p = d1 / 10.0;
            double t = (d2 * 10 + d3) / 100.0;
            if (m > 1e-12 && (p <= 0 || p >= 1))
                throw new ArgumentException($"Invalid camber position p={p} for non-zero camber m={m}.");
            return new Naca4DigitSpec(m, p, t);
        }

        public static bool TryParse(ReadOnlySpan<char> text, out Naca4DigitSpec spec)
        {
            spec = default;
            Span<int> d = stackalloc int[4];
            int n = 0;
            foreach (char c in text)
            {
                if (char.IsDigit(c))
                {
                    if (n < 4)
                        d[n++] = c - '0';
                    else
                    {
                        d[0] = d[1]; d[1] = d[2]; d[2] = d[3]; d[3] = c - '0';
                    }
                }
            }
            while (n < 4)
            {
                for (int i = n; i > 0; i--)
                    d[i] = d[i - 1];
                d[0] = 0;
                n++;
            }
            if (n != 4) return false;
            int code = d[0] * 1000 + d[1] * 100 + d[2] * 10 + d[3];
            try
            {
                spec = FromCode(code);
                return true;
            }
            catch
            {
                return false;
            }
        }

        public static Naca4DigitSpec Parse(string text)
        {
            if (text == null || !TryParse(text.AsSpan(), out var spec))
                throw new FormatException("Expected a four-digit NACA code (e.g. \"2412\" or \"NACA 0012\").");
            return spec;
        }

        public bool Equals(Naca4DigitSpec other) =>
            MaxCamber == other.MaxCamber && MaxCamberPosition == other.MaxCamberPosition && MaxThickness == other.MaxThickness;

        public override bool Equals(object obj) => obj is Naca4DigitSpec other && Equals(other);
        public override int GetHashCode() => HashCode.Combine(MaxCamber, MaxCamberPosition, MaxThickness);
    }

    /// <summary>
    /// Evaluates NACA 4-digit coordinates in normalized chord space (LE at origin, chord on +x to 1).
    /// Use <see cref="BuildHermiteSplineData"/> with <see cref="PlotterSketcher.AddNaca4DigitAirfoil"/> for sketch splines.
    /// Default sketch path trims the last slice of chord at the TE and closes with a short segment (see <see cref="DefaultTrailingEdgeTrimChordFraction"/>).
    /// </summary>
    public static class Naca4DigitAirfoil
    {
        /// <summary>Fraction of chord length omitted at the trailing edge (TE) when building sketch splines; avoids near–zero-thickness TE slivers.</summary>
        public const double DefaultTrailingEdgeTrimChordFraction = 0.01;
        /// <summary>
        /// Half-thickness y<sub>t</sub>(x) / chord from the standard 4-term + sqrt closure polynomial (t as max thickness fraction).
        /// </summary>
        public static double ThicknessY(double x, double thicknessFraction)
        {
            if (x <= 0) return 0;
            if (x > 1) x = 1;
            double sx = Math.Sqrt(x);
            // -0.1036 (not -0.1015) closes the trailing edge so upper and lower meet at x = 1 (common NACA practice).
            double poly = 0.2969 * sx - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x;
            return (thicknessFraction / 0.2) * poly;
        }

        /// <summary>d(thickness)/dx for the same polynomial as <see cref="ThicknessY"/>; x must be in (0, 1].</summary>
        public static double ThicknessDerivativeY(double x, double thicknessFraction)
        {
            if (x <= 0)
                throw new ArgumentOutOfRangeException(nameof(x));
            if (x > 1) x = 1;
            double invSqrt = 1.0 / Math.Sqrt(x);
            double dpoly = 0.2969 * 0.5 * invSqrt - 0.1260 - 0.7032 * x + 0.8529 * x * x - 0.4144 * x * x * x;
            return (thicknessFraction / 0.2) * dpoly;
        }

        /// <summary>Second derivative of the mean line w.r.t. chordwise x (0 at max camber position has a curvature jump).</summary>
        public static double MeanLineSecondDerivative(in Naca4DigitSpec spec, double x)
        {
            double m = spec.MaxCamber;
            if (m <= 1e-15)
                return 0;
            double p = spec.MaxCamberPosition;
            if (x < p)
                return -2 * m / (p * p);
            return -2 * m / ((1 - p) * (1 - p));
        }

        /// <summary>Derivative of upper-surface chord coordinates w.r.t. chordwise station x (LE→TE); x ∈ (0, 1].</summary>
        public static void UpperChordDerivativeWrtX(in Naca4DigitSpec spec, double x, out double dxuDx, out double dyuDx)
        {
            if (x <= 0)
                throw new ArgumentOutOfRangeException(nameof(x));
            if (x > 1) x = 1;
            double yt = ThicknessY(x, spec.MaxThickness);
            double dyt = ThicknessDerivativeY(x, spec.MaxThickness);
            MeanLineAndDerivative(spec, x, out _, out double dyc);
            double d2yc = MeanLineSecondDerivative(spec, x);
            double theta = Math.Atan(dyc);
            double cosT = Math.Cos(theta);
            double sinT = Math.Sin(theta);
            double dTheta = d2yc / (1 + dyc * dyc);
            dxuDx = 1 - (dyt * sinT + yt * cosT * dTheta);
            dyuDx = dyc + (dyt * cosT - yt * sinT * dTheta);
        }

        /// <summary>Derivative of lower-surface chord coordinates w.r.t. chordwise station x; x ∈ (0, 1].</summary>
        public static void LowerChordDerivativeWrtX(in Naca4DigitSpec spec, double x, out double dxlDx, out double dylDx)
        {
            if (x <= 0)
                throw new ArgumentOutOfRangeException(nameof(x));
            if (x > 1) x = 1;
            double yt = ThicknessY(x, spec.MaxThickness);
            double dyt = ThicknessDerivativeY(x, spec.MaxThickness);
            MeanLineAndDerivative(spec, x, out _, out double dyc);
            double d2yc = MeanLineSecondDerivative(spec, x);
            double theta = Math.Atan(dyc);
            double cosT = Math.Cos(theta);
            double sinT = Math.Sin(theta);
            double dTheta = d2yc / (1 + dyc * dyc);
            dxlDx = 1 + (dyt * sinT + yt * cosT * dTheta);
            dylDx = dyc - (dyt * cosT - yt * sinT * dTheta);
        }

        public static void MeanLineAndDerivative(in Naca4DigitSpec spec, double x, out double yc, out double dycDx)
        {
            double m = spec.MaxCamber;
            double p = spec.MaxCamberPosition;
            if (m <= 1e-15)
            {
                yc = 0;
                dycDx = 0;
                return;
            }

            if (x < p)
            {
                double inv = 1.0 / (p * p);
                yc = m * inv * (2 * p * x - x * x);
                dycDx = (2 * m * inv) * (p - x);
            }
            else
            {
                double om = 1 - p;
                double inv = 1.0 / (om * om);
                yc = m * inv * ((1 - 2 * p) + 2 * p * x - x * x);
                dycDx = (2 * m * inv) * (p - x);
            }
        }

        /// <summary>Upper surface (x<sub>u</sub>, y<sub>u</sub>) in chord coordinates; x is chordwise station 0–1.</summary>
        public static void UpperChordCoords(in Naca4DigitSpec spec, double x, out double xu, out double yu)
        {
            if (x <= 0) { xu = 0; yu = 0; return; }
            if (x > 1) x = 1;
            double yt = ThicknessY(x, spec.MaxThickness);
            MeanLineAndDerivative(spec, x, out double yc, out double dycDx);
            double theta = Math.Atan(dycDx);
            xu = x - yt * Math.Sin(theta);
            yu = yc + yt * Math.Cos(theta);
        }

        /// <summary>Lower surface (x<sub>l</sub>, y<sub>l</sub>) in chord coordinates.</summary>
        public static void LowerChordCoords(in Naca4DigitSpec spec, double x, out double xl, out double yl)
        {
            if (x <= 0) { xl = 0; yl = 0; return; }
            if (x > 1) x = 1;
            double yt = ThicknessY(x, spec.MaxThickness);
            MeanLineAndDerivative(spec, x, out double yc, out double dycDx);
            double theta = Math.Atan(dycDx);
            xl = x + yt * Math.Sin(theta);
            yl = yc - yt * Math.Cos(theta);
        }

        /// <summary>
        /// Unit tangent in chord frame along the surface as chordwise station x increases (LE → TE on that surface).
        /// </summary>
        public static Vec2D ChordTangentUpper(in Naca4DigitSpec spec, double x)
        {
            const double h = 2e-5;
            double xm = Math.Max(0, x - h);
            double xp = Math.Min(1, x + h);
            if (xp <= xm) xp = Math.Min(1, xm + 1e-6);
            UpperChordCoords(spec, xm, out double x0, out double y0);
            UpperChordCoords(spec, xp, out double x1, out double y1);
            double dx = x1 - x0;
            double dy = y1 - y0;
            var v = new Vec2D(dx, dy);
            double len = v.Length();
            if (len < 1e-18) return new Vec2D(1, 0);
            return v / len;
        }

        public static Vec2D ChordTangentLower(in Naca4DigitSpec spec, double x)
        {
            const double h = 2e-5;
            double xm = Math.Max(0, x - h);
            double xp = Math.Min(1, x + h);
            if (xp <= xm) xp = Math.Min(1, xm + 1e-6);
            LowerChordCoords(spec, xm, out double x0, out double y0);
            LowerChordCoords(spec, xp, out double x1, out double y1);
            double dx = x1 - x0;
            double dy = y1 - y0;
            var v = new Vec2D(dx, dy);
            double len = v.Length();
            if (len < 1e-18) return new Vec2D(1, 0);
            return v / len;
        }

        /// <summary>Map chord-frame (x, y) to sketch 2D: scale by chord, rotate by angle, translate LE.</summary>
        public static Vec2D ToSketch(Vec2D chordXY, Vec2D leadingEdge, double chordLength, double chordAngleRadians)
        {
            double c = chordLength;
            double X = chordXY.X * c;
            double Y = chordXY.Y * c;
            double ca = Math.Cos(chordAngleRadians);
            double sa = Math.Sin(chordAngleRadians);
            return new Vec2D(leadingEdge.X + ca * X - sa * Y, leadingEdge.Y + sa * X + ca * Y);
        }

        public static Vec2D TransformTangent(Vec2D chordUnitTangent, double chordLength, double chordAngleRadians)
        {
            double X = chordUnitTangent.X * chordLength;
            double Y = chordUnitTangent.Y * chordLength;
            double ca = Math.Cos(chordAngleRadians);
            double sa = Math.Sin(chordAngleRadians);
            return new Vec2D(ca * X - sa * Y, sa * X + ca * Y);
        }

        /// <summary>
        /// Blends upper (LE→TE) and lower (TE→LE) analytic tangent directions at the first interior chord station so both
        /// Hermite splines use the same tangent <em>line</em> at the LE. Default chord tangents at LE are mirror images in y
        /// and are not collinear, which produces a visible corner even though both curves pass through the same point.
        /// </summary>
        private static Vec2D BlendLeadingEdgeTangentDirections(Vec2D upperOutboundSketch, Vec2D lowerInboundSketch)
        {
            double lu = upperOutboundSketch.Length(), li = lowerInboundSketch.Length();
            if (lu < 1e-18)
                return lowerInboundSketch;
            if (li < 1e-18)
                return upperOutboundSketch;
            Vec2D bu = upperOutboundSketch * (1.0 / lu);
            Vec2D bi = lowerInboundSketch * (1.0 / li);
            Vec2D sum = bu + bi;
            double ls = sum.Length();
            if (ls < 1e-10)
                return upperOutboundSketch;
            return sum;
        }

        /// <summary>
        /// Cosine-spaced x in [0, 1] with clustering at LE and TE (common for airfoil panels).
        /// </summary>
        public static void CosineChordwiseSamples(int count, List<double> dst)
        {
            dst.Clear();
            if (count < 2)
                throw new ArgumentOutOfRangeException(nameof(count));
            for (int i = 0; i < count; i++)
            {
                double theta = Math.PI * i / (count - 1);
                dst.Add(0.5 * (1 - Math.Cos(theta)));
            }
        }

        /// <summary>
        /// Bisection on chordwise station t ∈ [0,1] so that <see cref="UpperChordCoords"/>(t) has chord-frame x equal to
        /// <paramref name="targetChordwiseX"/> (the “vertical” cut line in standard airfoil coordinates). Typical 4-digit shapes are monotone in x<sub>u</sub>(t).
        /// </summary>
        public static double SolveUpperStationForChordwiseX(in Naca4DigitSpec spec, double targetChordwiseX, double absTol = 1e-12)
        {
            if (targetChordwiseX <= 0)
                return 0;
            UpperChordCoords(spec, 1, out double xuTe, out _);
            if (targetChordwiseX >= xuTe - absTol)
                return 1;

            double lo = 0, hi = 1;
            UpperChordCoords(spec, lo, out double xLo, out _);
            if (targetChordwiseX <= xLo + absTol)
                return lo;

            for (int iter = 0; iter < 80; iter++)
            {
                double mid = 0.5 * (lo + hi);
                UpperChordCoords(spec, mid, out double xu, out _);
                if (Math.Abs(xu - targetChordwiseX) <= absTol)
                    return mid;
                if (xu < targetChordwiseX)
                    lo = mid;
                else
                    hi = mid;
                if (hi - lo < 1e-15)
                    break;
            }
            return 0.5 * (lo + hi);
        }

        /// <summary>
        /// Bisection on chordwise station t so that <see cref="LowerChordCoords"/>(t) has chord-frame x equal to <paramref name="targetChordwiseX"/>.
        /// </summary>
        public static double SolveLowerStationForChordwiseX(in Naca4DigitSpec spec, double targetChordwiseX, double absTol = 1e-12)
        {
            if (targetChordwiseX <= 0)
                return 0;
            LowerChordCoords(spec, 1, out double xlTe, out _);
            if (targetChordwiseX >= xlTe - absTol)
                return 1;

            double lo = 0, hi = 1;
            LowerChordCoords(spec, lo, out double xLo, out _);
            if (targetChordwiseX <= xLo + absTol)
                return lo;

            for (int iter = 0; iter < 80; iter++)
            {
                double mid = 0.5 * (lo + hi);
                LowerChordCoords(spec, mid, out double xl, out _);
                if (Math.Abs(xl - targetChordwiseX) <= absTol)
                    return mid;
                if (xl < targetChordwiseX)
                    lo = mid;
                else
                    hi = mid;
                if (hi - lo < 1e-15)
                    break;
            }
            return 0.5 * (lo + hi);
        }

        /// <summary>
        /// Builds upper (LE→trim) and lower (trim→LE) knot positions in sketch space.
        /// When <paramref name="useAnalyticKnotTangents"/> is true, also returns per-knot tangent directions for
        /// <see cref="CubicHermiteSpline2D"/> (exact d(sketch)/d(chordwise x) from the NACA formulas). LE knots on upper and
        /// lower share a blended direction so the two splines are G¹ at the leading edge (same tangent line at the shared point).
        /// </summary>
        /// <param name="trailingEdgeTrimChordFraction">If positive, TE is cut at chord-frame x = 1 − value using bisection on the analytic contour; cosine samples span [0, s<sub>up</sub>] / [0, s<sub>lo</sub>] on each surface.</param>
        public static void BuildHermiteSplineData(
            in Naca4DigitSpec spec,
            Vec2D leadingEdge,
            double chordLength,
            double chordAngleRadians,
            int samplesPerSide,
            bool useAnalyticKnotTangents,
            double trailingEdgeTrimChordFraction,
            out List<Vec2D> upperPoints,
            out List<Vec2D> lowerPoints,
            out List<Vec2D?> upperKnotTangentDirections,
            out List<Vec2D?> lowerKnotTangentDirections)
        {
            if (samplesPerSide < 3)
                throw new ArgumentOutOfRangeException(nameof(samplesPerSide), "Use at least 3 samples per side.");
            if (chordLength <= 0)
                throw new ArgumentOutOfRangeException(nameof(chordLength));
            if (trailingEdgeTrimChordFraction < 0 || trailingEdgeTrimChordFraction >= 1)
                throw new ArgumentOutOfRangeException(nameof(trailingEdgeTrimChordFraction), "Must be in [0, 1).");

            var unitCos = new List<double>();
            CosineChordwiseSamples(samplesPerSide, unitCos);

            double sUp = 1, sLo = 1;
            if (trailingEdgeTrimChordFraction > 0)
            {
                double targetChordX = 1.0 - trailingEdgeTrimChordFraction;
                sUp = SolveUpperStationForChordwiseX(spec, targetChordX);
                sLo = SolveLowerStationForChordwiseX(spec, targetChordX);
            }

            var upperStations = new List<double>(samplesPerSide);
            for (int i = 0; i < samplesPerSide; i++)
                upperStations.Add(sUp * unitCos[i]);

            var xs = new List<double>(samplesPerSide);
            for (int i = 0; i < samplesPerSide; i++)
                xs.Add(sLo * unitCos[i]);

            upperPoints = new List<Vec2D>(samplesPerSide);
            lowerPoints = new List<Vec2D>(samplesPerSide);
            for (int i = 0; i < samplesPerSide; i++)
            {
                double x = upperStations[i];
                UpperChordCoords(spec, x, out double xu, out double yu);
                upperPoints.Add(ToSketch(new Vec2D(xu, yu), leadingEdge, chordLength, chordAngleRadians));
            }

            for (int i = samplesPerSide - 1; i >= 0; i--)
            {
                double x = xs[i];
                LowerChordCoords(spec, x, out double xl, out double yl);
                lowerPoints.Add(ToSketch(new Vec2D(xl, yl), leadingEdge, chordLength, chordAngleRadians));
            }

            if (!useAnalyticKnotTangents)
            {
                upperKnotTangentDirections = null;
                lowerKnotTangentDirections = null;
                return;
            }

            double x1 = upperStations[1];
            UpperChordDerivativeWrtX(spec, x1, out double leDux, out double leDuy);
            LowerChordDerivativeWrtX(spec, sLo * unitCos[1], out double leDxl, out double leDyl);
            Vec2D upperOutbound = TransformTangent(new Vec2D(leDux, leDuy), chordLength, chordAngleRadians);
            Vec2D lowerInbound = -TransformTangent(new Vec2D(leDxl, leDyl), chordLength, chordAngleRadians);
            Vec2D leSharedTangent = BlendLeadingEdgeTangentDirections(upperOutbound, lowerInbound);

            upperKnotTangentDirections = new List<Vec2D?>(samplesPerSide);
            for (int i = 0; i < samplesPerSide; i++)
            {
                if (i == 0)
                {
                    upperKnotTangentDirections.Add(leSharedTangent);
                    continue;
                }

                double x = upperStations[i];
                UpperChordDerivativeWrtX(spec, x, out double dxu, out double dyu);
                upperKnotTangentDirections.Add(TransformTangent(new Vec2D(dxu, dyu), chordLength, chordAngleRadians));
            }

            lowerKnotTangentDirections = new List<Vec2D?>(samplesPerSide);
            for (int k = 0; k < samplesPerSide; k++)
            {
                if (k == samplesPerSide - 1)
                {
                    lowerKnotTangentDirections.Add(leSharedTangent);
                    continue;
                }

                double x = xs[samplesPerSide - 1 - k];
                LowerChordDerivativeWrtX(spec, x, out double dxl, out double dyl);
                Vec2D dSketch = TransformTangent(new Vec2D(dxl, dyl), chordLength, chordAngleRadians);
                // Spline parameter increases TE→LE while chordwise x decreases → negate d(sketch)/dx.
                lowerKnotTangentDirections.Add(-dSketch);
            }
        }
    }

    /// <summary>Upper and lower <see cref="CubicHermiteSpline2D"/> curves forming a closed NACA 4-digit loop (upper, optional TE closure line, lower).</summary>
    public readonly struct Naca4AirfoilSplines
    {
        public CubicHermiteSpline2D Upper { get; init; }
        public CubicHermiteSpline2D Lower { get; init; }
        /// <summary>Segment from upper-surface TE trim to lower-surface TE trim (thickness direction in chord frame); null if trim is zero or endpoints coincide.</summary>
        public Line2D TeClosure { get; init; }
    }
}
