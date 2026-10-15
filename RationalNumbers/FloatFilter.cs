using System.Runtime.CompilerServices;

namespace GeoCore;

/// <summary>
/// Error-bounded floating point evaluation of exact predicates. Every coordinate is a
/// cached double within <see cref="CoordinateError"/> (relative) of its exact value. Each
/// operation propagates a rigorous absolute error bound, so a result whose magnitude
/// exceeds twice its bound has the sign the exact computation would give. Everything
/// else (including every exact zero) is reported as undecided and must use exact arithmetic.
/// Nothing is ever quantized or substituted: this only avoids exact work whose answer is clear.
/// </summary>
public static class FloatFilter
{
    // Cached conversion: numerator, denominator and quotient each round once (and the lower
    // limbs of huge values add under 2^-64): below 6 * 2^-53 = 6.7e-16. 1e-15 leaves margin.
    private const double CoordinateError = 1e-15;
    // One rounded operation: 2^-53 = 1.1e-16. Doubled for margin.
    private const double RoundingError = 2.3e-16;
    private const double MaxMagnitude = 1e40;
    private const double MinMagnitude = 1e-30;

    private readonly struct Approx
    {
        public readonly double Value;
        public readonly double Error;

        public Approx(double value, double error)
        {
            Value = value;
            Error = error;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Approx operator -(Approx a, Approx b)
        {
            double v = a.Value - b.Value;
            return new Approx(v, a.Error + b.Error + RoundingError * Math.Abs(v));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Approx operator +(Approx a, Approx b)
        {
            double v = a.Value + b.Value;
            return new Approx(v, a.Error + b.Error + RoundingError * Math.Abs(v));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Approx operator *(Approx a, Approx b)
        {
            double v = a.Value * b.Value;
            double e = Math.Abs(a.Value) * b.Error + Math.Abs(b.Value) * a.Error + a.Error * b.Error
                + RoundingError * Math.Abs(v);
            return new Approx(v, e);
        }

        /// <summary>-1, 0 or 1 when the sign is certain; 2 when it is not.</summary>
        public int CertainSign()
        {
            double magnitude = Math.Abs(Value);
            if (!(magnitude > 2 * Error)) // also rejects NaN
                return 2;
            return Value > 0 ? 1 : -1;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool Load(in BigRationalHybrid coordinate, out Approx approx)
    {
        approx = default;
        if (!coordinate.TryApproximate(out double v))
            return false;
        double magnitude = Math.Abs(v);
        if (magnitude != 0 && (magnitude < MinMagnitude || magnitude > MaxMagnitude))
            return false;
        approx = new Approx(v, CoordinateError * magnitude);
        return true;
    }

    /// <summary>
    /// Sign of the orientation determinant (same formula and sign convention as
    /// <see cref="BigRationalHybrid.SignOfOrient3D"/>). False when undecided.
    /// </summary>
    public static bool TryOrient3DSign(
        in BigRationalHybrid pax, in BigRationalHybrid pay, in BigRationalHybrid paz,
        in BigRationalHybrid pbx, in BigRationalHybrid pby, in BigRationalHybrid pbz,
        in BigRationalHybrid pcx, in BigRationalHybrid pcy, in BigRationalHybrid pcz,
        in BigRationalHybrid pdx, in BigRationalHybrid pdy, in BigRationalHybrid pdz,
        out int sign)
    {
        sign = 0;
        if (!Load(in pax, out var ax) || !Load(in pay, out var ay) || !Load(in paz, out var az) ||
            !Load(in pbx, out var bx) || !Load(in pby, out var by) || !Load(in pbz, out var bz) ||
            !Load(in pcx, out var cx) || !Load(in pcy, out var cy) || !Load(in pcz, out var cz) ||
            !Load(in pdx, out var dx) || !Load(in pdy, out var dy) || !Load(in pdz, out var dz))
            return false;

        Approx adx = ax - dx, ady = ay - dy, adz = az - dz;
        Approx bdx = bx - dx, bdy = by - dy, bdz = bz - dz;
        Approx cdx = cx - dx, cdy = cy - dy, cdz = cz - dz;
        Approx determinant = adz * (bdx * cdy - cdx * bdy)
            + bdz * (cdx * ady - adx * cdy)
            - cdz * (bdx * ady - adx * bdy);
        int certain = determinant.CertainSign();
        if (certain == 2)
            return false;
        sign = certain;
        return true;
    }

    /// <summary>
    /// Sign of (a - c) x (b - c), i.e. the orientation of the 2D triangle (a, b, c); same convention
    /// as <see cref="Rat2Hybrid.Orient2DSign"/>. False when undecided.
    /// </summary>
    public static bool TryOrient2DSign(
        in BigRationalHybrid pax, in BigRationalHybrid pay,
        in BigRationalHybrid pbx, in BigRationalHybrid pby,
        in BigRationalHybrid pcx, in BigRationalHybrid pcy,
        out int sign)
    {
        sign = 0;
        if (!Load(in pax, out var ax) || !Load(in pay, out var ay) ||
            !Load(in pbx, out var bx) || !Load(in pby, out var by) ||
            !Load(in pcx, out var cx) || !Load(in pcy, out var cy))
            return false;

        Approx acx = ax - cx, bcx = bx - cx, acy = ay - cy, bcy = by - cy;
        Approx determinant = acx * bcy - acy * bcx;
        int certain = determinant.CertainSign();
        if (certain == 2)
            return false;
        sign = certain;
        return true;
    }

    /// <summary>
    /// Sign of the in-circle determinant (positive: d inside the circle through a, b, c in
    /// counterclockwise order), same formula as the Delaunay predicate. False when undecided.
    /// </summary>
    public static bool TryInCircleSign(
        in BigRationalHybrid pax, in BigRationalHybrid pay,
        in BigRationalHybrid pbx, in BigRationalHybrid pby,
        in BigRationalHybrid pcx, in BigRationalHybrid pcy,
        in BigRationalHybrid pdx, in BigRationalHybrid pdy,
        out int sign)
    {
        sign = 0;
        if (!Load(in pax, out var ax) || !Load(in pay, out var ay) ||
            !Load(in pbx, out var bx) || !Load(in pby, out var by) ||
            !Load(in pcx, out var cx) || !Load(in pcy, out var cy) ||
            !Load(in pdx, out var dx) || !Load(in pdy, out var dy))
            return false;

        Approx adx = ax - dx, ady = ay - dy, bdx = bx - dx, bdy = by - dy, cdx = cx - dx, cdy = cy - dy;
        Approx abdet = adx * bdy - bdx * ady;
        Approx bcdet = bdx * cdy - cdx * bdy;
        Approx cadet = cdx * ady - adx * cdy;
        Approx alift = adx * adx + ady * ady;
        Approx blift = bdx * bdx + bdy * bdy;
        Approx clift = cdx * cdx + cdy * cdy;
        Approx result = alift * bcdet + blift * cadet + clift * abdet;
        int certain = result.CertainSign();
        if (certain == 2)
            return false;
        sign = certain;
        return true;
    }
}
