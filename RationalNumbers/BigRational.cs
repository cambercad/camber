using System.Numerics;
using System.Runtime.CompilerServices;

namespace GeoCore;

/// <summary>
/// Exact rational arithmetic backed by Camber's arbitrary-precision integer.
/// Fractions are kept unreduced during arithmetic; call <see cref="Simplify"/>
/// when a canonical numerator and denominator are useful.
/// </summary>
public struct BigRational : IComparable<BigRational>, IComparable, IEquatable<BigRational>
{
    private FastBigInteger numerator;
    private FastBigInteger denominator;
    private static readonly BigInteger DoubleMaximum = (BigInteger)double.MaxValue;
    private static readonly BigInteger DoubleMinimum = -DoubleMaximum;
    private static readonly BigInteger DoubleScale = BigInteger.Pow(10, 308);

    internal FastBigInteger FastNumerator => numerator;
    internal readonly FastBigInteger FastDenominator => EffectiveDenominator;
    private readonly FastBigInteger EffectiveDenominator => denominator.IsZero ? new FastBigInteger(1) : denominator;

    public BigInteger Numerator
    {
        readonly get => numerator.ToBigInteger();
        set => numerator = new FastBigInteger(value);
    }

    public BigInteger Denominator
    {
        readonly get => EffectiveDenominator.ToBigInteger();
        set
        {
            var valueFast = new FastBigInteger(value);
            if (valueFast.IsZero) throw new DivideByZeroException();
            if (valueFast.Sign < 0)
            {
                numerator = -numerator;
                valueFast = -valueFast;
            }
            denominator = valueFast;
        }
    }

    public readonly int Sign => numerator.Sign;
    public static BigRational Zero => new(0);
    public static BigRational One => new(1);
    public static BigRational MinusOne => new(-1);

    public BigRational(BigInteger numerator)
        : this(new FastBigInteger(numerator), new FastBigInteger(1)) { }

    public BigRational(long numerator)
        : this(new FastBigInteger(numerator), new FastBigInteger(1)) { }

    public BigRational(int numerator)
        : this(new FastBigInteger(numerator), new FastBigInteger(1)) { }

    public BigRational(BigInteger numerator, BigInteger denominator)
        : this(new FastBigInteger(numerator), new FastBigInteger(denominator)) { }

    public BigRational(BigInteger whole, BigInteger numerator, BigInteger denominator)
        : this(new BigRational(whole) + new BigRational(numerator, denominator)) { }

    public BigRational(double value)
    {
        if (!double.IsFinite(value))
            throw new ArgumentException("Argument must be a finite number.", nameof(value));

        ulong bits = BitConverter.DoubleToUInt64Bits(value);
        bool negative = (bits >> 63) != 0;
        int exponentBits = (int)((bits >> 52) & 0x7ff);
        ulong significand = bits & 0x000f_ffff_ffff_ffffUL;
        int exponent;
        if (exponentBits == 0)
            exponent = -1074;
        else
        {
            significand |= 1UL << 52;
            exponent = exponentBits - 1023 - 52;
        }

        if (significand == 0)
        {
            numerator = default;
            denominator = new FastBigInteger(1);
            return;
        }

        var n = new FastBigInteger(new BigInteger(significand));
        if (negative) n = -n;
        var d = new FastBigInteger(1);
        if (exponent >= 0)
            n = new FastBigInteger(n.ToBigInteger() << exponent);
        else
            d = new FastBigInteger(BigInteger.One << -exponent);

        numerator = n;
        denominator = d;
        Simplify();
    }

    public BigRational(decimal value)
    {
        int[] bits = decimal.GetBits(value);
        BigInteger n = ((BigInteger)(uint)bits[2] << 64) |
                       ((BigInteger)(uint)bits[1] << 32) | (uint)bits[0];
        if ((bits[3] & int.MinValue) != 0) n = -n;
        int scale = (bits[3] >> 16) & 0xff;
        numerator = new FastBigInteger(n);
        denominator = new FastBigInteger(BigInteger.Pow(10, scale));
        Simplify();
    }

    private BigRational(FastBigInteger numerator, FastBigInteger denominator)
    {
        if (denominator.IsZero) throw new DivideByZeroException();
        if (denominator.Sign < 0)
        {
            numerator = -numerator;
            denominator = -denominator;
        }
        this.numerator = numerator;
        this.denominator = numerator.IsZero ? new FastBigInteger(1) : denominator;
    }

    private BigRational(BigRational value)
    {
        numerator = value.numerator;
        denominator = value.EffectiveDenominator;
    }

    public void Simplify()
    {
        if (numerator.IsZero)
        {
            denominator = new FastBigInteger(1);
            return;
        }

        var den = EffectiveDenominator;
        var gcd = FastBigInteger.GreatestCommonDivisor(numerator, den);
        if (gcd != new FastBigInteger(1))
        {
            numerator /= gcd;
            denominator = den / gcd;
        }
        else
            denominator = den;
    }

    public readonly BigInteger GetWholePart() => numerator.ToBigInteger() / EffectiveDenominator.ToBigInteger();

    public readonly BigRational GetFractionPart()
    {
        var remainder = numerator % EffectiveDenominator;
        return new BigRational(remainder, EffectiveDenominator);
    }

    public readonly int CompareTo(BigRational other) => Compare(this, other);

    int IComparable.CompareTo(object obj)
    {
        if (obj is null) return 1;
        if (obj is not BigRational other)
            throw new ArgumentException("Object must be a BigRational.", nameof(obj));
        return CompareTo(other);
    }

    public readonly bool Equals(BigRational other)
    {
        if (denominator == other.denominator) return numerator == other.numerator;
        var leftDenominator = EffectiveDenominator;
        var rightDenominator = other.EffectiveDenominator;
        return leftDenominator == rightDenominator
            ? numerator == other.numerator
            : numerator * rightDenominator == other.numerator * leftDenominator;
    }
    public override readonly bool Equals(object obj) => obj is BigRational other && Equals(other);

    public override readonly int GetHashCode()
    {
        var canonical = this;
        canonical.Simplify();
        return canonical.GetNormalizedHashCode();
    }

    internal readonly int GetNormalizedHashCode()
    {
        // Internal callers have already simplified, including default zero.
        System.Diagnostics.Debug.Assert(!denominator.IsZero);
        if (denominator == new FastBigInteger(1) && numerator.TryGetInt64(out long integer))
            return integer.GetHashCode();
        return HashCode.Combine(numerator.GetHashCode(), denominator.GetHashCode());
    }

    public override readonly string ToString() => string.Concat(
        numerator.ToString(), "/", EffectiveDenominator.ToString());

    public static BigRational Abs(BigRational value) => value.Sign < 0 ? -value : value;
    public static BigRational Negate(BigRational value) => -value;
    public static BigRational Max(BigRational x, BigRational y) => x >= y ? x : y;
    public static BigRational Min(BigRational x, BigRational y) => x <= y ? x : y;
    public static BigRational Invert(BigRational value) => new(value.EffectiveDenominator, value.numerator);
    public static BigRational Add(BigRational x, BigRational y) => x + y;
    public static BigRational Subtract(BigRational x, BigRational y) => x - y;
    public static BigRational Multiply(BigRational x, BigRational y) => x * y;
    public static BigRational Divide(BigRational x, BigRational y) => x / y;
    public static BigRational Remainder(BigRational x, BigRational y) => x % y;

    public static BigRational DivRem(BigRational dividend, BigRational divisor, out BigRational remainder)
    {
        var ad = dividend.numerator * divisor.EffectiveDenominator;
        var bc = dividend.EffectiveDenominator * divisor.numerator;
        var bd = dividend.EffectiveDenominator * divisor.EffectiveDenominator;
        remainder = new BigRational(ad % bc, bd);
        return new BigRational(ad, bc);
    }

    public static BigRational Pow(BigRational value, BigInteger exponent)
    {
        if (exponent.IsZero) return One;
        if (exponent.Sign < 0)
        {
            value = Invert(value);
            exponent = BigInteger.Negate(exponent);
        }

        BigRational result = One;
        while (exponent > BigInteger.Zero)
        {
            if (!exponent.IsEven) result *= value;
            exponent >>= 1;
            if (!exponent.IsZero) value *= value;
        }
        return result;
    }

    public static BigInteger LeastCommonDenominator(BigRational x, BigRational y)
    {
        var a = x.EffectiveDenominator;
        var b = y.EffectiveDenominator;
        return (a * (b / FastBigInteger.GreatestCommonDivisor(a, b))).ToBigInteger();
    }

    public static int Compare(BigRational x, BigRational y)
    {
        return (x.numerator * y.EffectiveDenominator).CompareTo(
            y.numerator * x.EffectiveDenominator);
    }

    public static bool operator ==(BigRational x, BigRational y) => Compare(x, y) == 0;
    public static bool operator !=(BigRational x, BigRational y) => Compare(x, y) != 0;
    public static bool operator <(BigRational x, BigRational y) => Compare(x, y) < 0;
    public static bool operator <=(BigRational x, BigRational y) => Compare(x, y) <= 0;
    public static bool operator >(BigRational x, BigRational y) => Compare(x, y) > 0;
    public static bool operator >=(BigRational x, BigRational y) => Compare(x, y) >= 0;
    public static BigRational operator +(BigRational value) => value;
    public static BigRational operator -(BigRational value) => new(-value.numerator, value.EffectiveDenominator);
    public static BigRational operator ++(BigRational value) => value + One;
    public static BigRational operator --(BigRational value) => value - One;

    public static BigRational operator +(in BigRational x, in BigRational y)
    {
        var xd = x.EffectiveDenominator;
        var yd = y.EffectiveDenominator;
        if (xd == yd) return new BigRational(x.numerator + y.numerator, xd);
        if (TryAddSmall(in x, in y, false, out var small)) return small;
        var gcd = FastBigInteger.GreatestCommonDivisor(xd, yd);
        var xScale = yd / gcd;
        var yScale = xd / gcd;
        return new BigRational(x.numerator * xScale + y.numerator * yScale, xd * xScale);
    }

    public static BigRational operator -(in BigRational x, in BigRational y)
    {
        var xd = x.EffectiveDenominator;
        var yd = y.EffectiveDenominator;
        if (xd == yd) return new BigRational(x.numerator - y.numerator, xd);
        if (TryAddSmall(in x, in y, true, out var small)) return small;
        var gcd = FastBigInteger.GreatestCommonDivisor(xd, yd);
        var xScale = yd / gcd;
        var yScale = xd / gcd;
        return new BigRational(x.numerator * xScale - y.numerator * yScale, xd * xScale);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryAddSmall(in BigRational x, in BigRational y, bool subtract, out BigRational result)
    {
        result = default;
        if (!x.numerator.TryGetInt64(out long xn) || !x.EffectiveDenominator.TryGetInt64(out long xd) ||
            !y.numerator.TryGetInt64(out long yn) || !y.EffectiveDenominator.TryGetInt64(out long yd))
            return false;

        long a = xd, b = yd;
        while (b != 0) { long remainder = a % b; a = b; b = remainder; }
        long xScale = yd / a, yScale = xd / a;
        Int128 left = (Int128)xn * xScale, right = (Int128)yn * yScale;
        Int128 sum = subtract ? unchecked(left - right) : unchecked(left + right);
        bool overflow = subtract ? ((left ^ right) & (left ^ sum)) < 0
            : ((left ^ sum) & (right ^ sum)) < 0;
        if (overflow) return false;
        result = new BigRational(FastBigInteger.FromInt128(sum),
            FastBigInteger.FromInt128((Int128)xd * xScale));
        return true;
    }

    public static BigRational operator *(in BigRational x, in BigRational y)
    {
        var xd = x.EffectiveDenominator;
        var yd = y.EffectiveDenominator;
        return new BigRational(x.numerator * y.numerator, xd * yd);
    }

    public static BigRational operator /(in BigRational x, in BigRational y)
    {
        if (y.numerator.IsZero) throw new DivideByZeroException();
        var xd = x.EffectiveDenominator;
        var yd = y.EffectiveDenominator;
        return new BigRational(x.numerator * yd, xd * y.numerator);
    }

    public static BigRational operator %(BigRational x, BigRational y)
    {
        var ad = x.numerator * y.EffectiveDenominator;
        var bc = x.EffectiveDenominator * y.numerator;
        var bd = x.EffectiveDenominator * y.EffectiveDenominator;
        return new BigRational(ad % bc, bd);
    }

    public static explicit operator BigInteger(BigRational value) => value.GetWholePart();
    public static explicit operator sbyte(BigRational value) => (sbyte)value.GetWholePart();
    public static explicit operator byte(BigRational value) => (byte)value.GetWholePart();
    public static explicit operator short(BigRational value) => (short)value.GetWholePart();
    public static explicit operator ushort(BigRational value) => (ushort)value.GetWholePart();
    public static explicit operator int(BigRational value) => (int)value.GetWholePart();
    public static explicit operator uint(BigRational value) => (uint)value.GetWholePart();
    public static explicit operator long(BigRational value) => (long)value.GetWholePart();
    public static explicit operator ulong(BigRational value) => (ulong)value.GetWholePart();
    public static explicit operator float(BigRational value) => (float)(double)value;
    public static explicit operator double(BigRational value) => value.ToDouble();

    public static explicit operator decimal(BigRational value)
    {
        BigInteger scaled = value.Numerator * BigInteger.Pow(10, 28) / value.Denominator;
        if (scaled.IsZero) return 0m;
        int scale = 28;
        while (BigInteger.Abs(scaled) > (BigInteger)decimal.MaxValue)
        {
            if (scale == 0) throw new OverflowException();
            scaled /= 10;
            scale--;
        }
        decimal result = (decimal)scaled;
        for (int i = 0; i < scale; i++) result /= 10m;
        return result;
    }

    public static implicit operator BigRational(sbyte value) => new(value);
    public static implicit operator BigRational(byte value) => new(value);
    public static implicit operator BigRational(short value) => new(value);
    public static implicit operator BigRational(ushort value) => new(value);
    public static implicit operator BigRational(int value) => new(value);
    public static implicit operator BigRational(uint value) => new((BigInteger)value);
    public static implicit operator BigRational(long value) => new(value);
    public static implicit operator BigRational(ulong value) => new((BigInteger)value);
    public static implicit operator BigRational(BigInteger value) => new(value);
    public static implicit operator BigRational(float value) => new((double)value);
    public static implicit operator BigRational(double value) => new(value);
    public static implicit operator BigRational(decimal value) => new(value);

    private readonly double ToDouble()
    {
        if (numerator.IsZero) return 0d;
        // Preserve the existing scale-based conversion: changing its rounding
        // shifts mesh coordinates enough to perturb downstream topology.
        BigInteger n = Numerator;
        BigInteger d = Denominator;
        static bool FitsDouble(BigInteger value) => value >= DoubleMinimum && value <= DoubleMaximum;
        if (FitsDouble(n) && FitsDouble(d)) return (double)n / (double)d;

        const int scale = 308;
        BigInteger scaled = (n * DoubleScale) / d;
        if (scaled.IsZero) return Sign < 0 ? -0d : 0d;
        double result = 0;
        bool converted = false;
        for (int i = 0; i < scale; i++)
        {
            if (!converted)
            {
                if (FitsDouble(scaled))
                {
                    result = (double)scaled;
                    converted = true;
                }
                else
                    scaled /= 10;
            }
            result /= 10;
        }
        if (!converted) return Sign < 0 ? double.NegativeInfinity : double.PositiveInfinity;
        return result;
    }
}
