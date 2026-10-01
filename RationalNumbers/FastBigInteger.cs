using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics.X86;

namespace GeoCore;

/// <summary>
/// Signed arbitrary precision integer with an allocation-free Int64 representation.
/// Large values use immutable, little-endian UInt64 limbs. The struct can be copied
/// freely: arithmetic never changes an array held by an existing value.
/// </summary>
public readonly struct FastBigInteger : IEquatable<FastBigInteger>, IComparable<FastBigInteger>
{
    private readonly long small;
    private readonly ulong wideLow;
    private readonly ulong wideHigh;
    private readonly ulong[] limbs;
    private bool wide => limbs == null && (wideLow != 0 || wideHigh != 0);
    private bool negative => small < 0;

    public FastBigInteger(long value) => (small, wideLow, wideHigh, limbs) = (value, 0, 0, null);

    private FastBigInteger(ulong low, ulong high, bool isNegative)
    {
        small = isNegative ? -1 : 1;
        wideLow = low;
        wideHigh = high;
        limbs = null;
    }

    private static FastBigInteger FromMagnitude(UInt128 magnitude, bool isNegative)
    {
        ulong low = (ulong)magnitude, high = (ulong)(magnitude >> 64);
        if (high == 0 && low <= (isNegative ? 0x8000000000000000UL : long.MaxValue))
            return new FastBigInteger(isNegative ? unchecked(-(long)low) : (long)low);
        return new FastBigInteger(low, high, isNegative);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static FastBigInteger FromInt128(Int128 value)
    {
        if (value >= long.MinValue && value <= long.MaxValue) return new FastBigInteger((long)value);
        return value < 0
            ? FromMagnitude((UInt128)(-(value + 1)) + 1, true)
            : FromMagnitude((UInt128)value, false);
    }

    private FastBigInteger(ulong[] magnitude, bool isNegative)
    {
        int length = magnitude.Length;
        while (length > 0 && magnitude[length - 1] == 0) length--;
        if (length == 0) { small = 0; wideLow = 0; wideHigh = 0; limbs = null; return; }
        if (length == 1 && magnitude[0] <= (isNegative ? 0x8000000000000000UL : long.MaxValue))
        {
            small = isNegative ? unchecked(-(long)magnitude[0]) : (long)magnitude[0];
            wideLow = 0; wideHigh = 0;
            limbs = null;
            return;
        }
        if (length <= 2)
        {
            small = isNegative ? -1 : 1;
            wideLow = magnitude[0];
            wideHigh = length == 2 ? magnitude[1] : 0;
            limbs = null;
            return;
        }
        small = isNegative ? -1 : 1;
        wideLow = 0; wideHigh = 0;
        limbs = length == magnitude.Length ? magnitude : magnitude.AsSpan(0, length).ToArray();
    }

    public FastBigInteger(BigInteger value)
    {
        if (value >= long.MinValue && value <= long.MaxValue)
        {
            small = (long)value; wideLow = 0; wideHigh = 0; limbs = null;
            return;
        }
        BigInteger abs = BigInteger.Abs(value);
        small = value.Sign < 0 ? -1 : 1;
        if (abs.GetByteCount(isUnsigned: true) <= 16)
        {
            Span<byte> stackBytes = stackalloc byte[16];
            abs.TryWriteBytes(stackBytes, out _, isUnsigned: true);
            wideLow = BitConverter.ToUInt64(stackBytes);
            wideHigh = BitConverter.ToUInt64(stackBytes[8..]);
            limbs = null;
            return;
        }
        wideLow = 0; wideHigh = 0;
        limbs = new ulong[(abs.GetByteCount(isUnsigned: true) + 7) / 8];
        abs.TryWriteBytes(MemoryMarshal.AsBytes(limbs.AsSpan()), out _, isUnsigned: true);
    }

    public int Sign => limbs == null && !wide ? Math.Sign(small) : negative ? -1 : 1;
    public bool IsZero => limbs == null && !wide && small == 0;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetInt64(out long value) { value = small; return limbs == null && !wide; }

    public BigInteger ToBigInteger()
    {
        if (limbs == null && !wide) return new BigInteger(small);
        if (wide)
        {
            Span<ulong> words = stackalloc ulong[2] { wideLow, wideHigh };
            BigInteger wideValue = new BigInteger(MemoryMarshal.AsBytes(words), isUnsigned: true);
            return negative ? -wideValue : wideValue;
        }
        BigInteger value = new BigInteger(MemoryMarshal.AsBytes(limbs.AsSpan()), isUnsigned: true);
        return negative ? -value : value;
    }

    private int Length => limbs?.Length ?? (wide ? wideHigh == 0 ? 1 : 2 : small == 0 ? 0 : 1);
    private ulong Limb(int index) => limbs != null ? index < limbs.Length ? limbs[index] : 0 : wide ? index == 0 ? wideLow : index == 1 ? wideHigh : 0 : index == 0 ? Magnitude(small) : 0;
    private UInt128 Magnitude128 => ((UInt128)Limb(1) << 64) | Limb(0);
    private static ulong Magnitude(long x) => x < 0 ? unchecked((ulong)(-(x + 1)) + 1) : (ulong)x;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static unsafe ulong MultiplyLimb(ulong a, ulong b, out ulong low)
    {
        if (Bmi2.X64.IsSupported)
        {
            ulong resultLow;
            ulong high = Bmi2.X64.MultiplyNoFlags(a, b, &resultLow);
            low = resultLow;
            return high;
        }
        return Math.BigMul(a, b, out low);
    }

    private static int CompareMagnitude(in FastBigInteger a, in FastBigInteger b)
    {
        if (a.limbs == null && b.limbs == null)
        {
            // The hex-bolt trace spends significant time comparing one- and
            // two-word values. Avoid repeated length and limb dispatch here.
            ulong aHigh = a.wide ? a.wideHigh : 0;
            ulong bHigh = b.wide ? b.wideHigh : 0;
            if (aHigh != bHigh) return aHigh > bHigh ? 1 : -1;
            ulong aLow = a.wide ? a.wideLow : Magnitude(a.small);
            ulong bLow = b.wide ? b.wideLow : Magnitude(b.small);
            return aLow == bLow ? 0 : aLow > bLow ? 1 : -1;
        }
        int aLength = a.Length, bLength = b.Length;
        if (aLength != bLength) return aLength > bLength ? 1 : -1;
        if (ReferenceEquals(a.limbs, b.limbs)) return 0;
        for (int i = aLength - 1; i >= 0; i--)
        {
            ulong left = a.Limb(i), right = b.Limb(i);
            if (left != right) return left > right ? 1 : -1;
        }
        return 0;
    }

    public int CompareTo(FastBigInteger other)
    {
        int aSign = Sign, bSign = other.Sign;
        if (aSign != bSign) return aSign.CompareTo(bSign);
        return aSign < 0 ? -CompareMagnitude(this, other) : CompareMagnitude(this, other);
    }

    public bool Equals(FastBigInteger other) => CompareTo(other) == 0;
    public override bool Equals(object obj) => obj is FastBigInteger other && Equals(other);
    public override int GetHashCode()
    {
        if (limbs == null && !wide) return small.GetHashCode();
        var hash = new HashCode();
        hash.Add(negative);
        if (wide) { hash.Add(wideLow); if (wideHigh != 0) hash.Add(wideHigh); return hash.ToHashCode(); }
        foreach (ulong limb in limbs) hash.Add(limb);
        return hash.ToHashCode();
    }

    public static FastBigInteger operator -(FastBigInteger value)
    {
        if (value.limbs != null) return new FastBigInteger(value.limbs, !value.negative);
        if (value.wide) return new FastBigInteger(value.wideLow, value.wideHigh, !value.negative);
        return value.small == long.MinValue
            ? new FastBigInteger(0x8000000000000000UL, 0, false)
            : new FastBigInteger(-value.small);
    }

    private static FastBigInteger AddMagnitude(in FastBigInteger a, in FastBigInteger b, bool sign)
    {
        if (a.Length <= 2 && b.Length <= 2)
        {
            UInt128 x = ((UInt128)a.Limb(1) << 64) | a.Limb(0);
            UInt128 y = ((UInt128)b.Limb(1) << 64) | b.Limb(0);
            UInt128 sum = unchecked(x + y);
            if (sum >= x) return FromMagnitude(sum, sign);
        }
        int length = Math.Max(a.Length, b.Length);
        var result = new ulong[length];
        ulong carry = 0;
        for (int i = 0; i < length; i++)
        {
            ulong x = a.Limb(i), y = b.Limb(i);
            ulong sum = unchecked(x + y);
            ulong next = unchecked(sum + carry);
            carry = (sum < x || next < sum) ? 1UL : 0;
            result[i] = next;
        }
        if (carry != 0)
        {
            Array.Resize(ref result, length + 1);
            result[length] = carry;
        }
        return new FastBigInteger(result, sign);
    }

    private static FastBigInteger SubtractMagnitude(in FastBigInteger a, in FastBigInteger b, bool sign)
    {
        if (a.Length <= 2) return FromMagnitude(((((UInt128)a.Limb(1) << 64) | a.Limb(0)) - (((UInt128)b.Limb(1) << 64) | b.Limb(0))), sign);
        var result = new ulong[a.Length];
        ulong borrow = 0;
        for (int i = 0; i < result.Length; i++)
        {
            ulong x = a.Limb(i), y = b.Limb(i);
            ulong difference = unchecked(x - y);
            result[i] = unchecked(difference - borrow);
            borrow = (x < y || difference < borrow) ? 1UL : 0;
        }
        return new FastBigInteger(result, sign);
    }

    public static FastBigInteger operator +(FastBigInteger a, FastBigInteger b)
    {
        if (a.limbs == null && !a.wide && b.limbs == null && !b.wide)
        {
            long sum = unchecked(a.small + b.small);
            if (((a.small ^ sum) & (b.small ^ sum)) >= 0) return new FastBigInteger(sum);
        }
        bool aNegative = a.Sign < 0, bNegative = b.Sign < 0;
        if (aNegative == bNegative) return AddMagnitude(a, b, aNegative);
        int comparison = CompareMagnitude(a, b);
        return comparison >= 0 ? SubtractMagnitude(a, b, aNegative) : SubtractMagnitude(b, a, bNegative);
    }

    public static FastBigInteger operator -(FastBigInteger a, FastBigInteger b) => a + -b;

    public static FastBigInteger operator /(FastBigInteger a, FastBigInteger b)
    {
        if (b.IsZero) throw new DivideByZeroException();
        if (a.limbs == null && !a.wide && b.limbs == null && !b.wide && !(a.small == long.MinValue && b.small == -1))
            return new FastBigInteger(a.small / b.small);
        if (a.Length <= 2 && b.Length <= 2)
            return FromMagnitude(a.Magnitude128 / b.Magnitude128, (a.Sign < 0) != (b.Sign < 0));
        return new FastBigInteger(a.ToBigInteger() / b.ToBigInteger());
    }

    public static FastBigInteger operator *(FastBigInteger a, FastBigInteger b)
    {
        if (a.IsZero || b.IsZero) return default;
        if (a.limbs == null && !a.wide && b.limbs == null && !b.wide)
        {
            long high = Math.BigMul(a.small, b.small, out long low);
            if (high == low >> 63) return new FastBigInteger(low);
            Int128 signedProduct = ((Int128)high << 64) | (ulong)low;
            return FromMagnitude((UInt128)(signedProduct < 0 ? -signedProduct : signedProduct), signedProduct < 0);
        }
        int na = a.Length, nb = b.Length;
        // At this size the runtime's subquadratic multiplication overtakes the
        // limb loop, even after the conversion cost. Keep unbalanced products
        // on the linear-in-the-large-operand schoolbook path.
        if (na >= 24 && nb >= 24)
            return new FastBigInteger(a.ToBigInteger() * b.ToBigInteger());
        int topBits = 128 - BitOperations.LeadingZeroCount(a.Limb(na - 1)) - BitOperations.LeadingZeroCount(b.Limb(nb - 1));
        var result = new ulong[na + nb - (topBits <= 64 ? 1 : 0)];
        for (int i = 0; i < na; i++)
        {
            ulong carry = 0, x = a.Limb(i);
            for (int j = 0; j < nb; j++)
            {
                ulong high = MultiplyLimb(x, b.Limb(j), out ulong low);
                ulong sum = unchecked(result[i + j] + low);
                ulong overflow = sum < low ? 1UL : 0;
                ulong next = unchecked(sum + carry);
                if (next < sum) overflow++;
                result[i + j] = next;
                carry = unchecked(high + overflow);
            }
            if (i + nb < result.Length) result[i + nb] = carry;
            else if (carry != 0) throw new InvalidOperationException("Multiplication carry exceeds the allocated product.");
        }
        return new FastBigInteger(result, (a.Sign < 0) != (b.Sign < 0));
    }

    public static FastBigInteger GreatestCommonDivisor(FastBigInteger a, FastBigInteger b)
    {
        if (a.limbs == null && !a.wide && b.limbs == null && !b.wide)
        {
            ulong x = Magnitude(a.small), y = Magnitude(b.small);
            while (y != 0) { ulong remainder = x % y; x = y; y = remainder; }
            return FromMagnitude(x, false);
        }
        if (a.Length <= 2 && b.Length <= 2)
        {
            UInt128 x = a.Magnitude128, y = b.Magnitude128;
            if (x == 0) return FromMagnitude(y, false);
            if (y == 0) return FromMagnitude(x, false);
            UInt128 distance = x >= y ? x - y : y - x;
            if (distance == 0) return FromMagnitude(x, false);
            if (distance <= ulong.MaxValue)
            {
                ulong right = (ulong)distance;
                ulong left = (ulong)(x % right);
                while (right != 0) { ulong remainder = left % right; left = right; right = remainder; }
                return FromMagnitude(left, false);
            }
            while (y != 0) { UInt128 remainder = x % y; x = y; y = remainder; }
            return FromMagnitude(x, false);
        }
        // For many limbs the runtime's Lehmer reduction wins by a wide margin.
        // BinaryGreatestCommonDivisor remains available when a self-contained
        // calculation is required, but the rational hot path uses the faster one.
        return new FastBigInteger(BigInteger.GreatestCommonDivisor(a.ToBigInteger(), b.ToBigInteger()));
    }

    public static FastBigInteger BinaryGreatestCommonDivisor(FastBigInteger a, FastBigInteger b)
    {
        if (a.IsZero) return b.Sign < 0 ? -b : b;
        if (b.IsZero) return a.Sign < 0 ? -a : a;
        ulong[] x = a.CopyMagnitude(), y = b.CopyMagnitude();
        int nx = x.Length, ny = y.Length;
        int xZeros = TrailingZeros(x, nx), yZeros = TrailingZeros(y, ny);
        int commonZeros = Math.Min(xZeros, yZeros);
        ShiftRight(x, ref nx, xZeros);
        ShiftRight(y, ref ny, yZeros);
        while (ny != 0)
        {
            if (CompareMagnitude(x, nx, y, ny) > 0)
            {
                (x, y) = (y, x);
                (nx, ny) = (ny, nx);
            }
            SubtractInPlace(y, ref ny, x, nx);
            if (ny != 0) ShiftRight(y, ref ny, TrailingZeros(y, ny));
        }
        int wordShift = commonZeros / 64, bitShift = commonZeros % 64;
        int resultLength = nx + wordShift + (bitShift != 0 && (x[nx - 1] >> (64 - bitShift)) != 0 ? 1 : 0);
        var result = new ulong[resultLength];
        for (int i = 0; i < nx; i++)
        {
            result[i + wordShift] |= x[i] << bitShift;
            if (bitShift != 0 && i + wordShift + 1 < resultLength)
                result[i + wordShift + 1] = x[i] >> (64 - bitShift);
        }
        return new FastBigInteger(result, false);
    }

    private ulong[] CopyMagnitude()
    {
        if (limbs != null) return (ulong[])limbs.Clone();
        return Length == 2 ? [Limb(0), Limb(1)] : [Limb(0)];
    }

    private static int TrailingZeros(ulong[] value, int length)
    {
        int i = 0;
        while (i < length && value[i] == 0) i++;
        return i * 64 + BitOperations.TrailingZeroCount(value[i]);
    }

    private static void ShiftRight(ulong[] value, ref int length, int bits)
    {
        int words = bits / 64, shift = bits % 64;
        int newLength = length - words;
        for (int i = 0; i < newLength; i++)
        {
            ulong low = value[i + words] >> shift;
            ulong high = shift != 0 && i + words + 1 < length ? value[i + words + 1] << (64 - shift) : 0;
            value[i] = low | high;
        }
        while (newLength > 0 && value[newLength - 1] == 0) newLength--;
        length = newLength;
    }

    private static int CompareMagnitude(ulong[] a, int na, ulong[] b, int nb)
    {
        if (na != nb) return na > nb ? 1 : -1;
        for (int i = na - 1; i >= 0; i--)
            if (a[i] != b[i]) return a[i] > b[i] ? 1 : -1;
        return 0;
    }

    private static void SubtractInPlace(ulong[] a, ref int na, ulong[] b, int nb)
    {
        ulong borrow = 0;
        for (int i = 0; i < na; i++)
        {
            ulong x = a[i], y = i < nb ? b[i] : 0;
            ulong difference = unchecked(x - y);
            a[i] = unchecked(difference - borrow);
            borrow = x < y || difference < borrow ? 1UL : 0;
        }
        while (na > 0 && a[na - 1] == 0) na--;
    }

    public static bool operator ==(FastBigInteger a, FastBigInteger b) => a.Equals(b);
    public static bool operator !=(FastBigInteger a, FastBigInteger b) => !a.Equals(b);
    public static bool operator <(FastBigInteger a, FastBigInteger b) => a.CompareTo(b) < 0;
    public static bool operator >(FastBigInteger a, FastBigInteger b) => a.CompareTo(b) > 0;
    public static bool operator <=(FastBigInteger a, FastBigInteger b) => a.CompareTo(b) <= 0;
    public static bool operator >=(FastBigInteger a, FastBigInteger b) => a.CompareTo(b) >= 0;
    public static implicit operator FastBigInteger(long value) => new(value);
    public static explicit operator FastBigInteger(BigInteger value) => new(value);
    public static explicit operator BigInteger(FastBigInteger value) => value.ToBigInteger();
    public override string ToString() => ToBigInteger().ToString();
}

