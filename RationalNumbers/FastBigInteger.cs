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
        : this(magnitude.AsSpan(), isNegative, magnitude)
    {
    }

    private FastBigInteger(ReadOnlySpan<ulong> magnitude, bool isNegative, ulong[] ownedMagnitude = null)
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
        limbs = ownedMagnitude != null && length == ownedMagnitude.Length
            ? ownedMagnitude
            : magnitude.Slice(0, length).ToArray();
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
    internal bool IsUnit => limbs == null && !wide && (small == 1 || small == -1);
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
        if (a.IsZero) return b;
        if (b.IsZero) return a;
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
        if (b.IsUnit) return b.small == 1 ? a : -a;
        if (a.Length <= 2 && b.Length <= 2)
            return FromMagnitude(a.Magnitude128 / b.Magnitude128, (a.Sign < 0) != (b.Sign < 0));
        DivRemMagnitudes(a, b, out FastBigInteger quotient, out _, (a.Sign < 0) != (b.Sign < 0));
        return quotient;
    }

    public static FastBigInteger operator %(FastBigInteger a, FastBigInteger b)
    {
        if (b.IsZero) throw new DivideByZeroException();
        if (b.IsUnit) return default;
        if (a.limbs == null && !a.wide && b.limbs == null && !b.wide)
        {
            if (a.small == long.MinValue && b.small == -1) return default;
            return new FastBigInteger(a.small % b.small);
        }
        if (a.Length <= 2 && b.Length <= 2)
            return FromMagnitude(a.Magnitude128 % b.Magnitude128, a.Sign < 0);
        DivRemMagnitudes(a, b, out _, out FastBigInteger remainder, false);
        return remainder;
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
        // Immutable magnitudes can be reused for identity arithmetic. This also
        // avoids BigInteger conversions when a rational denominator is one.
        if (a.IsUnit) return a.small == 1 ? b : -b;
        if (b.IsUnit) return b.small == 1 ? a : -a;
        int na = a.Length, nb = b.Length;
        bool negative = (a.Sign < 0) != (b.Sign < 0);
        if (nb == 1) return MultiplyByLimb(a, b.Limb(0), negative);
        if (na == 1) return MultiplyByLimb(b, a.Limb(0), negative);
        if (na <= 2 && nb <= 2) return Multiply128(a.Limb(0), a.Limb(1), b.Limb(0), b.Limb(1), negative);
        // Measured against System.Numerics.BigInteger on 64-bit limbs: once both
        // operands reach 12 limbs, the runtime multiply (including the conversion)
        // beats this schoolbook loop. Unbalanced products stay here.
        if (na >= 12 && nb >= 12)
            return new FastBigInteger(a.ToBigInteger() * b.ToBigInteger());
        return MultiplySchoolbook(a, na, b, nb, negative);
    }

    public static FastBigInteger GreatestCommonDivisor(FastBigInteger a, FastBigInteger b)
    {
        if (a.IsZero) return b.Sign < 0 ? -b : b;
        if (b.IsZero) return a.Sign < 0 ? -a : a;
        if (a.limbs == null && !a.wide && b.limbs == null && !b.wide)
        {
            ulong x = Magnitude(a.small), y = Magnitude(b.small);
            while (y != 0) { ulong remainder = x % y; x = y; y = remainder; }
            return FromMagnitude(x, false);
        }
        if (a.IsUnit || b.IsUnit) return new FastBigInteger(1);
        if (a.Length <= 2 && b.Length <= 2)
            return GcdUpTo128(a.Limb(0), a.Length > 1 ? a.Limb(1) : 0, b.Limb(0), b.Length > 1 ? b.Limb(1) : 0);
        // One remainder brings an unbalanced pair back to the 128-bit path.
        // Past that, the runtime Lehmer GCD is faster than a limb binary GCD.
        if (a.Length <= 2 || b.Length <= 2)
        {
            FastBigInteger small = a.Length <= b.Length ? a : b;
            FastBigInteger large = a.Length <= b.Length ? b : a;
            return GreatestCommonDivisor(small, large % small);
        }
        return new FastBigInteger(BigInteger.GreatestCommonDivisor(a.ToBigInteger(), b.ToBigInteger()));
    }

    private static FastBigInteger MultiplyByLimb(in FastBigInteger value, ulong limb, bool negative)
    {
        int n = value.Length;
        Span<ulong> result = n + 1 <= 32 ? stackalloc ulong[n + 1] : new ulong[n + 1];
        ulong carry = 0;
        for (int i = 0; i < n; i++)
        {
            ulong high = MultiplyLimb(value.limbs != null ? value.limbs[i] : value.Limb(i), limb, out ulong low);
            ulong sum = low + carry;
            result[i] = sum;
            carry = high + (sum < low ? 1UL : 0);
        }
        result[n] = carry;
        return new FastBigInteger(result, negative);
    }

    private static FastBigInteger Multiply128(ulong x0, ulong x1, ulong y0, ulong y1, bool negative)
    {
        ulong z1 = MultiplyLimb(x0, y0, out ulong z0);
        ulong c1 = MultiplyLimb(x0, y1, out ulong c0);
        ulong d1 = MultiplyLimb(x1, y0, out ulong d0);
        ulong e1 = MultiplyLimb(x1, y1, out ulong e0);
        ulong limb1 = z1 + c0;
        ulong carry = limb1 < z1 ? 1UL : 0;
        limb1 += d0;
        if (limb1 < d0) carry++;
        UInt128 limb2 = (UInt128)c1 + d1 + e0 + carry;
        ulong limb3 = e1 + (ulong)(limb2 >> 64);
        ulong low2 = (ulong)limb2;
        if (limb3 != 0) return new FastBigInteger([z0, limb1, low2, limb3], negative);
        if (low2 != 0)
        {
            Span<ulong> three = stackalloc ulong[3] { z0, limb1, low2 };
            return new FastBigInteger(three, negative);
        }
        return FromMagnitude(((UInt128)limb1 << 64) | z0, negative);
    }

    private static FastBigInteger MultiplySchoolbook(in FastBigInteger a, int na, in FastBigInteger b, int nb, bool negative)
    {
        Span<ulong> xs = na <= 32 ? stackalloc ulong[na] : new ulong[na];
        Span<ulong> ys = nb <= 32 ? stackalloc ulong[nb] : new ulong[nb];
        CopyLimbs(a, xs);
        CopyLimbs(b, ys);
        int topBits = 128 - BitOperations.LeadingZeroCount(xs[na - 1]) - BitOperations.LeadingZeroCount(ys[nb - 1]);
        int resultLength = na + nb - (topBits <= 64 ? 1 : 0);
        ulong[] owned = null;
        Span<ulong> result = resultLength <= 32 ? stackalloc ulong[resultLength] : (owned = new ulong[resultLength]);
        result.Clear();
        for (int i = 0; i < na; i++)
        {
            ulong carry = 0, x = xs[i];
            for (int j = 0; j < nb; j++)
            {
                ulong high = MultiplyLimb(x, ys[j], out ulong low);
                ulong sum = result[i + j] + low;
                ulong overflow = sum < low ? 1UL : 0;
                ulong next = sum + carry;
                if (next < sum) overflow++;
                result[i + j] = next;
                carry = high + overflow;
            }
            if (i + nb < resultLength) result[i + nb] = carry;
            else if (carry != 0) throw new InvalidOperationException("Multiplication carry exceeds the allocated product.");
        }
        return new FastBigInteger(result, negative, owned);
    }

    private static FastBigInteger GcdUpTo128(ulong x0, ulong x1, ulong y0, ulong y1)
    {
        if ((x0 | x1) == 0) return FromMagnitude(((UInt128)y1 << 64) | y0, false);
        if ((y0 | y1) == 0) return FromMagnitude(((UInt128)x1 << 64) | x0, false);
        UInt128 x = ((UInt128)x1 << 64) | x0;
        UInt128 y = ((UInt128)y1 << 64) | y0;
        UInt128 distance = x >= y ? x - y : y - x;
        if (distance == 0) return FromMagnitude(x, false);
        if (distance <= ulong.MaxValue)
        {
            ulong right = (ulong)distance;
            ulong left = (ulong)(x % right);
            while (right != 0) { ulong remainder = left % right; left = right; right = remainder; }
            return FromMagnitude(left, false);
        }
        int zx = TrailingZeros128(x0, x1), zy = TrailingZeros128(y0, y1);
        int shared = zx < zy ? zx : zy;
        ShiftRight128(ref x0, ref x1, zx);
        while ((y0 | y1) != 0)
        {
            ShiftRight128(ref y0, ref y1, TrailingZeros128(y0, y1));
            if (x1 > y1 || (x1 == y1 && x0 > y0))
            {
                (x0, y0) = (y0, x0);
                (x1, y1) = (y1, x1);
            }
            ulong borrow = y0 < x0 ? 1UL : 0;
            y0 -= x0;
            y1 = y1 - x1 - borrow;
        }
        ShiftLeft128(ref x0, ref x1, shared);
        return FromMagnitude(((UInt128)x1 << 64) | x0, false);
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

    internal static void RemoveSharedSmallFactors(ref FastBigInteger left, ref FastBigInteger right)
    {
        int sharedTwos = Math.Min(TrailingZeroCount(left), TrailingZeroCount(right));
        if (sharedTwos > 0)
        {
            left = ShiftRight(left, sharedTwos);
            right = ShiftRight(right, sharedTwos);
        }
        RemoveSharedPrime(ref left, ref right, 3);
        RemoveSharedPrime(ref left, ref right, 5);
    }

    private static void RemoveSharedPrime(ref FastBigInteger left, ref FastBigInteger right, ulong prime)
    {
        while (LimbSumMod(left, prime) == 0 && LimbSumMod(right, prime) == 0)
        {
            left = DivideExact(left, prime);
            right = DivideExact(right, prime);
        }
    }

    private static int TrailingZeroCount(in FastBigInteger value)
    {
        if (value.IsZero) return 0;
        int length = value.Length, words = 0;
        while (words < length && value.Limb(words) == 0) words++;
        return words * 64 + BitOperations.TrailingZeroCount(value.Limb(words));
    }

    private static ulong LimbSumMod(in FastBigInteger value, ulong modulus)
    {
        ulong sum = 0;
        int length = value.Length;
        for (int i = 0; i < length; i++)
        {
            sum += value.Limb(i) % modulus;
            if (sum >= modulus) sum -= modulus;
        }
        return sum;
    }

    private static FastBigInteger DivideExact(in FastBigInteger value, ulong divisor)
    {
        int length = value.Length;
        Span<ulong> limbs = length <= 128 ? stackalloc ulong[length] : new ulong[length];
        CopyLimbs(value, limbs);
        ulong remainder = 0;
        for (int i = length - 1; i >= 0; i--)
            limbs[i] = Div128By64(remainder, limbs[i], divisor, out remainder);
        if (remainder != 0)
            throw new InvalidOperationException("Small-factor division was not exact.");
        return new FastBigInteger(limbs, value.Sign < 0);
    }

    private static FastBigInteger ShiftRight(in FastBigInteger value, int bits)
    {
        if (bits <= 0 || value.IsZero) return value;
        int length = value.Length;
        int words = bits / 64;
        if (words >= length) return default;
        int shift = bits % 64;
        int newLength = length - words;
        Span<ulong> source = length <= 128 ? stackalloc ulong[length] : new ulong[length];
        CopyLimbs(value, source);
        Span<ulong> destination = newLength <= 128 ? stackalloc ulong[newLength] : new ulong[newLength];
        if (shift == 0)
            source.Slice(words, newLength).CopyTo(destination);
        else
            ShiftRight(source.Slice(words, newLength), destination, shift);
        return new FastBigInteger(destination, value.Sign < 0);
    }

    private static void CopyLimbs(in FastBigInteger value, Span<ulong> dest)
    {
        if (value.limbs != null)
        {
            value.limbs.AsSpan(0, dest.Length).CopyTo(dest);
            return;
        }
        dest[0] = value.wide ? value.wideLow : Magnitude(value.small);
        if (dest.Length > 1) dest[1] = value.wideHigh;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int TrailingZeros128(ulong low, ulong high)
        => low != 0 ? BitOperations.TrailingZeroCount(low) : 64 + BitOperations.TrailingZeroCount(high);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ShiftRight128(ref ulong low, ref ulong high, int bits)
    {
        if (bits == 0) return;
        if (bits >= 64)
        {
            low = high >> (bits - 64);
            high = 0;
            return;
        }
        low = (low >> bits) | (high << (64 - bits));
        high >>= bits;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ShiftLeft128(ref ulong low, ref ulong high, int bits)
    {
        if (bits == 0) return;
        if (bits >= 64)
        {
            high = low << (bits - 64);
            low = 0;
            return;
        }
        high = (high << bits) | (low >> (64 - bits));
        low <<= bits;
    }

    private static void DivRemMagnitudes(in FastBigInteger dividend, in FastBigInteger divisor, out FastBigInteger quotient, out FastBigInteger remainder, bool quotientNegative)
    {
        int na = dividend.Length, nb = divisor.Length;
        int comparison = na != nb ? (na > nb ? 1 : -1) : CompareMagnitude(dividend, divisor);
        if (comparison < 0)
        {
            quotient = default;
            remainder = dividend;
            return;
        }
        if (comparison == 0)
        {
            quotient = new FastBigInteger(quotientNegative ? -1 : 1);
            remainder = default;
            return;
        }
        Span<ulong> u = na <= 128 ? stackalloc ulong[na] : new ulong[na];
        Span<ulong> v = nb <= 128 ? stackalloc ulong[nb] : new ulong[nb];
        CopyLimbs(dividend, u);
        CopyLimbs(divisor, v);
        int quotientLength = na - nb + 1;
        Span<ulong> q = quotientLength <= 128 ? stackalloc ulong[quotientLength] : new ulong[quotientLength];
        DivideMagnitudes(u, v, q);
        quotient = new FastBigInteger(q, quotientNegative);
        remainder = new FastBigInteger(u, dividend.Sign < 0);
    }

    private static void DivideMagnitudes(Span<ulong> dividend, ReadOnlySpan<ulong> divisor, Span<ulong> quotient)
    {
        quotient.Clear();
        if (divisor.Length == 1)
        {
            DivideByLimb(dividend, divisor[0], quotient);
            return;
        }
        int n = divisor.Length;
        int shift = BitOperations.LeadingZeroCount(divisor[n - 1]);
        Span<ulong> vn = n <= 128 ? stackalloc ulong[n] : new ulong[n];
        Span<ulong> un = dividend.Length + 1 <= 129 ? stackalloc ulong[dividend.Length + 1] : new ulong[dividend.Length + 1];
        ShiftLeft(divisor, vn, shift, extraLimb: false);
        ShiftLeft(dividend, un, shift, extraLimb: true);
        ulong vn1 = vn[n - 1], vn2 = vn[n - 2];
        for (int j = dividend.Length - n; j >= 0; j--)
        {
            ulong qhat = EstimateQuotient(un[j + n], un[j + n - 1], un[j + n - 2], vn1, vn2);
            if (SubtractMul(un.Slice(j, n + 1), vn, qhat) != 0)
            {
                qhat--;
                AddBack(un.Slice(j, n + 1), vn);
            }
            quotient[j] = qhat;
        }
        ShiftRight(un.Slice(0, n), dividend.Slice(0, n), shift);
        if (n < dividend.Length) dividend.Slice(n).Clear();
    }

    private static void DivideByLimb(Span<ulong> dividend, ulong divisor, Span<ulong> quotient)
    {
        ulong remainder = 0;
        for (int i = dividend.Length - 1; i >= 0; i--)
        {
            ulong digit = Div128By64(remainder, dividend[i], divisor, out remainder);
            quotient[i] = digit;
            dividend[i] = 0;
        }
        dividend[0] = remainder;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong Div128By64(ulong high, ulong low, ulong divisor, out ulong remainder)
    {
        if (high == 0)
        {
            (ulong q, ulong r) = Math.DivRem(low, divisor);
            remainder = r;
            return q;
        }
#pragma warning disable SYSLIB5004
        if (X86Base.X64.IsSupported)
        {
            (ulong q, ulong r) = X86Base.X64.DivRem(low, high, divisor);
            remainder = r;
            return q;
        }
#pragma warning restore SYSLIB5004
        UInt128 value = ((UInt128)high << 64) | low;
        ulong quotient = (ulong)(value / divisor);
        remainder = (ulong)(value - (UInt128)quotient * divisor);
        return quotient;
    }

    private static ulong EstimateQuotient(ulong u2, ulong u1, ulong u0, ulong v1, ulong v0)
    {
        ulong qhat, rhat;
        if (u2 >= v1)
        {
            qhat = ulong.MaxValue;
            rhat = u2 + u1;
            if (rhat < u2) return qhat;
        }
        else
        {
            qhat = Div128By64(u2, u1, v1, out rhat);
        }
        for (int guard = 0; guard < 2; guard++)
        {
            if ((UInt128)qhat * v0 <= (((UInt128)rhat << 64) | u0)) break;
            qhat--;
            UInt128 widened = (UInt128)rhat + v1;
            if (widened > ulong.MaxValue) break;
            rhat = (ulong)widened;
        }
        return qhat;
    }

    private static ulong SubtractMul(Span<ulong> target, ReadOnlySpan<ulong> value, ulong multiplier)
    {
        ulong mulCarry = 0, borrow = 0;
        int n = value.Length;
        for (int i = 0; i < n; i++)
        {
            ulong high = MultiplyLimb(multiplier, value[i], out ulong low);
            ulong product = low + mulCarry;
            mulCarry = high + (product < low ? 1UL : 0);
            ulong difference = target[i] - product;
            ulong nextBorrow = target[i] < product || difference < borrow ? 1UL : 0;
            target[i] = difference - borrow;
            borrow = nextBorrow;
        }
        ulong top = target[n] - mulCarry;
        ulong topBorrow = target[n] < mulCarry || top < borrow ? 1UL : 0;
        target[n] = top - borrow;
        return topBorrow;
    }

    private static void AddBack(Span<ulong> target, ReadOnlySpan<ulong> value)
    {
        ulong carry = 0;
        for (int i = 0; i < value.Length; i++)
        {
            UInt128 sum = (UInt128)target[i] + value[i] + carry;
            target[i] = (ulong)sum;
            carry = (ulong)(sum >> 64);
        }
        target[value.Length] += carry;
    }

    private static void ShiftLeft(ReadOnlySpan<ulong> source, Span<ulong> destination, int shift, bool extraLimb)
    {
        if (shift == 0)
        {
            source.CopyTo(destination);
            if (extraLimb) destination[source.Length] = 0;
            return;
        }
        int back = 64 - shift;
        ulong carry = 0;
        for (int i = 0; i < source.Length; i++)
        {
            ulong limb = source[i];
            destination[i] = (limb << shift) | carry;
            carry = limb >> back;
        }
        if (extraLimb) destination[source.Length] = carry;
    }

    private static void ShiftRight(ReadOnlySpan<ulong> source, Span<ulong> destination, int shift)
    {
        if (shift == 0)
        {
            source.CopyTo(destination);
            return;
        }
        int back = 64 - shift;
        ulong carry = 0;
        for (int i = source.Length - 1; i >= 0; i--)
        {
            ulong limb = source[i];
            destination[i] = (limb >> shift) | carry;
            carry = limb << back;
        }
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

