using System.Globalization;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Text;

namespace GeoCore
{
    //    public struct BigRational
    //    {
    //        public static readonly BigRational Zero = new BigRational(0, 1);
    //        public static readonly BigRational One = new BigRational(1, 1);

    //        public BigInteger Num;
    //        public BigInteger Denom;

    //        public BigRational(int value)
    //        {
    //            Num = value;
    //            Denom = 1;
    //        }
    //        public BigRational(long value)
    //        {
    //            Num = value;
    //            Denom = 1;
    //        }
    //        public BigRational(BigInteger num, BigInteger denom)
    //        {
    //            Num = num;
    //            Denom = denom;
    //        }
    //        public BigRational(BigInteger num)
    //        {
    //            Num = num;
    //            Denom = 1;
    //        }

    //        public static BigRational Abs(BigRational r)
    //        {
    //            if (r.Num < 0)
    //                r.Num = -r.Num;
    //            if (r.Denom < 0)
    //                r.Denom = -r.Denom;
    //            return r;
    //        }

    //        public void Simplify()
    //        {
    //            if (Denom.Sign < 0)
    //            {
    //                Denom = -Denom;
    //                Num = -Num;
    //            }

    //            var gcd = BigInteger.GreatestCommonDivisor(BigInteger.Abs(Num), Denom);
    //            Num /= gcd;
    //            Denom /= gcd;
    //        }

    //        public void GetRange(out int lower, out int upper)
    //        {
    //            int sign = Sign;
    //            if (sign == 0)
    //            {
    //                lower = 0;
    //                upper = 0;
    //                return;
    //            }

    //            var div = BigInteger.Abs(Num) / BigInteger.Abs(Denom);
    //            lower = (int)div;
    //            upper = lower + 1;

    //            if (sign < 0)
    //            {
    //                int tmp = lower;
    //                lower = -upper;
    //                upper = -tmp;
    //            }
    //        }

    //        public int Sign
    //        {
    //            get { return Num.Sign * Denom.Sign; }
    //        }

    //        public static BigRational operator /(BigRational lhs, BigInteger rhs)
    //        {
    //            return new BigRational(lhs.Num, lhs.Denom * rhs);
    //        }
    //        public static BigRational operator /(BigRational lhs, BigRational rhs)
    //        {
    //            return new BigRational(lhs.Num * rhs.Denom, lhs.Denom * rhs.Num);
    //        }
    //        public static BigRational operator *(BigRational lhs, BigRational rhs)
    //        {
    //            return new BigRational(lhs.Num * rhs.Num, lhs.Denom * rhs.Denom);
    //        }
    //        public static BigRational operator -(BigRational lhs, BigRational rhs)
    //        {
    //            return new BigRational(lhs.Num * rhs.Denom - rhs.Num * lhs.Denom, lhs.Denom * rhs.Denom);
    //        }
    //        public static BigRational operator -(BigRational v)
    //        {
    //            return new BigRational(-v.Num, v.Denom);
    //        }
    //        public static BigRational operator +(BigRational lhs, BigRational rhs)
    //        {
    //            return new BigRational(lhs.Num * rhs.Denom + rhs.Num * lhs.Denom, lhs.Denom * rhs.Denom);
    //        }
    //        public int CompareTo(BigRational r)
    //        {
    //            BigRational diff = this - r;
    //            return diff.Sign;
    //        }
    //        public static bool operator >=(BigRational lhs, BigRational rhs)
    //        {
    //            return lhs.CompareTo(rhs) >= 0;
    //        }
    //        public static bool operator <=(BigRational lhs, BigRational rhs)
    //        {
    //            return lhs.CompareTo(rhs) <= 0;
    //        }

    //        public static BigRational Min(BigRational lhs, BigRational rhs)
    //        {
    //            int c = lhs.CompareTo(rhs);
    //            if (c < 0)
    //                return lhs;
    //            return rhs;
    //        }

    //        public static BigRational Max(BigRational lhs, BigRational rhs)
    //        {
    //            int c = lhs.CompareTo(rhs);
    //            if (c > 0)
    //                return lhs;
    //            return rhs;
    //        }

    //        public long ToLong()
    //        {
    //            //int sign = Sign;

    //            //BigInteger num = Num;
    //            //BigInteger denom = Denom;
    //            //if (num < 0)
    //            //    num = -num;
    //            //if (denom < 0)
    //            //    denom = -denom;

    //            //BigInteger remainder;
    //            //BigInteger div = BigInteger.DivRem(num, denom, out remainder);

    //            //long result = (long)div;

    //            //if(sign < 0)
    //            //{

    //            //}

    //            BigInteger div = Num / Denom;

    //#if DEBUG
    //            if (div > long.MaxValue || div < long.MinValue)
    //                throw new System.Exception();
    //#endif

    //            long result = (long)div;
    //            return result;
    //        }

    //        public int ToInt()
    //        {
    //            BigInteger div = Num / Denom;

    //#if DEBUG
    //            if (div > int.MaxValue || div < int.MinValue)
    //                throw new System.Exception();
    //#endif

    //            int result = (int)div;
    //            return result;
    //        }

    //        public override string ToString()
    //        {
    //            BigInteger remainder;
    //            BigInteger div = BigInteger.DivRem(Num, Denom, out remainder);
    //            double d = (double)div;
    //            d += (double)remainder / (double)Denom;
    //            return d.ToString() + "   [" + remainder.ToString() + "]";
    //        }

    //        public override int GetHashCode()
    //        {
    //            int hash = Num.GetHashCode();
    //            hash ^= Denom.GetHashCode() + (hash << 6) + (hash >> 2);
    //            return hash;
    //        }


    //        public override bool Equals(object obj)
    //        {
    //            BigRational r = (BigRational)obj;
    //            return this.Num == r.Num && this.Denom == r.Denom;
    //        }

    //        public double ToDouble()
    //        {
    //            BigInteger remainder;
    //            BigInteger div = BigInteger.DivRem(Num, Denom, out remainder);
    //            double d = (double)div;
    //            d += (double)remainder / (double)Denom;
    //            return d;
    //        }
    //    }


    //https://github.com/microsoftarchive/bcl/blob/master/Libraries/BigRational/BigRationalLibrary/BigRational.cs
    //   Copyright (c) Microsoft Corporation.  All rights reserved.
    /*============================================================
    ** Class: BigRational
    **
    ** Purpose: 
    ** --------
    ** This class is used to represent an arbitrary precision
    ** BigRational number
    **
    ** A rational number (commonly called a fraction) is a ratio
    ** between two integers.  For example (3/6) = (2/4) = (1/2)
    **
    ** Arithmetic
    ** ----------
    ** a/b = c/d, iff ad = bc
    ** a/b + c/d  == (ad + bc)/bd
    ** a/b - c/d  == (ad - bc)/bd
    ** a/b % c/d  == (ad % bc)/bd
    ** a/b * c/d  == (ac)/(bd)
    ** a/b / c/d  == (ad)/(bc)
    ** -(a/b)     == (-a)/b
    ** (a/b)^(-1) == b/a, if a != 0
    **
    ** Reduction Algorithm
    ** ------------------------
    ** Euclid's algorithm is used to simplify the fraction.
    ** Calculating the greatest common divisor of two n-digit
    ** numbers can be found in
    **
    ** O(n(log n)^5 (log log n)) steps as n -> +infinity
    ============================================================*/
    [Serializable]
    [ComVisible(false)]
    public struct BigRational : IComparable, IComparable<BigRational>, IDeserializationCallback, IEquatable<BigRational>, ISerializable
    {

        // ---- SECTION:  members supporting exposed properties -------------*
        public BigInteger Numerator;
        public BigInteger Denominator;

        private static readonly BigRational s_brZero = new BigRational(BigInteger.Zero);
        private static readonly BigRational s_brOne = new BigRational(BigInteger.One);
        private static readonly BigRational s_brMinusOne = new BigRational(BigInteger.MinusOne);

        // ---- SECTION:  members for internal support ---------*
        #region Members for Internal Support
        [StructLayout(LayoutKind.Explicit)]
        internal struct DoubleUlong
        {
            [FieldOffset(0)]
            public double dbl;
            [FieldOffset(0)]
            public ulong uu;
        }
        private const int DoubleMaxScale = 308;
        private static readonly BigInteger s_bnDoublePrecision = BigInteger.Pow(10, DoubleMaxScale);
        private static readonly BigInteger s_bnDoubleMaxValue = (BigInteger)Double.MaxValue;
        private static readonly BigInteger s_bnDoubleMinValue = (BigInteger)Double.MinValue;

        [StructLayout(LayoutKind.Explicit)]
        internal struct DecimalUInt32
        {
            [FieldOffset(0)]
            public Decimal dec;
            [FieldOffset(0)]
            public int flags;
        }
        private const int DecimalScaleMask = 0x00FF0000;
        private const int DecimalSignMask = unchecked((int)0x80000000);
        private const int DecimalMaxScale = 28;
        private static readonly BigInteger s_bnDecimalPrecision = BigInteger.Pow(10, DecimalMaxScale);
        private static readonly BigInteger s_bnDecimalMaxValue = (BigInteger)Decimal.MaxValue;
        private static readonly BigInteger s_bnDecimalMinValue = (BigInteger)Decimal.MinValue;

        private const String c_solidus = @"/";
        #endregion Members for Internal Support

        // ---- SECTION: public properties --------------*
        #region Public Properties
        public static BigRational Zero
        {
            get
            {
                return s_brZero;
            }
        }

        public static BigRational One
        {
            get
            {
                return s_brOne;
            }
        }

        public static BigRational MinusOne
        {
            get
            {
                return s_brMinusOne;
            }
        }

        public Int32 Sign
        {
            get
            {
                return Numerator.Sign;
            }
        }       

        #endregion Public Properties

        // ---- SECTION: public instance methods --------------*
        #region Public Instance Methods

        // GetWholePart() and GetFractionPart()
        // 
        // BigRational == Whole, Fraction
        //  0/2        ==     0,  0/2
        //  1/2        ==     0,  1/2
        // -1/2        ==     0, -1/2
        //  1/1        ==     1,  0/1
        // -1/1        ==    -1,  0/1
        // -3/2        ==    -1, -1/2
        //  3/2        ==     1,  1/2
        public BigInteger GetWholePart()
        {
            return BigInteger.Divide(Numerator, Denominator);
        }

        public BigRational GetFractionPart()
        {
            return new BigRational(BigInteger.Remainder(Numerator, Denominator), Denominator);
        }

        public override bool Equals(Object obj)
        {
            if (obj == null)
                return false;

            if (!(obj is BigRational))
                return false;
            return this.Equals((BigRational)obj);
        }

        public override int GetHashCode()
        {
            // Arithmetic permits unreduced fractions; hash the exact canonical
            // value without mutating the caller's representation.
            var canonical = this;
            canonical.Simplify();
            return HashCode.Combine(canonical.Numerator.GetHashCode(), canonical.Denominator.GetHashCode());
        }

        // IComparable
        int IComparable.CompareTo(Object obj)
        {
            if (obj == null)
                return 1;
            if (!(obj is BigRational))
                throw new ArgumentException("Argument must be of type BigRational", "obj");
            return Compare(this, (BigRational)obj);
        }

        // IComparable<BigRational>
        public int CompareTo(BigRational other)
        {
            return Compare(this, other);
        }

        // Object.ToString
        public override string ToString()
        {
            StringBuilder ret = new StringBuilder();
            ret.Append(Numerator.ToString("R", CultureInfo.InvariantCulture));
            ret.Append(c_solidus);
            ret.Append(Denominator.ToString("R", CultureInfo.InvariantCulture));
            return ret.ToString();
        }

        // IEquatable<BigRational>
        // a/b = c/d, iff ad = bc
        public Boolean Equals(BigRational other)
        {
            if (this.Denominator == other.Denominator)
            {
                return Numerator == other.Numerator;
            }
            else
            {
                return (Numerator * other.Denominator) == (Denominator * other.Numerator);
            }
        }

        #endregion Public Instance Methods

        // -------- SECTION: constructors -----------------*
        #region Constructors

        public BigRational(BigInteger numerator)
        {
            Numerator = numerator;
            Denominator = BigInteger.One;
        }

        public BigRational(int numerator)
        {
            Numerator = numerator;
            Denominator = BigInteger.One;
        }

        // BigRational(Double)
        public BigRational(Double value)
        {
            if (Double.IsNaN(value))
            {
                throw new ArgumentException("Argument is not a number", "value");
            }
            else if (Double.IsInfinity(value))
            {
                throw new ArgumentException("Argument is infinity", "value");
            }

            bool isFinite;
            int sign;
            int exponent;
            ulong significand;
            SplitDoubleIntoParts(value, out sign, out exponent, out significand, out isFinite);

            if (significand == 0)
            {
                this = BigRational.Zero;
                return;
            }

            Numerator = significand;
            // SplitDoubleIntoParts returns value = sign * significand * 2^exponent.
            // Shift powers of two; exponentiating the significand changes the value.
            Denominator = BigInteger.One;
            if (exponent >= 0)
                Numerator <<= exponent;
            else
                Denominator <<= -exponent;
            if (sign < 0)
            {
                Numerator = BigInteger.Negate(Numerator);
            }
            Simplify();
        }

        // BigRational(Decimal) -
        //
        // The Decimal type represents floating point numbers exactly, with no rounding error.
        // Values such as "0.1" in Decimal are actually representable, and convert cleanly
        // to BigRational as "11/10"
        public BigRational(Decimal value)
        {
            int[] bits = Decimal.GetBits(value);
            if (bits == null || bits.Length != 4 || (bits[3] & ~(DecimalSignMask | DecimalScaleMask)) != 0 || (bits[3] & DecimalScaleMask) > (28 << 16))
            {
                throw new ArgumentException("invalid Decimal", "value");
            }

            if (value == Decimal.Zero)
            {
                this = BigRational.Zero;
                return;
            }

            // build up the numerator
            ulong ul = (((ulong)(uint)bits[2]) << 32) | ((ulong)(uint)bits[1]);   // (hi    << 32) | (mid)
            Numerator = (new BigInteger(ul) << 32) | (uint)bits[0];             // (hiMid << 32) | (low)

            bool isNegative = (bits[3] & DecimalSignMask) != 0;
            if (isNegative)
            {
                Numerator = BigInteger.Negate(Numerator);
            }

            // build up the denominator
            int scale = (bits[3] & DecimalScaleMask) >> 16;     // 0-28, power of 10 to divide numerator by
            Denominator = BigInteger.Pow(10, scale);

            Simplify();
        }

        public BigRational(BigInteger numerator, BigInteger denominator)
        {
            if (denominator.Sign == 0)
            {
                throw new DivideByZeroException();
            }
            else if (numerator.Sign == 0)
            {
                // 0/m -> 0/1
                Numerator = BigInteger.Zero;
                Denominator = BigInteger.One;
            }
            else if (denominator.Sign < 0)
            {
                Numerator = BigInteger.Negate(numerator);
                Denominator = BigInteger.Negate(denominator);
            }
            else
            {
                Numerator = numerator;
                Denominator = denominator;
            }
            //Simplify();
        }

        public BigRational(BigInteger whole, BigInteger numerator, BigInteger denominator)
        {
            if (denominator.Sign == 0)
            {
                throw new DivideByZeroException();
            }
            else if (numerator.Sign == 0 && whole.Sign == 0)
            {
                Numerator = BigInteger.Zero;
                Denominator = BigInteger.One;
            }
            else if (denominator.Sign < 0)
            {
                Denominator = BigInteger.Negate(denominator);
                Numerator = (BigInteger.Negate(whole) * Denominator) + BigInteger.Negate(numerator);
            }
            else
            {
                Denominator = denominator;
                Numerator = (whole * denominator) + numerator;
            }
            Simplify();
        }
        #endregion Constructors

        // -------- SECTION: public static methods -----------------*
        #region Public Static Methods

        public static BigRational Abs(BigRational r)
        {
            return (r.Numerator.Sign < 0 ? new BigRational(BigInteger.Abs(r.Numerator), r.Denominator) : r);
        }

        public static BigRational Negate(BigRational r)
        {
            return new BigRational(BigInteger.Negate(r.Numerator), r.Denominator);
        }

        public static BigRational Invert(BigRational r)
        {
            return new BigRational(r.Denominator, r.Numerator);
        }

        public static BigRational Add(BigRational x, BigRational y)
        {
            return x + y;
        }

        public static BigRational Subtract(BigRational x, BigRational y)
        {
            return x - y;
        }


        public static BigRational Multiply(BigRational x, BigRational y)
        {
            return x * y;
        }

        public static BigRational Divide(BigRational dividend, BigRational divisor)
        {
            return dividend / divisor;
        }

        public static BigRational Remainder(BigRational dividend, BigRational divisor)
        {
            return dividend % divisor;
        }

        public static BigRational DivRem(BigRational dividend, BigRational divisor, out BigRational remainder)
        {
            // a/b / c/d  == (ad)/(bc)
            // a/b % c/d  == (ad % bc)/bd

            // (ad) and (bc) need to be calculated for both the division and the remainder operations.
            BigInteger ad = dividend.Numerator * divisor.Denominator;
            BigInteger bc = dividend.Denominator * divisor.Numerator;
            BigInteger bd = dividend.Denominator * divisor.Denominator;

            remainder = new BigRational(ad % bc, bd);
            return new BigRational(ad, bc);
        }


        public static BigRational Pow(BigRational baseValue, BigInteger exponent)
        {
            if (exponent.Sign == 0)
            {
                // 0^0 -> 1
                // n^0 -> 1
                return BigRational.One;
            }
            else if (exponent.Sign < 0)
            {
                if (baseValue == BigRational.Zero)
                {
                    throw new ArgumentException("cannot raise zero to a negative power", "baseValue");
                }
                // n^(-e) -> (1/n)^e
                baseValue = BigRational.Invert(baseValue);
                exponent = BigInteger.Negate(exponent);
            }

            BigRational result = baseValue;
            while (exponent > BigInteger.One)
            {
                result = result * baseValue;
                exponent--;
            }

            return result;
        }

        // Least Common Denominator (LCD)
        //
        // The LCD is the least common multiple of the two denominators.  For instance, the LCD of
        // {1/2, 1/4} is 4 because the least common multiple of 2 and 4 is 4.  Likewise, the LCD
        // of {1/2, 1/3} is 6.
        //       
        // To find the LCD:
        //
        // 1) Find the Greatest Common Divisor (GCD) of the denominators
        // 2) Multiply the denominators together
        // 3) Divide the product of the denominators by the GCD
        public static BigInteger LeastCommonDenominator(BigRational x, BigRational y)
        {
            // LCD( a/b, c/d ) == (bd) / gcd(b,d)
            return (x.Denominator * y.Denominator) / BigInteger.GreatestCommonDivisor(x.Denominator, y.Denominator);
        }

        public static int Compare(BigRational r1, BigRational r2)
        {
            //     a/b = c/d, iff ad = bc
            return BigInteger.Compare(r1.Numerator * r2.Denominator, r2.Numerator * r1.Denominator);
        }
        #endregion Public Static Methods

        #region Operator Overloads
        public static bool operator ==(BigRational x, BigRational y)
        {
            return Compare(x, y) == 0;
        }

        public static bool operator !=(BigRational x, BigRational y)
        {
            return Compare(x, y) != 0;
        }

        public static bool operator <(BigRational x, BigRational y)
        {
            return Compare(x, y) < 0;
        }

        public static bool operator <=(BigRational x, BigRational y)
        {
            return Compare(x, y) <= 0;
        }

        public static bool operator >(BigRational x, BigRational y)
        {
            return Compare(x, y) > 0;
        }

        public static bool operator >=(BigRational x, BigRational y)
        {
            return Compare(x, y) >= 0;
        }

        public static BigRational operator +(BigRational r)
        {
            return r;
        }

        public static BigRational operator -(BigRational r)
        {
            return new BigRational(-r.Numerator, r.Denominator);
        }

        public static BigRational operator ++(BigRational r)
        {
            return r + BigRational.One;
        }

        public static BigRational operator --(BigRational r)
        {
            return r - BigRational.One;
        }

        public static BigRational operator +(BigRational r1, BigRational r2)
        {
            if (r1.Denominator == r2.Denominator)
                return new BigRational(r1.Numerator + r2.Numerator, r1.Denominator);
            // Use the least common denominator. Geometry frequently combines
            // related, unreduced fractions; multiplying their denominators
            // repeatedly creates huge intermediates without adding precision.
            var common = BigInteger.GreatestCommonDivisor(r1.Denominator, r2.Denominator);
            var leftScale = r2.Denominator / common;
            var rightScale = r1.Denominator / common;
            return new BigRational(r1.Numerator * leftScale + r2.Numerator * rightScale,
                r1.Denominator * leftScale);
        }

        public static BigRational operator -(BigRational r1, BigRational r2)
        {
            if (r1.Denominator == r2.Denominator)
                return new BigRational(r1.Numerator - r2.Numerator, r1.Denominator);
            var common = BigInteger.GreatestCommonDivisor(r1.Denominator, r2.Denominator);
            var leftScale = r2.Denominator / common;
            var rightScale = r1.Denominator / common;
            return new BigRational(r1.Numerator * leftScale - r2.Numerator * rightScale,
                r1.Denominator * leftScale);
        }

        public static BigRational operator *(BigRational r1, BigRational r2)
        {
            // a/b * c/d  == (ac)/(bd)
            return new BigRational((r1.Numerator * r2.Numerator), (r1.Denominator * r2.Denominator));
        }

        public static BigRational operator /(BigRational r1, BigRational r2)
        {
            // a/b / c/d  == (ad)/(bc)
            return new BigRational((r1.Numerator * r2.Denominator), (r1.Denominator * r2.Numerator));
        }

        public static BigRational operator %(BigRational r1, BigRational r2)
        {
            // a/b % c/d  == (ad % bc)/bd
            return new BigRational((r1.Numerator * r2.Denominator) % (r1.Denominator * r2.Numerator), (r1.Denominator * r2.Denominator));
        }
        #endregion Operator Overloads

        // ----- SECTION: explicit conversions from BigRational to numeric base types  ----------------*
        #region explicit conversions from BigRational
        public static explicit operator SByte(BigRational value)
        {
            return (SByte)(BigInteger.Divide(value.Numerator, value.Denominator));
        }

        public static explicit operator UInt16(BigRational value)
        {
            return (UInt16)(BigInteger.Divide(value.Numerator, value.Denominator));
        }

        public static explicit operator UInt32(BigRational value)
        {
            return (UInt32)(BigInteger.Divide(value.Numerator, value.Denominator));
        }

        public static explicit operator UInt64(BigRational value)
        {
            return (UInt64)(BigInteger.Divide(value.Numerator, value.Denominator));
        }

        public static explicit operator Byte(BigRational value)
        {
            return (Byte)(BigInteger.Divide(value.Numerator, value.Denominator));
        }

        public static explicit operator Int16(BigRational value)
        {
            return (Int16)(BigInteger.Divide(value.Numerator, value.Denominator));
        }

        public static explicit operator Int32(BigRational value)
        {
            return (Int32)(BigInteger.Divide(value.Numerator, value.Denominator));
        }

        public static explicit operator Int64(BigRational value)
        {
            return (Int64)(BigInteger.Divide(value.Numerator, value.Denominator));
        }

        public static explicit operator BigInteger(BigRational value)
        {
            return BigInteger.Divide(value.Numerator, value.Denominator);
        }

        public static explicit operator Single(BigRational value)
        {
            // The Single value type represents a single-precision 32-bit number with
            // values ranging from negative 3.402823e38 to positive 3.402823e38      
            // values that do not fit into this range are returned as Infinity
            return (Single)((Double)value);
        }

        public static explicit operator Double(BigRational value)
        {
            // The Double value type represents a double-precision 64-bit number with
            // values ranging from -1.79769313486232e308 to +1.79769313486232e308
            // values that do not fit into this range are returned as +/-Infinity
            if (SafeCastToDouble(value.Numerator) && SafeCastToDouble(value.Denominator))
            {
                return (Double)value.Numerator / (Double)value.Denominator;
            }

            // scale the numerator to preseve the fraction part through the integer division
            BigInteger denormalized = (value.Numerator * s_bnDoublePrecision) / value.Denominator;
            if (denormalized.IsZero)
                return (value.Sign < 0) ? BitConverter.Int64BitsToDouble(unchecked((long)0x8000000000000000)) : 0d; // underflow to -+0

            Double result = 0;
            bool isDouble = false;
            int scale = DoubleMaxScale;

            while (scale > 0)
            {
                if (!isDouble)
                {
                    if (SafeCastToDouble(denormalized))
                    {
                        result = (Double)denormalized;
                        isDouble = true;
                    }
                    else
                    {
                        denormalized = denormalized / 10;
                    }
                }
                result = result / 10;
                scale--;
            }

            if (!isDouble)
                return (value.Sign < 0) ? Double.NegativeInfinity : Double.PositiveInfinity;
            else
                return result;
        }

        public static explicit operator Decimal(BigRational value)
        {
            // The Decimal value type represents decimal numbers ranging
            // from +79,228,162,514,264,337,593,543,950,335 to -79,228,162,514,264,337,593,543,950,335
            // the binary representation of a Decimal value is of the form, ((-2^96 to 2^96) / 10^(0 to 28))
            if (SafeCastToDecimal(value.Numerator) && SafeCastToDecimal(value.Denominator))
            {
                return (Decimal)value.Numerator / (Decimal)value.Denominator;
            }

            // scale the numerator to preseve the fraction part through the integer division
            BigInteger denormalized = (value.Numerator * s_bnDecimalPrecision) / value.Denominator;
            if (denormalized.IsZero)
            {
                return Decimal.Zero; // underflow - fraction is too small to fit in a decimal
            }
            for (int scale = DecimalMaxScale; scale >= 0; scale--)
            {
                if (!SafeCastToDecimal(denormalized))
                {
                    denormalized = denormalized / 10;
                }
                else
                {
                    DecimalUInt32 dec = new DecimalUInt32();
                    dec.dec = (Decimal)denormalized;
                    dec.flags = (dec.flags & ~DecimalScaleMask) | (scale << 16);
                    return dec.dec;
                }
            }
            throw new OverflowException("Value was either too large or too small for a Decimal.");
        }
        #endregion explicit conversions from BigRational

        // ----- SECTION: implicit conversions from numeric base types to BigRational  ----------------*
        #region implicit conversions to BigRational

        public static implicit operator BigRational(SByte value)
        {
            return new BigRational((BigInteger)value);
        }

        public static implicit operator BigRational(UInt16 value)
        {
            return new BigRational((BigInteger)value);
        }

        public static implicit operator BigRational(UInt32 value)
        {
            return new BigRational((BigInteger)value);
        }

        public static implicit operator BigRational(UInt64 value)
        {
            return new BigRational((BigInteger)value);
        }

        public static implicit operator BigRational(Byte value)
        {
            return new BigRational((BigInteger)value);
        }

        public static implicit operator BigRational(Int16 value)
        {
            return new BigRational((BigInteger)value);
        }

        public static implicit operator BigRational(Int32 value)
        {
            return new BigRational((BigInteger)value);
        }

        public static implicit operator BigRational(Int64 value)
        {
            return new BigRational((BigInteger)value);
        }

        public static implicit operator BigRational(BigInteger value)
        {
            return new BigRational(value);
        }

        public static implicit operator BigRational(Single value)
        {
            return new BigRational((Double)value);
        }

        public static implicit operator BigRational(Double value)
        {
            return new BigRational(value);
        }

        public static implicit operator BigRational(Decimal value)
        {
            return new BigRational(value);
        }

        #endregion implicit conversions to BigRational

        // ----- SECTION: private serialization instance methods  ----------------*
        #region serialization
        void IDeserializationCallback.OnDeserialization(Object sender)
        {
            try
            {
                // verify that the deserialized number is well formed
                if (Denominator.Sign == 0 || Numerator.Sign == 0)
                {
                    // n/0 -> 0/1
                    // 0/m -> 0/1
                    Numerator = BigInteger.Zero;
                    Denominator = BigInteger.One;
                }
                else if (Denominator.Sign < 0)
                {
                    Numerator = BigInteger.Negate(Numerator);
                    Denominator = BigInteger.Negate(Denominator);
                }
                Simplify();
            }
            catch (ArgumentException e)
            {
                throw new SerializationException("invalid serialization data", e);
            }
        }

        void ISerializable.GetObjectData(SerializationInfo info, StreamingContext context)
        {
            if (info == null)
            {
                throw new ArgumentNullException("info");
            }

            info.AddValue("Numerator", Numerator);
            info.AddValue("Denominator", Denominator);
        }

        BigRational(SerializationInfo info, StreamingContext context)
        {
            if (info == null)
            {
                throw new ArgumentNullException("info");
            }

            Numerator = (BigInteger)info.GetValue("Numerator", typeof(BigInteger));
            Denominator = (BigInteger)info.GetValue("Denominator", typeof(BigInteger));
        }
        #endregion serialization

        // ----- SECTION: private instance utility methods ----------------*
        #region instance helper methods
        public void Simplify()
        {
            // * if the numerator is {0, +1, -1} then the fraction is already reduced
            // * if the denominator is {+1} then the fraction is already reduced
            if (Numerator == BigInteger.Zero)
            {
                Denominator = BigInteger.One;
            }

            BigInteger gcd = BigInteger.GreatestCommonDivisor(Numerator, Denominator);
            if (gcd > BigInteger.One)
            {
                Numerator = Numerator / gcd;
                Denominator = Denominator / gcd;
            }

#if DEBUG
            if (Denominator < 0)
                throw new Exception("The standard form requires Denominator >= 0");
#endif
        }
        #endregion instance helper methods

        // ----- SECTION: private static utility methods -----------------*
        #region static helper methods
        private static bool SafeCastToDouble(BigInteger value)
        {
            return s_bnDoubleMinValue <= value && value <= s_bnDoubleMaxValue;
        }

        private static bool SafeCastToDecimal(BigInteger value)
        {
            return s_bnDecimalMinValue <= value && value <= s_bnDecimalMaxValue;
        }

        private static void SplitDoubleIntoParts(double dbl, out int sign, out int exp, out ulong man, out bool isFinite)
        {
            DoubleUlong du;
            du.uu = 0;
            du.dbl = dbl;

            sign = 1 - ((int)(du.uu >> 62) & 2);
            man = du.uu & 0x000FFFFFFFFFFFFF;
            exp = (int)(du.uu >> 52) & 0x7FF;
            if (exp == 0)
            {
                // Denormalized number.
                isFinite = true;
                if (man != 0)
                    exp = -1074;
            }
            else if (exp == 0x7FF)
            {
                // NaN or Infinite.
                isFinite = false;
                exp = Int32.MaxValue;
            }
            else
            {
                isFinite = true;
                man |= 0x0010000000000000; // mask in the implied leading 53rd significand bit
                exp -= 1075;
            }
        }

        private static double GetDoubleFromParts(int sign, int exp, ulong man)
        {
            DoubleUlong du;
            du.dbl = 0;

            if (man == 0)
            {
                du.uu = 0;
            }
            else
            {
                // Normalize so that 0x0010 0000 0000 0000 is the highest bit set
                int cbitShift = CbitHighZero(man) - 11;
                if (cbitShift < 0)
                    man >>= -cbitShift;
                else
                    man <<= cbitShift;

                // Move the point to just behind the leading 1: 0x001.0 0000 0000 0000
                // (52 bits) and skew the exponent (by 0x3FF == 1023)
                exp += 1075;

                if (exp >= 0x7FF)
                {
                    // Infinity
                    du.uu = 0x7FF0000000000000;
                }
                else if (exp <= 0)
                {
                    // Denormalized
                    exp--;
                    if (exp < -52)
                    {
                        // Underflow to zero
                        du.uu = 0;
                    }
                    else
                    {
                        du.uu = man >> -exp;
                    }
                }
                else
                {
                    // Mask off the implicit high bit
                    du.uu = (man & 0x000FFFFFFFFFFFFF) | ((ulong)exp << 52);
                }
            }

            if (sign < 0)
            {
                du.uu |= 0x8000000000000000;
            }

            return du.dbl;
        }

        private static int CbitHighZero(ulong uu)
        {
            if ((uu & 0xFFFFFFFF00000000) == 0)
                return 32 + CbitHighZero((uint)uu);
            return CbitHighZero((uint)(uu >> 32));
        }

        private static int CbitHighZero(uint u)
        {
            if (u == 0)
                return 32;

            int cbit = 0;
            if ((u & 0xFFFF0000) == 0)
            {
                cbit += 16;
                u <<= 16;
            }
            if ((u & 0xFF000000) == 0)
            {
                cbit += 8;
                u <<= 8;
            }
            if ((u & 0xF0000000) == 0)
            {
                cbit += 4;
                u <<= 4;
            }
            if ((u & 0xC0000000) == 0)
            {
                cbit += 2;
                u <<= 2;
            }
            if ((u & 0x80000000) == 0)
                cbit += 1;
            return cbit;
        }

        public static BigRational Max(BigRational a, BigRational b)
        {
            return a > b ? a : b;
        }

        public static BigRational Min(BigRational a, BigRational b)
        {
            return a < b ? a : b;
        }

        #endregion static helper methods
    } // BigRational
}
