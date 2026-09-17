//#define FAST_BIG_RAT

using System.Numerics;

#if FAST_BIG_RAT
using Rat = GeoCore.BigRat;
#else
using Rat = GeoCore.BigRational;
#endif

namespace GeoCore
{
    // Used to have a nullable BigRational - consumes less memory if not used
    public class BigRationalClass
    {
        //public BigRational Value;
        public Rat Value;

        public BigRationalClass(Rat value)
        {
            Value = value;
        }

#if FAST_BIG_RAT
        public void Simplify()
        {
            // * if the numerator is {0, +1, -1} then the fraction is already reduced
            // * if the denominator is {+1} then the fraction is already reduced
            var m_numerator = Numerator();
            var m_denominator = Denominator();
            if (m_numerator == BigInteger.Zero)
            {
                m_denominator = BigInteger.One;
            }

            BigInteger gcd = BigInteger.GreatestCommonDivisor(m_numerator, m_denominator);
            if (gcd > BigInteger.One)
            {
                m_numerator = m_numerator / gcd;
                m_denominator = m_denominator / gcd;
            }

#if DEBUG
            if (m_denominator < 0)
                throw new Exception("The standard form requires m_denominator >= 0");
#endif
            Value = (Rat)m_numerator / (Rat)m_denominator;
            //Value.Normalize();
        }
        public int Sign()
        {
            return Value.CompareTo(0);
        }
        public BigInteger Numerator()
        {
            return (BigInteger)Value.Numerator();
        }
        public BigInteger Denominator()
        {
            return (BigInteger)Value.Denominator();
        }
#else
        public void Simplify()
        {
            Value.Simplify();
        }
        public int Sign()
        {
            return Value.Sign;
        }
        public BigInteger Numerator()
        {
            return Value.Numerator;
        }
        public BigInteger Denominator()
        {
            return Value.Denominator;
        }
#endif
    }

    public struct BigRationalHybrid : IEquatable<BigRationalHybrid>
    {
        private BigRationalClass rationalValue; // Lazy initialization for performance
        private long value;
        private bool isSimplified;
        private bool isLongValue; // true if long value is active, false if rationalValue is active

        public static readonly BigRationalHybrid Zero = new BigRationalHybrid(0);
        public static readonly BigRationalHybrid One = new BigRationalHybrid(1);

        public BigRationalHybrid(int value)
        {
            this.value = value;
            rationalValue = null; // Lazy initialization in GetRationalValue()
            isSimplified = true;
            isLongValue = true;
        }
        public BigRationalHybrid(long value)
        {
            this.value = value;
            rationalValue = null; // Lazy initialization in GetRationalValue()
            isSimplified = true;
            isLongValue = true;
        }
#if FAST_BIG_RAT
        public BigRationalHybrid(BigInteger num, BigInteger denom)
            : this()
        {
            throw new NotImplementedException();
        }
#else
        public BigRationalHybrid(BigInteger num, BigInteger denom)
            : this(new Rat(num, denom))
        {            
        }
#endif

        public BigInteger Numerator()
        {
            if (isLongValue)
                return value;
            return rationalValue.Numerator();
        }
        public BigInteger Denominator()
        {
            if (isLongValue)
                return 1;
            return rationalValue.Denominator();
        }


        // Copy constructor
        public BigRationalHybrid(BigRationalHybrid source)
        {
            this.value = source.value;
            // Only copy rationalValue if it was already initialized
            if (source.rationalValue != null)
                rationalValue = new BigRationalClass(source.rationalValue.Value);
            else
                rationalValue = null;
            isSimplified = source.isSimplified;
            isLongValue = source.isLongValue;
        }

        private BigRationalHybrid(Rat value)
        {
            this.value = 0; // Not used when isLongValue is false
            rationalValue = new BigRationalClass(value);
            isSimplified = false;
            isLongValue = false;
        }

        // This should be the only method that mutates a BigRationalHybrid in place
        public void Simplify()
        {
            if (isSimplified)
                return;
            
            rationalValue.Simplify();

            if(rationalValue.Denominator() == 1)
            {
                var num = rationalValue.Numerator();
                if(num >= long.MinValue && num <= long.MaxValue)
                {
                    this.value = (long)num;
                    isLongValue = true;
                }
            }

            isSimplified= true;
        }        

        public bool IsInt32
        {
            get { return isLongValue && value >= int.MinValue && value <= int.MaxValue; }
        }

        public double ToDouble()
        {
            if (isLongValue)
                return value;

            return (double)rationalValue.Value;
        }
        //TODO: return long
        public int ToIntRoundDown()
        {
            if (isLongValue)
                return (int)value;
            int v = (int)rationalValue.Value;
            while (v > rationalValue.Value)
                --v;
            return v;
        }
        public long ToLongRoundDown()
        {
            if (isLongValue)
                return value;
            long v = (long)rationalValue.Value;
            while (v > rationalValue.Value)
                --v;
            return v;
        }
        public long ToLongRoundUp()
        {
            if (isLongValue)
                return value;
            long v = (long)rationalValue.Value;
            while (v < rationalValue.Value)
                ++v;
            return v;
        }

        public int Sign()
        {
            if (!isLongValue)
            {
                var s = rationalValue.Sign();
                return s;
                //if (rationalValue.Value > 0)
                //    return 1;
                //else if (rationalValue.Value < 0)
                //    return -1;
                //return 0;
            }
            return Math.Sign(value);
        }

        public static BigRationalHybrid Abs(BigRationalHybrid x)
        {
            if (x < Zero)
                return -x;
            return x;
        }

        public static BigRationalHybrid Max(in BigRationalHybrid a, in BigRationalHybrid b)
        {
            // If a is greater than or equal to b, return a; otherwise return b.
            return (a >= b) ? a : b;
        }

        public static BigRationalHybrid Min(in BigRationalHybrid a, in BigRationalHybrid b)
        {
            // If a is less than or equal to b, return a; otherwise return b.
            return (a <= b) ? a : b;
        }

        public int CompareTo(in BigRationalHybrid b)
        {
            // Returns -1 if a < b, 0 if they are equal, or 1 if a > b.
            if (this < b) return -1;
            if (this > b) return 1;
            return 0;
        }

        public static int Sign(in BigRationalHybrid a)
        {
            if (a < Zero) return -1;
            if (a > Zero) return 1;
            return 0;
        }

        /// <summary>
        /// Exact sign of (ux·vy − uy·vx). Fuses the two products and subtraction into one BigInteger
        /// comparison when rationals are involved, and uses 128-bit integer arithmetic when all four
        /// values are in the long fast path (same result as <c>ux * vy - uy * vx</c> then <see cref="Sign"/>).
        /// </summary>
        public static int SignOfCrossProduct(in BigRationalHybrid ux, in BigRationalHybrid vy,
            in BigRationalHybrid uy, in BigRationalHybrid vx)
        {
            if (ux.isLongValue && vy.isLongValue && uy.isLongValue && vx.isLongValue)
            {
                Int128 p = (Int128)ux.value * vy.value;
                Int128 q = (Int128)uy.value * vx.value;
                Int128 d = p - q;
                if (d > 0) return 1;
                if (d < 0) return -1;
                return 0;
            }

            GetNumDenom(in ux, out BigInteger nUx, out BigInteger dUx);
            GetNumDenom(in vy, out BigInteger nVy, out BigInteger dVy);
            GetNumDenom(in uy, out BigInteger nUy, out BigInteger dUy);
            GetNumDenom(in vx, out BigInteger nVx, out BigInteger dVx);

            // ux*vy − uy*vx = (nUx*nVy*dUy*dVx − nUy*nVx*dUx*dVy) / (dUx*dVy*dUy*dVx); denominator > 0.
            BigInteger t1 = nUx * nVy * dUy * dVx;
            BigInteger t2 = nUy * nVx * dUx * dVy;
            return BigInteger.Compare(t1, t2);
        }

        /// <summary>
        /// Exact in-circle sign when all eight coordinates use the long fast path.
        /// Same predicate as Delaunay <c>InCircle</c>; uses <see cref="Int128"/> like integer coordinates.
        /// Returns false when rational arithmetic is required.
        /// </summary>
        public static bool TryInCircleSign(
            in BigRationalHybrid pax, in BigRationalHybrid pay,
            in BigRationalHybrid pbx, in BigRationalHybrid pby,
            in BigRationalHybrid pcx, in BigRationalHybrid pcy,
            in BigRationalHybrid pdx, in BigRationalHybrid pdy,
            out int sign)
        {
            sign = 0;
            if (!pax.isLongValue || !pay.isLongValue || !pbx.isLongValue || !pby.isLongValue ||
                !pcx.isLongValue || !pcy.isLongValue || !pdx.isLongValue || !pdy.isLongValue)
                return false;

            // Widen before subtracting, including opposite Int32 endpoints.
            Int128 adx = (Int128)pax.value - pdx.value;
            Int128 ady = (Int128)pay.value - pdy.value;
            Int128 bdx = (Int128)pbx.value - pdx.value;
            Int128 bdy = (Int128)pby.value - pdy.value;
            Int128 cdx = (Int128)pcx.value - pdx.value;
            Int128 cdy = (Int128)pcy.value - pdy.value;
            // Six fourth-degree products are bounded by 12 * limit^4.
            // Larger inputs must use the caller's arbitrary-precision path.
            const long limit = 1L << 30;
            if (Int128.Abs(adx) > limit || Int128.Abs(ady) > limit ||
                Int128.Abs(bdx) > limit || Int128.Abs(bdy) > limit ||
                Int128.Abs(cdx) > limit || Int128.Abs(cdy) > limit)
                return false;

            Int128 abdet = adx * bdy - bdx * ady;
            Int128 bcdet = bdx * cdy - cdx * bdy;
            Int128 cadet = cdx * ady - adx * cdy;
            Int128 alift = adx * adx + ady * ady;
            Int128 blift = bdx * bdx + bdy * bdy;
            Int128 clift = cdx * cdx + cdy * cdy;

            Int128 result = alift * bcdet + blift * cadet + clift * abdet;

            if (result > 0) sign = 1;
            else if (result < 0) sign = -1;
            return true;
        }

        public static long Orient3DFastCount;
        public static long Orient3DSlowCount;
        public static long Orient3DFilterHitCount;
        public static long Orient3DFilterMissCount;

        /// <summary>
        /// Exact orientation sign. Uses Int128 for Int32 coordinates, certified
        /// integer bounds for suitable rationals, and BigInteger otherwise.
        /// See ExactOrientation.md for error and overflow bounds.
        /// </summary>
        public static int SignOfOrient3D(
            in BigRationalHybrid pax, in BigRationalHybrid pay, in BigRationalHybrid paz,
            in BigRationalHybrid pbx, in BigRationalHybrid pby, in BigRationalHybrid pbz,
            in BigRationalHybrid pcx, in BigRationalHybrid pcy, in BigRationalHybrid pcz,
            in BigRationalHybrid pdx, in BigRationalHybrid pdy, in BigRationalHybrid pdz)
        {
            // Int32 coordinate differences need at most 33 bits. The cubic
            // determinant then fits Int128; arbitrary Int64 coordinates do not.
            if (pax.IsInt32 && pay.IsInt32 && paz.IsInt32 &&
                pbx.IsInt32 && pby.IsInt32 && pbz.IsInt32 &&
                pcx.IsInt32 && pcy.IsInt32 && pcz.IsInt32 &&
                pdx.IsInt32 && pdy.IsInt32 && pdz.IsInt32)
            {
                Int128 adx = (Int128)pax.value - pdx.value;
                Int128 bdx = (Int128)pbx.value - pdx.value;
                Int128 cdx = (Int128)pcx.value - pdx.value;
                Int128 ady = (Int128)pay.value - pdy.value;
                Int128 bdy = (Int128)pby.value - pdy.value;
                Int128 cdy = (Int128)pcy.value - pdy.value;
                Int128 adz = (Int128)paz.value - pdz.value;
                Int128 bdz = (Int128)pbz.value - pdz.value;
                Int128 cdz = (Int128)pcz.value - pdz.value;

                Int128 bdxcdy = bdx * cdy;
                Int128 cdxbdy = cdx * bdy;
                Int128 cdxady = cdx * ady;
                Int128 adxcdy = adx * cdy;
                Int128 bdxady = bdx * ady;
                Int128 adxbdy = adx * bdy;

                Int128 left = adz * (bdxcdy - cdxbdy) + bdz * (cdxady - adxcdy);
                Int128 right = cdz * (bdxady - adxbdy);
                Int128 det = left - right;
                if (det > 0) return 1;
                if (det < 0) return -1;
                return 0;
            }

            // Integer bounds may prove a nonzero sign without constructing the
            // full rational determinant. Uncertain or out-of-range cases fall
            // through to arbitrary precision; geometry is never quantized here.
            if (TrySignOfOrient3DIntegerBounds(
                pax, pay, paz, pbx, pby, pbz, pcx, pcy, pcz, pdx, pdy, pdz, out int boundedSign))
                return boundedSign;
            return SignOfOrient3DExact(
                in pax, in pay, in paz, in pbx, in pby, in pbz,
                in pcx, in pcy, in pcz, in pdx, in pdy, in pdz);
        }

        private const int OrientBoundsFractionBits = 20;
        private const long OrientBoundsCoordinateLimit = 1L << 40;

        /// <summary>
        /// Certifies a nonzero orientation using integer bounds only. False
        /// means "unknown", including exact zero; use SignOfOrient3DExact then.
        /// Coordinates and rational values are neither modified nor replaced.
        /// </summary>
        public static bool TrySignOfOrient3DIntegerBounds(
            in BigRationalHybrid pax, in BigRationalHybrid pay, in BigRationalHybrid paz,
            in BigRationalHybrid pbx, in BigRationalHybrid pby, in BigRationalHybrid pbz,
            in BigRationalHybrid pcx, in BigRationalHybrid pcy, in BigRationalHybrid pcz,
            in BigRationalHybrid pdx, in BigRationalHybrid pdy, in BigRationalHybrid pdz,
            out int sign)
        {
            sign = 0;
            if (!TryOrientBoundCoordinate(pax, out long ax) || !TryOrientBoundCoordinate(pay, out long ay) ||
                !TryOrientBoundCoordinate(paz, out long az) || !TryOrientBoundCoordinate(pbx, out long bx) ||
                !TryOrientBoundCoordinate(pby, out long by) || !TryOrientBoundCoordinate(pbz, out long bz) ||
                !TryOrientBoundCoordinate(pcx, out long cx) || !TryOrientBoundCoordinate(pcy, out long cy) ||
                !TryOrientBoundCoordinate(pcz, out long cz) || !TryOrientBoundCoordinate(pdx, out long dx) ||
                !TryOrientBoundCoordinate(pdy, out long dy) || !TryOrientBoundCoordinate(pdz, out long dz))
                return false;

            long adx=ax-dx, ady=ay-dy, adz=az-dz;
            long bdx=bx-dx, bdy=by-dy, bdz=bz-dz;
            long cdx=cx-dx, cdy=cy-dy, cdz=cz-dz;
            Int128 determinant = (Int128)adz*((Int128)bdx*cdy-(Int128)cdx*bdy)
                + (Int128)bdz*((Int128)cdx*ady-(Int128)adx*cdy)
                - (Int128)cdz*((Int128)bdx*ady-(Int128)adx*bdy);

            long max = Math.Max(Math.Max(Math.Abs(adx),Math.Abs(ady)),Math.Abs(adz));
            max = Math.Max(max,Math.Max(Math.Max(Math.Abs(bdx),Math.Abs(bdy)),Math.Abs(bdz)));
            max = Math.Max(max,Math.Max(Math.Max(Math.Abs(cdx),Math.Abs(cdy)),Math.Abs(cdz)));
            Int128 m=max;
            // q=trunc(2^20*x) differs from the exact scaled coordinate by <1.
            // Each coordinate difference therefore has absolute error <2.
            // For each determinant monomial abc, |a|,|b|,|c|<=M:
            // |(a+e)(b+f)(c+g)-abc| <= 6M^2+12M+8.
            // There are six signed monomials, so E=36M^2+72M+48 bounds
            // the total error. Strict comparison alone certifies a sign.
            Int128 error = 36*m*m + 72*m + 48;
            // |q|<=2^40 => M<=2^41. Both 6*M^3 and E fit Int128,
            // including every intermediate operation above. No wrapping occurs.
            if (determinant > error) { sign=1; return true; }
            if (determinant < -error) { sign=-1; return true; }
            return false;
        }

        private static bool TryOrientBoundCoordinate(in BigRationalHybrid coordinate, out long scaled)
        {
            if (coordinate.isLongValue)
            {
                const long limit = OrientBoundsCoordinateLimit >> OrientBoundsFractionBits;
                if (coordinate.value < -limit || coordinate.value > limit)
                { scaled=0; return false; }
                scaled=coordinate.value << OrientBoundsFractionBits;
                return true;
            }
            GetNumDenom(coordinate, out var numerator, out var denominator);
            // BigInteger division truncates toward zero, for either sign.
            // This gives an exact error bound even for thousand-bit rationals.
            var quotient=(numerator << OrientBoundsFractionBits)/denominator;
            if (quotient.CompareTo(-OrientBoundsCoordinateLimit)<0 ||
                quotient.CompareTo(OrientBoundsCoordinateLimit)>0)
            { scaled=0; return false; }
            scaled=(long)quotient;
            return true;
        }

        /// <summary>
        /// Compatibility entry point. Uses the integer-only certified bounds;
        /// no floating-point conversion or heuristic error estimate is used.
        /// </summary>
        [Obsolete("Use TrySignOfOrient3DIntegerBounds; this compatibility method also uses integer bounds.")]
        public static bool TrySignOfOrient3DDoubleFilter(
            in BigRationalHybrid pax, in BigRationalHybrid pay, in BigRationalHybrid paz,
            in BigRationalHybrid pbx, in BigRationalHybrid pby, in BigRationalHybrid pbz,
            in BigRationalHybrid pcx, in BigRationalHybrid pcy, in BigRationalHybrid pcz,
            in BigRationalHybrid pdx, in BigRationalHybrid pdy, in BigRationalHybrid pdz,
            out int sign)
        {
            return TrySignOfOrient3DIntegerBounds(
                pax, pay, paz, pbx, pby, pbz, pcx, pcy, pcz, pdx, pdy, pdz, out sign);
        }

        /// <summary>Exact BigInteger determinant sign (no float filter, no Int128 fast path).</summary>
        public static int SignOfOrient3DExact(
            in BigRationalHybrid pax, in BigRationalHybrid pay, in BigRationalHybrid paz,
            in BigRationalHybrid pbx, in BigRationalHybrid pby, in BigRationalHybrid pbz,
            in BigRationalHybrid pcx, in BigRationalHybrid pcy, in BigRationalHybrid pcz,
            in BigRationalHybrid pdx, in BigRationalHybrid pdy, in BigRationalHybrid pdz)
        {
            // Clear each coordinate column by its positive least common
            // denominator. This scales the determinant by a positive factor,
            // preserving its exact sign without multiplying denominators anew
            // at every subtraction, minor and sum.
            ScaleCoordinateDifferences(pax, pbx, pcx, pdx, out var ax, out var bx, out var cx);
            ScaleCoordinateDifferences(pay, pby, pcy, pdy, out var ay, out var by, out var cy);
            ScaleCoordinateDifferences(paz, pbz, pcz, pdz, out var az, out var bz, out var cz);
            return (az * (bx * cy - cx * by) + bz * (cx * ay - ax * cy)
                - cz * (bx * ay - ax * by)).Sign;
        }

        private static void ScaleCoordinateDifferences(
            in BigRationalHybrid a, in BigRationalHybrid b,
            in BigRationalHybrid c, in BigRationalHybrid d,
            out BigInteger ax, out BigInteger bx, out BigInteger cx)
        {
            GetNumDenom(a, out var an, out var ad);
            GetNumDenom(b, out var bn, out var bd);
            GetNumDenom(c, out var cn, out var cd);
            GetNumDenom(d, out var dn, out var dd);
            static BigInteger Lcm(BigInteger x, BigInteger y) =>
                x == y ? x : x / BigInteger.GreatestCommonDivisor(x, y) * y;
            var common = Lcm(Lcm(ad, bd), Lcm(cd, dd));
            var origin = dn * (common / dd);
            ax = an * (common / ad) - origin;
            bx = bn * (common / bd) - origin;
            cx = cn * (common / cd) - origin;
        }

        /// <summary>
        /// True when [seg0,seg1] and [t0,t1,t2] are separated on one axis (1D AABB disjoint).
        /// Uses long arithmetic when all five values are on the long fast path.
        /// </summary>
        public static bool AxisSeparated(
            in BigRationalHybrid seg0, in BigRationalHybrid seg1,
            in BigRationalHybrid t0, in BigRationalHybrid t1, in BigRationalHybrid t2)
        {
            if (seg0.isLongValue && seg1.isLongValue &&
                t0.isLongValue && t1.isLongValue && t2.isLongValue)
            {
                long segMin = seg0.value <= seg1.value ? seg0.value : seg1.value;
                long segMax = seg0.value >= seg1.value ? seg0.value : seg1.value;
                long triMin = t0.value;
                if (t1.value < triMin) triMin = t1.value;
                if (t2.value < triMin) triMin = t2.value;
                long triMax = t0.value;
                if (t1.value > triMax) triMax = t1.value;
                if (t2.value > triMax) triMax = t2.value;
                return segMax < triMin || segMin > triMax;
            }

            var segMinR = Min(seg0, seg1);
            var segMaxR = Max(seg0, seg1);
            var triMinR = Min(Min(t0, t1), t2);
            var triMaxR = Max(Max(t0, t1), t2);
            return segMaxR.CompareTo(triMinR) < 0 || segMinR.CompareTo(triMaxR) > 0;
        }

        /// <summary>
        /// Exact sign of px*nx + py*ny + pz*nz + offset (signed distance to plane ax+by+cz+d=0 in hybrid form).
        /// Otherwise computes the full sum once; if <paramref name="rationalSumWasReturned"/> is true,
        /// <paramref name="rationalSum"/> equals that value (avoids recomputation e.g. in <c>Trim</c>).
        /// </summary>
        public static int SignOfPlaneDistance(
            in BigRationalHybrid px, in BigRationalHybrid nx,
            in BigRationalHybrid py, in BigRationalHybrid ny,
            in BigRationalHybrid pz, in BigRationalHybrid nz,
            in BigRationalHybrid offset,
            out bool rationalSumWasReturned,
            out BigRationalHybrid rationalSum)
        {
            rationalSum = px * nx + py * ny + pz * nz + offset;
            rationalSumWasReturned = true;
            return rationalSum.Sign();
        }

        /// <summary>Exact sign of ax*bx + ay*by + az*bz (same as <see cref="Rat3Hybrid.Dot"/> then <see cref="Sign"/>).</summary>
        public static int SignOfDot3(
            in BigRationalHybrid ax, in BigRationalHybrid bx,
            in BigRationalHybrid ay, in BigRationalHybrid by,
            in BigRationalHybrid az, in BigRationalHybrid bz)
        {
            // Three arbitrary Int64 products can overflow Int128 when summed.
            // Int32 products are bounded; wider values use BigInteger below.
            if (ax.IsInt32 && bx.IsInt32 && ay.IsInt32 && by.IsInt32 && az.IsInt32 && bz.IsInt32)
            {
                Int128 s = (Int128)ax.value * bx.value;
                s += (Int128)ay.value * by.value;
                s += (Int128)az.value * bz.value;
                if (s > 0) return 1;
                if (s < 0) return -1;
                return 0;
            }

            BigRationalHybrid sum = ax * bx + ay * by + az * bz;
            return sum.Sign();
        }

        private static void GetNumDenom(in BigRationalHybrid x, out BigInteger n, out BigInteger d)
        {
            if (x.isLongValue)
            {
                n = x.value;
                d = BigInteger.One;
            }
            else
            {
                n = x.rationalValue.Numerator();
                d = x.rationalValue.Denominator();
            }
        }

        private Rat GetRationalValue()
        {
            // Lazy initialization: create rationalValue if it doesn't exist yet
            if (rationalValue == null)
                rationalValue = new BigRationalClass((Rat)value);
            return rationalValue.Value;
        }


        //private static bool WillOverflowAdd(int a, int b)
        //{
        //    long aa = a;
        //    long bb = b;

        //    long result = aa + bb;

        //    return result > int.MaxValue || result < int.MinValue;

        //    //if (a > 0 && b > int.MaxValue - a) return true; // Positive overflow
        //    //if (a < 0 && b < int.MinValue - a) return true; // Negative overflow
        //    //return false;
        //}
        public static bool TryAdd(long a, long b, out long result)
        {
            result = a + b;
            return ((a ^ result) & (b ^ result)) >= 0;
        }

        public static bool TrySubtract(long a, long b, out long result)
        {
            result = a - b;
            return ((a ^ b) & (a ^ result)) >= 0;
        }


        public static BigRationalHybrid operator +(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            if (!lhs.isLongValue || !rhs.isLongValue || !TryAdd(lhs.value, rhs.value, out var result))
            {
                return new BigRationalHybrid(lhs.GetRationalValue() + rhs.GetRationalValue());
            }            
            return new BigRationalHybrid(result);
        }
        public static BigRationalHybrid operator -(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            if (!lhs.isLongValue || !rhs.isLongValue || !TrySubtract(lhs.value, rhs.value, out var result))
            {
                return new BigRationalHybrid(lhs.GetRationalValue() - rhs.GetRationalValue());
            }
            return new BigRationalHybrid(result);
        }
        public static BigRationalHybrid operator -(in BigRationalHybrid rhs)
        {
            if (!rhs.isLongValue || !TrySubtract(0, rhs.value, out var result))
            {
                return new BigRationalHybrid(-rhs.GetRationalValue());
            }
            return new BigRationalHybrid(result);
        }

        //private static bool WillOverflowMul(int a, int b)
        //{
        //    long aa = a;
        //    long bb = b;

        //    long result = aa * bb;

        //    return result > int.MaxValue || result < int.MinValue;
        //}
        public static bool TryMultiply(long a, long b, out long result)
        {
            if ((int)a == a && (int)b == b)
            {
                result = a * b;
                return true;
            }

            long high = Math.BigMul(a, b, out result);
            return high == result >> 63;
        }

        public static BigRationalHybrid operator *(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            if (!lhs.isLongValue || !rhs.isLongValue || !TryMultiply(lhs.value, rhs.value, out var result))
            {
                return new BigRationalHybrid(lhs.GetRationalValue() * rhs.GetRationalValue());
            }
            return new BigRationalHybrid(result);
        }

        private static bool TryDivide(long a, long b, out long result)
        {
            if (b == 0 || (a == long.MinValue && b == -1))
            {
                result = 0;
                return false;
            }

            result = Math.DivRem(a, b, out long remainder);
            return remainder == 0;
        }

        public static BigRationalHybrid operator /(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            if (!lhs.isLongValue || !rhs.isLongValue || !TryDivide(lhs.value, rhs.value, out var result))
            {
                return new BigRationalHybrid(lhs.GetRationalValue() / rhs.GetRationalValue());
            }
            return new BigRationalHybrid(result);
        }

        public static bool operator ==(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            if (lhs.isLongValue && rhs.isLongValue)
                return lhs.value == rhs.value;
            return lhs.GetRationalValue() == rhs.GetRationalValue();
        }

        public static bool operator !=(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            return !(lhs == rhs);
        }

        public static bool operator >(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            if (lhs.isLongValue && rhs.isLongValue)
                return lhs.value > rhs.value;
            return lhs.GetRationalValue() > rhs.GetRationalValue();
        }

        public static bool operator <(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            if (lhs.isLongValue && rhs.isLongValue)
                return lhs.value < rhs.value;
            return lhs.GetRationalValue() < rhs.GetRationalValue();
        }

        public static bool operator >=(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            if (lhs.isLongValue && rhs.isLongValue)
                return lhs.value >= rhs.value;
            return lhs.GetRationalValue() >= rhs.GetRationalValue();
        }

        public static bool operator <=(in BigRationalHybrid lhs, in BigRationalHybrid rhs)
        {
            if (lhs.isLongValue && rhs.isLongValue)
                return lhs.value <= rhs.value;
            return lhs.GetRationalValue() <= rhs.GetRationalValue();
        }

        public override string ToString()
        {
            string s = ToDouble().ToString();
            if (!isLongValue)
            {
                s += " (" + rationalValue.Numerator().ToString() + " / " + rationalValue.Denominator().ToString() + ")";
            }

            if (isSimplified)
                s += "s";

            return s;
        }

        public string DebugString
        {
            get
            {
                if (!isLongValue)
                {
                    return rationalValue.Numerator().ToString() + " / " + rationalValue.Denominator().ToString();
                }

                return value.ToString();
            }
        }

        public override int GetHashCode()
        {
#if DEBUG
            if (!isSimplified)
                throw new Exception("Hash code is not necessarily unique for non simplified values");
#endif
            int result = 0;
            if (!isLongValue)
                result = rationalValue.Value.GetHashCode();
            else
                result = value.GetHashCode();

            //Console.WriteLine("Hash: " + result);
            return result;
        }

        public bool Equals(BigRationalHybrid other)
        {
#if DEBUG
            if (!isSimplified)
                throw new Exception("Hash code is not necessarily unique for non simplified values");
            if (!other.isSimplified)
                throw new Exception("Hash code is not necessarily unique for non simplified values");
#endif
            if (isLongValue != other.isLongValue)
                return false;
            if (!isLongValue)
                return rationalValue.Value == other.rationalValue.Value;

            return value == other.value;
        }

        public override bool Equals(object obj)
        {
            return obj is BigRationalHybrid other && Equals(other);
        }
    }
}
