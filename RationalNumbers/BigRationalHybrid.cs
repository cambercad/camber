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

            Int128 adx, ady, bdx, bdy, cdx, cdy;
            if (pax.IsInt32 && pay.IsInt32 && pbx.IsInt32 && pby.IsInt32 &&
                pcx.IsInt32 && pcy.IsInt32 && pdx.IsInt32 && pdy.IsInt32)
            {
                adx = (int)pax.value - (int)pdx.value;
                ady = (int)pay.value - (int)pdy.value;
                bdx = (int)pbx.value - (int)pdx.value;
                bdy = (int)pby.value - (int)pdy.value;
                cdx = (int)pcx.value - (int)pdx.value;
                cdy = (int)pcy.value - (int)pdy.value;
            }
            else
            {
                adx = (Int128)pax.value - pdx.value;
                ady = (Int128)pay.value - pdy.value;
                bdx = (Int128)pbx.value - pdx.value;
                bdy = (Int128)pby.value - pdy.value;
                cdx = (Int128)pcx.value - pdx.value;
                cdy = (Int128)pcy.value - pdy.value;
            }

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

        /// <summary>
        /// Exact sign of the 3×3 determinant with columns (pa−pd, pb−pd, pc−pd) — same as CSG Orient3DSign.
        /// Uses <see cref="Int128"/> when all twelve coordinates are on the long fast path.
        /// </summary>
        public static long Orient3DFastCount;
        public static long Orient3DSlowCount;
        public static long Orient3DFilterHitCount;
        public static long Orient3DFilterMissCount;

        /// <summary>
        /// Machine epsilon (2^-52). Shewchuk o3derrboundA is about 7ε; conversion of
        /// rationals to double adds more, so the filter uses a larger multiple and
        /// abstains whenever |det| is not safely above the bound.
        /// </summary>
        private const double Orient3DDoubleEps = 2.2204460492503131e-16;

        public static int SignOfOrient3D(
            in BigRationalHybrid pax, in BigRationalHybrid pay, in BigRationalHybrid paz,
            in BigRationalHybrid pbx, in BigRationalHybrid pby, in BigRationalHybrid pbz,
            in BigRationalHybrid pcx, in BigRationalHybrid pcy, in BigRationalHybrid pcz,
            in BigRationalHybrid pdx, in BigRationalHybrid pdy, in BigRationalHybrid pdz)
        {
            if (pax.isLongValue && pay.isLongValue && paz.isLongValue &&
                pbx.isLongValue && pby.isLongValue && pbz.isLongValue &&
                pcx.isLongValue && pcy.isLongValue && pcz.isLongValue &&
                pdx.isLongValue && pdy.isLongValue && pdz.isLongValue)
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

            int filtered;
            if (TrySignOfOrient3DDoubleFilter(
                    in pax, in pay, in paz, in pbx, in pby, in pbz,
                    in pcx, in pcy, in pcz, in pdx, in pdy, in pdz, out filtered))
            {
                return filtered;
            }

            return SignOfOrient3DExact(
                in pax, in pay, in paz, in pbx, in pby, in pbz,
                in pcx, in pcy, in pcz, in pdx, in pdy, in pdz);
        }

        /// <summary>
        /// Double filter for <see cref="SignOfOrient3D"/>. Returns false when the
        /// sign is not certified (caller must use the exact path). Never returns a
        /// sign that disagrees with the exact determinant.
        /// </summary>
        public static bool TrySignOfOrient3DDoubleFilter(
            in BigRationalHybrid pax, in BigRationalHybrid pay, in BigRationalHybrid paz,
            in BigRationalHybrid pbx, in BigRationalHybrid pby, in BigRationalHybrid pbz,
            in BigRationalHybrid pcx, in BigRationalHybrid pcy, in BigRationalHybrid pcz,
            in BigRationalHybrid pdx, in BigRationalHybrid pdy, in BigRationalHybrid pdz,
            out int sign)
        {
            sign = 0;
            double ax = pax.ToDouble(), ay = pay.ToDouble(), az = paz.ToDouble();
            double bx = pbx.ToDouble(), by = pby.ToDouble(), bz = pbz.ToDouble();
            double cx = pcx.ToDouble(), cy = pcy.ToDouble(), cz = pcz.ToDouble();
            double dx = pdx.ToDouble(), dy = pdy.ToDouble(), dz = pdz.ToDouble();
            if (!double.IsFinite(ax) || !double.IsFinite(ay) || !double.IsFinite(az) ||
                !double.IsFinite(bx) || !double.IsFinite(by) || !double.IsFinite(bz) ||
                !double.IsFinite(cx) || !double.IsFinite(cy) || !double.IsFinite(cz) ||
                !double.IsFinite(dx) || !double.IsFinite(dy) || !double.IsFinite(dz))
                return false;

            double adx = ax - dx, bdx = bx - dx, cdx = cx - dx;
            double ady = ay - dy, bdy = by - dy, cdy = cy - dy;
            double adz = az - dz, bdz = bz - dz, cdz = cz - dz;

            double bdxcdy = bdx * cdy;
            double cdxbdy = cdx * bdy;
            double cdxady = cdx * ady;
            double adxcdy = adx * cdy;
            double bdxady = bdx * ady;
            double adxbdy = adx * bdy;

            double left = adz * (bdxcdy - cdxbdy) + bdz * (cdxady - adxcdy);
            double right = cdz * (bdxady - adxbdy);
            double det = left - right;
            if (!double.IsFinite(det))
                return false;

            // Same grouping as the exact Int128 path; absolute values bound rounding.
            double permanent =
                Math.Abs(adz) * (Math.Abs(bdxcdy) + Math.Abs(cdxbdy)) +
                Math.Abs(bdz) * (Math.Abs(cdxady) + Math.Abs(adxcdy)) +
                Math.Abs(cdz) * (Math.Abs(bdxady) + Math.Abs(adxbdy));
            if (!double.IsFinite(permanent))
                return false;

            double maxAbs = 0.0;
            if (Math.Abs(ax) > maxAbs) maxAbs = Math.Abs(ax);
            if (Math.Abs(ay) > maxAbs) maxAbs = Math.Abs(ay);
            if (Math.Abs(az) > maxAbs) maxAbs = Math.Abs(az);
            if (Math.Abs(bx) > maxAbs) maxAbs = Math.Abs(bx);
            if (Math.Abs(by) > maxAbs) maxAbs = Math.Abs(by);
            if (Math.Abs(bz) > maxAbs) maxAbs = Math.Abs(bz);
            if (Math.Abs(cx) > maxAbs) maxAbs = Math.Abs(cx);
            if (Math.Abs(cy) > maxAbs) maxAbs = Math.Abs(cy);
            if (Math.Abs(cz) > maxAbs) maxAbs = Math.Abs(cz);
            if (Math.Abs(dx) > maxAbs) maxAbs = Math.Abs(dx);
            if (Math.Abs(dy) > maxAbs) maxAbs = Math.Abs(dy);
            if (Math.Abs(dz) > maxAbs) maxAbs = Math.Abs(dz);
            double scaleTerm = maxAbs * maxAbs * maxAbs;

            // Shewchuk's ~7ε assumes exact double inputs. Converting rationals adds
            // another O(ε M³) term; a 32ε bound mis-certified coincident cylinder
            // meridians (ValidateCluster). Stay conservative: 256ε plus 1e-12.
            double errbound =
                (256.0 * Orient3DDoubleEps) * (permanent + scaleTerm) +
                1e-12 * (permanent + 1.0) +
                (256.0 * double.Epsilon);
            if (Math.Abs(det) <= errbound)
                return false;

            sign = det > 0.0 ? 1 : -1;
            return true;
        }

        /// <summary>Exact BigInteger determinant sign (no float filter, no Int128 fast path).</summary>
        public static int SignOfOrient3DExact(
            in BigRationalHybrid pax, in BigRationalHybrid pay, in BigRationalHybrid paz,
            in BigRationalHybrid pbx, in BigRationalHybrid pby, in BigRationalHybrid pbz,
            in BigRationalHybrid pcx, in BigRationalHybrid pcy, in BigRationalHybrid pcz,
            in BigRationalHybrid pdx, in BigRationalHybrid pdy, in BigRationalHybrid pdz)
        {
            RationalDiff(in pax, in pdx, out BigInteger adxN, out BigInteger adxD);
            RationalDiff(in pbx, in pdx, out BigInteger bdxN, out BigInteger bdxD);
            RationalDiff(in pcx, in pdx, out BigInteger cdxN, out BigInteger cdxD);
            RationalDiff(in pay, in pdy, out BigInteger adyN, out BigInteger adyD);
            RationalDiff(in pby, in pdy, out BigInteger bdyN, out BigInteger bdyD);
            RationalDiff(in pcy, in pdy, out BigInteger cdyN, out BigInteger cdyD);
            RationalDiff(in paz, in pdz, out BigInteger adzN, out BigInteger adzD);
            RationalDiff(in pbz, in pdz, out BigInteger bdzN, out BigInteger bdzD);
            RationalDiff(in pcz, in pdz, out BigInteger cdzN, out BigInteger cdzD);

            RationalMinorDiff(bdxN, bdxD, cdyN, cdyD, cdxN, cdxD, bdyN, bdyD, out BigInteger minor1N, out BigInteger minor1D);
            RationalMinorDiff(cdxN, cdxD, adyN, adyD, adxN, adxD, cdyN, cdyD, out BigInteger minor2N, out BigInteger minor2D);
            RationalMinorDiff(bdxN, bdxD, adyN, adyD, adxN, adxD, bdyN, bdyD, out BigInteger minor3N, out BigInteger minor3D);

            RationalProduct(adzN, adzD, minor1N, minor1D, out BigInteger term1N, out BigInteger term1D);
            RationalProduct(bdzN, bdzD, minor2N, minor2D, out BigInteger term2N, out BigInteger term2D);
            RationalProduct(cdzN, cdzD, minor3N, minor3D, out BigInteger rightN, out BigInteger rightD);

            RationalAdd(term1N, term1D, term2N, term2D, out BigInteger leftN, out BigInteger leftD);
            return CompareRational(leftN, leftD, rightN, rightD);
        }

        private static void RationalDiff(in BigRationalHybrid a, in BigRationalHybrid b, out BigInteger n, out BigInteger d)
        {
            GetNumDenom(in a, out BigInteger na, out BigInteger da);
            GetNumDenom(in b, out BigInteger nb, out BigInteger db);
            n = na * db - nb * da;
            d = da * db;
        }

        private static void RationalProduct(
            BigInteger n1, BigInteger d1, BigInteger n2, BigInteger d2,
            out BigInteger n, out BigInteger d)
        {
            n = n1 * n2;
            d = d1 * d2;
        }

        private static void RationalMinorDiff(
            BigInteger nBdx, BigInteger dBdx, BigInteger nCdy, BigInteger dCdy,
            BigInteger nCdx, BigInteger dCdx, BigInteger nBdy, BigInteger dBdy,
            out BigInteger n, out BigInteger d)
        {
            BigInteger p1N = nBdx * nCdy;
            BigInteger p1D = dBdx * dCdy;
            BigInteger p2N = nCdx * nBdy;
            BigInteger p2D = dCdx * dBdy;
            n = p1N * p2D - p2N * p1D;
            d = p1D * p2D;
        }

        private static void RationalAdd(
            BigInteger n1, BigInteger d1, BigInteger n2, BigInteger d2,
            out BigInteger n, out BigInteger d)
        {
            n = n1 * d2 + n2 * d1;
            d = d1 * d2;
        }

        private static int CompareRational(BigInteger n1, BigInteger d1, BigInteger n2, BigInteger d2)
        {
            return BigInteger.Compare(n1 * d2, n2 * d1);
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
            if (ax.isLongValue && bx.isLongValue && ay.isLongValue && by.isLongValue && az.isLongValue && bz.isLongValue)
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
