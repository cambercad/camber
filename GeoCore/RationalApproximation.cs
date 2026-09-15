using System.Numerics;

namespace GeoCore
{
    /// <summary>
    /// Bounded-denominator rational approximation for bridging <see cref="double"/> geometry
    /// with <see cref="BigRationalHybrid"/> pipelines (e.g. mesh copy transforms).
    /// </summary>
    public static class RationalApproximation
    {
        /// <summary>
        /// Best rational approximation <c>p/q</c> to <paramref name="value"/> with <c>1 ≤ q ≤ maxDenominator</c>
        /// minimizing <c>|value − p/q|</c> (brute-force scan; intended for moderate <paramref name="maxDenominator"/>).
        /// </summary>
        public static BigRationalHybrid ApproximateDouble(double value, long maxDenominator)
        {
            if (maxDenominator < 1)
                throw new ArgumentOutOfRangeException(nameof(maxDenominator));
            if (double.IsNaN(value) || double.IsInfinity(value))
                throw new ArgumentException("Value must be finite.", nameof(value));

            long sign = 1;
            double x = value;
            if (x < 0)
            {
                sign = -1;
                x = -x;
            }

            long bestNum = 0;
            long bestDen = 1;
            double bestErr = x;

            for (long q = 1; q <= maxDenominator; q++)
            {
                long p = (long)Math.Round(x * q);
                double approx = p / (double)q;
                double err = Math.Abs(x - approx);
                if (err < bestErr)
                {
                    bestErr = err;
                    bestNum = p;
                    bestDen = q;
                }
            }

            if (sign < 0)
                bestNum = -bestNum;

            ReduceSigned(ref bestNum, ref bestDen);
            return new BigRationalHybrid(new BigInteger(bestNum), new BigInteger(bestDen));
        }

        private static void ReduceSigned(ref long num, ref long den)
        {
            if (den < 0)
            {
                num = -num;
                den = -den;
            }
            if (num == 0)
            {
                den = 1;
                return;
            }
            long g = (long)BigInteger.GreatestCommonDivisor(new BigInteger(num), new BigInteger(den));
            if (g > 1)
            {
                num /= g;
                den /= g;
            }
        }
    }
}
