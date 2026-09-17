using System.Numerics;

namespace GeoCore
{
    public static class Rat3HybridExtensions
    {
        public static Int3 ToInt(this Rat3Hybrid p)
        {
            return new Int3(p.X.ToIntRoundDown(), p.Y.ToIntRoundDown(), p.Z.ToIntRoundDown());
        }

        public static Box3I GetBox(this Rat3Hybrid p)
        {
            GetAxisBounds(p.X, out int lx, out int ux);
            GetAxisBounds(p.Y, out int ly, out int uy);
            GetAxisBounds(p.Z, out int lz, out int uz);
            return new Box3I(new Int3(lx, ly, lz), new Int3(ux, uy, uz));
        }

        private static void GetAxisBounds(in BigRationalHybrid coordinate, out int lower, out int upper)
        {
            int truncated;
            bool negative;
            bool fractional;
            if (coordinate.IsInt32)
            {
                truncated = coordinate.ToIntRoundDown();
                negative = truncated < 0;
                fractional = false;
            }
            else
            {
                // Exact division avoids constructing Abs(coordinate) and
                // comparing a rounded integer with the full rational again.
                var numerator = coordinate.Numerator();
                var quotient = BigInteger.DivRem(numerator, coordinate.Denominator(), out var remainder);
                truncated = (int)quotient; // Checked by BigInteger conversion.
                negative = numerator.Sign < 0;
                fractional = !remainder.IsZero;
            }

            if (negative)
            {
                upper = truncated;
                if (truncated == int.MinValue)
                {
                    if (fractional) throw new OverflowException("Coordinate is outside the Int32 bounding-box domain.");
                    lower = int.MinValue;
                }
                else lower = truncated - 1;
            }
            else
            {
                lower = truncated;
                if (truncated == int.MaxValue)
                {
                    if (fractional) throw new OverflowException("Coordinate is outside the Int32 bounding-box domain.");
                    upper = int.MaxValue;
                }
                else upper = truncated + 1;
            }
            // Preserve the previous conservative padding at exact integers,
            // except at the domain endpoints where padding must not wrap.
        }
    }
}
