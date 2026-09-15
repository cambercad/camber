namespace GeoCore
{
    public static class Algorithms
    {
        public static void Swap<T>(ref T a, ref T b)
        {
            T temp = a;
            a = b;
            b = temp;
        }

        public static long Key(int a, int b)
        {
            if (a < b)
                return (((long)a) << 32) | ((long)b);
            else
                return (((long)b) << 32) | ((long)a);
        }

        public static void DecomposeKey(long k, out int a, out int b)
        {
            a = (int)(k >> 32);
            b = (int)(k & 0x00000000FFFFFFFF);
        }

        public static double Clamp(double d, double lower, double upper)
        {
            if (d < lower) return lower;
            if (d > upper) return upper;
            return d;
        }

        //Returns the number of solutions
        public static int SolveQuadraticEquation(double a, double b, double c, out double alpha1, out double alpha2)
        {
            alpha1 = double.NaN;
            alpha2 = double.NaN;

            if (a == 0)
            {
                //Solve linear equation
                if (b == 0)
                {
                    return 0;
                }
                alpha1 = -c / b;
                return 1;
            }

            double d = b * b - 4.0 * a * c;
            if (d < 0)
            {
                alpha1 = 0;
                alpha2 = 0;
                return 0;
            }
            d = Math.Sqrt(d);

            alpha1 = (-b - d) / (2.0 * a);
            alpha2 = (-b + d) / (2.0 * a);

            return d == 0.0 ? 1 : 2;
        }
    }
}
