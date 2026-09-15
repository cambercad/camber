namespace GeoSolver.Sparse
{
    internal static class Buf
    {
        public static void I(ref int[] a, int n)
        {
            if (a == null || a.Length < n)
                Array.Resize(ref a, GrowCap(a == null ? 0 : a.Length, n));
        }

        public static void D(ref double[] a, int n)
        {
            if (a == null || a.Length < n)
                Array.Resize(ref a, GrowCap(a == null ? 0 : a.Length, n));
        }

        public static void Fill(int[] a, int n, int value)
        {
            for (int i = 0; i < n; i++)
                a[i] = value;
        }

        public static void Zero(int[] a, int n)
        {
            Array.Clear(a, 0, n);
        }

        public static void Zero(double[] a, int n)
        {
            Array.Clear(a, 0, n);
        }

        public static int NextStamp(int[] mark, int n, int stamp)
        {
            stamp++;
            if (stamp != int.MaxValue)
                return stamp;
            Array.Clear(mark, 0, n);
            return 1;
        }

        public static int GrowCap(int current, int needed)
        {
            int cap = current < 16 ? 16 : current;
            while (cap < needed)
            {
                int next = cap + (cap >> 1);
                if (next < cap)
                    return needed;
                cap = next;
            }
            return cap;
        }

        /// <summary>Sort keys[lo..hi) and permute vals with them, then sum duplicate keys. Returns new hi.</summary>
        public static int SortSumUnique(int[] keys, double[] vals, int lo, int hi)
        {
            int n = hi - lo;
            if (n <= 1)
                return hi;
            Array.Sort(keys, vals, lo, n);
            int w = lo;
            for (int p = lo + 1; p < hi; p++)
            {
                if (keys[p] == keys[w])
                    vals[w] += vals[p];
                else
                {
                    w++;
                    keys[w] = keys[p];
                    vals[w] = vals[p];
                }
            }
            return w + 1;
        }

        public static int SortUnique(int[] keys, int lo, int hi)
        {
            int n = hi - lo;
            if (n <= 1)
                return hi;
            Array.Sort(keys, lo, n);
            int w = lo;
            for (int p = lo + 1; p < hi; p++)
            {
                if (keys[p] != keys[w])
                    keys[++w] = keys[p];
            }
            return w + 1;
        }
    }
}
