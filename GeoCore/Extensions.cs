using System.Runtime.CompilerServices;

namespace GeoCore
{   
    public static class Extensions
    {
        public static bool ContainsAll(this ref Tri t, int a, int b, int c)
        {
            return t.Contains(a) && t.Contains(b) && t.Contains(c);
        }
        public static void Replace(this ref Tri t, int oldVal, int newVal)
        {
            if (t.A == oldVal)
                t.A = newVal;
            if (t.B == oldVal)
                t.B = newVal;
            if (t.C == oldVal)
                t.C = newVal;
            throw new Exception();
        }

        public static int IndexOf(this Tri t, int vertexIndex)
        {
            if (t.A == vertexIndex)
                return 0;
            if (t.B == vertexIndex)
                return 1;
            if (t.C == vertexIndex)
                return 2;

            return -1;
        }

        public static int GetRemaining(this Tri t, int first, int second)
        {
            if ((t.A == first && t.B == second) || (t.A == second && t.B == first))
                return t.C;
            if ((t.B == first && t.C == second) || (t.B == second && t.C == first))
                return t.A;
            if ((t.C == first && t.A == second) || (t.C == second && t.A == first))
                return t.B;
            throw new Exception();
        }

        public static double ToRadians(this double angleDegree)
        {
            return angleDegree * (Math.PI / 180.0);
        }

        public static bool ContainsDuplicateIndex(this Tri t)
        {
            return t.A == t.B || t.A == t.C || t.B == t.C;
        }

        public static bool ContainsAll(this Tri t, int a, int b)
        {
            return t.Contains(a) && t.Contains(b);
        }
        public static bool Contains(this Tri t, int a)
        {
            return t.A == a || t.B == a || t.C == a;
        }

        public static bool Contains(this Int2 c, int value)
        {
            return c.X == value || c.Y == value;
        }

        public unsafe static ref int Get(this Tri t, int cornerId)
        {
            var ptr = (int*)Unsafe.AsPointer(ref t.A);
            return ref ptr[cornerId];
        }

        public unsafe static ref double Get(this Vec2D t, int cornerId)
        {
            var ptr = (double*)Unsafe.AsPointer(ref t.X);
            return ref ptr[cornerId];
        }

        public unsafe static ref int Get(this Int2 t, int cornerId)
        {
            var ptr = (int*)Unsafe.AsPointer(ref t.X);
            return ref ptr[cornerId];
        }

        public static void MinMax(this IList<Vec2D> list, out Vec2D min, out Vec2D max)
        {
            min = new Vec2D(double.MaxValue, double.MaxValue);
            max = new Vec2D(double.MinValue, double.MinValue);
            for (int i = 0; i < list.Count; i++)
            {
                var v = list[i];
                if (v.X < min.X) min.X = v.X;
                if (v.Y < min.Y) min.Y = v.Y;
                if (v.X > max.X) max.X = v.X;
                if (v.Y > max.Y) max.Y = v.Y;
            }
        }

        public static void MinMax(this IList<Vec3D> list, out Vec3D min, out Vec3D max)
        {
            min = new Vec3D(double.MaxValue, double.MaxValue, double.MaxValue);
            max = new Vec3D(double.MinValue, double.MinValue, double.MinValue);
            for (int i = 0; i < list.Count; i++)
            {
                var v = list[i];
                if (v.X < min.X) min.X = v.X;
                if (v.Y < min.Y) min.Y = v.Y;
                if (v.Z < min.Z) min.Z = v.Z;
                if (v.X > max.X) max.X = v.X;
                if (v.Y > max.Y) max.Y = v.Y;
                if (v.Z > max.Z) max.Z = v.Z;
            }
        }
    }
}
