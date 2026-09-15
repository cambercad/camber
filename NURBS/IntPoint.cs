#define use_xyz

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

using cInt = System.Int64;
namespace NURBS
{
    //------------------------------------------------------------------------------
    //------------------------------------------------------------------------------
    [StructLayout(LayoutKind.Explicit)]
    public unsafe struct IntPoint
    {
        [FieldOffset(0)]
        public cInt X;
        [FieldOffset(8)]
        public cInt Y;
#if use_xyz
        [FieldOffset(16)]
        public cInt Z;
        [FieldOffset(16)]
        public int LowerZ;
        [FieldOffset(20)]
        public int UpperZ;

        public IntPoint(cInt x, cInt y, cInt z = 0)
        {
            UpperZ = 0; LowerZ = 0;
            this.X = x; this.Y = y; this.Z = z;
        }

        public ref long Get(int index)
        {
            return ref ((cInt*)Unsafe.AsPointer<cInt>(ref X))[index];
        }

        public IntPoint(double x, double y, double z = 0)
        {
            UpperZ = 0; LowerZ = 0;
            this.X = (cInt)x; this.Y = (cInt)y; this.Z = (cInt)z;
        }

        /*public IntPoint(DoublePoint dp)
        {
            UpperZ = 0; LowerZ = 0;
            this.X = (cInt)dp.X; this.Y = (cInt)dp.Y; this.Z = 0;
        }*/

        public IntPoint(IntPoint pt)
        {
            UpperZ = 0; LowerZ = 0;
            this.X = pt.X; this.Y = pt.Y; this.Z = pt.Z;
        }
#else
        public IntPoint(cInt X, cInt Y)
        {
            this.X = X; this.Y = Y;
        }
        public IntPoint(double x, double y)
        {
            this.X = (cInt)x; this.Y = (cInt)y;
        }

        public IntPoint(IntPoint pt)
        {
            this.X = pt.X; this.Y = pt.Y;
        }
#endif

        public static bool operator ==(IntPoint a, IntPoint b)
        {
            return a.X == b.X && a.Y == b.Y;
        }

        public static bool operator !=(IntPoint a, IntPoint b)
        {
            return a.X != b.X || a.Y != b.Y;
        }

        public static IntPoint operator -(IntPoint a, IntPoint b)
        {
            a.X = a.X - b.X;
            a.Y = a.Y - b.Y;
            return a;
        }

        public static IntPoint operator +(IntPoint a, IntPoint b)
        {
            a.X = a.X + b.X;
            a.Y = a.Y + b.Y;
            return a;
        }

        public override bool Equals(object obj)
        {
            if (obj == null) return false;
            if (obj is IntPoint)
            {
                IntPoint a = (IntPoint)obj;
                return (X == a.X) && (Y == a.Y);
            }
            else return false;
        }

        public override int GetHashCode()
        {
            //simply prevents a compiler warning
            return base.GetHashCode();
        }

    }// end struct IntPoint
}
