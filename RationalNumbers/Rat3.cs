using System.Numerics;

namespace GeoCore
{
    public struct Rat3
    {
        public static readonly Rat3 Zero = new Rat3(BigRational.Zero, BigRational.Zero, BigRational.Zero);

        public BigRational X;
        public BigRational Y;
        public BigRational Z;

        public Rat3(BigRational x, BigRational y, BigRational z)
        {
            X = x;
            Y = y;
            Z = z;
        }
        public Rat3(int x, int y, int z)
        {
            X = new BigRational(x);
            Y = new BigRational(y);
            Z = new BigRational(z);
        }
        public Rat3(BigInteger x, BigInteger y, BigInteger z)
        {
            X = new BigRational(x);
            Y = new BigRational(y);
            Z = new BigRational(z);
        }

        public void Simplify()
        {
            X.Simplify();
            Y.Simplify();
            Z.Simplify();
        }

        public static Rat3 operator /(Rat3 lhs, BigInteger rhs)
        {
            return new Rat3(lhs.X / rhs, lhs.Y / rhs, lhs.Z / rhs);
        }
        public static Rat3 operator +(Rat3 lhs, Rat3 rhs)
        {
            return new Rat3(lhs.X + rhs.X, lhs.Y + rhs.Y, lhs.Z + rhs.Z);
        }
        public static Rat3 operator -(Rat3 lhs, Rat3 rhs)
        {
            return new Rat3(lhs.X - rhs.X, lhs.Y - rhs.Y, lhs.Z - rhs.Z);
        }
        public static Rat3 operator *(BigRational lhs, Rat3 rhs)
        {
            return new Rat3(lhs * rhs.X, lhs * rhs.Y, lhs * rhs.Z);
        }
        public static Rat3 operator /(Rat3 lhs, BigRational rhs)
        {
            return new Rat3(lhs.X / rhs, lhs.Y / rhs, lhs.Z / rhs);
        }

        public static bool operator ==(Rat3 lhs, Rat3 rhs)
        {
            var diff = lhs - rhs;
            return diff.X.Sign == 0 && diff.Y.Sign == 0 && diff.Z.Sign == 0;
        }
        public static bool operator !=(Rat3 lhs, Rat3 rhs)
        {
            return !(lhs == rhs);
        }

        //public LongXYZ ToLong()
        //{
        //    return new LongXYZ(X.ToLong(), Y.ToLong(), Z.ToLong());
        //}

        //public Int3 ToInt()
        //{
        //    return new Int3(X.ToInt(), Y.ToInt(), Z.ToInt());
        //}

        public BigRational this[int index]
        {
            get
            {
                switch(index)
                {
                    case 0:
                        return X;
                    case 1:
                        return Y;
                    case 2:
                        return Z;
                }
                return BigRational.Zero;
            }
        }

        //public Box3I GetBox()
        //{
        //    var p = this;
        //    p.X = Rat.Abs(p.X);
        //    p.Y = Rat.Abs(p.Y);
        //    p.Z = Rat.Abs(p.Z);

        //    var lower = p.ToInt();
        //    var upper = new Int3(lower.X + 1, lower.Y + 1, lower.Z + 1);

        //    if (p.X.Sign < 0)
        //    {
        //        lower.X = -lower.X;
        //        upper.X = -upper.X;
        //        Algorithms.Swap(ref lower.X, ref upper.X);
        //    }
        //    if (p.Y.Sign < 0)
        //    {
        //        lower.Y = -lower.Y;
        //        upper.Y = -upper.Y;
        //        Algorithms.Swap(ref lower.Y, ref upper.Y);
        //    }
        //    if (p.Z.Sign < 0)
        //    {
        //        lower.Z = -lower.Z;
        //        upper.Z = -upper.Z;
        //        Algorithms.Swap(ref lower.Z, ref upper.Z);
        //    }

        //    return new Box3I(lower, upper);
        //}

        public static Rat3 Cross(in Rat3 left, in Rat3 right)
        {
            return new Rat3(left.Y * right.Z - left.Z * right.Y,
                left.Z * right.X - left.X * right.Z,
                left.X * right.Y - left.Y * right.X);
        }

        public static BigRational Dot(in Rat3 left, in Rat3 right)
        {
            return left.X * right.X + left.Y * right.Y + left.Z * right.Z;
        }

        public override int GetHashCode()
        {
            int hash = X.GetHashCode();
            hash ^= Y.GetHashCode() + (hash << 6) + (hash >> 2);
            hash ^= Z.GetHashCode() + (hash << 6) + (hash >> 2);
            return hash;
        }        

        public override bool Equals(object obj)
        {
            Rat3 r = (Rat3)obj;
            return X.Equals(r.X) && Y.Equals(r.Y) && Z.Equals(r.Z);
        }
    }
}
