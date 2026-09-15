namespace GeoCore
{
    public struct Rat2Hybrid
    {
        public BigRationalHybrid X;
        public BigRationalHybrid Y;

        public Rat2Hybrid(BigRationalHybrid x, BigRationalHybrid y)
        {
            X = x;
            Y = y;
        }
        public Rat2Hybrid(int x, int y)
        {
            X = new BigRationalHybrid(x);
            Y = new BigRationalHybrid(y);
        }
        public Rat2Hybrid(long x, long y)
        {
            X = new BigRationalHybrid(x);
            Y = new BigRationalHybrid(y);
        }

        // Copy constructor
        public Rat2Hybrid(in Rat2Hybrid source)
        {
            X = new BigRationalHybrid(source.X);
            Y = new BigRationalHybrid(source.Y);
        }

        public void Simplify()
        {
            X.Simplify();
            Y.Simplify();
        }


        public override int GetHashCode()
        {
            return HashCode.Combine(X.GetHashCode(), Y.GetHashCode());
        }

        public override bool Equals(object obj)
        {
            Rat2Hybrid other = (Rat2Hybrid)obj;
            return X.Equals(other.X) && Y.Equals(other.Y);
        }

        //public static int Orient2D(in Rat2 pa, in Rat2 pb, in Rat2 pc)
        //{
        //    BigRational acx = pa.X - pc.X;
        //    BigRational bcx = pb.X - pc.X;
        //    BigRational acy = pa.Y - pc.Y;
        //    BigRational bcy = pb.Y - pc.Y;
        //    return (acx * bcy - acy * bcx).Sign;
        //}

        public static Rat2Hybrid operator +(in Rat2Hybrid lhs, in Rat2Hybrid rhs)
        {
            return new Rat2Hybrid(lhs.X + rhs.X, lhs.Y + rhs.Y);
        }
        public static Rat2Hybrid operator -(in Rat2Hybrid lhs, in Rat2Hybrid rhs)
        {
            return new Rat2Hybrid(lhs.X - rhs.X, lhs.Y - rhs.Y);
        }

        public static BigRationalHybrid Dot(in Rat2Hybrid lhs, in Rat2Hybrid rhs)
        {
            return lhs.X * rhs.X + lhs.Y * rhs.Y;
        }

        /// <summary>Scalar z-component of u×v (signed parallelogram area).</summary>
        public static BigRationalHybrid Cross(in Rat2Hybrid u, in Rat2Hybrid v)
        {
            return u.X * v.Y - u.Y * v.X;
        }

        /// <summary>Exact <see cref="BigRationalHybrid.Sign"/> of <see cref="Cross"/>; faster when only the sign is needed.</summary>
        public static int CrossSign(in Rat2Hybrid u, in Rat2Hybrid v) =>
            BigRationalHybrid.SignOfCrossProduct(in u.X, in v.Y, in u.Y, in v.X);

        /// <summary>Signed 2×2 determinant (b−a)×(c−a); same as <see cref="Cross"/>((b−a), (c−a)).</summary>
        public static BigRationalHybrid Orient2D(in Rat2Hybrid a, in Rat2Hybrid b, in Rat2Hybrid c)
        {
            return Cross(b - a, c - a);
        }

        /// <summary>Exact sign of <see cref="Orient2D"/>; faster when only the sign is needed.</summary>
        public static int Orient2DSign(in Rat2Hybrid a, in Rat2Hybrid b, in Rat2Hybrid c)
        {
            var acx = a.X - c.X;
            var bcx = b.X - c.X;
            var acy = a.Y - c.Y;
            var bcy = b.Y - c.Y;
            return BigRationalHybrid.SignOfCrossProduct(in acx, in bcy, in acy, in bcx);
        }

        public BigRationalHybrid this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0:
                        return X;
                    case 1:
                        return Y;
                }
                return BigRationalHybrid.Zero;
            }
            set
            {
                switch (index)
                {
                    case 0:
                        X = value;
                        break;
                    case 1:
                        Y = value;
                        break;
                }
            }
        }

        public override string ToString()
        {
            return X.ToString() + " " + Y.ToString();
        }
    }
}
