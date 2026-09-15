namespace GeoCore
{
    public struct Rat2
    {
        public BigRational X;
        public BigRational Y;

        public Rat2(BigRational x, BigRational y)
        {
            X = x;
            Y = y;
        }
        public Rat2(int x, int y)
        {
            X = new BigRational(x);
            Y = new BigRational(y);
        }

        //public static int Orient2D(in Rat2 pa, in Rat2 pb, in Rat2 pc)
        //{
        //    BigRational acx = pa.X - pc.X;
        //    BigRational bcx = pb.X - pc.X;
        //    BigRational acy = pa.Y - pc.Y;
        //    BigRational bcy = pb.Y - pc.Y;
        //    return (acx * bcy - acy * bcx).Sign;
        //}

        public static Rat2 operator +(in Rat2 lhs, in Rat2 rhs)
        {
            return new Rat2(lhs.X + rhs.X, lhs.Y + rhs.Y);
        }
        public static Rat2 operator -(in Rat2 lhs, in Rat2 rhs)
        {
            return new Rat2(lhs.X - rhs.X, lhs.Y - rhs.Y);
        }

        public static BigRational Dot(in Rat2 lhs, in Rat2 rhs)
        {
            return lhs.X * rhs.X + lhs.Y * rhs.Y;
        }

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
                }
                return BigRational.Zero;
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
