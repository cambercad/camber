namespace GeoCore
{
    public struct Rat3Hybrid : IEquatable<Rat3Hybrid>
    {
        public BigRationalHybrid X;
        public BigRationalHybrid Y;
        public BigRationalHybrid Z;

        public static readonly Rat3Hybrid Zero = new Rat3Hybrid(0, 0, 0);
        
        public Rat3Hybrid(int x, int y, int z)
        {
            X = new BigRationalHybrid(x);
            Y = new BigRationalHybrid(y);
            Z = new BigRationalHybrid(z);
        }

        // Copy constructor
        public Rat3Hybrid(in Rat3Hybrid source)
        {
            X = new BigRationalHybrid(source.X);
            Y = new BigRationalHybrid(source.Y);
            Z = new BigRationalHybrid(source.Z);
        }

        public Rat3Hybrid(BigRationalHybrid x, BigRationalHybrid y, BigRationalHybrid z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public void Simplify()
        {
            X.Simplify();
            Y.Simplify();
            Z.Simplify();
        }

        public bool IsZero()
        {
            return X.Sign() == 0 && Y.Sign() == 0 && Z.Sign() == 0;
        }

        public BigRationalHybrid LengthSquared()
        {
            return X * X + Y * Y + Z * Z;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(X.GetHashCode(), Y.GetHashCode(), Z.GetHashCode());
        }

        public bool Equals(Rat3Hybrid other)
        {
            return X.Equals(other.X) && Y.Equals(other.Y) && Z.Equals(other.Z);
        }

        public override bool Equals(object obj)
        {
            return obj is Rat3Hybrid other && Equals(other);
        }

        public static Rat3Hybrid operator +(in Rat3Hybrid lhs, in Rat3Hybrid rhs)
        {
            return new Rat3Hybrid(lhs.X + rhs.X, lhs.Y + rhs.Y, lhs.Z + rhs.Z);
        }
        public static Rat3Hybrid operator -(in Rat3Hybrid lhs, in Rat3Hybrid rhs)
        {
            return new Rat3Hybrid(lhs.X - rhs.X, lhs.Y - rhs.Y, lhs.Z - rhs.Z);
        }
        public static Rat3Hybrid operator -(in Rat3Hybrid rhs)
        {
            return new Rat3Hybrid( - rhs.X,  - rhs.Y,  - rhs.Z);
        }

        public static Rat3Hybrid operator *(in BigRationalHybrid lhs, in Rat3Hybrid rhs)
        {
            return new Rat3Hybrid(lhs* rhs.X, lhs* rhs.Y, lhs* rhs.Z);
        }
        public static Rat3Hybrid operator *(in Rat3Hybrid lhs, in BigRationalHybrid rhs)
        {
            return new Rat3Hybrid(lhs.X * rhs, lhs.Y * rhs, lhs.Z * rhs);
        }
        public static Rat3Hybrid operator /(in Rat3Hybrid lhs, in BigRationalHybrid rhs)
        {
            return new Rat3Hybrid(lhs.X / rhs, lhs.Y / rhs, lhs.Z / rhs);
        }

        public static bool operator ==(in Rat3Hybrid lhs, in Rat3Hybrid rhs)
        {
            return lhs.X == rhs.X && lhs.Y == rhs.Y && lhs.Z == rhs.Z;
        }
        public static bool operator !=(in Rat3Hybrid lhs, in Rat3Hybrid rhs)
        {
            return lhs.X != rhs.X || lhs.Y != rhs.Y || lhs.Z != rhs.Z;
        }

        public static BigRationalHybrid Dot(in Rat3Hybrid lhs, in Rat3Hybrid rhs)
        {
            return lhs.X * rhs.X + lhs.Y * rhs.Y + lhs.Z * rhs.Z;
        }

        /// <summary>Exact <see cref="BigRationalHybrid.Sign"/> of <see cref="Dot"/>; cheaper when all components use the long fast path.</summary>
        public static int DotSign(in Rat3Hybrid lhs, in Rat3Hybrid rhs) =>
            BigRationalHybrid.SignOfDot3(in lhs.X, in rhs.X, in lhs.Y, in rhs.Y, in lhs.Z, in rhs.Z);

        /// <summary>
        /// Non-zero vectors are anti-parallel (collinear with dot &lt; 0).
        /// Uses <see cref="DotSign"/> then cross components with early exit (no <see cref="Cross"/> allocation).
        /// Dot and cross share no product terms (same-index vs cross-index), so only the sign filter is shared.
        /// </summary>
        public static bool AreOppositeCollinear(in Rat3Hybrid a, in Rat3Hybrid b)
        {
            if (DotSign(a, b) >= 0)
                return false;

            if ((a.Y * b.Z - a.Z * b.Y).Sign() != 0)
                return false;
            if ((a.Z * b.X - a.X * b.Z).Sign() != 0)
                return false;
            return (a.X * b.Y - a.Y * b.X).Sign() == 0;
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
                    case 2:
                        return Z;
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
                    case 2:
                        Z = value;
                        break;
                }
            }
        }

        public override string ToString()
        {
            return X.ToString() + " " + Y.ToString() + " " + Z.ToString();
        }

        public static Rat3Hybrid Cross(in Rat3Hybrid lhs, in Rat3Hybrid rhs)
        {
            return new Rat3Hybrid(
                lhs.Y * rhs.Z - lhs.Z * rhs.Y,
                lhs.Z * rhs.X - lhs.X * rhs.Z,
                lhs.X * rhs.Y - lhs.Y * rhs.X
            );
        }

        public static BigRationalHybrid Abs(BigRationalHybrid x)
        {
            if (x < BigRationalHybrid.Zero)
                return -x;
            return x;
        }

        //public static BigRationalHybrid Dot(in Rat3Hybrid lhs, in Rat3Hybrid rhs)
        //{
        //    return lhs.X * rhs.X + lhs.Y * rhs.Y + lhs.Z * rhs.Z;
        //}
    }
}
