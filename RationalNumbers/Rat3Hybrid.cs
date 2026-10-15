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

        public void LimitGrowth()
        {
            X.LimitGrowth();
            Y.LimitGrowth();
            Z.LimitGrowth();
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

        /// <summary>Exact sign of (u × v) · (p × q), without building either cross product.</summary>
        public static int SignOfCrossDot(in Rat3Hybrid u, in Rat3Hybrid v, in Rat3Hybrid p, in Rat3Hybrid q) =>
            BigRationalHybrid.SignOfCrossDot(u.X, u.Y, u.Z, v.X, v.Y, v.Z, p.X, p.Y, p.Z, q.X, q.Y, q.Z);

        /// <summary>
        /// Exact sign of dot((b1 - a1) x (c1 - a1), (b0 - a0) x (c0 - a0)): whether the
        /// triangle (a1,b1,c1) keeps the orientation of (a0,b0,c0). A floating point
        /// evaluation with a rigorous error bound decides clear cases; every other case,
        /// including each exact zero, is settled by exact arithmetic.
        /// </summary>
        public static int SignOfTriangleNormalDot(in Rat3Hybrid a1, in Rat3Hybrid b1, in Rat3Hybrid c1,
            in Rat3Hybrid a0, in Rat3Hybrid b0, in Rat3Hybrid c0)
        {
            if (TryFilteredNormalDotSign(in a1, in b1, in c1, in a0, in b0, in c0, out int sign))
                return sign;
            return SignOfCrossDot(b1 - a1, c1 - a1, b0 - a0, c0 - a0);
        }

        // Every coordinate is within RelativeConversion of its exact value. A difference of two
        // coordinates is therefore within 2e-15 * (|p| + |q|) (conversion plus its own rounding),
        // and the products below add first-order terms plus 4e-16 relative per rounded operation.
        // The final bound is doubled again for second-order terms.
        private const double RelativeCoordinateError = 2e-15;
        private const double RoundingError = 4e-16;
        private const double MaxFilterMagnitude = 1e40;
        private const double MinFilterMagnitude = 1e-30;

        private static bool TryFilteredNormalDotSign(in Rat3Hybrid a1, in Rat3Hybrid b1, in Rat3Hybrid c1,
            in Rat3Hybrid a0, in Rat3Hybrid b0, in Rat3Hybrid c0, out int sign)
        {
            sign = 0;
            Span<double> p = stackalloc double[18];
            if (!TryLoad(in a1, p, 0) || !TryLoad(in b1, p, 3) || !TryLoad(in c1, p, 6) ||
                !TryLoad(in a0, p, 9) || !TryLoad(in b0, p, 12) || !TryLoad(in c0, p, 15))
                return false;

            Span<double> w1 = stackalloc double[3];
            Span<double> e1 = stackalloc double[3];
            Span<double> w0 = stackalloc double[3];
            Span<double> e0 = stackalloc double[3];
            CrossWithError(p, 0, 3, 6, w1, e1);
            CrossWithError(p, 9, 12, 15, w0, e0);

            double dot = 0, error = 0;
            for (int k = 0; k < 3; k++)
            {
                double product = w1[k] * w0[k];
                dot += product;
                error += Math.Abs(w1[k]) * e0[k] + Math.Abs(w0[k]) * e1[k] + e1[k] * e0[k]
                    + RoundingError * Math.Abs(product);
            }
            error *= 2;
            if (!(Math.Abs(dot) > error)) // also rejects NaN
                return false;
            sign = dot > 0 ? 1 : -1;
            return true;
        }

        private static bool TryLoad(in Rat3Hybrid point, Span<double> destination, int offset)
        {
            if (!point.X.TryApproximate(out double x) || !point.Y.TryApproximate(out double y) ||
                !point.Z.TryApproximate(out double z))
                return false;
            return Usable(x) && Usable(y) && Usable(z) && Store(destination, offset, x, y, z);
        }

        private static bool Usable(double v)
        {
            double magnitude = Math.Abs(v);
            return magnitude == 0 || (magnitude >= MinFilterMagnitude && magnitude <= MaxFilterMagnitude);
        }

        private static bool Store(Span<double> destination, int offset, double x, double y, double z)
        {
            destination[offset] = x;
            destination[offset + 1] = y;
            destination[offset + 2] = z;
            return true;
        }

        /// <summary>w = (b - a) x (c - a) with a componentwise absolute error bound per component.</summary>
        private static void CrossWithError(ReadOnlySpan<double> p, int a, int b, int c, Span<double> w, Span<double> error)
        {
            Span<double> u = stackalloc double[3];
            Span<double> v = stackalloc double[3];
            Span<double> eu = stackalloc double[3];
            Span<double> ev = stackalloc double[3];
            for (int k = 0; k < 3; k++)
            {
                u[k] = p[b + k] - p[a + k];
                v[k] = p[c + k] - p[a + k];
                eu[k] = RelativeCoordinateError * (Math.Abs(p[b + k]) + Math.Abs(p[a + k]));
                ev[k] = RelativeCoordinateError * (Math.Abs(p[c + k]) + Math.Abs(p[a + k]));
            }
            for (int k = 0; k < 3; k++)
            {
                int i = (k + 1) % 3, j = (k + 2) % 3;
                w[k] = u[i] * v[j] - u[j] * v[i];
                error[k] = Math.Abs(u[i]) * ev[j] + Math.Abs(v[j]) * eu[i] + eu[i] * ev[j]
                    + Math.Abs(u[j]) * ev[i] + Math.Abs(v[i]) * eu[j] + eu[j] * ev[i]
                    + RoundingError * (Math.Abs(u[i] * v[j]) + Math.Abs(u[j] * v[i]));
            }
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
            // Opposite collinear vectors have opposite signs on every axis, and a zero
            // only opposite another zero. Sign is much cheaper than a product, and a
            // mismatch is exact: it cannot hide a truly anti-parallel pair.
            if (AxisRejectsOpposite(a.X, b.X) || AxisRejectsOpposite(a.Y, b.Y) || AxisRejectsOpposite(a.Z, b.Z))
                return false;

            if (DotSign(a, b) >= 0)
                return false;

            if (BigRationalHybrid.SignOf2D(a.Y, a.Z, b.Y, b.Z) != 0)
                return false;
            if (BigRationalHybrid.SignOf2D(a.Z, a.X, b.Z, b.X) != 0)
                return false;
            return BigRationalHybrid.SignOf2D(a.X, a.Y, b.X, b.Y) == 0;
        }

        private static bool AxisRejectsOpposite(in BigRationalHybrid a, in BigRationalHybrid b)
        {
            int sa = a.Sign();
            int sb = b.Sign();
            if (sa == 0)
                return sb != 0;
            if (sb == 0)
                return true;
            return sa == sb;
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
