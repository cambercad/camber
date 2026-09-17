using GeoCore;

namespace GeoCore
{
    public enum InCircleResult
    {
        Inside,
        Outside,
        OnCircle
    }

    public interface ITriangulationOptimizerArithmetic<Vec>
    {
        InCircleResult InCircle(in Vec a, in Vec b, in Vec c, Vec candidate);
    }

    public interface ITriangulationArithmetic<Vec, Scalar>
    {
        Box2D GetBounds(in Vec v);

        int Orient2D(in Vec a, in Vec b, in Vec c);
        BigRationalHybrid SignedArea2DTimesTwo(in Vec pa, in Vec pb, in Vec pc); //Only for debuggign - result is squared, might need more space/bits, therefore fixed return type of BigRationalHybrid
        Scalar Get(in Vec v, int componentIndex); //0 for x, 1 for y, 2 for z etc 
        void Set(ref Vec v, int componentIndex, Scalar value);
        Scalar Min(in Scalar a, in Scalar b);
        Scalar Max(in Scalar a, in Scalar b);
        //Scalar Add(in Scalar a, in Scalar b);
        //int Sign(in Scalar a);
        int Compare(in Scalar a, in Scalar b);

        //InCircleResult InCircle(in Vec a, in Vec b, in Vec c, Vec candidate);
    }

    public struct Vec2DCircleArithmeticNoPredicates : ITriangulationOptimizerArithmetic<Vec2D>
    {
        public InCircleResult InCircle(in Vec2D pa, in Vec2D pb, in Vec2D pc, Vec2D pd)
        {
            const double eps = 1e-8; // This avoids enless forth and back flipping which can happen with floating point precision

            double adx, ady, bdx, bdy, cdx, cdy;
            double abdet, bcdet, cadet;
            double alift, blift, clift;

            adx = pa.X - pd.X;
            ady = pa.Y - pd.Y;
            bdx = pb.X - pd.X;
            bdy = pb.Y - pd.Y;
            cdx = pc.X - pd.X;
            cdy = pc.Y - pd.Y;

            abdet = adx * bdy - bdx * ady;
            bcdet = bdx * cdy - cdx * bdy;
            cadet = cdx * ady - adx * cdy;
            alift = adx * adx + ady * ady;
            blift = bdx * bdx + bdy * bdy;
            clift = cdx * cdx + cdy * cdy;

            double result = alift * bcdet + blift * cadet + clift * abdet;

            if (result > eps)
                return InCircleResult.Inside;
            else 
                return InCircleResult.Outside;        
        }
    }

    public struct Int2CircleArithmetic : ITriangulationOptimizerArithmetic<Int2>
    {
        public InCircleResult InCircle(in Int2 pa, in Int2 pb, in Int2 pc, Int2 pd)
        {
            long adx, ady, bdx, bdy, cdx, cdy;
            Int128 abdet, bcdet, cadet;
            Int128 alift, blift, clift;

            adx = pa.X - pd.X;
            ady = pa.Y - pd.Y;
            bdx = pb.X - pd.X;
            bdy = pb.Y - pd.Y;
            cdx = pc.X - pd.X;
            cdy = pc.Y - pd.Y;

            abdet = adx * bdy - bdx * ady;
            bcdet = bdx * cdy - cdx * bdy;
            cadet = cdx * ady - adx * cdy;
            alift = adx * adx + ady * ady;
            blift = bdx * bdx + bdy * bdy;
            clift = cdx * cdx + cdy * cdy;

            Int128 result = alift * bcdet + blift * cadet + clift * abdet;

            if (result > 0)
                return InCircleResult.Inside;
            else if (result < 0)
                return InCircleResult.Outside;
            else
                return InCircleResult.OnCircle;
        }
    }

    public struct Rat2HybridCircleArithmetic : ITriangulationOptimizerArithmetic<Rat2Hybrid>
    {
        public InCircleResult InCircle(in Rat2Hybrid pa, in Rat2Hybrid pb, in Rat2Hybrid pc, Rat2Hybrid pd)
        {
            if (BigRationalHybrid.TryInCircleSign(
                    in pa.X, in pa.Y, in pb.X, in pb.Y, in pc.X, in pc.Y, in pd.X, in pd.Y, out int sign))
            {
                if (sign > 0)
                    return InCircleResult.Inside;
                if (sign < 0)
                    return InCircleResult.Outside;
                return InCircleResult.OnCircle;
            }

            BigRationalHybrid adx, ady, bdx, bdy, cdx, cdy;
            BigRationalHybrid abdet, bcdet, cadet;
            BigRationalHybrid alift, blift, clift;

            adx = pa.X - pd.X;
            ady = pa.Y - pd.Y;
            bdx = pb.X - pd.X;
            bdy = pb.Y - pd.Y;
            cdx = pc.X - pd.X;
            cdy = pc.Y - pd.Y;

            abdet = adx * bdy - bdx * ady;
            bcdet = bdx * cdy - cdx * bdy;
            cadet = cdx * ady - adx * cdy;
            alift = adx * adx + ady * ady;
            blift = bdx * bdx + bdy * bdy;
            clift = cdx * cdx + cdy * cdy;

            BigRationalHybrid result = alift * bcdet + blift * cadet + clift * abdet;

            if (result > BigRationalHybrid.Zero)
                return InCircleResult.Inside;
            else if (result < BigRationalHybrid.Zero)
                return InCircleResult.Outside;
            else
                return InCircleResult.OnCircle;
        }
    }

    public struct Vec2DArithmeticNoPredicates : ITriangulationArithmetic<Vec2D, double>
    {
        public int Compare(in double a, in double b) { return a.CompareTo(b); }
        public double Get(in Vec2D v, int componentIndex) { return v.Get(componentIndex); }
        public void Set(ref Vec2D v, int componentIndex, double value) { v.Get(componentIndex) = value; }
        public double Max(in double a, in double b) { return Math.Max(a, b); }
        public double Min(in double a, in double b) { return Math.Min(a, b); }
        public int Orient2D(in Vec2D pa, in Vec2D pb, in Vec2D pc) 
        {
            return Math.Sign(SignedArea2DTimesTwoBasic(pa, pb, pc));
        }
        public double SignedArea2DTimesTwoBasic(in Vec2D pa, in Vec2D pb, in Vec2D pc)
        {
            double acx, bcx, acy, bcy;

            acx = pa.X - pc.X;
            bcx = pb.X - pc.X;
            acy = pa.Y - pc.Y;
            bcy = pb.Y - pc.Y;
            return acx * bcy - acy * bcx;
        }

        public BigRationalHybrid SignedArea2DTimesTwo(in Vec2D pa, in Vec2D pb, in Vec2D pc)
        {
            BigRational br = SignedArea2DTimesTwoBasic(pa, pb, pc);
            return new BigRationalHybrid(br.Numerator, br.Denominator);
        }

        public Box2D GetBounds(in Vec2D v)
        {
            return new Box2D(new Vec2D(v.X, v.Y), new Vec2D(v.X, v.Y));
        }
    }

    public struct Int2Arithmetic : ITriangulationArithmetic<Int2, int>
    {
        public int Compare(in int a, in int b) { return a.CompareTo(b); }
        public int Get(in Int2 v, int componentIndex) { return v.Get(componentIndex); }
        public void Set(ref Int2 v, int componentIndex, int value) { v.Get(componentIndex) = value; }
        public int Max(in int a, in int b) { return Math.Max(a, b); }
        public int Min(in int a, in int b) { return Math.Min(a, b); }
        public int Orient2D(in Int2 pa, in Int2 pb, in Int2 pc)
        {
            return Math.Sign(SignedArea2DTimesTwoBasic(pa, pb, pc));
        }
        public long SignedArea2DTimesTwoBasic(in Int2 pa, in Int2 pb, in Int2 pc)
        {
            long acx, bcx, acy, bcy;

            acx = pa.X - pc.X;
            bcx = pb.X - pc.X;
            acy = pa.Y - pc.Y;
            bcy = pb.Y - pc.Y;
            //long mul = Math.BigMul(acx, bcy);
            return acx * bcy - acy * bcx;
        }

        public BigRationalHybrid SignedArea2DTimesTwo(in Int2 pa, in Int2 pb, in Int2 pc)
        {
            return new BigRationalHybrid(SignedArea2DTimesTwoBasic(pa, pb, pc));
        }

        public Box2D GetBounds(in Int2 v)
        {
            return new Box2D(new Vec2D(v.X, v.Y), new Vec2D(v.X, v.Y));
        }
    }

    public struct Rat2HybridArithmetic : ITriangulationArithmetic<Rat2Hybrid, BigRationalHybrid>
    {
        //returns -1 if a b
        public int Compare(in BigRationalHybrid a, in BigRationalHybrid b) { return a.CompareTo(b); }

        public BigRationalHybrid Get(in Rat2Hybrid v, int componentIndex) { return v[componentIndex]; }
        public void Set(ref Rat2Hybrid v, int componentIndex, BigRationalHybrid value) { v[componentIndex] = value; }

        public BigRationalHybrid Max(in BigRationalHybrid a, in BigRationalHybrid b) { return BigRationalHybrid.Max(a, b); }

        public BigRationalHybrid Min(in BigRationalHybrid a, in BigRationalHybrid b) { return BigRationalHybrid.Min(a, b); }

        public int Orient2D(in Rat2Hybrid pa, in Rat2Hybrid pb, in Rat2Hybrid pc)
        {
            return Rat2Hybrid.Orient2DSign(in pa, in pb, in pc);
        }

        public BigRationalHybrid SignedArea2DTimesTwo(in Rat2Hybrid pa, in Rat2Hybrid pb, in Rat2Hybrid pc)
        {
            var acx = pa.X - pc.X;
            var bcx = pb.X - pc.X;
            var acy = pa.Y - pc.Y;
            var bcy = pb.Y - pc.Y;
            return acx * bcy - acy * bcx;
        }

        public Box2D GetBounds(in Rat2Hybrid v)
        {
            var x = Enclose(v.X);
            var y = Enclose(v.Y);
            return new Box2D(new Vec2D(x.Min, y.Min), new Vec2D(x.Max, y.Max));
        }

        private static (double Min, double Max) Enclose(BigRationalHybrid value)
        {
            double estimate = value.ToDouble();
            if (value.IsInt32) return (estimate, estimate);
            var exact = new BigRational(value.Numerator(), value.Denominator());
            double lower = double.IsPositiveInfinity(estimate) ? double.MaxValue : estimate;
            double upper = double.IsNegativeInfinity(estimate) ? -double.MaxValue : estimate;
            // A rounded point is not an enclosure. Move outward until exact
            // comparison certifies each bound; no geometric tolerance is used.
            while (double.IsFinite(lower) && new BigRational(lower) > exact)
                lower = Math.BitDecrement(lower);
            while (double.IsFinite(upper) && new BigRational(upper) < exact)
                upper = Math.BitIncrement(upper);
            return (lower, upper);
        }
    }
}
