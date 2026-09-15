namespace GeoCore
{
    public static class Rat3HybridExtensions
    {
        public static Int3 ToInt(this Rat3Hybrid p)
        {
            return new Int3(p.X.ToIntRoundDown(), p.Y.ToIntRoundDown(), p.Z.ToIntRoundDown());
        }

        public static Box3I GetBox(this Rat3Hybrid p)
        {
            // Store original signs before taking absolute values
            int signX = p.X.Sign();
            int signY = p.Y.Sign();
            int signZ = p.Z.Sign();

            p.X = Rat3Hybrid.Abs(p.X);
            p.Y = Rat3Hybrid.Abs(p.Y);
            p.Z = Rat3Hybrid.Abs(p.Z);

            var lower = p.ToInt();
            var upper = new Int3(lower.X + 1, lower.Y + 1, lower.Z + 1);

            if (signX < 0)
            {
                lower.X = -lower.X;
                upper.X = -upper.X;
                Algorithms.Swap(ref lower.X, ref upper.X);
            }
            if (signY < 0)
            {
                lower.Y = -lower.Y;
                upper.Y = -upper.Y;
                Algorithms.Swap(ref lower.Y, ref upper.Y);
            }
            if (signZ < 0)
            {
                lower.Z = -lower.Z;
                upper.Z = -upper.Z;
                Algorithms.Swap(ref lower.Z, ref upper.Z);
            }

            return new Box3I(lower, upper);
        }
    }
}
