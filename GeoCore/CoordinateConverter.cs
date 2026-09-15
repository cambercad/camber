namespace GeoCore
{
    public struct CoordinateConverter
    {
        private readonly Box3D operatingSpace;
        private readonly Vec3D offset;
        private readonly Box3I integerBoundingBox;
        private readonly double toIntegerScaling;
        private readonly double toOriginalScaling;

        /// <param name="operatingSpaceSlices">Number of lattice divisions along the longest box axis (default 1M ≈ former 20-bit grid).</param>
        public CoordinateConverter(Box3D operatingSpace = default, int operatingSpaceSlices = 1_000_000)
        {
            if (operatingSpace.IsEmpty())
                operatingSpace = new Box3D(new Vec3D(-1), new Vec3D(1));

            if (operatingSpaceSlices < 2)
                throw new ArgumentOutOfRangeException(nameof(operatingSpaceSlices), "operatingSpaceSlices must be at least 2.");

            int max = operatingSpaceSlices - 1;
            integerBoundingBox = new Box3I(new Int3(0, 0, 0), new Int3(max, max, max));
            this.operatingSpace = operatingSpace;

            Vec3D boxSize = new Vec3D(
                operatingSpace.Max.X - operatingSpace.Min.X,
                operatingSpace.Max.Y - operatingSpace.Min.Y,
                operatingSpace.Max.Z - operatingSpace.Min.Z
            );
            offset = operatingSpace.Min; // 0.5*(operatingSpace.Min + operatingSpace.Max);
            double s = Math.Max(boxSize.X, Math.Max(boxSize.Y, boxSize.Z));
            toIntegerScaling = (integerBoundingBox.Max.X - integerBoundingBox.Min.X) * (1.0 / s);

            toOriginalScaling = 1.0 / toIntegerScaling;
        }

        public double SmallestUnit()
        {
            return toOriginalScaling;
        }

        public CoordinateConverter(List<Vec3D> points, int operatingSpaceSlices = 1_000_000) : this(GetBox(points), operatingSpaceSlices)
        {
        }

        public CoordinateConverter(List<Vec3D> pointsA, List<Vec3D> pointsB, int operatingSpaceSlices = 1_000_000) : this(GetBox(pointsA, pointsB), operatingSpaceSlices)
        {
        }

        private static Box3D GetBox(List<Vec3D> points)
        {
            Box3D box = default;
            points.MinMax(out box.Min, out box.Max);
            return box;
        }

        private static Box3D GetBox(List<Vec3D> pointsA, List<Vec3D> pointsB)
        {
            var a = GetBox(pointsA);
            var b = GetBox(pointsB);
            a.IncludeBox(b);
            return a;
        }

        public Box3D OperatingSpace
        {
            get { return operatingSpace; }
        }

        public Box3I IntegerBoundingBox
        {
            get { return integerBoundingBox; }
        }

        public Int3 ConvertDirection(Vec3D p)
        {
            Int3 result;
            checked
            {
                result.X = (int)((p.X) * toIntegerScaling);
                result.Y = (int)((p.Y) * toIntegerScaling);
                result.Z = (int)((p.Z) * toIntegerScaling);
            }
            return result;
        }

        public Int3 Convert(Vec3D p)
        {
            Int3 result;
            checked
            {
                result.X = (int)((p.X - offset.X) * toIntegerScaling);
                result.Y = (int)((p.Y - offset.Y) * toIntegerScaling);
                result.Z = (int)((p.Z - offset.Z) * toIntegerScaling);
            }
            return result; // integerBoundingBox.Clamp(result);
        }

        public Vec3D Convert(Int3 p)
        {
            return new Vec3D(
                p.X * toOriginalScaling + offset.X,
                p.Y * toOriginalScaling + offset.Y,
                p.Z * toOriginalScaling + offset.Z
            );
        }
        public Vec3D Convert(in Rat3Hybrid p)
        {
            return new Vec3D(
                p.X.ToDouble() * toOriginalScaling + offset.X,
                p.Y.ToDouble() * toOriginalScaling + offset.Y,
                p.Z.ToDouble() * toOriginalScaling + offset.Z
            );
        }
        public List<Int3> Convert(List<Vec3D> p)
        {
            List<Int3> result = new List<Int3>(p.Count);
            for (int i = 0; i < p.Count; ++i)
                result.Add(Convert(p[i]));
            return result;
        }

        public List<Vec3D> Convert(List<Rat3Hybrid> p)
        {
            List<Vec3D> result = new List<Vec3D>();
            for (int i = 0; i < p.Count; ++i)
                result.Add(Convert(p[i]));
            return result;
        }
    }
}
