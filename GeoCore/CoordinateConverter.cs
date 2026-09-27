namespace GeoCore
{
    public struct CoordinateConverter
    {
        private readonly Box3D operatingSpace;
        private readonly Vec3D offset;
        private readonly Box3I integerBoundingBox;
        private readonly double toIntegerScaling;
        private readonly double toOriginalScaling;
        private readonly BigRationalHybrid exactToOriginalScaling;

        /// <param name="operatingSpaceSlices">Number of lattice divisions along the longest box axis (default 1M ≈ former 20-bit grid).</param>
        public CoordinateConverter(Box3D operatingSpace = default, int operatingSpaceSlices = 1_000_000)
        {
            if (operatingSpace.IsEmpty())
                operatingSpace = new Box3D(new Vec3D(-1), new Vec3D(1));

            if (operatingSpaceSlices < 2)
                throw new ArgumentOutOfRangeException(nameof(operatingSpaceSlices), "operatingSpaceSlices must be at least 2.");

            this.operatingSpace = operatingSpace;

            Vec3D boxSize = new Vec3D(
                operatingSpace.Max.X - operatingSpace.Min.X,
                operatingSpace.Max.Y - operatingSpace.Min.Y,
                operatingSpace.Max.Z - operatingSpace.Min.Z
            );
            offset = operatingSpace.Min; // 0.5*(operatingSpace.Min + operatingSpace.Max);
            double longestAxisSize = Math.Max(boxSize.X, Math.Max(boxSize.Y, boxSize.Z));
            int longestAxisDivisions = operatingSpaceSlices - 1;
            toIntegerScaling = longestAxisDivisions / longestAxisSize;
            exactToOriginalScaling = ExactRational(longestAxisSize) / new BigRationalHybrid(longestAxisDivisions);

            // Use the same uniform scale on every axis. The integer bounds describe
            // the operating box in that isotropic lattice; shorter axes therefore
            // have proportionally fewer divisions instead of being stretched to a cube.
            integerBoundingBox = new Box3I(
                new Int3(0, 0, 0),
                new Int3(
                    GetIntegerExtent(boxSize.X, longestAxisSize, longestAxisDivisions),
                    GetIntegerExtent(boxSize.Y, longestAxisSize, longestAxisDivisions),
                    GetIntegerExtent(boxSize.Z, longestAxisSize, longestAxisDivisions)));

            toOriginalScaling = 1.0 / toIntegerScaling;
        }

        private static int GetIntegerExtent(double axisSize, double longestAxisSize, int longestAxisDivisions)
        {
            if (axisSize == longestAxisSize)
                return longestAxisDivisions;

            // Round bounds outward: this is a conservative integer AABB even if the
            // operating-space endpoint falls between lattice coordinates. A minimum
            // extent of one preserves the non-degenerate-box invariant at resolutions
            // too coarse to represent a very thin axis.
            double extent = Math.Ceiling(axisSize / longestAxisSize * longestAxisDivisions);
            return Math.Min(longestAxisDivisions, Math.Max(1, checked((int)extent)));
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

        /// <summary>
        /// Re-express an exact point from another converter's integer lattice in
        /// this converter's lattice. No rounding is performed; coordinates may
        /// therefore be rational even when the two grids do not align.
        /// </summary>
        public Rat3Hybrid ConvertExact(in Rat3Hybrid point, CoordinateConverter source)
        {
            var sourceUnit = source.exactToOriginalScaling;
            var targetUnit = exactToOriginalScaling;
            return new Rat3Hybrid(
                ((point.X * sourceUnit + ExactRational(source.offset.X) - ExactRational(offset.X)) / targetUnit),
                ((point.Y * sourceUnit + ExactRational(source.offset.Y) - ExactRational(offset.Y)) / targetUnit),
                ((point.Z * sourceUnit + ExactRational(source.offset.Z) - ExactRational(offset.Z)) / targetUnit));
        }

        /// <summary>Whether both converters have the same exact scale and world origin.</summary>
        public bool HasSameLattice(CoordinateConverter other) =>
            offset.X == other.offset.X && offset.Y == other.offset.Y && offset.Z == other.offset.Z &&
            exactToOriginalScaling.CompareTo(other.exactToOriginalScaling) == 0;

        private static BigRationalHybrid ExactRational(double value)
        {
            if (!double.IsFinite(value))
                throw new ArgumentOutOfRangeException(nameof(value), "Lattice parameters must be finite.");
            BigRational rational = value;
            return new BigRationalHybrid(rational.Numerator, rational.Denominator);
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
