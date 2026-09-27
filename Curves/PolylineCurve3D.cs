using GeoCore;

namespace Curves
{
    /// <summary>
    /// 3D polyline whose tessellation is an authored list of <see cref="CurveVertex3D"/>.
    /// Typically used for sketch-to-mesh projections (<c>Up</c> = surface normal).
    /// </summary>
    public class PolylineCurve3D : Curve3D
    {
        private List<CurveVertex3D> _vertices;

        /// <summary>
        /// Exact lattice coordinates aligned with <see cref="Vertices"/> when
        /// available. Null for curves authored from ordinary doubles.
        /// </summary>
        public IReadOnlyList<Rat3Hybrid> PrecisionPositions { get; private set; }

        /// <summary>The converter that maps <see cref="PrecisionPositions"/> to world coordinates.</summary>
        public CoordinateConverter? PrecisionConverter { get; private set; }

        public IReadOnlyList<CurveVertex3D> Vertices => _vertices;

        public PolylineCurve3D(string name = null) : base(name)
        {
            _vertices = new List<CurveVertex3D>();
        }

        public PolylineCurve3D(List<CurveVertex3D> vertices, string name = null) : base(name)
        {
            _vertices = vertices == null ? new List<CurveVertex3D>() : new List<CurveVertex3D>(vertices);
        }

        public PolylineCurve3D(List<CurveVertex3D> vertices,
            IReadOnlyList<Rat3Hybrid> precisionPositions, CoordinateConverter converter,
            string name = null) : base(name)
        {
            if (vertices == null) throw new ArgumentNullException(nameof(vertices));
            if (precisionPositions == null) throw new ArgumentNullException(nameof(precisionPositions));
            if (precisionPositions.Count != vertices.Count)
                throw new ArgumentException("Exact points must align one-to-one with polyline vertices.", nameof(precisionPositions));
            _vertices = new List<CurveVertex3D>(vertices);
            PrecisionPositions = Array.AsReadOnly(precisionPositions.ToArray());
            PrecisionConverter = converter;
        }

        public override CurveVertex3D Evaluate(double uniform)
        {
            if (_vertices == null || _vertices.Count == 0)
                throw new InvalidOperationException("PolylineCurve3D has no vertices.");
            if (_vertices.Count == 1)
                return _vertices[0];

            double u = uniform;
            if (u < 0) u = 0;
            if (u > 1) u = 1;

            double scaled = u * (_vertices.Count - 1);
            int i0 = (int)Math.Floor(scaled);
            if (i0 >= _vertices.Count - 1)
                return _vertices[_vertices.Count - 1];
            int i1 = i0 + 1;
            double local = scaled - i0;
            CurveVertex3D a = _vertices[i0];
            CurveVertex3D b = _vertices[i1];
            Vec3D origin = (1 - local) * a.Origin + local * b.Origin;
            Vec3D tangent = ((1 - local) * a.Tangent + local * b.Tangent).Normalized();
            Vec3D up = ((1 - local) * a.Up + local * b.Up).Normalized();
            // Keep Up perpendicular to Tangent.
            up = (up - Vec3DOps.Dot(up, tangent) * tangent).Normalized();
            if (up.LengthSquared() < 1e-20)
                up = a.Up;
            return new CurveVertex3D(origin, tangent, up, u);
        }

        public override List<CurveVertex3D> Tessellate(double maxDeviation)
        {
            return new List<CurveVertex3D>(_vertices);
        }
    }
}
