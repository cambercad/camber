using GeoCore;

namespace Curves
{
    /// <summary>
    /// Compound sketch curve: offset of a connected source strip, rebuilt after constraint solves.
    /// </summary>
    public class OffsetSketchStrip2D : Curve2D, IDependentSketchCurve, IOffsetSketchPieces
    {
        private CurveStrip2D _geometry;
        private readonly SketchStripOffsetOptions _options;

        public IReadOnlyList<Curve2D> SourceStrip { get; }
        public IReadOnlyList<Curve2D> Sources { get { return SourceStrip; } }
        public double Offset { get; }
        public bool IsClosed { get; private set; }
        public CurveStrip2D Geometry => _geometry;
        public IReadOnlyList<OffsetSketchPiece> Pieces { get { return _pieces; } }

        List<OffsetSketchPiece> _pieces = new List<OffsetSketchPiece>();
        long _fingerprint = long.MinValue;

        public OffsetSketchStrip2D(
            IReadOnlyList<Curve2D> sourceStrip,
            double offset,
            SketchStripOffsetOptions options = default,
            CurveFlags flags = CurveFlags.None)
        {
            SourceStrip = sourceStrip ?? throw new ArgumentNullException(nameof(sourceStrip));
            Offset = offset;
            _options = options;
            Flags = flags;
            Update();
        }

        public void Update()
        {
            long fingerprint = OffsetSketchAssembler.SourceFingerprint(SourceStrip);
            if (fingerprint == _fingerprint && _geometry != null && _pieces.Count > 0)
                return;
            _fingerprint = fingerprint;

            _geometry = SketchStripOffsetBuilder.Build(
                SourceStrip, Offset, _options, out bool closed, out var loops, out var vertexIds, out var map);
            IsClosed = closed;
            _pieces = OffsetSketchAssembler.Assemble(SourceStrip, loops, vertexIds, map, closed, Offset);
        }

        public List<OffsetSampledCurve2D> CreateSampledCurves(CurveFlags flags = CurveFlags.None)
        {
            return OffsetSketchAssembler.CreateSampledCurves(this, flags);
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
            => _geometry.EvaluateAtNormalizedArcLength(uniform);

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
        {
            var result = new List<CurveVertex2D>();
            foreach (var segment in _geometry.Segments)
            {
                var part = segment.Tessellate(maxDeviation);
                if (result.Count > 0 && part.Count > 0)
                    part.RemoveAt(0);
                result.AddRange(part);
            }

            return result;
        }

        public override List<CurveVertex2D> Tessellate(int pointCount)
        {
            if (pointCount < 2)
                pointCount = 2;

            var result = new List<CurveVertex2D>(pointCount);
            for (int i = 0; i < pointCount; i++)
            {
                double u = i / (double)(pointCount - 1);
                result.Add(EvaluateVertex(u));
            }

            return result;
        }

        public override double Length()
            => _geometry.TotalLength;

        public override List<Vec2D> ToReferencePoints()
        {
            var points = new List<Vec2D>();
            foreach (var segment in _geometry.Segments)
            {
                if (points.Count == 0)
                    points.Add(segment.StartPosition);
                points.Add(segment.EndPosition);
            }

            return points;
        }

        public override void UpdateFromReferencePoints(List<Vec2D> referencePoints)
            => throw new NotSupportedException("Offset sketch strips are derived geometry and cannot be updated from reference points.");

        public override Curve2D GetCopy()
        {
            var copy = new OffsetSketchStrip2D(SourceStrip, Offset, _options, Flags);
            copy.Name = Name;
            return copy;
        }

        public override Curve2D Reverse()
        {
            var points = ToReferencePoints();
            if (points.Count >= 2)
            {
                points.Reverse();
                bool closed = IsClosed && points.Count > 2 &&
                              Vec2DOps.DistanceSquared(points[0], points[^1]) <= SketchStripTessellator.DefaultClosedToleranceSq;
                var reversed = CurveStrip2D.FromPolyline(points, closed);
                var copy = new OffsetSketchStrip2D(SourceStrip, Offset, _options, Flags);
                copy._geometry = reversed;
                copy.IsClosed = closed;
                copy.Name = Name;
                return copy;
            }

            return GetCopy();
        }
    }
}
