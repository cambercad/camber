using GeoCore;

namespace Curves
{
    /// <summary>
    /// Shared offset of a curve graph. Rebuilds after constraint solves; sampled
    /// children read this result.
    /// </summary>
    public class OffsetSketchNetwork2D : IOffsetSketchPieces
    {
        readonly SketchStripOffsetOptions _options;
        List<CurveStrip2D> _loops = new List<CurveStrip2D>();
        List<OffsetSketchPiece> _pieces = new List<OffsetSketchPiece>();
        long _fingerprint = long.MinValue;

        public IReadOnlyList<Curve2D> Sources { get; }
        public double Offset { get; }
        public IReadOnlyList<CurveStrip2D> Loops { get { return _loops; } }
        public IReadOnlyList<OffsetSketchPiece> Pieces { get { return _pieces; } }

        public OffsetSketchNetwork2D(
            IReadOnlyList<Curve2D> sources,
            double offset,
            SketchStripOffsetOptions options)
        {
            Sources = sources ?? throw new ArgumentNullException(nameof(sources));
            Offset = offset;
            _options = options;
            Update();
        }

        public void Update()
        {
            long fingerprint = OffsetSketchAssembler.SourceFingerprint(Sources);
            if (fingerprint == _fingerprint && _pieces.Count > 0)
                return;
            _fingerprint = fingerprint;

            List<List<Vec2D>> raw = SketchStripOffsetBuilder.BuildNetwork(
                Sources, Offset, _options, out List<List<int>> vertexIds, out OffsetSourceVertexMap map);
            var loops = new List<CurveStrip2D>(raw.Count);
            for (int i = 0; i < raw.Count; i++)
                loops.Add(CurveStrip2D.FromPolyline(raw[i], closed: true));
            _loops = loops;
            _pieces = OffsetSketchAssembler.Assemble(Sources, raw, vertexIds, map, closed: true, Offset);
        }

        public List<OffsetSketchNetworkLoop2D> CreateLoopCurves(CurveFlags flags = CurveFlags.None)
        {
            var result = new List<OffsetSketchNetworkLoop2D>(_loops.Count);
            for (int i = 0; i < _loops.Count; i++)
                result.Add(new OffsetSketchNetworkLoop2D(this, i, flags));
            return result;
        }

        public List<OffsetSampledCurve2D> CreateSampledCurves(CurveFlags flags = CurveFlags.None)
        {
            return OffsetSketchAssembler.CreateSampledCurves(this, flags);
        }
    }

    /// <summary>One closed loop of a network offset; tracks the parent graph after solves.</summary>
    public class OffsetSketchNetworkLoop2D : Curve2D, IDependentSketchCurve
    {
        readonly OffsetSketchNetwork2D _network;
        readonly int _index;

        public OffsetSketchNetworkLoop2D(OffsetSketchNetwork2D network, int index, CurveFlags flags = CurveFlags.None)
        {
            _network = network ?? throw new ArgumentNullException(nameof(network));
            _index = index;
            Flags = flags;
        }

        public OffsetSketchNetwork2D Network { get { return _network; } }

        public bool IsClosed { get { return true; } }

        CurveStrip2D Geometry
        {
            get
            {
                var loops = _network.Loops;
                if (loops == null || _index < 0 || _index >= loops.Count)
                    throw new InvalidOperationException("Network offset loop is no longer available.");
                return loops[_index];
            }
        }

        public void Update()
        {
            _network.Update();
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
            => Geometry.EvaluateAtNormalizedArcLength(uniform);

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
        {
            var result = new List<CurveVertex2D>();
            foreach (var segment in Geometry.Segments)
            {
                var part = segment.Tessellate(maxDeviation);
                if (result.Count > 0 && part.Count > 0)
                    part.RemoveAt(0);
                result.AddRange(part);
            }

            if (result.Count >= 2 &&
                Vec2DOps.DistanceSquared(result[0].Position, result[^1].Position) > 1e-16)
                result.Add(result[0]);
            return result;
        }

        public override List<CurveVertex2D> Tessellate(int pointCount)
        {
            if (pointCount < 2)
                pointCount = 2;
            var result = new List<CurveVertex2D>(pointCount);
            for (int i = 0; i < pointCount; i++)
                result.Add(EvaluateVertex(i / (double)(pointCount - 1)));
            return result;
        }

        public override double Length()
            => Geometry.TotalLength;

        public override List<Vec2D> ToReferencePoints()
        {
            var points = new List<Vec2D>();
            foreach (var segment in Geometry.Segments)
            {
                if (points.Count == 0)
                    points.Add(segment.StartPosition);
                points.Add(segment.EndPosition);
            }
            return points;
        }

        public override void UpdateFromReferencePoints(List<Vec2D> referencePoints)
            => throw new NotSupportedException("Network offset loops are derived geometry.");

        public override Curve2D GetCopy()
        {
            var copy = new OffsetSketchNetworkLoop2D(_network, _index, Flags);
            copy.Name = Name;
            return copy;
        }

        public override Curve2D Reverse()
        {
            return GetCopy();
        }
    }
}
