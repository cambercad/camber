using GeoCore;

namespace Curves
{
    /// <summary>
    /// Offset fragment stored as a sampled polyline. Rebuilds with the parent
    /// network/strip after constraint solves.
    /// </summary>
    public class OffsetSampledCurve2D : SampledCurve, IDependentSketchCurve
    {
        readonly IOffsetSketchPieces _owner;
        readonly int _index;

        public OffsetSampledCurve2D(IOffsetSketchPieces owner, int index, CurveFlags flags = CurveFlags.None)
        {
            _owner = owner ?? throw new ArgumentNullException(nameof(owner));
            _index = index;
            Flags = flags;
            ApplyPiece();
        }

        public IOffsetSketchPieces Owner { get { return _owner; } }
        public int PieceIndex { get { return _index; } }

        public int LoopIndex
        {
            get
            {
                var piece = CurrentPiece();
                return piece == null ? 0 : piece.LoopIndex;
            }
        }

        public int SourceIndex
        {
            get
            {
                var piece = CurrentPiece();
                return piece == null ? -1 : piece.SourceIndex;
            }
        }

        public OffsetSketchPieceSide Side
        {
            get
            {
                var piece = CurrentPiece();
                return piece == null ? OffsetSketchPieceSide.None : piece.Side;
            }
        }

        public void Update()
        {
            _owner.Update();
            ApplyPiece();
        }

        public override Curve2D GetCopy()
        {
            var copy = new OffsetSampledCurve2D(_owner, _index, Flags);
            copy.Name = Name;
            return copy;
        }

        OffsetSketchPiece CurrentPiece()
        {
            var pieces = _owner.Pieces;
            if (pieces == null || _index < 0 || _index >= pieces.Count)
                return null;
            return pieces[_index];
        }

        void ApplyPiece()
        {
            var piece = CurrentPiece();
            List<Vec2D> points = piece != null && piece.Points != null
                ? new List<Vec2D>(piece.Points)
                : new List<Vec2D> { Vec2DOps.Zero, Vec2DOps.Zero };
            if (points.Count < 2)
            {
                Vec2D p = points.Count == 1 ? points[0] : Vec2DOps.Zero;
                points = new List<Vec2D> { p, p };
            }

            var normals = new List<Vec2D>(points.Count);
            for (int i = 0; i < points.Count; i++)
            {
                Vec2D d = i + 1 < points.Count
                    ? points[i + 1] - points[i]
                    : points[i] - points[i - 1];
                double len = d.Length();
                if (len > 1e-12)
                    d = new Vec2D(-d.Y / len, d.X / len);
                else
                    d = Vec2DOps.Zero;
                normals.Add(d);
            }

            Reset(points, normals);
        }
    }
}
