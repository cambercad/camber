using System;
using System.Collections.Generic;
using GeoCore;

namespace Curves
{
    /// <summary>
    /// Ordered, connected <see cref="Curve2D"/> segments with cached cumulative arc length.
    /// Supports evaluation at normalized arc length u ∈ [0,1] by mapping to segment-local parameter
    /// (arc-length proportional: t = localDistance / segmentLength), which is exact for lines and
    /// circular arcs whose parameter is proportional to swept angle.
    /// </summary>
    public sealed class CurveStrip2D
    {
        private readonly Curve2D[] _segments;
        /// <summary>Length before segment i; length = segments.Count + 1; last entry equals <see cref="TotalLength"/>.</summary>
        private readonly double[] _cumStart;
        private readonly Vec2D[] _polyVertices;
        private readonly Vec2D[] _polyNormals;
        private readonly bool _closedPolyline;
        private readonly double[] _polyDistBuf;

        private CurveStrip2D(
            Curve2D[] segments,
            double[] cumStart,
            double totalLength,
            Vec2D[] polyVertices,
            Vec2D[] polyNormals,
            bool closedPolyline,
            double[] polyDistBuf)
        {
            _segments = segments;
            _cumStart = cumStart;
            TotalLength = totalLength;
            _polyVertices = polyVertices;
            _polyNormals = polyNormals;
            _closedPolyline = closedPolyline;
            _polyDistBuf = polyDistBuf;
        }

        public IReadOnlyList<Curve2D> Segments => _segments;

        /// <summary>Total arc length of the strip (including closing edge when built as a closed polyline ring).</summary>
        public double TotalLength { get; }

        /// <summary>Cumulative arc length at the start of each segment and one past the end; same length as <see cref="Segments"/> + 1.</summary>
        public IReadOnlyList<double> CumulativeLengthAtSegmentStart => _cumStart;

        public bool HasPolylineSamplingData => _polyVertices != null && _polyNormals != null && _polyDistBuf != null;

        public IReadOnlyList<Vec2D> PolylineVertices => _polyVertices;


        public IReadOnlyList<Vec2D> PolylineVertexNormals => _polyNormals;

        /// <summary>Open-chain distance from first vertex along poly edges (no closing chord); same as <see cref="LineStrip2D.BuildDistanceBuffer"/>.</summary>
        public IReadOnlyList<double> PolylineOpenChainDistances => _polyDistBuf;

        public bool IsClosedPolyline => _closedPolyline;

        /// <summary>Builds a strip of <see cref="Line2D"/> edges along consecutive vertices; if <paramref name="closed"/>, adds edge from last to first.</summary>
        public static CurveStrip2D FromPolyline(IReadOnlyList<Vec2D> vertices, bool closed)
        {
            if (vertices == null || vertices.Count < 2)
                throw new ArgumentException("Polyline strip requires at least two vertices.", nameof(vertices));
            int n = vertices.Count;
            int segCount = closed ? n : n - 1;
            var segments = new Curve2D[segCount];
            for (int i = 0; i < segCount; i++)
            {
                Vec2D a = vertices[i];
                Vec2D b = closed && i == segCount - 1 ? vertices[0] : vertices[i + 1];
                segments[i] = new Line2D(a, b);
            }
            return FromSegments(segments);
        }

        /// <summary>
        /// Tessellated sketch polyline with per-vertex normals (loft / extruder style). Enables
        /// <see cref="EvaluatePositionAndVertexNormalAtNormalizedArcLength"/> identical to piecewise-linear
        /// <see cref="LineStrip2D"/> sampling with cyclic closure.
        /// </summary>
        public static CurveStrip2D FromTessellatedPolyline(
            IReadOnlyList<Vec2D> vertices,
            IReadOnlyList<Vec2D> vertexNormals,
            bool closed)
        {
            if (vertices == null || vertexNormals == null || vertices.Count != vertexNormals.Count)
                throw new ArgumentException("Vertices and vertex normals must have the same length.", nameof(vertices));
            if (vertices.Count < 2)
                throw new ArgumentException("Need at least two vertices.", nameof(vertices));

            int n = vertices.Count;
            var pv = new Vec2D[n];
            var pn = new Vec2D[n];
            for (int i = 0; i < n; i++)
            {
                pv[i] = vertices[i];
                pn[i] = vertexNormals[i];
            }

            int segCount = closed ? n : n - 1;
            var segments = new Curve2D[segCount];
            for (int i = 0; i < segCount; i++)
            {
                Vec2D a = pv[i];
                Vec2D b = closed && i == segCount - 1 ? pv[0] : pv[i + 1];
                segments[i] = new Line2D(a, b);
            }

            var cum = new double[segments.Length + 1];
            double total = 0;
            for (int i = 0; i < segments.Length; i++)
            {
                cum[i] = total;
                total += segments[i].Length();
            }
            cum[segments.Length] = total;
            if (total <= 1e-30)
                throw new ArgumentException("Degenerate strip (zero total length).", nameof(vertices));

            double[] distBuf = LineStrip2D.BuildDistanceBuffer(pv);
            return new CurveStrip2D(segments, cum, total, pv, pn, closed, distBuf);
        }

        /// <summary>Creates a strip from already-connected segments (open chain: parameter spans the sum of lengths).</summary>
        public static CurveStrip2D FromSegments(IReadOnlyList<Curve2D> segments)
        {
            if (segments == null || segments.Count == 0)
                throw new ArgumentException("At least one segment is required.", nameof(segments));
            var arr = new Curve2D[segments.Count];
            for (int i = 0; i < segments.Count; i++)
                arr[i] = segments[i] ?? throw new ArgumentException("Null segment.", nameof(segments));

            var cum = new double[arr.Length + 1];
            double t = 0;
            for (int i = 0; i < arr.Length; i++)
            {
                cum[i] = t;
                t += arr[i].Length();
            }
            cum[arr.Length] = t;
            if (t <= 1e-30)
                throw new ArgumentException("Degenerate strip (zero total length).", nameof(segments));
            return new CurveStrip2D(arr, cum, t, null, null, false, null);
        }

        /// <summary>Closed-loop total length matching loft: open polyline length plus closing chord when <paramref name="closed"/>.</summary>
        public static double TotalLengthOfPolyline(IReadOnlyList<Vec2D> vertices, bool closed)
        {
            var strip = new LineStrip2D(vertices is List<Vec2D> l ? l : new List<Vec2D>(vertices));
            double total = strip.TotalLength;
            if (closed && vertices.Count >= 2)
            {
                Vec2D a = vertices[^1];
                Vec2D b = vertices[0];
                double dx = b.X - a.X, dy = b.Y - a.Y;
                total += Math.Sqrt(dx * dx + dy * dy);
            }
            return total;
        }

        /// <summary>Normalized arc parameter u ∈ [0,1] at each polyline vertex (loft merge anchors).</summary>
        public static List<double> VertexNormalizedUParameters(IReadOnlyList<Vec2D> vertices, bool closed)
        {
            double total = TotalLengthOfPolyline(vertices, closed);
            if (total < 1e-30)
                return new List<double>();
            double[] distBuf = LineStrip2D.BuildDistanceBuffer(vertices is List<Vec2D> l ? l : new List<Vec2D>(vertices));
            var list = new List<double>(vertices.Count);
            for (int i = 0; i < vertices.Count; i++)
                list.Add(distBuf[i] / total);
            return list;
        }

        /// <inheritdoc cref="VertexNormalizedUParameters"/>
        public double NormalizedUAtPolylineVertex(int vertexIndex)
        {
            if (_polyDistBuf == null)
                throw new InvalidOperationException("Strip was not built with tessellated polyline data.");
            if (vertexIndex < 0 || vertexIndex >= _polyDistBuf.Length)
                throw new ArgumentOutOfRangeException(nameof(vertexIndex));
            return _polyDistBuf[vertexIndex] / TotalLength;
        }

        /// <summary>
        /// Position and curve normal at <paramref name="u"/>; works for tessellated polylines and analytic <see cref="FromSegments"/> strips.
        /// </summary>
        public void EvaluatePositionAndNormalAtNormalizedArcLength(double u, out Vec2D position, out Vec2D normal)
        {
            if (HasPolylineSamplingData)
            {
                EvaluatePositionAndVertexNormalAtNormalizedArcLength(u, out position, out normal);
                return;
            }

            CurveVertex2D v = EvaluateAtNormalizedArcLength(u);
            position = v.Position;
            normal = v.Normal;
        }

        /// <summary>
        /// Unit tangent in the direction of increasing arc length (finite-difference; robust for analytic strips).
        /// </summary>
        public Vec2D TangentUnitAtNormalizedArcLength(double u)
        {
            double eps = Math.Max(TotalLength * 1e-10, 1e-9) / Math.Max(TotalLength, 1e-30);
            double ua = Math.Clamp(u - eps, 0, 1);
            double ub = Math.Clamp(u + eps, 0, 1);
            if (ub <= ua + 1e-15)
                ub = Math.Min(1, ua + eps);
            Vec2D pa = EvaluateAtNormalizedArcLength(ua).Position;
            Vec2D pb = EvaluateAtNormalizedArcLength(ub).Position;
            var t = pb - pa;
            double len = Math.Sqrt(t.X * t.X + t.Y * t.Y);
            if (len < 1e-30)
                return new Vec2D(1, 0);
            return new Vec2D(t.X / len, t.Y / len);
        }

        /// <summary>
        /// Normalized u at each segment start (0, and cumulative / total); deduplicated.</summary>
        public List<double> SegmentStartNormalizedUParameters()
        {
            double t = TotalLength;
            if (t < 1e-30)
                return new List<double>();
            var list = new List<double>(_segments.Length + 1);
            for (int i = 0; i <= _segments.Length; i++)
            {
                double u = _cumStart[i] / t;
                if (list.Count == 0 || Math.Abs(u - list[^1]) > 1e-14)
                    list.Add(u);
            }
            return list;
        }

        public CurveVertex2D EvaluateAtNormalizedArcLength(double u)
        {
            u = Math.Clamp(u, 0, 1);
            double d = u * TotalLength;
            int si = 0;
            for (int i = 0; i < _segments.Length; i++)
            {
                if (_cumStart[i + 1] <= d + 1e-12)
                    si = i;
                else
                    break;
            }
            double local = d - _cumStart[si];
            double len = _segments[si].Length();
            double t = len < 1e-30 ? 0 : Math.Clamp(local / len, 0, 1);
            return _segments[si].EvaluateVertex(t);
        }

        /// <summary>Position and interpolated vertex normal (tessellated polyline strips only).</summary>
        public void EvaluatePositionAndVertexNormalAtNormalizedArcLength(double u, out Vec2D position, out Vec2D normal)
        {
            if (_polyVertices == null || _polyNormals == null || _polyDistBuf == null)
                throw new InvalidOperationException("Use FromTessellatedPolyline for normal-aware evaluation.");

            u = Math.Clamp(u, 0, 1);
            double distance = u * TotalLength;
            position = LineStrip2D.EvaluateAtDistance(_polyVertices, _polyDistBuf, distance, out double idx, 0, _closedPolyline);
            int n = _polyVertices.Length;
            int i0 = (int)Math.Floor(idx + 1e-9);
            if (i0 < 0) i0 = 0;
            if (i0 >= n) i0 = n - 1;
            double w = idx - i0;
            if (w < 0) w = 0;
            if (w > 1) w = 1;
            int i1 = i0 + 1;
            if (_closedPolyline && i1 >= n)
                i1 = 0;
            else if (!_closedPolyline && i1 >= n)
                i1 = n - 1;
            Vec2D na = _polyNormals[i0];
            Vec2D nb = _polyNormals[i1];
            normal = new Vec2D(na.X * (1 - w) + nb.X * w, na.Y * (1 - w) + nb.Y * w);
            double nl = Math.Sqrt(normal.X * normal.X + normal.Y * normal.Y);
            if (nl > 1e-14)
                normal = new Vec2D(normal.X / nl, normal.Y / nl);
        }
    }
}
