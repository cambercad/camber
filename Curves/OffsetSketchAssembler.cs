using GeoCore;
using GeoMeta;

namespace Curves
{
    public enum OffsetSketchPieceSide
    {
        None = 0,
        In = 1,
        Out = 2,
        Cap = 3,
    }

    public class OffsetSketchPiece
    {
        public int LoopIndex { get; set; }
        public int SourceIndex { get; set; }
        public OffsetSketchPieceSide Side { get; set; }
        public string AssignedName { get; set; }
        public List<Vec2D> Points { get; set; }

        public OffsetSketchPiece()
        {
            LoopIndex = 0;
            SourceIndex = -1;
            Side = OffsetSketchPieceSide.None;
            AssignedName = null;
            Points = new List<Vec2D>();
        }

        public bool IsNamedOffset
        {
            get
            {
                if (!string.IsNullOrEmpty(AssignedName))
                    return true;
                return SourceIndex >= 0 && Side != OffsetSketchPieceSide.None;
            }
        }
    }

    public interface IOffsetSketchPieces
    {
        void Update();
        IReadOnlyList<Curve2D> Sources { get; }
        IReadOnlyList<OffsetSketchPiece> Pieces { get; }
    }

    /// <summary>
    /// Splits Clipper offset polylines using source-vertex ids carried in Clipper Z.
    /// Two source curves that meet at a vertex produce offset pieces that meet at
    /// that vertex's offset. End caps (same source, in and out at a free endpoint)
    /// take <c>start_cap</c> / <c>end_cap</c>. Remaining fragments (corner joins)
    /// are named <c>cap[a,b]</c> from the two named curves they connect.
    /// </summary>
    public static class OffsetSketchAssembler
    {
        public static long SourceFingerprint(IReadOnlyList<Curve2D> sources)
        {
            unchecked
            {
                long h = 17;
                if (sources == null)
                    return h;
                for (int i = 0; i < sources.Count; i++)
                {
                    Vec2D a = sources[i].StartPosition;
                    Vec2D b = sources[i].EndPosition;
                    h = h * 31 + BitConverter.DoubleToInt64Bits(a.X);
                    h = h * 31 + BitConverter.DoubleToInt64Bits(a.Y);
                    h = h * 31 + BitConverter.DoubleToInt64Bits(b.X);
                    h = h * 31 + BitConverter.DoubleToInt64Bits(b.Y);
                }
                return h;
            }
        }

        public static List<OffsetSketchPiece> Assemble(
            IReadOnlyList<Curve2D> sources,
            IReadOnlyList<List<Vec2D>> loops,
            bool closed,
            double offset = 0)
        {
            return Assemble(sources, loops, null, null, closed, offset);
        }

        public static List<OffsetSketchPiece> Assemble(
            IReadOnlyList<Curve2D> sources,
            IReadOnlyList<List<Vec2D>> loops,
            IReadOnlyList<List<int>> vertexIds,
            OffsetSourceVertexMap map,
            bool closed,
            double offset = 0)
        {
            var pieces = new List<OffsetSketchPiece>();
            if (sources == null || loops == null)
                return pieces;

            for (int loopIndex = 0; loopIndex < loops.Count; loopIndex++)
            {
                List<Vec2D> loop = loops[loopIndex];
                if (loop == null || loop.Count < 2)
                    continue;
                List<int> tags = vertexIds != null && loopIndex < vertexIds.Count
                    ? new List<int>(vertexIds[loopIndex])
                    : null;
                if (tags != null && map != null)
                    InsertMissingJoints(loop, tags, map, closed);
                pieces.AddRange(SplitLoop(sources, loop, tags, map, closed, loopIndex));
            }

            AssignEndCapsFromNeighbors(pieces, sources);
            AssignJoinCapsFromNeighbors(pieces, sources);
            return pieces;
        }

        public static List<OffsetSampledCurve2D> CreateSampledCurves(
            IOffsetSketchPieces owner,
            CurveFlags flags = CurveFlags.None)
        {
            var result = new List<OffsetSampledCurve2D>();
            if (owner == null)
                return result;

            IReadOnlyList<OffsetSketchPiece> pieces = owner.Pieces;
            IReadOnlyList<Curve2D> sources = owner.Sources;
            if (pieces == null)
                return result;

            int[] ranks = NameOccurrencesByLength(pieces, sources);
            for (int i = 0; i < pieces.Count; i++)
            {
                var curve = new OffsetSampledCurve2D(owner, i, flags);
                string name = NameForPiece(pieces[i], sources, ranks[i]);
                if (!string.IsNullOrEmpty(name))
                    curve.Name = name;
                result.Add(curve);
            }
            return result;
        }

        public static string NameForPiece(
            OffsetSketchPiece piece,
            IReadOnlyList<Curve2D> sources,
            int occurrence)
        {
            if (piece == null)
                return null;

            if (!string.IsNullOrEmpty(piece.AssignedName))
            {
                if (occurrence <= 0)
                    return piece.AssignedName;
                return piece.AssignedName + "_" + (occurrence + 1).ToString();
            }

            if (!piece.IsNamedOffset ||
                piece.SourceIndex < 0 || sources == null || piece.SourceIndex >= sources.Count)
                return null;

            string sourceName = sources[piece.SourceIndex].Name;
            if (string.IsNullOrEmpty(sourceName))
                return null;

            if (piece.Side == OffsetSketchPieceSide.Cap)
            {
                return EntityNaming.FormatSketchOffsetEndCap(
                    sourceName, CapIsAtSourceStart(piece, sources), occurrence);
            }

            return EntityNaming.FormatSketchOffsetCurve(
                sourceName, piece.Side == OffsetSketchPieceSide.Out, occurrence);
        }

        public static int[] NameOccurrencesByLength(
            IReadOnlyList<OffsetSketchPiece> pieces,
            IReadOnlyList<Curve2D> sources)
        {
            int count = pieces == null ? 0 : pieces.Count;
            var ranks = new int[count];
            var groups = new Dictionary<string, List<int>>();
            for (int i = 0; i < count; i++)
            {
                var piece = pieces[i];
                if (piece == null || !piece.IsNamedOffset)
                    continue;
                string key = NameGroupKey(piece, sources);
                if (string.IsNullOrEmpty(key))
                    continue;
                if (!groups.TryGetValue(key, out List<int> list))
                {
                    list = new List<int>();
                    groups[key] = list;
                }
                list.Add(i);
            }

            foreach (var list in groups.Values)
            {
                list.Sort((a, b) => PolylineLength(pieces[b].Points).CompareTo(PolylineLength(pieces[a].Points)));
                for (int r = 0; r < list.Count; r++)
                    ranks[list[r]] = r;
            }

            return ranks;
        }

        static string NameGroupKey(OffsetSketchPiece piece, IReadOnlyList<Curve2D> sources)
        {
            if (!string.IsNullOrEmpty(piece.AssignedName))
                return "join:" + piece.AssignedName;
            if (piece.Side == OffsetSketchPieceSide.Cap)
                return "cap:" + piece.SourceIndex + ":" + (CapIsAtSourceStart(piece, sources) ? "s" : "e");
            return "side:" + piece.SourceIndex + ":" + ((int)piece.Side).ToString();
        }

        static double PolylineLength(List<Vec2D> pts)
        {
            if (pts == null || pts.Count < 2)
                return 0;
            double len = 0;
            for (int i = 1; i < pts.Count; i++)
                len += (pts[i] - pts[i - 1]).Length();
            return len;
        }

        /// <summary>
        /// An unnamed fragment whose endpoints join in-offset and out-offset of the
        /// same source is that source's end cap (the free endpoint of an open path).
        /// </summary>
        static void AssignEndCapsFromNeighbors(
            List<OffsetSketchPiece> pieces,
            IReadOnlyList<Curve2D> sources)
        {
            if (pieces == null || pieces.Count == 0)
                return;

            foreach (List<int> loop in GroupLoopIndices(pieces).Values)
            {
                int n = loop.Count;
                if (n < 3)
                    continue;
                bool closed = LoopIsClosed(pieces[loop[0]], pieces[loop[n - 1]]);
                if (!closed)
                    continue;

                for (int i = 0; i < n; i++)
                {
                    OffsetSketchPiece piece = pieces[loop[i]];
                    if (piece.IsNamedOffset)
                        continue;

                    OffsetSketchPiece prev = pieces[loop[(i - 1 + n) % n]];
                    OffsetSketchPiece next = pieces[loop[(i + 1) % n]];
                    if (!SharesEndpoint(piece, prev) || !SharesEndpoint(piece, next))
                        continue;
                    if (!IsSourceSideName(prev) || !IsSourceSideName(next))
                        continue;
                    if (prev.SourceIndex < 0 || prev.SourceIndex != next.SourceIndex)
                        continue;
                    if (sources != null && prev.SourceIndex >= sources.Count)
                        continue;
                    if (prev.Side == next.Side)
                        continue;
                    if (prev.Side != OffsetSketchPieceSide.In && prev.Side != OffsetSketchPieceSide.Out)
                        continue;
                    if (next.Side != OffsetSketchPieceSide.In && next.Side != OffsetSketchPieceSide.Out)
                        continue;

                    piece.SourceIndex = prev.SourceIndex;
                    piece.Side = OffsetSketchPieceSide.Cap;
                }
            }
        }

        /// <summary>
        /// Remaining unnamed fragments (round/square/miter corner joins) take
        /// <c>cap[leftName,rightName]</c> from the two named curves they connect.
        /// </summary>
        static void AssignJoinCapsFromNeighbors(
            List<OffsetSketchPiece> pieces,
            IReadOnlyList<Curve2D> sources)
        {
            if (pieces == null || pieces.Count == 0)
                return;

            int[] ranks = NameOccurrencesByLength(pieces, sources);
            foreach (List<int> loop in GroupLoopIndices(pieces).Values)
            {
                int n = loop.Count;
                if (n < 3)
                    continue;
                bool closed = LoopIsClosed(pieces[loop[0]], pieces[loop[n - 1]]);

                for (int i = 0; i < n; i++)
                {
                    OffsetSketchPiece piece = pieces[loop[i]];
                    if (piece.IsNamedOffset)
                        continue;

                    int prevLocal = i - 1;
                    int nextLocal = i + 1;
                    if (closed)
                    {
                        prevLocal = (i - 1 + n) % n;
                        nextLocal = (i + 1) % n;
                    }
                    else if (prevLocal < 0 || nextLocal >= n)
                        continue;

                    int prevIndex = loop[prevLocal];
                    int nextIndex = loop[nextLocal];
                    OffsetSketchPiece prev = pieces[prevIndex];
                    OffsetSketchPiece next = pieces[nextIndex];
                    if (!SharesEndpoint(piece, prev) || !SharesEndpoint(piece, next))
                        continue;
                    if (!IsSourceDerivedName(prev) || !IsSourceDerivedName(next))
                        continue;

                    string nameA = NameForPiece(prev, sources, ranks[prevIndex]);
                    string nameB = NameForPiece(next, sources, ranks[nextIndex]);
                    if (string.IsNullOrEmpty(nameA) || string.IsNullOrEmpty(nameB))
                        continue;

                    piece.AssignedName = EntityNaming.FormatSketchOffsetJoinCap(nameA, nameB);
                }
            }
        }

        static Dictionary<int, List<int>> GroupLoopIndices(List<OffsetSketchPiece> pieces)
        {
            var loops = new Dictionary<int, List<int>>();
            for (int i = 0; i < pieces.Count; i++)
            {
                int loop = pieces[i].LoopIndex;
                if (!loops.TryGetValue(loop, out List<int> list))
                {
                    list = new List<int>();
                    loops[loop] = list;
                }
                list.Add(i);
            }
            return loops;
        }

        static bool IsSourceSideName(OffsetSketchPiece piece)
        {
            return piece != null
                && piece.SourceIndex >= 0
                && (piece.Side == OffsetSketchPieceSide.In || piece.Side == OffsetSketchPieceSide.Out);
        }

        static bool IsSourceDerivedName(OffsetSketchPiece piece)
        {
            return piece != null
                && string.IsNullOrEmpty(piece.AssignedName)
                && piece.SourceIndex >= 0
                && piece.Side != OffsetSketchPieceSide.None;
        }

        static bool LoopIsClosed(OffsetSketchPiece first, OffsetSketchPiece last)
        {
            if (first == null || last == null || first.Points == null || last.Points == null)
                return false;
            if (first.Points.Count == 0 || last.Points.Count == 0)
                return false;
            return Vec2DOps.DistanceSquared(first.Points[0], last.Points[last.Points.Count - 1]) <= 1e-16;
        }

        static bool SharesEndpoint(OffsetSketchPiece a, OffsetSketchPiece b)
        {
            if (a == null || b == null || a.Points == null || b.Points == null)
                return false;
            if (a.Points.Count == 0 || b.Points.Count == 0)
                return false;
            Vec2D a0 = a.Points[0];
            Vec2D a1 = a.Points[a.Points.Count - 1];
            Vec2D b0 = b.Points[0];
            Vec2D b1 = b.Points[b.Points.Count - 1];
            const double tol = 1e-16;
            return Vec2DOps.DistanceSquared(a0, b0) <= tol
                || Vec2DOps.DistanceSquared(a0, b1) <= tol
                || Vec2DOps.DistanceSquared(a1, b0) <= tol
                || Vec2DOps.DistanceSquared(a1, b1) <= tol;
        }

        static bool CapIsAtSourceStart(OffsetSketchPiece piece, IReadOnlyList<Curve2D> sources)
        {
            if (piece == null || piece.Points == null || piece.Points.Count == 0)
                return true;
            if (sources == null || piece.SourceIndex < 0 || piece.SourceIndex >= sources.Count)
                return true;

            Curve2D source = sources[piece.SourceIndex];
            Vec2D start = source.StartPosition;
            Vec2D end = source.EndPosition;
            double bestStart = double.MaxValue;
            double bestEnd = double.MaxValue;
            for (int i = 0; i < piece.Points.Count; i++)
            {
                double d0 = Vec2DOps.DistanceSquared(piece.Points[i], start);
                double d1 = Vec2DOps.DistanceSquared(piece.Points[i], end);
                if (d0 < bestStart)
                    bestStart = d0;
                if (d1 < bestEnd)
                    bestEnd = d1;
            }
            return bestStart <= bestEnd;
        }

        /// <summary>
        /// Clipper may still drop a collinear joint. Re-insert the offset of that
        /// source vertex on the fused edge so the two source offsets meet there.
        /// </summary>
        static void InsertMissingJoints(List<Vec2D> pts, List<int> tags, OffsetSourceVertexMap map, bool closed)
        {
            if (pts == null || tags == null || map == null)
                return;
            while (tags.Count < pts.Count)
                tags.Add(-1);
            if (tags.Count > pts.Count)
                tags.RemoveRange(pts.Count, tags.Count - pts.Count);

            int n = pts.Count;
            int edgeCount = closed ? n : n - 1;
            if (edgeCount < 1)
                return;
            var inserts = new List<JointInsert>();
            for (int e = 0; e < edgeCount; e++)
            {
                int za = tags[e];
                int zbIndex = closed ? (e + 1) % n : e + 1;
                int zb = tags[zbIndex];
                if (!map.IsValid(za) || !map.IsValid(zb) || za == zb)
                    continue;
                List<int> joints = map.JointsBetween(za, zb);
                if (joints.Count == 0)
                    continue;
                double total = map.WalkLength(za, zb);
                if (total <= 1e-12)
                    continue;
                Vec2D a = pts[e];
                Vec2D b = pts[zbIndex];
                for (int j = 0; j < joints.Count; j++)
                {
                    int id = joints[j];
                    double t = map.WalkLength(za, id) / total;
                    if (t <= 1e-6 || t >= 1.0 - 1e-6)
                        continue;
                    inserts.Add(new JointInsert { Edge = e, T = t, Point = a + t * (b - a), VertexId = id });
                }
            }

            inserts.Sort((u, v) =>
            {
                int c = v.Edge.CompareTo(u.Edge);
                if (c != 0)
                    return c;
                return v.T.CompareTo(u.T);
            });

            for (int i = 0; i < inserts.Count; i++)
            {
                JointInsert ins = inserts[i];
                if (ins.Edge < 0 || ins.Edge >= pts.Count)
                    continue;
                Vec2D a = pts[ins.Edge];
                Vec2D b = pts[(ins.Edge + 1) % pts.Count];
                if (Vec2DOps.DistanceSquared(ins.Point, a) <= 1e-24 ||
                    Vec2DOps.DistanceSquared(ins.Point, b) <= 1e-24)
                    continue;
                pts.Insert(ins.Edge + 1, ins.Point);
                tags.Insert(ins.Edge + 1, ins.VertexId);
            }
        }

        class JointInsert
        {
            public int Edge { get; set; }
            public double T { get; set; }
            public Vec2D Point { get; set; }
            public int VertexId { get; set; }
        }

        static List<OffsetSketchPiece> SplitLoop(
            IReadOnlyList<Curve2D> sources,
            List<Vec2D> loop,
            List<int> tags,
            OffsetSourceVertexMap map,
            bool closed,
            int loopIndex)
        {
            int n = loop.Count;
            int edgeCount = closed ? n : n - 1;
            var sourceOf = new int[edgeCount];
            var sideOf = new OffsetSketchPieceSide[edgeCount];
            for (int i = 0; i < edgeCount; i++)
            {
                int za = TagAt(tags, i);
                int zb = TagAt(tags, (i + 1) % n);
                Vec2D a = loop[i];
                Vec2D b = loop[(i + 1) % n];
                int src = SourceOfEdge(sources, map, za, zb, b - a);
                sourceOf[i] = src;
                sideOf[i] = src >= 0 && src < sources.Count
                    ? SideOf(sources[src], a, b)
                    : OffsetSketchPieceSide.None;
            }

            var pieces = new List<OffsetSketchPiece>();
            int start = 0;
            while (start < edgeCount)
            {
                int end = start;
                while (end + 1 < edgeCount &&
                       sourceOf[end + 1] == sourceOf[start] &&
                       sideOf[end + 1] == sideOf[start])
                    end++;

                pieces.Add(MakePiece(loop, closed, loopIndex, sourceOf[start], sideOf[start], start, end));
                start = end + 1;
            }

            if (closed && pieces.Count >= 2 &&
                pieces[0].SourceIndex == pieces[^1].SourceIndex &&
                pieces[0].Side == pieces[^1].Side)
            {
                var merged = new List<Vec2D>(pieces[^1].Points);
                if (merged.Count > 0 && pieces[0].Points.Count > 0 &&
                    Vec2DOps.DistanceSquared(merged[^1], pieces[0].Points[0]) <= 1e-24)
                    merged.RemoveAt(merged.Count - 1);
                merged.AddRange(pieces[0].Points);
                pieces[^1].Points = merged;
                pieces.RemoveAt(0);
            }

            return pieces;
        }

        static int TagAt(List<int> tags, int i)
        {
            if (tags == null || i < 0 || i >= tags.Count)
                return -1;
            return tags[i];
        }

        static int SourceOfEdge(
            IReadOnlyList<Curve2D> sources,
            OffsetSourceVertexMap map,
            int za,
            int zb,
            Vec2D edge)
        {
            if (map == null)
                return -1;
            if (za >= 0 && za == zb)
                return -1;

            int src = -1;
            if (map.IsValid(za) && map.IsValid(zb))
            {
                src = map.AdjacentEdgeSource(za, zb);
                if (src < 0)
                    src = map.UniqueSourceBetween(za, zb);
            }
            if (src < 0 && map.IsValid(za) && !map.IsValid(zb))
                src = map.SourceAtVertex(za, edge, sources);
            if (src < 0 && map.IsValid(zb) && !map.IsValid(za))
                src = map.SourceAtVertex(zb, edge, sources);
            return src;
        }

        static OffsetSketchPieceSide SideOf(Curve2D source, Vec2D edgeStart, Vec2D edgeEnd)
        {
            Vec2D point = 0.5 * (edgeStart + edgeEnd);
            if (!TryClosestOnCurve(source, point, out Vec2D closest, out Vec2D tangent, out double distSq))
                return OffsetSketchPieceSide.None;
            double tLen = tangent.Length();
            if (tLen <= 1e-12 || distSq <= 1e-24)
                return OffsetSketchPieceSide.None;
            Vec2D tu = tangent * (1.0 / tLen);
            Vec2D delta = point - closest;
            double cross = tu.X * delta.Y - tu.Y * delta.X;
            return cross < 0 ? OffsetSketchPieceSide.Out : OffsetSketchPieceSide.In;
        }

        static OffsetSketchPiece MakePiece(
            List<Vec2D> loop,
            bool closed,
            int loopIndex,
            int sourceIndex,
            OffsetSketchPieceSide side,
            int firstEdge,
            int lastEdge)
        {
            int n = loop.Count;
            var points = new List<Vec2D>();
            points.Add(loop[firstEdge]);
            for (int e = firstEdge; e <= lastEdge; e++)
                points.Add(loop[(e + 1) % n]);

            if (!closed && lastEdge == n - 2 &&
                Vec2DOps.DistanceSquared(points[^1], loop[n - 1]) > 1e-24)
                points.Add(loop[n - 1]);

            return new OffsetSketchPiece
            {
                LoopIndex = loopIndex,
                SourceIndex = sourceIndex,
                Side = side,
                Points = points,
            };
        }

        static bool TryClosestOnCurve(
            Curve2D curve,
            Vec2D point,
            out Vec2D closest,
            out Vec2D tangent,
            out double distSq)
        {
            closest = default;
            tangent = default;
            distSq = double.MaxValue;
            if (curve == null)
                return false;

            if (curve is Line2D line)
            {
                Vec2D a = line.StartPosition;
                Vec2D b = line.EndPosition;
                Vec2D ab = b - a;
                double lenSq = ab.LengthSquared();
                if (lenSq <= 1e-24)
                    return false;
                double t = Vec2DOps.Dot(point - a, ab) / lenSq;
                if (t < 0) t = 0;
                else if (t > 1) t = 1;
                closest = a + t * ab;
                tangent = ab;
                distSq = Vec2DOps.DistanceSquared(point, closest);
                return true;
            }

            List<CurveVertex2D> samples = curve.Tessellate(24);
            if (samples == null || samples.Count < 2)
                return false;

            for (int i = 0; i < samples.Count - 1; i++)
            {
                Vec2D a = samples[i].Position;
                Vec2D b = samples[i + 1].Position;
                Vec2D ab = b - a;
                double lenSq = ab.LengthSquared();
                Vec2D c;
                Vec2D tan;
                if (lenSq <= 1e-24)
                {
                    c = a;
                    tan = samples[i].Tangent;
                }
                else
                {
                    double t = Vec2DOps.Dot(point - a, ab) / lenSq;
                    if (t < 0) t = 0;
                    else if (t > 1) t = 1;
                    c = a + t * ab;
                    tan = ab;
                }

                double d2 = Vec2DOps.DistanceSquared(point, c);
                if (d2 < distSq)
                {
                    distSq = d2;
                    closest = c;
                    tangent = tan;
                }
            }

            return distSq < double.MaxValue;
        }
    }
}
