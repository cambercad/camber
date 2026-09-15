using GeoCore;

namespace Curves
{
    /// <summary>
    /// Maps Clipper 0-based vertex ids (Convert Z-1) back to source curves.
    /// Vertex id i is the joint of the polyline edges around tessellated source
    /// vertex i; the offset of two curves that meet there must meet at that vertex's offset.
    /// </summary>
    public class OffsetSourceVertexMap
    {
        class PathRec
        {
            public int Start { get; set; }
            public int Count { get; set; }
            public bool Closed { get; set; }
        }

        readonly List<PathRec> _paths = new List<PathRec>();
        readonly List<int> _edgeSource = new List<int>();
        readonly List<bool> _joint = new List<bool>();
        readonly List<Vec2D> _pos = new List<Vec2D>();

        public int Count { get { return _edgeSource.Count; } }

        public void AddPath(List<Vec2D> polyline, List<int> edgeSource, List<bool> joint, bool closed)
        {
            if (polyline == null || polyline.Count == 0)
                return;

            int n = polyline.Count;
            var rec = new PathRec
            {
                Start = _edgeSource.Count,
                Count = n,
                Closed = closed,
            };
            _paths.Add(rec);

            for (int i = 0; i < n; i++)
            {
                _pos.Add(polyline[i]);
                _joint.Add(joint != null && i < joint.Count && joint[i]);
                int src = -1;
                if (edgeSource != null)
                {
                    if (closed)
                        src = i < edgeSource.Count ? edgeSource[i] : -1;
                    else if (i < n - 1)
                        src = i < edgeSource.Count ? edgeSource[i] : -1;
                }
                _edgeSource.Add(src);
            }
        }

        public bool IsValid(int id)
        {
            return id >= 0 && id < _edgeSource.Count;
        }

        public bool IsJoint(int id)
        {
            return IsValid(id) && _joint[id];
        }

        public Vec2D Position(int id)
        {
            return IsValid(id) ? _pos[id] : default;
        }

        public int EdgeSource(int id)
        {
            return IsValid(id) ? _edgeSource[id] : -1;
        }

        public bool TryGetPath(int id, out int pathIndex, out int local, out bool closed, out int count)
        {
            pathIndex = -1;
            local = -1;
            closed = false;
            count = 0;
            if (!IsValid(id))
                return false;
            for (int p = 0; p < _paths.Count; p++)
            {
                PathRec rec = _paths[p];
                if (id >= rec.Start && id < rec.Start + rec.Count)
                {
                    pathIndex = p;
                    local = id - rec.Start;
                    closed = rec.Closed;
                    count = rec.Count;
                    return true;
                }
            }
            return false;
        }

        public int AdjacentEdgeSource(int a, int b)
        {
            if (!TryGetPath(a, out int pa, out int la, out bool closed, out int n))
                return -1;
            if (!TryGetPath(b, out int pb, out int lb, out _, out _))
                return -1;
            if (pa != pb)
                return -1;
            if (closed)
            {
                if ((la + 1) % n == lb)
                    return _edgeSource[a];
                if ((lb + 1) % n == la)
                    return _edgeSource[b];
            }
            else
            {
                if (la + 1 == lb)
                    return _edgeSource[a];
                if (lb + 1 == la)
                    return _edgeSource[b];
            }
            return -1;
        }

        public int UniqueSourceBetween(int fromId, int toId)
        {
            int adj = AdjacentEdgeSource(fromId, toId);
            if (adj >= 0)
                return adj;
            if (!TryWalk(fromId, toId, out List<int> verts))
                return -1;
            int src = -1;
            for (int i = 0; i < verts.Count - 1; i++)
            {
                int edgeSrc = AdjacentEdgeSource(verts[i], verts[i + 1]);
                if (edgeSrc < 0)
                    return -1;
                if (src < 0)
                    src = edgeSrc;
                else if (src != edgeSrc)
                    return -1;
            }
            return src;
        }

        public double WalkLength(int fromId, int toId)
        {
            if (!TryWalk(fromId, toId, out List<int> verts))
                return 0;
            double len = 0;
            for (int i = 0; i < verts.Count - 1; i++)
                len += (_pos[verts[i + 1]] - _pos[verts[i]]).Length();
            return len;
        }

        public double WalkLengthTo(int fromId, int viaId, int toId)
        {
            return WalkLength(fromId, viaId);
        }

        bool TryWalk(int fromId, int toId, out List<int> verts)
        {
            verts = new List<int>();
            if (!TryGetPath(fromId, out int pa, out int la, out bool closed, out int n))
                return false;
            if (!TryGetPath(toId, out int pb, out int lb, out _, out _))
                return false;
            if (pa != pb)
                return false;
            PathRec rec = _paths[pa];
            List<int> forward = WalkLocals(la, lb, n, closed, +1);
            List<int> backward = WalkLocals(la, lb, n, closed, -1);
            List<int> walk = forward.Count <= backward.Count ? forward : backward;
            verts.Add(fromId);
            for (int i = 0; i < walk.Count; i++)
                verts.Add(rec.Start + walk[i]);
            verts.Add(toId);
            return true;
        }

        public bool AreAdjacent(int a, int b)
        {
            if (!TryGetPath(a, out int pa, out int la, out bool closed, out int n))
                return false;
            if (!TryGetPath(b, out int pb, out int lb, out _, out _))
                return false;
            if (pa != pb)
                return false;
            if (closed)
                return (la + 1) % n == lb || (lb + 1) % n == la;
            return la + 1 == lb || lb + 1 == la;
        }

        /// <summary>
        /// Source joints strictly between <paramref name="fromId"/> and <paramref name="toId"/>
        /// along the shorter path walk. Empty if the vertices are adjacent or on different paths.
        /// </summary>
        public List<int> JointsBetween(int fromId, int toId)
        {
            var result = new List<int>();
            if (!TryGetPath(fromId, out int pa, out int la, out bool closed, out int n))
                return result;
            if (!TryGetPath(toId, out int pb, out int lb, out _, out _))
                return result;
            if (pa != pb || fromId == toId)
                return result;

            PathRec rec = _paths[pa];
            List<int> forward = WalkLocals(la, lb, n, closed, +1);
            List<int> backward = WalkLocals(la, lb, n, closed, -1);
            List<int> walk = forward.Count <= backward.Count ? forward : backward;
            for (int i = 0; i < walk.Count; i++)
            {
                int id = rec.Start + walk[i];
                if (_joint[id])
                    result.Add(id);
            }
            return result;
        }

        public int SourceAtVertex(int id, Vec2D edgeDir, IReadOnlyList<Curve2D> sources)
        {
            int outgoing = EdgeSource(id);
            int incoming = IncomingEdgeSource(id);
            double outAlign = AlignToSource(outgoing, edgeDir, sources);
            double inAlign = AlignToSource(incoming, edgeDir, sources);
            if (outAlign >= inAlign && outgoing >= 0)
                return outgoing;
            if (incoming >= 0)
                return incoming;
            return outgoing;
        }

        int IncomingEdgeSource(int id)
        {
            if (!TryGetPath(id, out int p, out int local, out bool closed, out int n))
                return -1;
            PathRec rec = _paths[p];
            if (closed)
            {
                int prev = (local + n - 1) % n;
                return _edgeSource[rec.Start + prev];
            }
            if (local <= 0)
                return -1;
            return _edgeSource[rec.Start + local - 1];
        }

        static List<int> WalkLocals(int from, int to, int n, bool closed, int step)
        {
            var locals = new List<int>();
            int cur = from;
            int guard = 0;
            while (guard++ < n)
            {
                if (closed)
                    cur = (cur + step + n) % n;
                else
                {
                    cur += step;
                    if (cur < 0 || cur >= n)
                        break;
                }
                if (cur == to)
                    break;
                locals.Add(cur);
            }
            return locals;
        }

        static double AlignToSource(int sourceIndex, Vec2D edgeDir, IReadOnlyList<Curve2D> sources)
        {
            if (sourceIndex < 0 || sources == null || sourceIndex >= sources.Count)
                return -1;
            Vec2D t = sources[sourceIndex].EndPosition - sources[sourceIndex].StartPosition;
            double tl = t.Length();
            double el = edgeDir.Length();
            if (tl < 1e-12 || el < 1e-12)
                return -1;
            return Math.Abs((t.X * edgeDir.X + t.Y * edgeDir.Y) / (tl * el));
        }
    }
}
