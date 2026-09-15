using System.Linq;
using Curves;
using Geo.NurbsConstruction;
using GeoCore;
using NURBS;

namespace Geo.BRep
{
    /// <summary>
    /// Trim curves for B-rep export. Mesh chords are grouped into CAD strips
    /// (iso-U / iso-V patch sides, or runs between vertices where 3+ faces meet)
    /// so STEP/IGES write one edge per Camber strip instead of one edge per tessellation chord.
    /// </summary>
    public readonly struct BRepTrimSegment
    {
        public BRepTrimSegment(BSplineCurve curve, Vec2D uv0, Vec2D uv1)
        {
            Curve = curve;
            Uv0 = uv0;
            Uv1 = uv1;
        }

        public BSplineCurve Curve { get; }
        public Vec2D Uv0 { get; }
        public Vec2D Uv1 { get; }
    }

    public static class BRepTrimCurves
    {
        public const double MinSegmentLength = 1e-9;
        public const double CollinearAngleTolerance = 1e-4;
        public const double ArcRadiusTolerance = 1e-4;
        public const double ArcPlaneTolerance = 1e-4;

        public static BSplineCurve ChordSegment(Vec3D p0, Vec3D p1)
        {
            return Curve3DToBSpline.LineToBSpline(new Line3D(p0, p1));
        }

        public static List<BSplineCurve> SegmentsFromLoop(BRepTrimLoop loop) =>
            SegmentsFromLoopWithUv(loop).Select(s => s.Curve).ToList();

        public static List<BRepTrimSegment> SegmentsFromLoopWithUv(BRepTrimLoop loop, bool mergeCollinear = true, bool fitArcs = true)
        {
            var segments = new List<BRepTrimSegment>();
            if (loop.WorldPoints == null || loop.WorldPoints.Count < 2)
                return segments;

            var points = loop.WorldPoints;
            int i = 0;
            while (i < points.Count - 1)
            {
                // Prefer mesh-collinear runs as lines. Arc-fit only after that, and only when
                // interior turning angles stay smooth — otherwise a square's four corners
                // lie on a circle and would be exported as one CIRCLE.
                if (mergeCollinear)
                {
                    int lineEnd = TryFindLineRun(points, i);
                    if (lineEnd > i + 1)
                    {
                        segments.Add(new BRepTrimSegment(
                            ChordSegment(points[i], points[lineEnd]), UvAt(loop, i), UvAt(loop, lineEnd)));
                        i = lineEnd;
                        continue;
                    }
                }

                if (fitArcs)
                {
                    int arcEnd = TryFindArcRun(points, i, out var arcCurve);
                    if (arcEnd > i + 1 && arcCurve != null)
                    {
                        segments.Add(new BRepTrimSegment(arcCurve, UvAt(loop, i), UvAt(loop, arcEnd)));
                        i = arcEnd;
                        continue;
                    }
                }

                segments.Add(new BRepTrimSegment(
                    ChordSegment(points[i], points[i + 1]), UvAt(loop, i), UvAt(loop, i + 1)));
                i++;
            }

            return segments;
        }

        /// <summary>Chord-only trim segments; 3D lines align with UV lines for STEP pcurves.</summary>
        public static List<BRepTrimSegment> ChordSegmentsFromLoopWithUv(BRepTrimLoop loop) =>
            SegmentsFromLoopWithUv(loop, mergeCollinear: false, fitArcs: false);

        private static Vec2D UvAt(BRepTrimLoop loop, int index)
        {
            if (loop.UvPoints != null && index < loop.UvPoints.Count)
                return loop.UvPoints[index];
            return new Vec2D(0, 0);
        }

        public static void WalkLoopStrips(
            BRepSolid solid,
            BRepTrimLoop loop,
            Action<BRepEdge, int, int> onStrip)
        {
            if (loop == null || loop.EdgeIds == null || loop.EdgeIds.Count == 0)
                return;
            if (loop.WorldPoints == null || loop.WorldPoints.Count < 2)
                return;

            int p = 0;
            for (int i = 0; i < loop.EdgeIds.Count; i++)
            {
                int n = 2;
                if (loop.EdgePointCounts != null && i < loop.EdgePointCounts.Count)
                    n = loop.EdgePointCounts[i];
                if (n < 2)
                    continue;
                if (p + n - 1 >= loop.WorldPoints.Count)
                    break;

                var edge = solid.Edges[loop.EdgeIds[i]];
                onStrip(edge, p, n);
                p += n - 1;
            }
        }

        public static void CollectStripPoints(
            BRepTrimLoop loop,
            int startIndex,
            int sampleCount,
            List<Vec3D> world,
            List<Vec2D> uv)
        {
            world.Clear();
            uv.Clear();
            for (int k = 0; k < sampleCount; k++)
            {
                int idx = startIndex + k;
                var p = loop.WorldPoints[idx];
                if (world.Count > 0 && (p - world[world.Count - 1]).Length() < MinSegmentLength)
                    continue;
                world.Add(p);
                uv.Add(UvAt(loop, idx));
            }
        }

        public static void PopulateSolidEdges(BRepSolid solid)
        {
            if (solid.Edges.Count > 0)
                return;

            const double tol = 1e-6;
            var vertexMap = new List<BRepVertex>();

            int GetOrCreateVertex(Vec3D p)
            {
                for (int i = 0; i < vertexMap.Count; i++)
                {
                    if ((vertexMap[i].Position - p).Length() < tol)
                        return i;
                }
                int id = vertexMap.Count;
                vertexMap.Add(new BRepVertex { Id = id, Position = p });
                return id;
            }

            var loopVids = new Dictionary<BRepTrimLoop, int[]>();
            foreach (var face in solid.Shell.Faces)
            {
                foreach (var loop in face.Loops)
                {
                    if (loop.WorldPoints == null || loop.WorldPoints.Count < 2)
                        continue;
                    var vids = new int[loop.WorldPoints.Count];
                    for (int i = 0; i < loop.WorldPoints.Count; i++)
                        vids[i] = GetOrCreateVertex(loop.WorldPoints[i]);
                    loopVids[loop] = vids;
                }
            }

            var facesAtVertex = new HashSet<int>[vertexMap.Count];
            for (int i = 0; i < facesAtVertex.Length; i++)
                facesAtVertex[i] = new HashSet<int>();
            foreach (var face in solid.Shell.Faces)
            {
                foreach (var loop in face.Loops)
                {
                    if (!loopVids.TryGetValue(loop, out var vids))
                        continue;
                    for (int i = 0; i < vids.Length; i++)
                        facesAtVertex[vids[i]].Add(face.Id);
                }
            }

            var faceCount = new int[vertexMap.Count];
            for (int i = 0; i < faceCount.Length; i++)
                faceCount[i] = facesAtVertex[i].Count;

            var globalCorners = new HashSet<int>();
            foreach (var face in solid.Shell.Faces)
            {
                foreach (var loop in face.Loops)
                {
                    if (!loopVids.TryGetValue(loop, out var vids) || vids.Length < 2)
                        continue;
                    bool closed = (loop.WorldPoints[0] - loop.WorldPoints[loop.WorldPoints.Count - 1]).Length() < tol;
                    bool[] local = CornerFlags(loop, vids, faceCount, closed);
                    int n = closed ? loop.WorldPoints.Count - 1 : loop.WorldPoints.Count;
                    for (int i = 0; i < n && i < local.Length; i++)
                    {
                        if (local[i])
                            globalCorners.Add(vids[i]);
                    }
                }
            }

            var edgeByChords = new Dictionary<long, int>();

            foreach (var face in solid.Shell.Faces)
            {
                foreach (var loop in face.Loops)
                {
                    loop.EdgeIds.Clear();
                    loop.EdgePointCounts.Clear();
                    if (!loopVids.TryGetValue(loop, out var vids) || vids.Length < 2)
                        continue;

                    bool closed = (loop.WorldPoints[0] - loop.WorldPoints[loop.WorldPoints.Count - 1]).Length() < tol;
                    bool[] corners = ApplyGlobalCorners(loop, loopVids[loop], globalCorners, closed);
                    int firstCorner = FirstCorner(corners);
                    if (closed && firstCorner > 0)
                    {
                        RotateClosedLoopTo(loop, firstCorner);
                        vids = new int[loop.WorldPoints.Count];
                        for (int i = 0; i < loop.WorldPoints.Count; i++)
                            vids[i] = GetOrCreateVertex(loop.WorldPoints[i]);
                        loopVids[loop] = vids;
                        corners = ApplyGlobalCorners(loop, vids, globalCorners, closed: true);
                    }

                    foreach (var range in StripRanges(loop, corners, closed))
                    {
                        int start = range.Item1;
                        int end = range.Item2;
                        int sampleCount = end - start + 1;
                        if (sampleCount < 2)
                            continue;

                        int v0 = vids[start];
                        int v1 = vids[end];
                        long key = ChordSetKey(vids, start, end);
                        if (key == 0)
                            continue;

                        if (edgeByChords.TryGetValue(key, out int existing))
                        {
                            loop.EdgeIds.Add(existing);
                            loop.EdgePointCounts.Add(sampleCount);
                            continue;
                        }

                        var world = new List<Vec3D>();
                        var uv = new List<Vec2D>();
                        CollectStripPoints(loop, start, sampleCount, world, uv);
                        if (world.Count < 2)
                            continue;

                        var curve = PolylineToBSpline(world);
                        if (curve == null)
                            continue;
                        var p0 = curve.EvaluateUniform(0);
                        var p1 = curve.EvaluateUniform(1);
                        if ((p1 - p0).Length() < tol && v0 != v1)
                            continue;

                        int edgeId = solid.Edges.Count;
                        solid.Edges.Add(new BRepEdge
                        {
                            Id = edgeId,
                            Curve = curve,
                            StartVertexId = v0,
                            EndVertexId = v1
                        });
                        edgeByChords[key] = edgeId;
                        loop.EdgeIds.Add(edgeId);
                        loop.EdgePointCounts.Add(sampleCount);
                    }
                }
            }

            solid.Vertices = CompactVertices(vertexMap, solid.Edges);
        }

        private static List<BRepVertex> CompactVertices(List<BRepVertex> vertexMap, List<BRepEdge> edges)
        {
            var used = new bool[vertexMap.Count];
            foreach (var edge in edges)
            {
                if (edge.StartVertexId >= 0 && edge.StartVertexId < used.Length)
                    used[edge.StartVertexId] = true;
                if (edge.EndVertexId >= 0 && edge.EndVertexId < used.Length)
                    used[edge.EndVertexId] = true;
            }

            var compact = new List<BRepVertex>();
            var remap = new int[vertexMap.Count];
            for (int i = 0; i < vertexMap.Count; i++)
            {
                if (!used[i])
                {
                    remap[i] = -1;
                    continue;
                }
                remap[i] = compact.Count;
                compact.Add(new BRepVertex { Id = compact.Count, Position = vertexMap[i].Position });
            }

            foreach (var edge in edges)
            {
                edge.StartVertexId = remap[edge.StartVertexId];
                edge.EndVertexId = remap[edge.EndVertexId];
            }

            return compact;
        }

        public static BSplineCurve PolylineToBSpline(IReadOnlyList<Vec3D> points)
        {
            return StripToBSpline(points, polyline: true);
        }

        /// <summary>
        /// IGES importers explode degree-1 polylines into one edge per span.
        /// Use a cubic control polygon through the samples so each strip stays one curve.
        /// </summary>
        public static BSplineCurve SmoothStripToBSpline(IReadOnlyList<Vec3D> points)
        {
            return StripToBSpline(points, polyline: false);
        }

        public static bool IsCollinearUv(IReadOnlyList<Vec2D> uv)
        {
            if (uv == null || uv.Count < 2)
                return true;
            var pts = new Vec3D[uv.Count];
            for (int i = 0; i < uv.Count; i++)
                pts[i] = new Vec3D(uv[i].X, uv[i].Y, 0);
            return IsCollinearRun(pts);
        }

        private static BSplineCurve StripToBSpline(IReadOnlyList<Vec3D> points, bool polyline)
        {
            if (points == null || points.Count < 2)
                return null;
            if (points.Count == 2 || IsCollinearRun(points))
                return ChordSegment(points[0], points[points.Count - 1]);
            var pts = new Vec3D[points.Count];
            for (int i = 0; i < points.Count; i++)
                pts[i] = points[i];
            int degree = polyline ? 1 : Math.Min(3, pts.Length - 1);
            return new BSplineCurve(degree, pts, BSplineCurve.UniformKnotVector(degree, pts.Length), false);
        }

        private static bool IsCollinearRun(IReadOnlyList<Vec3D> points)
        {
            if (points.Count < 3)
                return true;
            Vec3D origin = points[0];
            Vec3D dir = points[points.Count - 1] - origin;
            double span = dir.Length();
            if (span < MinSegmentLength)
                return false;
            dir *= 1.0 / span;
            for (int i = 1; i < points.Count - 1; i++)
            {
                var toPoint = points[i] - origin;
                var perp = toPoint - dir * Vec3DOps.Dot(toPoint, dir);
                if (perp.Length() / span > CollinearAngleTolerance)
                    return false;
            }
            return true;
        }

        private const double IsoUvTolerance = 0.02;

        private static int IsoFamily(Vec2D a, Vec2D b)
        {
            double du = Math.Abs(a.X - b.X);
            double dv = Math.Abs(a.Y - b.Y);
            if (du < IsoUvTolerance && dv > IsoUvTolerance)
                return 1;
            if (dv < IsoUvTolerance && du > IsoUvTolerance)
                return 2;
            return 0;
        }

        private static bool[] ApplyGlobalCorners(
            BRepTrimLoop loop, int[] vids, HashSet<int> globalCorners, bool closed)
        {
            int n = closed ? loop.WorldPoints.Count - 1 : loop.WorldPoints.Count;
            var flags = new bool[Math.Max(n, 0)];
            for (int i = 0; i < n; i++)
            {
                if (globalCorners.Contains(vids[i]))
                    flags[i] = true;
            }

            return flags;
        }

        private static bool[] CornerFlags(BRepTrimLoop loop, int[] vids, int[] faceCount, bool closed)
        {
            int n = closed ? loop.WorldPoints.Count - 1 : loop.WorldPoints.Count;
            var flags = new bool[Math.Max(n, 0)];
            if (n < 2)
                return flags;

            for (int i = 0; i < n; i++)
            {
                if (vids[i] >= 0 && vids[i] < faceCount.Length && faceCount[vids[i]] >= 3)
                    flags[i] = true;

                int prev = closed ? (i - 1 + n) % n : i - 1;
                int next = closed ? (i + 1) % n : i + 1;
                if (prev < 0 || next >= n)
                    continue;
                int f0 = IsoFamily(UvAt(loop, prev), UvAt(loop, i));
                int f1 = IsoFamily(UvAt(loop, i), UvAt(loop, next));
                if (f0 != 0 && f1 != 0 && f0 != f1)
                    flags[i] = true;
            }

            return flags;
        }

        private static int FirstCorner(bool[] corners)
        {
            if (corners == null)
                return -1;
            for (int i = 0; i < corners.Length; i++)
            {
                if (corners[i])
                    return i;
            }
            return -1;
        }

        private static List<(int, int)> StripRanges(BRepTrimLoop loop, bool[] corners, bool closed)
        {
            var ranges = new List<(int, int)>();
            int last = loop.WorldPoints.Count - 1;
            int first = FirstCorner(corners);
            if (first < 0)
            {
                ranges.Add((0, last));
                return ranges;
            }

            var splits = new List<int>();
            for (int i = 0; i < corners.Length; i++)
            {
                if (corners[i])
                    splits.Add(i);
            }

            if (closed)
            {
                for (int i = 0; i < splits.Count - 1; i++)
                    ranges.Add((splits[i], splits[i + 1]));
                ranges.Add((splits[splits.Count - 1], last));
            }
            else
            {
                if (splits[0] > 0)
                    ranges.Add((0, splits[0]));
                for (int i = 0; i < splits.Count - 1; i++)
                    ranges.Add((splits[i], splits[i + 1]));
                if (splits[splits.Count - 1] < last)
                    ranges.Add((splits[splits.Count - 1], last));
            }

            return ranges;
        }

        private static void RotateClosedLoopTo(BRepTrimLoop loop, int start)
        {
            if (start <= 0 || loop.WorldPoints.Count < 3)
                return;

            int n = loop.WorldPoints.Count - 1;
            var world = new List<Vec3D>(loop.WorldPoints.Count);
            var uv = new List<Vec2D>(loop.WorldPoints.Count);
            for (int i = 0; i < n; i++)
            {
                int src = (start + i) % n;
                world.Add(loop.WorldPoints[src]);
                uv.Add(UvAt(loop, src));
            }

            world.Add(world[0]);
            uv.Add(uv[0]);
            loop.WorldPoints = world;
            loop.UvPoints = uv;
        }

        private static long ChordSetKey(int[] vids, int start, int end)
        {
            var keys = new List<long>(end - start);
            for (int i = start; i < end; i++)
            {
                int a = vids[i];
                int b = vids[i + 1];
                if (a == b)
                    continue;
                int lo = Math.Min(a, b);
                int hi = Math.Max(a, b);
                keys.Add(((long)lo << 32) | (uint)hi);
            }

            if (keys.Count == 0)
                return 0;

            keys.Sort();
            long h = 17;
            for (int i = 0; i < keys.Count; i++)
                h = h * 31 + keys[i];
            return h * 31 + keys.Count;
        }

        private static int TryFindLineRun(IReadOnlyList<Vec3D> points, int start)
        {
            if (start >= points.Count - 1)
                return start;

            int end = start + 1;
            Vec3D origin = points[start];
            Vec3D dir = (points[end] - origin).Normalized();

            for (int j = start + 2; j < points.Count; j++)
            {
                var toPoint = points[j] - origin;
                if (toPoint.Length() < MinSegmentLength)
                    continue;

                var perp = toPoint - dir * Vec3DOps.Dot(toPoint, dir);
                double perpLen = perp.Length();
                double span = toPoint.Length();
                if (span < MinSegmentLength || perpLen / span > CollinearAngleTolerance)
                    break;
                end = j;
            }

            return end;
        }

        private static int TryFindArcRun(IReadOnlyList<Vec3D> points, int start, out BSplineCurve curve)
        {
            curve = null;
            if (start + 2 >= points.Count)
                return start;

            int bestEnd = start;
            for (int end = start + 2; end < points.Count; end++)
            {
                int mid = (start + end) / 2;
                if (!TryBuildArcCurve(points[start], points[mid], points[end], out var candidate))
                    break;
                if (!PointsLieOnArc(points, start, end, points[start], points[mid], points[end]))
                    break;
                if (!TurnsAreSmooth(points, start, end))
                    break;
                curve = candidate;
                bestEnd = end;
            }

            return bestEnd;
        }

        private static bool TryBuildArcCurve(Vec3D start, Vec3D mid, Vec3D end, out BSplineCurve curve)
        {
            curve = null;
            if ((mid - start).Length() < MinSegmentLength || (end - mid).Length() < MinSegmentLength)
                return false;
            try
            {
                var arc = new Arc3D(start, mid, end);
                curve = Curve3DToBSpline.ArcToBSpline(arc);
                return true;
            }
            catch
            {
                return false;
            }
        }

        private static bool PointsLieOnArc(
            IReadOnlyList<Vec3D> points,
            int start,
            int end,
            Vec3D arcStart,
            Vec3D arcMid,
            Vec3D arcEnd)
        {
            Arc3D arc;
            try
            {
                arc = new Arc3D(arcStart, arcMid, arcEnd);
            }
            catch
            {
                return false;
            }

            var normal = arc.Normal;
            double radius = arc.Radius;
            var center = arc.Center;

            for (int i = start; i <= end; i++)
            {
                var p = points[i];
                if (Math.Abs((p - center).Length() - radius) > ArcRadiusTolerance)
                    return false;
                if (Math.Abs(Vec3DOps.Dot(p - center, normal)) > ArcPlaneTolerance)
                    return false;
            }

            return true;
        }

        /// <summary>
        /// Reject polyline corners that cannot be a tessellated circular arc (e.g. a square).
        /// Regular n-gons with n≥6 stay under this limit; 90° rectangle corners do not.
        /// </summary>
        private static bool TurnsAreSmooth(IReadOnlyList<Vec3D> points, int start, int end)
        {
            const double maxTurn = 70.0 * Math.PI / 180.0;
            for (int i = start + 1; i < end; i++)
            {
                var d0 = points[i] - points[i - 1];
                var d1 = points[i + 1] - points[i];
                double l0 = d0.Length();
                double l1 = d1.Length();
                if (l0 < MinSegmentLength || l1 < MinSegmentLength)
                    continue;
                d0 *= 1.0 / l0;
                d1 *= 1.0 / l1;
                double cos = Vec3DOps.Dot(d0, d1);
                if (cos > 1.0) cos = 1.0;
                if (cos < -1.0) cos = -1.0;
                if (Math.Acos(cos) > maxTurn)
                    return false;
            }
            return true;
        }
    }
}
