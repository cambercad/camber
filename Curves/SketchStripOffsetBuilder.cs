using GeoCore;
using NURBS;

namespace Curves
{
    public static class SketchStripOffsetBuilder
    {
        private const double DegenerateLength = 1e-12;
        private const double MiterLimit = 2.0;
        private const int MinRoundSegmentsPerQuarter = 6;

        public static CurveStrip2D Build(
            IReadOnlyList<Curve2D> sourceStrip,
            double offset,
            SketchStripOffsetOptions options,
            out bool isClosed)
        {
            return Build(sourceStrip, offset, options, out isClosed, out _, out _, out _);
        }

        public static CurveStrip2D Build(
            IReadOnlyList<Curve2D> sourceStrip,
            double offset,
            SketchStripOffsetOptions options,
            out bool isClosed,
            out List<List<Vec2D>> loops,
            out List<List<int>> vertexIds,
            out OffsetSourceVertexMap map)
        {
            if (sourceStrip == null || sourceStrip.Count == 0)
                throw new ArgumentException("Source strip must not be empty.", nameof(sourceStrip));

            bool sourceClosed = IsClosedSourceStrip(sourceStrip, options.ConnectionTolerance);
            SketchCurveGraph.Path path = OrderedStripPath(sourceStrip, sourceClosed);
            if (!TryBuildTaggedPath(
                sourceStrip, path, options,
                out List<Vec2D> polyline, out List<int> edgeSource, out List<bool> joint, out bool closed))
                throw new InvalidOperationException("Source strip tessellation produced too few points.");

            if (closed)
                ReverseClosedIfClockwise(polyline, joint, edgeSource);

            map = new OffsetSourceVertexMap();
            map.AddPath(polyline, edgeSource, joint, closed);

            if (!closed && options.OpenMode == SketchOffsetOpenMode.Parallel)
            {
                var ids = new List<int>();
                List<Vec2D> parallel = OffsetOpenParallel(
                    polyline, offset, options.JoinType,
                    EffectiveRoundArcTolerance(options.CornerArcTolerance, offset),
                    ids);
                isClosed = false;
                loops = new List<List<Vec2D>> { parallel };
                vertexIds = new List<List<int>> { ids };
                return CurveStrip2D.FromPolyline(parallel, closed: false);
            }

            EndType endType = closed
                ? EndType.etClosedPolygon
                : ToOpenEndType(options.EndCap);
            double clipperArcTolerance = PolygonManipulator.WorldDistanceToClipperUnits(
                EffectiveRoundArcTolerance(options.CornerArcTolerance, offset),
                polyline);
            var rawIds = new List<List<int>>();
            List<List<Vec2D>> offsetLoops = PolygonManipulator.Offset(
                polyline, offset, rawIds, ToJoinType(options.JoinType), endType, clipperArcTolerance);

            if (offsetLoops == null || offsetLoops.Count == 0)
                throw new InvalidOperationException("Offset collapsed (distance too large?).");

            int primary = IndexOfPrimaryOffsetLoop(offsetLoops);
            List<Vec2D> resultPolyline = offsetLoops[primary];
            List<int> resultIds = primary < rawIds.Count ? rawIds[primary] : new List<int>();
            resultPolyline = NormalizeClosedLoop(resultPolyline, resultIds, options);
            if (resultPolyline.Count < 2)
                throw new InvalidOperationException("Offset produced too few points.");

            isClosed = true;
            loops = new List<List<Vec2D>> { resultPolyline };
            vertexIds = new List<List<int>> { resultIds };
            return CurveStrip2D.FromPolyline(resultPolyline, closed: true);
        }

        public static CurveStrip2D Build(
            IReadOnlyList<Curve2D> sourceStrip,
            double offset,
            SketchStripOffsetOptions options)
            => Build(sourceStrip, offset, options, out _);

        /// <summary>
        /// Offset a graph of curves (T-junctions / forks allowed). Open paths use Outline
        /// (both sides + caps) so overlapping wall regions union at joints.
        /// </summary>
        public static List<List<Vec2D>> BuildNetwork(
            IReadOnlyList<Curve2D> sources,
            double offset,
            SketchStripOffsetOptions options)
        {
            return BuildNetwork(sources, offset, options, out _, out _);
        }

        public static List<List<Vec2D>> BuildNetwork(
            IReadOnlyList<Curve2D> sources,
            double offset,
            SketchStripOffsetOptions options,
            out List<List<int>> vertexIds,
            out OffsetSourceVertexMap vertexMap)
        {
            vertexIds = new List<List<int>>();
            vertexMap = new OffsetSourceVertexMap();
            if (sources == null || sources.Count == 0)
                throw new ArgumentException("Source curve list must not be empty.", nameof(sources));

            var paths = SketchCurveGraph.SplitIntoPaths(sources, options.ConnectionTolerance);
            var polylines = new List<List<Vec2D>>();
            var endTypes = new List<EndType>();
            for (int i = 0; i < paths.Count; i++)
            {
                if (!TryBuildTaggedPath(
                    sources, paths[i], options,
                    out List<Vec2D> polyline, out List<int> edgeSource, out List<bool> joint, out bool closed))
                    continue;

                if (closed)
                    endTypes.Add(EndType.etClosedLine);
                else
                    endTypes.Add(ToOpenEndType(options.EndCap));

                vertexMap.AddPath(polyline, edgeSource, joint, closed);
                polylines.Add(polyline);
            }

            if (polylines.Count == 0)
                throw new InvalidOperationException("Network tessellation produced no paths.");

            var refs = polylines.ToArray();
            double clipperArcTolerance = PolygonManipulator.WorldDistanceToClipperUnits(
                EffectiveRoundArcTolerance(options.CornerArcTolerance, offset), refs);
            var rawIds = new List<List<int>>();
            List<List<Vec2D>> loops = PolygonManipulator.Offset(
                polylines,
                offset,
                rawIds,
                endTypes,
                ToJoinType(options.JoinType),
                clipperArcTolerance);

            if (loops == null || loops.Count == 0)
                throw new InvalidOperationException("Offset collapsed (distance too large?).");

            var kept = new List<List<Vec2D>>();
            var keptIds = new List<List<int>>();
            for (int i = 0; i < loops.Count; i++)
            {
                if (loops[i] == null)
                    continue;
                List<int> ids = i < rawIds.Count ? rawIds[i] : new List<int>();
                List<Vec2D> loop = NormalizeClosedLoop(loops[i], ids, options);
                if (loop.Count >= 3 && Math.Abs(SignedArea(loop)) > 1e-8)
                {
                    kept.Add(loop);
                    keptIds.Add(ids);
                }
            }
            if (kept.Count == 0)
                throw new InvalidOperationException("Offset produced no usable loops.");

            var order = new int[kept.Count];
            for (int i = 0; i < order.Length; i++)
                order[i] = i;
            Array.Sort(order, (a, b) => Math.Abs(SignedArea(kept[b])).CompareTo(Math.Abs(SignedArea(kept[a]))));
            var sortedLoops = new List<List<Vec2D>>(kept.Count);
            var sortedIds = new List<List<int>>(kept.Count);
            for (int i = 0; i < order.Length; i++)
            {
                sortedLoops.Add(kept[order[i]]);
                sortedIds.Add(keptIds[order[i]]);
            }
            vertexIds = sortedIds;
            return sortedLoops;
        }

        static List<Curve2D> OrientPath(SketchCurveGraph.Path path)
        {
            var oriented = new List<Curve2D>(path.Curves.Count);
            for (int i = 0; i < path.Curves.Count; i++)
            {
                Curve2D curve = path.Curves[i];
                oriented.Add(path.Reversed[i] ? curve.Reverse() : curve);
            }
            return oriented;
        }

        static bool TryBuildTaggedPath(
            IReadOnlyList<Curve2D> sources,
            SketchCurveGraph.Path path,
            SketchStripOffsetOptions options,
            out List<Vec2D> polyline,
            out List<int> edgeSource,
            out List<bool> joint,
            out bool closed)
        {
            polyline = null;
            edgeSource = null;
            joint = null;
            closed = path.IsClosed;

            var sourceIndex = new int[path.Curves.Count];
            for (int i = 0; i < path.Curves.Count; i++)
                sourceIndex[i] = IndexOfSource(sources, path.Curves[i]);

            List<Curve2D> oriented = OrientPath(path);
            var tessOptions = new SketchStripTessellateOptions
            {
                IncludeHelperGeometry = true,
                AllowOpenContour = !closed,
                SkipValidation = true,
                CurveMatchingTolerance = options.ConnectionTolerance,
            };
            SketchStripTessellator.TessellateStrip(
                oriented,
                options.TessellationTolerance,
                tessOptions,
                out var segmentPoints,
                out var segmentNormals);
            SketchStripTessellator.FlattenStripWithCreases(
                segmentPoints, segmentNormals, out polyline, out _, out List<int> creases);
            if (polyline == null || polyline.Count < 2)
                return false;

            int n = polyline.Count;
            joint = new List<bool>(n);
            edgeSource = new List<int>(n);
            for (int i = 0; i < n; i++)
            {
                joint.Add(false);
                edgeSource.Add(-1);
            }
            joint[0] = true;
            joint[n - 1] = true;

            var splits = new List<int>();
            for (int c = 0; c < creases.Count; c++)
            {
                int idx = creases[c];
                if (idx <= 0 || idx >= n)
                    continue;
                joint[idx] = true;
                splits.Add(idx);
            }
            splits.Sort();

            int curve = 0;
            int nextSplit = 0;
            for (int i = 0; i < n; i++)
            {
                while (nextSplit < splits.Count && i >= splits[nextSplit])
                {
                    curve++;
                    nextSplit++;
                }
                if (curve >= sourceIndex.Length)
                    curve = sourceIndex.Length - 1;
                if (closed || i < n - 1)
                    edgeSource[i] = sourceIndex[Math.Max(0, curve)];
            }

            if (closed)
            {
                DropDuplicateClose(polyline, joint, edgeSource, options.ConnectionTolerance);
                if (polyline.Count < 3)
                    return false;
            }
            else
            {
                EnsureMinimumVerticesForClipper(polyline, joint, edgeSource);
            }

            closed = closed || SketchStripTessellator.IsClosedPolyline(polyline);
            return polyline.Count >= 2;
        }

        static SketchCurveGraph.Path OrderedStripPath(IReadOnlyList<Curve2D> strip, bool closed)
        {
            var path = new SketchCurveGraph.Path();
            path.IsClosed = closed;
            for (int i = 0; i < strip.Count; i++)
            {
                path.Curves.Add(strip[i]);
                path.Reversed.Add(false);
            }
            return path;
        }

        static int IndexOfSource(IReadOnlyList<Curve2D> sources, Curve2D curve)
        {
            for (int i = 0; i < sources.Count; i++)
            {
                if (ReferenceEquals(sources[i], curve))
                    return i;
            }
            return -1;
        }

        static List<Vec2D> TessellatePath(
            IReadOnlyList<Curve2D> oriented, bool closed, SketchStripOffsetOptions options)
        {
            var tessOptions = new SketchStripTessellateOptions
            {
                IncludeHelperGeometry = true,
                AllowOpenContour = !closed,
                SkipValidation = true,
                CurveMatchingTolerance = options.ConnectionTolerance,
            };
            SketchStripTessellator.TessellateStrip(
                oriented,
                options.TessellationTolerance,
                tessOptions,
                out var segmentPoints,
                out var segmentNormals);
            SketchStripTessellator.FlattenStripWithCreases(
                segmentPoints, segmentNormals, out var polyline, out _, out _);
            return polyline;
        }

        /// <summary>
        /// One-sided open polyline offset. Positive <paramref name="offset"/> is to the left of travel.
        /// </summary>
        private static List<Vec2D> OffsetOpenParallel(
            List<Vec2D> polyline,
            double offset,
            SketchOffsetJoinType joinType,
            double arcTolerance,
            List<int> vertexIds)
        {
            vertexIds.Clear();
            if (Math.Abs(offset) <= DegenerateLength)
            {
                for (int i = 0; i < polyline.Count; i++)
                    vertexIds.Add(i);
                return new List<Vec2D>(polyline);
            }

            int n = polyline.Count;
            var edgeDirs = new Vec2D[n - 1];
            var edgeNormals = new Vec2D[n - 1];
            for (int i = 0; i < n - 1; i++)
            {
                Vec2D d = polyline[i + 1] - polyline[i];
                double len = d.Length();
                if (len <= DegenerateLength)
                {
                    edgeDirs[i] = Vec2DOps.Zero;
                    edgeNormals[i] = Vec2DOps.Zero;
                    continue;
                }

                edgeDirs[i] = d * (1.0 / len);
                edgeNormals[i] = new Vec2D(-edgeDirs[i].Y, edgeDirs[i].X);
            }

            int first = IndexOfFirstValidEdge(edgeNormals);
            int last = IndexOfLastValidEdge(edgeNormals);
            if (first < 0 || last < 0)
                throw new InvalidOperationException("Open parallel offset requires a non-degenerate strip.");

            var result = new List<Vec2D>();
            AppendTagged(result, vertexIds, polyline[0] + offset * edgeNormals[first], 0);

            for (int i = 1; i < n - 1; i++)
            {
                int prev = PreviousValidEdge(edgeNormals, i - 1);
                int next = NextValidEdge(edgeNormals, i);
                if (prev < 0 || next < 0)
                    continue;

                AppendJoin(
                    result,
                    vertexIds,
                    i,
                    polyline[i],
                    edgeDirs[prev],
                    edgeNormals[prev],
                    edgeDirs[next],
                    edgeNormals[next],
                    offset,
                    joinType,
                    arcTolerance);
            }

            AppendTagged(result, vertexIds, polyline[n - 1] + offset * edgeNormals[last], n - 1);
            return result;
        }

        private static void AppendJoin(
            List<Vec2D> result,
            List<int> vertexIds,
            int vertexId,
            Vec2D vertex,
            Vec2D dir0,
            Vec2D n0,
            Vec2D dir1,
            Vec2D n1,
            double offset,
            SketchOffsetJoinType joinType,
            double arcTolerance)
        {
            Vec2D left0 = vertex + offset * n0;
            Vec2D left1 = vertex + offset * n1;

            double cross = dir0.X * dir1.Y - dir0.Y * dir1.X;
            double dot = Vec2DOps.Dot(dir0, dir1);

            // Nearly collinear same direction — single point.
            if (Math.Abs(cross) <= 1e-10 && dot > 0)
            {
                AppendTagged(result, vertexIds, left0, vertexId);
                return;
            }

            switch (joinType)
            {
                case SketchOffsetJoinType.Round:
                    AppendRoundJoin(result, vertexIds, vertexId, vertex, n0, n1, offset, arcTolerance);
                    break;

                case SketchOffsetJoinType.Square:
                    AppendTagged(result, vertexIds, left0, vertexId);
                    AppendTagged(result, vertexIds, left1, vertexId);
                    break;

                default: // Miter
                    if (TryIntersectLines(left0, dir0, left1, dir1, out Vec2D miter))
                    {
                        double miterLen = (miter - vertex).Length();
                        double maxMiter = Math.Abs(offset) * MiterLimit;
                        if (miterLen <= maxMiter + 1e-9)
                        {
                            AppendTagged(result, vertexIds, miter, vertexId);
                            break;
                        }
                    }

                    AppendTagged(result, vertexIds, left0, vertexId);
                    AppendTagged(result, vertexIds, left1, vertexId);
                    break;
            }
        }

        private static void AppendRoundJoin(
            List<Vec2D> result,
            List<int> vertexIds,
            int vertexId,
            Vec2D center,
            Vec2D n0,
            Vec2D n1,
            double offset,
            double arcTolerance)
        {
            double radius = Math.Abs(offset);
            if (radius <= DegenerateLength)
            {
                AppendTagged(result, vertexIds, center, vertexId);
                return;
            }

            Vec2D v0 = offset * n0;
            Vec2D v1 = offset * n1;
            double a0 = Math.Atan2(v0.Y, v0.X);
            double a1 = Math.Atan2(v1.Y, v1.X);
            double delta = a1 - a0;
            while (delta <= -Math.PI) delta += 2 * Math.PI;
            while (delta > Math.PI) delta -= 2 * Math.PI;

            if (Math.Abs(delta) < 1e-10)
            {
                AppendTagged(result, vertexIds, center + v0, vertexId);
                return;
            }

            double absDelta = Math.Abs(delta);
            double maxStep = arcTolerance > 0
                ? 2.0 * Math.Acos(Math.Max(-1.0, Math.Min(1.0, 1.0 - arcTolerance / radius)))
                : absDelta;
            if (maxStep <= 1e-8 || double.IsNaN(maxStep))
                maxStep = absDelta;

            int steps = Math.Max(1, (int)Math.Ceiling(absDelta / maxStep));
            for (int i = 0; i <= steps; i++)
            {
                double t = i / (double)steps;
                double a = a0 + t * delta;
                AppendTagged(result, vertexIds, center + new Vec2D(radius * Math.Cos(a), radius * Math.Sin(a)), vertexId);
            }
        }

        private static bool TryIntersectLines(
            Vec2D p0, Vec2D d0, Vec2D p1, Vec2D d1, out Vec2D intersection)
        {
            double cross = d0.X * d1.Y - d0.Y * d1.X;
            if (Math.Abs(cross) <= 1e-12)
            {
                intersection = default;
                return false;
            }

            Vec2D w = p1 - p0;
            double t = (w.X * d1.Y - w.Y * d1.X) / cross;
            intersection = p0 + t * d0;
            return true;
        }

        private static void AppendTagged(List<Vec2D> points, List<int> ids, Vec2D p, int vertexId)
        {
            if (points.Count == 0 ||
                Vec2DOps.DistanceSquared(points[points.Count - 1], p) > DegenerateLength * DegenerateLength)
            {
                points.Add(p);
                ids.Add(vertexId);
            }
        }

        private static int IndexOfFirstValidEdge(Vec2D[] normals)
        {
            for (int i = 0; i < normals.Length; i++)
            {
                if (normals[i].LengthSquared() > DegenerateLength * DegenerateLength)
                    return i;
            }
            return -1;
        }

        private static int IndexOfLastValidEdge(Vec2D[] normals)
        {
            for (int i = normals.Length - 1; i >= 0; i--)
            {
                if (normals[i].LengthSquared() > DegenerateLength * DegenerateLength)
                    return i;
            }
            return -1;
        }

        private static int PreviousValidEdge(Vec2D[] normals, int fromInclusive)
        {
            for (int i = fromInclusive; i >= 0; i--)
            {
                if (normals[i].LengthSquared() > DegenerateLength * DegenerateLength)
                    return i;
            }
            return -1;
        }

        private static int NextValidEdge(Vec2D[] normals, int fromInclusive)
        {
            for (int i = fromInclusive; i < normals.Length; i++)
            {
                if (normals[i].LengthSquared() > DegenerateLength * DegenerateLength)
                    return i;
            }
            return -1;
        }

        /// <summary>
        /// Chordal error for Clipper / parallel round joins. Never coarser than
        /// <paramref name="cornerTol"/>, and never so loose that a 90° corner
        /// collapses to a chamfer (Clipper would otherwise use 1–2 segments when
        /// part tolerance is a large fraction of the offset radius).
        /// </summary>
        private static double EffectiveRoundArcTolerance(double cornerTol, double offset)
        {
            double radius = Math.Abs(offset);
            if (radius <= DegenerateLength)
                return cornerTol > 0 ? cornerTol : 0.01;

            double absTol = cornerTol > 0 ? cornerTol : radius * 0.01;
            if (absTol >= radius)
                absTol = 0.5 * radius;

            double fromTol = 2.0 * Math.Acos(Math.Max(-1.0, Math.Min(1.0, 1.0 - absTol / radius)));
            double minQuarter = (0.5 * Math.PI) / MinRoundSegmentsPerQuarter;
            double step = Math.Min(fromTol, minQuarter);
            if (step <= 1e-8 || double.IsNaN(step))
                step = minQuarter;
            return radius * (1.0 - Math.Cos(0.5 * step));
        }

        private static JoinType ToJoinType(SketchOffsetJoinType joinType) => joinType switch
        {
            SketchOffsetJoinType.Square => JoinType.jtSquare,
            SketchOffsetJoinType.Miter => JoinType.jtMiter,
            _ => JoinType.jtRound,
        };

        private static EndType ToOpenEndType(SketchOffsetEndCap endCap) => endCap switch
        {
            SketchOffsetEndCap.Butt => EndType.etOpenButt,
            SketchOffsetEndCap.Square => EndType.etOpenSquare,
            _ => EndType.etOpenRound,
        };

        private static bool IsClosedSourceStrip(IReadOnlyList<Curve2D> sourceStrip, double tolerance)
        {
            if (sourceStrip.Count == 1 && sourceStrip[0] is Circle2D)
                return true;

            double tolSq = tolerance * tolerance;
            return Vec2DOps.DistanceSquared(
                sourceStrip[0].StartPosition,
                sourceStrip[^1].EndPosition) <= tolSq;
        }

        /// <summary>
        /// Clipper may repeat the first vertex or emit edges shorter than the mesh grid.
        /// Those slivers become skipped side quads on extrude (full-height holes in the wall).
        /// </summary>
        private static List<Vec2D> NormalizeClosedLoop(List<Vec2D> polyline, SketchStripOffsetOptions options)
        {
            return NormalizeClosedLoop(polyline, null, options);
        }

        private static List<Vec2D> NormalizeClosedLoop(
            List<Vec2D> polyline, List<int> vertexIds, SketchStripOffsetOptions options)
        {
            double tol = options.ConnectionTolerance;
            if (options.TessellationTolerance > tol)
                tol = options.TessellationTolerance;
            if (tol <= 0)
                tol = 1e-6;
            var result = new List<Vec2D>(polyline.Count);
            for (int i = 0; i < polyline.Count; i++)
                result.Add(polyline[i]);
            DropDuplicateClose(result, vertexIds, tol);
            RemoveConsecutiveDuplicates(result, vertexIds, tol);
            DropDuplicateClose(result, vertexIds, tol);
            return result;
        }

        private static void RemoveConsecutiveDuplicates(List<Vec2D> polyline, double tolerance)
        {
            RemoveConsecutiveDuplicates(polyline, null, tolerance);
        }

        private static void RemoveConsecutiveDuplicates(List<Vec2D> polyline, List<int> vertexIds, double tolerance)
        {
            if (polyline == null || polyline.Count < 2)
                return;
            double tolSq = (tolerance > 0 ? tolerance : 1e-6) * (tolerance > 0 ? tolerance : 1e-6);
            for (int i = polyline.Count - 1; i > 0; i--)
            {
                if (Vec2DOps.DistanceSquared(polyline[i], polyline[i - 1]) <= tolSq)
                {
                    polyline.RemoveAt(i);
                    if (vertexIds != null && i < vertexIds.Count)
                        vertexIds.RemoveAt(i);
                }
            }
        }

        private static void DropDuplicateClose(List<Vec2D> polyline, double tolerance)
        {
            DropDuplicateClose(polyline, (List<int>)null, tolerance);
        }

        private static void DropDuplicateClose(List<Vec2D> polyline, List<int> vertexIds, double tolerance)
        {
            if (polyline == null || polyline.Count < 2)
                return;
            double tol = tolerance > 0 ? tolerance : 1e-6;
            if (Vec2DOps.DistanceSquared(polyline[0], polyline[^1]) <= tol * tol)
            {
                polyline.RemoveAt(polyline.Count - 1);
                if (vertexIds != null && vertexIds.Count > polyline.Count)
                    vertexIds.RemoveAt(vertexIds.Count - 1);
            }
        }

        static void DropDuplicateClose(List<Vec2D> polyline, List<bool> joint, List<int> edgeSource, double tolerance)
        {
            if (polyline == null || polyline.Count < 2)
                return;
            double tol = tolerance > 0 ? tolerance : 1e-6;
            if (Vec2DOps.DistanceSquared(polyline[0], polyline[^1]) > tol * tol)
                return;
            int lastOwner = edgeSource != null && edgeSource.Count >= 2
                ? edgeSource[edgeSource.Count - 2]
                : -1;
            polyline.RemoveAt(polyline.Count - 1);
            if (joint != null && joint.Count > polyline.Count)
            {
                joint[0] = true;
                joint.RemoveAt(joint.Count - 1);
            }
            if (edgeSource != null && edgeSource.Count > polyline.Count)
            {
                edgeSource.RemoveAt(edgeSource.Count - 1);
                if (edgeSource.Count > 0 && lastOwner >= 0)
                    edgeSource[edgeSource.Count - 1] = lastOwner;
            }
        }

        private static void EnsureMinimumVerticesForClipper(List<Vec2D> polyline)
        {
            EnsureMinimumVerticesForClipper(polyline, null, null);
        }

        static void EnsureMinimumVerticesForClipper(List<Vec2D> polyline, List<bool> joint, List<int> edgeSource)
        {
            while (polyline.Count < 3)
            {
                int best = 0;
                double bestLen = 0.0;
                for (int i = 0; i < polyline.Count - 1; i++)
                {
                    double len = Vec2DOps.DistanceSquared(polyline[i], polyline[i + 1]);
                    if (len > bestLen)
                    {
                        bestLen = len;
                        best = i;
                    }
                }

                Vec2D mid = 0.5 * (polyline[best] + polyline[best + 1]);
                polyline.Insert(best + 1, mid);
                if (joint != null)
                    joint.Insert(best + 1, false);
                if (edgeSource != null)
                    edgeSource.Insert(best + 1, edgeSource[best]);
            }
        }

        static void ReverseClosedIfClockwise(List<Vec2D> polyline, List<bool> joint, List<int> edgeSource)
        {
            if (polyline == null || polyline.Count < 3 || SignedArea(polyline) >= 0)
                return;

            int n = polyline.Count;
            polyline.Reverse();
            if (joint != null && joint.Count == n)
                joint.Reverse();
            if (edgeSource == null || edgeSource.Count != n)
                return;

            var old = new int[n];
            for (int i = 0; i < n; i++)
                old[i] = edgeSource[i];
            for (int j = 0; j < n; j++)
                edgeSource[j] = old[(n - 2 - j + n) % n];
        }

        private static void EnsureCounterClockwise(List<Vec2D> polyline)
        {
            ReverseClosedIfClockwise(polyline, null, null);
        }

        private static double SignedArea(IReadOnlyList<Vec2D> polyline)
        {
            double area = 0;
            int n = polyline.Count;
            for (int i = 0; i < n; i++)
            {
                Vec2D a = polyline[i];
                Vec2D b = polyline[(i + 1) % n];
                area += a.X * b.Y - b.X * a.Y;
            }

            return area * 0.5;
        }

        private static int IndexOfPrimaryOffsetLoop(List<List<Vec2D>> loops)
        {
            int bestIndex = 0;
            double bestArea = Math.Abs(SignedArea(loops[0]));
            for (int i = 1; i < loops.Count; i++)
            {
                double area = Math.Abs(SignedArea(loops[i]));
                if (area > bestArea)
                {
                    bestArea = area;
                    bestIndex = i;
                }
            }
            return bestIndex;
        }

        private static List<Vec2D> SelectPrimaryOffsetLoop(List<List<Vec2D>> loops)
        {
            if (loops.Count == 1)
                return loops[0];
            return loops[IndexOfPrimaryOffsetLoop(loops)];
        }
    }
}
