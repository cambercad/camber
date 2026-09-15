#pragma warning disable CS8625 // Cannot convert null literal to non-nullable reference type

using GeoCore;
using GeoMeta;

namespace Geo
{
    public static class GroupEdgeExtractor
    {
        // The triangles must be watertight
        public static List<GroupEdge> ExtractGroupEdges(List<Tri> triangles, List<int> groupIdPerTriangle, 
            List<Vec3D> points, List<Rat3Hybrid> pointsExact, Dictionary<int, string> triangleGroupToName,
            Dictionary<string, SurfaceMetaData> surfaceMetaData = null,
            bool preferLexClosedLoopStarts = false)
        {
            if (triangles == null || groupIdPerTriangle == null || triangles.Count != groupIdPerTriangle.Count)
                return new List<GroupEdge>();

            int numTris = triangles.Count;
            
            // Step 1: Collect all edges as (edgeKey, triangleIndex) pairs - no dictionary needed
            var edgeTriPairs = new (long edge, int tri)[numTris * 3];
            for (int i = 0; i < numTris; i++)
            {
                var t = triangles[i];
                edgeTriPairs[i * 3] = (Algorithms.Key(t.A, t.B), i);
                edgeTriPairs[i * 3 + 1] = (Algorithms.Key(t.B, t.C), i);
                edgeTriPairs[i * 3 + 2] = (Algorithms.Key(t.C, t.A), i);
            }
            
            // Step 2: Sort by edge key - O(n log n) but cache-friendly
            Array.Sort(edgeTriPairs, (a, b) => a.edge.CompareTo(b.edge));
            
            // Step 3: Process sorted edges - find group boundaries
            // Adjacent entries with same edge key share that edge
            var groupEdgeSegments = new Dictionary<long, List<Int2>>();
            
            int idx = 0;
            int totalPairs = edgeTriPairs.Length;
            while (idx < totalPairs)
            {
                long currentEdge = edgeTriPairs[idx].edge;
                int startIdx = idx;
                
                // Find all triangles sharing this edge
                while (idx < totalPairs && edgeTriPairs[idx].edge == currentEdge)
                    idx++;
                
                int count = idx - startIdx;

                // Collect distinct groups among all triangles sharing this edge
                // For 2 triangles (manifold) this is trivially fast; for more it still works
                int firstGroup = groupIdPerTriangle[edgeTriPairs[startIdx].tri];
                bool allSameGroup = true;
                for (int k = startIdx + 1; k < idx; k++)
                {
                    if (groupIdPerTriangle[edgeTriPairs[k].tri] != firstGroup)
                    {
                        allSameGroup = false;
                        break;
                    }
                }

                if (allSameGroup)
                    continue;

                Algorithms.DecomposeKey(currentEdge, out int edgeV1, out int edgeV2);
                var segment = new Int2(edgeV1, edgeV2);

                // Collect all distinct group pairs present along this edge
                for (int k = startIdx; k < idx; k++)
                {
                    int groupK = groupIdPerTriangle[edgeTriPairs[k].tri];
                    for (int m = k + 1; m < idx; m++)
                    {
                        int groupM = groupIdPerTriangle[edgeTriPairs[m].tri];
                        if (groupK != groupM)
                        {
                            long groupPair = Algorithms.Key(groupK, groupM);
                            if (!groupEdgeSegments.TryGetValue(groupPair, out var segs))
                                groupEdgeSegments[groupPair] = segs = new List<Int2>();

                            // Avoid adding the same segment twice for the same group pair
                            if (segs.Count == 0 || segs[segs.Count - 1].X != edgeV1 || segs[segs.Count - 1].Y != edgeV2)
                                segs.Add(segment);
                        }
                    }
                }
            }

            // Step 3: Split segments into connected components and create separate GroupEdge objects
            var result = new List<GroupEdge>();
            
            foreach (var groupSegmentEntry in groupEdgeSegments)
            {
                long groupPair = groupSegmentEntry.Key;
                var segments = groupSegmentEntry.Value;

                Algorithms.DecomposeKey(groupPair, out int groupId1, out int groupId2);
                string groupName1 = triangleGroupToName[groupId1];
                string groupName2 = triangleGroupToName[groupId2];
                string baseName = EntityNaming.FormatGroupEdgeName(groupName1, groupName2);
                
                // Use HashSegmentConnector to find connected components
                var connectedComponents = HashSegmentConnector.ConnectAndResolve(
                    segments,
                    delegate (Int2 s) { return s.X; },
                    delegate (Int2 s) { return s.Y; },
                    out List<bool> listClosed);
                
                // Create a list to hold GroupEdge objects for this group pair
                var groupEdgesForPair = new List<GroupEdge>();
                
                // Create a separate GroupEdge for each connected component
                for (int i = 0; i < connectedComponents.Count; i++)
                {
                    var component = connectedComponents[i];
                    bool isClosed = listClosed[i];
                    
                    // Convert the connected vertex indices back to edge segments
                    var componentSegments = new List<Int2>();
                    for (int j = 0; j < component.Count - 1; j++)
                    {
                        componentSegments.Add(new Int2(component[j], component[j + 1]));
                    }
                    if (isClosed && component.Count > 0)
                    {
                        // For closed loops, add the closing segment
                        componentSegments.Add(new Int2(component[component.Count - 1], component[0]));
                    }
                    
                    bool preferLex = preferLexClosedLoopStarts
                        || EntityNaming.IsBlendOrChamferPatchName(groupName1)
                        || EntityNaming.IsBlendOrChamferPatchName(groupName2);

                    var groupEdge = new GroupEdge(baseName, groupId1, groupId2, componentSegments, points, pointsExact,
                        preferLexClosedLoopStart: preferLex);
                    if (surfaceMetaData != null &&
                        TryResolveClosedLoopZeroFrame(groupName1, groupName2, surfaceMetaData, out var zeroFrame))
                    {
                        groupEdge.ClosedLoopZeroFrame = zeroFrame;
                        // Ctor already canonicalized before the frame was set — clear lock and rebuild
                        // (Axis used for winding; RefDir start skipped when PreferLexClosedLoopStart).
                        groupEdge.PreferredClosedLoopStartVertex = null;
                        groupEdge.PreferredClosedLoopNextVertex = null;
                        if (pointsExact != null && points != null)
                            groupEdge.UpdateExactPositions(points, pointsExact);
                        else if (points != null)
                            groupEdge.UpdatePositions(points);
                    }
                    groupEdgesForPair.Add(groupEdge);
                }
                
                // Sort the edges spatially for consistent ordering (only if points are available)
                if (points != null)
                {
                    SortGroupEdgesSpatially(groupEdgesForPair, points);
                }
                
                // Add suffix to names for edges after the first one
                for (int i = 0; i < groupEdgesForPair.Count; i++)
                {
                    if (i > 0)
                    {
                        groupEdgesForPair[i].Name = EntityNaming.FormatGroupEdgeName(groupName1, groupName2, i);
                    }
                    result.Add(groupEdgesForPair[i]);
                }
            }

            return result;
        }
        
        /// <summary>
        /// Sorts GroupEdge objects spatially based on their smallest point
        /// </summary>
        private static void SortGroupEdgesSpatially(List<GroupEdge> edges, List<Vec3D> points)
        {
            edges.Sort((a, b) =>
            {
                // Get the smallest point in each edge
                Vec3D minPointA = GetSmallestPointInGroupEdge(a, points);
                Vec3D minPointB = GetSmallestPointInGroupEdge(b, points);
                
                // Compare using lexicographic ordering (X, then Y, then Z)
                if (minPointA.X != minPointB.X)
                    return minPointA.X.CompareTo(minPointB.X);
                if (minPointA.Y != minPointB.Y)
                    return minPointA.Y.CompareTo(minPointB.Y);
                return minPointA.Z.CompareTo(minPointB.Z);
            });
        }
        
        /// <summary>
        /// Gets the smallest point (lexicographically) in a GroupEdge
        /// </summary>
        private static Vec3D GetSmallestPointInGroupEdge(GroupEdge edge, List<Vec3D> points)
        {
            Vec3D? smallest = null;
            
            foreach (var segment in edge.EdgeSegments)
            {
                Vec3D p1 = points[segment.X];
                Vec3D p2 = points[segment.Y];
                
                if (smallest == null || IsPointGeometricallySmaller(p1, smallest.Value))
                    smallest = p1;
                if (IsPointGeometricallySmaller(p2, smallest.Value))
                    smallest = p2;
            }
            
            return smallest ?? new Vec3D(0, 0, 0);
        }
        
        /// <summary>
        /// Checks if point a is geometrically smaller than point b (lexicographic ordering)
        /// </summary>
        private static bool IsPointGeometricallySmaller(Vec3D a, Vec3D b)
        {
            if (a.X != b.X)
                return a.X < b.X;
            if (a.Y != b.Y)
                return a.Y < b.Y;
                return a.Z < b.Z;
        }

        /// <summary>
        /// Prefer cylindrical RefDir (sketch circle angle 0), else planar RefDir from either adjacent patch.
        /// </summary>
        private static bool TryResolveClosedLoopZeroFrame(
            string groupName1,
            string groupName2,
            Dictionary<string, SurfaceMetaData> surfaceMetaData,
            out ClosedLoopZeroFrame frame)
        {
            frame = default;
            if (surfaceMetaData == null)
                return false;

            // Cylinder RefDir is the authoritative circle angle-0 direction.
            if (TryFrameFromCylinder(groupName1, surfaceMetaData, out frame) ||
                TryFrameFromCylinder(groupName2, surfaceMetaData, out frame))
                return true;

            if (TryFrameFromPlane(groupName1, surfaceMetaData, out frame) ||
                TryFrameFromPlane(groupName2, surfaceMetaData, out frame))
                return true;

            return false;
        }

        private static bool TryFrameFromCylinder(
            string groupName,
            Dictionary<string, SurfaceMetaData> surfaceMetaData,
            out ClosedLoopZeroFrame frame)
        {
            frame = default;
            if (!surfaceMetaData.TryGetValue(groupName, out var meta) || meta == null)
                return false;
            if (meta.SurfaceType != SurfaceType.Cylindrical || meta.CylinderParams == null)
                return false;

            var c = meta.CylinderParams;
            if (c.RefDir.LengthSquared() < 1e-20 || c.Axis.LengthSquared() < 1e-20)
                return false;
            frame = new ClosedLoopZeroFrame(c.RefDir.Normalized(), c.Axis.Normalized());
            return true;
        }

        private static bool TryFrameFromPlane(
            string groupName,
            Dictionary<string, SurfaceMetaData> surfaceMetaData,
            out ClosedLoopZeroFrame frame)
        {
            frame = default;
            if (!surfaceMetaData.TryGetValue(groupName, out var meta) || meta == null)
                return false;
            if (meta.SurfaceType != SurfaceType.Planar || meta.PlaneParams == null)
                return false;

            var p = meta.PlaneParams;
            if (p.RefDir.LengthSquared() < 1e-20 || p.Normal.LengthSquared() < 1e-20)
                return false;
            frame = new ClosedLoopZeroFrame(p.RefDir.Normalized(), p.Normal.Normalized());
            return true;
        }

    }

    /// <summary>
    /// Preferred angular frame for closed group-edge loops (typically circular).
    /// <see cref="RefDir"/> is angle 0; <see cref="Axis"/> defines the loop plane / winding sense.
    /// </summary>
    public readonly struct ClosedLoopZeroFrame
    {
        public readonly Vec3D RefDir;
        public readonly Vec3D Axis;

        public ClosedLoopZeroFrame(Vec3D refDir, Vec3D axis)
        {
            RefDir = refDir;
            Axis = axis;
        }
    }

    public class GroupEdge
    {
        public string Name { get; set; }
        public readonly int GroupIdA;
        public readonly int GroupIdB;
        public readonly List<Int2> EdgeSegments;
        public List<LineStrip3D> LineStrips3D = new List<LineStrip3D>();
        public List<List<Rat3Hybrid>> LineStripsExact = new List<List<Rat3Hybrid>>();

        /// <summary>
        /// When set, closed loops are phased so uniform 0 lies along RefDir from the loop centroid
        /// (sketch / cylinder angle 0), instead of the world lexicographic vertex.
        /// </summary>
        public ClosedLoopZeroFrame? ClosedLoopZeroFrame { get; set; }

        public EdgeMetaData MetaData { get; set; }

        /// <summary>
        /// Implicit curves (CSG / blend / chamfer): use world lex-min start instead of RefDir.
        /// </summary>
        public bool PreferLexClosedLoopStart { get; set; }

        /// <summary>
        /// Mesh vertex index at uniform 0 after the first canonicalization. Survives rigid transforms
        /// (unlike world-space <see cref="ClosedLoopZeroFrame"/>).
        /// </summary>
        public int? PreferredClosedLoopStartVertex { get; set; }

        /// <summary>
        /// Mesh vertex immediately after the start along the chosen winding.
        /// </summary>
        public int? PreferredClosedLoopNextVertex { get; set; }

        public GroupEdge(string name, int groupIdA, int groupIdB, List<Int2> edgeSegments, List<Vec3D> points, List<Rat3Hybrid> pointsExact,
            bool preferLexClosedLoopStart = false)
        {
            Name = name;
            GroupIdA = groupIdA;
            GroupIdB = groupIdB;
            EdgeSegments = edgeSegments;
            PreferLexClosedLoopStart = preferLexClosedLoopStart;
           
            if(pointsExact != null && points != null)
            {
                UpdateExactPositions(points, pointsExact);
            }
            else if (points != null)
            {
                UpdatePositions(points);
            }
        }

        public void UpdatePositions(List<Vec3D> points)
        {
            var strips = GetLineStrips(points);
            LineStrips3D = new List<LineStrip3D>(strips.Count);
            for (int i = 0; i < strips.Count; i++)
            {
                var strip = strips[i];
                List<Vec3D> stripPoints = new List<Vec3D>(strip.Count);
                for (int j = 0; j < strip.Count; j++)
                    stripPoints.Add(points[strip[j]]);
                LineStrips3D.Add(new LineStrip3D(stripPoints));
            }
        }
        public void UpdateExactPositions(List<Vec3D> points, List<Rat3Hybrid> pointsExact)
        {
            var strips = GetLineStrips(points);
            LineStripsExact = new List<List<Rat3Hybrid>>(strips.Count);
            LineStrips3D = new List<LineStrip3D>(strips.Count);
            for (int i = 0; i < strips.Count; i++)
            {
                var strip = strips[i];
                List<Rat3Hybrid> stripPointsExact = new List<Rat3Hybrid>(strip.Count);
                List<Vec3D> stripPoints = new List<Vec3D>(strip.Count);
                for (int j = 0; j < strip.Count; j++)
                {
                    stripPointsExact.Add(pointsExact[strip[j]]);
                    stripPoints.Add(points[strip[j]]);
                }
                LineStripsExact.Add(stripPointsExact);
                LineStrips3D.Add(new LineStrip3D(stripPoints));
            }
        }


        // Make order consistent of strips. Make orientation of strips consistent. Make start of closed loops consistent. 
        // This is all based on geometric criteria (not just on indices).
        private List<List<int>> GetLineStrips(List<Vec3D> points)
        {
            var result = HashSegmentConnector.ConnectAndResolve(EdgeSegments,
                delegate (Int2 s) { return s.X; },
                delegate (Int2 s) { return s.Y; },
                out List<bool> listClosed);

            // Apply geometric consistency to each strip
            for (int i = 0; i < result.Count; ++i)
            {
                var strip = result[i];
                bool isClosed = listClosed[i];
                
                if (isClosed)
                {
                    // For closed loops: find the geometrically "best" starting point
                    MakeClosedLoopConsistent(strip, points);
                    // Add the first point again to close the loop
                    strip.Add(strip[0]);
                }
                else
                {
                    // For open strips: ensure consistent orientation
                    MakeOpenStripConsistent(strip, points);
                }
            }

            // Sort strips by geometric criteria for consistent ordering
            SortStripsByGeometry(result, listClosed, points);

            return result;
        }

        private void MakeClosedLoopConsistent(List<int> strip, List<Vec3D> points)
        {
            if (strip.Count < 3) return;

            // Prefer a previously locked seam (stable across rigid transforms).
            if (PreferredClosedLoopStartVertex.HasValue &&
                TryApplyPreferredClosedLoopVertices(strip))
            {
                return;
            }

            Vec3D loopCentroid = ComputeLoopCentroid(strip, points);
            Vec3D orientationAxis = ClosedLoopZeroFrame?.Axis ?? CalculatePolygonNormal(strip, points);

            // Orient first, then choose start — Reverse() would otherwise move the start vertex.
            EnsureConsistentOrientation(strip, points, orientationAxis, loopCentroid);

            bool useLexStart = PreferLexClosedLoopStart
                || !ClosedLoopZeroFrame.HasValue
                || ClosedLoopZeroFrame.Value.RefDir.LengthSquared() <= 1e-20;

            int bestStartIndex;
            if (!useLexStart)
            {
                // Authored parametric zero: vertex most aligned with RefDir from centroid.
                Vec3D refDir = ClosedLoopZeroFrame.Value.RefDir.Normalized();
                double bestScore = double.NegativeInfinity;
                bestStartIndex = 0;
                Vec3D bestPoint = points[strip[0]];

                for (int i = 0; i < strip.Count; i++)
                {
                    Vec3D currentPoint = points[strip[i]];
                    Vec3D radial = currentPoint - loopCentroid;
                    double radialLenSq = radial.LengthSquared();
                    if (radialLenSq < 1e-20)
                        continue;

                    double score = Vec3DOps.Dot(radial / Math.Sqrt(radialLenSq), refDir);
                    if (score > bestScore + 1e-12 ||
                        (Math.Abs(score - bestScore) <= 1e-12 && IsPointGeometricallySmaller(currentPoint, bestPoint)))
                    {
                        bestScore = score;
                        bestStartIndex = i;
                        bestPoint = currentPoint;
                    }
                }
            }
            else
            {
                bestStartIndex = FindLexMinStripIndex(strip, points, throwOnDuplicateMin: PreferLexClosedLoopStart);
            }

            RotateStripToStart(strip, bestStartIndex);
            LockPreferredClosedLoopVertices(strip);
        }

        /// <summary>
        /// World lex-min vertex index in the strip (x, then y, then z).
        /// When <paramref name="throwOnDuplicateMin"/>, throws if another vertex shares that exact position.
        /// </summary>
        private int FindLexMinStripIndex(List<int> strip, List<Vec3D> points, bool throwOnDuplicateMin)
        {
            int bestStartIndex = 0;
            Vec3D bestPoint = points[strip[0]];
            for (int i = 1; i < strip.Count; i++)
            {
                Vec3D currentPoint = points[strip[i]];
                if (IsPointGeometricallySmaller(currentPoint, bestPoint))
                {
                    bestStartIndex = i;
                    bestPoint = currentPoint;
                }
            }

            if (throwOnDuplicateMin)
            {
                for (int i = 0; i < strip.Count; i++)
                {
                    if (i == bestStartIndex)
                        continue;
                    Vec3D p = points[strip[i]];
                    if (p.X == bestPoint.X && p.Y == bestPoint.Y && p.Z == bestPoint.Z)
                    {
                        throw new InvalidOperationException(
                            $"Closed group-edge '{Name}' has duplicate vertices at lex-min start {bestPoint}; " +
                            "cannot uniquely choose uniform 0.");
                    }
                }
            }

            return bestStartIndex;
        }

        private bool TryApplyPreferredClosedLoopVertices(List<int> strip)
        {
            int startVertex = PreferredClosedLoopStartVertex.Value;
            int startIndex = strip.IndexOf(startVertex);
            if (startIndex < 0)
                return false;

            RotateStripToStart(strip, startIndex);

            if (PreferredClosedLoopNextVertex.HasValue && strip.Count > 1)
            {
                int nextVertex = PreferredClosedLoopNextVertex.Value;
                if (strip[1] != nextVertex)
                {
                    // Wrong winding: reverse, then restore start.
                    strip.Reverse();
                    startIndex = strip.IndexOf(startVertex);
                    if (startIndex < 0)
                        return false;
                    RotateStripToStart(strip, startIndex);
                }
            }

            return true;
        }

        private void LockPreferredClosedLoopVertices(List<int> strip)
        {
            if (strip.Count < 2)
                return;
            PreferredClosedLoopStartVertex = strip[0];
            PreferredClosedLoopNextVertex = strip[1];
        }

        private static void RotateStripToStart(List<int> strip, int bestStartIndex)
        {
            if (bestStartIndex == 0)
                return;

            var rotatedStrip = new List<int>(strip.Count);
            for (int i = bestStartIndex; i < strip.Count; i++)
                rotatedStrip.Add(strip[i]);
            for (int i = 0; i < bestStartIndex; i++)
                rotatedStrip.Add(strip[i]);

            strip.Clear();
            strip.AddRange(rotatedStrip);
        }

        private static Vec3D ComputeLoopCentroid(List<int> strip, List<Vec3D> points)
        {
            Vec3D sum = new Vec3D(0, 0, 0);
            for (int i = 0; i < strip.Count; i++)
                sum += points[strip[i]];
            return sum / strip.Count;
        }

        private void MakeOpenStripConsistent(List<int> strip, List<Vec3D> points)
        {
            if (strip.Count < 2) return;

            // For open strips, ensure the strip goes from the geometrically smaller endpoint
            // to the geometrically larger endpoint
            Vec3D startPoint = points[strip[0]];
            Vec3D endPoint = points[strip[strip.Count - 1]];

            if (IsPointGeometricallySmaller(endPoint, startPoint))
            {
                // Reverse the strip
                strip.Reverse();
            }
        }

        private void EnsureConsistentOrientation(List<int> strip, List<Vec3D> points, Vec3D preferredAxis, Vec3D origin)
        {
            if (strip.Count < 3) return;

            Vec3D axis = preferredAxis;
            if (axis.LengthSquared() < 1e-20)
                axis = CalculatePolygonNormal(strip, points);
            if (axis.LengthSquared() < 1e-20)
                return;
            axis = axis.Normalized();

            // Signed projected area around the preferred axis (RH with respect to axis).
            double wind = 0;
            for (int i = 0; i < strip.Count; i++)
            {
                int j = (i + 1) % strip.Count;
                Vec3D a = points[strip[i]] - origin;
                Vec3D b = points[strip[j]] - origin;
                wind += Vec3DOps.Dot(Vec3DOps.Cross(a, b), axis);
            }

            // Prefer counterclockwise when viewed along +axis (positive wind).
            if (wind < 0)
                strip.Reverse();
        }

        private Vec3D CalculatePolygonNormal(List<int> strip, List<Vec3D> points)
        {
            // Use Newell's method to calculate polygon normal robustly
            Vec3D normal = new Vec3D(0, 0, 0);
            
            for (int i = 0; i < strip.Count; i++)
            {
                int j = (i + 1) % strip.Count;
                Vec3D pi = points[strip[i]];
                Vec3D pj = points[strip[j]];
                
                normal.X += (pi.Y - pj.Y) * (pi.Z + pj.Z);
                normal.Y += (pi.Z - pj.Z) * (pi.X + pj.X);
                normal.Z += (pi.X - pj.X) * (pi.Y + pj.Y);
            }
            
            return normal;
        }

        private void SortStripsByGeometry(List<List<int>> strips, List<bool> listClosed, List<Vec3D> points)
        {
            // Create indices for sorting
            var stripIndices = new List<int>();
            for (int i = 0; i < strips.Count; i++)
                stripIndices.Add(i);

            // Sort by geometric criteria:
            // 1. Closed loops first, then open strips
            // 2. Within each category, sort by the geometrically smallest point in the strip
            stripIndices.Sort((a, b) =>
            {
                bool aIsClosed = listClosed[a];
                bool bIsClosed = listClosed[b];
                
                // Closed loops come first
                if (aIsClosed != bIsClosed)
                    return bIsClosed.CompareTo(aIsClosed);

                // Within the same category, sort by the smallest point in each strip
                Vec3D minPointA = GetSmallestPointInStrip(strips[a], points);
                Vec3D minPointB = GetSmallestPointInStrip(strips[b], points);
                
                return ComparePointsGeometrically(minPointA, minPointB);
            });

            // Reorder the strips and closed flags based on the sorted indices
            var sortedStrips = new List<List<int>>();
            var sortedClosed = new List<bool>();
            
            foreach (int index in stripIndices)
            {
                sortedStrips.Add(strips[index]);
                sortedClosed.Add(listClosed[index]);
            }

            // Replace the original lists
            strips.Clear();
            strips.AddRange(sortedStrips);
            listClosed.Clear();
            listClosed.AddRange(sortedClosed);
        }

        private Vec3D GetSmallestPointInStrip(List<int> strip, List<Vec3D> points)
        {
            if (strip.Count == 0) return new Vec3D(0, 0, 0);
            
            Vec3D smallest = points[strip[0]];
            for (int i = 1; i < strip.Count; i++)
            {
                Vec3D current = points[strip[i]];
                if (IsPointGeometricallySmaller(current, smallest))
                    smallest = current;
            }
            return smallest;
        }

        private bool IsPointGeometricallySmaller(Vec3D a, Vec3D b)
        {
            if (a.X != b.X)
                return a.X < b.X;
            if (a.Y != b.Y)
                return a.Y < b.Y;
            return a.Z < b.Z;
        }

        private int ComparePointsGeometrically(Vec3D a, Vec3D b)
        {
            if (a.X != b.X)
                return a.X.CompareTo(b.X);
            if (a.Y != b.Y)
                return a.Y.CompareTo(b.Y);
            return a.Z.CompareTo(b.Z);
        }

        public List<List<int>> GetLineStripsOriginal(List<Vec3D> points)
        {
            var result = HashSegmentConnector.ConnectAndResolve(EdgeSegments,
                delegate (Int2 s) { return s.X; },
                delegate (Int2 s) { return s.Y; },
                out List<bool> listClosed);

            for (int i = 0; i < listClosed.Count; ++i)
            {
                if (listClosed[i])
                    result[i].Add(result[i][0]);
            }

            return result;
        }
    }

    /// <summary>
    /// Represents an edge in the edge graph with start and end nodes.
    /// Contains all GroupEdge properties including EdgeSegments.
    /// </summary>
    public class GraphEdge
    {
        public int Id { get; set; }
        public string Name { get; set; }
        public int GroupIdA { get; set; }
        public int GroupIdB { get; set; }
        public List<Int2> EdgeSegments { get; set; }
        public LineStrip3D LineStrip { get; set; }
        public List<Rat3Hybrid> LineStripExact { get; set; } = null!;
        public EdgeGraphNode StartNode { get; set; } = null!;
        public EdgeGraphNode EndNode { get; set; } = null!;
        public int StartNodeId { get; set; }
        public int EndNodeId { get; set; }
        public EdgeBlendType BlendType { get; set; }

        public GraphEdge(string name, int groupIdA, int groupIdB, List<Int2> edgeSegments, LineStrip3D lineStrip, List<Rat3Hybrid> lineStripExact)
        {
            Name = name;
            GroupIdA = groupIdA;
            GroupIdB = groupIdB;
            EdgeSegments = edgeSegments;
            LineStrip = lineStrip;
            LineStripExact = lineStripExact;
        }
    }

    /// <summary>
    /// Represents a node in the edge graph. A node is a point where one or more edges meet.
    /// </summary>
    public class EdgeGraphNode
    {
        public int Id { get; set; }
        public Vec3D Position { get; set; }
        public Rat3Hybrid PositionExact { get; set; }
        public List<int> ConnectedEdgeIndices { get; private set; }
        public List<GraphEdge> ConnectedEdges { get; private set; }

        public EdgeGraphNode(Vec3D position, Rat3Hybrid positionExact)
        {
            Position = position; 
            PositionExact = positionExact;
            ConnectedEdgeIndices = new List<int>();
            ConnectedEdges = new List<GraphEdge>();           
        }

        public void AddConnectedEdge(int edgeIndex, GraphEdge edge)
        {
            if (!ConnectedEdgeIndices.Contains(edgeIndex))
            {
                ConnectedEdgeIndices.Add(edgeIndex);
                ConnectedEdges.Add(edge);
            }
        }
    }

    /// <summary>
    /// Represents a complete graph of all edges of a body.
    /// Nodes represent connection points, edges are represented by GraphEdge objects (split into single LineStrips).
    /// Uses EXACT ARITHMETIC for perfect robustness - no tolerance values!
    /// </summary>
    public class EdgeGraph
    {
        public List<EdgeGraphNode> Nodes { get; private set; }
        public List<GraphEdge> Edges { get; private set; }
        
        // Dictionary for O(1) node lookup by exact position
        private Dictionary<Rat3Hybrid, int> positionToNodeIndex;

        public EdgeGraph(List<Tri> triangles, List<int> groupIdPerTriangle, 
            List<Vec3D> points, List<Rat3Hybrid> pointsExact, Dictionary<int, string> triangleGroupToName)
        {
            Nodes = new List<EdgeGraphNode>();
            Edges = new List<GraphEdge>();
            positionToNodeIndex = new Dictionary<Rat3Hybrid, int>();

            // Extract group edges using the existing function
            var groupEdges = GroupEdgeExtractor.ExtractGroupEdges(
                triangles, groupIdPerTriangle, points, pointsExact, triangleGroupToName);

            // Split each GroupEdge into individual GraphEdges (one per LineStrip3D)
            Split(groupEdges);

            // Build nodes from edge endpoints and set node references in edges
            BuildNodes();
        }

        /// <summary>
        /// Converts GroupEdges into GraphEdges.
        /// With the new extraction logic, each GroupEdge should contain exactly one LineStrip3D,
        /// but we still handle multiple strips for backward compatibility.
        /// </summary>
        private void Split(List<GroupEdge> groupEdges)
        {
            foreach (var groupEdge in groupEdges)
            {
                if (groupEdge.LineStrips3D.Count == 0)
                    continue;
                    
                if (groupEdge.LineStrips3D.Count == 1)
                {
                    // Normal case: one line strip per GroupEdge (after our extraction changes)
                    var lineStrip = groupEdge.LineStrips3D[0];
                    var lineStripExact = groupEdge.LineStripsExact.Count > 0 ? groupEdge.LineStripsExact[0] : null;

                    var graphEdge = new GraphEdge(
                        groupEdge.Name,  // Use the name as-is
                        groupEdge.GroupIdA,
                        groupEdge.GroupIdB,
                        groupEdge.EdgeSegments,
                        lineStrip,
                        lineStripExact);
                    
                    graphEdge.Id = Edges.Count;
                    Edges.Add(graphEdge);
                }
                else
                {
                    // Backward compatibility: if somehow there are multiple line strips in one GroupEdge
                    // This should not happen with the new extraction logic
                    for (int i = 0; i < groupEdge.LineStrips3D.Count; i++)
                    {
                        var lineStrip = groupEdge.LineStrips3D[i];
                        var lineStripExact = i < groupEdge.LineStripsExact.Count ? groupEdge.LineStripsExact[i] : null;

                        string name = groupEdge.Name;
                        if (i > 0)
                            name += "_" + i;

                        var graphEdge = new GraphEdge(
                            name,
                            groupEdge.GroupIdA,
                            groupEdge.GroupIdB,
                            groupEdge.EdgeSegments,
                            lineStrip,
                            lineStripExact);
                        
                        graphEdge.Id = Edges.Count;
                        Edges.Add(graphEdge);
                    }
                }
            }
        }

        /// <summary>
        /// Builds the node graph by finding unique endpoints of all edges and sets node references in edges.
        /// Uses EXACT ARITHMETIC for perfect robustness - no tolerance values!
        /// Dictionary-based lookup for O(1) performance instead of O(n).
        /// </summary>
        private void BuildNodes()
        {
            for (int edgeIndex = 0; edgeIndex < Edges.Count; edgeIndex++)
            {
                var edge = Edges[edgeIndex];
                var lineStripExact = edge.LineStripExact;
                
                if (lineStripExact == null || lineStripExact.Count == 0)
                    continue;

                // Get start and end points of the line strip using EXACT positions
                Rat3Hybrid startPointExact = lineStripExact[0];
                Rat3Hybrid endPointExact = lineStripExact[lineStripExact.Count - 1];
                
                Vec3D startPoint = edge.LineStrip.Points[0];
                Vec3D endPoint = edge.LineStrip.Points[edge.LineStrip.Points.Count - 1];

                // Add or find the start node using exact arithmetic
                int startNodeIndex = GetOrCreateNode(startPoint, startPointExact);
                EdgeGraphNode startNode = Nodes[startNodeIndex];
                startNode.AddConnectedEdge(edgeIndex, edge);
                edge.StartNode = startNode;
                edge.StartNodeId = startNodeIndex;

                // Add or find the end node (if different from start)
                // Uses EXACT comparison - no tolerance!
                if (!ArePositionsEqual(startPointExact, endPointExact))
                {
                    int endNodeIndex = GetOrCreateNode(endPoint, endPointExact);
                    EdgeGraphNode endNode = Nodes[endNodeIndex];
                    endNode.AddConnectedEdge(edgeIndex, edge);
                    edge.EndNode = endNode;
                    edge.EndNodeId = endNodeIndex;
                }
                else
                {
                    // Closed loop - start and end are the same
                    edge.EndNode = startNode;
                    edge.EndNodeId = startNodeIndex;
                }
            }
        }

        /// <summary>
        /// Gets an existing node or creates a new one for the given position.
        /// Uses exact arithmetic with dictionary for O(1) lookup - fully robust!
        /// </summary>
        private int GetOrCreateNode(Vec3D position, Rat3Hybrid positionExact)
        {
            // Try to find existing node using EXACT position as key
            if (positionToNodeIndex.TryGetValue(positionExact, out int existingIndex))
            {
                return existingIndex;
            }

            // Create a new node with both approximate and exact positions
            var newNode = new EdgeGraphNode(position, positionExact);
            int newIndex = Nodes.Count;
            newNode.Id = newIndex;
            Nodes.Add(newNode);
            positionToNodeIndex[positionExact] = newIndex;
            return newIndex;
        }

        /// <summary>
        /// Checks if two positions are equal within tolerance.
        /// </summary>
        private bool ArePositionsEqual(Rat3Hybrid a, Rat3Hybrid b)
        {
            return a == b;
        }

        /// <summary>
        /// Gets all edges connected to a specific node.
        /// </summary>
        public List<GraphEdge> GetEdgesForNode(int nodeIndex)
        {
            if (nodeIndex < 0 || nodeIndex >= Nodes.Count)
                return new List<GraphEdge>();

            var result = new List<GraphEdge>();
            var node = Nodes[nodeIndex];

            foreach (int edgeIndex in node.ConnectedEdgeIndices)
            {
                if (edgeIndex >= 0 && edgeIndex < Edges.Count)
                {
                    result.Add(Edges[edgeIndex]);
                }
            }

            return result;
        }

        /// <summary>
        /// Finds the node index for a given exact position in O(1) time using dictionary lookup.
        /// Uses EXACT ARITHMETIC - no tolerance values!
        /// </summary>
        /// <param name="positionExact">The exact position to find</param>
        /// <returns>The node index if found, -1 otherwise</returns>
        public int FindNodeIndex(Rat3Hybrid positionExact)
        {
            if (positionToNodeIndex.TryGetValue(positionExact, out int nodeIndex))
            {
                return nodeIndex;
            }
            return -1;
        }

        /// <summary>
        /// Try to find an edge by its name.
        /// Supports names like "[SurfaceA,SurfaceB]" or "[SurfaceA,SurfaceB]_0" or full edge names.
        /// </summary>
        /// <param name="edgeName">Name of the edge to find</param>
        /// <param name="edge">The found edge, or null if not found</param>
        /// <returns>True if the edge was found, false otherwise</returns>
        public bool TryGetEdge(string edgeName, out GraphEdge edge)
        {
            edge = null;
            
            if (string.IsNullOrEmpty(edgeName))
                return false;

            // Try exact match first
            foreach (var e in Edges)
            {
                if (EntityNaming.MatchesGroupEdgeName(e.Name, edgeName))
                {
                    edge = e;
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Get all edges between two surface groups (by group ID).
        /// </summary>
        /// <param name="groupIdA">First group ID</param>
        /// <param name="groupIdB">Second group ID</param>
        /// <returns>List of edges between the two groups</returns>
        public List<GraphEdge> GetEdgesBetweenGroups(int groupIdA, int groupIdB)
        {
            List<GraphEdge> result = new List<GraphEdge>();
            
            foreach (var edge in Edges)
            {
                if ((edge.GroupIdA == groupIdA && edge.GroupIdB == groupIdB) ||
                    (edge.GroupIdA == groupIdB && edge.GroupIdB == groupIdA))
                {
                    result.Add(edge);
                }
            }
            
            return result;
        }
    }
}
