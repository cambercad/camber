#pragma warning disable CS8603 // Possible null reference return
#pragma warning disable CS8625 // Cannot convert null literal to non-nullable reference type
#pragma warning disable CS8618 // Non-nullable property must contain a non-null value when exiting constructor

using CSG;
using GeoCore;
using System.Collections.Generic;

namespace Geo
{
    /// <summary>
    /// Represents a rounded edge blend between two surfaces.
    /// Stores the center curve, the two boundary curves, and connection information to corners.
    /// </summary>
    public class BlendEdge
    {
        public int StartCornerId { get; set; }
        public int EndCornerId { get; set; }

        public GraphEdge SourceEdge { get; set; }
        public int SurfaceIndexA { get; set; }
        public int SurfaceIndexB { get; set; }
        public double BlendRadius { get; set; }
        internal bool HasPlanarSourceFaces { get; private set; }
        
        // The three defining curves of the cylindrical blend
        public List<Rat3Hybrid> CenterCurve { get; set; }
        public List<Vec3D> CenterCurveVec3 { get; set; }
        public List<Rat3Hybrid> ProjectedBoundaryCurveA { get; set; } // On surface A (for visualization/generation)
        public List<Vec3D> ProjectedBoundaryCurveAVec3D { get; set; }
        public List<Rat3Hybrid> ProjectedBoundaryCurveB { get; set; } // On surface B (for visualization/generation)
        public List<Vec3D> ProjectedBoundaryCurveBVec3D { get; set; }

        // Intersection segments with barycentric coordinates (for exact trimming)
        public List<LineSegmentOnTriangleEx> BoundarySegmentsA { get; set; }
        public List<LineSegmentOnTriangleEx> BoundarySegmentsB { get; set; }

        // The raw cylindrical surface (untrimmed)
        public UVSurface RawSurface { get; private set; }

        // The tessellated and trimmed cylindrical surface
        public UVSurface BlendSurface { get; private set; }
        public UVSurface EdgeStartGapFillSurface;
        public UVSurface EdgeEndGapFillSurface;

        public List<List<Rat3Hybrid>> TrimArcs { get; private set; }

        // Metadata about whether this edge adds or removes material
        public EdgeBlendType BlendType { get; set; }
        
        // Store the coordinate converter for conversions
        private CoordinateConverter cc;
        private UVSurface supportA, supportB;

        public BlendEdge(GraphEdge sourceEdge, int surfaceIndexA, int surfaceIndexB, double blendRadius)
        {
            // Validation
            if (sourceEdge == null)
                throw new ArgumentNullException(nameof(sourceEdge));
            if (blendRadius <= 0)
                throw new ArgumentException("Blend radius must be positive", nameof(blendRadius));
            if (surfaceIndexA == surfaceIndexB)
                throw new ArgumentException("Surface indices must be different");
                
            SourceEdge = sourceEdge;
            SurfaceIndexA = surfaceIndexA;
            SurfaceIndexB = surfaceIndexB;
            BlendRadius = blendRadius;
            BlendType = EdgeBlendType.Concave; // Default

            // Initialize to default values (will be set later during computation)
            CenterCurve = new List<Rat3Hybrid>();
            ProjectedBoundaryCurveA = new List<Rat3Hybrid>();
            ProjectedBoundaryCurveB = new List<Rat3Hybrid>();
            BoundarySegmentsA = new List<LineSegmentOnTriangleEx>();
            BoundarySegmentsB = new List<LineSegmentOnTriangleEx>();
            RawSurface = null;
            BlendSurface = null;
            EdgeStartGapFillSurface = null;
            EdgeEndGapFillSurface = null;

            TrimArcs = new List<List<Rat3Hybrid>>();
        }

        /// <summary>
        /// Compute spine and boundary curves by intersecting offset surfaces.
        /// </summary>
        public bool ComputeSpineAndBoundaries(
            UVSurface originalSurfaceA,
            UVSurface originalSurfaceB,
            UVSurface enlargedSurfaceA,
            UVSurface enlargedSurfaceB,
            UVSurface enlargedAndOffsetSurfaceA,
            UVSurface enlargedAndOffsetSurfaceB,
            CoordinateConverter cc)
        {
            this.cc = cc;
            supportA = enlargedSurfaceA;
            supportB = enlargedSurfaceB;
            HasPlanarSourceFaces = originalSurfaceA.IsSurfacePlanar() && originalSurfaceB.IsSurfacePlanar();

            var intersectionStrips = Intersector.IntersectSurfaceStrips(
                enlargedAndOffsetSurfaceA, enlargedAndOffsetSurfaceB, cc);
            if (intersectionStrips.Count == 0)
                return false;

            (List<LineSegmentOnTriangleEx> intersectionA, List<LineSegmentOnTriangleEx> intersectionB) selected;
            // Extended faces may intersect on unrelated open branches. A closed
            // target must correspond to one closed contour, not their concatenation.
            if (SourceEdge.LineStripExact[0] == SourceEdge.LineStripExact[^1])
            {
                var closedContours = new List<(List<LineSegmentOnTriangleEx> intersectionA,
                    List<LineSegmentOnTriangleEx> intersectionB)>();
                foreach (var strip in intersectionStrips)
                    if (strip.intersectionA.Count > 0 &&
                        strip.intersectionA[0].PointStart == strip.intersectionA[^1].PointEnd)
                        closedContours.Add(strip);
                if (closedContours.Count != 1)
                    throw new InvalidOperationException($"Closed edge '{SourceEdge.Name}' requires one closed offset contour; found {closedContours.Count}.");
                selected = closedContours[0];
            }
            else
                selected = SelectOpenIntersectionStrip(intersectionStrips, enlargedSurfaceA, enlargedSurfaceB);

            List<LineSegmentOnTriangleEx> intersectA = selected.intersectionA;
            List<LineSegmentOnTriangleEx> intersectB = selected.intersectionB;

            CenterCurve = ExtractCurveFromSegments(intersectA);
            if (CenterCurve == null || CenterCurve.Count < 2)
                return false;

            BoundarySegmentsA = intersectA;
            BoundarySegmentsB = intersectB;
            ProjectedBoundaryCurveA = BarycentricProjection(intersectA, enlargedSurfaceA);
            ProjectedBoundaryCurveB = BarycentricProjection(intersectB, enlargedSurfaceB);

            for (int i = 0; i < ProjectedBoundaryCurveA.Count; ++i)
            {
                Rat3Hybrid p = ProjectedBoundaryCurveA[i];
                p.Simplify();
                ProjectedBoundaryCurveA[i] = p;
            }
            for (int i = 0; i < ProjectedBoundaryCurveB.Count; ++i)
            {
                Rat3Hybrid p = ProjectedBoundaryCurveB[i];
                p.Simplify();
                ProjectedBoundaryCurveB[i] = p;
            }

            CenterCurveVec3 = cc.Convert(CenterCurve);
            ProjectedBoundaryCurveAVec3D = cc.Convert(ProjectedBoundaryCurveA);
            ProjectedBoundaryCurveBVec3D = cc.Convert(ProjectedBoundaryCurveB);

            if (CenterCurve.Count != ProjectedBoundaryCurveA.Count)
                throw new Exception();
            if (CenterCurve.Count != ProjectedBoundaryCurveB.Count)
                throw new Exception();

            if (ProjectedBoundaryCurveA == null || ProjectedBoundaryCurveA.Count < 2 ||
                ProjectedBoundaryCurveB == null || ProjectedBoundaryCurveB.Count < 2)
                return false;

            return true;
        }

        private (List<LineSegmentOnTriangleEx> intersectionA, List<LineSegmentOnTriangleEx> intersectionB)
            SelectOpenIntersectionStrip(
                List<(List<LineSegmentOnTriangleEx> intersectionA, List<LineSegmentOnTriangleEx> intersectionB)> strips,
                UVSurface enlargedSurfaceA, UVSurface enlargedSurfaceB)
        {
            // With one contour there is no correspondence choice to make.
            // Keep the exact distance comparison for genuinely competing strips.
            if (strips.Count == 1 && strips[0].intersectionA.Count > 0)
                return strips[0];

            BigRationalHybrid? bestScore = null;
            (List<LineSegmentOnTriangleEx> intersectionA, List<LineSegmentOnTriangleEx> intersectionB) best = default;
            bool ambiguous = false;
            foreach (var strip in strips)
            {
                if (strip.intersectionA.Count == 0) continue;
                var projectedA = BarycentricProjection(strip.intersectionA, enlargedSurfaceA);
                var projectedB = BarycentricProjection(strip.intersectionB, enlargedSurfaceB);
                var score = MaxSourceEdgeDeviationSquared(projectedA, projectedB, SourceEdge.LineStripExact);
                int comparison = bestScore.HasValue ? score.CompareTo(bestScore.Value) : -1;
                if (comparison < 0)
                {
                    bestScore = score;
                    best = strip;
                    ambiguous = false;
                }
                else if (comparison == 0)
                    ambiguous = true;
            }

            if (!bestScore.HasValue)
                throw new InvalidOperationException($"Open edge '{SourceEdge.Name}' has no offset intersection contour.");
            if (ambiguous)
                throw new InvalidOperationException(
                    $"Open edge '{SourceEdge.Name}' matches multiple offset contours equally; correspondence is ambiguous.");
            return best;
        }

        internal static BigRationalHybrid MaxSourceEdgeDeviationSquared(List<Rat3Hybrid> curveA,
            List<Rat3Hybrid> curveB, List<Rat3Hybrid> sourceEdge)
        {
            var maximum = BigRationalHybrid.Zero;
            MeasureDeviation(curveA, sourceEdge);
            MeasureDeviation(curveB, sourceEdge);
            MeasureDeviation(sourceEdge, curveA);
            MeasureDeviation(sourceEdge, curveB);
            return maximum;

            void MeasureDeviation(List<Rat3Hybrid> from, List<Rat3Hybrid> to)
            {
                foreach (var point in from)
                {
                    BigRationalHybrid? nearest = null;
                    for (int i = 1; i < to.Count; i++)
                    {
                        var start = to[i - 1];
                        var end = to[i];
                        // The distance to the segment cannot be smaller than
                        // the distance to its bounding box. Prove this bound
                        // exactly before doing the rational projection.
                        if (nearest.HasValue)
                        {
                            var lowerBound = BigRationalHybrid.Zero;
                            for (int axis = 0; axis < 3; axis++)
                            {
                                var low = start[axis] < end[axis] ? start[axis] : end[axis];
                                var high = start[axis] > end[axis] ? start[axis] : end[axis];
                                var outside = point[axis] < low ? low - point[axis] :
                                    point[axis] > high ? point[axis] - high : BigRationalHybrid.Zero;
                                lowerBound += outside * outside;
                                if (lowerBound >= nearest.Value) break;
                            }
                            if (lowerBound >= nearest.Value) continue;
                        }
                        var direction = end - start;
                        var lengthSquared = Rat3Hybrid.Dot(direction, direction);
                        if (lengthSquared == BigRationalHybrid.Zero) continue;
                        var t = Rat3Hybrid.Dot(point - start, direction) / lengthSquared;
                        if (t < BigRationalHybrid.Zero) t = BigRationalHybrid.Zero;
                        else if (t > BigRationalHybrid.One) t = BigRationalHybrid.One;
                        var delta = point - (start + t * direction);
                        var distanceSquared = Rat3Hybrid.Dot(delta, delta);
                        if (!nearest.HasValue || distanceSquared < nearest.Value)
                            nearest = distanceSquared;
                    }
                    if (!nearest.HasValue)
                        throw new InvalidOperationException("Cannot match an intersection contour to a degenerate source edge.");
                    if (nearest.Value > maximum) maximum = nearest.Value;
                }
            }
        }

        internal bool AnchorCorner(int cornerId, Rat3Hybrid center, Rat3Hybrid contactA, Rat3Hybrid contactB)
        {
            Vec3D node = cornerId == StartCornerId ? SourceEdge.StartNode.Position : SourceEdge.EndNode.Position;
            bool first = (CenterCurveVec3[0] - node).LengthSquared() <
                         (CenterCurveVec3[^1] - node).LengthSquared();
            int index = CenterCurve.FindIndex(point => point == center);
            bool onVertex = index >= 0;
            if (!onVertex)
            {
                index = -1;
                for (int i = 0; i + 1 < CenterCurve.Count; i++)
                    if (PointOnBoundarySegment(CenterCurve[i], CenterCurve[i + 1], center))
                    {
                        index = i;
                        break;
                    }
            }
            if (index < 0)
                throw new InvalidOperationException(
                    $"Fillet edge '{SourceEdge.Name}' does not pass through its exact corner support intersection.");

            int before = first ? (onVertex ? index : index + 1) : 0;
            int after = first ? CenterCurve.Count : index + 1;
            var centers = CenterCurve.GetRange(before, after - before);
            var railA = ProjectedBoundaryCurveA.GetRange(before, after - before);
            var railB = ProjectedBoundaryCurveB.GetRange(before, after - before);
            if (first)
            {
                if (onVertex) { centers[0] = center; railA[0] = contactA; railB[0] = contactB; }
                else { centers.Insert(0, center); railA.Insert(0, contactA); railB.Insert(0, contactB); }
            }
            else
            {
                if (onVertex) { centers[^1] = center; railA[^1] = contactA; railB[^1] = contactB; }
                else { centers.Add(center); railA.Add(contactA); railB.Add(contactB); }
            }
            if (centers.Count < 2)
                throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' collapses at its exact corner junction.");
            CenterCurve = centers;
            ProjectedBoundaryCurveA = railA;
            ProjectedBoundaryCurveB = railB;
            CenterCurveVec3 = cc.Convert(centers);
            ProjectedBoundaryCurveAVec3D = cc.Convert(railA);
            ProjectedBoundaryCurveBVec3D = cc.Convert(railB);
            return first;
        }

        internal bool TryProjectCornerCenter(Rat3Hybrid reference, double maxDeviation,
            out Rat3Hybrid center, out double deviation)
        {
            center = default;
            deviation = double.PositiveInfinity;
            for (int i = 0; i + 1 < CenterCurve.Count; i++)
            {
                var start = CenterCurve[i];
                var direction = CenterCurve[i + 1] - start;
                var lengthSquared = Rat3Hybrid.Dot(direction, direction);
                if (lengthSquared.Sign() == 0) continue;
                var parameter = Rat3Hybrid.Dot(reference - start, direction) / lengthSquared;
                if (parameter < BigRationalHybrid.Zero) parameter = BigRationalHybrid.Zero;
                else if (parameter > BigRationalHybrid.One) parameter = BigRationalHybrid.One;
                var candidate = start + parameter * direction;
                double distance = (cc.Convert(candidate) - cc.Convert(reference)).Length();
                if (distance >= deviation) continue;
                center = candidate;
                deviation = distance;
            }
            return deviation <= maxDeviation;
        }

        internal void RegisterCornerArc(bool first)
        {
            int rowLength = RawSurface.PointsPrecise.Count / CenterCurve.Count;
            int offset = first ? 0 : RawSurface.PointsPrecise.Count - rowLength;
            TrimArcs.Add(RawSurface.PointsPrecise.GetRange(offset, rowLength));
        }

        /// <summary>
        /// Build a fillet strip surface from precomputed spine and boundary curves.
        /// </summary>
        public bool BuildFilletStripSurface(CoordinateConverter cc, double maxDiscretizationDeviation)
        {
            this.cc = cc;

            double maxArcAngle = 0;
            var arcAngles = new double[CenterCurve.Count];
            for (int i = 0; i < CenterCurve.Count; ++i)
            {
                Vec3D dirStart = RailDirection(ProjectedBoundaryCurveA[i], CenterCurve[i]).Normalized();
                Vec3D dirEnd = RailDirection(ProjectedBoundaryCurveB[i], CenterCurve[i]).Normalized();
                // atan2 retains shallow arcs that acos(dot) rounds to zero.
                double angle = Math.Atan2(Vec3DOps.Cross(dirStart, dirEnd).Length(), Vec3DOps.Dot(dirStart, dirEnd));
                if (!(angle > 0 && angle < Math.PI))
                    throw new InvalidOperationException("Fillet contact radii must define a nondegenerate arc shorter than a semicircle.");
                arcAngles[i] = angle;
                maxArcAngle = Math.Max(angle, maxArcAngle);
            }
            // Calculate number of arc points based on deviation tolerance
            int numArcPoints = Math.Max(4, CalculateArcSegments(maxArcAngle, BlendRadius, maxDiscretizationDeviation));


            List<Rat3Hybrid> concatenatedArcPointsPrecise = new List<Rat3Hybrid>();
            List<Vec3D> concatenatedArcPoints = new List<Vec3D>();
            List<Vec3D> normals = new List<Vec3D>();
            List<Vec2D> uv = new List<Vec2D>();
            var distances = new double[CenterCurve.Count];
            for (int i = 1; i < distances.Length; ++i)
                distances[i] = distances[i - 1] + RailDirection(CenterCurve[i], CenterCurve[i - 1]).Length();
            double totalLength = distances[^1];
            if (!(totalLength > 0) || !double.IsFinite(totalLength))
                throw new InvalidOperationException("Fillet spine must have a finite positive length.");
            for (int i = 0; i < CenterCurve.Count; ++i)
            {
                var start = ProjectedBoundaryCurveA[i];
                var end = ProjectedBoundaryCurveB[i];

                double arcAngle = arcAngles[i];
                
                // Use the same binary direction precision as construction frames.
                // Interpolating shared exact boundaries avoids independently snapping
                // nearby strip rows into folded slivers in world lattice coordinates.
                const long coefficientScale = 1L << 40;
                BigRationalHybrid Coefficient(double value) =>
                    new BigRationalHybrid(checked((long)Math.Round(value * coefficientScale)), coefficientScale);
                var arcPrecise = new List<Rat3Hybrid>(numArcPoints);
                for (int j = 0; j < numArcPoints; ++j)
                {
                    double t = j / (double)(numArcPoints - 1);
                    double weightA = Math.Sin((1-t)*arcAngle) / Math.Sin(arcAngle);
                    double weightB = Math.Sin(t*arcAngle) / Math.Sin(arcAngle);
                    var exact = CenterCurve[i] + (start-CenterCurve[i])*Coefficient(weightA) +
                                                 (end-CenterCurve[i])*Coefficient(weightB);
                    exact.Simplify();
                    arcPrecise.Add(exact);
                }

                arcPrecise[0] = start;
                arcPrecise[arcPrecise.Count - 1] = end;

                // Positions and shading must describe the same authoritative
                // rational strip. Subtract before conversion, as for chamfers.
                for (int j = 0; j < numArcPoints; ++j)
                {
                    var normal = RailDirection(arcPrecise[j], CenterCurve[i]).Normalized();
                    normals.Add(BlendType == EdgeBlendType.Concave ? -normal : normal);
                    uv.Add(new Vec2D(distances[i] / totalLength, j / (double)(numArcPoints - 1)));
                }
                concatenatedArcPoints.AddRange(cc.Convert(arcPrecise));
                concatenatedArcPointsPrecise.AddRange(arcPrecise);
            }


            List<Tri> triangles = BuildRectangularTriangleBuffer(CenterCurve.Count, numArcPoints, out _, out _);

            if(!MeshAnalysis.AreTrianglesConsistentlyOriented(triangles))
                throw new Exception();

            EnsureExactStripWinding(triangles, concatenatedArcPointsPrecise, CenterCurve, numArcPoints, BlendType);

            RawSurface = new UVSurface(concatenatedArcPoints, normals, uv, triangles, concatenatedArcPointsPrecise);
            return true;
        }

        /// <summary>
        /// Build a chamfer strip surface (ruled linear cross-sections) from precomputed spine and boundaries.
        /// </summary>
        public bool BuildChamferStripSurface(CoordinateConverter cc, double maxDiscretizationDeviation)
        {
            this.cc = cc;

            const int numPointsPerStrip = 2;
            int numStrips = CenterCurve.Count;

            List<Vec3D> points = new List<Vec3D>(numStrips * numPointsPerStrip);
            List<Rat3Hybrid> precise = new List<Rat3Hybrid>(numStrips * numPointsPerStrip);
            List<Vec3D> normals = new List<Vec3D>(numStrips * numPointsPerStrip);
            List<Vec2D> uv = new List<Vec2D>(numStrips * numPointsPerStrip);
            LineStrip3D centerCurveStrip = new LineStrip3D(CenterCurveVec3);
            double uMax = centerCurveStrip.TotalLength;

            for (int i = 0; i < numStrips; i++)
            {
                var startD = ProjectedBoundaryCurveAVec3D[i];
                var endD = ProjectedBoundaryCurveBVec3D[i];
                double uCoord = centerCurveStrip.GetDistanceFromBuffer(i);
                double u = uMax > 0 ? uCoord / uMax : 0;

                Vec3D across = RailDirection(ProjectedBoundaryCurveB[i], ProjectedBoundaryCurveA[i]);
                // A ruled strip can have different longitudinal tangents at its
                // two boundaries. A forward spine chord is neither derivative.
                bool closed = ProjectedBoundaryCurveA[0] == ProjectedBoundaryCurveA[^1]
                    && ProjectedBoundaryCurveB[0] == ProjectedBoundaryCurveB[^1];
                Vec3D normalA = Vec3DOps.Cross(across, RailTangent(ProjectedBoundaryCurveA, i, closed)).Normalized();
                Vec3D normalB = Vec3DOps.Cross(across, RailTangent(ProjectedBoundaryCurveB, i, closed)).Normalized();
                if (BlendType == EdgeBlendType.Concave)
                {
                    normalA = -normalA;
                    normalB = -normalB;
                }

                points.Add(startD);
                points.Add(endD);
                normals.Add(normalA);
                normals.Add(normalB);
                uv.Add(new Vec2D(u, 0));
                uv.Add(new Vec2D(u, 1));
                precise.Add(ProjectedBoundaryCurveA[i]);
                precise.Add(ProjectedBoundaryCurveB[i]);
            }

            List<Tri> triangles = BuildRectangularTriangleBuffer(numStrips, numPointsPerStrip, out _, out _);
            if (!MeshAnalysis.AreTrianglesConsistentlyOriented(triangles))
                throw new Exception();

            if (!TryEnsureExactStripWinding(triangles, precise, CenterCurve, numPointsPerStrip, BlendType))
                EnsureChamferStripWinding(triangles, points, precise, normals);
            RawSurface = new UVSurface(points, normals, uv, triangles, precise);
            return true;
        }

        private static Vec3D RailDirection(Rat3Hybrid end, Rat3Hybrid start)
        {
            // Subtract before converting: distinct exact vertices can have the
            // same rounded world position, but still define a valid direction.
            var difference = end - start;
            return new Vec3D(difference.X.ToDouble(), difference.Y.ToDouble(), difference.Z.ToDouble());
        }

        // Differentiate the local quadratic in chord-length parameter. This
        // avoids segment-wise shading and accounts for nonuniform rail samples.
        private static Vec3D RailTangent(IReadOnlyList<Rat3Hybrid> rail, int index, bool closed)
        {
            int last = rail.Count - 1;
            if (rail.Count == 2) return RailDirection(rail[1], rail[0]);
            if (closed && index == last) index = 0;
            int previous = index == 0 ? (closed ? last - 1 : 0) : index - 1;
            int next = index == last ? last : index + 1;
            if (!closed && (index == 0 || index == last))
            {
                int step = index == 0 ? 1 : -1;
                Vec3D first = RailDirection(rail[index + step], rail[index]);
                Vec3D second = RailDirection(rail[index + 2 * step], rail[index + step]);
                double h0 = first.Length(), h1 = second.Length();
                if (h0 == 0 || h1 == 0) throw new InvalidOperationException("Chamfer rail contains repeated adjacent samples.");
                return step * ((first / h0) * (2 * h0 + h1) - (second / h1) * h0) / (h0 + h1);
            }
            Vec3D before = RailDirection(rail[index], rail[previous]);
            Vec3D after = RailDirection(rail[next], rail[index]);
            double left = before.Length(), right = after.Length();
            if (left == 0 || right == 0) throw new InvalidOperationException("Chamfer rail contains repeated adjacent samples.");
            return ((before / left) * right + (after / right) * left) / (left + right);
        }

        /// <summary>
        /// Legacy entry: compute spine/boundaries and build a fillet strip.
        /// </summary>
        public bool ComputeBlendGeometry(
            UVSurface originalSurfaceA,
            UVSurface originalSurfaceB,
            UVSurface enlargedSurfaceA,
            UVSurface enlargedSurfaceB,
            UVSurface enlargedAndOffsetSurfaceA,
            UVSurface enlargedAndOffsetSurfaceB,
            CoordinateConverter cc, double maxDiscretizationDeviation)
        {
            if (!ComputeSpineAndBoundaries(
                    originalSurfaceA, originalSurfaceB,
                    enlargedSurfaceA, enlargedSurfaceB,
                    enlargedAndOffsetSurfaceA, enlargedAndOffsetSurfaceB, cc))
                return false;

            return BuildFilletStripSurface(cc, maxDiscretizationDeviation);
        }

        private static void RemoveCollinear3(List<Rat3Hybrid> lineStripA,
            List<Rat3Hybrid> lineStripB, List<Rat3Hybrid> lineStripC)
        {
            if (lineStripA == null || lineStripB == null || lineStripC == null)
                return;
            
            if (lineStripA.Count != lineStripB.Count || lineStripA.Count != lineStripC.Count)
                throw new ArgumentException("All line strips must have the same number of points");
            
            if (lineStripA.Count <= 2)
                return;
            
            // Cache which indices should be removed
            // A point is only removed if it's collinear in ALL three strips
            List<int> indicesToRemove = new List<int>();
            
            for (int i = 1; i < lineStripA.Count - 1; i++)
            {
                bool collinearInA = IsCollinearAt(lineStripA, i);
                bool collinearInB = IsCollinearAt(lineStripB, i);
                bool collinearInC = IsCollinearAt(lineStripC, i);
                collinearInC = true;
                
                // Only remove if collinear in all three strips
                if (collinearInA && collinearInB && collinearInC)
                {
                    indicesToRemove.Add(i);
                }
            }
            
            // Remove in reverse order to avoid index shifting issues
            for (int i = indicesToRemove.Count - 1; i >= 0; i--)
            {
                int indexToRemove = indicesToRemove[i];
                lineStripA.RemoveAt(indexToRemove);
                lineStripB.RemoveAt(indexToRemove);
                lineStripC.RemoveAt(indexToRemove);
            }
        }
        
        private static bool IsCollinearAt(List<Rat3Hybrid> lineStrip, int index)
        {
            // Get vectors for segments before and after this point
            Rat3Hybrid backward = lineStrip[index] - lineStrip[index - 1];
            Rat3Hybrid forward = lineStrip[index + 1] - lineStrip[index];
            
            // Check if cross product is zero (collinear)
            Rat3Hybrid cross = Rat3Hybrid.Cross(backward, forward);
            
            return cross.IsZero();
        }

        //private static List<Rat3Hybrid> RemoveCollinear(List<Rat3Hybrid> lineStrip)
        //{
        //    if (lineStrip == null || lineStrip.Count <= 2)
        //        return lineStrip;
            
        //    List<Rat3Hybrid> result = new List<Rat3Hybrid>();
        //    result.Add(lineStrip[0]); // Always keep first point
            
        //    for (int i = 1; i < lineStrip.Count - 1; i++)
        //    {
        //        // Get vectors for segments before and after this point
        //        Rat3Hybrid backward = lineStrip[i] - lineStrip[i - 1];
        //        Rat3Hybrid forward = lineStrip[i + 1] - lineStrip[i];
                
        //        // Check if cross product is zero (collinear)
        //        Rat3Hybrid cross = Rat3Hybrid.Cross(backward, forward);
                
        //        // If cross product is not zero, the point is not collinear - keep it
        //        if (!cross.IsZero())
        //        {
        //            result.Add(lineStrip[i]);
        //        }
        //    }
            
        //    result.Add(lineStrip[lineStrip.Count - 1]); // Always keep last point
            
        //    return result;
        //}

        public bool TrimByVolume(MeshNormalUV volume, CoordinateConverter cc)
        {
            if (BlendType == EdgeBlendType.Concave)
            {
                BlendSurface = RawSurface; //TODO: This is a hack
                return true;
            }

            // Convert raw surface to mesh
            MeshNormalUV rawSurfaceMesh = ToMesh(RawSurface, cc, groupId: 0);
            
            // Perform boolean operation
            MeshNormalUV trimmed = MeshNormalUV.BooleanOperation(rawSurfaceMesh, volume, 
                BooleanOp.AAsSurfaceBAsTrimVolumeKeepInside, cc);

            // Rebuild surface with the cluster connected to boundary curves
            BlendSurface = RebuildSurfaceFromTrimmedMesh(trimmed);
            
            return true;
        }

        /// <summary>
        /// Rebuild a surface from a trimmed mesh by selecting the cluster connected to boundary curves.
        /// </summary>
        /// <param name="trimmedMesh">The trimmed mesh to rebuild from</param>
        /// <returns>A new UVSurface containing only the cluster connected to boundary curves</returns>
        private UVSurface RebuildSurfaceFromTrimmedMesh(MeshNormalUV trimmedMesh, Rat3Hybrid? preferredAnchor = null)
        {
            // Find all connected clusters of triangles
            List<List<int>> trianglePatches = FindClusters(trimmedMesh);

            // Keep only the cluster that's connected to the boundary curves
            int keepClusterIndex = SelectClusterConnectedToBoundary(trianglePatches, trimmedMesh, preferredAnchor);
            
            // Extract the triangles from the selected cluster
            List<int> keepTriangles = trianglePatches[keepClusterIndex];
            
            // Create a filtered mesh with only the selected cluster
            MeshNormalUV filteredMesh = new MeshNormalUV();
            filteredMesh.Positions = trimmedMesh.Positions;
            filteredMesh.PrecisionPositions = trimmedMesh.PrecisionPositions;
            filteredMesh.Triangles = new List<Tri>();
            filteredMesh.TrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>();
            
            foreach (int triIndex in keepTriangles)
            {
                filteredMesh.Triangles.Add(trimmedMesh.Triangles[triIndex]);
                filteredMesh.TrianglesEx.Add(trimmedMesh.TrianglesEx[triIndex]);
            }
            
            // Preserve the source mesh's exact coordinates while splitting UV
            // seams; reconstructing them from display positions loses identity.
            filteredMesh.Decompose(out List<Vec3D> positions, out List<Vec3D> normals,
                out List<Vec2D> uvs, out List<Tri> triangles, out List<int> perTriangleGroup,
                out List<Rat3Hybrid> precisePositions);

            return new UVSurface(positions, normals, uvs, triangles, precisePositions);
        }

        /// <summary>
        /// Convert a UVSurface to a MeshNormalUV with the specified group ID.
        /// </summary>
        public static MeshNormalUV ToMesh(UVSurface surface, CoordinateConverter cc, int groupId = 0)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));
            if (surface.PointsPrecise == null)
                throw new ArgumentException("Surface must have PointsPrecise", nameof(surface));

            // Build triangle corner data and per-triangle group list
            List<MeshTriangle<TriangleVertexNormalUV>> triangleCornerData = new List<MeshTriangle<TriangleVertexNormalUV>>(surface.Triangles.Count);
            List<int> perTriangleGroup = new List<int>(surface.Triangles.Count);
            
            for (int i = 0; i < surface.Triangles.Count; i++)
            {
                var tri = surface.Triangles[i];
                
                // Create triangle corner data
                MeshTriangle<TriangleVertexNormalUV> cornerData = new MeshTriangle<TriangleVertexNormalUV>
                {
                    V0 = new TriangleVertexNormalUV 
                    { 
                        Normal = surface.Normals[tri.A], 
                        UV = surface.Uv[tri.A] 
                    },
                    V1 = new TriangleVertexNormalUV 
                    { 
                        Normal = surface.Normals[tri.B], 
                        UV = surface.Uv[tri.B] 
                    },
                    V2 = new TriangleVertexNormalUV 
                    { 
                        Normal = surface.Normals[tri.C], 
                        UV = surface.Uv[tri.C] 
                    },
                    GroupId = groupId
                };
                
                triangleCornerData.Add(cornerData);
                perTriangleGroup.Add(groupId);
            }
            
            // Create and return the mesh using the precise constructor
            return new MeshNormalUV(cc, surface.PointsPrecise, surface.Triangles, triangleCornerData, perTriangleGroup);
        }

        public static MeshNormalUV ToMesh(List<UVSurface> surfaces, CoordinateConverter cc, ref int groupId)
        {
            if (surfaces == null || surfaces.Count == 0)
                throw new ArgumentException("Surface list cannot be null or empty", nameof(surfaces));

            // Accumulate all data from all surfaces
            List<Rat3Hybrid> allPointsPrecise = new List<Rat3Hybrid>();
            List<Tri> allTriangles = new List<Tri>();
            List<MeshTriangle<TriangleVertexNormalUV>> triangleCornerData = new List<MeshTriangle<TriangleVertexNormalUV>>();
            List<int> perTriangleGroup = new List<int>();

            int vertexOffset = 0;

            foreach (var surface in surfaces)
            {
                // Add precise points from this surface
                if (surface.PointsPrecise != null)
                    allPointsPrecise.AddRange(surface.PointsPrecise);
                else
                {
                    throw new Exception();
                }

                // Add triangles with offset vertex indices and create corner data
                for (int i = 0; i < surface.Triangles.Count; i++)
                {
                    var tri = surface.Triangles[i];
                    
                    // Add triangle with offset indices
                    allTriangles.Add(new Tri(
                        tri.A + vertexOffset,
                        tri.B + vertexOffset,
                        tri.C + vertexOffset));
                    
                    // Create triangle corner data
                    MeshTriangle<TriangleVertexNormalUV> cornerData = new MeshTriangle<TriangleVertexNormalUV>
                    {
                        V0 = new TriangleVertexNormalUV 
                        { 
                            Normal = surface.Normals[tri.A], 
                            UV = surface.Uv[tri.A] 
                        },
                        V1 = new TriangleVertexNormalUV 
                        { 
                            Normal = surface.Normals[tri.B], 
                            UV = surface.Uv[tri.B] 
                        },
                        V2 = new TriangleVertexNormalUV 
                        { 
                            Normal = surface.Normals[tri.C], 
                            UV = surface.Uv[tri.C] 
                        },
                        GroupId = groupId
                    };
                    
                    triangleCornerData.Add(cornerData);
                    perTriangleGroup.Add(groupId);
                }

                // Update offset for next surface
                vertexOffset += surface.Points.Count;
                
                // Increment group ID for next surface
                groupId++;
            }

            // Create and return the combined mesh using the precise constructor
            return new MeshNormalUV(cc, allPointsPrecise, allTriangles, triangleCornerData, perTriangleGroup);
        }

        internal UVSurface PerpendicularCornerTrimSurface(UVSurface trimSurface)
        {
            if (!HasPlanarSourceFaces || !trimSurface.IsSurfacePlanar())
                return trimSurface;
            var direction = CenterCurve[^1] - CenterCurve[0];
            var lengthSquared = Rat3Hybrid.Dot(direction, direction);
            if (lengthSquared.Sign() == 0)
                return trimSurface;
            foreach (var triangle in trimSurface.Triangles)
            {
                var anchor = trimSurface.PointsPrecise[triangle.A];
                var normal = Rat3Hybrid.Cross(trimSurface.PointsPrecise[triangle.B] - anchor,
                    trimSurface.PointsPrecise[triangle.C] - anchor);
                var denominator = Rat3Hybrid.Dot(normal, direction);
                if (denominator.Sign() == 0)
                    continue;
                var parameter = Rat3Hybrid.Dot(normal, anchor - CenterCurve[0]) / denominator;
                // This support only defines an endpoint cut if its plane meets
                // the finite blend spine. Extending its perpendicular plane
                // from an intersection far outside the spine creates an
                // unrelated, potentially unbounded cutter.
                if (parameter < BigRationalHybrid.Zero || parameter > BigRationalHybrid.One)
                    return trimSurface;
                var center = CenterCurve[0] + direction * parameter;
                // A rolling ball ends a cylindrical strip at the common offset
                // support intersection, in a plane normal to the straight spine.
                int dropped = Enumerable.Range(0, 3).MaxBy(axis => Math.Abs(direction[axis].ToDouble()));
                int x = (dropped + 1) % 3, y = (dropped + 2) % 3;
                // The support is a plane, not the projected finite third face.
                // Its rectangle encloses every possible strip intersection:
                // triangle-plane intersections lie within the triangle bounds.
                // One exact coordinate unit makes these computational bounds strict.
                var minX = RawSurface.PointsPrecise.Select(point => point[x]).Aggregate((a, b) => a < b ? a : b) - new BigRationalHybrid(1);
                var maxX = RawSurface.PointsPrecise.Select(point => point[x]).Aggregate((a, b) => a > b ? a : b) + new BigRationalHybrid(1);
                var minY = RawSurface.PointsPrecise.Select(point => point[y]).Aggregate((a, b) => a < b ? a : b) - new BigRationalHybrid(1);
                var maxY = RawSurface.PointsPrecise.Select(point => point[y]).Aggregate((a, b) => a > b ? a : b) + new BigRationalHybrid(1);
                var planeD = Rat3Hybrid.Dot(direction, center);
                Rat3Hybrid Point(BigRationalHybrid a, BigRationalHybrid b)
                {
                    var values = new BigRationalHybrid[3];
                    values[x] = a;
                    values[y] = b;
                    values[dropped] = (planeD - direction[x] * a - direction[y] * b) / direction[dropped];
                    var point = new Rat3Hybrid(values[0], values[1], values[2]);
                    point.Simplify();
                    return point;
                }
                var points = new List<Rat3Hybrid> { Point(minX, minY), Point(maxX, minY),
                    Point(maxX, maxY), Point(minX, maxY) };
                var triangles = new List<Tri> { new(0, 1, 2), new(0, 2, 3) };
                if (direction[dropped].Sign() * denominator.Sign() < 0)
                    triangles = triangles.Select(t => new Tri(t.A, t.C, t.B)).ToList();
                var worldNormal = (CenterCurveVec3[^1] - CenterCurveVec3[0]).Normalized() * denominator.Sign();
                return new UVSurface(cc.Convert(points), Enumerable.Repeat(worldNormal, points.Count).ToList(),
                    new List<Vec2D> { new(0, 0), new(1, 0), new(1, 1), new(0, 1) }, triangles, points);
            }
            return trimSurface;
        }

        public bool TrimByOffsetSurface(UVSurface trimSurface)
        {
            var BlendSurface = this.RawSurface;

            // Convert blend surface to mesh
            MeshNormalUV blendSurfaceMesh = ToMesh(BlendSurface, cc, groupId: 0);

            MeshNormalUV trimMesh = ToMesh(trimSurface, cc, 0);

            
            List<List<IntersectionSegmentEx>> intersectionStrips = new List<List<IntersectionSegmentEx>>();
            // Convex corners retain the solid side of the offset support;
            // concave corners retain its normal side, inside the cavity.
            MeshNormalUV trimmed = MeshNormalUV.BooleanOperation(blendSurfaceMesh, trimMesh,
                BlendType == EdgeBlendType.Concave
                    ? BooleanOp.AAsSurfaceBAsTrimSurfaceKeepInTriNormalDirection
                    : BooleanOp.AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection, cc, intersectionStrips);

            // Rebuild surface with the cluster connected to boundary curves
            BlendSurface = RebuildSurfaceFromTrimmedMesh(trimmed);

            if (intersectionStrips.Count > 1)
                throw new Exception();

            if (intersectionStrips.Count == 1)
            {
                List<Rat3Hybrid> arc = ExtractCurveFromSegments(intersectionStrips[0]);
                TrimArcs.Add(arc);

#if DEBUG
                VerifyArc(arc);
#endif
            }

            this.RawSurface = BlendSurface;

            //FinalBoundaryCurveA = Intersector.TrimLineStrip(FinalBoundaryCurveA, trimSurface);
            //FinalBoundaryCurveB = Intersector.TrimLineStrip(FinalBoundaryCurveB, trimSurface);

            return true;
        }

        /// <summary>
        /// Trim the blend surface by trim surfaces at open ends.
        /// The part against the normal direction of the trim surface is kept.
        /// Trimming is only applied on open ends (corners that have trim surfaces).
        /// </summary>
        /// <param name="openCornerTrimSurfaces">Dictionary mapping corner IDs to their trim surfaces</param>
        /// <returns>True if trimming was successful</returns>
        public bool TrimBySurface(Dictionary<int, UVSurface> openCornerTrimSurfaces, Dictionary<int, UVSurface> openCornerTrimSurfacesExtensionOnly)
        {
            if (openCornerTrimSurfaces == null || openCornerTrimSurfaces.Count == 0)
            {
                // No trim surfaces provided - nothing to trim
                BlendSurface = RawSurface;
                return true;
            }

            var currentSurface = this.RawSurface;

            bool hasStart = openCornerTrimSurfaces.TryGetValue(StartCornerId, out UVSurface startTrimSurface);
            bool hasEnd = openCornerTrimSurfaces.TryGetValue(EndCornerId, out UVSurface endTrimSurface);
            // An open edge can enter and leave the same end face. Its two cuts
            // must split that face together; a second trim would lose the closures.
            bool sharedEndFace = hasStart && hasEnd && SamePreciseSurface(startTrimSurface, endTrimSurface);
            if (hasStart)
            {
                currentSurface = TrimSurfaceBySurface(currentSurface, startTrimSurface,
                    openCornerTrimSurfacesExtensionOnly[StartCornerId], out var closures);
                int expectedMaximum = sharedEndFace ? 2 : 1;
                if (closures.Count == 0 || closures.Count > expectedMaximum)
                    throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' has {closures.Count} endpoint closures; expected at most {expectedMaximum}.");
                EdgeStartGapFillSurface = closures[0];
                if (closures.Count == 2)
                    EdgeEndGapFillSurface = closures[1];
            }
            if (hasEnd && !sharedEndFace)
            {
                currentSurface = TrimSurfaceBySurface(currentSurface, endTrimSurface,
                    openCornerTrimSurfacesExtensionOnly[EndCornerId], out var closures);
                if (closures.Count != 1)
                    throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' has {closures.Count} closures at its end; expected one.");
                EdgeEndGapFillSurface = closures[0];
            }

            this.RawSurface = currentSurface;
            this.BlendSurface = currentSurface;
            
            return true;
        }

        internal bool TrimBySupportingCap(UVSurface cap, UVSurface extension, out List<UVSurface> closures)
        {
            closures = new List<UVSurface>();
            var triangle = cap.Triangles.First(t => Rat3Hybrid.Cross(
                cap.PointsPrecise[t.B] - cap.PointsPrecise[t.A],
                cap.PointsPrecise[t.C] - cap.PointsPrecise[t.A]) != new Rat3Hybrid(0, 0, 0));
            var origin = cap.PointsPrecise[triangle.A];
            var normal = Rat3Hybrid.Cross(cap.PointsPrecise[triangle.B] - origin, cap.PointsPrecise[triangle.C] - origin);
            var used  =  RawSurface.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct();
            if (!used.Any(index => Rat3Hybrid.Dot(RawSurface.PointsPrecise[index] - origin, normal).Sign() > 0))
                return false;
            RawSurface = TrimSurfaceBySurface(RawSurface, cap, extension, out closures);
            BlendSurface = RawSurface;
            return true;
        }

        private static bool SamePreciseSurface(UVSurface a, UVSurface b)
        {
            if (a.PointsPrecise.Count != b.PointsPrecise.Count || a.Triangles.Count != b.Triangles.Count)
                return false;
            for (int i = 0; i < a.PointsPrecise.Count; ++i)
                if (a.PointsPrecise[i] != b.PointsPrecise[i])
                    return false;
            for (int i = 0; i < a.Triangles.Count; ++i)
                if (a.Triangles[i].A != b.Triangles[i].A || a.Triangles[i].B != b.Triangles[i].B || a.Triangles[i].C != b.Triangles[i].C)
                    return false;
            return true;
        }

        /// <summary>
        /// Trim a surface by another surface, keeping the part against the normal direction of the trim surface.
        /// </summary>
        /// <param name="surfaceToTrim">The surface to be trimmed</param>
        /// <param name="trimSurface">The surface to trim with</param>
        /// <returns>The trimmed surface</returns>
        private UVSurface TrimSurfaceBySurface(UVSurface surfaceToTrim, UVSurface trimSurface, 
            UVSurface trimSurfaceExtensionOnly,
            out List<UVSurface> finalGapFillSurfaces)
        {
            // Convert surfaces to meshes
            MeshNormalUV surfaceToTrimMesh = ToMesh(surfaceToTrim, cc, groupId: 0);
            // Cap orientation follows the source edge's side of the plane.
            bool keepNormalSide = false;
            if (trimSurface.IsSurfacePlanar())
            {
                var triangle = trimSurface.Triangles.First(t => !Rat3Hybrid.Cross(
                    trimSurface.PointsPrecise[t.B] - trimSurface.PointsPrecise[t.A],
                    trimSurface.PointsPrecise[t.C] - trimSurface.PointsPrecise[t.A]).IsZero());
                var origin = trimSurface.PointsPrecise[triangle.A];
                var normal = Rat3Hybrid.Cross(trimSurface.PointsPrecise[triangle.B] - origin,
                    trimSurface.PointsPrecise[triangle.C] - origin);
                var sides = SourceEdge.LineStripExact.Select(point => Rat3Hybrid.Dot(point - origin, normal).Sign())
                    .Where(sign => sign != 0).Distinct().ToArray();
                keepNormalSide = sides.Length == 1 && sides[0] > 0;
            }

            var intersectionStrips = new List<List<IntersectionSegmentEx>>();
            MeshNormalUV trimmed = null;
            bool useLocalContours = false;
            try
            {
                var trimMesh = ToMesh(trimSurface, cc, groupId: 1);
                trimmed = MeshNormalUV.BooleanOperation(surfaceToTrimMesh, trimMesh,
                    keepNormalSide ? BooleanOp.AAsSurfaceBAsTrimSurfaceKeepInTriNormalDirection :
                        BooleanOp.AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection,
                    cc, intersectionStrips);
            }
            catch (Exception error) when (error.Message.Contains("Partial cut detected"))
            { useLocalContours = true; }
            catch (AggregateException) when (intersectionStrips.Count > 0)
            { useLocalContours = true; }

            int middle = (SourceEdge.LineStripExact.Count - 1) / 2;
            var sourceAnchor = (SourceEdge.LineStripExact[middle] + SourceEdge.LineStripExact[middle + 1]) / new BigRationalHybrid(2);
            var retainAnchor = ClosestPointOnSurface(surfaceToTrim, sourceAnchor);
            Surface retainedRegion = null;
            {
                // Use exact local contours for cap construction even when the
                // CSG trim succeeded. The volume result is authoritative for
                // the retained strip, while these contours identify only the
                // endpoint loops that must be sewn to neighboring patches.
                intersectionStrips.Clear();
                Resolver.Resolve(BooleanOp.NoOpIntersectionContourOnly,
                    surfaceToTrim.PointsPrecise, surfaceToTrim.Triangles,
                    trimSurface.PointsPrecise, trimSurface.Triangles,
                    out _, out _, out _, intersectionStrips: intersectionStrips);
                for (int i = intersectionStrips.Count - 1; i >= 0; --i)
                {
                    var strip = intersectionStrips[i];
                    var first = strip[0].StartPoint;
                    var last = strip[^1].EndPoint;
                    // A cap can cut a lobe of a closed junction, entering and
                    // leaving the same contact rail. Completeness requires two
                    // boundary endpoints, not two different rails.
                    bool connectsRails =
                        (PointOnBoundaryPolyline(ProjectedBoundaryCurveA, first) ||
                         PointOnBoundaryPolyline(ProjectedBoundaryCurveB, first)) &&
                        (PointOnBoundaryPolyline(ProjectedBoundaryCurveA, last) ||
                         PointOnBoundaryPolyline(ProjectedBoundaryCurveB, last));
                    if (!connectsRails) intersectionStrips.RemoveAt(i);
                }
                if (intersectionStrips.Count == 0)
                    throw new InvalidOperationException(
                        $"Fillet edge '{SourceEdge.Name}' cannot terminate on the adjacent face: " +
                        "no intersection contour spans both fillet contact rails.");

                var sourceGroups = new List<int>(surfaceToTrim.Triangles.Count);
                for (int i = 0; i < surfaceToTrim.Triangles.Count; ++i) sourceGroups.Add(i);
                var source = new Surface(surfaceToTrim.Triangles, surfaceToTrim.PointsPrecise, sourceGroups);
                var trimCurves = new List<List<LineSegmentOnTriangle>>(intersectionStrips.Count);
                foreach (var strip in intersectionStrips)
                {
                    var curve = new List<LineSegmentOnTriangle>(strip.Count);
                    foreach (var segment in strip)
                        curve.Add(new LineSegmentOnTriangle(segment.StartPoint, segment.EndPoint, segment.TriIdA));
                    trimCurves.Add(curve);
                }
                var regions = Splitter.SplitUsingLineStrip(source, trimCurves,
                    throwOnInvalidTriangleIndex: true, validateTrimCurves: true);
                foreach (var region in regions)
                {
                    if (!ContainsExactPoint(region, retainAnchor)) continue;
                    if (retainedRegion != null)
                        throw new InvalidOperationException(
                            $"Fillet edge '{SourceEdge.Name}' retained endpoint anchor lies on a trim contour.");
                    retainedRegion = region;
                }
                if (retainedRegion == null)
                    throw new InvalidOperationException(
                        $"Fillet edge '{SourceEdge.Name}' endpoint trim removed its retained source-edge anchor.");

                // Extended supports can cross a pocket on both sides. Only
                // cuts bounding the source-edge component need endpoint caps.
                var boundary = Adjacency.BuildEdgeList(retainedRegion.Triangles);
                for (int i = intersectionStrips.Count - 1; i >= 0; --i)
                {
                    bool boundsRegion = false;
                    foreach (var segment in intersectionStrips[i])
                    {
                        foreach (var edge in boundary)
                            if (edge.IsOnBorder && OverlapsBoundarySegment(segment.StartPoint, segment.EndPoint,
                                retainedRegion.PointsPrecise[edge.Start], retainedRegion.PointsPrecise[edge.End]))
                            { boundsRegion = true; break; }
                        if (boundsRegion) break;
                    }
                    if (!boundsRegion) intersectionStrips.RemoveAt(i);
                }
            }

            int idOffset = trimSurface.Triangles.Count - trimSurfaceExtensionOnly.Triangles.Count;
            var splitCurves = new List<List<LineSegmentOnTriangle>>();
            var curveNormals = new List<List<Rat3Hybrid>>();
            var curveAnchors = new List<List<Rat3Hybrid>>();
            foreach (var sourceCurve in intersectionStrips)
            {
                var splitCurve = new List<LineSegmentOnTriangle>(sourceCurve.Count);
                var normals = new List<Rat3Hybrid>(sourceCurve.Count);
                var anchors = new List<Rat3Hybrid>(sourceCurve.Count);
                foreach (var segment in sourceCurve)
                {
                    int correctedId = segment.TriIdB - idOffset;
                    if (correctedId < 0)
                        throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' endpoint contour crosses the original trim face instead of its extension.");
                    splitCurve.Add(new LineSegmentOnTriangle(segment.StartPoint, segment.EndPoint, correctedId));
                    anchors.Add(segment.StartPoint);
                    var triangle = surfaceToTrimMesh.Triangles[segment.TriIdA];
                    var points = surfaceToTrimMesh.PrecisionPositions;
                    var closureNormal = Rat3Hybrid.Cross(points[triangle.B]-points[triangle.A], points[triangle.C]-points[triangle.A]);
                    // Added material and removed material use opposite sides
                    // of the same exact endpoint intersection contour.
                    normals.Add(BlendType == EdgeBlendType.Convex ? -closureNormal : closureNormal);
                }
                splitCurves.Add(splitCurve);
                curveNormals.Add(normals);
                curveAnchors.Add(anchors);
            }

            Surface surf = new Surface(trimSurfaceExtensionOnly.Triangles, trimSurfaceExtensionOnly.PointsPrecise,
                Enumerable.Range(0,trimSurfaceExtensionOnly.Triangles.Count).ToList());
            var allCuts = new List<List<LineSegmentOnTriangle>>(splitCurves);
            foreach (var support in new[] { supportA, supportB })
            {
                var (onCap, _) = Intersector.IntersectSurfaces(trimSurfaceExtensionOnly, support, cc);
                if (onCap != null && onCap.Count > 0)
                    allCuts.Add(onCap.Select(segment => new LineSegmentOnTriangle(segment.PointStart, segment.PointEnd, segment.TriangleId)).ToList());
            }
            List<Surface> splitSurfaces = Splitter.SplitUsingLineStrip(surf, allCuts, throwOnInvalidTriangleIndex: true, validateTrimCurves: true);
            var selected = new HashSet<Surface>();
            finalGapFillSurfaces = new List<UVSurface>();
            for (int i = 0; i < splitCurves.Count; ++i)
            {
                Surface closure = SelectSurfaceAgainstNormalDirection(splitSurfaces,
                    splitCurves[i], curveNormals[i], curveAnchors[i]);
                if (selected.Add(closure))
                {
                    var patch = InterpolateClosureAttributes(closure, trimSurfaceExtensionOnly, cc);
                    if ((BlendType == EdgeBlendType.Convex) != keepNormalSide)
                        patch = new UVSurface(patch.Points, patch.Normals.Select(n => -n).ToList(), patch.Uv,
                            patch.Triangles.Select(t => new Tri(t.A,t.C,t.B)).ToList(), patch.PointsPrecise);
                    finalGapFillSurfaces.Add(patch);
                }
                // This open-end boundary is closed by the patch above. Only
                // offset-support cuts contribute arcs to a multi-edge corner.
            }
            UVSurface result = useLocalContours
                ? InterpolateClosureAttributes(retainedRegion, surfaceToTrim, cc)
                : RebuildSurfaceFromTrimmedMesh(trimmed, retainAnchor);
            if (useLocalContours)
            {
                // Support cuts can subdivide the cap's shared contour further.
                // Propagate those exact vertices to both sides before sewing.
                foreach (var closure in finalGapFillSurfaces)
                    result = ConformBoundary(result, closure, cc);
                for (int i = 0; i < finalGapFillSurfaces.Count; ++i)
                    finalGapFillSurfaces[i] = ConformBoundary(finalGapFillSurfaces[i], result, cc);
            }

            return result;
        }

        private static UVSurface ConformBoundary(UVSurface surface, UVSurface neighbour, CoordinateConverter converter)
        {
            var candidates = new HashSet<Rat3Hybrid>();
            foreach (var edge in Adjacency.BuildEdgeList(neighbour.Triangles))
                if (edge.IsOnBorder)
                { candidates.Add(neighbour.PointsPrecise[edge.Start]); candidates.Add(neighbour.PointsPrecise[edge.End]); }
            var cuts = new List<List<LineSegmentOnTriangle>>();
            foreach (var edge in Adjacency.BuildEdgeList(surface.Triangles))
            {
                if (!edge.IsOnBorder) continue;
                var a = surface.PointsPrecise[edge.Start];
                var b = surface.PointsPrecise[edge.End];
                var points = new List<Rat3Hybrid> { a, b };
                foreach (var point in candidates)
                    if (point != a && point != b && PointOnBoundarySegment(a, b, point)) points.Add(point);
                if (points.Count == 2) continue;
                var direction = b - a;
                points.Sort((x, y) => Rat3Hybrid.Dot(x - a, direction).CompareTo(Rat3Hybrid.Dot(y - a, direction)));
                int triangle = edge.NeighbourIndex1 >= 0 ? edge.NeighbourIndex1 : edge.NeighbourIndex2;
                var cut = new List<LineSegmentOnTriangle>();
                for (int i = 1; i < points.Count; ++i)
                    cut.Add(new LineSegmentOnTriangle(points[i - 1], points[i], triangle));
                cuts.Add(cut);
            }
            if (cuts.Count == 0) return surface;
            var groups = new List<int>(surface.Triangles.Count);
            for (int i = 0; i < surface.Triangles.Count; ++i) groups.Add(i);
            var refined = Splitter.SplitUsingLineStrip(new Surface(surface.Triangles, surface.PointsPrecise, groups),
                cuts, throwOnInvalidTriangleIndex: true, validateTrimCurves: true);
            if (refined.Count != 1) throw new InvalidOperationException("Shared endpoint boundary refinement disconnected a patch.");
            return InterpolateClosureAttributes(refined[0], surface, converter);
        }

        private static bool ContainsExactPoint(Surface surface, Rat3Hybrid point)
        {
            foreach (var triangle in surface.Triangles)
            {
                var a = surface.PointsPrecise[triangle.A];
                var b = surface.PointsPrecise[triangle.B];
                var c = surface.PointsPrecise[triangle.C];
                var normal = Rat3Hybrid.Cross(b - a, c - a);
                if (!normal.IsZero() && Rat3Hybrid.Dot(normal, point - a).Sign() == 0 &&
                    new PlaneConvexPolygon(a, b, c).PointIsInsideOrOnBoundary(point, out _)) return true;
            }
            return false;
        }

        private static Rat3Hybrid ClosestPointOnSurface(UVSurface surface, Rat3Hybrid point)
        {
            Rat3Hybrid nearest = default;
            BigRationalHybrid? bestDistance = null;
            void Consider(Rat3Hybrid candidate)
            {
                var delta = candidate - point;
                var distance = Rat3Hybrid.Dot(delta, delta);
                if (!bestDistance.HasValue || distance < bestDistance.Value)
                { bestDistance = distance; nearest = candidate; }
            }
            foreach (var triangle in surface.Triangles)
            {
                var a = surface.PointsPrecise[triangle.A];
                var b = surface.PointsPrecise[triangle.B];
                var c = surface.PointsPrecise[triangle.C];
                var normal = Rat3Hybrid.Cross(b - a, c - a);
                if (normal.IsZero()) continue;
                var projected = point - normal * (Rat3Hybrid.Dot(point - a, normal) / Rat3Hybrid.Dot(normal, normal));
                if (new PlaneConvexPolygon(a, b, c).PointIsInsideOrOnBoundary(projected, out _)) Consider(projected);
                foreach (var edge in new[] { (a, b), (b, c), (c, a) })
                {
                    var direction = edge.Item2 - edge.Item1;
                    var t = Rat3Hybrid.Dot(point - edge.Item1, direction) / Rat3Hybrid.Dot(direction, direction);
                    t = BigRationalHybrid.Max(BigRationalHybrid.Zero, BigRationalHybrid.Min(BigRationalHybrid.One, t));
                    Consider(edge.Item1 + direction * t);
                }
            }
            if (!bestDistance.HasValue) throw new InvalidOperationException("Endpoint strip has no nondegenerate triangles.");
            return nearest;
        }

        private void VerifyArc(List<Rat3Hybrid> arc)
        {
            // Both, start and end point must lie exactly on either BoundaryCurveA or BoundaryCurveB
            var start = arc[0];
            var end = arc[arc.Count - 1];

            if (!PointOnBoundaryPolyline(ProjectedBoundaryCurveA, start) && !PointOnBoundaryPolyline(ProjectedBoundaryCurveB, start))
                throw new Exception();
            if (!PointOnBoundaryPolyline(ProjectedBoundaryCurveA, end) && !PointOnBoundaryPolyline(ProjectedBoundaryCurveB, end))
                throw new Exception();
        }

        private List<Rat3Hybrid> ExtractCurveFromSegments(List<IntersectionSegmentEx> segments)
        {
            if (segments.Count == 0)
                return null;

            List<Rat3Hybrid> result = new List<Rat3Hybrid>();
            result.Add(segments[0].StartPoint);
            for (int i = 0; i < segments.Count; ++i)
            {
                var seg = segments[i];
                if (seg.StartPoint != result[result.Count - 1])
                    throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' has a disconnected endpoint contour at segment {i} of {segments.Count}.");
                result.Add(seg.EndPoint);
            }
            return result;
        }

        public static MeshNormalUV ApplyBlendSurfaceToVolume(MeshNormalUV blendSurface, MeshNormalUV volume,
            CoordinateConverter cc, EdgeBlendType blendType, bool chamfer = false)
        {
            // TODO: Need a correct triangle group index
            // Convert blend surface to mesh
            //MeshNormalUV blendSurfaceMesh = ToMesh(blendSurface, cc, groupId: 0);
            
            // For concave edges: remove triangles in normal direction (removes material inside the blend)
            // For convex edges: keep triangles in normal direction (removes material outside the blend)
            BooleanOp operation = blendType == EdgeBlendType.Convex 
                ? BooleanOp.AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection
                : BooleanOp.AAsVolumeBAsTrimSurfaceKeepInTriNormalDirection;
            
            // A smooth transition can cross inside its faceted supporting shell
            // between exact contact vertices. Retain its exterior patch before
            // using it as a volume boundary; interior excursions are not closures.
            if (blendType == EdgeBlendType.Concave)
                blendSurface = MeshNormalUV.BooleanOperation(blendSurface, volume,
                    BooleanOp.AAsSurfaceBAsTrimVolumeKeepOutside, cc);

            // Apply blend surface to volume
            MeshNormalUV Trim(MeshNormalUV trimSurface) => blendType == EdgeBlendType.Concave
                ? MeshNormalUV.BooleanOperationRetainingExteriorTrimCaps(volume, trimSurface, operation, cc)
                : MeshNormalUV.BooleanOperation(volume, trimSurface, operation, cc);

            MeshNormalUV trimmed = Trim(blendSurface);
            double inputOrientation = MeshAnalysis.ComputeSignedMeshVolume(volume.Positions, volume.Triangles);
            double resultOrientation = MeshAnalysis.ComputeSignedMeshVolume(trimmed.Positions, trimmed.Triangles);
            if (chamfer && blendType == EdgeBlendType.Concave &&
                Math.Sign(inputOrientation) != Math.Sign(resultOrientation))
            {
                // CSG cap selection depends on the exposed patch boundary winding.
                // Retry with the opposite orientation only when the first trim
                // reverses the source solid's global orientation.
                var corrected = Trim(ReverseWinding(blendSurface));
                if (Math.Sign(inputOrientation) == Math.Sign(
                    MeshAnalysis.ComputeSignedMeshVolume(corrected.Positions, corrected.Triangles)))
                    trimmed = corrected;
            }
            trimmed.RunSanityChecks();
            return trimmed;
        }

        private static MeshNormalUV ReverseWinding(MeshNormalUV mesh)
        {
            var triangles = new List<Tri>(mesh.Triangles.Count);
            var data = new List<MeshTriangle<TriangleVertexNormalUV>>(mesh.TrianglesEx.Count);
            for (int i = 0; i < mesh.Triangles.Count; i++)
            {
                var triangle = mesh.Triangles[i];
                triangles.Add(new Tri(triangle.A, triangle.C, triangle.B));
                var corners = mesh.TrianglesEx[i];
                (corners.V1, corners.V2) = (corners.V2, corners.V1);
                data.Add(corners);
            }
            return new MeshNormalUV
            {
                Positions = mesh.Positions,
                PrecisionPositions = mesh.PrecisionPositions,
                Triangles = triangles,
                TrianglesEx = data,
            };
        }


        private static bool PointOnBoundaryPolyline(List<Rat3Hybrid> curve, Rat3Hybrid point)
        {
            if (curve == null) return false;
            for (int i = 1; i < curve.Count; i++)
                if (PointOnBoundarySegment(curve[i - 1], curve[i], point)) return true;
            return false;
        }

        private static bool PointOnBoundarySegment(Rat3Hybrid start, Rat3Hybrid end, Rat3Hybrid point)
        {
            static bool OutsideInterval(BigRationalHybrid value, BigRationalHybrid first, BigRationalHybrid second)
            {
                int a = value.CompareTo(first);
                int b = value.CompareTo(second);
                return (a < 0 && b < 0) || (a > 0 && b > 0);
            }
            if (OutsideInterval(point.X, start.X, end.X) ||
                OutsideInterval(point.Y, start.Y, end.Y) ||
                OutsideInterval(point.Z, start.Z, end.Z))
                return false;
            var direction = end - start;
            var offset = point - start;
            if (offset.X.Sign() == 0 && offset.Y.Sign() == 0 && offset.Z.Sign() == 0)
                return true;
            var lengthSquared = Rat3Hybrid.Dot(direction, direction);
            if (lengthSquared.Sign() == 0) return false;
            var cross = Rat3Hybrid.Cross(direction, offset);
            if (cross.X.Sign() != 0 || cross.Y.Sign() != 0 || cross.Z.Sign() != 0)
                return false;
            var along = Rat3Hybrid.Dot(offset, direction);
            return along.Sign() >= 0 && along.CompareTo(lengthSquared) <= 0;
        }

        internal static List<List<Rat3Hybrid>> RetainBoundaryArcs(UVSurface surface, IList<List<Rat3Hybrid>> arcs)
        {
            var boundary = AdjacencyEx.BuildEdgeToTrianglesMap(surface.Triangles)
                .Where(edge => edge.Value.Count == 1)
                .Select(edge => (Start: surface.PointsPrecise[edge.Key.Item1], End: surface.PointsPrecise[edge.Key.Item2]))
                .ToList();
            var boundaryPairs = boundary.Concat(boundary.Select(edge => (Start: edge.End, End: edge.Start))).ToHashSet();
            var result = new List<List<Rat3Hybrid>>();
            foreach (var arc in arcs)
            {
                if (arc.Count > 1 && Enumerable.Range(1, arc.Count - 1)
                    .All(i => boundaryPairs.Contains((arc[i - 1], arc[i]))))
                {
                    result.Add(arc);
                    continue;
                }
                // Later trims can shorten or completely remove an earlier cut.
                // Intersect each original segment with the current boundary; this
                // also handles either polyline being subdivided differently.
                var surviving = new List<(Rat3Hybrid Start, Rat3Hybrid End)>();
                foreach (var edge in boundary)
                    for (int i = 1; i < arc.Count; i++)
                    {
                        var overlap = new List<Rat3Hybrid>(2);
                        foreach (var point in new[] { edge.Start, edge.End, arc[i - 1], arc[i] })
                            if (PointOnBoundarySegment(edge.Start, edge.End, point) &&
                                PointOnBoundarySegment(arc[i - 1], arc[i], point) &&
                                !overlap.Any(existing => existing == point))
                                overlap.Add(point);
                        if (overlap.Count == 2)
                            surviving.Add((overlap[0], overlap[1]));
                    }
                if (surviving.Count > 0)
                    result.AddRange(SegmentConnector.ConnectAndResolve(surviving,
                        edge => edge.Start, edge => edge.End, (first, second) => first == second));
            }
            return result;
        }

        internal void RefreshTrimArcs()
        {
            if (RawSurface != null && TrimArcs.Count > 0)
                TrimArcs = RetainBoundaryArcs(RawSurface, TrimArcs);
        }

        private int SelectClusterConnectedToBoundary(List<List<int>> clusters, MeshNormalUV mesh,
            Rat3Hybrid? preferredAnchor = null)
        {
            // Create a HashSet of boundary curve points for fast lookup
            HashSet<Rat3Hybrid> boundaryPoints = new HashSet<Rat3Hybrid>();
            
            if (ProjectedBoundaryCurveA != null)
            {
                foreach (var point in ProjectedBoundaryCurveA)
                {
                    boundaryPoints.Add(point);
                }
            }
            
            if (ProjectedBoundaryCurveB != null)
            {
                foreach (var point in ProjectedBoundaryCurveB)
                {
                    boundaryPoints.Add(point);
                }
            }
            
            // Count how many boundary points each cluster references
            List<int> clusterBoundaryCounts = new List<int>();
            
            for (int clusterIdx = 0; clusterIdx < clusters.Count; clusterIdx++)
            {
                HashSet<int> clusterVertices = new HashSet<int>();
                
                // Collect all unique vertices referenced by this cluster's triangles
                foreach (int triIndex in clusters[clusterIdx])
                {
                    Tri tri = mesh.Triangles[triIndex];
                    clusterVertices.Add(tri.A);
                    clusterVertices.Add(tri.B);
                    clusterVertices.Add(tri.C);
                }
                
                // Corner trimming creates new vertices inside the original boundary
                // segments. Connectivity is exact segment incidence, not merely
                // equality with the pre-trim sample endpoints.
                int boundaryCount = 0;
                foreach (int vertexIndex in clusterVertices)
                {
                    Rat3Hybrid precisePos = mesh.PrecisionPositions[vertexIndex];
                    if (boundaryPoints.Contains(precisePos) ||
                        PointOnBoundaryPolyline(ProjectedBoundaryCurveA, precisePos) ||
                        PointOnBoundaryPolyline(ProjectedBoundaryCurveB, precisePos))
                    {
                        boundaryCount++;
                    }
                }
                
                clusterBoundaryCounts.Add(boundaryCount);
            }
            
            // Find clusters with non-zero boundary counts
            int clusterWithBoundary = -1;
            int nonZeroCount = 0;
            
            for (int i = 0; i < clusterBoundaryCounts.Count; i++)
            {
                if (clusterBoundaryCounts[i] > 0)
                {
                    clusterWithBoundary = i;
                    nonZeroCount++;
                }
            }
            
            if (nonZeroCount == 0)
            {
                throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' has no trimmed cluster connected to its boundary curves.");
            }
            
            if (nonZeroCount > 1)
            {
                if (preferredAnchor.HasValue)
                {
                    int anchoredCluster = -1;
                    for (int clusterIdx = 0; clusterIdx < clusters.Count; clusterIdx++)
                    {
                        if (clusterBoundaryCounts[clusterIdx] == 0 ||
                            !ClusterContainsPoint(clusters[clusterIdx], mesh, preferredAnchor.Value)) continue;
                        if (anchoredCluster >= 0)
                            throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' retained anchor lies on multiple trimmed clusters.");
                        anchoredCluster = clusterIdx;
                    }
                    if (anchoredCluster >= 0) return anchoredCluster;
                }
                throw new Exception($"Multiple clusters ({nonZeroCount}) are connected to boundary curves - ambiguous result");
            }
            
            return clusterWithBoundary;
        }

        private static bool ClusterContainsPoint(List<int> cluster, MeshNormalUV mesh, Rat3Hybrid point)
        {
            foreach (int index in cluster)
            {
                var triangle = mesh.Triangles[index];
                var a = mesh.PrecisionPositions[triangle.A];
                var b = mesh.PrecisionPositions[triangle.B];
                var c = mesh.PrecisionPositions[triangle.C];
                var normal = Rat3Hybrid.Cross(b - a, c - a);
                if (!normal.IsZero() && Rat3Hybrid.Dot(normal, point - a).Sign() == 0 &&
                    new PlaneConvexPolygon(a, b, c).PointIsInsideOrOnBoundary(point, out _)) return true;
            }
            return false;
        }

        private List<List<int>> FindClusters(MeshNormalUV mesh)
        {
            List<Tri> triangles = mesh.Triangles;
            
            // Build adjacency information for all triangles
            TriangleAdjacency[] adjacency = Adjacency.BuildAdjacencyInformation(triangles);
            
            // Track which triangles have been visited
            bool[] visited = new bool[triangles.Count];
            
            // Result clusters
            List<List<int>> clusters = new List<List<int>>();
            
            // Process each triangle using BFS flood fill
            for (int i = 0; i < triangles.Count; i++)
            {
                if (visited[i])
                    continue;
                    
                // Start a new cluster with BFS flood fill
                List<int> cluster = new List<int>();
                Queue<int> queue = new Queue<int>();
                
                queue.Enqueue(i);
                visited[i] = true;
                
                while (queue.Count > 0)
                {
                    int currentTriIndex = queue.Dequeue();
                    cluster.Add(currentTriIndex);
                    
                    TriangleAdjacency adj = adjacency[currentTriIndex];
                    
                    // Check all three neighbors
                    if (adj.NeighbourAB >= 0 && !visited[adj.NeighbourAB])
                    {
                        visited[adj.NeighbourAB] = true;
                        queue.Enqueue(adj.NeighbourAB);
                    }
                    
                    if (adj.NeighbourBC >= 0 && !visited[adj.NeighbourBC])
                    {
                        visited[adj.NeighbourBC] = true;
                        queue.Enqueue(adj.NeighbourBC);
                    }
                    
                    if (adj.NeighbourCA >= 0 && !visited[adj.NeighbourCA])
                    {
                        visited[adj.NeighbourCA] = true;
                        queue.Enqueue(adj.NeighbourCA);
                    }
                }
                
                clusters.Add(cluster);
            }
            
            return clusters;
        }

        public List<Vec3D> GetCenterCurve()
        {
            return cc.Convert(CenterCurve);
        }
        public List<Vec3D> GetSurfaceACurve()
        {
            return cc.Convert(ProjectedBoundaryCurveA);
        }
        public List<Vec3D> GetSurfaceBCurve()
        {
            return cc.Convert(ProjectedBoundaryCurveB);
        }

        private List<Rat3Hybrid> ExtractCurveFromSegments(List<LineSegmentOnTriangleEx> segments)
        {
            if (segments.Count == 0)
                return null;

            List<Rat3Hybrid> result = new List<Rat3Hybrid>();
            result.Add(segments[0].PointStart);
            for(int i=0;i<segments.Count;++i)
            {
                var seg = segments[i];
                if (seg.PointStart != result[result.Count - 1])
                    throw new InvalidOperationException(
                        $"Fillet edge '{SourceEdge.Name}' has a disconnected intersection contour " +
                        $"at segment {i + 1} of {segments.Count}; the next start does not match the previous end.");
                result.Add(seg.PointEnd);
            }
            return result;
        }

        /// <summary>
        /// Extract the boundary curve on the enlarged surface using barycentric coordinates.
        /// Uses intersection segment barycentric coordinates to interpolate positions on the
        /// enlarged (non-offset) surface.
        /// </summary>
        private List<Rat3Hybrid> BarycentricProjection(
            List<LineSegmentOnTriangleEx> segmentsIn,
            UVSurface enlargedSurface)
        {
            if (segmentsIn == null || segmentsIn.Count == 0)
                return null;

            List<Rat3Hybrid> enlargedPoints = enlargedSurface.PointsPrecise;
            if (enlargedPoints == null || enlargedPoints.Count == 0)
                throw new Exception("Enlarged surface must have PointsPrecise");

            List<Rat3Hybrid> curve = new List<Rat3Hybrid>();
            var triangles = enlargedSurface.Triangles;

            // Offset-surface barycentric weights applied to enlarged vertices (same topology).
            // Build like ExtractCurveFromSegments: first start, then each segment end.
            Rat3Hybrid first = InterpolateBarycentricOnSurface(
                segmentsIn[0].BarycentricCoordsStart,
                segmentsIn[0].TriangleId,
                triangles,
                enlargedPoints);
            first.Simplify();
            curve.Add(first);

            foreach (var seg in segmentsIn)
            {
                Rat3Hybrid endPoint = InterpolateBarycentricOnSurface(
                    seg.BarycentricCoordsEnd,
                    seg.TriangleId,
                    triangles,
                    enlargedPoints);
                endPoint.Simplify();
                if (!Rat3HybridCoincide(curve[curve.Count - 1], endPoint))
                    curve.Add(endPoint);
            }

            return curve;
        }

        private static bool Rat3HybridCoincide(Rat3Hybrid a, Rat3Hybrid b)
        {
            a.Simplify();
            b.Simplify();
            return a == b;
        }

        /// <summary>
        /// Interpolate a point on a surface using barycentric coordinates.
        /// Given barycentric coordinates (u, v, w) and a triangle, compute: u*A + v*B + w*C
        /// </summary>
        private static Rat3Hybrid InterpolateBarycentricOnSurface(
            Rat3Hybrid baryCoords,
            int triangleId,
            List<Tri> triangles,
            List<Rat3Hybrid> points)
        {
            Tri tri = triangles[triangleId];
            Rat3Hybrid pA = points[tri.A];
            Rat3Hybrid pB = points[tri.B];
            Rat3Hybrid pC = points[tri.C];

            // Interpolate: result = u*pA + v*pB + w*pC
            // where (u, v, w) are the barycentric coordinates
            return new Rat3Hybrid(
                baryCoords.X * pA.X + baryCoords.Y * pB.X + baryCoords.Z * pC.X,
                baryCoords.X * pA.Y + baryCoords.Y * pB.Y + baryCoords.Z * pC.Y,
                baryCoords.X * pA.Z + baryCoords.Y * pB.Z + baryCoords.Z * pC.Z
            );
        }


        /// <summary>
        /// Calculate the number of segments needed for a circular arc to stay within a deviation tolerance.
        /// </summary>
        /// <param name="angle">Arc angle in radians</param>
        /// <param name="radius">Arc radius</param>
        /// <param name="maxDeviation">Maximum allowed deviation from the true arc</param>
        /// <returns>Number of arc points (minimum 2)</returns>
        internal static int CalculateArcSegments(double angle, double radius, double maxDeviation)
        {
            if (angle <= 0 || maxDeviation <= 0)
                return 2;
            
            double ratio = maxDeviation / radius;
            if (ratio >= 1.0)
                return 2;
            
            // Same sagitta inversion as acos(1-ratio), without cancellation
            // when the deviation is small compared with the radius.
            double halfAnglePerSegment = 2 * Math.Asin(Math.Sqrt(ratio / 2));
            int numSegments = checked((int)Math.Ceiling(angle / (2.0 * halfAnglePerSegment)));
            return Math.Max(2, checked(numSegments + 1));
        }

        /// <summary>
        /// Build a rectangular triangle grid connecting consecutive arc strips.
        /// </summary>
        /// <param name="numStrips">Number of arc strips (e.g., number of center curve points)</param>
        /// <param name="numPointsPerStrip">Number of points in each arc</param>
        /// <param name="startRowIndices">Output: indices of the first row (boundary)</param>
        /// <param name="endRowIndices">Output: indices of the last row (boundary)</param>
        /// <returns>List of triangles forming a rectangular mesh</returns>
        private static List<Tri> BuildRectangularTriangleBuffer(int numStrips, int numPointsPerStrip, out List<int> startRowIndices, out List<int> endRowIndices)
        {
            List<Tri> triangles = new List<Tri>();
            
            // Build the first row indices (the boundary at the start)
            startRowIndices = new List<int>(numStrips);
            for (int i = 0; i < numStrips; ++i)
            {
                startRowIndices.Add(i * numPointsPerStrip);
            }
            
            // Build the last row indices (the boundary at the end)
            endRowIndices = new List<int>(numStrips);
            for (int i = 0; i < numStrips; ++i)
            {
                endRowIndices.Add(i * numPointsPerStrip + (numPointsPerStrip - 1));
            }
            
            for (int i = 0; i < numStrips - 1; ++i)
            {
                for (int j = 0; j < numPointsPerStrip - 1; ++j)
                {
                    int v0 = i * numPointsPerStrip + j;
                    int v1 = i * numPointsPerStrip + (j + 1);
                    int v2 = (i + 1) * numPointsPerStrip + j;
                    int v3 = (i + 1) * numPointsPerStrip + (j + 1);
                    
                    // Create two triangles for each quad
                    triangles.Add(new Tri(v0, v2, v1));
                    triangles.Add(new Tri(v1, v2, v3));
                }
            }
            
            return triangles;
        }

        // The rational strip vertices and spine give an exact winding sign.
        // Use the largest triangle: offset-support intersections can insert
        // nearly coincident rows whose tiny faces have unstable double normals.
        private static void EnsureExactStripWinding(List<Tri> triangles, List<Rat3Hybrid> points,
            List<Rat3Hybrid> centers, int rowLength, EdgeBlendType blendType)
        {
            if (!TryEnsureExactStripWinding(triangles, points, centers, rowLength, blendType))
                throw new InvalidOperationException("Blend strip has no nondegenerate oriented triangle.");
        }

        private static bool TryEnsureExactStripWinding(List<Tri> triangles, List<Rat3Hybrid> points,
            List<Rat3Hybrid> centers, int rowLength, EdgeBlendType blendType)
        {
            var largestArea = BigRationalHybrid.Zero;
            int orientation = 0;
            foreach (var tri in triangles)
            {
                var cross = Rat3Hybrid.Cross(points[tri.B] - points[tri.A],
                    points[tri.C] - points[tri.A]);
                var area = Rat3Hybrid.Dot(cross, cross);
                if (area <= largestArea) continue;
                var radial = points[tri.A] - centers[tri.A / rowLength] +
                    points[tri.B] - centers[tri.B / rowLength] +
                    points[tri.C] - centers[tri.C / rowLength];
                int sign = Rat3Hybrid.Dot(cross, radial).Sign();
                if (sign == 0) continue;
                largestArea = area;
                orientation = sign;
            }
            if (orientation == 0) return false;
            if ((blendType == EdgeBlendType.Convex ? orientation : -orientation) < 0)
                for (int i = 0; i < triangles.Count; i++)
                {
                    var tri = triangles[i];
                    triangles[i] = new Tri(tri.A, tri.C, tri.B);
                }
            return true;
        }

        // On a planar chamfer, radial winding has no sign: its triangles are
        // perpendicular to the across direction, while points minus spine lie
        // entirely across the strip. Use rail normals only in that degenerate case.
        private static void EnsureChamferStripWinding(List<Tri> triangles,
            IReadOnlyList<Vec3D> points, IReadOnlyList<Rat3Hybrid> precise,
            IReadOnlyList<Vec3D> normals)
        {
            BigRationalHybrid largestArea = BigRationalHybrid.Zero;
            double strongestAlignment = 0;
            foreach (var tri in triangles)
            {
                var exactCross = Rat3Hybrid.Cross(precise[tri.B] - precise[tri.A],
                    precise[tri.C] - precise[tri.A]);
                var area = Rat3Hybrid.Dot(exactCross, exactCross);
                if (area <= largestArea) continue;

                var geometricNormal = new Vec3D(exactCross.X.ToDouble(), exactCross.Y.ToDouble(), exactCross.Z.ToDouble());
                double alignment = Vec3DOps.Dot(geometricNormal,
                    normals[tri.A] + normals[tri.B] + normals[tri.C]);
                if (alignment == 0 || !double.IsFinite(alignment)) continue;
                largestArea = area;
                strongestAlignment = alignment;
            }

            if (largestArea == BigRationalHybrid.Zero || strongestAlignment == 0)
                throw new InvalidOperationException("Chamfer strip has no nondegenerate oriented triangle.");
            if (strongestAlignment < 0)
                for (int i = 0; i < triangles.Count; i++)
                {
                    var tri = triangles[i];
                    triangles[i] = new Tri(tri.A, tri.C, tri.B);
                }
        }

        /// <summary>
        /// Evaluate the interpolated normal at a given position within a triangle.
        /// </summary>
        /// <param name="triId">Triangle ID</param>
        /// <param name="position">Position within the triangle (in world space)</param>
        /// <param name="mesh">The mesh containing the triangle</param>
        /// <returns>Interpolated normal at the position</returns>
        private Vec3D EvaluateNormal(int triId, Vec3D position, MeshNormalUV mesh)
        {
            if (triId < 0 || triId >= mesh.Triangles.Count)
                throw new ArgumentOutOfRangeException(nameof(triId));

            Tri tri = mesh.Triangles[triId];
            Vec3D posA = mesh.Positions[tri.A];
            Vec3D posB = mesh.Positions[tri.B];
            Vec3D posC = mesh.Positions[tri.C];

            // Compute barycentric coordinates of the position within the triangle
            Vec3D baryCoords = ComputeBarycentricCoordinates(position, posA, posB, posC);

            // Get normals from the triangle corner data
            MeshTriangle<TriangleVertexNormalUV> triEx = mesh.TrianglesEx[triId];
            Vec3D normalA = triEx.V0.Normal;
            Vec3D normalB = triEx.V1.Normal;
            Vec3D normalC = triEx.V2.Normal;

            // Interpolate normal using barycentric coordinates
            Vec3D interpolatedNormal = normalA * baryCoords.X + normalB * baryCoords.Y + normalC * baryCoords.Z;
            
            // Normalize the result
            double length = interpolatedNormal.Length();
            if (length > 1e-10)
                return interpolatedNormal / length;
            
            // Fallback: return face normal if interpolation fails
            Vec3D edge1 = posB - posA;
            Vec3D edge2 = posC - posA;
            Vec3D faceNormal = Vec3DOps.Cross(edge1, edge2);
            double faceLength = faceNormal.Length();
            return faceLength > 1e-10 ? faceNormal / faceLength : new Vec3D(0, 0, 1);
        }

        /// <summary>
        /// Compute barycentric coordinates of a point with respect to a triangle.
        /// Returns (u, v, w) where point = u*A + v*B + w*C and u + v + w = 1
        /// </summary>
        private Vec3D ComputeBarycentricCoordinates(Vec3D point, Vec3D a, Vec3D b, Vec3D c)
        {
            Vec3D v0 = b - a;
            Vec3D v1 = c - a;
            Vec3D v2 = point - a;

            double d00 = Vec3DOps.Dot(v0, v0);
            double d01 = Vec3DOps.Dot(v0, v1);
            double d11 = Vec3DOps.Dot(v1, v1);
            double d20 = Vec3DOps.Dot(v2, v0);
            double d21 = Vec3DOps.Dot(v2, v1);

            double denom = d00 * d11 - d01 * d01;
            
            if (Math.Abs(denom) < 1e-10)
            {
                // Degenerate triangle - return equal weights
                return new Vec3D(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
            }

            double v = (d11 * d20 - d01 * d21) / denom;
            double w = (d00 * d21 - d01 * d20) / denom;
            double u = 1.0 - v - w;

            return new Vec3D(u, v, w);
        }

        /// <summary>
        /// Select the surface that faces against the given normal direction.
        /// Uses the split curve to find adjacent triangles and verifies consistency.
        /// </summary>
        /// <param name="surfaces">List of candidate surfaces</param>
        /// <param name="splitCurve">Line segments on triangles from the cut operation</param>
        /// <param name="normals">Reference normals along the cut line</param>
        /// <param name="normalAnchors">Anchor points where the normals are defined</param>
        /// <returns>The surface facing against the normal direction</returns>
        internal Surface SelectSurfaceAgainstNormalDirection(List<Surface> surfaces,
            List<LineSegmentOnTriangle> splitCurve, List<Rat3Hybrid> normals, List<Rat3Hybrid> normalAnchors)
        {
            if (surfaces == null || surfaces.Count == 0)
                throw new ArgumentException("Surface list cannot be empty", nameof(surfaces));
            
            if (splitCurve == null || splitCurve.Count == 0)
                throw new ArgumentException("Split curve cannot be empty", nameof(splitCurve));

            if (normals == null || normals.Count == 0)
                throw new ArgumentException("Normals list cannot be empty", nameof(normals));

            if (normalAnchors == null || normalAnchors.Count != normals.Count)
                throw new ArgumentException("Normal anchors must match normals count", nameof(normalAnchors));
            if (normals.Count != splitCurve.Count)
                throw new ArgumentException("Normals must match trim segments", nameof(normals));

            Surface selectedSurface = null;

            foreach (var surface in surfaces)
            {
                int againstCount = 0, withCount = 0;
                // Classify actual cut-edge fragments, not triangle interiors or
                // a single midpoint that may be a vertex after another cut.
                // Splitter retains source triangle IDs in the transient groups.
                foreach (var edge in Adjacency.BuildEdgeList(surface.Triangles))
                {
                    if (!edge.IsOnBorder) continue;
                    int triIdx = edge.NeighbourIndex1 >= 0 ? edge.NeighbourIndex1 : edge.NeighbourIndex2;
                    var triangle = surface.Triangles[triIdx];
                    var a = surface.PointsPrecise[edge.Start];
                    var b = surface.PointsPrecise[edge.End];
                    for (int segIdx = 0; segIdx < splitCurve.Count; segIdx++)
                    {
                        var seg = splitCurve[segIdx];
                        if (surface.GroupIdPerTriangle[triIdx] != seg.TriangleId) continue;
                        if (!OverlapsBoundarySegment(seg.PointStart, seg.PointEnd, a, b)) continue;
                        var opposite = surface.PointsPrecise[triangle.GetRemaining(edge.Start, edge.End)];
                        int side = Rat3Hybrid.Dot(opposite - normalAnchors[segIdx], normals[segIdx]).Sign();
                        if (side == 0)
                            throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' has a coplanar endpoint contact; the closure side is ambiguous.");
                        if (side < 0) againstCount++; else withCount++;
                        break;
                    }
                }
                if (againstCount > 0 && withCount > 0)
                {
                    throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' cap region crosses its trim contour: {againstCount} adjacent triangles against and {withCount} with the blend normal.");
                }

                // Each contour bounds one kept region. Triangle count is not a
                // geometric selection rule: disconnected candidates are ambiguous.
                if (againstCount > 0 && withCount == 0)
                {
                    if (selectedSurface != null)
                        throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' cap contour bounds multiple regions against the blend normal.");
                    selectedSurface = surface;
                }
            }

            if (selectedSurface == null)
                throw new InvalidOperationException($"Fillet edge '{SourceEdge.Name}' cap contour does not bound a region against the blend normal.");

            return selectedSurface;
        }

        private static bool OverlapsBoundarySegment(Rat3Hybrid start, Rat3Hybrid end,
            Rat3Hybrid a, Rat3Hybrid b)
        {
            var direction = end - start;
            if (direction.IsZero() || a == b ||
                !Rat3Hybrid.Cross(direction, a - start).IsZero() ||
                !Rat3Hybrid.Cross(direction, b - start).IsZero()) return false;
            var first = Rat3Hybrid.Dot(a - start, direction);
            var last = Rat3Hybrid.Dot(b - start, direction);
            return BigRationalHybrid.Min(first, last) < Rat3Hybrid.Dot(direction, direction) &&
                BigRationalHybrid.Max(first, last).Sign() > 0;
        }



        // Splitter preserves the source triangle index in its transient group
        // field, so endpoint patches inherit the support's exact interpolation.
        internal static UVSurface InterpolateClosureAttributes(Surface surface, UVSurface support, CoordinateConverter cc)
        {
            var normals = new Vec3D[surface.PointsPrecise.Count];
            var uv = new Vec2D[surface.PointsPrecise.Count];
            for (int i = 0; i < surface.Triangles.Count; ++i)
            {
                var source = support.Triangles[surface.GroupIdPerTriangle[i]];
                TriangleVertexNormalUV Vertex(int index) => new()
                    { Normal = support.Normals[index], UV = support.Uv[index] };
                var a = Vertex(source.A); var b = Vertex(source.B); var c = Vertex(source.C);
                var triangle = surface.Triangles[i];
                foreach (int index in new[] {triangle.A,triangle.B,triangle.C})
                {
                    var weights = InterpolationHelpers.GetExactBarycentricWeights(
                        surface.PointsPrecise[index], source, support.PointsPrecise);
                    var attributes = a.InterpolateExact(b,c,weights);
                    normals[index] = attributes.Normal;
                    uv[index] = attributes.UV;
                }
            }
            return new UVSurface(cc.Convert(surface.PointsPrecise),normals.ToList(),uv.ToList(),
                surface.Triangles,surface.PointsPrecise);
        }

        public override string ToString()
        {
            return $"BlendEdge: {SourceEdge?.Name ?? "unknown"} (radius={BlendRadius})";
        }
    }

    /// <summary>
    /// Type of edge blend (determines material addition/removal)
    /// </summary>
    public enum EdgeBlendType
    {
        Concave,   // Reentrant edge: adds material.
        Convex     // Exterior edge: removes material.
    }
}
