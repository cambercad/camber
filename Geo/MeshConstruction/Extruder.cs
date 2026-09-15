using GeoCore;
using GeoMeta;

namespace Geo
{
    public class Extruder
    {
        private static double Angle(Vec2D l, Vec2D r)
        {
            double d = (l.X * r.X + l.Y * r.Y) / Math.Sqrt((l.X * l.X + l.Y * l.Y) * (r.X * r.X + r.Y * r.Y));
            if (d <= -1) return Math.PI;
            if (d >= 1) return 0;
            return Math.Acos(d);
        }

        /// <summary>
        /// Rotates <paramref name="frame"/>'s X and Y about tangent Z by <paramref name="twistAngleRadians"/> (RH rule: X toward Y).
        /// </summary>
        private static CoordinateSystem CurveFrameWithTwistAboutZ(CoordinateSystem frame, double twistAngleRadians)
        {
            double c = Math.Cos(twistAngleRadians);
            double s = Math.Sin(twistAngleRadians);
            Vec3D x = c * frame.X + s * frame.Y;
            Vec3D y = -s * frame.X + c * frame.Y;
            return new CoordinateSystem(frame.Origin, x, y, frame.Z);
        }

        /// <summary>
        /// Largest distance from the sketch origin to any profile vertex in the XY plane (twist axis for linear extrude).
        /// </summary>
        private static double MaxProfileRadiusXY(List<ContourData> contourDatas)
        {
            double r2max = 0;
            foreach (var cd in contourDatas)
            {
                foreach (var seg in cd.Contour)
                {
                    foreach (var p in seg)
                    {
                        double r2 = p.X * p.X + p.Y * p.Y;
                        if (r2 > r2max)
                            r2max = r2;
                    }
                }
            }
            return Math.Sqrt(r2max);
        }

        /// <summary>
        /// Number of linear segments along Z so that approximating each profile point's helical path by chord strips
        /// keeps sagitta (arc vs chord in XY) ≤ <paramref name="maxDeviation"/> for radius ≤ <paramref name="profileRadiusMax"/>.
        /// </summary>
        private static int LinearTwistSegmentCountForMaxDeviation(double extrudeSpan, double twistRate, double profileRadiusMax, double maxDeviation)
        {
            if (double.IsNaN(extrudeSpan) || double.IsNaN(maxDeviation))
                return 1;
            if (Math.Abs(extrudeSpan) < 1e-15 || Math.Abs(twistRate) < 1e-20)
                return 1;
            double thetaTotal = Math.Abs(extrudeSpan * twistRate);
            if (thetaTotal < 1e-20)
                return 1;
            if (double.IsPositiveInfinity(maxDeviation) || maxDeviation <= 0)
                return 1;
            if (profileRadiusMax < 1e-15)
                return 1;

            double ratio = maxDeviation / profileRadiusMax;
            if (ratio >= 2.0)
                return 1;

            // Sagitta for chord spanning Δθ on radius r: r * (1 - cos(Δθ/2)). Require ≤ maxDeviation.
            double cosHalfMin = 1.0 - ratio;
            cosHalfMin = Math.Clamp(cosHalfMin, -1.0, 1.0);
            double maxDeltaTheta = 2.0 * Math.Acos(cosHalfMin);
            if (maxDeltaTheta < 1e-12)
                maxDeltaTheta = thetaTotal;

            int m = (int)Math.Ceiling(thetaTotal / maxDeltaTheta);
            m = Math.Max(1, m);
            const int cap = 500_000;
            if (m > cap)
                m = cap;
            return m;
        }

        /// <summary>
        /// Interpolates position and right-handed frame along the chord between two guide samples (used when subdividing the tessellated guide for twist tolerance).
        /// </summary>
        private static CoordinateSystem InterpolateFramesAlongChord(CoordinateSystem a, CoordinateSystem b, double t)
        {
            Vec3D o = (1.0 - t) * a.Origin + t * b.Origin;
            Vec3D z = (1.0 - t) * a.Z + t * b.Z;
            double zLen = z.Length();
            if (zLen < 1e-14)
                z = a.Z.LengthSquared() > 1e-20 ? a.Z.Normalized() : new Vec3D(0, 0, 1);
            else
                z = z / zLen;

            Vec3D xRaw = (1.0 - t) * a.X + t * b.X;
            double dotZX = Vec3DOps.Dot(z, xRaw);
            Vec3D x = xRaw - z * dotZX;
            double xLen = x.Length();
            if (xLen < 1e-14)
                x = Vec3DOps.GetOrthoNormal(z);
            else
                x = x / xLen;

            Vec3D y = Vec3DOps.Cross(z, x);
            double yLen = y.Length();
            if (yLen < 1e-14)
            {
                y = Vec3DOps.Cross(z, Vec3DOps.GetOrthoNormal(z));
                yLen = y.Length();
                if (yLen < 1e-14)
                    y = new Vec3D(0, 1, 0);
                else
                    y = y / yLen;
            }
            else
                y = y / yLen;

            if (Vec3DOps.Dot(Vec3DOps.Cross(x, y), z) < 0)
                y = -y;

            return new CoordinateSystem(o, x, y, z);
        }

        /// <summary>
        /// Inserts extra guide stations along each tessellated edge so chordal approximation of the twisted sweep
        /// meets <paramref name="maxDeviation"/> (same sagitta bound as linear twist subdivision).
        /// </summary>
        private static List<CoordinateSystem> RefineGuideVertexChain(
            List<CoordinateSystem> chain, double twistRatePerExtrudeDistance, double maxDeviation, double profileRadiusMax)
        {
            if (chain == null || chain.Count < 2)
                return chain == null ? new List<CoordinateSystem>() : new List<CoordinateSystem>(chain);

            var result = new List<CoordinateSystem> { chain[0] };
            for (int i = 0; i < chain.Count - 1; i++)
            {
                CoordinateSystem a = chain[i];
                CoordinateSystem b = chain[i + 1];
                double L = (b.Origin - a.Origin).Length();
                int M = LinearTwistSegmentCountForMaxDeviation(L, twistRatePerExtrudeDistance, profileRadiusMax, maxDeviation);
                for (int k = 1; k < M; k++)
                    result.Add(InterpolateFramesAlongChord(a, b, k / (double)M));
                result.Add(b);
            }
            return result;
        }

        private static List<List<CoordinateSystem>> RefineCurveSegmentsForTwist(
            List<List<CoordinateSystem>> curveSegments,
            double twistRatePerExtrudeDistance,
            double maxDeviation,
            double profileRadiusMax)
        {
            if (curveSegments == null || curveSegments.Count == 0)
                return curveSegments;
            if (Math.Abs(twistRatePerExtrudeDistance) < 1e-20
                || double.IsPositiveInfinity(maxDeviation) || maxDeviation <= 0
                || profileRadiusMax < 1e-15)
                return curveSegments;

            var refined = new List<List<CoordinateSystem>>(curveSegments.Count);
            foreach (var seg in curveSegments)
            {
                if (seg == null || seg.Count < 2)
                    refined.Add(seg != null ? new List<CoordinateSystem>(seg) : new List<CoordinateSystem>());
                else
                    refined.Add(RefineGuideVertexChain(seg, twistRatePerExtrudeDistance, maxDeviation, profileRadiusMax));
            }
            return refined;
        }

        /// <summary>
        /// Generates an extruded mesh from a single 2D contour (CCW when viewed from +Z).
        /// Segments the contour by angle threshold for smooth normals, then extrudes.
        /// </summary>
        /// <param name="twistRatePerExtrudeDistance">Radians of twist per unit distance along local +Z from the bottom of the extrusion to each station (total at top = height × rate for unidirectional extrude).</param>
        /// <param name="maxDeviation">
        /// For twisted linear extrusion, maximum allowed geometric deviation (same length units as the mesh) when approximating
        /// the true twisted sweep with chord strips between cross-sections. Use <see cref="double.PositiveInfinity"/> to keep a single side-wall span (legacy behavior).
        /// </param>
        public static int GenerateExtrudedMesh(CoordinateConverter converter, CoordinateSystem location, List<Vec2D> contour, double angleThresholdRadian, double height,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv, List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            double twistRatePerExtrudeDistance = 0, double maxDeviation = double.PositiveInfinity, int baseGroupIndex = 0)
        {
            if (contour == null || contour.Count < 3)
                return 0;

            contour = new List<Vec2D>(contour);
            if(!contour[0].Equals(contour[contour.Count - 1]))
            {
                contour.Add(contour[0]); // Ensure closed contour
            }

            // Split contour into segments based on angle threshold
            List<List<Vec2D>> segments = new List<List<Vec2D>>();
            List<Vec2D> currentSegment = new List<Vec2D>();
            currentSegment.Add(contour[0]);

            for (int i = 1; i <= contour.Count; i++)
            {
                int currentIndex = i % contour.Count;
                int prevIndex = (i - 1) % contour.Count;
                int nextIndex = (i + 1) % contour.Count;
                
                Vec2D prev = contour[prevIndex];
                Vec2D curr = contour[currentIndex];
                Vec2D next = contour[nextIndex];

                // Calculate angle between current edges
                Vec2D edge1 = new Vec2D(curr.X - prev.X, curr.Y - prev.Y);
                Vec2D edge2 = new Vec2D(next.X - curr.X, next.Y - curr.Y);

                // Skip if edges are too small
                if (edge1.X * edge1.X + edge1.Y * edge1.Y < 1e-10 || 
                    edge2.X * edge2.X + edge2.Y * edge2.Y < 1e-10)
                {
                    if (i < contour.Count) // Only add if not the wrap-around iteration
                        currentSegment.Add(curr);
                    continue;
                }

                double angle = Math.Abs(Angle(edge1, edge2));

                if (i < contour.Count) // Only add if not the wrap-around iteration
                    currentSegment.Add(curr);

                // Start new segment if angle exceeds threshold
                if (angle > angleThresholdRadian)
                {
                    if (currentSegment.Count >= 2)
                    {
                        segments.Add(new List<Vec2D>(currentSegment));
                    }
                    currentSegment.Clear();
                    currentSegment.Add(curr);
                }
            }

            // Add final segment if it has enough points
            if (currentSegment.Count >= 2)
            {
                segments.Add(new List<Vec2D>(currentSegment));
            }

            // Compute the contour normals - smoothing per segment strip
            List<List<Vec2D>> contourNormals = new List<List<Vec2D>>();
            for (int i = 0; i < segments.Count; ++i)
            {
                var normals2D = WeightedNormals2D(segments[i]);
                if (normals2D.Count > 0)
                {
                    contourNormals.Add(normals2D);
                }
            }

            if (segments.Count == 0 || contourNormals.Count == 0)
                return 0;

            // Generate mesh for each segment
            int numGroups = GenerateExtrudedMesh(converter, new List<List<List<Vec2D>>>() { segments },
                new List<List<List<Vec2D>>>() { contourNormals }, 0, height, triangles, vertices, normals, uv, triangleGroups, precisePositions,
                twistRatePerExtrudeDistance, maxDeviation, baseGroupIndex);

            // Transform vertices and normals from local coordinate system to world space
            for (int i = 0; i < vertices.Count; ++i)
                vertices[i] = location.PointFromCoordSysToWorld(vertices[i]);
            for (int i = 0; i < normals.Count; ++i)
                normals[i] = location.DirectionFromCoordSysToWorld(normals[i]);
            
            // Regenerate precise positions from transformed vertices
            precisePositions.Clear();
            for (int i = 0; i < vertices.Count; ++i)
            {
                Int3 intPos = converter.Convert(vertices[i]);
                precisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
            }

            return numGroups;
        }

        public static List<Vec2D> WeightedNormals2D(List<Vec2D> contour)
        {
            if (contour == null || contour.Count < 2)
                return new List<Vec2D>();

            int count = contour.Count;
            List<Vec2D> normals = new List<Vec2D>(count);
            
            // Step 1: Initialize normals with zero vectors
            for (int i = 0; i < count; i++)
                normals.Add(new Vec2D(0, 0));

            // Step 2: Accumulate weighted normals per node
            for (int i = 0; i < count - 1; i++)
            {
                Vec2D curr = contour[i];
                Vec2D next = contour[i + 1];
                
                // Calculate edge vector
                Vec2D edge = new Vec2D(next.X - curr.X, next.Y - curr.Y);
                
                // Calculate edge length (weight)
                double length = Math.Sqrt(edge.X * edge.X + edge.Y * edge.Y);
                
                if (length > 1e-8)
                {
                    // Right-hand perpendicular: outward for the CCW contours used by extrusion.
                    Vec2D normal = new Vec2D(edge.Y / length, -edge.X / length);
                    
                    // Add weighted normal to both vertices of the edge
                    normals[i] = new Vec2D(
                        normals[i].X + normal.X * length,
                        normals[i].Y + normal.Y * length
                    );
                    normals[i + 1] = new Vec2D(
                        normals[i + 1].X + normal.X * length,
                        normals[i + 1].Y + normal.Y * length
                    );
                }
            }

            // Step 3: Normalize all normals
            for (int i = 0; i < count; i++)
            {
                double len = Math.Sqrt(normals[i].X * normals[i].X + normals[i].Y * normals[i].Y);
                if (len > 1e-8)
                {
                    var n = normals[i];
                    n.X /= len;
                    n.Y /= len;
                    normals[i] = n;
                }
            }
            
            return normals;
        }

        /// <summary>
        /// Generates an extruded mesh from contour loops with naming (unidirectional: extrudes upward only).
        /// </summary>
        /// <param name="twistRatePerExtrudeDistance">Radians of twist per unit distance along the extrusion (local +Z). Total twist at the top is <c>(height) * twistRatePerExtrudeDistance</c>.</param>
        /// <param name="maxDeviation">Twisted linear side-wall tolerance (same units as geometry); use <c>double.PositiveInfinity</c> for one side-wall step only.</param>
        public static int GenerateExtrudedMesh(CoordinateConverter converter, CoordinateSystem location,
            List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals, double height,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            List<List<string>> contourNames, string extrudeName,
            out Dictionary<int, string> triangleGroupToName, double twistRatePerExtrudeDistance = 0,
            double maxDeviation = double.PositiveInfinity, int baseGroupIndex = 0)
        {
            return GenerateExtrudedMesh(converter, location, contours, contourNormals,
                height, 0, triangles, vertices, normals, uv, triangleGroups, precisePositions,
                contourNames, extrudeName, out triangleGroupToName, twistRatePerExtrudeDistance, maxDeviation, baseGroupIndex);
        }

        /// <summary>
        /// Generates an extruded mesh with bidirectional extrusion (MeshOutput/MeshNaming variant).
        /// </summary>
        /// <param name="twistRatePerExtrudeDistance">Radians of twist per unit distance along local +Z from <c>bottomZ = -heightNegative</c> to <c>topZ = heightPositive</c>.</param>
        /// <param name="maxDeviation">Twisted linear side-wall tolerance (see single-contour overload).</param>
        public static int GenerateExtrudedMesh(CoordinateConverter converter, CoordinateSystem location,
            List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals,
            double heightPositive, double heightNegative,
            MeshOutput output, MeshNaming naming, double twistRatePerExtrudeDistance = 0,
            double maxDeviation = double.PositiveInfinity, int baseGroupIndex = 0)
        {
            var reversalInfo = MeshConstructionHelpers.GetContourReversalInfo(contours);
            var orientedContours = MeshConstructionHelpers.ApplyReversalToContours(contours, reversalInfo);
            var orientedNormals = MeshConstructionHelpers.ApplyReversalToContourNormals(contourNormals, reversalInfo);
            MeshConstructionHelpers.OrientContourNormalsToContours(orientedContours, orientedNormals);

            double bottomZ = -heightNegative;
            double topZ = heightPositive;

            var res = GenerateExtrudedMesh(converter, orientedContours, orientedNormals, bottomZ, topZ,
                output.Triangles, output.Vertices, output.Normals, output.UVs, output.TriangleGroups,
                output.PrecisePositions, twistRatePerExtrudeDistance, maxDeviation, baseGroupIndex);

            PopulateExtrudeNaming(naming, orientedContours, baseGroupIndex);
            output.TransformToWorldSpace(converter, location);
            return res;
        }

        /// <summary>
        /// Generates an extruded mesh with bidirectional extrusion (list-based variant).
        /// </summary>
        public static int GenerateExtrudedMesh(CoordinateConverter converter, CoordinateSystem location,
            List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals,
            double heightPositive, double heightNegative,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            List<List<string>> contourNames, string extrudeName,
            out Dictionary<int, string> triangleGroupToName, double twistRatePerExtrudeDistance = 0,
            double maxDeviation = double.PositiveInfinity, int baseGroupIndex = 0)
        {
            var output = new MeshOutput();
            var naming = new MeshNaming { ContourNames = contourNames, OperationName = extrudeName };
            var res = GenerateExtrudedMesh(converter, location, contours, contourNormals,
                heightPositive, heightNegative, output, naming, twistRatePerExtrudeDistance, maxDeviation, baseGroupIndex);
            output.AppendTo(triangles, vertices, normals, uv, triangleGroups, precisePositions);
            triangleGroupToName = naming.TriangleGroupToName;
            return res;
        }

        /// <summary>
        /// Extrudes contours along a 3D curve path (given as multiple connected segments),
        /// creating cross-sections at each curve vertex. Flattens the segments and delegates
        /// to the single-list overload.
        /// </summary>
        /// <param name="twistRatePerExtrudeDistance">Radians of twist per unit arc length along the guide from the first vertex.</param>
        public static int GenerateExtrudeAlongCurve(CoordinateConverter converter, CoordinateSystem contourCS,
            List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals,
            List<List<CoordinateSystem>> extrudeGuide,
            CoordinateSystem profileSketchPlane,
            Vec3D sweepStartTangentWorld,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            List<List<string>> contourNames, string extrudeName,
            out Dictionary<int, string> triangleGroupToName, double twistRatePerExtrudeDistance = 0,
            double maxDeviation = double.PositiveInfinity, int baseGroupIndex = 0)
        {
            var flat = FlattenGuideSegments(extrudeGuide);
            return GenerateExtrudeAlongCurve(converter, contourCS, contours, contourNormals, flat,
                profileSketchPlane, sweepStartTangentWorld,
                triangles, vertices, normals, uv, triangleGroups, precisePositions,
                contourNames, extrudeName, out triangleGroupToName, twistRatePerExtrudeDistance, maxDeviation, baseGroupIndex);
        }

        private static List<CoordinateSystem> FlattenGuideSegments(List<List<CoordinateSystem>> segments)
        {
            var flat = new List<CoordinateSystem>();
            flat.AddRange(segments[0]);
            for (int i = 1; i < segments.Count; i++)
            {
                var last = flat[flat.Count - 1];
                var list = segments[i];
                if ((list[0].Origin - last.Origin).LengthSquared() > 1e-16)
                    throw new Exception("All curves in extrudeGuide must be adjacent");
                for (int j = 1; j < list.Count; j++)
                    flat.Add(list[j]);
            }
            return flat;
        }

        /// <summary>
        /// Extrudes contours along a multi-segment curve strip, creating separate triangle groups
        /// for each (contour segment, guide curve) combination.
        /// </summary>
        /// <param name="twistRatePerExtrudeDistance">Radians of twist per unit arc length along the flattened guide polyline.</param>
        public static int GenerateExtrudeAlongCurveStripSectioned(
            CoordinateConverter converter,
            CoordinateSystem contourCS,
            List<List<List<Vec2D>>> contours,
            List<List<List<Vec2D>>> contourNormals,
            List<List<CoordinateSystem>> curveSegments,
            List<string> guideCurveNames,
            CoordinateSystem profileSketchPlane,
            Vec3D sweepStartTangentWorld,
            List<Tri> triangles,
            List<Vec3D> vertices,
            List<Vec3D> normals,
            List<Vec2D> uv,
            List<int> triangleGroups,
            List<Rat3Hybrid> precisePositions,
            List<List<string>> contourNames,
            string extrudeName,
            out Dictionary<int, string> triangleGroupToName,
            double twistRatePerExtrudeDistance = 0,
            double maxDeviation = double.PositiveInfinity,
            int baseGroupIndex = 0)
        {
            return GenerateCurveExtrusionCore(converter, contourCS, contours, contourNormals,
                curveSegments, guideCurveNames, profileSketchPlane, sweepStartTangentWorld,
                triangles, vertices, normals, uv, triangleGroups, precisePositions,
                contourNames, extrudeName, out triangleGroupToName, twistRatePerExtrudeDistance, maxDeviation, baseGroupIndex);
        }

        /// <summary>
        /// Shared core for both single-guide and multi-segment curve extrusion.
        /// Flattens guide segments, generates cross-sections, caps, side walls, and transforms to world space.
        /// </summary>
        private static int GenerateCurveExtrusionCore(CoordinateConverter converter, CoordinateSystem contourCS,
            List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals,
            List<List<CoordinateSystem>> curveSegments, List<string> guideCurveNames,
            CoordinateSystem profileSketchPlane,
            Vec3D sweepStartTangentWorld,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            List<List<string>> contourNames, string extrudeName, out Dictionary<int, string> triangleGroupToName,
            double twistRatePerExtrudeDistance, double maxDeviation = double.PositiveInfinity, int baseGroupIndex = 0)
        {
            if (contours == null || contours.Count == 0 || curveSegments == null || curveSegments.Count == 0)
            {
                triangleGroupToName = new Dictionary<int, string>();
                return 0;
            }

            for (int i = 1; i < curveSegments.Count; i++)
            {
                var prevEnd = curveSegments[i - 1][curveSegments[i - 1].Count - 1].Origin;
                var currStart = curveSegments[i][0].Origin;
                if ((prevEnd - currStart).LengthSquared() > 1e-12)
                    throw new Exception("All curves in curveSegments must be adjacent");
            }

            Vec3D firstPos = curveSegments[0][0].Origin;
            Vec3D lastPos = curveSegments[curveSegments.Count - 1][curveSegments[curveSegments.Count - 1].Count - 1].Origin;
            bool isClosedLoop = IsClosedLoop(firstPos, lastPos);

            var reversalInfo = MeshConstructionHelpers.GetContourReversalInfo(contours);
            var orientedContours = MeshConstructionHelpers.ApplyReversalToContours(contours, reversalInfo);
            var orientedNormals = MeshConstructionHelpers.ApplyReversalToContourNormals(contourNormals, reversalInfo);
            MeshConstructionHelpers.OrientContourNormalsToContours(orientedContours, orientedNormals);
            AlignCurveExtrudeContourToSweepDirection(orientedContours, orientedNormals, profileSketchPlane, sweepStartTangentWorld);
            var contourDatas = PrepareContourData(orientedContours, orientedNormals, 0);

            int numContourSegments = CountContourSegments(orientedContours);

            double rMax = MaxProfileRadiusXY(contourDatas);
            List<List<CoordinateSystem>> guideForFlatten = RefineCurveSegmentsForTwist(
                curveSegments, twistRatePerExtrudeDistance, maxDeviation, rMax);

            // Flatten curve segments into a single vertex list, tracking which guide segment owns each wall
            var (allVertices, wallToGuideIndex) = FlattenCurveSegmentsWithWallMapping(guideForFlatten, isClosedLoop);

            if (allVertices.Count < 2)
            {
                triangleGroupToName = new Dictionary<int, string>();
                return 0;
            }

            // Compute arc lengths and generate cross-sections
            var arcLengths = ComputeArcLengths(allVertices);
            double curveTotalLength = arcLengths[arcLengths.Count - 1];

            var (crossSectionVertices, crossSectionNormals, crossSectionUVs) =
                GenerateAllCrossSections(contourDatas, allVertices, arcLengths, curveTotalLength, twistRatePerExtrudeDistance);

            // UV scaling for end caps
            MinMax(contourDatas, out Vec2D min, out Vec2D max);
            double size = Math.Max(max.X - min.X, max.Y - min.Y);
            double scaling = 1.0 / size;

            int bottomCapVertexIndex = vertices.Count;
            int topCapVertexIndex = vertices.Count;

            if (!isClosedLoop)
            {
                bottomCapVertexIndex = vertices.Count;
                EmitCapVertices(contourDatas, allVertices[0], -allVertices[0].Z, min, scaling,
                    converter, vertices, normals, uv, precisePositions, 0);

                topCapVertexIndex = vertices.Count;
                var lastVertex = allVertices[allVertices.Count - 1];
                EmitCapVertices(contourDatas, lastVertex, lastVertex.Z, min, scaling,
                    converter, vertices, normals, uv, precisePositions, curveTotalLength * twistRatePerExtrudeDistance);
            }

            int baseVertexIndex = vertices.Count;
            EmitCrossSectionVertices(crossSectionVertices, crossSectionNormals, crossSectionUVs,
                converter, vertices, normals, uv, precisePositions);

            int numCrossSectionsForWalls = allVertices.Count;
            if (isClosedLoop)
            {
                EmitSeamVertices(crossSectionVertices[0], crossSectionNormals[0], crossSectionUVs[0],
                    converter, vertices, normals, uv, precisePositions);
                wallToGuideIndex.Add(0);
                numCrossSectionsForWalls = allVertices.Count + 1;
            }

            // Build naming
            triangleGroupToName = new Dictionary<int, string>();
            int groupIndex = baseGroupIndex;
            for (int guideIdx = 0; guideIdx < curveSegments.Count; guideIdx++)
            {
                string guideName = guideCurveNames[guideIdx];
                for (int loopIndex = 0; loopIndex < orientedContours.Count; loopIndex++)
                {
                    var loopContourNames = contourNames[loopIndex];
                    for (int curveIndex = 0; curveIndex < orientedContours[loopIndex].Count; curveIndex++)
                    {
                        string contourName = loopContourNames[curveIndex];
                        EntityNaming.ValidateContourSegmentName(contourName);
                        triangleGroupToName.Add(groupIndex, EntityNaming.ExtrudeSideWithGuide(extrudeName, contourName, guideName));
                        groupIndex++;
                    }
                }
            }

            int numSideGroups = numContourSegments * curveSegments.Count;

            if (!isClosedLoop)
            {
                triangleGroupToName.Add(baseGroupIndex + numSideGroups, EntityNaming.ExtrudeBottom(extrudeName));
                triangleGroupToName.Add(baseGroupIndex + numSideGroups + 1, EntityNaming.ExtrudeTop(extrudeName));
                GenerateEndCapsForCurveExtrusion(contourDatas, precisePositions, bottomCapVertexIndex, topCapVertexIndex,
                    triangles, triangleGroups, numSideGroups, baseGroupIndex);
            }

            GenerateSideWallsForCurveStrip(contourDatas, numCrossSectionsForWalls, baseVertexIndex,
                wallToGuideIndex, numContourSegments, curveSegments.Count, false,
                triangles, triangleGroups, baseGroupIndex);

            TransformToWorldAndRegeneratePrecise(converter, contourCS, vertices, normals, precisePositions);

            return isClosedLoop ? numSideGroups : numSideGroups + 2;
        }

        /// <summary>
        /// After <see cref="MeshConstructionHelpers.GetContourReversalInfo"/> forces CCW in sketch (u,v), reverses every contour segment if
        /// <c>Cross(profile X, profile Y)</c> (RH normal of the sketch plane) points opposite the curve’s sweep tangent
        /// at the path start. Pass the tessellated vertex tangent from the guide (world space, before per-vertex frame
        /// remapping in <see cref="GeoAPI.OrientCurveFramesToProfile"/>), not <c>curveSegments[0][0].Z</c> after orient —
        /// that Z is aligned with sketch Z and would make the test tautological.
        /// </summary>
        private static void AlignCurveExtrudeContourToSweepDirection(
            List<List<List<Vec2D>>> contours,
            List<List<List<Vec2D>>> contourNormals,
            CoordinateSystem profileSketchPlane,
            Vec3D sweepStartTangentWorld)
        {
            Vec3D sketchNormal = Vec3DOps.Cross(profileSketchPlane.X, profileSketchPlane.Y);
            double nLen = sketchNormal.Length();
            double tLen = sweepStartTangentWorld.Length();
            if (nLen < 1e-20 || tLen < 1e-20)
                return;
            sketchNormal = sketchNormal / nLen;
            Vec3D tangent = sweepStartTangentWorld / tLen;

            if (Vec3DOps.Dot(sketchNormal, tangent) >= 0)
                return;
            ReverseAllCurveContourSegments(contours);
            ReverseAllCurveContourSegments(contourNormals);
        }

        private static void ReverseAllCurveContourSegments(List<List<List<Vec2D>>> loops)
        {
            foreach (List<List<Vec2D>> loop in loops)
            {
                foreach (List<Vec2D> segment in loop)
                    segment.Reverse();
                loop.Reverse();
            }
        }

        private static (List<CoordinateSystem> allVertices, List<int> wallToGuideIndex) FlattenCurveSegmentsWithWallMapping(
            List<List<CoordinateSystem>> curveSegments, bool isClosedLoop)
        {
            var allVertices = new List<CoordinateSystem>();
            var wallStartIndices = new List<int>();
            int currentWallIndex = 0;

            for (int segIdx = 0; segIdx < curveSegments.Count; segIdx++)
            {
                var segment = curveSegments[segIdx];
                int startJ = (segIdx == 0) ? 0 : 1;
                wallStartIndices.Add(currentWallIndex);
                int verticesAddedThisSegment = 0;

                for (int j = startJ; j < segment.Count; j++)
                {
                    if (isClosedLoop && segIdx == curveSegments.Count - 1 && j == segment.Count - 1)
                        continue;
                    allVertices.Add(segment[j]);
                    verticesAddedThisSegment++;
                }

                if (segIdx == 0)
                    currentWallIndex += verticesAddedThisSegment - 1;
                else
                    currentWallIndex += verticesAddedThisSegment;
            }

            int numWalls = isClosedLoop ? allVertices.Count : allVertices.Count - 1;
            var wallToGuideIndex = new List<int>(numWalls);
            for (int wallIdx = 0; wallIdx < numWalls; wallIdx++)
            {
                int guideIdx = curveSegments.Count - 1;
                for (int s = curveSegments.Count - 1; s >= 0; s--)
                {
                    if (wallIdx >= wallStartIndices[s])
                    {
                        guideIdx = s;
                        break;
                    }
                }
                wallToGuideIndex.Add(guideIdx);
            }

            return (allVertices, wallToGuideIndex);
        }

        private static List<double> ComputeArcLengths(List<CoordinateSystem> guideVertices)
        {
            var arcLengths = new List<double>(guideVertices.Count);
            arcLengths.Add(0);
            for (int i = 1; i < guideVertices.Count; i++)
            {
                double segLen = (guideVertices[i].Origin - guideVertices[i - 1].Origin).Length();
                arcLengths.Add(arcLengths[i - 1] + segLen);
            }
            return arcLengths;
        }

        private static (List<List<Vec3D>> verts, List<List<Vec3D>> norms, List<List<Vec2D>> uvs) GenerateAllCrossSections(
            List<ContourData> contourDatas, List<CoordinateSystem> guideVertices,
            List<double> arcLengths, double totalLength, double twistRatePerExtrudeDistance)
        {
            var crossSectionVertices = new List<List<Vec3D>>();
            var crossSectionNormals = new List<List<Vec3D>>();
            var crossSectionUVs = new List<List<Vec2D>>();

            for (int i = 0; i < guideVertices.Count; i++)
            {
                double normalizedArcLength = (totalLength > 1e-10)
                    ? (arcLengths[i] / totalLength)
                    : ((double)i / Math.Max(1, guideVertices.Count - 1));

                var sectionVerts = new List<Vec3D>();
                var sectionNorms = new List<Vec3D>();
                var sectionUVs = new List<Vec2D>();

                foreach (var contourData in contourDatas)
                {
                    var (verts, norms, uvs) = GenerateCrossSectionAtCurveVertex(
                        contourData, guideVertices[i], normalizedArcLength, arcLengths[i], twistRatePerExtrudeDistance);
                    sectionVerts.AddRange(verts);
                    sectionNorms.AddRange(norms);
                    sectionUVs.AddRange(uvs);
                }

                crossSectionVertices.Add(sectionVerts);
                crossSectionNormals.Add(sectionNorms);
                crossSectionUVs.Add(sectionUVs);
            }

            return (crossSectionVertices, crossSectionNormals, crossSectionUVs);
        }

        private static void EmitCapVertices(List<ContourData> contourDatas, CoordinateSystem curveVertex, Vec3D capNormal,
            Vec2D uvMin, double uvScaling, CoordinateConverter converter,
            List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv, List<Rat3Hybrid> precisePositions,
            double twistAngleRadians)
        {
            CoordinateSystem capFrame = Math.Abs(twistAngleRadians) < 1e-20
                ? curveVertex
                : CurveFrameWithTwistAboutZ(curveVertex, twistAngleRadians);
            foreach (var contourData in contourDatas)
            {
                for (int i = 0; i < contourData.ContourDuplicateFree.Count; i++)
                {
                    var v2d = contourData.ContourDuplicateFree[i];
                    Vec3D worldPos = capFrame.PointTo3D(v2d);
                    vertices.Add(worldPos);
                    uv.Add(new Vec2D((v2d.X - uvMin.X) * uvScaling, (v2d.Y - uvMin.Y) * uvScaling));
                    normals.Add(capNormal);
                    Int3 intPos = converter.Convert(worldPos);
                    precisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
                }
            }
        }

        private static void EmitCrossSectionVertices(
            List<List<Vec3D>> crossSectionVertices, List<List<Vec3D>> crossSectionNormals, List<List<Vec2D>> crossSectionUVs,
            CoordinateConverter converter, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv, List<Rat3Hybrid> precisePositions)
        {
            for (int i = 0; i < crossSectionVertices.Count; i++)
            {
                foreach (var vertex in crossSectionVertices[i])
                {
                    vertices.Add(vertex);
                    Int3 intPos = converter.Convert(vertex);
                    precisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
                }
                normals.AddRange(crossSectionNormals[i]);
                uv.AddRange(crossSectionUVs[i]);
            }
        }

        private static void EmitSeamVertices(List<Vec3D> firstSectionVerts, List<Vec3D> firstSectionNormals, List<Vec2D> firstSectionUVs,
            CoordinateConverter converter, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv, List<Rat3Hybrid> precisePositions)
        {
            foreach (var vertex in firstSectionVerts)
            {
                vertices.Add(vertex);
                Int3 intPos = converter.Convert(vertex);
                precisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
            }
            normals.AddRange(firstSectionNormals);
            foreach (var origUV in firstSectionUVs)
                uv.Add(new Vec2D(origUV.X, 1.0));
        }

        private static void TransformToWorldAndRegeneratePrecise(CoordinateConverter converter, CoordinateSystem contourCS,
            List<Vec3D> vertices, List<Vec3D> normals, List<Rat3Hybrid> precisePositions)
        {
            for (int i = 0; i < vertices.Count; ++i)
                vertices[i] = contourCS.PointFromCoordSysToWorld(vertices[i]);
            for (int i = 0; i < normals.Count; ++i)
                normals[i] = contourCS.DirectionFromCoordSysToWorld(normals[i]);

            precisePositions.Clear();
            for (int i = 0; i < vertices.Count; ++i)
            {
                Int3 intPos = converter.Convert(vertices[i]);
                precisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
            }
        }

        /// <summary>
        /// Generates side walls for curve strip extrusion with per-guide-curve triangle groups.
        /// wallToGuideIndex maps each wall index (cross-section i to i+1) to the owning guide curve index.
        /// </summary>
        private static void GenerateSideWallsForCurveStrip(List<ContourData> contourDatas, int numCrossSections,
            int baseVertexIndex, List<int> wallToGuideIndex, int numContourSegments, int numGuideCurves, bool isClosedLoop,
            List<Tri> triangles, List<int> triangleGroups, int baseGroupIndex)
        {
            int verticesPerSection = 0;
            foreach (var contourData in contourDatas)
                verticesPerSection += contourData.TotalVertexCount;

            // Generate walls between consecutive cross-sections
            int numWalls = isClosedLoop ? numCrossSections : numCrossSections - 1;
            
            for (int wallIdx = 0; wallIdx < numWalls; wallIdx++)
            {
                int currentSectionIdx = wallIdx;
                int nextSectionIdx = (wallIdx + 1) % numCrossSections;
                
                int currentSectionOffset = baseVertexIndex + currentSectionIdx * verticesPerSection;
                int nextSectionOffset = baseVertexIndex + nextSectionIdx * verticesPerSection;

                // Determine which guide curve this wall segment belongs to
                int guideIndex = wallToGuideIndex[wallIdx];

                // Generate walls with the appropriate group offset for this guide curve
                int groupOffset = baseGroupIndex + guideIndex * numContourSegments;

                GenerateSideWallsBetweenLayersWithGroupOffset(contourDatas, currentSectionOffset, nextSectionOffset,
                    triangles, triangleGroups, groupOffset);
            }
        }

        /// <summary>
        /// Generates side walls between two layers with a custom group offset
        /// </summary>
        private static void GenerateSideWallsBetweenLayersWithGroupOffset(List<ContourData> contourDatas,
            int bottomLayerOffset, int topLayerOffset, List<Tri> triangles, List<int> triangleGroups, int groupOffset)
        {
            int offset = 0;
            int contourSegmentIndex = 0;
            
            foreach (var contourData in contourDatas)
            {
                for (int segmentIndex = 0; segmentIndex < contourData.Contour.Count; segmentIndex++)
                {
                    var segment = contourData.Contour[segmentIndex];
                    int groupId = groupOffset + contourSegmentIndex;

                    for (int j = 1; j < segment.Count; j++)
                    {
                        int i = j - 1;

                        int i0 = bottomLayerOffset + i + offset;
                        int i1 = bottomLayerOffset + j + offset;
                        int i2 = topLayerOffset + i + offset;
                        int i3 = topLayerOffset + j + offset;

                        var tri1 = new Tri(i0, i1, i2);
                        var tri2 = new Tri(i1, i3, i2);

                        // Skip degenerate triangles with duplicate indices
                        if (!MeshConstructionHelpers.IsDegenerateTriangle(tri1))
                        {
                            triangles.Add(tri1);
                            triangleGroups.Add(groupId);
                        }
                        if (!MeshConstructionHelpers.IsDegenerateTriangle(tri2))
                        {
                            triangles.Add(tri2);
                            triangleGroups.Add(groupId);
                        }
                    }
                    offset += segment.Count;
                    contourSegmentIndex++;
                }
            }
        }
        /// <summary>
        /// Extrudes contours along a single guide curve. For multi-segment guides, use the List&lt;List&lt;CoordinateSystem&gt;&gt; overload.
        /// Wraps the guide as a single-segment strip and remaps group names to the simpler (no guide curve name) format.
        /// </summary>
        /// <param name="profileSketchPlane">
        /// <see cref="PlotterSketcherCoordSys.CoordinateSystem"/> of the profile: X,Y span the sketch; CCW in (X,Y) uses
        /// RH normal <c>Cross(X,Y)</c>. If that normal opposes the guide tangent at the start, contour winding is reversed.
        /// </param>
        /// <param name="sweepStartTangentWorld">
        /// Guide tangent at the sweep start in world space (normalized inside the extruder), e.g. first tessellated
        /// vertex tangent after the same cyclic segment ordering as <see cref="GeoAPI.OrientCurveFramesToProfile"/>.
        /// </param>
        /// <param name="twistRatePerExtrudeDistance">Radians of twist per unit arc length along the guide from the first vertex.</param>
        public static int GenerateExtrudeAlongCurve(CoordinateConverter converter, CoordinateSystem contourCS, List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals, List<CoordinateSystem> extrudeGuide,
            CoordinateSystem profileSketchPlane,
            Vec3D sweepStartTangentWorld,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv, List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            List<List<string>> contourNames, string extrudeName, out Dictionary<int, string> triangleGroupToName,
            double twistRatePerExtrudeDistance = 0, double maxDeviation = double.PositiveInfinity, int baseGroupIndex = 0)
        {
            if (contours == null || contours.Count == 0 || extrudeGuide == null || extrudeGuide.Count < 2)
            {
                triangleGroupToName = new Dictionary<int, string>();
                return 0;
            }

            const string singleGuideName = "";
            int result = GenerateCurveExtrusionCore(converter, contourCS, contours, contourNormals,
                new List<List<CoordinateSystem>> { extrudeGuide },
                new List<string> { singleGuideName },
                profileSketchPlane, sweepStartTangentWorld,
                triangles, vertices, normals, uv, triangleGroups, precisePositions,
                contourNames, extrudeName, out triangleGroupToName, twistRatePerExtrudeDistance, maxDeviation, baseGroupIndex);

            // Rewrite group names: strip uses "ExtrudeName-ContourName-GuideName" format,
            // but single-guide uses "ExtrudeName-ContourName" format. Remap by stripping the trailing dash.
            var remapped = new Dictionary<int, string>();
            foreach (var kv in triangleGroupToName)
            {
                string name = kv.Value;
                if (name.EndsWith("-"))
                    name = name.Substring(0, name.Length - 1);
                remapped[kv.Key] = name;
            }
            triangleGroupToName = remapped;
            return result;
        }

        /// <summary>
        /// Lower-level extrusion: contours must already be oriented CCW. Generates side walls, top and bottom caps.
        /// </summary>
        private static int GenerateExtrudedMesh(CoordinateConverter converter, List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals, double bottomZ, double topZ,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv, List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            double twistRatePerExtrudeDistance = 0, double maxDeviation = double.PositiveInfinity, int baseGroupIndex = 0)
        {
            int triangleBegin = triangles.Count;
            foreach (var c in contours)
            {
                if (c.Count == 1 && c[0][0] != c[0].Last())
                    throw new Exception();
            }

            double extrudeSpan = topZ - bottomZ;
            double topTwistAngle = extrudeSpan * twistRatePerExtrudeDistance;

            var contourDatas = PrepareContourData(contours, contourNormals, baseGroupIndex);

            double rMax = MaxProfileRadiusXY(contourDatas);
            int segmentCount = LinearTwistSegmentCountForMaxDeviation(extrudeSpan, twistRatePerExtrudeDistance, rMax, maxDeviation);

            Vec2D min, max;
            MinMax(contourDatas, out min, out max);
            double size = Math.Max(max.X - min.X, max.Y - min.Y);
            double scaling = 1.0 / size;

            int baseVertexIndex = vertices.Count;

            foreach (var contourData in contourDatas)
            {
                for (int i = 0; i < contourData.ContourDuplicateFree.Count; ++i)
                {
                    var v = contourData.ContourDuplicateFree[i];
                    Vec3D vertex = new Vec3D(v.X, v.Y, bottomZ);
                    vertices.Add(vertex);
                    uv.Add(new Vec2D((v.X - min.X) * scaling, (v.Y - min.Y) * scaling));
                    normals.Add(new Vec3D(0, 0, -1));
                    Int3 intPos = converter.Convert(vertex);
                    precisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
                }
            }

            int topOffsetIndex = vertices.Count;
            foreach (var contourData in contourDatas)
            {
                foreach (var v in contourData.ContourDuplicateFree)
                {
                    Vec2D vr = Vec2DOps.Rotated(v, topTwistAngle);
                    Vec3D vertex = new Vec3D(vr.X, vr.Y, topZ);
                    vertices.Add(vertex);
                    uv.Add(new Vec2D((v.X - min.X) * scaling, (v.Y - min.Y) * scaling));
                    normals.Add(new Vec3D(0, 0, 1));
                    Int3 intPos = converter.Convert(vertex);
                    precisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
                }
            }

            var polygonIndices = BuildCapPolygonIndices(contourDatas, baseVertexIndex);

            int contourIndexOffset = vertices.Count;

            int verticesPerSideLayer = 0;
            foreach (var contourData in contourDatas)
                verticesPerSideLayer += contourData.TotalVertexCount;

            for (int ring = 0; ring <= segmentCount; ring++)
            {
                double t = ring / (double)segmentCount;
                double z = bottomZ + t * extrudeSpan;
                double theta = t * topTwistAngle;

                foreach (var contourData in contourDatas)
                {
                    for (int segmentIndex = 0; segmentIndex < contourData.Contour.Count; ++segmentIndex)
                    {
                        var segment = contourData.Contour[segmentIndex];
                        var normalSegment = contourData.ContourNormals[segmentIndex];
                        var uvBottomSegment = contourData.ContourUVBottom[segmentIndex];

                        for (int i = 0; i < segment.Count; ++i)
                        {
                            var v = segment[i];
                            Vec2D vr = Vec2DOps.Rotated(v, theta);
                            Vec3D vertex = new Vec3D(vr.X, vr.Y, z);
                            vertices.Add(vertex);
                            uv.Add(new Vec2D(uvBottomSegment[i].X, t));
                            var n = normalSegment[i];
                            Vec2D nr = Vec2DOps.Rotated(n, theta);
                            normals.Add(new Vec3D(nr.X, nr.Y, 0.0));
                            Int3 intPos = converter.Convert(vertex);
                            precisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
                        }
                    }
                }
            }

            int numSideContours = CountContourSegments(contours);

            GenerateEndCapForLinearExtrusion(precisePositions, polygonIndices, baseVertexIndex, topOffsetIndex,
                triangles, triangleGroups, numSideContours, baseGroupIndex);

            for (int k = 0; k < segmentCount; k++)
            {
                int bottomOff = contourIndexOffset + k * verticesPerSideLayer;
                int topOff = contourIndexOffset + (k + 1) * verticesPerSideLayer;
                GenerateSideWallsBetweenLayers(contourDatas, bottomOff, topOff, triangles, triangleGroups, true, baseGroupIndex);
            }

            if (SignedVolumeFrom(vertices, triangles, triangleBegin) < 0)
            {
                FlipTriangleWindings(triangles, triangleBegin);
                for (int i = baseVertexIndex; i < normals.Count; i++)
                    normals[i] = -normals[i];
            }

            return numSideContours + 2;
        }

        static double SignedVolumeFrom(List<Vec3D> vertices, List<Tri> triangles, int start)
        {
            double vol6 = 0;
            for (int i = start; i < triangles.Count; i++)
            {
                Tri t = triangles[i];
                vol6 += Vec3DOps.Dot(vertices[t.A], Vec3DOps.Cross(vertices[t.B], vertices[t.C]));
            }
            return vol6 / 6.0;
        }

        static void FlipTriangleWindings(List<Tri> triangles, int start)
        {
            if (triangles == null)
                return;
            for (int i = start; i < triangles.Count; i++)
            {
                Tri t = triangles[i];
                triangles[i] = new Tri(t.A, t.C, t.B);
            }
        }

        private static void MinMax(List<ContourData> contourDatas, out Vec2D min, out Vec2D max)
        {
            min = new Vec2D(double.MaxValue, double.MaxValue);
            max = new Vec2D(double.MinValue, double.MinValue);
            foreach (var c in contourDatas)
            {
                c.ContourDuplicateFree.MinMax(out var mi, out var ma);
                if (mi.X < min.X) min.X = mi.X;
                if (mi.Y < min.Y) min.Y = mi.Y;
                if (ma.X > max.X) max.X = ma.X;
                if (ma.Y > max.Y) max.Y = ma.Y;
            }
        }

        /// <summary>
        /// Extracts X,Y components from Rat3Hybrid positions to create Rat2Hybrid polygons for 2D triangulation.
        /// This ensures the 2D triangulation uses the exact same precise coordinates as the 3D mesh.
        /// </summary>
        private static List<List<Rat2Hybrid>> ExtractRat2HybridFromPrecisePositions(
            List<Rat3Hybrid> precisePositions, List<List<int>> polygonIndices)
        {
            if (polygonIndices == null || polygonIndices.Count == 0)
                return new List<List<Rat2Hybrid>>();
            
            var result = new List<List<Rat2Hybrid>>(polygonIndices.Count);
            
            foreach (var polygon in polygonIndices)
            {
                var convertedPolygon = new List<Rat2Hybrid>(polygon.Count);
                foreach (var vertexIndex in polygon)
                {
                    // Extract X,Y components from the precise 3D position
                    var precisePoint = precisePositions[vertexIndex];
                    convertedPolygon.Add(new Rat2Hybrid(precisePoint.X, precisePoint.Y));
                }
                result.Add(convertedPolygon);
            }
            
            return result;
        }

        /// <summary>
        /// Generates end caps for linear extrusion (both bottom and top caps)
        /// This method extracts X,Y from 3D positions, so it only works when the end caps are in the XY plane.
        /// </summary>
        private static void GenerateEndCapForLinearExtrusion(
            List<Rat3Hybrid> precisePositions, List<List<int>> polygonIndices,
            int baseVertexIndex, int topOffsetIndex, List<Tri> triangles,
            List<int> triangleGroups, int numSideContours, int baseGroupIndex = 0)
        {
            // Extract X,Y components from precise 3D positions for 2D triangulation
            var precisePolygons = ExtractRat2HybridFromPrecisePositions(precisePositions, polygonIndices);

            // Generate bottom cap (flipped winding)
            GenerateEndCap(precisePolygons, 0, baseVertexIndex,
                triangles, triangleGroups, numSideContours, false, baseGroupIndex);

            // Generate top cap (normal winding)  
            GenerateEndCap(precisePolygons, 0, topOffsetIndex,
                triangles, triangleGroups, numSideContours + 1, true, baseGroupIndex);
        }

        /// <summary>
        /// Generates end caps for curve extrusion using principal component projection.
        /// The 3D rational positions are projected to 2D by selecting the axis pair 
        /// that gives the largest projected area (avoiding the dominant normal axis).
        /// </summary>
        private static List<List<int>> BuildCapPolygonIndices(List<ContourData> contourDatas, int startVertexIndex)
        {
            var ringCounts = new int[contourDatas.Count];
            for (int i = 0; i < contourDatas.Count; i++)
                ringCounts[i] = contourDatas[i].ContourDuplicateFree.Count;
            return MeshConstructionHelpers.BuildConsecutiveRingIndices(ringCounts, startVertexIndex);
        }

        private static void GenerateEndCapsForCurveExtrusion(
            List<ContourData> contourDatas, 
            List<Rat3Hybrid> precisePositions,
            int bottomCapVertexIndex, 
            int topCapVertexIndex,
            List<Tri> triangles, 
            List<int> triangleGroups, 
            int numSideContours, 
            int baseGroupIndex = 0)
        {
            var bottomPolygonIndices = BuildCapPolygonIndices(contourDatas, bottomCapVertexIndex);
            MeshConstructionHelpers.TriangulateAndEmitCap(
                precisePositions, bottomPolygonIndices, bottomCapVertexIndex, false,
                baseGroupIndex + numSideContours, triangles, triangleGroups);

            var topPolygonIndices = BuildCapPolygonIndices(contourDatas, topCapVertexIndex);
            MeshConstructionHelpers.TriangulateAndEmitCap(
                precisePositions, topPolygonIndices, topCapVertexIndex, true,
                baseGroupIndex + numSideContours + 1, triangles, triangleGroups);
        }


        private struct ContourData
        {
            public List<Vec2D> ContourDuplicateFree;
            public List<List<Vec2D>> Contour;
            public List<List<Vec2D>> ContourNormals;
            public List<List<Vec2D>> ContourUVBottom;
            public List<List<Vec2D>> ContourUVTop;
            public List<List<int>> ContourGroups;
            public int TotalVertexCount;
        }

        private static List<ContourData> PrepareContourData(List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals, int baseGroupIndex)
        {
            List<ContourData> result = new List<ContourData>(contours.Count);
            int currentGroupIndex = baseGroupIndex;
            
            for (int i = 0; i < contours.Count; i++)
            {
                result.Add(PrepareContourData(contours[i], contourNormals[i], currentGroupIndex));
                // Advance group index by the number of curves in this loop
                currentGroupIndex += contours[i].Count;
            }

            return result;
        }

        /// <summary>
        /// Prepares unified contour data from multiple contour segments
        /// </summary>
        private static ContourData PrepareContourData(List<List<Vec2D>> contours, List<List<Vec2D>> contourNormals, int baseGroupIndex)
        {
            var data = new ContourData
            {
                ContourDuplicateFree = new List<Vec2D>(),
                Contour = new List<List<Vec2D>>(),
                ContourNormals = new List<List<Vec2D>>(),
                ContourUVBottom = new List<List<Vec2D>>(),
                ContourUVTop = new List<List<Vec2D>>(),
                ContourGroups = new List<List<int>>()
            };

            int totalVertexCount = 0;
            for (int i = 0; i < contours.Count; ++i)
            {
                var c = contours[i];
                
                // Build ContourDuplicateFree (flattened, no duplicates)
                data.ContourDuplicateFree.AddRange(c);
                data.ContourDuplicateFree.RemoveAt(data.ContourDuplicateFree.Count - 1);
                
                // Build Contour as list of lists
                var contourSegment = new List<Vec2D>(c);
                contourSegment[contourSegment.Count - 1] = contours[(i + 1) % contours.Count][0];
                data.Contour.Add(contourSegment);
                
                // Build ContourNormals as list of lists
                data.ContourNormals.Add(new List<Vec2D>(contourNormals[i]));
                
                // Build UV and Groups as list of lists
                var uvBottomSegment = new List<Vec2D>();
                var uvTopSegment = new List<Vec2D>();
                var groupsSegment = new List<int>();
                
                LineStrip2D strip = new LineStrip2D(c);
                for (int j = 0; j < c.Count; ++j)
                {
                    groupsSegment.Add(baseGroupIndex + i);

                    if (j < c.Count - 1)
                    {
                        double u = strip.GetDistanceFromBuffer(j) / strip.TotalLength;
                        uvBottomSegment.Add(new Vec2D(u, 0));
                        uvTopSegment.Add(new Vec2D(u, 1));
                    }
                    else
                    {
                        uvBottomSegment.Add(new Vec2D(1, 0));
                        uvTopSegment.Add(new Vec2D(1, 1));
                    }
                }
                
                data.ContourUVBottom.Add(uvBottomSegment);
                data.ContourUVTop.Add(uvTopSegment);
                data.ContourGroups.Add(groupsSegment);
                
                totalVertexCount += c.Count;
            }

            data.TotalVertexCount = totalVertexCount;
            return data;
        }

        /// <summary>
        /// Generates a cross-section at a specific curve vertex
        /// </summary>
        private static (List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uvs) GenerateCrossSectionAtCurveVertex(
            ContourData contourData, CoordinateSystem curveVertex, double normalizedArcLength,
            double arcLength, double twistRatePerExtrudeDistance)
        {
            double twistAngle = arcLength * twistRatePerExtrudeDistance;
            CoordinateSystem frame = Math.Abs(twistAngle) < 1e-20
                ? curveVertex
                : CurveFrameWithTwistAboutZ(curveVertex, twistAngle);

            var vertices = new List<Vec3D>();
            var normals = new List<Vec3D>();
            var uvs = new List<Vec2D>();

            for (int segmentIndex = 0; segmentIndex < contourData.Contour.Count; segmentIndex++)
            {
                var segment = contourData.Contour[segmentIndex];
                var normalSegment = contourData.ContourNormals[segmentIndex];
                var uvBottomSegment = contourData.ContourUVBottom[segmentIndex];
                
                for (int i = 0; i < segment.Count; i++)
                {
                    var contourPoint = segment[i];
                    var contourNormal = normalSegment[i];

                    // Transform 2D contour point to 3D using the curve vertex frame
                    Vec3D worldPos = frame.PointTo3D(contourPoint);
                    vertices.Add(worldPos);

                    // Transform 2D normal to 3D using the curve vertex frame
                    Vec3D worldNormal = frame.DirectionTo3D(contourNormal);
                    normals.Add(worldNormal);

                    // UV coordinates: u from contour arc length, v from curve arc length
                    double u = uvBottomSegment[i].X;
                    uvs.Add(new Vec2D(u, normalizedArcLength));
                }
            }

            return (vertices, normals, uvs);
        }

        static bool OuterRingGoesForward(List<Tri> tris, List<List<Rat2Hybrid>> polygons)
        {
            if (tris == null || polygons == null || polygons.Count == 0 || polygons[0] == null)
                return true;
            int n = polygons[0].Count;
            if (n < 3)
                return true;
            int forward = 0;
            int backward = 0;
            for (int i = 0; i < n; i++)
            {
                int a = i;
                int b = (i + 1) % n;
                for (int t = 0; t < tris.Count; t++)
                {
                    if (HasDirectedEdge(tris[t], a, b))
                        forward++;
                    else if (HasDirectedEdge(tris[t], b, a))
                        backward++;
                }
            }
            return forward >= backward;
        }

        static bool HasDirectedEdge(Tri tri, int a, int b)
        {
            return (tri.A == a && tri.B == b) ||
                   (tri.B == a && tri.C == b) ||
                   (tri.C == a && tri.A == b);
        }

        /// <summary>
        /// Generates triangulated end cap (top or bottom face)
        /// </summary>
        private static void GenerateEndCap(List<List<Rat2Hybrid>> borderAndHolePolygons, int sectionIndex,
            int baseVertexIndex, List<Tri> triangles, List<int> triangleGroups,
            int groupId, bool flipWinding, int baseGroupIndex = 0)
        {
            // Explicitly call the List<List<Rat2Hybrid>> overload
            List<Tri> meshTriangles = Triangulator.TriangulatePolygon(borderAndHolePolygons);
            bool ringForward = OuterRingGoesForward(meshTriangles, borderAndHolePolygons);
            // Top wants the outer ring traversed forward (same as the side-wall contour);
            // bottom wants the opposite. Ear-clip may emit either winding.
            bool emitAsTriangulator = ringForward == flipWinding;

            foreach (var tri in meshTriangles)
            {
                Tri newTri;
                if (emitAsTriangulator)
                {
                    newTri = new Tri(
                        tri.A + baseVertexIndex,
                        tri.B + baseVertexIndex,
                        tri.C + baseVertexIndex);
                }
                else
                {
                    newTri = new Tri(
                        tri.A + baseVertexIndex,
                        tri.C + baseVertexIndex,
                        tri.B + baseVertexIndex);
                }
                
                // Skip degenerate triangles with duplicate indices
                if (!MeshConstructionHelpers.IsDegenerateTriangle(newTri))
                {
                    triangles.Add(newTri);
                    triangleGroups.Add(baseGroupIndex + groupId);
                }
            }
        }

        /// <summary>
        /// Unified method for generating side walls between two layers of vertices
        /// </summary>
        private static void GenerateSideWallsBetweenLayers(List<ContourData> contourDatas,
            int bottomLayerOffset, int topLayerOffset, List<Tri> triangles, List<int> triangleGroups,
            bool checkDuplicates, int baseGroupIndex = 0)
        {
            int offset = 0;
            foreach (var contourData in contourDatas)
            {
                for (int segmentIndex = 0; segmentIndex < contourData.Contour.Count; segmentIndex++)
                {
                    var segment = contourData.Contour[segmentIndex];
                    var groupSegment = contourData.ContourGroups[segmentIndex];
                    
                    for (int j = 1; j < segment.Count; j++)
                    {
                        int i = (j - 1) % segment.Count;

                        int i0 = bottomLayerOffset + i + offset;
                        int i1 = bottomLayerOffset + j + offset;
                        int i2 = topLayerOffset + i + offset;
                        int i3 = topLayerOffset + j + offset;

                        int group1 = groupSegment[i];
                        int group2 = groupSegment[j];

                        // Create two triangles for the quad
                        var tri1 = new Tri(i0, i1, i2);
                        var tri2 = new Tri(i1, i3, i2);

                        // Check for duplicate points if requested
                        if (checkDuplicates && (MeshConstructionHelpers.IsDegenerateTriangle(tri1) || MeshConstructionHelpers.IsDegenerateTriangle(tri2)))
                            continue;

                        triangles.Add(tri1);
                        triangleGroups.Add(group1);

                        triangles.Add(tri2);
                        triangleGroups.Add(group1);
                    }
                    offset += segment.Count;
                }
            }
        }

        private static void PopulateExtrudeNaming(MeshNaming naming, List<List<List<Vec2D>>> orientedContours, int baseGroupIndex)
        {
            if (naming == null) return;
            naming.TriangleGroupToName = new Dictionary<int, string>();

            int numSideContours = CountContourSegments(orientedContours);
            int groupIndex = 0;
            for (int loopIndex = 0; loopIndex < orientedContours.Count; loopIndex++)
            {
                var names = naming.ContourNames[loopIndex];
                for (int curveIndex = 0; curveIndex < orientedContours[loopIndex].Count; curveIndex++)
                {
                    var n = names[curveIndex];
                    EntityNaming.ValidateContourSegmentName(n);
                    naming.TriangleGroupToName.Add(groupIndex + baseGroupIndex, EntityNaming.ExtrudeSide(naming.OperationName, n));
                    groupIndex++;
                }
            }
            naming.TriangleGroupToName.Add(baseGroupIndex + numSideContours, EntityNaming.ExtrudeBottom(naming.OperationName));
            naming.TriangleGroupToName.Add(baseGroupIndex + numSideContours + 1, EntityNaming.ExtrudeTop(naming.OperationName));
        }

        private static int CountContourSegments(List<List<List<Vec2D>>> contours)
        {
            int count = 0;
            foreach (var c in contours)
                count += c.Count;
            return count;
        }

        internal static bool IsClosedLoop(Vec3D firstPos, Vec3D lastPos)
        {
            return (firstPos - lastPos).LengthSquared() < 1e-12;
        }

    }
}
