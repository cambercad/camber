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

        public int Id { get; private set; }
        public GraphEdge SourceEdge { get; set; }
        public int SurfaceIndexA { get; set; }
        public int SurfaceIndexB { get; set; }
        public double BlendRadius { get; set; }
        
        // The three defining curves of the cylindrical blend
        public List<Rat3Hybrid> CenterCurve { get; set; }
        public List<Vec3D> CenterCurveVec3 { get; set; }
        public List<Rat3Hybrid> ProjectedBoundaryCurveA { get; set; } // On surface A (for visualization/generation)
        public List<Vec3D> ProjectedBoundaryCurveAVec3D { get; set; }
        public List<Rat3Hybrid> ProjectedBoundaryCurveB { get; set; } // On surface B (for visualization/generation)
        public List<Vec3D> ProjectedBoundaryCurveBVec3D { get; set; }

        //public List<Rat3Hybrid> FinalBoundaryCurveA;
        //public List<Rat3Hybrid> FinalBoundaryCurveB;

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

        public BlendEdge(GraphEdge sourceEdge, int surfaceIndexA, int surfaceIndexB, double blendRadius)
        {
            // Validation
            if (sourceEdge == null)
                throw new ArgumentNullException(nameof(sourceEdge));
            if (blendRadius <= 0)
                throw new ArgumentException("Blend radius must be positive", nameof(blendRadius));
            if (surfaceIndexA == surfaceIndexB)
                throw new ArgumentException("Surface indices must be different");
                
            Id = GetHashCode(); // Use hash code as a simple unique identifier
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

            (List<LineSegmentOnTriangleEx> intersectA, List<LineSegmentOnTriangleEx> intersectB) =
                Intersector.IntersectSurfaces(enlargedAndOffsetSurfaceA, enlargedAndOffsetSurfaceB, cc);

            if (intersectA == null || intersectA.Count == 0)
                return false;

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

        /// <summary>
        /// Build a fillet strip surface from precomputed spine and boundary curves.
        /// </summary>
        public bool BuildFilletStripSurface(CoordinateConverter cc, double maxDiscretizationDeviation)
        {
            this.cc = cc;

            double maxArcAngle = 0;
            for (int i = 0; i < CenterCurve.Count; ++i)
            {
                var startD = ProjectedBoundaryCurveAVec3D[i];
                var endD = ProjectedBoundaryCurveBVec3D[i];
                var centerD = CenterCurveVec3[i];

                Vec3D dirStart = startD - centerD;
                Vec3D dirEnd = endD - centerD;
                Vec3D axis = Vec3DOps.Cross(dirStart, dirEnd);
                double angle = Vec3DOps.Angle(dirStart, dirEnd, axis);
                maxArcAngle = Math.Max(angle, maxArcAngle);
            }
            // Calculate number of arc points based on deviation tolerance
            int numArcPoints = Math.Max(4, CalculateArcSegments(maxArcAngle, BlendRadius, maxDiscretizationDeviation));


            List<Rat3Hybrid> concatenatedArcPointsPrecise = new List<Rat3Hybrid>();
            List<Vec3D> concatenatedArcPoints = new List<Vec3D>();
            List<Vec3D> normals = new List<Vec3D>();
            List<Vec2D> uv = new List<Vec2D>();
            LineStrip3D centerCurveStrip = new LineStrip3D(CenterCurveVec3); //Use for arc length when computing uv
            for (int i = 0; i < CenterCurve.Count; ++i)
            {
                var start = ProjectedBoundaryCurveA[i];
                var end = ProjectedBoundaryCurveB[i];

                var startD = ProjectedBoundaryCurveAVec3D[i];
                var endD = ProjectedBoundaryCurveBVec3D[i];
                var centerD = CenterCurveVec3[i];

                // Generate the shorter arc from start to end around center
                Vec3D dirStart = startD - centerD;
                Vec3D dirEnd = endD - centerD;
                Vec3D axis = Vec3DOps.Cross(dirStart, dirEnd);
                double arcAngle = Vec3DOps.Angle(dirStart, dirEnd, axis);
                
                // Ensure we use the shorter arc (should always be < 180 degrees for edge blending)
                if (arcAngle > Math.PI)
                {
                    arcAngle = 2.0 * Math.PI - arcAngle;
                    axis = -axis;
                }
                
                double radius = dirStart.Length();
                Vec3D dirStartNorm = dirStart.Normalized();
                Vec3D right = Vec3DOps.Cross(axis.Normalized(), dirStartNorm).Normalized();
                
                List<Vec3D> arc = new List<Vec3D>();
                List<Vec3D> arcNormals = new List<Vec3D>();
                List<Vec2D> arcUv = new List<Vec2D>();
                
                double uCoord = centerCurveStrip.GetDistanceFromBuffer(i);
                double uMax = centerCurveStrip.TotalLength;

                // Skip the first and last band
                //for (int j = 1; j < numArcPoints-1; ++j)
                for (int j = 0; j < numArcPoints; ++j)
                {
                    double t = j / (double)(numArcPoints - 1);
                    double a = t * arcAngle;
                    
                    Vec3D point = centerD + radius * (Math.Cos(a) * dirStartNorm + Math.Sin(a) * right);
                    Vec3D normal = (point - centerD).Normalized();
                    
                    // For convex edges, normals point inward (toward center)
                    if (BlendType == EdgeBlendType.Concave)
                        normal = -normal;
                    
                    arc.Add(point);
                    arcNormals.Add(normal);
                    arcUv.Add(new Vec2D(uCoord / uMax, t));
                }

                List<Rat3Hybrid> arcPrecise = new List<Rat3Hybrid>(arc.Count);
                var intPoints = cc.Convert(arc);
                for (int j = 0; j < intPoints.Count; ++j)
                {
                    var p = intPoints[j];
                    arcPrecise.Add(new Rat3Hybrid(p.X, p.Y, p.Z));
                }

                arcPrecise[0] = start;
                arcPrecise[arcPrecise.Count - 1] = end;

                concatenatedArcPoints.AddRange(arc);
                normals.AddRange(arcNormals);
                uv.AddRange(arcUv);
                concatenatedArcPointsPrecise.AddRange(arcPrecise);
            }


            // Build triangles and ensure correct winding
            // -2 because we skip the first and last band
            List<Tri> triangles = BuildRectangularTriangleBuffer(CenterCurve.Count, numArcPoints/*-2*/, out List<int> startRowIndices, out List<int> endRowIndices);

            if(!MeshAnalysis.AreTrianglesConsistentlyOriented(triangles))
                throw new Exception();

            // TriangulateBand also inserts the vertices coming from the boundary curves
            //int triCount = triangles.Count;
            //TriangulateBand(triangles, startRowIndices, concatenatedArcPointsPrecise, concatenatedArcPoints, 
            //    normals, uv, FinalBoundaryCurveA, originalSurfaceA, centerCurveStrip, 0.0);
            //if (!MeshAnalysis.AreTrianglesConsistentlyOriented(triangles))
            //    ReverseTriOri(triangles, triCount, triangles.Count);
            //triCount = triangles.Count;
            //TriangulateBand(triangles, endRowIndices, concatenatedArcPointsPrecise, concatenatedArcPoints, 
            //    normals, uv, FinalBoundaryCurveB, originalSurfaceB, centerCurveStrip, 1.0);
            //if (!MeshAnalysis.AreTrianglesConsistentlyOriented(triangles))
            //    ReverseTriOri(triangles, triCount, triangles.Count);

            EnsureCorrectTriangleWinding(triangles, concatenatedArcPoints, CenterCurveVec3, BlendType);

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

                Vec3D spineDir = i < numStrips - 1
                    ? CenterCurveVec3[i + 1] - CenterCurveVec3[i]
                    : CenterCurveVec3[i] - CenterCurveVec3[i - 1];
                Vec3D chamferDir = endD - startD;
                Vec3D normal = Vec3DOps.Cross(chamferDir, spineDir);
                if (normal.LengthSquared() < 1e-20)
                    normal = Vec3DOps.Cross(chamferDir, CenterCurveVec3[i] - CenterCurveVec3[Math.Max(0, i - 1)]);
                normal = normal.Normalized();
                if (BlendType == EdgeBlendType.Concave)
                    normal = -normal;

                points.Add(startD);
                points.Add(endD);
                normals.Add(normal);
                normals.Add(normal);
                uv.Add(new Vec2D(u, 0));
                uv.Add(new Vec2D(u, 1));
                precise.Add(ProjectedBoundaryCurveA[i]);
                precise.Add(ProjectedBoundaryCurveB[i]);
            }

            List<Tri> triangles = BuildRectangularTriangleBuffer(numStrips, numPointsPerStrip, out _, out _);
            if (!MeshAnalysis.AreTrianglesConsistentlyOriented(triangles))
                throw new Exception();

            EnsureCorrectTriangleWinding(triangles, points, CenterCurveVec3, BlendType);
            RawSurface = new UVSurface(points, normals, uv, triangles, precise);
            return true;
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
        private UVSurface RebuildSurfaceFromTrimmedMesh(MeshNormalUV trimmedMesh)
        {
            // Find all connected clusters of triangles
            List<List<int>> trianglePatches = FindClusters(trimmedMesh);

            // Keep only the cluster that's connected to the boundary curves
            int keepClusterIndex = SelectClusterConnectedToBoundary(trianglePatches, trimmedMesh);
            
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
            
            // Use Decompose to properly handle UV seams and rebuild the surface
            filteredMesh.Decompose(out List<Vec3D> positions, out List<Vec3D> normals, 
                out List<Vec2D> uvs, out List<Tri> triangles, out List<int> perTriangleGroup);
            
            // Convert positions back to precise coordinates
            List<Rat3Hybrid> precisePositions = new List<Rat3Hybrid>(positions.Count);
            for (int i = 0; i < positions.Count; i++)
            {
                Vec3D pos = positions[i];
                // Find the corresponding precise position from the original mesh
                int originalIndex = -1;
                for (int j = 0; j < trimmedMesh.Positions.Count; j++)
                {
                    if (trimmedMesh.Positions[j] == pos)
                    {
                        originalIndex = j;
                        break;
                    }
                }
                
                if (originalIndex >= 0)
                {
                    precisePositions.Add(trimmedMesh.PrecisionPositions[originalIndex]);
                }
                else
                {
                    // Fallback: convert from Vec3D
                    var intPos = cc.Convert(pos);
                    precisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
                }
            }
            
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

        public bool TrimByOffsetSurface(UVSurface trimSurface)
        {
            var BlendSurface = this.RawSurface;

            // Convert blend surface to mesh
            MeshNormalUV blendSurfaceMesh = ToMesh(BlendSurface, cc, groupId: 0);

            MeshNormalUV trimMesh = ToMesh(trimSurface, cc, 0);
            
            List<List<IntersectionSegmentEx>> intersectionStrips = new List<List<IntersectionSegmentEx>>();
            MeshNormalUV trimmed = MeshNormalUV.BooleanOperation(blendSurfaceMesh, trimMesh, 
                BooleanOp.AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection, cc, intersectionStrips);

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

            // Check if start corner needs trimming
            if (openCornerTrimSurfaces.TryGetValue(StartCornerId, out UVSurface startTrimSurface))
            {
                currentSurface = TrimSurfaceBySurface(currentSurface, startTrimSurface, openCornerTrimSurfacesExtensionOnly[StartCornerId], out EdgeStartGapFillSurface);
            }

            // Check if end corner needs trimming
            if (openCornerTrimSurfaces.TryGetValue(EndCornerId, out UVSurface endTrimSurface))
            {
                currentSurface = TrimSurfaceBySurface(currentSurface, endTrimSurface, openCornerTrimSurfacesExtensionOnly[EndCornerId], out EdgeEndGapFillSurface);
            }

            this.RawSurface = currentSurface;
            this.BlendSurface = currentSurface;
            
            return true;
        }

        /// <summary>
        /// Trim a surface by another surface, keeping the part against the normal direction of the trim surface.
        /// </summary>
        /// <param name="surfaceToTrim">The surface to be trimmed</param>
        /// <param name="trimSurface">The surface to trim with</param>
        /// <returns>The trimmed surface</returns>
        private UVSurface TrimSurfaceBySurface(UVSurface surfaceToTrim, UVSurface trimSurface, 
            UVSurface trimSurfaceExtensionOnly, out UVSurface finalGapFillSurface)
        {
            // Convert surfaces to meshes
            MeshNormalUV surfaceToTrimMesh = ToMesh(surfaceToTrim, cc, groupId: 0);
            MeshNormalUV trimMesh = ToMesh(trimSurface, cc, groupId: 1);
            
            // Perform boolean operation
            // RemoveInTriNormalDirection keeps the part against the normal direction
            List<List<IntersectionSegmentEx>> intersectionStrips = new List<List<IntersectionSegmentEx>>();
            MeshNormalUV trimmed = MeshNormalUV.BooleanOperation(
                surfaceToTrimMesh, 
                trimMesh, 
                BooleanOp.AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection, 
                cc, 
                intersectionStrips);

            if (intersectionStrips.Count != 1)
                throw new Exception();

            //List<Rat3Hybrid> splitCurve = ExtractCurveFromSegments(intersectionStrips[0]);
            var sourceCurve = intersectionStrips[0];
            int idOffset = trimSurface.Triangles.Count - trimSurfaceExtensionOnly.Triangles.Count;

            List<LineSegmentOnTriangle> splitCurve = new List<LineSegmentOnTriangle>(sourceCurve.Count);
            List<Vec3D> splitCurveNormals = new List<Vec3D>(sourceCurve.Count);
            List<Vec3D> splitCurveNormalAnchors = new List<Vec3D>(sourceCurve.Count);
            for (int i=0;i< sourceCurve.Count;++i)
            {
                var s = sourceCurve[i];

                int correctedId = s.TriIdB - idOffset;
                if (correctedId < 0)
                    throw new Exception();

                splitCurve.Add(new LineSegmentOnTriangle(s.StartPoint, s.EndPoint, correctedId));
                var p = 0.5 * (trimmed.Positions[s.Start] + trimmed.Positions[s.End]);
                splitCurveNormalAnchors.Add(p);
                splitCurveNormals.Add(EvaluateNormal(s.TriIdA, p, surfaceToTrimMesh));
            }

            Surface surf = new Surface(trimSurfaceExtensionOnly.Triangles, trimSurfaceExtensionOnly.PointsPrecise);
            List<Surface> splitSurfaces = Splitter.SplitUsingLineStrip(surf, new List<List<LineSegmentOnTriangle>>() { splitCurve });

            Surface finalGapFillSurfaceRaw = SelectSurfaceAgainstNormalDirection(splitSurfaces, splitCurve, splitCurveNormals, splitCurveNormalAnchors);
            finalGapFillSurface = ComputeUvAndNormal(finalGapFillSurfaceRaw);

            // Rebuild surface with the cluster connected to boundary curves
            UVSurface result = RebuildSurfaceFromTrimmedMesh(trimmed);

            // Store intersection arcs for corner generation
            if (intersectionStrips.Count > 1)
                throw new Exception("Expected at most one intersection strip per trim operation");

            if (intersectionStrips.Count == 1)
            {
                List<Rat3Hybrid> arc = ExtractCurveFromSegments(intersectionStrips[0]);
                TrimArcs.Add(arc);

#if DEBUG
                VerifyArc(arc);
#endif
            }

            return result;
        }

        private void VerifyArc(List<Rat3Hybrid> arc)
        {
            // Both, start and end point must lie exactly on either BoundaryCurveA or BoundaryCurveB
            var start = arc[0];
            var end = arc[arc.Count - 1];

            if (!CheckPointOnLineStrip(start, ProjectedBoundaryCurveA) && !CheckPointOnLineStrip(start, ProjectedBoundaryCurveB))
                throw new Exception();
            if (!CheckPointOnLineStrip(end, ProjectedBoundaryCurveA) && !CheckPointOnLineStrip(end, ProjectedBoundaryCurveB))
                throw new Exception();
        }

        private bool CheckPointOnLineStrip(Rat3Hybrid point, List<Rat3Hybrid> lineStrip)
        {
            if (lineStrip == null || lineStrip.Count < 2)
                return false;

            // Check each segment in the line strip
            for (int i = 0; i < lineStrip.Count - 1; i++)
            {
                var segmentStart = lineStrip[i];
                var segmentEnd = lineStrip[i + 1];

                // Check if point equals one of the endpoints
                if (point == segmentStart || point == segmentEnd)
                    return true;

                // Check if point is collinear with the segment
                // This is done by checking if the cross product of (point - start) and (end - start) is zero
                var dir = segmentEnd - segmentStart;
                var toPoint = point - segmentStart;
                var cross = Rat3Hybrid.Cross(dir, toPoint);

                // If cross product is zero, the point is collinear with the segment
                if (cross.X == BigRationalHybrid.Zero && 
                    cross.Y == BigRationalHybrid.Zero && 
                    cross.Z == BigRationalHybrid.Zero)
                {
                    // Now check if the point lies between start and end
                    // We can use the dot product to check if the point is within the segment bounds
                    var dotProduct = Rat3Hybrid.Dot(toPoint, dir);
                    var segmentLengthSquared = Rat3Hybrid.Dot(dir, dir);

                    // Point is on the segment if 0 <= dotProduct <= segmentLengthSquared
                    if (dotProduct >= BigRationalHybrid.Zero && dotProduct <= segmentLengthSquared)
                        return true;
                }
            }

            return false;
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
                    throw new Exception();
                result.Add(seg.EndPoint);
            }
            return result;
        }

        public static MeshNormalUV ApplyBlendSurfaceToVolume(MeshNormalUV blendSurface, MeshNormalUV volume, CoordinateConverter cc, EdgeBlendType blendType)
        {
            // TODO: Need a correct triangle group index
            // Convert blend surface to mesh
            //MeshNormalUV blendSurfaceMesh = ToMesh(blendSurface, cc, groupId: 0);
            
            // For concave edges: remove triangles in normal direction (removes material inside the blend)
            // For convex edges: keep triangles in normal direction (removes material outside the blend)
            BooleanOp operation = blendType == EdgeBlendType.Convex 
                ? BooleanOp.AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection
                : BooleanOp.AAsVolumeBAsTrimSurfaceKeepInTriNormalDirection;
            
            // Apply blend surface to volume
            MeshNormalUV trimmed = MeshNormalUV.BooleanOperation(volume, blendSurface, operation, cc);
            trimmed.RunSanityChecks();
            return trimmed;
        }


        private int SelectClusterConnectedToBoundary(List<List<int>> clusters, MeshNormalUV mesh)
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
                
                // Count how many of these vertices are on the boundary curves
                int boundaryCount = 0;
                foreach (int vertexIndex in clusterVertices)
                {
                    Rat3Hybrid precisePos = mesh.PrecisionPositions[vertexIndex];
                    if (boundaryPoints.Contains(precisePos))
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
                throw new Exception("No cluster found connected to boundary curves");
            }
            
            if (nonZeroCount > 1)
            {
                throw new Exception($"Multiple clusters ({nonZeroCount}) are connected to boundary curves - ambiguous result");
            }
            
            return clusterWithBoundary;
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
                    throw new Exception();
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
        private static int CalculateArcSegments(double angle, double radius, double maxDeviation)
        {
            if (angle <= 0 || maxDeviation <= 0)
                return 2;
            
            double ratio = maxDeviation / radius;
            if (ratio >= 1.0)
                return 2;
            
            // For a circular arc: deviation = radius * (1 - cos(angle/(2*segments)))
            // Solving for segments: segments >= angle / (2 * acos(1 - deviation/radius))
            double halfAnglePerSegment = Math.Acos(1.0 - ratio);
            int numSegments = (int)Math.Ceiling(angle / (2.0 * halfAnglePerSegment));
            return Math.Max(2, numSegments + 1);
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

        /// <summary>
        /// Ensure triangles have correct winding order so normals point in the correct direction.
        /// For concave edges: normals point outward from the center curve.
        /// For convex edges: normals point inward toward the center curve.
        /// </summary>
        /// <param name="triangles">List of triangles to check and potentially flip</param>
        /// <param name="vertices">Vertex positions</param>
        /// <param name="centerCurve">Center curve points to determine direction</param>
        /// <param name="blendType">Type of blend (concave or convex)</param>
        private static void EnsureCorrectTriangleWinding(List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> centerCurve, EdgeBlendType blendType)
        {
            if (triangles.Count == 0 || centerCurve.Count == 0)
                return;
            
            // Pick a sample triangle (middle of the mesh for robustness)
            int sampleTriIndex = triangles.Count / 2;
            Tri sampleTri = triangles[sampleTriIndex];
            
            // Get triangle vertices
            Vec3D p0 = vertices[sampleTri.A];
            Vec3D p1 = vertices[sampleTri.B];
            Vec3D p2 = vertices[sampleTri.C];
            
            // Compute triangle normal using cross product
            Vec3D edge1 = p1 - p0;
            Vec3D edge2 = p2 - p0;
            Vec3D triangleNormal = Vec3DOps.Cross(edge1, edge2).Normalized();
            
            // Compute triangle center
            Vec3D triangleCenter = (p0 + p1 + p2) / 3.0;
            
            // Find the closest center curve point to determine the expected direction
            double minDistSq = double.MaxValue;
            Vec3D closestCenterPoint = centerCurve[0];
            for (int i = 0; i < centerCurve.Count; i++)
            {
                double distSq = (centerCurve[i] - triangleCenter).LengthSquared();
                if (distSq < minDistSq)
                {
                    minDistSq = distSq;
                    closestCenterPoint = centerCurve[i];
                }
            }
            
            // Direction from center curve to triangle
            Vec3D outwardDir = (triangleCenter - closestCenterPoint).Normalized();
            double alignment = Vec3DOps.Dot(triangleNormal, outwardDir);
            
            // For concave edges: normals should point away from center (alignment > 0)
            // For convex edges: normals should point toward center (alignment < 0)
            bool shouldFlip = blendType == EdgeBlendType.Convex ? (alignment < 0) : (alignment > 0);
            
            if (shouldFlip)
            {
                for (int i = 0; i < triangles.Count; i++)
                {
                    Tri tri = triangles[i];
                    triangles[i] = new Tri(tri.A, tri.C, tri.B); // Swap B and C to flip winding
                }
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
        private Surface SelectSurfaceAgainstNormalDirection(List<Surface> surfaces, 
            List<LineSegmentOnTriangle> splitCurve, List<Vec3D> normals, List<Vec3D> normalAnchors)
        {
            if (surfaces == null || surfaces.Count == 0)
                throw new ArgumentException("Surface list cannot be empty", nameof(surfaces));
            
            if (surfaces.Count == 1)
                return surfaces[0];

            if (splitCurve == null || splitCurve.Count == 0)
                throw new ArgumentException("Split curve cannot be empty", nameof(splitCurve));

            if (normals == null || normals.Count == 0)
                throw new ArgumentException("Normals list cannot be empty", nameof(normals));

            if (normalAnchors == null || normalAnchors.Count != normals.Count)
                throw new ArgumentException("Normal anchors must match normals count", nameof(normalAnchors));

            Surface bestSurface = null;
            int bestConsistentCount = -1;

            // Build a set of triangle IDs from the split curve
            HashSet<int> cutTriangleIds = new HashSet<int>();
            foreach (var seg in splitCurve)
            {
                cutTriangleIds.Add(seg.TriangleId);
            }

            foreach (var surface in surfaces)
            {
                List<Vec3D> surfacePoints = cc.Convert(surface.PointsPrecise);
                
                // Find which triangles in this surface correspond to the cut triangles
                // We need to check which triangles from the split curve ended up in this surface
                List<int> adjacentTriangles = new List<int>();
                Dictionary<int, int> segmentToTriangleMap = new Dictionary<int, int>();
                
                for (int segIdx = 0; segIdx < splitCurve.Count; segIdx++)
                {
                    var seg = splitCurve[segIdx];
                    
                    // Find triangles in this surface that contain both segment endpoints
                    for (int triIdx = 0; triIdx < surface.Triangles.Count; triIdx++)
                    {
                        Tri tri = surface.Triangles[triIdx];
                        Vec3D p0 = surfacePoints[tri.A];
                        Vec3D p1 = surfacePoints[tri.B];
                        Vec3D p2 = surfacePoints[tri.C];
                        
                        // Check if this triangle contains the segment (approximately)
                        Vec3D startPt = cc.Convert(seg.PointStart);
                        Vec3D endPt = cc.Convert(seg.PointEnd);
                        
                        // Check if segment midpoint is inside or near this triangle
                        Vec3D segMid = 0.5 * (startPt + endPt);
                        Vec3D bary = ComputeBarycentricCoordinates(segMid, p0, p1, p2);
                        
                        double tolerance = 1e-6;
                        if (bary.X >= -tolerance && bary.Y >= -tolerance && bary.Z >= -tolerance)
                        {
                            adjacentTriangles.Add(triIdx);
                            segmentToTriangleMap[segIdx] = triIdx;
                            break;
                        }
                    }
                }

                if (adjacentTriangles.Count == 0)
                    continue;

                // For each adjacent triangle paired with its corresponding segment's normal and anchor,
                // check if the triangle center is against the normal direction
                int againstCount = 0;  // Count of triangles against the normal
                int withCount = 0;     // Count of triangles with the normal
                
                foreach (var kvp in segmentToTriangleMap)
                {
                    int segIdx = kvp.Key;
                    int triIdx = kvp.Value;
                    
                    Tri tri = surface.Triangles[triIdx];
                    
                    // Compute triangle center
                    Vec3D triCenter = (surfacePoints[tri.A] + surfacePoints[tri.B] + surfacePoints[tri.C]) / 3.0;
                    
                    // Get the corresponding normal and anchor for this segment
                    Vec3D anchor = normalAnchors[segIdx];
                    Vec3D normal = normals[segIdx];
                    
                    // Vector from anchor to triangle center
                    Vec3D toTriCenter = triCenter - anchor;
                    
                    // Check if triangle is against the normal (negative dot product)
                    double alignment = Vec3DOps.Dot(toTriCenter, normal);
                    
                    if (alignment < 0)
                        againstCount++;
                    else
                        withCount++;
                }

                // Check consistency: all triangles should agree on the direction
                int totalChecked = againstCount + withCount;
                bool isConsistent = (againstCount == totalChecked) || (withCount == totalChecked);
                
                if (!isConsistent)
                {
                    throw new Exception($"Surface triangles are inconsistent: {againstCount} against, {withCount} with the normal direction");
                }

                // Select the surface with most triangles against the normal
                if (againstCount > bestConsistentCount)
                {
                    bestConsistentCount = againstCount;
                    bestSurface = surface;
                }
            }

            if (bestSurface == null)
                throw new Exception("No suitable surface found");

            return bestSurface;
        }



        /// <summary>
        /// Compute UV coordinates and normals for a Surface, returning a UVSurface.
        /// Uses angle-weighted normals and projects points onto a fitted plane for UV calculation.
        /// </summary>
        /// <param name="surface">The surface to process</param>
        /// <returns>A UVSurface with computed normals and UV coordinates</returns>
        private UVSurface ComputeUvAndNormal(Surface surface)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));

            // Convert precise points to Vec3D
            List<Vec3D> points = cc.Convert(surface.PointsPrecise);

            // Compute angle-weighted normals
            List<Vec3D> normals = UVSurface.ComputeAngleWeightedNormals(points, surface.Triangles);

            // Fit a plane to the points
            (Vec3D planeNormal, Vec3D planeOrigin) = PlaneFitter.FitPlane(points, surface.Triangles);

            // Create orthonormal basis for the plane
            // planeNormal is already normalized by FitPlane
            Vec3D tangentU = GetPerpendicularVector(planeNormal).Normalized();
            Vec3D tangentV = Vec3DOps.Cross(planeNormal, tangentU).Normalized();

            // Project points onto the plane and compute UV coordinates
            List<Vec2D> uvs = new List<Vec2D>(points.Count);
            
            for (int i = 0; i < points.Count; i++)
            {
                // Project point onto plane
                Vec3D toPoint = points[i] - planeOrigin;
                double u = Vec3DOps.Dot(toPoint, tangentU);
                double v = Vec3DOps.Dot(toPoint, tangentV);
                uvs.Add(new Vec2D(u, v));
            }

            return new UVSurface(points, normals, uvs, surface.Triangles, surface.PointsPrecise);
        }

        /// <summary>
        /// Get a vector perpendicular to the given vector.
        /// </summary>
        private Vec3D GetPerpendicularVector(Vec3D v)
        {
            // Choose the axis that is least aligned with v
            Vec3D axis = Math.Abs(v.X) < Math.Abs(v.Y) 
                ? (Math.Abs(v.X) < Math.Abs(v.Z) ? new Vec3D(1, 0, 0) : new Vec3D(0, 0, 1))
                : (Math.Abs(v.Y) < Math.Abs(v.Z) ? new Vec3D(0, 1, 0) : new Vec3D(0, 0, 1));

            return Vec3DOps.Cross(v, axis);
        }

        public override string ToString()
        {
            return $"BlendEdge {Id}: {SourceEdge?.Name ?? "unknown"} (radius={BlendRadius})";
        }
    }

    /// <summary>
    /// Type of edge blend (determines material addition/removal)
    /// </summary>
    public enum EdgeBlendType
    {
        Concave,    // Rounds a convex edge (removes material)
        Convex    // Rounds a concave edge (adds material)
    }
}

