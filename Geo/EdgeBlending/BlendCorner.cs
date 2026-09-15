#pragma warning disable CS8625 // Cannot convert null literal to non-nullable reference type
#pragma warning disable CS8618 // Non-nullable property must contain a non-null value when exiting constructor

using GeoCore;

namespace Geo
{
    public static class BlendCorner
    {
        public static UVSurface TessellateSphereCap(List<Vec3D> borderLoop, double maxDeviation, List<Rat3Hybrid> exactBoundary, List<int> cornerIndices, CoordinateConverter cc, EdgeBlendType blendType)
        {
            SphereFitter.FitSphere(borderLoop, out var sphereCenter, out var cornerRadius);
            
            double maxSegmentLength = MaxSegmentLengthFromMaxDeviation(cornerRadius, maxDeviation);

            double maxOpeningAngle = FindMaxAngle(borderLoop, sphereCenter);
            double maxArcLength = maxOpeningAngle * cornerRadius;

            int spiderNetResolution = (int)(maxArcLength / maxSegmentLength) + 1;

            // Compute the center point of the patch on the sphere surface
            // Use weighted centroid based on edge segment lengths
            Vec3D centroid = new Vec3D(0);
            double totalLength = 0;
            
            for (int i = 0; i < borderLoop.Count; ++i)
            {
                int nextI = (i + 1) % borderLoop.Count;
                Vec3D p0 = borderLoop[i];
                Vec3D p1 = borderLoop[nextI];
                
                // Midpoint of segment
                Vec3D midpoint = (p0 + p1) * 0.5;
                
                // Length of segment
                double segmentLength = (p1 - p0).Length();
                
                // Accumulate weighted centroid
                centroid += midpoint * segmentLength;
                totalLength += segmentLength;
            }
            
            if (totalLength > 0)
                centroid /= totalLength;
            else
                centroid = borderLoop[0]; // Fallback

            // Project centroid onto sphere surface
            Vec3D directionToCenter = (centroid - sphereCenter).Normalized();
            Vec3D spiderNetCenter = sphereCenter + directionToCenter * cornerRadius;

            List<Vec3D> finalPoints = new List<Vec3D>();
            List<Rat3Hybrid> finalPointsPrecise = new List<Rat3Hybrid>();
            List<Vec3D> finalNormals = new List<Vec3D>();
            List<Vec2D> finalUV = new List<Vec2D>();
            
            // Add center point
            finalPoints.Add(spiderNetCenter);
            var centerPrecise = cc.Convert(spiderNetCenter);
            finalPointsPrecise.Add(new Rat3Hybrid(centerPrecise.X, centerPrecise.Y, centerPrecise.Z));
            
            // For concave edges: normals point outward from sphere center
            // For convex edges: normals point inward toward sphere center
            Vec3D centerNormal = (spiderNetCenter - sphereCenter).Normalized();
            if (blendType == EdgeBlendType.Concave)
                centerNormal = -centerNormal;
            finalNormals.Add(centerNormal);
            
            finalUV.Add(new Vec2D(0.5, 0.5)); // Center of UV space
            
            List<int> prevLoop = new List<int>() { 0 };
            List<Tri> finalTriangles = new List<Tri>();
            
            for (int outwardsId = 1; outwardsId <= spiderNetResolution; ++outwardsId)
            {
                double fraction = (double)outwardsId / spiderNetResolution;

                List<Vec3D> ring = new List<Vec3D>();
                for(int ringId = 0; ringId < borderLoop.Count; ++ringId)
                {
                    var p = Slerp(spiderNetCenter - sphereCenter, borderLoop[ringId] - sphereCenter, fraction);
                    // Slerp returns a normalized direction vector, so scale it by cornerRadius
                    ring.Add(sphereCenter + p * cornerRadius);
                }

                if(outwardsId < spiderNetResolution)
                    ring = Resample(ring, maxSegmentLength, cornerIndices);
                else
                {
                    var ringShifted = new List<Vec3D>(ring.Count);
                    for(int i=0;i<ring.Count;++i)
                        ringShifted.Add(ring[(i + cornerIndices[0]) % ring.Count]);
                    ring = ringShifted;
                }

                List<int> currentLoop = new List<int>(ring.Count);
                int offset = finalPoints.Count;

                // Create a closed ring for arc length computation
                List<Vec3D> ringClosed = new List<Vec3D>(ring);
                ringClosed.Add(ring[0]); // Close the ring
                LineStrip3D ringStrip = new LineStrip3D(ringClosed);
                double totalRingLength = ringStrip.TotalLength;
                
                for (int i = 0; i < ring.Count; ++i)
                {
                    Vec3D p = ring[i];
                    finalPoints.Add(p);
                    
                    if (outwardsId == spiderNetResolution)
                        finalPointsPrecise.Add(exactBoundary[(i + cornerIndices[0]) % ring.Count]); // Outermost points must match exactly
                    else
                    {
                        var p2 = cc.Convert(p);
                        finalPointsPrecise.Add(new Rat3Hybrid(p2.X, p2.Y, p2.Z));
                    }
                    
                    // For concave edges: normals point outward from sphere center
                    // For convex edges: normals point inward toward sphere center
                    Vec3D normal = (p - sphereCenter).Normalized();
                    if (blendType == EdgeBlendType.Concave)
                        normal = -normal;
                    finalNormals.Add(normal);
                    
                    // Compute UV using arc length-based angle for better distribution
                    double arcLength = ringStrip.GetDistanceFromBuffer(i);
                    double angle = (arcLength / totalRingLength) * 2.0 * Math.PI;
                    double u = 0.5 + 0.5 * fraction * Math.Cos(angle);
                    double v = 0.5 + 0.5 * fraction * Math.Sin(angle);
                    finalUV.Add(new Vec2D(u, v));

                    currentLoop.Add(i + offset);
                }

                ConnectRings(finalPoints, prevLoop, currentLoop, finalTriangles);

                prevLoop = currentLoop;
            }

            // For convex edges, flip all triangle winding
            if (blendType == EdgeBlendType.Concave)
            {
                for (int i = 0; i < finalTriangles.Count; i++)
                {
                    Tri tri = finalTriangles[i];
                    finalTriangles[i] = new Tri(tri.A, tri.C, tri.B);
                }
            }

            return new UVSurface(finalPoints, finalNormals, finalUV, finalTriangles, finalPointsPrecise);
        }

        /// <summary>
        /// Tessellate a planar corner cap from a closed boundary loop (symmetric chamfer junction).
        /// </summary>
        public static UVSurface TessellatePlanarCap(
            List<Vec3D> borderLoop,
            List<Rat3Hybrid> exactBoundary,
            List<int> cornerIndices,
            CoordinateConverter cc,
            EdgeBlendType blendType)
        {
            if (borderLoop == null || borderLoop.Count < 3)
                throw new ArgumentException("Planar cap needs at least three boundary points.");

            (Vec3D planeNormal, Vec3D planeOrigin) = PlaneFitter.FitPlane(borderLoop, BuildTriangleList(borderLoop.Count));
            Vec3D tangentU = GetPerpendicularVector(planeNormal).Normalized();
            Vec3D tangentV = Vec3DOps.Cross(planeNormal, tangentU).Normalized();

            Vec3D centroid = new Vec3D(0);
            foreach (var p in borderLoop)
                centroid += p;
            centroid /= borderLoop.Count;

            Vec3D toCentroid = centroid - planeOrigin;
            centroid = planeOrigin + tangentU * Vec3DOps.Dot(toCentroid, tangentU) + tangentV * Vec3DOps.Dot(toCentroid, tangentV);

            List<Vec3D> finalPoints = new List<Vec3D>(borderLoop.Count + 1);
            List<Rat3Hybrid> finalPointsPrecise = new List<Rat3Hybrid>(borderLoop.Count + 1);
            List<Vec3D> finalNormals = new List<Vec3D>(borderLoop.Count + 1);
            List<Vec2D> finalUV = new List<Vec2D>(borderLoop.Count + 1);
            List<Tri> finalTriangles = new List<Tri>();

            finalPoints.Add(centroid);
            var centerPrecise = cc.Convert(centroid);
            finalPointsPrecise.Add(new Rat3Hybrid(centerPrecise.X, centerPrecise.Y, centerPrecise.Z));

            Vec3D centerNormal = planeNormal.Normalized();
            if (blendType == EdgeBlendType.Concave)
                centerNormal = -centerNormal;
            finalNormals.Add(centerNormal);
            finalUV.Add(new Vec2D(0.5, 0.5));

            int boundaryStartIndex = 1;
            for (int i = 0; i < borderLoop.Count; i++)
            {
                finalPoints.Add(borderLoop[i]);
                finalPointsPrecise.Add(exactBoundary[i]);
                finalNormals.Add(centerNormal);
                Vec3D rel = borderLoop[i] - planeOrigin;
                double u = Vec3DOps.Dot(rel, tangentU);
                double v = Vec3DOps.Dot(rel, tangentV);
                finalUV.Add(new Vec2D(u, v));
            }

            List<int> boundaryLoop = new List<int>(borderLoop.Count);
            for (int i = 0; i < borderLoop.Count; i++)
                boundaryLoop.Add(boundaryStartIndex + i);

            ConnectRings(finalPoints, new List<int> { 0 }, boundaryLoop, finalTriangles);

            if (blendType == EdgeBlendType.Concave)
            {
                for (int i = 0; i < finalTriangles.Count; i++)
                {
                    Tri tri = finalTriangles[i];
                    finalTriangles[i] = new Tri(tri.A, tri.C, tri.B);
                }
            }

            return new UVSurface(finalPoints, finalNormals, finalUV, finalTriangles, finalPointsPrecise);
        }

        private static List<Tri> BuildTriangleList(int vertexCount)
        {
            var tris = new List<Tri>();
            if (vertexCount < 3)
                return tris;
            for (int i = 1; i < vertexCount - 1; i++)
                tris.Add(new Tri(0, i, i + 1));
            return tris;
        }

        private static Vec3D GetPerpendicularVector(Vec3D v)
        {
            Vec3D axis = Math.Abs(v.X) < Math.Abs(v.Y)
                ? (Math.Abs(v.X) < Math.Abs(v.Z) ? new Vec3D(1, 0, 0) : new Vec3D(0, 0, 1))
                : (Math.Abs(v.Y) < Math.Abs(v.Z) ? new Vec3D(0, 1, 0) : new Vec3D(0, 0, 1));
            return Vec3DOps.Cross(v, axis);
        }

        /// <summary>
        /// Resample a ring of points to maintain a maximum arc length between consecutive points.
        /// The ring is split into segments at cornerIndices, and each segment is resampled independently.
        /// </summary>
        private static List<Vec3D> Resample(List<Vec3D> ring, double maxSegmentLength, List<int> cornerIndices)
        {
            if (ring.Count == 0 || cornerIndices.Count == 0)
                return ring;

            List<Vec3D> resampled = new List<Vec3D>();

            // Process each segment between consecutive corner indices
            for (int segIdx = 0; segIdx < cornerIndices.Count; segIdx++)
            {
                int startIdx = cornerIndices[segIdx];
                int endIdx = cornerIndices[(segIdx + 1) % cornerIndices.Count];

                // Extract segment points
                List<Vec3D> segment = new List<Vec3D>();
                
                if (endIdx > startIdx)
                {
                    // Simple case: segment doesn't wrap around
                    for (int i = startIdx; i <= endIdx; i++)
                        segment.Add(ring[i]);
                }
                else
                {
                    // Segment wraps around the ring
                    for (int i = startIdx; i < ring.Count; i++)
                        segment.Add(ring[i]);
                    for (int i = 0; i <= endIdx; i++)
                        segment.Add(ring[i]);
                }

                if (segment.Count < 2)
                    continue;

                // Create line strip for this segment
                LineStrip3D segmentStrip = new LineStrip3D(segment);
                double segmentLength = segmentStrip.TotalLength;

                // Calculate number of points needed for this segment
                int numPoints = Math.Max(2, (int)Math.Ceiling(segmentLength / maxSegmentLength) + 1);

                // Resample the segment
                // Add all points except the last one (to avoid duplicates at corners)
                for (int i = 0; i < numPoints - 1; i++)
                {
                    double t = (double)i / (numPoints - 1);
                    resampled.Add(segmentStrip.EvaluateUniform(t));
                }
            }

            return resampled;
        }

        /// <summary>
        /// Connect two rings of points with triangles using a greedy algorithm.
        /// Rings must be oriented the same way (both clockwise or both counter-clockwise).
        /// </summary>
        public static void ConnectRings(IList<Vec3D> points, List<int> loopA, List<int> loopB, IList<Tri> triangles)
        {
            if (loopA.Count == 0 || loopB.Count == 0)
                return;

            int aLength = loopA.Count;
            int bLength = loopB.Count;
            
            // Special case: single point in loopA (center point)
            if (aLength == 1)
            {
                // Create fan triangles from center to all edges of loopB
                for (int i = 0; i < bLength; i++)
                {
                    int nextI = (i + 1) % bLength;
                    triangles.Add(new Tri(loopA[0], loopB[i], loopB[nextI]));
                }
                return;
            }
            
            // Special case: single point in loopB
            if (bLength == 1)
            {
                // Create fan triangles from center to all edges of loopA
                for (int i = 0; i < aLength; i++)
                {
                    int nextI = (i + 1) % aLength;
                    triangles.Add(new Tri(loopA[i], loopB[0], loopA[nextI]));
                }
                return;
            }
            
            // General case: both loops have multiple points
            // Find the best starting alignment by finding closest point on loopB to loopA[0]
            int startB = FindClosestPointIndex(points, loopA[0], loopB);
            
            int currentA = 0;
            int currentB = startB;
            int stepsA = 0;
            int stepsB = 0;
            
            // Continue until we've wrapped around both loops
            while (stepsA < aLength || stepsB < bLength)
            {
                int nextA = (currentA + 1) % aLength;
                int nextB = (currentB + 1) % bLength;

                bool canAdvanceA = stepsA < aLength;
                bool canAdvanceB = stepsB < bLength;
                
                if (!canAdvanceA && !canAdvanceB)
                    break;
                
                // Calculate distances for both options
                double moveA = canAdvanceA ? (points[loopB[currentB]] - points[loopA[nextA]]).LengthSquared() : double.MaxValue;
                double moveB = canAdvanceB ? (points[loopA[currentA]] - points[loopB[nextB]]).LengthSquared() : double.MaxValue;

                if ((moveA <= moveB || !canAdvanceB) && canAdvanceA)
                {
                    // Advance A
                    triangles.Add(new Tri(loopA[currentA], loopB[currentB], loopA[nextA]));
                    currentA = nextA;
                    stepsA++;
                }
                else if (canAdvanceB)
                {
                    // Advance B
                    triangles.Add(new Tri(loopA[currentA], loopB[currentB], loopB[nextB]));
                    currentB = nextB;
                    stepsB++;
                }
            }
        }
        
        /// <summary>
        /// Find the index in the loop that is closest to the given point index.
        /// </summary>
        private static int FindClosestPointIndex(IList<Vec3D> points, int targetPointIndex, List<int> loop)
        {
            if (loop.Count == 0)
                return 0;
                
            Vec3D targetPoint = points[targetPointIndex];
            int closestIndex = 0;
            double minDistSq = (points[loop[0]] - targetPoint).LengthSquared();
            
            for (int i = 1; i < loop.Count; i++)
            {
                double distSq = (points[loop[i]] - targetPoint).LengthSquared();
                if (distSq < minDistSq)
                {
                    minDistSq = distSq;
                    closestIndex = i;
                }
            }
            
            return closestIndex;
        }

        /// <summary>
        /// Calculate the maximum segment length for a given deviation on a sphere.
        /// Based on the chord-to-arc relationship.
        /// </summary>
        private static double MaxSegmentLengthFromMaxDeviation(double radius, double maxDeviation)
        {
            if (maxDeviation <= 0 || radius <= 0)
                return radius * 0.1; // Fallback

            double ratio = maxDeviation / radius;
            if (ratio >= 1.0)
                return radius * 0.5; // Fallback for large deviations

            // For a circular arc: deviation = radius * (1 - cos(angle/2))
            // Solving for angle: angle = 2 * acos(1 - deviation/radius)
            // Arc length = radius * angle
            double halfAngle = Math.Acos(1.0 - ratio);
            double angle = 2.0 * halfAngle;
            return radius * angle;
        }

        /// <summary>
        /// Find the maximum angle between any two points in the border loop relative to the sphere center.
        /// </summary>
        private static double FindMaxAngle(List<Vec3D> borderLoop, Vec3D sphereCenter)
        {
            double maxAngle = 0;
            
            for (int i = 0; i < borderLoop.Count; i++)
            {
                for (int j = i + 1; j < borderLoop.Count; j++)
                {
                    Vec3D dir1 = (borderLoop[i] - sphereCenter).Normalized();
                    Vec3D dir2 = (borderLoop[j] - sphereCenter).Normalized();
                    
                    double dot = Vec3DOps.Dot(dir1, dir2);
                    // Clamp to avoid numerical errors in acos
                    dot = Math.Max(-1.0, Math.Min(1.0, dot));
                    double angle = Math.Acos(dot);
                    
                    if (angle > maxAngle)
                        maxAngle = angle;
                }
            }
            
            return maxAngle;
        }

        /// <summary>
        /// Spherical linear interpolation between two direction vectors.
        /// Both inputs should be direction vectors (will be normalized internally).
        /// </summary>
        private static Vec3D Slerp(Vec3D start, Vec3D end, double t)
        {
            // Normalize inputs
            Vec3D v0 = start.Normalized();
            Vec3D v1 = end.Normalized();
            
            // Compute the angle between vectors
            double dot = Vec3DOps.Dot(v0, v1);
            
            // Clamp dot product to avoid numerical errors
            dot = Math.Max(-1.0, Math.Min(1.0, dot));
            
            double angle = Math.Acos(dot);
            
            // If angle is very small, use linear interpolation
            if (angle < 1e-6)
            {
                return (start * (1.0 - t) + end * t).Normalized();
            }
            
            // Perform spherical interpolation
            double sinAngle = Math.Sin(angle);
            double w0 = Math.Sin((1.0 - t) * angle) / sinAngle;
            double w1 = Math.Sin(t * angle) / sinAngle;
            
            return v0 * w0 + v1 * w1;
        }
    }
}

