#pragma warning disable CS8625 // Cannot convert null literal to non-nullable reference type
#pragma warning disable CS8618 // Non-nullable property must contain a non-null value when exiting constructor

using GeoCore;

namespace Geo
{
    public static class BlendCorner
    {
        internal static UVSurface PlaneCoveringSurface(UVSurface target, Rat3Hybrid origin,
            Rat3Hybrid normal, CoordinateConverter converter)
        {
            if (target == null) throw new ArgumentNullException(nameof(target));
            if (normal == new Rat3Hybrid(0, 0, 0)) throw new ArgumentException("Plane normal must be nonzero.", nameof(normal));
            if (target.Triangles.Count == 0) throw new ArgumentException("Target surface must contain triangles.", nameof(target));

            int dropped = 0;
            for (int axis = 1; axis < 3; axis++)
                if (Math.Abs(normal[axis].ToDouble()) > Math.Abs(normal[dropped].ToDouble())) dropped = axis;
            int x = (dropped + 1) % 3, y = (dropped + 2) % 3;
            var usedIndices = new HashSet<int>();
            int firstIndex = -1;
            foreach (var triangle in target.Triangles)
            {
                if (firstIndex < 0) firstIndex = triangle.A;
                usedIndices.Add(triangle.A);
                usedIndices.Add(triangle.B);
                usedIndices.Add(triangle.C);
            }
            if (usedIndices.Count == 0) throw new ArgumentException("Target surface has no referenced vertices.", nameof(target));
            var margin = new BigRationalHybrid(converter.ConvertDirection(new Vec3D(converter.SmallestUnit(), 0, 0)).X);
            if (margin.Sign() == 0) margin = BigRationalHybrid.One;
            var first = target.PointsPrecise[firstIndex];
            var minX = first[x]; var maxX = first[x];
            var minY = first[y]; var maxY = first[y];
            foreach (int index in usedIndices)
            {
                var point = target.PointsPrecise[index];
                if (point[x] < minX) minX = point[x];
                if (point[x] > maxX) maxX = point[x];
                if (point[y] < minY) minY = point[y];
                if (point[y] > maxY) maxY = point[y];
            }
            minX -= margin; maxX += margin;
            minY -= margin; maxY += margin;
            var planeD = Rat3Hybrid.Dot(normal, origin);
            Rat3Hybrid Point(BigRationalHybrid a, BigRationalHybrid b)
            {
                var coordinates = new BigRationalHybrid[3];
                coordinates[x] = a;
                coordinates[y] = b;
                coordinates[dropped] = (planeD - normal[x] * a - normal[y] * b) / normal[dropped];
                var point = new Rat3Hybrid(coordinates[0], coordinates[1], coordinates[2]);
                point.Simplify();
                return point;
            }
            var points = new List<Rat3Hybrid> { Point(minX, minY), Point(maxX, minY),
                Point(maxX, maxY), Point(minX, maxY) };
            var triangles = new List<Tri> { new(0, 1, 2), new(0, 2, 3) };
            if (normal[dropped].Sign() < 0)
                triangles = new List<Tri> { new(0, 2, 1), new(0, 3, 2) };
            var direction = new Vec3D(normal.X.ToDouble(), normal.Y.ToDouble(), normal.Z.ToDouble()).Normalized();
            var normals = new List<Vec3D> { direction, direction, direction, direction };
            return new UVSurface(converter.Convert(points), normals,
                new List<Vec2D> { new(0, 0), new(1, 0), new(1, 1), new(0, 1) }, triangles, points);
        }

        public static UVSurface TessellateSphereCap(List<Vec3D> borderLoop, double maxDeviation, List<Rat3Hybrid> exactBoundary, List<int> cornerIndices, CoordinateConverter cc, EdgeBlendType blendType)
        {
            SphereFitter.FitSphere(borderLoop, out var sphereCenter, out var cornerRadius);
            
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
            return TessellateSphereCap(borderLoop, maxDeviation, exactBoundary, cornerIndices,
                cc, blendType, sphereCenter, cornerRadius, directionToCenter);
        }

        // A planar equator cannot determine the sphere pole by fitting.
        // The collapsed-cylinder construction supplies its center and direction.
        internal static UVSurface TessellateSphereCap(List<Vec3D> borderLoop, double maxDeviation,
            List<Rat3Hybrid> exactBoundary, List<int> cornerIndices, CoordinateConverter cc,
            EdgeBlendType blendType, Vec3D sphereCenter, double cornerRadius, Vec3D directionToCenter)
        {
            double maxSegmentLength = MaxSegmentLengthFromMaxDeviation(cornerRadius, maxDeviation);

            double maxOpeningAngle = FindMaxAngle(borderLoop, sphereCenter);
            double maxArcLength = maxOpeningAngle * cornerRadius;

            int spiderNetResolution = (int)(maxArcLength / maxSegmentLength) + 1;

            Vec3D spiderNetCenter = sphereCenter + directionToCenter * cornerRadius;

            List<Vec3D> finalPoints = new List<Vec3D>();
            List<Rat3Hybrid> finalPointsPrecise = new List<Rat3Hybrid>();
            List<Vec3D> finalNormals = new List<Vec3D>();
            List<Vec2D> finalUV = new List<Vec2D>();
            
            BigRationalHybrid Scalar(double value)
            {
                var exact = new BigRational(value);
                return new BigRationalHybrid(exact.Numerator, exact.Denominator);
            }
            Rat3Hybrid FittedPoint(Vec3D point)
            {
                var delta = (point - borderLoop[0]) / cc.SmallestUnit();
                return exactBoundary[0] + new Rat3Hybrid(Scalar(delta.X), Scalar(delta.Y), Scalar(delta.Z));
            }
            var sphereOriginExact = FittedPoint(sphereCenter);
            var centerPrecise = FittedPoint(spiderNetCenter);
            // The shared pole and each boundary ray define an exact radial plane.
            // Independently snapping interior XYZ samples can reverse tiny cells.
            finalPoints.Add(cc.Convert(centerPrecise));
            finalPointsPrecise.Add(centerPrecise);
            
            // Convex removal uses outward sphere normals; concave filling reverses them.
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

                var ringPrecise = new List<Rat3Hybrid>(borderLoop.Count);
                for (int i = 0; i < borderLoop.Count; i++)
                {
                    if (outwardsId == spiderNetResolution)
                    {
                        ringPrecise.Add(exactBoundary[i]);
                        continue;
                    }
                    var poleDirection = (spiderNetCenter - sphereCenter).Normalized();
                    var boundaryDirection = (borderLoop[i] - sphereCenter).Normalized();
                    double angle = Math.Acos(Math.Clamp(Vec3DOps.Dot(poleDirection, boundaryDirection), -1, 1));
                    double sine = Math.Sin(angle);
                    double poleWeight = sine == 0 ? 1 - fraction : Math.Sin((1 - fraction) * angle) / sine;
                    double boundaryWeight = sine == 0 ? fraction : Math.Sin(fraction * angle) / sine;
                    var point = sphereOriginExact + (centerPrecise - sphereOriginExact) * Scalar(poleWeight)
                        + (exactBoundary[i] - sphereOriginExact) * Scalar(boundaryWeight);
                    point.Simplify();
                    ringPrecise.Add(point);
                }
                // Preserve corresponding rays instead of geometric rematching.
                var ring = cc.Convert(ringPrecise);

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
                    
                    finalPointsPrecise.Add(ringPrecise[i]);

                    // Convex removal uses outward sphere normals; concave filling reverses them.
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

                if (prevLoop.Count == 1)
                    ConnectRings(finalPoints, prevLoop, currentLoop, finalTriangles);
                else
                    for (int i = 0; i < currentLoop.Count; i++)
                    {
                        int next = (i + 1) % currentLoop.Count;
                        finalTriangles.Add(new Tri(prevLoop[i], currentLoop[i], currentLoop[next]));
                        finalTriangles.Add(new Tri(prevLoop[i], currentLoop[next], prevLoop[next]));
                    }

                prevLoop = currentLoop;
            }

            // The boundary connector may traverse either direction. Orient the
            // patch from its known sphere center before applying concave polarity.
            foreach (var triangle in finalTriangles)
            {
                var a = finalPointsPrecise[triangle.A];
                var normal = Rat3Hybrid.Cross(finalPointsPrecise[triangle.B] - a, finalPointsPrecise[triangle.C] - a);
                var orientation = Rat3Hybrid.Dot(normal, a - sphereOriginExact);
                if (orientation == BigRationalHybrid.Zero) continue;
                if (orientation < BigRationalHybrid.Zero)
                    for (int i = 0; i < finalTriangles.Count; i++)
                    {
                        var t = finalTriangles[i];
                        finalTriangles[i] = new Tri(t.A, t.C, t.B);
                    }
                break;
            }

            // Concave filling reverses the sphere surface orientation.
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
        // The connector may traverse a corner loop either way. Shared exact
        // boundary edges determine winding; fitted plane normals cannot.
        internal static void OrientBlendPatches(IReadOnlyList<UVSurface> surfaces)
        {
            static IEnumerable<(Rat3Hybrid Start, Rat3Hybrid End)> Boundary(UVSurface surface)
            {
                foreach (var edge in AdjacencyEx.BuildEdgeToTrianglesMap(surface.Triangles))
                {
                    if (edge.Value.Count != 1) continue;
                    var (a, b) = edge.Key;
                    var triangle = surface.Triangles[edge.Value[0]];
                    if (!((triangle.A == a && triangle.B == b) ||
                          (triangle.B == a && triangle.C == b) ||
                          (triangle.C == a && triangle.A == b)))
                        (a, b) = (b, a);
                    yield return (surface.PointsPrecise[a], surface.PointsPrecise[b]);
                }
            }

            static int Compare(Rat3Hybrid a, Rat3Hybrid b)
            {
                int result = a.X.CompareTo(b.X);
                if (result != 0) return result;
                result = a.Y.CompareTo(b.Y);
                return result != 0 ? result : a.Z.CompareTo(b.Z);
            }

            var uses = new Dictionary<(Rat3Hybrid A, Rat3Hybrid B), List<(int Surface, bool Forward)>>();
            for (int surfaceId = 0; surfaceId < surfaces.Count; surfaceId++)
                foreach (var edge in Boundary(surfaces[surfaceId]))
                {
                    bool forward = Compare(edge.Start, edge.End) < 0;
                    var key = forward ? (edge.Start, edge.End) : (edge.End, edge.Start);
                    if (!uses.TryGetValue(key, out var list)) uses.Add(key, list = new());
                    list.Add((surfaceId, forward));
                }

            var links = new List<(int Other, bool Flip)>[surfaces.Count];
            for (int i = 0; i < links.Length; i++) links[i] = new();
            foreach (var use in uses.Values)
            {
                if (use.Count < 2) continue;
                if (use.Count != 2)
                    throw new InvalidOperationException("Blend patches have a non-manifold shared boundary.");
                bool flip = use[0].Forward == use[1].Forward;
                links[use[0].Surface].Add((use[1].Surface, flip));
                links[use[1].Surface].Add((use[0].Surface, flip));
            }
            int[] orientation = new int[surfaces.Count];
            Array.Fill(orientation, -1);
            for (int root = 0; root < surfaces.Count; root++)
            {
                if (orientation[root] >= 0 || links[root].Count == 0) continue;
                // The boundary graph fixes relative winding, not which side of
                // each connected sheet is material. Seed that remaining sign
                // from a nondegenerate patch triangle and its outward normal.
                double strongest = 0;
                var surface = surfaces[root];
                foreach (var tri in surface.Triangles)
                {
                    var winding = Vec3DOps.Cross(surface.Points[tri.B] - surface.Points[tri.A],
                        surface.Points[tri.C] - surface.Points[tri.A]);
                    var normal = surface.Normals[tri.A] + surface.Normals[tri.B] + surface.Normals[tri.C];
                    double alignment = Vec3DOps.Dot(winding, normal);
                    if (Math.Abs(alignment) > Math.Abs(strongest)) strongest = alignment;
                }
                if (strongest == 0)
                    throw new InvalidOperationException("Blend patch component has no oriented, nondegenerate root triangle.");
                orientation[root] = strongest < 0 ? 1 : 0;
                var queue = new Queue<int>();
                queue.Enqueue(root);
                while (queue.Count > 0)
                {
                    int current = queue.Dequeue();
                    foreach (var link in links[current])
                    {
                        int required = orientation[current] ^ (link.Flip ? 1 : 0);
                        if (orientation[link.Other] < 0)
                        {
                            orientation[link.Other] = required;
                            queue.Enqueue(link.Other);
                        }
                        else if (orientation[link.Other] != required)
                            throw new InvalidOperationException("Blend patch junction is not orientable.");
                    }
                }
            }

            for (int surfaceId = 0; surfaceId < surfaces.Count; surfaceId++)
                if (orientation[surfaceId] == 1)
                {
                    var surface = surfaces[surfaceId];
                    for (int i = 0; i < surface.Triangles.Count; i++)
                    {
                        var triangle = surface.Triangles[i];
                        surface.Triangles[i] = new Tri(triangle.A, triangle.C, triangle.B);
                    }
                    for (int i = 0; i < surface.Normals.Count; i++) surface.Normals[i] = -surface.Normals[i];
                }
        }

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

    }
}
