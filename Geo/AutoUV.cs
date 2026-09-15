using GeoCore;

namespace Geo
{

    public struct OrientedRectangle2D
    {
        public Vec2D Center;
        public Vec2D Size; // Width and Height
        public double Angle; // Rotation angle in radians
        public double Area;

        public OrientedRectangle2D(Vec2D center, Vec2D size, double angle)
        {
            Center = center;
            Size = size;
            Angle = angle;
            Area = size.X * size.Y;
        }

        /// <summary>
        /// Gets the four corners of the rectangle in counter-clockwise order
        /// </summary>
        public Vec2D[] GetCorners()
        {
            double cos = Math.Cos(Angle);
            double sin = Math.Sin(Angle);

            double halfWidth = Size.X * 0.5;
            double halfHeight = Size.Y * 0.5;

            Vec2D[] corners = new Vec2D[4];

            // Local corners (before rotation)
            Vec2D[] localCorners = {
                new Vec2D(-halfWidth, -halfHeight),
                new Vec2D(halfWidth, -halfHeight),
                new Vec2D(halfWidth, halfHeight),
                new Vec2D(-halfWidth, halfHeight)
            };

            // Rotate and translate
            for (int i = 0; i < 4; i++)
            {
                double x = localCorners[i].X * cos - localCorners[i].Y * sin;
                double y = localCorners[i].X * sin + localCorners[i].Y * cos;
                corners[i] = new Vec2D(x + Center.X, y + Center.Y);
            }

            return corners;
        }
    }


    public struct TriangleUv
    {
        public Vec2D UvA;
        public Vec2D UvB;
        public Vec2D UvC;

        public TriangleUv(Vec2D uvA, Vec2D uvB, Vec2D uvC)
        {
            UvA = uvA;
            UvB = uvB;
            UvC = uvC;
        }
    }


    // Very experimental and does not produce very good uv coords except for surface patches that are planar or very close to planar
    public static class AutoUV
    {
        //TODO: Maybe use xatlas?

        public static List<TriangleUv> AutoUvPerPatch(List<Vec3D> points, List<Tri> triangles, List<int> groupPerTriangle)
        {
            if (triangles.Count != groupPerTriangle.Count)
                throw new ArgumentException("Triangle count must match group count");

            var uvPerTriangle = new List<TriangleUv>(triangles.Count);
            
            // Initialize with default UV coordinates
            for (int i = 0; i < triangles.Count; i++)
            {
                uvPerTriangle.Add(new TriangleUv(new Vec2D(0, 0), new Vec2D(0, 0), new Vec2D(0, 0)));
            }

            // Group triangles by patch ID
            var patchGroups = new Dictionary<int, List<int>>();
            for (int i = 0; i < triangles.Count; i++)
            {
                int groupId = groupPerTriangle[i];
                if (!patchGroups.ContainsKey(groupId))
                {
                    patchGroups[groupId] = new List<int>();
                }
                patchGroups[groupId].Add(i);
            }

            // Process each patch separately
            foreach (var kvp in patchGroups)
            {
                List<int> triangleIndicesForPatch = kvp.Value;
                
                // Generate UV coordinates for this patch directly into the main list
                AutoUvSinglePatch(points, triangles, triangleIndicesForPatch, uvPerTriangle);
            }

            return uvPerTriangle;
        }

        private static void AutoUvSinglePatch(List<Vec3D> points, List<Tri> triangles, List<int> triangleIndicesForPatch, List<TriangleUv> uvPerTriangle)
        {
            // Create an enumerator that yields triangles based on triangleIndicesForPatch
            IEnumerable<Tri> patchTriangles = triangleIndicesForPatch.Select(index => triangles[index]);
            
            var (normal, pointOnPlane) = PlaneFitter.FitPlane(points, patchTriangles);

            //For later: Maybe use some minimal distortion uv algorithm?

            var x = Vec3DOps.GetOrthoNormal(normal);
            var y = Vec3DOps.Cross(normal, x);
            y.Normalize();

            // Project all points to 2D plane
            Vec2D[] projectedPoints = new Vec2D[points.Count];
            List<Vec2D> usedProjectedPoints = new List<Vec2D>();
            bool[] pointUsed = new bool[points.Count];
            
            // Mark which points are used by triangles in this patch
            foreach (int triangleIndex in triangleIndicesForPatch)
            {
                var tri = triangles[triangleIndex];
                pointUsed[tri.A] = true;
                pointUsed[tri.B] = true;
                pointUsed[tri.C] = true;
            }
            
            // Project all points and collect used ones
            for (int i = 0; i < points.Count; i++)
            {
                var p = points[i] - pointOnPlane;
                projectedPoints[i] = new Vec2D(Vec3DOps.Dot(x, p), Vec3DOps.Dot(y, p));
                
                if (pointUsed[i])
                {
                    usedProjectedPoints.Add(projectedPoints[i]);
                }
            }

            if (!HasConsistentWinding(triangles, triangleIndicesForPatch, projectedPoints))
            {
                if (TryAssignCylindricalUv(points, triangles, triangleIndicesForPatch, uvPerTriangle))
                    return;
            }

            // Find the smallest area rectangle for the used points
            var rectangle = FindSmallestAreaRectangle(usedProjectedPoints);
            
            // Transform all projected points to rectangle's local UV coordinate system
            Vec2D[] uvCoordinates = TransformToRectangleUV(projectedPoints, rectangle);
            
            // Populate UV coordinates directly into the pre-sized list at the correct indices
            foreach (int triangleIndex in triangleIndicesForPatch)
            {
                var tri = triangles[triangleIndex];
                uvPerTriangle[triangleIndex] = new TriangleUv(                
                    uvCoordinates[tri.A],
                    uvCoordinates[tri.B],
                    uvCoordinates[tri.C]
                );
            }
        }

        /// <summary>
        /// Checks that all triangles have consistent winding when projected onto the 2D plane.
        /// Throws an exception if triangles have inconsistent orientation.
        /// </summary>
        /// <param name="triangles">List of all triangles</param>
        /// <param name="triangleIndicesForPatch">Indices of triangles in this patch</param>
        /// <param name="projectedPoints">2D projected points</param>
        private static bool HasConsistentWinding(List<Tri> triangles, List<int> triangleIndicesForPatch, Vec2D[] projectedPoints)
        {
            if (triangleIndicesForPatch.Count == 0)
                return true;

            // Calculate the signed area (cross product) of the first triangle to determine expected winding
            int firstTriIndex = triangleIndicesForPatch[0];
            var firstTri = triangles[firstTriIndex];
            var a = projectedPoints[firstTri.A];
            var b = projectedPoints[firstTri.B];
            var c = projectedPoints[firstTri.C];
            
            double expectedSign = Math.Sign(Vec2DOps.Cross(b - a, c - a));
            
            // If the first triangle is degenerate, find the first non-degenerate one
            if (Math.Abs(expectedSign) < 1e-12)
            {
                for (int i = 1; i < triangleIndicesForPatch.Count; i++)
                {
                    int triIndex = triangleIndicesForPatch[i];
                    var tri = triangles[triIndex];
                    a = projectedPoints[tri.A];
                    b = projectedPoints[tri.B];
                    c = projectedPoints[tri.C];
                    
                    expectedSign = Math.Sign(Vec2DOps.Cross(b - a, c - a));
                    if (Math.Abs(expectedSign) >= 1e-12)
                        break;
                }
                
                if (Math.Abs(expectedSign) < 1e-12)
                    return true;
            }

            // Check all triangles have the same winding
            foreach (int triangleIndex in triangleIndicesForPatch)
            {
                var tri = triangles[triangleIndex];
                a = projectedPoints[tri.A];
                b = projectedPoints[tri.B];
                c = projectedPoints[tri.C];
                
                double crossProduct = Vec2DOps.Cross(b - a, c - a);
                double currentSign = Math.Sign(crossProduct);
                
                // Skip degenerate triangles
                if (Math.Abs(crossProduct) < 1e-12)
                    continue;
                
                if (currentSign != expectedSign)
                    return false;
            }
            return true;
        }

        private static bool TryAssignCylindricalUv(
            List<Vec3D> points,
            List<Tri> triangles,
            List<int> triangleIndicesForPatch,
            List<TriangleUv> uvPerTriangle)
        {
            if (triangleIndicesForPatch.Count < 8)
                return false;

            var normals = new Vec3D[points.Count];
            var used = new HashSet<int>();
            foreach (int triangleIndex in triangleIndicesForPatch)
            {
                var tri = triangles[triangleIndex];
                used.Add(tri.A);
                used.Add(tri.B);
                used.Add(tri.C);
                var n = Vec3DOps.Cross(points[tri.B] - points[tri.A], points[tri.C] - points[tri.A]);
                normals[tri.A] += n;
                normals[tri.B] += n;
                normals[tri.C] += n;
            }
            var normalList = new List<Vec3D>(points.Count);
            for (int i = 0; i < points.Count; i++)
            {
                double len = normals[i].Length();
                normalList.Add(len > 1e-15 ? normals[i] * (1.0 / len) : new Vec3D(0, 0, 1));
            }

            if (!CylinderAxisFit.TryFit(points, normalList, used,
                    out var axis, out var pointOnAxis, out double radius, out double minH, out double maxH))
                return false;
            double height = maxH - minH;

            var refDir = Vec3DOps.GetOrthoNormal(axis);
            var binormal = Vec3DOps.Cross(axis, refDir).Normalized();
            var uv = new Vec2D[points.Count];
            foreach (int i in used)
            {
                var d = points[i] - pointOnAxis;
                double h = Vec3DOps.Dot(d, axis);
                var radial = d - axis * h;
                double angle = Math.Atan2(Vec3DOps.Dot(radial, binormal), Vec3DOps.Dot(radial, refDir));
                uv[i] = new Vec2D((angle + Math.PI) / (2 * Math.PI), (h - minH) / height);
            }

            foreach (int triangleIndex in triangleIndicesForPatch)
            {
                var tri = triangles[triangleIndex];
                uvPerTriangle[triangleIndex] = new TriangleUv(uv[tri.A], uv[tri.B], uv[tri.C]);
            }
            return true;
        }

        /// <summary>
        /// Planar UV in [0,1]² from a 2D point set via the minimum-area oriented rectangle (rotating calipers on the convex hull).
        /// Longer rectangle side maps to U. Degenerate input maps to (0,0).
        /// </summary>
        public static Vec2D[] ComputePlanarUvFromPoints2D(IReadOnlyList<Vec2D> points)
        {
            if (points == null || points.Count == 0)
                throw new ArgumentException("Point list cannot be empty", nameof(points));

            OrientedRectangle2D rectangle = NormalizeRectangleForUv(FindMinimumAreaRectangle(points));
            var arr = new Vec2D[points.Count];
            for (int i = 0; i < points.Count; i++)
                arr[i] = points[i];
            return TransformToRectangleUV(arr, rectangle);
        }

        /// <summary>
        /// Maps a single 2D point into the UV chart of <see cref="ComputePlanarUvFromPoints2D"/> for the same point set.
        /// </summary>
        public static Vec2D ComputePlanarUvPoint(Vec2D point, IReadOnlyList<Vec2D> chartPoints)
        {
            if (chartPoints == null || chartPoints.Count == 0)
                throw new ArgumentException("Point list cannot be empty", nameof(chartPoints));
            OrientedRectangle2D rectangle = NormalizeRectangleForUv(FindMinimumAreaRectangle(chartPoints));
            return TransformToRectangleUV(new[] { point }, rectangle)[0];
        }

        /// <summary>
        /// Minimum-area oriented rectangle of a 2D point set (AABB when the hull is axis-aligned).
        /// </summary>
        public static OrientedRectangle2D FindMinimumAreaRectangle(IReadOnlyList<Vec2D> points)
        {
            if (points == null || points.Count == 0)
                throw new ArgumentException("Point list cannot be empty", nameof(points));
            return FindSmallestAreaRectangle(points as List<Vec2D> ?? new List<Vec2D>(points));
        }

        private static OrientedRectangle2D NormalizeRectangleForUv(OrientedRectangle2D rectangle)
        {
            // Prefer longer side as U for stable chart orientation (e.g. airfoil chord).
            if (rectangle.Size.Y > rectangle.Size.X)
            {
                return new OrientedRectangle2D(
                    rectangle.Center,
                    new Vec2D(rectangle.Size.Y, rectangle.Size.X),
                    rectangle.Angle + Math.PI * 0.5);
            }
            return rectangle;
        }

        /// <summary>
        /// Transforms projected 2D points to UV coordinates (0-1 range) within the oriented rectangle.
        /// Uses one corner as origin, with rectangle sides defining U and V axes.
        /// </summary>
        public static Vec2D[] TransformToRectangleUV(Vec2D[] projectedPoints, OrientedRectangle2D rectangle)
        {
            Vec2D[] uvCoordinates = new Vec2D[projectedPoints.Length];
            
            // Get rectangle corners (counter-clockwise order)
            Vec2D[] corners = rectangle.GetCorners();
            
            // Use corner[0] as origin, corner[1] defines U axis, corner[3] defines V axis
            Vec2D origin = corners[0];
            Vec2D uAxis = corners[1] - corners[0]; // Right side of rectangle
            Vec2D vAxis = corners[3] - corners[0]; // Top side of rectangle
            
            // Normalize axes to get unit vectors and lengths
            double uLength = uAxis.Length();
            double vLength = vAxis.Length();
            
            if (uLength < 1e-12 || vLength < 1e-12)
            {
                // Degenerate rectangle - all points map to (0,0)
                for (int i = 0; i < projectedPoints.Length; i++)
                {
                    uvCoordinates[i] = new Vec2D(0, 0);
                }
                return uvCoordinates;
            }
            
            Vec2D uUnit = uAxis / uLength;
            Vec2D vUnit = vAxis / vLength;
            
            // Transform each point to UV coordinates
            for (int i = 0; i < projectedPoints.Length; i++)
            {
                Vec2D localPoint = projectedPoints[i] - origin;
                
                // Project onto U and V axes and normalize by rectangle dimensions
                double u = Vec2DOps.Dot(localPoint, uUnit) / uLength;
                double v = Vec2DOps.Dot(localPoint, vUnit) / vLength;
                
                uvCoordinates[i] = new Vec2D(u, v);
            }
            
            return uvCoordinates;
        }

        private static OrientedRectangle2D FindSmallestAreaRectangle(List<Vec2D> projectedPoints)
        {
            if (projectedPoints.Count == 0)
                throw new ArgumentException("Point list cannot be empty");
            
            if (projectedPoints.Count == 1)
                return new OrientedRectangle2D(projectedPoints[0], new Vec2D(0, 0), 0);
            
            if (projectedPoints.Count == 2)
            {
                Vec2D center = (projectedPoints[0] + projectedPoints[1]) * 0.5;
                Vec2D diff = projectedPoints[1] - projectedPoints[0];
                double length = diff.Length();
                double angle = Math.Atan2(diff.Y, diff.X);
                return new OrientedRectangle2D(center, new Vec2D(length, 0), angle);
            }

            // Compute convex hull first
            var hull = ComputeConvexHull(projectedPoints);
            
            if (hull.Count < 3)
            {
                // Degenerate case - all points are collinear
                return FindSmallestAreaRectangleForCollinearPoints(hull);
            }

            // Use rotating calipers to find minimum area rectangle
            return FindMinimumAreaRectangleRotatingCalipers(hull);
        }

        private static List<Vec2D> ComputeConvexHull(List<Vec2D> points)
        {
            // Andrew's monotone chain algorithm
            var sortedPoints = points.OrderBy(p => p.X).ThenBy(p => p.Y).ToList();
            
            // Build lower hull
            var lower = new List<Vec2D>();
            for (int i = 0; i < sortedPoints.Count; i++)
            {
                while (lower.Count >= 2 && Vec2DOps.Cross(lower[lower.Count - 2], lower[lower.Count - 1], sortedPoints[i]) <= 0)
                    lower.RemoveAt(lower.Count - 1);
                lower.Add(sortedPoints[i]);
            }
            
            // Build upper hull
            var upper = new List<Vec2D>();
            for (int i = sortedPoints.Count - 1; i >= 0; i--)
            {
                while (upper.Count >= 2 && Vec2DOps.Cross(upper[upper.Count - 2], upper[upper.Count - 1], sortedPoints[i]) <= 0)
                    upper.RemoveAt(upper.Count - 1);
                upper.Add(sortedPoints[i]);
            }
            
            // Remove last point of each half because it's repeated
            lower.RemoveAt(lower.Count - 1);
            upper.RemoveAt(upper.Count - 1);
            
            lower.AddRange(upper);
            return lower;
        }


        private static OrientedRectangle2D FindSmallestAreaRectangleForCollinearPoints(List<Vec2D> points)
        {
            if (points.Count <= 1)
                return new OrientedRectangle2D(points[0], new Vec2D(0, 0), 0);
            
            Vec2D min = points[0];
            Vec2D max = points[0];
            
            foreach (var point in points)
            {
                if (point.X < min.X || (point.X == min.X && point.Y < min.Y))
                    min = point;
                if (point.X > max.X || (point.X == max.X && point.Y > max.Y))
                    max = point;
            }
            
            Vec2D center = (min + max) * 0.5;
            Vec2D diff = max - min;
            double length = diff.Length();
            double angle = Math.Atan2(diff.Y, diff.X);
            
            return new OrientedRectangle2D(center, new Vec2D(length, 0), angle);
        }

        private static OrientedRectangle2D FindMinimumAreaRectangleRotatingCalipers(List<Vec2D> hull)
        {
            int n = hull.Count;
            if (n < 3) return FindSmallestAreaRectangleForCollinearPoints(hull);
            
            double minArea = double.MaxValue;
            OrientedRectangle2D bestRectangle = new OrientedRectangle2D();
            
            // For each edge of the convex hull
            for (int i = 0; i < n; i++)
            {
                Vec2D edge = hull[(i + 1) % n] - hull[i];
                double angle = Math.Atan2(edge.Y, edge.X);
                
                // Rotate all points to align this edge with x-axis
                var rotatedPoints = new List<Vec2D>();
                double cos = Math.Cos(-angle);
                double sin = Math.Sin(-angle);
                
                foreach (var point in hull)
                {
                    double x = point.X * cos - point.Y * sin;
                    double y = point.X * sin + point.Y * cos;
                    rotatedPoints.Add(new Vec2D(x, y));
                }
                
                // Find axis-aligned bounding box
                double minX = rotatedPoints[0].X;
                double maxX = rotatedPoints[0].X;
                double minY = rotatedPoints[0].Y;
                double maxY = rotatedPoints[0].Y;
                
                foreach (var point in rotatedPoints)
                {
                    minX = Math.Min(minX, point.X);
                    maxX = Math.Max(maxX, point.X);
                    minY = Math.Min(minY, point.Y);
                    maxY = Math.Max(maxY, point.Y);
                }
                
                double width = maxX - minX;
                double height = maxY - minY;
                double area = width * height;
                
                if (area < minArea)
                {
                    minArea = area;
                    
                    // Transform center back to original coordinate system
                    double centerX = (minX + maxX) * 0.5;
                    double centerY = (minY + maxY) * 0.5;
                    
                    cos = Math.Cos(angle);
                    sin = Math.Sin(angle);
                    double originalCenterX = centerX * cos - centerY * sin;
                    double originalCenterY = centerX * sin + centerY * cos;
                    
                    bestRectangle = new OrientedRectangle2D(
                        new Vec2D(originalCenterX, originalCenterY),
                        new Vec2D(width, height),
                        angle
                    );
                }
            }
            
            return bestRectangle;
        }
    }
}
