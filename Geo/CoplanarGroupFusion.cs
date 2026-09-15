using GeoCore;

namespace Geo
{

    public static class CoplanarGroupFusion
    {        
        // Generic plane representation for exact arithmetic
        private struct RationalPlane
        {
            public Rat3Hybrid Normal;
            public BigRationalHybrid PlaneD;

            public RationalPlane(Rat3Hybrid normal, Rat3Hybrid pointOnPlane)
            {
                Normal = normal;
                PlaneD = -(Rat3Hybrid.Dot(normal, pointOnPlane));
            }

            public BigRationalHybrid SignedDistance(Rat3Hybrid point)
            {
                return Rat3Hybrid.Dot(point, Normal) + PlaneD;
            }
        }



        public static Dictionary<int, List<int>> ExtractTriangleIdsPerGroup(List<int> groupIdPerTriangle)
        {
            var result = new Dictionary<int, List<int>>();

            for (int i = 0; i < groupIdPerTriangle.Count; i++)
            {
                int groupId = groupIdPerTriangle[i];

                if (!result.ContainsKey(groupId))
                    result[groupId] = new List<int>();

                result[groupId].Add(i);
            }

            return result;
        }

        // Helper method to compute plane from triangles using exact arithmetic
        private static RationalPlane ComputeRationalPlaneFromTriangles(List<Tri> triangles, List<Rat3Hybrid> points, List<int> triIndices)
        {
            // Use the first triangle to compute the plane (all should be coplanar if group is planar)
            var tri = triangles[triIndices[0]];
            Rat3Hybrid a = points[tri.A];
            Rat3Hybrid b = points[tri.B];
            Rat3Hybrid c = points[tri.C];

            // Compute normal using cross product
            Rat3Hybrid normal = Rat3Hybrid.Cross(b - a, c - a);

            return new RationalPlane(normal, a);
        }

        // Helper method to check if two rational planes are exactly coplanar
        private static bool RationalPlanesAreCoplanar(RationalPlane planeA, RationalPlane planeB)
        {
            // Two planes are coplanar if their normals are parallel and they have the same PlaneD
            // For exact arithmetic: n1 × n2 = 0 (cross product is zero) and d1 = d2

            // Check if normals are parallel: cross product should be zero
            Rat3Hybrid crossProduct = Rat3Hybrid.Cross(planeA.Normal, planeB.Normal);
            if (!crossProduct.IsZero())
                return false;

            // Require same-facing normals (reject anti-parallel)
            if (Rat3Hybrid.Dot(planeA.Normal, planeB.Normal).Sign() <= 0)
                return false;

            // Normals are parallel and same-facing; check if planes coincide
            // We need to normalize the plane equations first
            // Plane equation: n·p + d = 0
            // Two planes are the same if (n1, d1) = k*(n2, d2) for some scalar k

            // Find a non-zero component of normal A to use as reference
            BigRationalHybrid scaleA = BigRationalHybrid.Zero;
            BigRationalHybrid scaleB = BigRationalHybrid.Zero;

            if (planeA.Normal.X.Sign() != 0 && planeB.Normal.X.Sign() != 0)
            {
                scaleA = planeA.Normal.X;
                scaleB = planeB.Normal.X;
            }
            else if (planeA.Normal.Y.Sign() != 0 && planeB.Normal.Y.Sign() != 0)
            {
                scaleA = planeA.Normal.Y;
                scaleB = planeB.Normal.Y;
            }
            else if (planeA.Normal.Z.Sign() != 0 && planeB.Normal.Z.Sign() != 0)
            {
                scaleA = planeA.Normal.Z;
                scaleB = planeB.Normal.Z;
            }
            else
            {
                // Degenerate case: at least one normal is zero
                return false;
            }

            // Check if d1/n1[i] == d2/n2[i]
            // Equivalent to: d1 * n2[i] == d2 * n1[i]
            return planeA.PlaneD * scaleB == planeB.PlaneD * scaleA;
        }

        // Core fusion logic extracted to be reusable
        private static Dictionary<int, int> FuseCoplanarGroupsCore<TPlane>(
            List<Tri> triangles,
            List<int> groupIdPerTriangle,
            Dictionary<int, string> triangleGroupToName,
            Dictionary<int, TPlane> groupPlanes,
            Dictionary<int, bool> groupIsPlanar,
            Func<TPlane, TPlane, bool> arePlanesCoplanar,
            List<Rat3Hybrid> points,
            out List<int> newGroupIdPerTriangle,
            out Dictionary<int, string> newTriangleGroupToName)
        {
            var groupAdjacency = GroupEdgeExtractor.ExtractGroupEdges(triangles, groupIdPerTriangle, null, points, triangleGroupToName);
            var triangleIdsPerGroup = ExtractTriangleIdsPerGroup(groupIdPerTriangle);

            // Build adjacency map: groupId -> list of adjacent groupIds
            var adjacencyMap = new Dictionary<int, HashSet<int>>();
            foreach (var groupEdge in groupAdjacency)
            {
                if (!adjacencyMap.ContainsKey(groupEdge.GroupIdA))
                    adjacencyMap[groupEdge.GroupIdA] = new HashSet<int>();
                if (!adjacencyMap.ContainsKey(groupEdge.GroupIdB))
                    adjacencyMap[groupEdge.GroupIdB] = new HashSet<int>();

                adjacencyMap[groupEdge.GroupIdA].Add(groupEdge.GroupIdB);
                adjacencyMap[groupEdge.GroupIdB].Add(groupEdge.GroupIdA);
            }

            // Flood fill to merge coplanar groups
            var groupMapping = new Dictionary<int, int>();
            var processedGroups = new HashSet<int>();

            foreach (int startGroupId in triangleIdsPerGroup.Keys.OrderBy(x => x))
            {
                if (processedGroups.Contains(startGroupId))
                    continue;

                // Only start flood fill from planar groups
                if (!groupIsPlanar[startGroupId])
                {
                    processedGroups.Add(startGroupId);
                    groupMapping[startGroupId] = startGroupId;
                    continue;
                }

                // Start flood fill from this group
                var queue = new Queue<int>();
                queue.Enqueue(startGroupId);
                processedGroups.Add(startGroupId);
                groupMapping[startGroupId] = startGroupId;

                TPlane startPlane = groupPlanes[startGroupId];

                while (queue.Count > 0)
                {
                    int currentGroupId = queue.Dequeue();

                    // Check all adjacent groups
                    if (adjacencyMap.ContainsKey(currentGroupId))
                    {
                        foreach (int adjacentGroupId in adjacencyMap[currentGroupId])
                        {
                            if (processedGroups.Contains(adjacentGroupId))
                                continue;

                            // Only merge if the adjacent group is also planar
                            if (!groupIsPlanar[adjacentGroupId])
                                continue;

                            // Check if adjacent group is coplanar with the start group
                            TPlane adjacentPlane = groupPlanes[adjacentGroupId];
                            if (arePlanesCoplanar(startPlane, adjacentPlane))
                            {
                                // Merge this group into the start group
                                processedGroups.Add(adjacentGroupId);
                                groupMapping[adjacentGroupId] = startGroupId;
                                queue.Enqueue(adjacentGroupId);
                            }
                        }
                    }
                }
            }

            // Apply the mapping to create new group assignments
            newGroupIdPerTriangle = new List<int>(groupIdPerTriangle.Count);
            for (int i = 0; i < groupIdPerTriangle.Count; i++)
            {
                int oldGroupId = groupIdPerTriangle[i];
                int newGroupId = groupMapping.ContainsKey(oldGroupId) ? groupMapping[oldGroupId] : oldGroupId;
                newGroupIdPerTriangle.Add(newGroupId);
            }

            // Create new group name mapping
            newTriangleGroupToName = new Dictionary<int, string>();
            var usedNewGroupIds = new HashSet<int>(newGroupIdPerTriangle);
            foreach (int groupId in usedNewGroupIds)
            {
                if (triangleGroupToName.ContainsKey(groupId))
                {
                    newTriangleGroupToName[groupId] = triangleGroupToName[groupId];
                }
            }

            // Create dictionary with representative triangles ONLY for groups that were fused
            // A group was fused if multiple old group IDs map to the same new group ID
            var representativeTriangles = new Dictionary<int, int>();
            
            // Count how many old groups map to each new group
            var groupMappingCounts = new Dictionary<int, int>();
            foreach (var mapping in groupMapping)
            {
                int newGroupId = mapping.Value;
                if (!groupMappingCounts.ContainsKey(newGroupId))
                    groupMappingCounts[newGroupId] = 0;
                groupMappingCounts[newGroupId]++;
            }
            
            // Only include groups where fusion actually occurred (more than 1 old group mapped to it)
            foreach (int groupId in usedNewGroupIds)
            {
                if (groupMappingCounts.ContainsKey(groupId) && groupMappingCounts[groupId] > 1)
                {
                    // Get the first triangle index from the original group that won
                    if (triangleIdsPerGroup.ContainsKey(groupId))
                    {
                        List<int> triIndices = triangleIdsPerGroup[groupId];
                        if (triIndices.Count > 0)
                        {
                            int firstTriIndex = triIndices[0];
                            representativeTriangles[groupId] = firstTriIndex;
                        }
                    }
                    else
                        throw new Exception();
                }
            }

            return representativeTriangles;
        }

        public static Dictionary<int, int> FuseCoplanarGroups(List<Tri> triangles, List<int> groupIdPerTriangle,
            List<Rat3Hybrid> points, Dictionary<int, string> triangleGroupToName, HashSet<int> idsOfPlanarGroups,
            out List<int> newGroupIdPerTriangle, out Dictionary<int, string> newTriangleGroupToName)
        {
            // Extract triangle indices per group
            var triangleIdsPerGroup = ExtractTriangleIdsPerGroup(groupIdPerTriangle);

            // Compute exact rational plane for each group
            var groupPlanes = new Dictionary<int, RationalPlane>();
            var groupIsPlanar = new Dictionary<int, bool>();

            foreach (var groupEntry in triangleIdsPerGroup)
            {
                int groupId = groupEntry.Key;
                List<int> triIndices = groupEntry.Value;

                // Check if this group is marked as planar
                bool isPlanar = idsOfPlanarGroups.Contains(groupId);
                groupIsPlanar[groupId] = isPlanar;

                // Compute plane only for planar groups
                if (isPlanar)
                {
                    groupPlanes[groupId] = ComputeRationalPlaneFromTriangles(triangles, points, triIndices);
                }
                else
                {
                    // For non-planar groups, create a dummy plane (won't be used)
                    groupPlanes[groupId] = new RationalPlane(new Rat3Hybrid(0, 0, 1), Rat3Hybrid.Zero);
                }
            }

            // Use the core fusion logic with exact comparison
            return FuseCoplanarGroupsCore(
                triangles,
                groupIdPerTriangle,
                triangleGroupToName,
                groupPlanes,
                groupIsPlanar,
                RationalPlanesAreCoplanar,
                points,
                out newGroupIdPerTriangle,
                out newTriangleGroupToName);
        }

        public static Dictionary<int, int> FuseCoplanarGroups(List<Tri> triangles, List<int> groupIdPerTriangle,
            List<Vec3D> points, Dictionary<int, string> triangleGroupToName,
            out List<int> newGroupIdPerTriangle, out Dictionary<int, string> newTriangleGroupToName, double tolerance = 1e-8)
        {
            // Extract triangle indices per group
            var triangleIdsPerGroup = ExtractTriangleIdsPerGroup(groupIdPerTriangle);

            // Compute plane for each group
            var groupPlanes = new Dictionary<int, Plane>();
            var groupIsPlanar = new Dictionary<int, bool>();

            foreach (var groupEntry in triangleIdsPerGroup)
            {
                int groupId = groupEntry.Key;
                List<int> triIndices = groupEntry.Value;

                // Fit a plane to all triangles in the group using PCA-based fitting
                var (normal, pointOnPlane) = PlaneFitter.FitPlane(points, new SubsetEnumerator<Tri>(triIndices, triangles));
                var plane = new Plane(normal, pointOnPlane);
                groupPlanes[groupId] = plane;

                // Compute the max distance of all points in the group to the plane
                // Only surfaces that pass the planar test are eligible for coplanar fusion
                double maxDistance = 0;
                foreach (int triIndex in triIndices)
                {
                    var tri = triangles[triIndex];

                    // Check distance for all three vertices
                    double distA = Math.Abs(plane.SignedDistance(points[tri.A]));
                    double distB = Math.Abs(plane.SignedDistance(points[tri.B]));
                    double distC = Math.Abs(plane.SignedDistance(points[tri.C]));

                    maxDistance = Math.Max(maxDistance, Math.Max(distA, Math.Max(distB, distC)));
                }

                // Mark group as planar if all points are within tolerance
                groupIsPlanar[groupId] = maxDistance <= tolerance;
            }

            // Use the core fusion logic with tolerance-based comparison
            return FuseCoplanarGroupsCore(
                triangles,
                groupIdPerTriangle,
                triangleGroupToName,
                groupPlanes,
                groupIsPlanar,
                (planeA, planeB) => GroupsAreCoplanar(planeA, planeB, tolerance),
                null,
                out newGroupIdPerTriangle,
                out newTriangleGroupToName);
        }

        private static bool GroupsAreCoplanar(Plane groupPlaneA, Plane groupPlaneB, double tolerance = 1e-8)
        {
            // Require same-facing normals (reject anti-parallel)
            double dotProduct = Vec3DOps.Dot(groupPlaneA.Normal, groupPlaneB.Normal);
            if (dotProduct < 1.0 - tolerance)
                return false;

            // Check if planes are at the same distance from origin
            // For parallel planes, we need to check if they coincide
            // We can do this by checking if the distance between the planes is zero
            double planeDiff = Math.Abs(groupPlaneA.PlaneD - groupPlaneB.PlaneD);

            return planeDiff < tolerance;
        }
    }
}
