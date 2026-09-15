using GeoCore;

namespace Geo
{
    public static class CoplanarGroupFusionUVUpdate
    {
        /// <summary>
        /// Updates UV coordinates for triangles in fused coplanar groups.
        /// Triangles from the core group (lowest ID) keep their UVs.
        /// Triangles from merged groups get new UVs based on the reference triangle's coordinate system.
        /// </summary>
        public static void UpdateUVsForFusedGroups<T>(
            List<Vec3D> positions,
            List<Tri> triangles,
            List<MeshTriangle<T>> trianglesEx,
            List<int> newGroupIdPerTriangle,
            Dictionary<int, int> representativeTriangles) where T : ITriangleVertex<T>
        {
            // Group triangles by their new group ID
            var triangleIdsPerGroup = CoplanarGroupFusion.ExtractTriangleIdsPerGroup(newGroupIdPerTriangle);

            // For each fused group, update UVs based on the reference triangle
            foreach (var groupEntry in representativeTriangles)
            {
                int groupId = groupEntry.Key;
                int refTriIndex = groupEntry.Value;

                // Get all triangles in this group
                if (!triangleIdsPerGroup.ContainsKey(groupId))
                    continue;

                List<int> triangleIndices = triangleIdsPerGroup[groupId];

                // Get reference triangle data
                Tri refTri = triangles[refTriIndex];
                MeshTriangle<T> refTriEx = trianglesEx[refTriIndex];

                // Extract reference triangle's 3D positions
                Vec3D refP0 = positions[refTri.A];
                Vec3D refP1 = positions[refTri.B];
                Vec3D refP2 = positions[refTri.C];

                // Extract reference triangle's UV coordinates
                var refVertex0 = refTriEx.V0 as TriangleVertexNormalUV?;
                var refVertex1 = refTriEx.V1 as TriangleVertexNormalUV?;
                var refVertex2 = refTriEx.V2 as TriangleVertexNormalUV?;

                if (!refVertex0.HasValue || !refVertex1.HasValue || !refVertex2.HasValue)
                    continue; // Skip if not TriangleVertexNormalUV

                Vec2D refUV0 = refVertex0.Value.UV;
                Vec2D refUV1 = refVertex1.Value.UV;
                Vec2D refUV2 = refVertex2.Value.UV;

                // Compute the 3D to UV transformation based on the reference triangle
                // This creates a coordinate system where the reference triangle's vertices
                // map exactly to their existing UV coordinates
                Compute3DToUVTransform(refP0, refP1, refP2, refUV0, refUV1, refUV2,
                    out Vec3D origin, out Vec3D uAxis3D, out Vec3D vAxis3D,
                    out Vec2D uvOrigin, out Vec2D uvUAxis, out Vec2D uvVAxis);

                // Update UV coordinates for all triangles in this group
                foreach (int triIndex in triangleIndices)
                {
                    Tri tri = triangles[triIndex];
                    MeshTriangle<T> triEx = trianglesEx[triIndex];

                    // Compute new UV coordinates for each vertex
                    Vec2D newUV0 = Transform3DToUV(positions[tri.A], origin, uAxis3D, vAxis3D, uvOrigin, uvUAxis, uvVAxis);
                    Vec2D newUV1 = Transform3DToUV(positions[tri.B], origin, uAxis3D, vAxis3D, uvOrigin, uvUAxis, uvVAxis);
                    Vec2D newUV2 = Transform3DToUV(positions[tri.C], origin, uAxis3D, vAxis3D, uvOrigin, uvUAxis, uvVAxis);

                    // Update the triangle's UV coordinates
                    if (triEx.V0 is TriangleVertexNormalUV v0 &&
                        triEx.V1 is TriangleVertexNormalUV v1 &&
                        triEx.V2 is TriangleVertexNormalUV v2)
                    {
                        v0.UV = newUV0;
                        v1.UV = newUV1;
                        v2.UV = newUV2;

                        triEx.V0 = (T)(object)v0;
                        triEx.V1 = (T)(object)v1;
                        triEx.V2 = (T)(object)v2;

                        trianglesEx[triIndex] = triEx;
                    }
                }
            }
        }

        /// <summary>
        /// Computes the transformation from 3D space to UV space based on a reference triangle.
        /// Uses the triangle's edges directly (not orthonormalized) to preserve the mapping.
        /// </summary>
        private static void Compute3DToUVTransform(
            Vec3D p0, Vec3D p1, Vec3D p2,
            Vec2D uv0, Vec2D uv1, Vec2D uv2,
            out Vec3D origin, out Vec3D edge1_3D, out Vec3D edge2_3D,
            out Vec2D uvOrigin, out Vec2D edge1_UV, out Vec2D edge2_UV)
        {
            // 3D space: use p0 as origin, edges as basis vectors (not normalized)
            origin = p0;
            edge1_3D = p1 - p0;
            edge2_3D = p2 - p0;

            // UV space: use uv0 as origin, edges as basis vectors
            uvOrigin = uv0;
            edge1_UV = uv1 - uv0;
            edge2_UV = uv2 - uv0;
        }

        /// <summary>
        /// Transforms a 3D point to UV coordinates using the computed transformation.
        /// The transformation preserves the affine relationship between 3D and UV space.
        /// </summary>
        private static Vec2D Transform3DToUV(
            Vec3D point3D,
            Vec3D origin3D, Vec3D edge1_3D, Vec3D edge2_3D,
            Vec2D uvOrigin, Vec2D edge1_UV, Vec2D edge2_UV)
        {
            // Express the point in terms of the reference triangle's edges
            // We need to solve: point3D - origin3D = s * edge1_3D + t * edge2_3D
            //
            // This is a 3D system, but the point should lie in the plane of the triangle.
            // We project onto the plane and solve the 2D problem.

            Vec3D localPoint = point3D - origin3D;

            // Compute plane normal
            Vec3D normal = Vec3DOps.Cross(edge1_3D, edge2_3D);
            normal.Normalize();

            // Project localPoint onto the plane (remove any component perpendicular to the plane)
            double perpDist = Vec3DOps.Dot(localPoint, normal);
            Vec3D projectedPoint = localPoint - perpDist * normal;

            // Now solve: projectedPoint = s * edge1_3D + t * edge2_3D
            // This is a 2D problem in the plane. We can use the cross product method.
            //
            // Taking cross product with edge2_3D:
            // projectedPoint × edge2_3D = s * (edge1_3D × edge2_3D)
            // s = |projectedPoint × edge2_3D| / |edge1_3D × edge2_3D|
            //
            // Similarly for t with edge1_3D

            Vec3D cross1 = Vec3DOps.Cross(projectedPoint, edge2_3D);
            Vec3D cross2 = Vec3DOps.Cross(edge1_3D, edge2_3D);
            double s = Vec3DOps.Dot(cross1, normal) / Vec3DOps.Dot(cross2, normal);

            Vec3D cross3 = Vec3DOps.Cross(edge1_3D, projectedPoint);
            double t = Vec3DOps.Dot(cross3, normal) / Vec3DOps.Dot(cross2, normal);

            // Now map (s, t) to UV space using the same coefficients
            Vec2D uv = uvOrigin + s * edge1_UV + t * edge2_UV;

            return uv;
        }
    }
}
