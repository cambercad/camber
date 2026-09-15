using CSG;
using GeoCore;

namespace Geo
{
    /// <summary>
    /// Result of an exact finite-segment cast against a triangle mesh (lattice arithmetic).
    /// </summary>
    public class RayMeshHit
    {
        public Vec3D Point { get; set; }
        public Rat3Hybrid PointPrecise { get; set; }
        /// <summary>Unit geometric normal from triangle winding (world space).</summary>
        public Vec3D GeometricNormal { get; set; }
        public int TriangleIndex { get; set; }
        public int GroupId { get; set; }
        /// <summary>Normalized distance along the cast segment (0 = start, 1 = end).</summary>
        public double ParameterT { get; set; }
    }

    /// <summary>
    /// Exact raycast against <see cref="MeshNormalUV.PrecisionPositions"/> using
    /// <see cref="TriangleSegmentIntersector"/> (finite segment covering the mesh AABB).
    /// </summary>
    public static class RayMeshExact
    {
        /// <summary>
        /// Casts from <paramref name="origin"/> along <paramref name="direction"/> and returns the nearest
        /// intersection with <paramref name="mesh"/> (smallest t &gt; 0). Returns false on a miss.
        /// </summary>
        public static bool TryCast(
            AnchorMesh mesh,
            CoordinateConverter converter,
            Vec3D origin,
            Vec3D direction,
            out RayMeshHit hit)
        {
            hit = null;
            if (mesh == null || mesh.Mesh == null || mesh.Mesh.Triangles == null || mesh.Mesh.Triangles.Count == 0)
                return false;

            double dirLen = direction.Length();
            if (dirLen < 1e-30)
                return false;
            Vec3D dirUnit = direction * (1.0 / dirLen);

            Box3D aabb = ComputeAabb(mesh.Mesh.Positions);
            double maxT = MaxForwardHitDistance(aabb, origin, dirUnit);
            if (maxT < 1e-12)
                return false;

            // Pad so the segment fully covers the AABB in lattice space.
            double pad = Math.Max(converter.SmallestUnit() * 4.0, (aabb.Max - aabb.Min).Length() * 1e-6);
            maxT += pad;

            Vec3D endWorld = origin + dirUnit * maxT;
            Int3 startI = converter.Convert(origin);
            Int3 endI = converter.Convert(endWorld);
            Rat3Hybrid segmentStart = new Rat3Hybrid(startI.X, startI.Y, startI.Z);
            Rat3Hybrid segmentEnd = new Rat3Hybrid(endI.X, endI.Y, endI.Z);

            if (segmentStart == segmentEnd)
                return false;

            var positions = mesh.Mesh.PrecisionPositions;
            var triangles = mesh.Mesh.Triangles;
            var groups = mesh.Mesh.GetTriangleGroups();
            var worldPos = mesh.Mesh.Positions;

            bool found = false;
            double bestT = double.MaxValue;
            Rat3Hybrid bestPoint = default;
            int bestTri = -1;
            int bestGroup = -1;
            Vec3D bestNormal = default;

            for (int i = 0; i < triangles.Count; i++)
            {
                Tri tri = triangles[i];
                Rat3Hybrid a = positions[tri.A];
                Rat3Hybrid b = positions[tri.B];
                Rat3Hybrid c = positions[tri.C];

                var kind = TriangleSegmentIntersector.SegmentIntersectsTriangle(
                    in segmentStart, in segmentEnd, in a, in b, in c,
                    out Rat3Hybrid intersection, out _, out _, out _);

                if (kind != SegmentTriangleIntersectionType.Intersect)
                    continue;

                Vec3D hitWorld = converter.Convert(intersection);
                double t = Vec3DOps.Dot(hitWorld - origin, dirUnit);
                if (t <= 1e-12 || t >= bestT)
                    continue;

                Vec3D wa = worldPos[tri.A];
                Vec3D wb = worldPos[tri.B];
                Vec3D wc = worldPos[tri.C];
                Vec3D n = Vec3DOps.Cross(wb - wa, wc - wa);
                double nLen = n.Length();
                if (nLen < 1e-30)
                    continue;
                n = n * (1.0 / nLen);

                bestT = t;
                bestPoint = intersection;
                bestTri = i;
                bestGroup = groups[i];
                bestNormal = n;
                found = true;
            }

            if (!found)
                return false;

            hit = new RayMeshHit
            {
                Point = converter.Convert(bestPoint),
                PointPrecise = bestPoint,
                GeometricNormal = bestNormal,
                TriangleIndex = bestTri,
                GroupId = bestGroup,
                ParameterT = bestT / maxT
            };
            return true;
        }

        private static Box3D ComputeAabb(List<Vec3D> positions)
        {
            if (positions == null || positions.Count == 0)
                return Box3D.Empty;
            Box3D box = new Box3D(positions[0]);
            for (int i = 1; i < positions.Count; i++)
                box.IncludePoint(positions[i]);
            return box;
        }

        private static double MaxForwardHitDistance(Box3D aabb, Vec3D origin, Vec3D dirUnit)
        {
            if (aabb.IsEmpty())
                return 0;

            double maxT = 0;
            // All 8 corners of the AABB projected onto the ray.
            for (int ix = 0; ix < 2; ix++)
            {
                double x = ix == 0 ? aabb.Min.X : aabb.Max.X;
                for (int iy = 0; iy < 2; iy++)
                {
                    double y = iy == 0 ? aabb.Min.Y : aabb.Max.Y;
                    for (int iz = 0; iz < 2; iz++)
                    {
                        double z = iz == 0 ? aabb.Min.Z : aabb.Max.Z;
                        double t = Vec3DOps.Dot(new Vec3D(x, y, z) - origin, dirUnit);
                        if (t > maxT)
                            maxT = t;
                    }
                }
            }
            return maxT;
        }
    }
}
