using GeoCore;
using GeoMeta;
using NURBS;

namespace Geo.NurbsConstruction
{
    public static class BlendNurbsMetadataBuilder
    {
        private const int ArcProfileSamples = 4;

        public static void AttachBlendPatchNurbs(
            Dictionary<string, SurfaceMetaData> meta,
            IReadOnlyList<BlendEdge> blendEdges,
            AnchorMesh probeMesh)
        {
            foreach (var edge in blendEdges)
            {
                if (edge?.SourceEdge == null || edge.BlendSurface == null)
                    continue;

                string name = EntityNaming.BlendEdge(edge.SourceEdge.Name);
                if (TryBuildBlendEdgeSurface(edge, out var surface))
                {
                    var patch = new SurfaceMetaData(SurfaceType.Unknown, surface, ParametricRange.UnitSquare);
                    // A constant-radius fillet between two planes is cylindrical.
                    // Retain this construction identity for subsequent tangent-chain features.
                    if (edge.HasPlanarSourceFaces &&
                        probeMesh.groupIdToExtendedName.TryGetValue(edge.SurfaceIndexA, out var nameA) &&
                        probeMesh.groupIdToExtendedName.TryGetValue(edge.SurfaceIndexB, out var nameB) &&
                        meta.TryGetValue(nameA, out var sourceA) && meta.TryGetValue(nameB, out var sourceB) &&
                        sourceA.PlaneParams != null && sourceB.PlaneParams != null)
                    {
                        var a = sourceA.PlaneParams;
                        var b = sourceB.PlaneParams;
                        var na = a.Normal.Normalized();
                        var nb = b.Normal.Normalized();
                        double dot = Vec3DOps.Dot(na, nb);
                        double determinant = 1 - dot * dot;
                        if (determinant > 0)
                        {
                            var start = edge.CenterCurveVec3[0];
                            double offset = edge.BlendType == EdgeBlendType.Convex ? -edge.BlendRadius : edge.BlendRadius;
                            double da = offset - Vec3DOps.Dot(start - a.Origin, na);
                            double db = offset - Vec3DOps.Dot(start - b.Origin, nb);
                            var origin = start + na * ((da - dot * db) / determinant) +
                                                 nb * ((db - dot * da) / determinant);
                            var axis = Vec3DOps.Cross(na, nb).Normalized();
                            if (Vec3DOps.Dot(edge.CenterCurveVec3[^1]-start, axis) < 0) axis = -axis;
                            patch.SurfaceType = SurfaceType.Cylindrical;
                            patch.CylinderParams = new CylinderSurfaceParams {
                                Origin = origin, Axis = axis, RefDir = na, Radius = edge.BlendRadius,
                                Height = Math.Abs(Vec3DOps.Dot(edge.CenterCurveVec3[^1] - start, axis))
                            };
                        }
                    }
                    meta[name] = patch;
                }
            }

            foreach (var kv in probeMesh.extendedNameToGroupId)
            {
                if (!kv.Key.StartsWith(EntityNaming.BlendCornerPrefix, StringComparison.Ordinal))
                    continue;
                if (meta.TryGetValue(kv.Key, out var existing) && existing.PlaneParams != null)
                    continue;
                if (!probeMesh.TryGetTopologySurface(kv.Key, out var patch))
                    continue;
                if (TryBuildCornerSphere(patch, out var sphere))
                    meta[kv.Key] = new SurfaceMetaData(SurfaceType.Spherical, sphere, ParametricRange.UnitSquare);
            }
        }

        public static void AttachChamferPatchNurbs(
            Dictionary<string, SurfaceMetaData> meta,
            IReadOnlyList<BlendEdge> blendEdges,
            AnchorMesh probeMesh)
        {
            foreach (var edge in blendEdges)
            {
                if (edge?.SourceEdge == null || edge.BlendSurface == null)
                    continue;

                string name = EntityNaming.ChamferEdge(edge.SourceEdge.Name);
                if (TryBuildChamferEdgeSurface(edge, out var surface))
                    meta[name] = new SurfaceMetaData(SurfaceType.Unknown, surface, ParametricRange.UnitSquare);
            }

            foreach (var kv in probeMesh.extendedNameToGroupId)
            {
                if (!kv.Key.StartsWith(EntityNaming.ChamferCornerPrefix, StringComparison.Ordinal))
                    continue;
                if (!probeMesh.TryGetTopologySurface(kv.Key, out var patch))
                    continue;
                if (TryBuildCornerPlane(patch, out var planeParams))
                {
                    meta[kv.Key] = new SurfaceMetaData(SurfaceType.Planar, ParametricRange.UnitSquare)
                    {
                        PlaneParams = planeParams
                    };
                }
            }
        }

        /// <summary>
        /// Ruled loft between line profiles along the chamfer spine.
        /// </summary>
        public static bool TryBuildChamferEdgeSurface(BlendEdge edge, out INurbsSurface surface)
        {
            surface = null;
            if (edge.CenterCurveVec3 == null || edge.CenterCurveVec3.Count < 2)
                return false;
            if (edge.ProjectedBoundaryCurveAVec3D == null || edge.ProjectedBoundaryCurveBVec3D == null)
                return false;
            if (edge.CenterCurveVec3.Count != edge.ProjectedBoundaryCurveAVec3D.Count)
                return false;

            var profiles = new List<IReadOnlyList<Vec3D>>(edge.CenterCurveVec3.Count);
            for (int i = 0; i < edge.CenterCurveVec3.Count; i++)
            {
                profiles.Add(new List<Vec3D>
                {
                    edge.ProjectedBoundaryCurveAVec3D[i],
                    edge.ProjectedBoundaryCurveBVec3D[i]
                });
            }

            try
            {
                var loft = NurbsSurfaceFactory.LoftFromProfileSamples(profiles, LoftStyle.Ruled);
                var railPoints = edge.CenterCurveVec3;
                int degree = Math.Min(3, railPoints.Count - 1);
                var railCurve = new BSplineCurve(
                    degree,
                    railPoints.ToArray(),
                    BSplineCurve.UniformKnotVector(degree, railPoints.Count),
                    closedCurve: false);

                var lineProfile = BuildLineProfileCurve(profiles[0]);
                surface = NurbsSurfaceFactory.WrapForMeshUv(
                    loft,
                    profileCurveForArcLengthU: railCurve,
                    railCurveForArcLengthV: lineProfile,
                    swapMeshUv: true);
                return true;
            }
            catch
            {
                return false;
            }
        }

        private static BSplineCurve BuildLineProfileCurve(IReadOnlyList<Vec3D> lineEndpoints)
        {
            if (lineEndpoints.Count < 2)
                throw new ArgumentException("Line profile needs two endpoints.");
            return Curve3DToBSpline.LineToBSpline(new Curves.Line3D(lineEndpoints[0], lineEndpoints[1]));
        }

        private static bool TryBuildCornerPlane(UVSurface patch, out PlaneSurfaceParams planeParams)
        {
            planeParams = null;
            if (patch?.Points == null || patch.Points.Count < 4)
                return false;

            var boundary = ExtractPatchBoundaryPositions(patch);
            if (boundary.Count < 3)
                return false;

            (Vec3D planeNormal, Vec3D planeOrigin) = PlaneFitter.FitPlane(boundary, BuildFanTriangles(boundary.Count));
            Vec3D refDir = GetPerpendicularVector(planeNormal).Normalized();
            planeParams = new PlaneSurfaceParams
            {
                Origin = planeOrigin,
                Normal = planeNormal.Normalized(),
                RefDir = refDir
            };
            return true;
        }

        private static List<Tri> BuildFanTriangles(int vertexCount)
        {
            var tris = new List<Tri>();
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
        /// Ruled loft between exact circular-arc profiles along the blend spine (rolling-ball fillet strip).
        /// </summary>
        public static bool TryBuildBlendEdgeSurface(BlendEdge edge, out INurbsSurface surface)
        {
            surface = null;
            if (edge.CenterCurveVec3 == null || edge.CenterCurveVec3.Count < 2)
                return false;
            if (edge.ProjectedBoundaryCurveAVec3D == null || edge.ProjectedBoundaryCurveBVec3D == null)
                return false;
            if (edge.CenterCurveVec3.Count != edge.ProjectedBoundaryCurveAVec3D.Count)
                return false;

            var profiles = new List<IReadOnlyList<Vec3D>>(edge.CenterCurveVec3.Count);
            for (int i = 0; i < edge.CenterCurveVec3.Count; i++)
            {
                profiles.Add(SampleBlendArc(
                    edge.ProjectedBoundaryCurveAVec3D[i],
                    edge.ProjectedBoundaryCurveBVec3D[i],
                    edge.CenterCurveVec3[i],
                    ArcProfileSamples));
            }

            try
            {
                var loft = NurbsSurfaceFactory.LoftFromProfileSamples(profiles, LoftStyle.Ruled);
                var railPoints = edge.CenterCurveVec3;
                int degree = Math.Min(3, railPoints.Count - 1);
                var railCurve = new BSplineCurve(
                    degree,
                    railPoints.ToArray(),
                    BSplineCurve.UniformKnotVector(degree, railPoints.Count),
                    closedCurve: false);

                var arcProfile = BuildArcProfileCurve(profiles[0]);
                surface = NurbsSurfaceFactory.WrapForMeshUv(
                    loft,
                    profileCurveForArcLengthU: railCurve,
                    railCurveForArcLengthV: arcProfile,
                    swapMeshUv: true);
                return true;
            }
            catch
            {
                return false;
            }
        }

        private static List<Vec3D> SampleBlendArc(Vec3D start, Vec3D end, Vec3D center, int samples)
        {
            var dirStart = start - center;
            var dirEnd = end - center;
            var axis = Vec3DOps.Cross(dirStart, dirEnd);
            double arcAngle = Vec3DOps.Angle(dirStart, dirEnd, axis);
            if (arcAngle > Math.PI)
            {
                arcAngle = 2.0 * Math.PI - arcAngle;
                axis = -axis;
            }

            double radius = dirStart.Length();
            var dirStartNorm = dirStart.Normalized();
            var right = Vec3DOps.Cross(axis.Normalized(), dirStartNorm).Normalized();

            var points = new List<Vec3D>(samples);
            for (int j = 0; j < samples; j++)
            {
                double t = samples == 1 ? 0 : (double)j / (samples - 1);
                double a = t * arcAngle;
                points.Add(center + radius * (Math.Cos(a) * dirStartNorm + Math.Sin(a) * right));
            }

            points[0] = start;
            points[points.Count - 1] = end;
            return points;
        }

        private static BSplineCurve BuildArcProfileCurve(IReadOnlyList<Vec3D> arcSamples)
        {
            if (arcSamples.Count < 2)
                throw new ArgumentException("Arc profile needs at least two points.");
            if (arcSamples.Count == 2)
                return Curve3DToBSpline.LineToBSpline(new Curves.Line3D(arcSamples[0], arcSamples[1]));

            int mid = arcSamples.Count / 2;
            var arc = new Curves.Arc3D(arcSamples[0], arcSamples[mid], arcSamples[^1]);
            return Curve3DToBSpline.ArcToBSpline(arc);
        }

        private static bool TryBuildCornerSphere(UVSurface patch, out INurbsSurface sphere)
        {
            sphere = null;
            if (patch?.Points == null || patch.Points.Count < 4)
                return false;

            var boundary = ExtractPatchBoundaryPositions(patch);
            if (boundary.Count < 4)
                return false;

            SphereFitter.FitSphere(boundary, out var center, out var radius);
            if (radius < 1e-9)
                return false;

            sphere = new BSplineSphere(center, radius);
            return true;
        }

        private static List<Vec3D> ExtractPatchBoundaryPositions(UVSurface patch)
        {
            var edgeCount = new Dictionary<long, int>();
            var edgeVerts = new Dictionary<long, Int2>();
            foreach (var tri in patch.Triangles)
            {
                CountEdge(edgeCount, edgeVerts, tri.A, tri.B);
                CountEdge(edgeCount, edgeVerts, tri.B, tri.C);
                CountEdge(edgeCount, edgeVerts, tri.C, tri.A);
            }

            var boundary = new List<Int2>();
            foreach (var kv in edgeCount)
            {
                if (kv.Value == 1)
                    boundary.Add(edgeVerts[kv.Key]);
            }

            if (boundary.Count == 0)
                return new List<Vec3D>();

            var adjacency = new Dictionary<int, List<int>>();
            foreach (var e in boundary)
            {
                AddAdj(adjacency, e.X, e.Y);
                AddAdj(adjacency, e.Y, e.X);
            }

            var used = new HashSet<long>();
            var loop = new List<Vec3D>();
            var startEdge = boundary[0];
            int current = startEdge.X;
            int next = startEdge.Y;
            loop.Add(patch.Points[current]);

            int guard = 0;
            while (guard++ < boundary.Count + 2)
            {
                used.Add(Algorithms.Key(current, next));
                loop.Add(patch.Points[next]);
                if (next == startEdge.X && loop.Count > 2)
                    break;

                if (!adjacency.TryGetValue(next, out var neighbors))
                    break;

                int candidate = -1;
                foreach (int n in neighbors)
                {
                    long k = Algorithms.Key(next, n);
                    if (!used.Contains(k))
                    {
                        candidate = n;
                        break;
                    }
                }

                if (candidate < 0)
                    break;
                current = next;
                next = candidate;
            }

            return loop;
        }

        private static void CountEdge(Dictionary<long, int> edgeCount, Dictionary<long, Int2> edgeVerts, int a, int b)
        {
            long key = Algorithms.Key(a, b);
            if (!edgeVerts.ContainsKey(key))
                edgeVerts[key] = new Int2(a, b);
            edgeCount.TryGetValue(key, out int c);
            edgeCount[key] = c + 1;
        }

        private static void AddAdj(Dictionary<int, List<int>> adjacency, int from, int to)
        {
            if (!adjacency.TryGetValue(from, out var list))
            {
                list = new List<int>();
                adjacency[from] = list;
            }
            list.Add(to);
        }
    }
}
