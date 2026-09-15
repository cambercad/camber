using GeoCore;
using GeoMeta;
using System;

namespace Geo
{
    public static class Revolver
    {
        /// <summary>
        /// Single-profile convenience overload. Creates a default converter internally.
        /// </summary>
        public static int GenerateRevolvedMesh(List<Vec2D> profile, double angle,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, double maxError, int baseGroupIndex = 0)
        {
            if (angle <= 0 || angle > 2 * Math.PI)
                throw new ArgumentException("Angle must be between 0 and 2π");

            var profiles = new List<List<Vec2D>> { profile };
            var profileNormals = new List<List<Vec2D>> { Extruder.WeightedNormals2D(profile) };
            var converter = MakeDefaultConverter(new List<List<List<Vec2D>>> { profiles });
            var precisePositions = new List<Rat3Hybrid>();

            return GenerateRevolvedMeshCore(converter,
                new List<List<List<Vec2D>>> { profiles },
                new List<List<List<Vec2D>>> { profileNormals },
                angle, triangles, vertices, normals, uv, triangleGroups, precisePositions,
                maxError, baseGroupIndex, null);
        }

        /// <summary>
        /// Multi-profile overload without converter or naming (single outer contour, multiple segments).
        /// </summary>
        public static int GenerateRevolvedMesh(List<List<Vec2D>> profiles, List<List<Vec2D>> profileNormals, double angle,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, double maxError, int baseGroupIndex = 0)
        {
            var converter = MakeDefaultConverter(new List<List<List<Vec2D>>> { profiles });
            var precisePositions = new List<Rat3Hybrid>();
            return GenerateRevolvedMeshCore(converter,
                new List<List<List<Vec2D>>> { profiles },
                new List<List<List<Vec2D>>> { profileNormals },
                angle, triangles, vertices, normals, uv, triangleGroups, precisePositions,
                maxError, baseGroupIndex, null);
        }

        /// <summary>
        /// Named multi-profile overload without converter. Populates triangleGroupToName.
        /// </summary>
        public static int GenerateRevolvedMesh(List<List<Vec2D>> profiles, List<List<Vec2D>> profileNormals, double angle,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv, List<int> triangleGroups, double maxError,
            List<string> profileNames, string revolveName, out Dictionary<int, string> triangleGroupToName, int baseGroupIndex = 0)
        {
            var converter = MakeDefaultConverter(new List<List<List<Vec2D>>> { profiles });
            var precisePositions = new List<Rat3Hybrid>();
            return GenerateRevolvedMesh(converter, profiles, profileNormals, angle,
                triangles, vertices, normals, uv, triangleGroups, precisePositions, maxError,
                profileNames, revolveName, out triangleGroupToName, null, baseGroupIndex);
        }

        /// <summary>
        /// Named multi-profile overload with converter + precise positions (single contour).
        /// </summary>
        public static int GenerateRevolvedMesh(CoordinateConverter converter,
            List<List<Vec2D>> profiles, List<List<Vec2D>> profileNormals, double angle,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, List<Rat3Hybrid> precisePositions, double maxError,
            List<string> profileNames, string revolveName, out Dictionary<int, string> triangleGroupToName,
            CoordinateSystem? sketchFrame = null, int baseGroupIndex = 0)
        {
            return GenerateRevolvedMesh(converter,
                new List<List<List<Vec2D>>> { profiles },
                new List<List<List<Vec2D>>> { profileNormals },
                angle, triangles, vertices, normals, uv, triangleGroups, precisePositions, maxError,
                new List<List<string>> { profileNames }, revolveName, out triangleGroupToName, sketchFrame, baseGroupIndex);
        }

        /// <summary>
        /// MeshOutput overload with converter (single contour). Bundles all outputs into MeshOutput.
        /// </summary>
        public static int GenerateRevolvedMesh(CoordinateConverter converter,
            List<List<Vec2D>> profiles, List<List<Vec2D>> profileNormals, double angle,
            MeshOutput output, double maxError,
            List<string> profileNames, string revolveName, out Dictionary<int, string> triangleGroupToName,
            CoordinateSystem? sketchFrame = null, int baseGroupIndex = 0)
        {
            return GenerateRevolvedMesh(converter,
                new List<List<List<Vec2D>>> { profiles },
                new List<List<List<Vec2D>>> { profileNormals },
                angle, output, maxError,
                new List<List<string>> { profileNames }, revolveName, out triangleGroupToName, sketchFrame, baseGroupIndex);
        }

        /// <summary>
        /// Multi-contour overload (outer + nested holes). Same contour layout as <see cref="Extruder"/>.
        /// </summary>
        public static int GenerateRevolvedMesh(CoordinateConverter converter,
            List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals, double angle,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, List<Rat3Hybrid> precisePositions, double maxError,
            List<List<string>> contourNames, string revolveName, out Dictionary<int, string> triangleGroupToName,
            CoordinateSystem? sketchFrame = null, int baseGroupIndex = 0)
        {
            var res = GenerateRevolvedMeshCore(converter, contours, contourNormals, angle,
                triangles, vertices, normals, uv, triangleGroups, precisePositions,
                maxError, baseGroupIndex, sketchFrame);
            triangleGroupToName = PopulateRevolveNaming(contourNames, angle, revolveName, baseGroupIndex);
            return res;
        }

        /// <summary>
        /// Multi-contour MeshOutput overload (outer + nested holes).
        /// </summary>
        public static int GenerateRevolvedMesh(CoordinateConverter converter,
            List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals, double angle,
            MeshOutput output, double maxError,
            List<List<string>> contourNames, string revolveName, out Dictionary<int, string> triangleGroupToName,
            CoordinateSystem? sketchFrame = null, int baseGroupIndex = 0)
        {
            var res = GenerateRevolvedMeshCore(converter, contours, contourNormals, angle,
                output.Triangles, output.Vertices, output.Normals, output.UVs,
                output.TriangleGroups, output.PrecisePositions,
                maxError, baseGroupIndex, sketchFrame);
            triangleGroupToName = PopulateRevolveNaming(contourNames, angle, revolveName, baseGroupIndex);
            return res;
        }

        private static Dictionary<int, string> PopulateRevolveNaming(
            List<List<string>> contourNames, double angle, string revolveName, int baseGroupIndex)
        {
            var map = new Dictionary<int, string>();
            int groupIndex = 0;
            if (contourNames != null)
            {
                for (int loop = 0; loop < contourNames.Count; loop++)
                {
                    var names = contourNames[loop];
                    for (int i = 0; i < names.Count; i++)
                        map.Add(baseGroupIndex + groupIndex++, EntityNaming.RevolveSide(revolveName, names[i]));
                }
            }

            bool isFullRevolution = Math.Abs(angle - 2 * Math.PI) < 1e-10;
            if (!isFullRevolution)
            {
                map.Add(baseGroupIndex + groupIndex, EntityNaming.RevolveStartCap(revolveName));
                map.Add(baseGroupIndex + groupIndex + 1, EntityNaming.RevolveEndCap(revolveName));
            }
            return map;
        }

        /// <summary>
        /// Maps revolver local space (profile X = revolution axis, Y = radius in the plane of rotation)
        /// into world space, matching <see cref="Extruder"/> sketch placement. Identity frame is a no-op.
        /// </summary>
        private static void TransformRevolveOutputToWorld(CoordinateConverter converter, CoordinateSystem sketchFrame,
            List<Vec3D> vertices, List<Vec3D> normals, List<Rat3Hybrid> precisePositions)
        {
            for (int i = 0; i < vertices.Count; i++)
                vertices[i] = sketchFrame.PointFromCoordSysToWorld(vertices[i]);
            for (int i = 0; i < normals.Count; i++)
                normals[i] = sketchFrame.DirectionFromCoordSysToWorld(normals[i]);

            precisePositions.Clear();
            for (int i = 0; i < vertices.Count; i++)
                precisePositions.Add(MeshConstructionHelpers.ToPrecise(converter, vertices[i]));
        }

        /// <summary>
        /// Core implementation. Requires a converter and precisePositions list.
        /// Cap triangulation uses principal-plane projection with no accuracy loss.
        /// Contours: first = outer, subsequent = nested holes (same layout as Extruder).
        /// </summary>
        private static int GenerateRevolvedMeshCore(CoordinateConverter converter,
            List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals, double angle,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            double maxError, int baseGroupIndex = 0, CoordinateSystem? sketchFrame = null)
        {
            if (angle <= 0 || angle > 2 * Math.PI)
                throw new ArgumentException("Angle must be between 0 and 2π");
            if (contours == null || contours.Count == 0)
                throw new ArgumentException("At least one contour is required");

            bool isOpenContour = DetectAndSnapOpenContour(contours);
            if (isOpenContour && contours.Count > 1)
                throw new ArgumentException("Nested holes require closed contours; open revolve profiles cannot include holes.");

            // Weld strip joints within each contour.
            for (int c = 0; c < contours.Count; c++)
            {
                var profiles = contours[c];
                for (int i = 0; i < profiles.Count - 1; i++)
                    profiles[i][profiles[i].Count - 1] = profiles[i + 1][0];
                if (!isOpenContour)
                    profiles[profiles.Count - 1][profiles[profiles.Count - 1].Count - 1] = profiles[0][0];
            }

            MeshConstructionHelpers.OrientRevolveContours(ref contours, ref contourNormals, isOpenContour);

            double maxY = MaxY(contours);
            int numSamples = NumRevolveSamples(maxY, maxError);
            bool isFullRevolution = Math.Abs(angle - 2 * Math.PI) < 1e-10;

            int sideGroupCount = 0;
            for (int c = 0; c < contours.Count; c++)
            {
                EmitRevolveSideWalls(converter, contours[c], contourNormals[c], angle, numSamples, isFullRevolution,
                    triangles, vertices, normals, uv, triangleGroups, precisePositions,
                    baseGroupIndex + sideGroupCount);
                sideGroupCount += contours[c].Count;
            }

            int groupCount;
            if (!isFullRevolution)
            {
                GenerateRevolveCaps(converter, contours, angle,
                    vertices, normals, uv, triangles, triangleGroups, precisePositions,
                    baseGroupIndex, sideGroupCount);
                groupCount = sideGroupCount + 2;
            }
            else
                groupCount = sideGroupCount;

            TransformRevolveOutputToWorld(converter, sketchFrame ?? CoordinateSystem.Default,
                vertices, normals, precisePositions);

            EnsurePositiveSignedVolume(vertices, normals, triangles);

            return groupCount;
        }

        /// <summary>
        /// Closed off-axis profiles can emerge with inverted winding; flip tris/normals if needed
        /// so solids have positive signed volume (matches <see cref="AnchorMesh"/> invariants).
        /// </summary>
        private static void EnsurePositiveSignedVolume(List<Vec3D> vertices, List<Vec3D> normals, List<Tri> triangles)
        {
            if (triangles.Count == 0)
                return;
            if (MeshAnalysis.ComputeSignedMeshVolume(vertices, triangles) >= 0)
                return;

            for (int i = 0; i < triangles.Count; i++)
            {
                Tri t = triangles[i];
                triangles[i] = new Tri(t.A, t.C, t.B);
            }
            for (int i = 0; i < normals.Count; i++)
                normals[i] = -normals[i];
        }

        /// <summary>
        /// True when the (single) contour is open; snaps near-axis endpoints to Y=0.
        /// </summary>
        private static bool DetectAndSnapOpenContour(List<List<List<Vec2D>>> contours)
        {
            if (contours.Count != 1)
                return false;

            var profiles = contours[0];
            Vec2D globalStart = profiles[0][0];
            Vec2D globalEnd = profiles[profiles.Count - 1][profiles[profiles.Count - 1].Count - 1];
            bool isOpenContour = Vec2DOps.DistanceSquared(globalStart, globalEnd) > 1e-10 * 1e-10;
            if (!isOpenContour)
                return false;

            const double axisTolerance = 1e-6;
            var firstProfile = profiles[0];
            var lastProfile = profiles[profiles.Count - 1];
            if (Math.Abs(firstProfile[0].Y) <= axisTolerance)
                firstProfile[0] = new Vec2D(firstProfile[0].X, 0.0);
            if (Math.Abs(lastProfile[lastProfile.Count - 1].Y) <= axisTolerance)
                lastProfile[lastProfile.Count - 1] = new Vec2D(lastProfile[lastProfile.Count - 1].X, 0.0);
            return true;
        }

        private static void EmitRevolveSideWalls(
            CoordinateConverter converter,
            List<List<Vec2D>> profiles, List<List<Vec2D>> profileNormals,
            double angle, int numSamples, bool isFullRevolution,
            List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            int baseGroupIndex)
        {
            int actualSamples = numSamples + 1;

            double totalProfileLength = 0;
            LineStrip2D[] strips = new LineStrip2D[profiles.Count];
            for (int profileIdx = 0; profileIdx < profiles.Count; profileIdx++)
            {
                strips[profileIdx] = new LineStrip2D(profiles[profileIdx]);
                totalProfileLength += strips[profileIdx].TotalLength;
            }

            int[][,] vertexIndices = new int[profiles.Count][,];
            bool[][] segIsOnAxis = new bool[profiles.Count][];
            Vec3D[][] segAxisPos = new Vec3D[profiles.Count][];
            Vec3D[][] segAxisNrm = new Vec3D[profiles.Count][];
            double[][] segU = new double[profiles.Count][];

            double cumulativeLength = 0;
            for (int profileIdx = 0; profileIdx < profiles.Count; profileIdx++)
            {
                var profile = profiles[profileIdx];
                var normals2D = profileNormals[profileIdx];
                int ptCount = profile.Count;
                LineStrip2D strip = strips[profileIdx];

                vertexIndices[profileIdx] = new int[actualSamples, ptCount];
                segIsOnAxis[profileIdx] = new bool[ptCount];
                segAxisPos[profileIdx] = new Vec3D[ptCount];
                segAxisNrm[profileIdx] = new Vec3D[ptCount];
                segU[profileIdx] = new double[ptCount];

                for (int i = 0; i < ptCount; i++)
                {
                    segU[profileIdx][i] = (cumulativeLength + strip.GetDistanceFromBuffer(i)) / totalProfileLength;
                    segIsOnAxis[profileIdx][i] = profile[i].Y == 0;
                    if (segIsOnAxis[profileIdx][i])
                    {
                        segAxisPos[profileIdx][i] = new Vec3D(profile[i].X, 0, 0);
                        segAxisNrm[profileIdx][i] = new Vec3D(normals2D[i].X, normals2D[i].Y, 0).Normalized();
                    }
                }

                cumulativeLength += strip.TotalLength;

                for (int sample = 0; sample < actualSamples; sample++)
                {
                    int angleSample = (isFullRevolution && sample == numSamples) ? 0 : sample;
                    double currentAngle = (double)angleSample / numSamples * angle;
                    double cosAngle = Math.Cos(currentAngle);
                    double sinAngle = Math.Sin(currentAngle);
                    double v = (double)sample / numSamples;

                    for (int i = 0; i < ptCount; i++)
                    {
                        if (segIsOnAxis[profileIdx][i])
                        {
                            vertexIndices[profileIdx][sample, i] = -1;
                        }
                        else
                        {
                            Vec2D point = profile[i];
                            Vec2D normal2D = normals2D[i];
                            Vec3D vertex3D = new Vec3D(point.X, point.Y * cosAngle, point.Y * sinAngle);
                            Vec3D normal3D = new Vec3D(normal2D.X, normal2D.Y * cosAngle, normal2D.Y * sinAngle);

                            vertexIndices[profileIdx][sample, i] = vertices.Count;
                            vertices.Add(vertex3D);
                            normals.Add(normal3D);
                            uv.Add(new Vec2D(segU[profileIdx][i], v));
                            precisePositions.Add(MeshConstructionHelpers.ToPrecise(converter, vertex3D));
                        }
                    }
                }
            }

            for (int profileIdx = 0; profileIdx < profiles.Count; profileIdx++)
            {
                var profile = profiles[profileIdx];
                int ptCount = profile.Count;

                for (int sample = 0; sample < numSamples; sample++)
                {
                    int nextSample = sample + 1;

                    for (int j = 0; j < ptCount - 1; j++)
                    {
                        int j1 = j + 1;
                        bool axis0 = segIsOnAxis[profileIdx][j];
                        bool axis1 = segIsOnAxis[profileIdx][j1];

                        if (axis0 && axis1)
                            continue;

                        int v0 = vertexIndices[profileIdx][sample, j];
                        int v1 = vertexIndices[profileIdx][sample, j1];
                        int v2 = vertexIndices[profileIdx][nextSample, j];
                        int v3 = vertexIndices[profileIdx][nextSample, j1];

                        if (axis0)
                        {
                            double midV = ((double)sample + 0.5) / numSamples;
                            int poleVert = vertices.Count;
                            vertices.Add(segAxisPos[profileIdx][j]);
                            normals.Add(segAxisNrm[profileIdx][j]);
                            uv.Add(new Vec2D(segU[profileIdx][j], midV));
                            precisePositions.Add(MeshConstructionHelpers.ToPrecise(converter, segAxisPos[profileIdx][j]));
                            triangles.Add(new Tri(poleVert, v1, v3));
                            triangleGroups.Add(baseGroupIndex + profileIdx);
                        }
                        else if (axis1)
                        {
                            double midV = ((double)sample + 0.5) / numSamples;
                            int poleVert = vertices.Count;
                            vertices.Add(segAxisPos[profileIdx][j1]);
                            normals.Add(segAxisNrm[profileIdx][j1]);
                            uv.Add(new Vec2D(segU[profileIdx][j1], midV));
                            precisePositions.Add(MeshConstructionHelpers.ToPrecise(converter, segAxisPos[profileIdx][j1]));
                            triangles.Add(new Tri(v0, poleVert, v2));
                            triangleGroups.Add(baseGroupIndex + profileIdx);
                        }
                        else
                        {
                            triangles.Add(new Tri(v0, v1, v2));
                            triangleGroups.Add(baseGroupIndex + profileIdx);
                            triangles.Add(new Tri(v1, v3, v2));
                            triangleGroups.Add(baseGroupIndex + profileIdx);
                        }
                    }
                }
            }
        }

        private static int NumRevolveSamples(double radius, double maxError)
        {
            if (radius <= 0 || maxError <= 0)
                return 4;

            double theta = 2.0 * Math.Acos(1.0 - maxError / radius);
            int numSamples = (int)Math.Ceiling(2.0 * Math.PI / theta);
            numSamples = Math.Max(4, numSamples);
            // Round up to a multiple of 4 so full-revolve 0/90/180/270 lie on vertices.
            return ((numSamples + 3) / 4) * 4;
        }

        /// <summary>
        /// Generates start and end caps for a partial revolve via principal-plane projection.
        /// Outer boundary first; nested hole rings follow (same triangulation path as Extruder).
        /// </summary>
        private static void GenerateRevolveCaps(
            CoordinateConverter converter,
            List<List<List<Vec2D>>> contours, double angle,
            List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<Tri> triangles, List<int> triangleGroups, List<Rat3Hybrid> precisePositions,
            int baseGroupIndex, int sideGroupCount)
        {
            int startCapGroupId = baseGroupIndex + sideGroupCount;
            int endCapGroupId = baseGroupIndex + sideGroupCount + 1;

            var flatRings = BuildCapRings(contours);
            var ringCounts = new int[flatRings.Count];
            for (int i = 0; i < flatRings.Count; i++)
                ringCounts[i] = flatRings[i].Count;
            // Cap vertices are emitted starting at index 0 of the local cap list; TriangulateAndEmitCap
            // offsets by baseVertexIndex, so ring indices are relative to the cap vertex block.
            var capPolygonIndices = MeshConstructionHelpers.BuildConsecutiveRingIndices(ringCounts, 0);

            Vec3D startCapNormal = new Vec3D(0, 0, -1);
            GenerateSingleCap(converter, capPolygonIndices, flatRings, 0.0, startCapNormal, startCapGroupId, false,
                vertices, normals, uv, triangles, triangleGroups, precisePositions);

            double cosA = Math.Cos(angle);
            double sinA = Math.Sin(angle);
            Vec3D endCapNormal = new Vec3D(0, -sinA, cosA);
            GenerateSingleCap(converter, capPolygonIndices, flatRings, angle, endCapNormal, endCapGroupId, true,
                vertices, normals, uv, triangles, triangleGroups, precisePositions);
        }

        /// <summary>
        /// Flattened duplicate-free rings for cap triangulation (outer first, then holes).
        /// </summary>
        private static List<List<Vec2D>> BuildCapRings(List<List<List<Vec2D>>> contours)
        {
            var rings = new List<List<Vec2D>>(contours.Count);
            for (int c = 0; c < contours.Count; c++)
            {
                var flat = FlattenProfiles(contours[c]);
                if (flat.Count > 1 &&
                    Vec2DOps.DistanceSquared(flat[0], flat[flat.Count - 1]) < 1e-20)
                {
                    flat.RemoveAt(flat.Count - 1);
                }
                rings.Add(flat);
            }
            return rings;
        }

        private static List<Vec2D> FlattenProfiles(List<List<Vec2D>> profiles)
        {
            var flat = new List<Vec2D>();
            for (int i = 0; i < profiles.Count; i++)
            {
                int startJ = (i == 0) ? 0 : 1;
                for (int j = startJ; j < profiles[i].Count; j++)
                    flat.Add(profiles[i][j]);
            }
            return flat;
        }

        /// <summary>
        /// Generates a single cap for a partial revolve. Emits cap vertices with precise
        /// positions, then triangulates using principal-plane projection (no accuracy loss).
        /// </summary>
        private static void GenerateSingleCap(
            CoordinateConverter converter,
            List<List<int>> capPolygonIndices, List<List<Vec2D>> flatRings,
            double angle, Vec3D capNormal, int groupId, bool flipWinding,
            List<Vec3D> vertices, List<Vec3D> normals, List<Vec2D> uv,
            List<Tri> triangles, List<int> triangleGroups, List<Rat3Hybrid> precisePositions)
        {
            double cosAngle = Math.Cos(angle);
            double sinAngle = Math.Sin(angle);

            int baseVertexIndex = vertices.Count;

            double minX = double.MaxValue, maxX = double.MinValue;
            double minY = double.MaxValue, maxY = double.MinValue;
            foreach (var ring in flatRings)
            {
                foreach (var pt in ring)
                {
                    if (pt.X < minX) minX = pt.X;
                    if (pt.X > maxX) maxX = pt.X;
                    if (pt.Y < minY) minY = pt.Y;
                    if (pt.Y > maxY) maxY = pt.Y;
                }
            }
            double size = Math.Max(maxX - minX, maxY - minY);
            double scaling = size > 1e-10 ? 1.0 / size : 1.0;

            var capPrecise = new List<Rat3Hybrid>();
            foreach (var ring in flatRings)
            {
                foreach (var pt in ring)
                {
                    Vec3D vertex3D = new Vec3D(pt.X, pt.Y * cosAngle, pt.Y * sinAngle);
                    vertices.Add(vertex3D);
                    normals.Add(capNormal);
                    uv.Add(new Vec2D((pt.X - minX) * scaling, (pt.Y - minY) * scaling));

                    var precise = MeshConstructionHelpers.ToPrecise(converter, vertex3D);
                    capPrecise.Add(precise);
                    precisePositions.Add(precise);
                }
            }

            MeshConstructionHelpers.TriangulateAndEmitCap(
                capPrecise, capPolygonIndices, baseVertexIndex, flipWinding,
                groupId, triangles, triangleGroups);
        }

        private static CoordinateConverter MakeDefaultConverter(List<List<List<Vec2D>>> contours)
        {
            double maxVal = 0;
            foreach (var contour in contours)
                foreach (var profile in contour)
                    foreach (var pt in profile)
                        maxVal = Math.Max(maxVal, Math.Max(Math.Abs(pt.X), Math.Abs(pt.Y)));

            double extent = maxVal * 1.5 + 1.0;
            return new CoordinateConverter(new Box3D(new Vec3D(-extent), new Vec3D(extent)));
        }

        private static double MaxY(List<List<List<Vec2D>>> contours)
        {
            double maxY = 0;
            foreach (var contour in contours)
                foreach (var profile in contour)
                    foreach (var point in profile)
                        maxY = Math.Max(maxY, Math.Abs(point.Y));
            return maxY;
        }
    }
}
