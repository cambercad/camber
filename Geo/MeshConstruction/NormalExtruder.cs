using GeoCore;
using GeoMeta;

namespace Geo
{
    /// <summary>
    /// Extrudes a projected closed sketch along stored per-vertex surface normals.
    /// Caps are triangulated in sketch 2D and mapped onto the hit / offset rings.
    /// </summary>
    public static class NormalExtruder
    {
        /// <summary>
        /// Builds a watertight solid from <paramref name="projected"/>.
        /// Positive <paramref name="height"/> follows surface normals; negative goes opposite.
        /// </summary>
        public static int Generate(
            CoordinateConverter converter,
            ProjectedSketch projected,
            double height,
            MeshOutput output,
            MeshNaming naming,
            int baseGroupIndex = 0)
        {
            if (projected == null)
                throw new ArgumentNullException(nameof(projected));
            if (projected.Contours2D == null || projected.Contours2D.Count == 0)
                throw new ArgumentException("Projected sketch has no contours.");
            if (projected.Vertices == null || projected.Vertices.Count != projected.Contours2D.Count)
                throw new ArgumentException("Projected vertex strips must match Contours2D.");
            if (Math.Abs(height) < 1e-30)
                throw new ArgumentException("Extrusion height must be non-zero.");

            var contours = projected.Contours2D;
            var contourNormals = projected.ContourNormals2D ?? BuildZeroNormals(contours);
            var contourNames = projected.ContourNames;
            var projectedVerts = projected.Vertices;

            var reversalInfo = MeshConstructionHelpers.GetContourReversalInfo(contours);
            contours = MeshConstructionHelpers.ApplyReversalToContours(contours, reversalInfo);
            contourNormals = MeshConstructionHelpers.ApplyReversalToContourNormals(contourNormals, reversalInfo);
            projectedVerts = ApplyReversalToProjected(projectedVerts, reversalInfo);
            contourNames = ApplyReversalToNames(contourNames, reversalInfo);

            naming.ContourNames = contourNames;
            naming.OperationName = naming.OperationName ?? projected.Name;

            var rings = BuildRings(contours, projectedVerts, baseGroupIndex);
            int numSideContours = 0;
            foreach (var ring in rings)
                numSideContours += ring.SegmentCount;

            int bottomBase = output.Vertices.Count;
            foreach (var ring in rings)
            {
                for (int i = 0; i < ring.Hits.Count; i++)
                {
                    Vec3D p = ring.Hits[i];
                    Vec3D n = ring.Normals[i];
                    output.Vertices.Add(p);
                    output.Normals.Add(-n);
                    output.UVs.Add(ring.UVs[i]);
                    output.PrecisePositions.Add(MeshConstructionHelpers.ToPrecise(converter, p));
                }
            }

            int topBase = output.Vertices.Count;
            foreach (var ring in rings)
            {
                for (int i = 0; i < ring.Hits.Count; i++)
                {
                    Vec3D n = ring.Normals[i];
                    Vec3D p = ring.Hits[i] + height * n;
                    output.Vertices.Add(p);
                    output.Normals.Add(n);
                    output.UVs.Add(ring.UVs[i]);
                    output.PrecisePositions.Add(MeshConstructionHelpers.ToPrecise(converter, p));
                }
            }

            // Caps from sketch-2D triangulation mapped onto rings.
            var polygons2D = new List<List<Vec2D>>();
            foreach (var ring in rings)
                polygons2D.Add(new List<Vec2D>(ring.Uv2D));

            int bottomCapGroup = baseGroupIndex + numSideContours;
            int topCapGroup = baseGroupIndex + numSideContours + 1;
            EmitCapFrom2D(polygons2D, bottomBase, flipWinding: false, bottomCapGroup,
                output.Triangles, output.TriangleGroups);
            EmitCapFrom2D(polygons2D, topBase, flipWinding: true, topCapGroup,
                output.Triangles, output.TriangleGroups);

            // Side walls: one group per sketch curve segment.
            EmitSideWalls(rings, bottomBase, topBase, output.Triangles, output.TriangleGroups);

            int triangleBegin = 0;
            if (SignedVolume(output.Vertices, output.Triangles) < 0)
                FlipWindings(output.Triangles, triangleBegin);

            PopulateNaming(naming, contourNames, numSideContours, baseGroupIndex);
            return numSideContours + 2;
        }

        private static List<List<List<Vec2D>>> BuildZeroNormals(List<List<List<Vec2D>>> contours)
        {
            var result = new List<List<List<Vec2D>>>(contours.Count);
            foreach (var strip in contours)
            {
                var stripN = new List<List<Vec2D>>(strip.Count);
                foreach (var seg in strip)
                {
                    var n = new List<Vec2D>(seg.Count);
                    for (int i = 0; i < seg.Count; i++)
                        n.Add(new Vec2D(0, 0));
                    stripN.Add(n);
                }
                result.Add(stripN);
            }
            return result;
        }

        private static List<List<List<ProjectedVertex>>> ApplyReversalToProjected(
            List<List<List<ProjectedVertex>>> data, List<bool> reversalInfo)
        {
            if (data == null || reversalInfo == null)
                return data;
            var result = new List<List<List<ProjectedVertex>>>(data.Count);
            for (int i = 0; i < data.Count; i++)
            {
                var strip = data[i];
                if (i < reversalInfo.Count && reversalInfo[i])
                {
                    var rev = new List<List<ProjectedVertex>>(strip.Count);
                    for (int s = strip.Count - 1; s >= 0; s--)
                    {
                        var seg = new List<ProjectedVertex>(strip[s]);
                        seg.Reverse();
                        rev.Add(seg);
                    }
                    result.Add(rev);
                }
                else
                {
                    result.Add(strip);
                }
            }
            return result;
        }

        private static List<List<string>> ApplyReversalToNames(List<List<string>> names, List<bool> reversalInfo)
        {
            if (names == null || reversalInfo == null)
                return names;
            var result = new List<List<string>>(names.Count);
            for (int i = 0; i < names.Count; i++)
            {
                var strip = new List<string>(names[i]);
                if (i < reversalInfo.Count && reversalInfo[i])
                    strip.Reverse();
                result.Add(strip);
            }
            return result;
        }

        private class RingData
        {
            public List<Vec2D> Uv2D = new List<Vec2D>();
            public List<Vec3D> Hits = new List<Vec3D>();
            public List<Vec3D> Normals = new List<Vec3D>();
            public List<Vec2D> UVs = new List<Vec2D>();
            public List<List<int>> SegmentLocalIndices = new List<List<int>>();
            public List<int> SegmentGroupIds = new List<int>();
            public int SegmentCount;
            public int VertexBaseOffset;
        }

        private static List<RingData> BuildRings(
            List<List<List<Vec2D>>> contours,
            List<List<List<ProjectedVertex>>> projected,
            int baseGroupIndex)
        {
            var rings = new List<RingData>(contours.Count);
            int groupCursor = baseGroupIndex;
            int vertexOffset = 0;

            for (int stripIndex = 0; stripIndex < contours.Count; stripIndex++)
            {
                var strip = contours[stripIndex];
                var hitStrip = projected[stripIndex];
                if (strip.Count != hitStrip.Count)
                    throw new Exception("Projected strip segment count mismatch.");

                var ring = new RingData { VertexBaseOffset = vertexOffset, SegmentCount = strip.Count };

                for (int seg = 0; seg < strip.Count; seg++)
                {
                    var pts = strip[seg];
                    var hits = hitStrip[seg];
                    if (pts.Count != hits.Count)
                        throw new Exception("Projected segment vertex count mismatch.");
                    if (pts.Count < 2)
                        throw new Exception("Contour segment must have at least 2 points.");

                    // ContourDuplicateFree: all but last of each segment.
                    for (int i = 0; i < pts.Count - 1; i++)
                    {
                        ring.Uv2D.Add(pts[i]);
                        ring.Hits.Add(hits[i].Hit);
                        ring.Normals.Add(hits[i].SurfaceNormal.Normalized());
                        ring.UVs.Add(new Vec2D(0, 0));
                    }

                    ring.SegmentGroupIds.Add(groupCursor + seg);
                }

                // Side-wall index lists: each segment uses Contour layout (including shared end).
                // Rebuild local indices into the duplicate-free ring, wrapping the last point of
                // the last segment to the first of the next (or strip start).
                int local = 0;
                for (int seg = 0; seg < strip.Count; seg++)
                {
                    int count = strip[seg].Count;
                    var indices = new List<int>(count);
                    for (int i = 0; i < count - 1; i++)
                        indices.Add(local++);
                    // Shared end = start of next segment in the ring (or 0).
                    int nextStart = (seg + 1 < strip.Count) ? local : 0;
                    indices.Add(nextStart);
                    ring.SegmentLocalIndices.Add(indices);
                }

                // UV along ring for caps.
                for (int i = 0; i < ring.Uv2D.Count; i++)
                {
                    double u = ring.Uv2D.Count <= 1 ? 0 : (double)i / ring.Uv2D.Count;
                    ring.UVs[i] = new Vec2D(u, 0);
                }

                vertexOffset += ring.Hits.Count;
                groupCursor += strip.Count;
                rings.Add(ring);
            }

            return rings;
        }

        private static void EmitCapFrom2D(
            List<List<Vec2D>> polygons2D,
            int baseVertexIndex,
            bool flipWinding,
            int groupId,
            List<Tri> triangles,
            List<int> triangleGroups)
        {
            List<Tri> capTriangles = Triangulator.TriangulatePolygon(polygons2D);
            foreach (var tri in capTriangles)
            {
                Tri newTri;
                if (flipWinding)
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

                if (!MeshConstructionHelpers.IsDegenerateTriangle(newTri))
                {
                    triangles.Add(newTri);
                    triangleGroups.Add(groupId);
                }
            }
        }

        private static void EmitSideWalls(
            List<RingData> rings,
            int bottomBase,
            int topBase,
            List<Tri> triangles,
            List<int> triangleGroups)
        {
            foreach (var ring in rings)
            {
                for (int seg = 0; seg < ring.SegmentLocalIndices.Count; seg++)
                {
                    var indices = ring.SegmentLocalIndices[seg];
                    int groupId = ring.SegmentGroupIds[seg];
                    for (int j = 1; j < indices.Count; j++)
                    {
                        int i0 = bottomBase + ring.VertexBaseOffset + indices[j - 1];
                        int i1 = bottomBase + ring.VertexBaseOffset + indices[j];
                        int i2 = topBase + ring.VertexBaseOffset + indices[j - 1];
                        int i3 = topBase + ring.VertexBaseOffset + indices[j];

                        var tri1 = new Tri(i0, i1, i2);
                        var tri2 = new Tri(i1, i3, i2);
                        if (MeshConstructionHelpers.IsDegenerateTriangle(tri1) ||
                            MeshConstructionHelpers.IsDegenerateTriangle(tri2))
                            continue;

                        triangles.Add(tri1);
                        triangleGroups.Add(groupId);
                        triangles.Add(tri2);
                        triangleGroups.Add(groupId);
                    }
                }
            }
        }

        private static void PopulateNaming(
            MeshNaming naming,
            List<List<string>> contourNames,
            int numSideContours,
            int baseGroupIndex)
        {
            naming.TriangleGroupToName = new Dictionary<int, string>();
            int groupIndex = 0;
            for (int loopIndex = 0; loopIndex < contourNames.Count; loopIndex++)
            {
                var names = contourNames[loopIndex];
                for (int curveIndex = 0; curveIndex < names.Count; curveIndex++)
                {
                    string n = names[curveIndex];
                    EntityNaming.ValidateContourSegmentName(n);
                    naming.TriangleGroupToName.Add(
                        groupIndex + baseGroupIndex,
                        EntityNaming.ExtrudeSide(naming.OperationName, n));
                    groupIndex++;
                }
            }
            naming.TriangleGroupToName.Add(baseGroupIndex + numSideContours, EntityNaming.ExtrudeBottom(naming.OperationName));
            naming.TriangleGroupToName.Add(baseGroupIndex + numSideContours + 1, EntityNaming.ExtrudeTop(naming.OperationName));
        }

        private static double SignedVolume(List<Vec3D> vertices, List<Tri> triangles)
        {
            double vol6 = 0;
            for (int i = 0; i < triangles.Count; i++)
            {
                Tri t = triangles[i];
                vol6 += Vec3DOps.Dot(vertices[t.A], Vec3DOps.Cross(vertices[t.B], vertices[t.C]));
            }
            return vol6 / 6.0;
        }

        private static void FlipWindings(List<Tri> triangles, int start)
        {
            for (int i = start; i < triangles.Count; i++)
            {
                Tri t = triangles[i];
                triangles[i] = new Tri(t.A, t.C, t.B);
            }
        }
    }
}
