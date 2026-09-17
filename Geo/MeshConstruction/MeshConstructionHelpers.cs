using GeoCore;

namespace Geo
{
    /// <summary>
    /// Shared mesh construction utilities: 2D contour strip orientation (extrude / revolve, nested polygons),
    /// cap triangulation helpers, and degeneracy checks.
    /// </summary>
    public static class MeshConstructionHelpers
    {
        /// <summary>
        /// Converts a Vec3D position to a precise Rat3Hybrid via the integer grid.
        /// </summary>
        public static Rat3Hybrid ToPrecise(CoordinateConverter converter, Vec3D v)
        {
            Int3 intPos = converter.Convert(v);
            return new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z);
        }

        public static bool IsDegenerateTriangle(Tri tri) =>
            tri.A == tri.B || tri.A == tri.C || tri.B == tri.C;

        /// <summary>
        /// Squared Euclidean norm of <c>(b−a)×(c−a)</c>. Zero for colinear or duplicate vertices (zero-area triangle in 3D).
        /// </summary>
        public static double SquaredTriangleCrossNorm(Vec3D a, Vec3D b, Vec3D c)
        {
            var ab = b - a;
            var ac = c - a;
            var cr = Vec3DOps.Cross(ab, ac);
            return Vec3DOps.Dot(cr, cr);
        }

        /// <summary>
        /// True if the triangle has repeated vertex indices or 3D area below a tolerance (squared cross norm threshold).
        /// </summary>
        public static bool IsDegenerateTriangleMesh(Tri tri, Vec3D a, Vec3D b, Vec3D c, double minSquaredCrossNorm)
        {
            if (IsDegenerateTriangle(tri))
                return true;
            if (minSquaredCrossNorm <= 0)
                return false;
            return SquaredTriangleCrossNorm(a, b, c) < minSquaredCrossNorm;
        }

        /// <summary>Maximum side length of the axis-aligned bounding box of <paramref name="positions"/>.</summary>
        public static double MeshAxisAlignedMaxExtent(IReadOnlyList<Vec3D> positions)
        {
            if (positions == null || positions.Count == 0)
                return 1.0;
            double minX = double.MaxValue, minY = minX, minZ = minX;
            double maxX = double.MinValue, maxY = maxX, maxZ = maxX;
            for (int i = 0; i < positions.Count; i++)
            {
                Vec3D p = positions[i];
                if (p.X < minX) minX = p.X;
                if (p.Y < minY) minY = p.Y;
                if (p.Z < minZ) minZ = p.Z;
                if (p.X > maxX) maxX = p.X;
                if (p.Y > maxY) maxY = p.Y;
                if (p.Z > maxZ) maxZ = p.Z;
            }

            double dx = maxX - minX;
            double dy = maxY - minY;
            double dz = maxZ - minZ;
            double L = Math.Max(dx, Math.Max(dy, dz));
            return L < 1e-30 ? 1.0 : L;
        }

        /// <summary>
        /// Minimum ‖(b−a)×(c−a)‖² used when emitting loft side/cap triangles; matches <c>L⁴·1e-22</c> with floor 1e-30.
        /// </summary>
        public static double LoftMinSquaredCrossNormFromMaxExtent(double maxExtentL) =>
            Math.Max(1e-30, maxExtentL * maxExtentL * maxExtentL * maxExtentL * 1e-22);

        /// <summary>
        /// Orients revolution meridian strips (X = axis, Y = radius in the sketch plane) so side quads use
        /// ∂s×∂θ with ∂θ along +Z at θ=0 when Y&gt;0 (see <see cref="Revolver"/>), matching outward normals for outers
        /// and the opposite sense for nested holes.
        /// </summary>
        public static void OrientRevolveProfileStrips(ref List<List<Vec2D>> profileStrips, ref List<List<Vec2D>> profileNormalStrips, bool openContour = false)
        {
            if (profileStrips == null || profileStrips.Count == 0)
                return;

            var wrapP = new List<List<List<Vec2D>>> { profileStrips };
            List<List<List<Vec2D>>> wrapN = profileNormalStrips != null && profileNormalStrips.Count > 0
                ? new List<List<List<Vec2D>>> { profileNormalStrips }
                : null;
            OrientRevolveContours(ref wrapP, ref wrapN, openContour);
            profileStrips = wrapP[0];
            if (wrapN != null)
                profileNormalStrips = wrapN[0];
        }

        /// <summary>
        /// Orients all revolve contours together so nesting depth (outer vs hole) is respected.
        /// </summary>
        public static void OrientRevolveContours(
            ref List<List<List<Vec2D>>> contours,
            ref List<List<List<Vec2D>>> contourNormals,
            bool openContour = false)
        {
            if (contours == null || contours.Count == 0)
                return;

            var reversalInfo = GetRevolveContourReversalInfo(contours, openContour);
            contours = ApplyReversalToContours(contours, reversalInfo);
            if (contourNormals != null && contourNormals.Count > 0)
                contourNormals = ApplyReversalToContourNormals(contourNormals, reversalInfo);
        }

        /// <summary>
        /// Builds consecutive vertex-index rings for cap triangulation (outer first, then holes).
        /// Shared by <see cref="Extruder"/> and <see cref="Revolver"/>.
        /// </summary>
        public static List<List<int>> BuildConsecutiveRingIndices(IReadOnlyList<int> ringVertexCounts, int startVertexIndex = 0)
        {
            var polygonIndices = new List<List<int>>(ringVertexCounts?.Count ?? 0);
            if (ringVertexCounts == null || ringVertexCounts.Count == 0)
                return polygonIndices;

            int current = startVertexIndex;
            for (int r = 0; r < ringVertexCounts.Count; r++)
            {
                int count = ringVertexCounts[r];
                var ring = new List<int>(count);
                for (int i = 0; i < count; i++)
                    ring.Add(current++);
                polygonIndices.Add(ring);
            }
            return polygonIndices;
        }

        /// <summary>
        /// Reversal flags for revolution side strips. Closed meridians use contour nesting
        /// and signed radius; open meridians use radial orientation at their axis closure.
        /// </summary>
        public static List<bool> GetRevolveContourReversalInfo(List<List<List<Vec2D>>> contours, bool openContour = false)
        {
            if (contours == null || contours.Count == 0)
                return new List<bool>();

            if (!openContour)
            {
                // A closed meridian's material side follows its nesting depth.
                // A local radial normal cannot classify an off-axis closed loop:
                // opposite sides of a torus point toward and away from the axis.
                var closedReversals = GetContourReversalInfo(contours, outerBoundaryTargetIsCCW: true);
                var closedPolygons = ExtractContourPolygons(contours);
                for (int i = 0; i < closedPolygons.Count; ++i)
                {
                    // Negative signed radii reverse the revolution Jacobian.
                    var radialPoint = closedPolygons[i].FirstOrDefault(p => p.Y != 0);
                    if (radialPoint.Y < 0) closedReversals[i] = !closedReversals[i];
                }
                return closedReversals;
            }

            var result = new List<bool>(contours.Count);
            for (int i = 0; i < contours.Count; i++)
                result.Add(false);

            List<List<Vec2D>> allPolygons = ExtractContourPolygons(contours, openContour);
            if (allPolygons.Count == 0)
                return result;

            if (allPolygons.Count == 1)
            {
                if (!TryGetRevolveStripReversal(allPolygons[0], isHole: false, out bool rev, openContour))
                {
                    if (openContour)
                        return result;
                    return GetContourReversalInfo(contours, outerBoundaryTargetIsCCW: false);
                }
                result[0] = rev;
                return result;
            }

            var polyTrees = PolygonOps<Vec2DArithmeticNoPredicates, Vec2D, double>.BuildPolyTree(allPolygons);
            var depthByPoly = new List<int>(allPolygons.Count);
            for (int i = 0; i < allPolygons.Count; i++)
                depthByPoly.Add(-1);
            foreach (PolygonTree root in polyTrees)
                AssignPolyTreeDepth(root, depthByPoly, 0);

            List<bool> fallback = GetContourReversalInfo(contours, outerBoundaryTargetIsCCW: false);

            for (int i = 0; i < allPolygons.Count; i++)
            {
                int d = depthByPoly[i] >= 0 ? depthByPoly[i] : 0;
                bool isHole = (d % 2) == 1;
                if (!TryGetRevolveStripReversal(allPolygons[i], isHole, out bool rev, openContour))
                    rev = fallback[i];
                // Nested holes: invert the strip-reversal decision so outer/hole keep opposite
                // winding (cavity), analogous to Extrude's CCW-outer / CW-hole relationship.
                result[i] = isHole ? !rev : rev;
            }

            return result;
        }

        private static List<List<Vec2D>> ExtractContourPolygons(List<List<List<Vec2D>>> contours, bool openContour = false)
        {
            var allPolygons = new List<List<Vec2D>>();
            for (int polyIndex = 0; polyIndex < contours.Count; polyIndex++)
            {
                List<List<Vec2D>> contour = contours[polyIndex];
                var duplicateFreePoly = new List<Vec2D>();

                if (openContour)
                {
                    AppendOpenContourStrips(contour, duplicateFreePoly);
                }
                else
                {
                    if (contour.Count == 0)
                    {
                        allPolygons.Add(duplicateFreePoly);
                        continue;
                    }

                    Vec2D prevEnd = contour[contour.Count - 1].Last();
                    foreach (List<Vec2D> curvePoints in contour)
                    {
                        if (curvePoints[0] != prevEnd)
                            throw new Exception("Curve strip is not connected");

                        for (int i = 0; i < curvePoints.Count - 1; ++i)
                            duplicateFreePoly.Add(curvePoints[i]);

                        prevEnd = curvePoints.Last();
                    }
                }

                allPolygons.Add(duplicateFreePoly);
            }

            return allPolygons;
        }

        private static void AppendOpenContourStrips(List<List<Vec2D>> contour, List<Vec2D> duplicateFreePoly)
        {
            Vec2D prevEnd = default;
            bool firstStrip = true;
            foreach (List<Vec2D> curvePoints in contour)
            {
                if (curvePoints == null || curvePoints.Count == 0)
                    continue;

                if (!firstStrip && curvePoints[0] != prevEnd)
                    throw new Exception("Curve strip is not connected");

                for (int i = 0; i < curvePoints.Count - 1; ++i)
                    duplicateFreePoly.Add(curvePoints[i]);

                prevEnd = curvePoints.Last();
                firstStrip = false;
            }
        }

        /// <summary>
        /// Uses a meridian edge with significant ∂s×∂θ · radial; ∂θ aligns with (0,0,sign(Y)) at θ=0.
        /// </summary>
        private static bool TryGetRevolveStripReversal(List<Vec2D> poly, bool isHole, out bool reverse, bool openPolyline = false)
        {
            reverse = false;
            const double axisEps = 1e-9;
            const double minDot = 1e-14;
            int n = poly.Count;
            if (n < 2)
                return false;

            double bestAbsDot = minDot;
            double bestDot = 0;

            int edgeCount = openPolyline ? n - 1 : n;
            for (int i = 0; i < edgeCount; i++)
            {
                Vec2D a = poly[i];
                Vec2D b = poly[openPolyline ? i + 1 : (i + 1) % n];
                double midY = 0.5 * (a.Y + b.Y);
                if (Math.Abs(midY) < axisEps)
                    continue;
                if (Math.Max(Math.Abs(a.Y), Math.Abs(b.Y)) < axisEps)
                    continue;

                Vec2D e = b - a;
                double el = e.Length();
                if (el < 1e-14)
                    continue;

                double tx = e.X / el;
                double ty = e.Y / el;
                Vec3D t3 = new Vec3D(tx, ty, 0);
                double ps = Math.Sign(midY);
                Vec3D dTheta = new Vec3D(0, 0, ps);
                Vec3D nSurf = t3.Cross(dTheta);
                Vec3D radial = isHole ? new Vec3D(0, -ps, 0) : new Vec3D(0, ps, 0);
                double dot = nSurf.Dot(radial);
                double ad = Math.Abs(dot);
                if (ad > bestAbsDot)
                {
                    bestAbsDot = ad;
                    bestDot = dot;
                }
            }

            if (bestAbsDot <= minDot)
                return false;

            reverse = bestDot < 0;
            return true;
        }

        private static void AssignPolyTreeDepth(PolygonTree node, List<int> depthByPolyId, int depth)
        {
            depthByPolyId[node.PolyId] = depth;
            foreach (PolygonTree child in node.Children)
                AssignPolyTreeDepth(child, depthByPolyId, depth + 1);
        }

        /// <summary>
        /// Which top-level contour polygons (each a chain of curve strips) must be reversed so winding matches the target.
        /// Extrusion: outer boundaries CCW in (X,Y) from +Z; use <see cref="GetRevolveContourReversalInfo"/> for revolve side orientation.
        /// </summary>
        /// <param name="outerBoundaryTargetIsCCW">True for extrusion (default); false for CW outer in the sketch plane.</param>
        public static List<bool> GetContourReversalInfo(List<List<List<Vec2D>>> contours, bool outerBoundaryTargetIsCCW = true)
        {
            if (contours == null || contours.Count == 0)
                return new List<bool>();

            var result = new List<bool>(contours.Count);
            for (int i = 0; i < contours.Count; i++)
                result.Add(false);

            List<List<Vec2D>> allPolygons = ExtractContourPolygons(contours);

            if (allPolygons.Count == 0)
                return result;

            if (allPolygons.Count == 1)
            {
                bool currentIsCCW = Polygon.IsPolygonCCW(allPolygons[0]);
                result[0] = currentIsCCW != outerBoundaryTargetIsCCW;
                return result;
            }

            var polyTrees = PolygonOps<Vec2DArithmeticNoPredicates, Vec2D, double>.BuildPolyTree(allPolygons);
            var shouldBeCCW = new List<bool>(allPolygons.Count);
            for (int i = 0; i < allPolygons.Count; ++i)
                shouldBeCCW.Add(true);
            foreach (var tree in polyTrees)
                MarkContourOrientationTargets(tree, shouldBeCCW, outerBoundaryTargetIsCCW);

            for (int polygonIndex = 0; polygonIndex < allPolygons.Count; polygonIndex++)
            {
                var polygon = allPolygons[polygonIndex];
                bool currentIsCCW = Polygon.IsPolygonCCW(polygon);
                bool targetIsCCW = (polygonIndex < shouldBeCCW.Count) ? shouldBeCCW[polygonIndex] : outerBoundaryTargetIsCCW;
                result[polygonIndex] = currentIsCCW != targetIsCCW;
            }

            return result;
        }

        /// <summary>
        /// Applies <see cref="GetContourReversalInfo"/> to contour point strips or matching normal strips.
        /// </summary>
        public static List<List<List<Vec2D>>> ApplyReversalToContours(List<List<List<Vec2D>>> contours, List<bool> reversalInfo)
        {
            if (contours == null || contours.Count == 0)
                return contours ?? new List<List<List<Vec2D>>>();

            var result = new List<List<List<Vec2D>>>();

            for (int loopIndex = 0; loopIndex < contours.Count; loopIndex++)
            {
                var loop = contours[loopIndex];
                var resultLoop = new List<List<Vec2D>>();

                for (int segmentIndex = 0; segmentIndex < loop.Count; segmentIndex++)
                {
                    var segment = loop[segmentIndex];
                    if (segment == null)
                    {
                        resultLoop.Add(new List<Vec2D>());
                        continue;
                    }

                    resultLoop.Add(new List<Vec2D>(segment));
                }

                bool reverse = loopIndex < reversalInfo.Count && reversalInfo[loopIndex];
                if (reverse)
                {
                    foreach (List<Vec2D> segment in resultLoop)
                        segment.Reverse();
                    resultLoop.Reverse();
                }

                result.Add(resultLoop);
            }

            return result;
        }

        /// <summary>
        /// Applies contour reversal to matching normal strips. Reversing a curve changes
        /// both sample order and tangent direction, so its right-hand normal must be negated.
        /// </summary>
        public static List<List<List<Vec2D>>> ApplyReversalToContourNormals(
            List<List<List<Vec2D>>> contourNormals, List<bool> reversalInfo)
        {
            var result = ApplyReversalToContours(contourNormals, reversalInfo);
            for (int loopIndex = 0; loopIndex < result.Count; loopIndex++)
            {
                if (loopIndex >= reversalInfo.Count || !reversalInfo[loopIndex])
                    continue;
                foreach (List<Vec2D> segment in result[loopIndex])
                {
                    for (int i = 0; i < segment.Count; i++)
                        segment[i] = -segment[i];
                }
            }
            return result;
        }

        /// <summary>
        /// Ensures each normal strip points to the right of its matching directed
        /// contour segment, which is outward for extrusion's CCW outer loops.
        /// </summary>
        public static void OrientContourNormalsToContours(
            List<List<List<Vec2D>>> contours, List<List<List<Vec2D>>> contourNormals)
        {
            if (contours == null || contourNormals == null)
                return;
            int loopCount = Math.Min(contours.Count, contourNormals.Count);
            for (int loopIndex = 0; loopIndex < loopCount; loopIndex++)
            {
                int segmentCount = Math.Min(contours[loopIndex].Count, contourNormals[loopIndex].Count);
                for (int segmentIndex = 0; segmentIndex < segmentCount; segmentIndex++)
                {
                    List<Vec2D> points = contours[loopIndex][segmentIndex];
                    List<Vec2D> normals = contourNormals[loopIndex][segmentIndex];
                    int edgeCount = Math.Min(points.Count - 1, normals.Count);
                    double alignment = 0;
                    for (int i = 0; i < edgeCount; i++)
                    {
                        Vec2D edge = points[i + 1] - points[i];
                        Vec2D right = new Vec2D(edge.Y, -edge.X);
                        alignment += right.Dot(normals[i]);
                    }
                    if (alignment < 0)
                    {
                        for (int i = 0; i < normals.Count; i++)
                            normals[i] = -normals[i];
                    }
                }
            }
        }

        private static void MarkContourOrientationTargets(PolygonTree tree, List<bool> shouldBeCCW, bool targetCCW)
        {
            shouldBeCCW[tree.PolyId] = targetCCW;
            foreach (var child in tree.Children)
                MarkContourOrientationTargets(child, shouldBeCCW, !targetCCW);
        }

        /// <summary>
        /// Determines the principal projection plane for a set of 3D polygon vertices.
        /// Selects the largest projected signed area of the outer polygon
        /// for 2D triangulation. Also computes whether the projection flips winding.
        /// </summary>
        public static void DeterminePrincipalPlane(
            List<Rat3Hybrid> precisePositions,
            List<List<int>> polygonIndices,
            out int axis1,
            out int axis2,
            out bool flipWinding)
        {
            // Use exact projected areas, not extents or floating approximations:
            // thin tilted caps and huge rational coordinates must retain a valid
            // projection. Input ring order is arbitrary (a hole may come first).
            // The largest absolute ring area belongs to an outer boundary, since
            // every hole has smaller area than its enclosing ring in this plane.
            axis1 = 0;
            axis2 = 1;
            flipWinding = false;
            var largestArea = BigRationalHybrid.Zero;
            foreach (var ring in polygonIndices)
            {
                var areaYZ = BigRationalHybrid.Zero;
                var areaXZ = BigRationalHybrid.Zero;
                var areaXY = BigRationalHybrid.Zero;
                for (int i = 0; i < ring.Count; i++)
                {
                    var p = precisePositions[ring[i]];
                    var q = precisePositions[ring[(i + 1) % ring.Count]];
                    areaYZ += p.Y * q.Z - q.Y * p.Z;
                    areaXZ += p.X * q.Z - q.X * p.Z;
                    areaXY += p.X * q.Y - q.X * p.Y;
                }
                Consider(areaYZ, 1, 2, ref largestArea, ref axis1, ref axis2, ref flipWinding);
                Consider(areaXZ, 0, 2, ref largestArea, ref axis1, ref axis2, ref flipWinding);
                Consider(areaXY, 0, 1, ref largestArea, ref axis1, ref axis2, ref flipWinding);
            }

            static void Consider(BigRationalHybrid area, int a, int b,
                ref BigRationalHybrid largest, ref int axis1, ref int axis2, ref bool flip)
            {
                bool negative = area < BigRationalHybrid.Zero;
                var magnitude = negative ? -area : area;
                if (magnitude <= largest)
                    return;
                largest = magnitude;
                axis1 = a;
                axis2 = b;
                flip = negative;
            }
        }


        /// <summary>
        /// Projects precise 3D positions onto two chosen axes, producing Rat2Hybrid polygons.
        /// </summary>
        public static List<List<Rat2Hybrid>> ExtractRat2HybridWithPrincipalPlane(
            List<Rat3Hybrid> precisePositions,
            List<List<int>> polygonIndices,
            int axis1,
            int axis2)
        {
            var result = new List<List<Rat2Hybrid>>(polygonIndices.Count);

            foreach (var polygon in polygonIndices)
            {
                var convertedPolygon = new List<Rat2Hybrid>(polygon.Count);
                foreach (var vertexIndex in polygon)
                {
                    var pt = precisePositions[vertexIndex];
                    BigRationalHybrid coord1 = (axis1 == 0) ? pt.X : (axis1 == 1) ? pt.Y : pt.Z;
                    BigRationalHybrid coord2 = (axis2 == 0) ? pt.X : (axis2 == 1) ? pt.Y : pt.Z;
                    convertedPolygon.Add(new Rat2Hybrid(coord1, coord2));
                }
                result.Add(convertedPolygon);
            }

            return result;
        }

        /// <summary>
        /// Triangulates a polygon described by precise positions + index rings, then emits
        /// the resulting triangles (offset by baseVertexIndex) into the output lists.
        /// Uses principal-plane projection to choose the best 2D plane.
        /// </summary>
        public static void TriangulateAndEmitCap(
            List<Rat3Hybrid> capPrecise,
            List<List<int>> polygonIndices,
            int baseVertexIndex,
            bool flipWinding,
            int groupId,
            List<Tri> triangles,
            List<int> triangleGroups)
        {
            DeterminePrincipalPlane(capPrecise, polygonIndices, out int axis1, out int axis2, out bool projFlip);
            var polygons2D = ExtractRat2HybridWithPrincipalPlane(capPrecise, polygonIndices, axis1, axis2);
            List<Tri> capTriangles = Triangulator.TriangulatePolygon(polygons2D);

            bool actualFlip = flipWinding ^ projFlip;

            foreach (var tri in capTriangles)
            {
                Tri newTri;
                if (actualFlip)
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

                if (!IsDegenerateTriangle(newTri))
                {
                    triangles.Add(newTri);
                    triangleGroups.Add(groupId);
                }
            }
        }
    }
}
