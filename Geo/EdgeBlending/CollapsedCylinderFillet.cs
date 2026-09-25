using CSG;
using GeoCore;
using NURBS;

namespace Geo;

/// <summary>Face replacement at the zero-spine-radius limit of a closed cylindrical rim.</summary>
internal static class CollapsedCylinderFillet
{
    internal static bool TryCreate(AnchorMesh mesh, GraphEdge edge, double radius,
        CoordinateConverter converter, double maxDeviation, ref int groupIdOffset,
        out MeshNormalUV result, out SurfaceMetaData metadata, Func<int, int> allocateGroupIds = null)
    {
        result = null; metadata = null;
        if (edge.LineStripExact[0] != edge.LineStripExact[^1]) return false;
        var source = FindCylinderCap(mesh, edge, radius, maxDeviation, out radius);
        if (source == null) return false;
        var (cylinderId, planeId, cylinder, plane, wall, cap, pole) = source;
        // This branch consumes a disk face. Additional incident boundaries in
        // the consumed region require a multi-face corner construction.
        var rim = edge.EdgeSegments.Select(e => Algorithms.Key(e.X, e.Y)).ToHashSet();
        if (!BoundaryEdges(cap).SetEquals(rim)) return false;
        MeshNormalUV retainedWall = null;
        List<Rat3Hybrid> boundary;
        bool hasBoundaryCenter = false;
        var boundaryCenter = new Vec3D(0);
        // At H=R the entire cylindrical wall disappears. Use the existing
        // opposite disk boundary directly; no offset surface remains to intersect.
        if (!TryGetOppositeDiskBoundary(mesh, source, radius, rim, out boundary))
        {
            var offsetCap = OffsetPlanarCap(cap, pole, radius, converter);
            if (TryGetWallBoundaryOnPlane(wall, rim, offsetCap, maxDeviation, converter,
                out boundary, out boundaryCenter))
                hasBoundaryCenter = true;
            else
            {
                var face = offsetCap.Triangles[0];
                var planePoint = offsetCap.PointsPrecise[face.A];
                var planeNormal = Rat3Hybrid.Cross(offsetCap.PointsPrecise[face.B] - planePoint,
                    offsetCap.PointsPrecise[face.C] - planePoint);
                foreach (long key in BoundaryEdges(wall).Except(rim))
                {
                    Algorithms.DecomposeKey(key, out int first, out int second);
                    if (Rat3Hybrid.Dot(planeNormal, wall.PointsPrecise[first] - planePoint) > BigRationalHybrid.Zero ||
                       Rat3Hybrid.Dot(planeNormal, wall.PointsPrecise[second] - planePoint) > BigRationalHybrid.Zero)
                        return false;
                }
                boundary = ContactCurve(wall, offsetCap, converter);
                var wallMesh = BlendEdge.ToMesh(wall, converter, cylinderId);
                var planeMesh = BlendEdge.ToMesh(offsetCap, converter);
                retainedWall = MeshNormalUV.BooleanOperation(wallMesh, planeMesh,
                    BooleanOp.AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection, converter);
            }
        }
        if (boundary[0] != boundary[^1]) throw new InvalidOperationException("The spherical fillet contact circle is open.");
        boundary.RemoveAt(boundary.Count - 1);
        var points = converter.Convert(boundary);
        double axialDistance = Vec3DOps.Dot(plane.Origin - cylinder.Origin, pole) - radius;
        var center = hasBoundaryCenter ? boundaryCenter : cylinder.Origin + pole * axialDistance;
        var area = new Vec3D(0);
        for (int i = 0; i < points.Count; i++) area += Vec3DOps.Cross(points[i] - center, points[(i + 1) % points.Count] - center);
        if (Vec3DOps.Dot(area, pole) < 0) { boundary.Reverse(); points.Reverse(); }
        var patch = BlendCorner.TessellateSphereCap(points, maxDeviation, boundary, new List<int> { 0 },
            converter, EdgeBlendType.Convex, center, radius, pole);
        int patchGroup = allocateGroupIds?.Invoke(1) ?? groupIdOffset;
        groupIdOffset = checked(patchGroup + 1);
        var sphereMesh = BlendEdge.ToMesh(patch, converter, patchGroup);
        // The end disk vanishes to a pole. Sew its replacement along the exact
        // circle instead of asking a crossing-only trim to classify point contact.
        var positions = new List<Rat3Hybrid>(mesh.Mesh.PrecisionPositions);
        var triangles = new List<Tri>();
        var corners = new List<MeshTriangle<TriangleVertexNormalUV>>();
        var groups = new List<int>();
        for (int i = 0; i < mesh.Mesh.Triangles.Count; i++)
        {
            var corner = mesh.Mesh.TrianglesEx[i];
            if (corner.GroupId == planeId || corner.GroupId == cylinderId) continue;
            triangles.Add(mesh.Mesh.Triangles[i]); corners.Add(corner); groups.Add(corner.GroupId);
        }
        foreach (var surface in new[] { retainedWall, sphereMesh }.Where(surface => surface != null))
        {
            int shift = positions.Count; positions.AddRange(surface.PrecisionPositions);
            for (int i = 0; i < surface.Triangles.Count; i++)
            {
                var t = surface.Triangles[i]; var corner = surface.TrianglesEx[i];
                triangles.Add(new Tri(t.A + shift, t.B + shift, t.C + shift)); corners.Add(corner); groups.Add(corner.GroupId);
            }
        }
        result = new MeshNormalUV(converter, positions, triangles, corners, groups);
        metadata = new SurfaceMetaData(SurfaceType.Spherical, new BSplineSphere(center, radius), ParametricRange.UnitSquare)
        { SphereParams = new SphereSurfaceParams { Center = center, Radius = radius, Axis = pole, RefDir = cylinder.RefDir } };
        return true;
    }
    internal static bool TryGetSectorContact(AnchorMesh mesh, GraphEdge edge, double radius,
        CoordinateConverter converter, out int cylinderId, out List<Rat3Hybrid> contact)
    {
        cylinderId = -1; contact = null;
        if (edge.LineStripExact[0] == edge.LineStripExact[^1]) return false;
        var source = FindCylinderCap(mesh, edge, radius, 0, out _);
        if (source == null) return false;
        cylinderId = source.CylinderId;
        var offsetCap = OffsetPlanarCap(source.Cap, source.Pole, radius, converter);
        contact = ContactCurve(source.Wall, offsetCap, converter);
        return true;
    }

    private sealed record CylinderCap(int CylinderId, int PlaneId, CylinderSurfaceParams Cylinder,
        PlaneSurfaceParams Plane, UVSurface Wall, UVSurface Cap, Vec3D Pole);

    private static CylinderCap FindCylinderCap(AnchorMesh mesh, GraphEdge edge, double requestedRadius,
        double maxDeviation, out double effectiveRadius)
    {
        effectiveRadius = requestedRadius;
        if (edge.BlendType != EdgeBlendType.Convex) return null;
        if (!mesh.surfaceMetaData.TryGetValue(mesh.groupIdToExtendedName[edge.GroupIdA], out var a) ||
            !mesh.surfaceMetaData.TryGetValue(mesh.groupIdToExtendedName[edge.GroupIdB], out var b)) return null;
        var cylinder = a.CylinderParams ?? b.CylinderParams;
        var plane = a.PlaneParams ?? b.PlaneParams;
        if (cylinder == null || plane == null) return null;
        double collapseGap = cylinder.Radius - requestedRadius;
        if (collapseGap < 0 || collapseGap > maxDeviation) return null;
        // A toroidal rim fillet tends to a hemisphere as its major radius
        // (cylinder.Radius - requestedRadius) tends to zero. Within the
        // requested surface deviation, replacing that tiny torus by its
        // analytic hemispherical limit avoids intersecting a sub-tessellation
        // offset cylinder. In a cross-section, corresponding arc points differ
        // by at most this gap, so this remains within maxDeviation.
        effectiveRadius = cylinder.Radius;
        int cylinderId = a.CylinderParams != null ? edge.GroupIdA : edge.GroupIdB;
        int planeId = a.PlaneParams != null ? edge.GroupIdA : edge.GroupIdB;
        if (!mesh.TryGetTopologySurface(mesh.groupIdToExtendedName[cylinderId], out var wall) ||
            !mesh.TryGetTopologySurface(mesh.groupIdToExtendedName[planeId], out var cap) || cap.Triangles.Count == 0) return null;
        // Use exact face winding, not a potentially smoothed boundary vertex normal.
        var triangle = cap.Triangles[0];
        var aPoint = cap.PointsPrecise[triangle.A];
        var faceNormal = Rat3Hybrid.Cross(cap.PointsPrecise[triangle.B] - aPoint,
            cap.PointsPrecise[triangle.C] - aPoint);
        var pole = new Vec3D(faceNormal.X.ToDouble(), faceNormal.Y.ToDouble(), faceNormal.Z.ToDouble()).Normalized();
        if (Vec3DOps.Cross(pole, cylinder.Axis.Normalized()).LengthSquared() > 1e-24) return null;
        return new CylinderCap(cylinderId, planeId, cylinder, plane, wall, cap, pole);
    }

    private static UVSurface OffsetPlanarCap(UVSurface cap, Vec3D normal, double radius,
        CoordinateConverter converter)
    {
        var flatNormals = Enumerable.Repeat(normal, cap.Points.Count).ToList();
        var planarCap = new UVSurface(cap.Points, flatNormals, cap.Uv, cap.Triangles, cap.PointsPrecise);
        return planarCap.GetExtendedSurface(2 * radius, converter, out _).GetOffsetSurface(-radius, converter);
    }

    private static bool TryGetWallBoundaryOnPlane(UVSurface wall, HashSet<long> rim,
        UVSurface plane, double maxDeviation, CoordinateConverter converter,
        out List<Rat3Hybrid> boundary, out Vec3D center)
    {
        boundary = null;
        center = new Vec3D(0);
        var remaining = BoundaryEdges(wall);
        remaining.ExceptWith(rim);
        if (remaining.Count == 0) return false;
        var segments = remaining.Select(key =>
        {
            Algorithms.DecomposeKey(key, out int first, out int second);
            return new Int2(first, second);
        }).ToList();
        var loops = SegmentConnector.Connect(segments, edge => edge.X, edge => edge.Y,
            (first, second) => first == second, out var closed);
        if (loops.Count != 1 || !closed[0]) return false;

        var loop = new List<Rat3Hybrid>(loops[0].Count + 1);
        foreach (int id in loops[0])
        {
            int vertex = SegmentConnector.GetStart(segments, id, edge => edge.X, edge => edge.Y);
            loop.Add(wall.PointsPrecise[vertex]);
        }
        if (loop.Count < 3) return false;

        var normal = Rat3Hybrid.Cross(loop[1] - loop[0], loop[2] - loop[0]);
        if (normal.IsZero()) return false;
        var planeFace = plane.Triangles[0];
        var planePoint = plane.PointsPrecise[planeFace.A];
        var planeNormal = Rat3Hybrid.Cross(plane.PointsPrecise[planeFace.B] - planePoint,
            plane.PointsPrecise[planeFace.C] - planePoint);
        if (!Rat3Hybrid.Cross(normal, planeNormal).IsZero()) return false;
        foreach (var point in loop)
            if (Rat3Hybrid.Dot(normal, point - loop[0]) != BigRationalHybrid.Zero) return false;

        var wallPoint = converter.Convert(new List<Rat3Hybrid> { loop[0] })[0];
        var supportPoint = converter.Convert(new List<Rat3Hybrid> { planePoint })[0];
        var normalWorld = new Vec3D(planeNormal.X.ToDouble(), planeNormal.Y.ToDouble(), planeNormal.Z.ToDouble()).Normalized();
        if (Math.Abs(Vec3DOps.Dot(normalWorld, wallPoint - supportPoint)) > maxDeviation) return false;

        center = wallPoint;
        loop.Add(loop[0]);
        boundary = loop;
        return true;
    }

    private static bool TryGetOppositeDiskBoundary(AnchorMesh mesh, CylinderCap source, double radius,
        HashSet<long> rim, out List<Rat3Hybrid> boundary)
    {
        boundary = null;
        if (source.Cylinder.Height != radius) return false;
        // TryGetSurface retains the parent's complete vertex arrays and triangle
        // indices. Wall, cap and disk IDs therefore refer to this same AnchorMesh.
        var remaining = BoundaryEdges(source.Wall);
        remaining.ExceptWith(rim);
        foreach (var pair in mesh.groupIdToExtendedName)
        {
            if (pair.Key == source.PlaneId || !mesh.surfaceMetaData.TryGetValue(pair.Value, out var meta) ||
                meta.PlaneParams == null) continue;
            double station = Vec3DOps.Dot(meta.PlaneParams.Origin - source.Cylinder.Origin, source.Cylinder.Axis.Normalized());
            if (Math.Min(Math.Abs(station), Math.Abs(station - radius)) > 1e-12 * Math.Max(1, radius)) continue;
            mesh.TryGetTopologySurface(pair.Value, out var disk);
            if (!BoundaryEdges(disk).SetEquals(remaining)) continue;
            var segments = remaining.Select(key => { Algorithms.DecomposeKey(key, out int first, out int second); return new Int2(first, second); }).ToList();
            var loops = SegmentConnector.Connect(segments, edge => edge.X, edge => edge.Y,
                (first, second) => first == second, out var closed);
            if (loops.Count != 1 || !closed[0]) return false;
            boundary = loops[0].Select(id => source.Wall.PointsPrecise[
                SegmentConnector.GetStart(segments, id, edge => edge.X, edge => edge.Y)]).ToList();
            boundary.Add(boundary[0]);
            return true;
        }
        return false;
    }

    private static List<Rat3Hybrid> ContactCurve(UVSurface wall, UVSurface offsetCap, CoordinateConverter converter)
    {
        var (segments, _) = Intersector.IntersectSurfaces(wall, offsetCap, converter);
        if (segments.Count == 0) throw new InvalidOperationException("The spherical fillet has no cylinder contact curve.");
        var contact = new List<Rat3Hybrid> { segments[0].PointStart };
        foreach (var segment in segments)
        {
            if (segment.PointStart != contact[^1]) throw new InvalidOperationException("The spherical fillet requires one connected contact curve.");
            contact.Add(segment.PointEnd);
        }
        return contact;
    }

    private static HashSet<long> BoundaryEdges(UVSurface surface)
    {
        var counts = new Dictionary<long, int>();
        foreach (var triangle in surface.Triangles)
            foreach (var pair in new[] { new Int2(triangle.A, triangle.B), new Int2(triangle.B, triangle.C), new Int2(triangle.C, triangle.A) })
            {
                long key = Algorithms.Key(pair.X, pair.Y);
                counts.TryGetValue(key, out int count);
                counts[key] = count + 1;
            }
        return counts.Where(p => p.Value == 1).Select(p => p.Key).ToHashSet();
    }

}
