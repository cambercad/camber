using CSG;
using GeoCore;
using GeoMeta;

namespace Geo.Shelling;

/// <summary>
/// Builds a mitered offset solid from common surface supports. CSG performs trimming,
/// rim construction and intersection validation; this keeps shell topology independent
/// of the supported surface kinds.
/// </summary>
internal sealed class ShellTopologyBuilder
{
    internal sealed class OffsetSkin
    {
        internal List<Vec3D> Points { get; init; }
        internal List<Rat3Hybrid> PrecisePoints { get; init; }
        internal Dictionary<int, IShellSurfaceSupport> Supports { get; init; }
    }

    private readonly CoordinateConverter _converter;
    private readonly ShellSurfaceRegistry _surfaces;
    private readonly double _deviation;

    internal ShellTopologyBuilder(CoordinateConverter converter, ShellSurfaceRegistry surfaces, double deviation = -1)
    { _converter = converter; _surfaces = surfaces; _deviation = deviation; }

    internal AnchorMesh BuildOffset(AnchorMesh source, HashSet<int> openingGroups,
        double retainedOffset, double openingOffset, int firstGroupId, bool exterior)
    {
        if (retainedOffset > 0)
        {
            bool curvedJunction = HasCurvedJunction(source);
            if (curvedJunction)
            {
                bool convex = HasConvexPatchJunctions(source);
                if (convex && HasPlanarOpenings(source, openingGroups))
                    return BuildLayerOffset(source, openingGroups, retainedOffset, firstGroupId);
            }
        }
        // A planar support is a trim, not a constraint on the source triangle
        // rows. Offset rows can cross its final plane without the continuous
        // offset collapsing. Build with exterior temporary caps, then exact CSG
        // reconstructs the trimmed rows and caps. This also covers removed faces.
        var trimSurfaces = new Dictionary<int, UVSurface>();
        var sourceSurfaces = new Dictionary<int, UVSurface>();
        bool hasCurvedSupport = false;
        if (retainedOffset > 0)
        foreach (var (id, patch) in source.groupIdToExtendedName)
        {
            // Extending a reentrant/local face to an infinite plane would cut
            // unrelated parts of a concave body. Only global supporting planes
            // can be replaced with half-space trims.
            if (!source.TryGetTopologySurface(patch, out var surface) || surface.Triangles.Count == 0) continue;
            sourceSurfaces[id] = surface;
            if (!surface.IsSurfacePlanar()) hasCurvedSupport = true;
            else if (SupportSurfaceExtension.IsSupportingPlane(surface, source.Mesh)) trimSurfaces[id] = surface;
        }
        // Straight planar rows need no retriangulation or additional CSG work.
        if (!hasCurvedSupport) trimSurfaces.Clear();
        // A plane tangent to a curved neighbor is part of the same smooth skin,
        // not an independent cap. Moving it to an exterior temporary position
        // would make their coincident tangent constraints inconsistent.
        foreach (var edge in source.GroupEdges)
        {
            if (!trimSurfaces.ContainsKey(edge.GroupIdA) && !trimSurfaces.ContainsKey(edge.GroupIdB)) continue;
            var a = sourceSurfaces[edge.GroupIdA]; var b = sourceSurfaces[edge.GroupIdB];
            bool tangent = edge.EdgeSegments.Count > 0;
            foreach (var segment in edge.EdgeSegments)
                foreach (int vertex in new[] { segment.X, segment.Y })
                {
                    // Normals are evaluated doubles (sin(pi) is not exactly
                    // zero). Conservatively retain the original constraint for
                    // seams indistinguishable from tangency at roundoff scale;
                    // this does not snap points or relax exact trim predicates.
                    var na = a.Normals[vertex].Normalized(); var nb = b.Normals[vertex].Normalized();
                    double roundoff = 64 * Math.ScaleB(1, -52);
                    if (Vec3DOps.Cross(na, nb).LengthSquared() > roundoff * roundoff ||
                        Vec3DOps.Dot(na, nb) <= 0) tangent = false;
                }
            if (tangent) { trimSurfaces.Remove(edge.GroupIdA); trimSurfaces.Remove(edge.GroupIdB); }
        }
        double Amount(int id) => openingGroups.Contains(id) ? openingOffset : retainedOffset;
        var skin = ResolveOffsetSkin(source,
            id => trimSurfaces.ContainsKey(id) ? Math.Min(Amount(id), 0) - retainedOffset : Amount(id));
        var support = skin.Supports;

        var cavityGroups = new Dictionary<int, string>();
        var cavityMeta = new Dictionary<string, SurfaceMetaData>();
        var remap = new Dictionary<int, int>();
        int next = firstGroupId;
        foreach (var (id, patch) in source.groupIdToExtendedName)
        {
            if (!support.ContainsKey(id)) continue;
            int target = next++;
            remap[id] = target;
            // The opening cutter is intentionally private implementation detail;
            // retained supports become the public, stable cavity-face names.
            string name = openingGroups.Contains(id) ? "ShellOpening_" + patch :
                exterior ? patch : EntityNaming.ShellInner(patch);
            cavityGroups[target] = name;
            cavityMeta[name] = support[id].Metadata.Clone();
        }
        var corners = new List<MeshTriangle<TriangleVertexNormalUV>>(source.Mesh.TrianglesEx);
        for (int i = 0; i < corners.Count; i++) { var c = corners[i]; c.GroupId = remap[c.GroupId]; corners[i] = c; }
        var cavity = new MeshNormalUV {
            Positions = skin.Points, PrecisionPositions = skin.PrecisePoints,
            Triangles = new List<Tri>(source.Mesh.Triangles), TrianglesEx = corners
        };
        ValidateOffset(source.Mesh, cavity, requireClosedVolume: true);
        if (trimSurfaces.Count > 0)
        {
            foreach (var (id, surface) in trimSurfaces)
            {
                var trim = SupportSurfaceExtension.CoverPlanarMesh(
                    surface.GetOffsetSurface(-Amount(id), _converter), cavity.PrecisionPositions, _converter);
                var groups = new List<int>(trim.Triangles.Count);
                for (int i = 0; i < trim.Triangles.Count; i++) groups.Add(remap[id]);
                var cutter = new MeshNormalUV(trim.PointsPrecise, trim.Points, trim.Normals,
                    trim.Uv, trim.Triangles, groups);
                cavity = MeshNormalUV.BooleanOperation(cavity, cutter,
                    BooleanOp.AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection, _converter);
                if (support[id] is PlanarShellSupport plane)
                {
                    var metadata = plane.Metadata.Clone();
                    metadata.PlaneParams.Origin += plane.Plane.Normal *
                        (Math.Min(Amount(id), 0) - retainedOffset - Amount(id));
                    cavityMeta[cavityGroups[remap[id]]] = metadata;
                }
            }
            if (!MeshAnalysis.IsWatertightMesh(cavity.PrecisionPositions, cavity.Triangles, true))
                throw new InvalidOperationException("Shell opening trim is not watertight.");
            UpdateCylinderTrims(cavity, support, remap, cavityGroups, cavityMeta);
        }
        return new AnchorMesh(source.Name + "_shellOffset", cavity, cavityGroups, cavityMeta,
            deferCoplanarPostProcess: true, isVolume: true);
    }

    /// <summary>Resolve one signed offset skin without imposing closed-volume topology.</summary>
    internal OffsetSkin ResolveOffsetSkin(AnchorMesh source, double inwardOffset) =>
        ResolveOffsetSkin(source, _ => inwardOffset);

    private OffsetSkin ResolveOffsetSkin(AnchorMesh source, Func<int, double> offsetForGroup)
    {
        var support = CreateSupports(source, offsetForGroup);

        var incident = BuildIncidentGroups(source);
        var points = new List<Vec3D>(source.Mesh.Positions.Count);
        foreach (var vertex in Enumerable.Range(0, source.Mesh.Positions.Count))
        {
            var groups = incident[vertex];
            // Boolean meshes may retain unreferenced points. They carry no
            // topology and therefore need no offset constraint.
            if (groups.Count == 0) { points.Add(source.Mesh.Positions[vertex]); continue; }
            var supports = groups.Select(id => support[id]).ToList();
            // Each offset vertex must satisfy every incident surface support at
            // once. Averaging normals or snapping the resulting mesh can break
            // shared edges; if the supports have no valid common solution, fail
            // rather than silently changing topology or manufacturing a seam.
            if (!_surfaces.TryResolveVertex(vertex, source.Mesh.Positions[vertex], supports,
                    Math.Max(_converter.SmallestUnit() * .1, 1e-8), out var point))
                throw new ArgumentException($"Shell offset changes topology or uses an unsupported surface junction at vertex {vertex} ({source.Mesh.Positions[vertex]}): " +
                    string.Join(", ", groups.Select(id => source.groupIdToExtendedName[id] + " (" + support[id].Metadata.SurfaceType + ")")) + ".");
            points.Add(point);
        }

        foreach (var (id, item) in support)
            if (item is CylindricalShellSupport cylinder)
            {
                var vertices = Enumerable.Range(0, source.Mesh.Triangles.Count)
                    .Where(i => source.Mesh.TrianglesEx[i].GroupId == id)
                    .SelectMany(i => {
                        var t = source.Mesh.Triangles[i];
                        return new[] { t.A, t.B, t.C };
                    }).Distinct().Select(i => points[i]);
                cylinder.UpdateTrim(vertices);
            }

        var precise = new List<Rat3Hybrid>(points.Count);
        for (int i = 0; i < points.Count; i++)
        {
            var converted = _converter.Convert(points[i]);
            precise.Add(new Rat3Hybrid(converted.X, converted.Y, converted.Z));
        }
        var candidate = new MeshNormalUV {
            Positions = points, PrecisionPositions = precise,
            Triangles = new List<Tri>(source.Mesh.Triangles),
            TrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>(source.Mesh.TrianglesEx)
        };
        ValidateOffset(source.Mesh, candidate, requireClosedVolume: false);
        return new OffsetSkin { Points = points, PrecisePoints = precise, Supports = support };
    }

    private Dictionary<int, IShellSurfaceSupport> CreateSupports(AnchorMesh source, Func<int, double> offsetForGroup)
    {
        var support = new Dictionary<int, IShellSurfaceSupport>();
        double maxExtension = 0;
        foreach (int id in source.groupIdToExtendedName.Keys)
            maxExtension = Math.Max(maxExtension, Math.Abs(offsetForGroup(id)));
        foreach (var (id, patch) in source.groupIdToExtendedName)
        {
            if (!source.TryGetTopologySurface(patch, out var uv))
                throw new ArgumentException($"Shell cannot resolve topology for patch '{patch}'.");
            if (uv.Triangles.Count == 0) continue;
            SurfaceMetaData meta = null;
            source.surfaceMetaData?.TryGetValue(patch, out meta);
            meta ??= new SurfaceMetaData(SurfaceType.Unknown);
            double amount = offsetForGroup(id);
            var adapter = _surfaces.Resolve(meta, uv, amount);
            if (adapter == null)
                throw new NotSupportedException($"Shell does not support surface patch '{patch}' ({meta.SurfaceType}).");
            int triangleIndex = source.Mesh.TrianglesEx.FindIndex(t => t.GroupId == id);
            var tri = source.Mesh.Triangles[triangleIndex];
            var outward = Vec3DOps.Cross(source.Mesh.Positions[tri.B] - source.Mesh.Positions[tri.A],
                source.Mesh.Positions[tri.C] - source.Mesh.Positions[tri.A]);
            support[id] = adapter.CreateOffsetSupport(meta, uv, outward.Normalized(), amount, _converter);
            if (support[id] is MeshShellSupport meshSupport)
                meshSupport.ConfigureContinuation(-amount, _deviation, maxExtension);
        }

        return support;
    }

    private static bool HasCurvedJunction(AnchorMesh source)
    {
        foreach (var edge in source.GroupEdges)
            if (source.TryGetTopologySurface(source.groupIdToExtendedName[edge.GroupIdA], out var a) &&
                source.TryGetTopologySurface(source.groupIdToExtendedName[edge.GroupIdB], out var b) &&
                !a.IsSurfacePlanar() && !b.IsSurfacePlanar()) return true;
        return false;
    }

    private static bool HasPlanarOpenings(AnchorMesh source, HashSet<int> openings)
    {
        foreach (int id in openings)
            if (!source.TryGetTopologySurface(source.groupIdToExtendedName[id], out var surface) || !surface.IsSurfacePlanar())
                return false;
        return true;
    }

    private static bool HasConvexPatchJunctions(AnchorMesh source)
    {
        var mesh = source.Mesh;
        foreach (var (id, patch) in source.groupIdToExtendedName)
            if (source.TryGetTopologySurface(patch, out var surface) && surface.Triangles.Count > 0 &&
                surface.IsSurfacePlanar() && !SupportSurfaceExtension.IsSupportingPlane(surface, mesh)) return false;
        var map = DuplicatePointRemover.DuplicateMap(mesh.PrecisionPositions);
        var triangles = new List<Tri>(mesh.Triangles.Count);
        foreach (var t in mesh.Triangles) triangles.Add(new Tri(map[t.A], map[t.B], map[t.C]));
        var adjacency = Adjacency.BuildAdjacencyInformation(triangles, out _, skipInvalidEdges: false);
        for (int i = 0; i < triangles.Count; i++)
        {
            var t = triangles[i]; var a = mesh.PrecisionPositions[t.A];
            var normal = Rat3Hybrid.Cross(mesh.PrecisionPositions[t.B] - a, mesh.PrecisionPositions[t.C] - a);
            foreach (int neighbor in new[] { adjacency[i].NeighbourAB, adjacency[i].NeighbourBC, adjacency[i].NeighbourCA })
            {
                if (neighbor < 0) return false;
                // Curved patches retain their actual tessellated interior; no
                // facet is promoted to a global half-space. Only the junction
                // between different patches must be convex for this trimming
                // construction. Reentrant junctions need local seam treatment.
                if (mesh.TrianglesEx[i].GroupId == mesh.TrianglesEx[neighbor].GroupId) continue;
                var other = triangles[neighbor];
                foreach (int vertex in new[] { other.A, other.B, other.C })
                    if (Rat3Hybrid.Dot(normal, mesh.PrecisionPositions[vertex] - a).Sign() > 0)
                        return false;
            }
        }
        return MeshConnectivity.FindExactPositionComponents(mesh.PrecisionPositions, mesh.Triangles).Count == 1;
    }

    private AnchorMesh BuildLayerOffset(AnchorMesh source, HashSet<int> openings, double thickness, int firstGroupId)
    {
        // Remove the closed layer between each source patch and its offset.
        // An open, extended patch is not necessarily a separator of the whole
        // solid, especially after Boolean splitting. Volume subtraction has no
        // such requirement. CSG creates shared seams with exact intersections,
        // instead of dragging source triangle rows across a moving seam.
        var supports = CreateSupports(source, id => openings.Contains(id) ? 0 : thickness);
        var names = new Dictionary<int, string>();
        var metadata = new Dictionary<string, SurfaceMetaData>();
        var remap = new Dictionary<int, int>();
        int next = firstGroupId;
        foreach (var (id, patch) in source.groupIdToExtendedName)
        {
            if (!supports.ContainsKey(id)) continue;
            int target = next++; remap[id] = target;
            string name = openings.Contains(id) ? "ShellOpening_" + patch : EntityNaming.ShellInner(patch);
            names[target] = name; metadata[name] = supports[id].Metadata.Clone();
        }
        var corners = new List<MeshTriangle<TriangleVertexNormalUV>>(source.Mesh.TrianglesEx);
        for (int i = 0; i < corners.Count; i++)
        {
            var c = corners[i];
            if (openings.Contains(c.GroupId)) c.GroupId = remap[c.GroupId];
            corners[i] = c;
        }
        var cavity = new MeshNormalUV { Positions = source.Mesh.Positions, PrecisionPositions = source.Mesh.PrecisionPositions,
            Triangles = source.Mesh.Triangles, TrianglesEx = corners };
        var layers = new List<MeshNormalUV>();
        foreach (var (id, patch) in source.groupIdToExtendedName)
        {
            if (openings.Contains(id) || !supports.ContainsKey(id)) continue;
            source.TryGetTopologySurface(patch, out var surface);
            var offset = surface.GetOffsetSurface(-thickness, _converter);
            for (int i = 0; i < surface.Triangles.Count; i++)
            {
                var t = surface.Triangles[i];
                var original = Rat3Hybrid.Cross(surface.PointsPrecise[t.B] - surface.PointsPrecise[t.A],
                    surface.PointsPrecise[t.C] - surface.PointsPrecise[t.A]);
                var displaced = Rat3Hybrid.Cross(offset.PointsPrecise[t.B] - offset.PointsPrecise[t.A],
                    offset.PointsPrecise[t.C] - offset.PointsPrecise[t.A]);
                if (Rat3Hybrid.Dot(original, displaced).Sign() <= 0)
                    throw new InvalidOperationException("Shell offset collapsed or inverted a support triangle.");
            }
            layers.Add(CreateOffsetLayer(surface, offset, remap[id], GeoAPI.ReserveGroupIds(1)));
        }
        while (layers.Count > 1)
        {
            var mergedLayers = new List<MeshNormalUV>((layers.Count + 1) / 2);
            for (int i = 0; i < layers.Count; i += 2)
            {
                mergedLayers.Add(i + 1 < layers.Count
                    ? MeshNormalUV.BooleanOperation(layers[i], layers[i + 1], BooleanOp.Union, _converter)
                    : layers[i]);
            }
            layers = mergedLayers;
        }
        if (layers.Count > 0)
        {
            try
            {
                cavity = MeshNormalUV.BooleanOperation(cavity, layers[0], BooleanOp.Subtract, _converter);
            }
            catch (InvalidOperationException error)
            {
                throw new InvalidOperationException("Shell offset layer subtraction failed.", error);
            }
        }
        // The layer's side walls are NOT offset support geometry. Adjacent
        // layers must remove them, and every retained source face must move.
        // Reject incomplete coverage instead of publishing artificial seams.
        foreach (var triangle in cavity.TrianglesEx)
            if (!names.ContainsKey(triangle.GroupId))
                throw new InvalidOperationException("Shell offset layers do not cover the complete cavity.");
        var used = new HashSet<int>();
        foreach (var triangle in cavity.TrianglesEx) used.Add(triangle.GroupId);
        foreach (int id in remap.Values)
            if (!used.Contains(id)) throw new InvalidOperationException("Shell offset collapsed a source patch; the requested thickness changes topology.");
        bool watertight = MeshAnalysis.IsWatertightMesh(cavity.PrecisionPositions, cavity.Triangles, true);
        bool positive = MeshAnalysis.ComputeSignedMeshVolume(cavity.Positions, cavity.Triangles) > 0;
        int components = MeshConnectivity.FindExactPositionComponents(cavity.PrecisionPositions, cavity.Triangles).Count;
        if (!watertight || !positive || components != 1)
            throw new InvalidOperationException("Shell trimmed offset collapsed or is not watertight.");
        UpdateCylinderTrims(cavity, supports, remap, names, metadata);
        return new AnchorMesh(source.Name + "_shellOffset", cavity, names, metadata, deferCoplanarPostProcess: true, isVolume: true);
    }

    private MeshNormalUV CreateOffsetLayer(UVSurface source, UVSurface offset, int innerGroup, int sideGroup)
    {
        int count = offset.Points.Count;
        var precise = new List<Rat3Hybrid>(offset.PointsPrecise);
        precise.AddRange(source.PointsPrecise);
        var normals = new List<Vec3D>(count * 2);
        foreach (var normal in offset.Normals) normals.Add(-normal);
        normals.AddRange(source.Normals);
        var uv = new List<Vec2D>(offset.Uv); uv.AddRange(source.Uv);
        var triangles = new List<Tri>(); var groups = new List<int>();
        foreach (var t in offset.Triangles)
        {
            triangles.Add(new Tri(t.C, t.B, t.A)); groups.Add(innerGroup);
            triangles.Add(new Tri(t.A + count, t.B + count, t.C + count)); groups.Add(sideGroup);
        }
        foreach (var edge in Adjacency.BuildEdgeList(offset.Triangles))
        {
            if (!edge.IsOnBorder) continue;
            int face = edge.NeighbourIndex1 >= 0 ? edge.NeighbourIndex1 : edge.NeighbourIndex2;
            var triangle = offset.Triangles[face];
            int a = edge.Start, b = edge.End;
            if ((triangle.IndexOf(a) + 1) % 3 != triangle.IndexOf(b)) (a, b) = (b, a);
            triangles.Add(new Tri(a, b, b + count)); groups.Add(sideGroup);
            triangles.Add(new Tri(a, b + count, a + count)); groups.Add(sideGroup);
        }
        if (!MeshAnalysis.IsWatertightMesh(precise, triangles, true))
            throw new InvalidOperationException("Shell offset layer is not watertight.");
        return new MeshNormalUV(precise, _converter.Convert(precise), normals, uv, triangles, groups);
    }

    private static void UpdateCylinderTrims(MeshNormalUV mesh, Dictionary<int, IShellSurfaceSupport> supports,
        Dictionary<int, int> remap, Dictionary<int, string> names, Dictionary<string, SurfaceMetaData> metadata)
    {
        foreach (var (id, support) in supports)
            if (support is CylindricalShellSupport cylinder)
            {
                var vertices = new List<Vec3D>();
                for (int i = 0; i < mesh.Triangles.Count; i++)
                    if (mesh.TrianglesEx[i].GroupId == remap[id])
                    {
                        var triangle = mesh.Triangles[i];
                        vertices.Add(mesh.Positions[triangle.A]);
                        vertices.Add(mesh.Positions[triangle.B]);
                        vertices.Add(mesh.Positions[triangle.C]);
                    }
                cylinder.UpdateTrim(vertices);
                metadata[names[remap[id]]] = cylinder.Metadata.Clone();
            }
    }

    private static Dictionary<int, List<int>> BuildIncidentGroups(AnchorMesh mesh)
    {
        var exactMap = DuplicatePointRemover.DuplicateMap(mesh.Mesh.PrecisionPositions);
        var byExactPoint = new Dictionary<int, HashSet<int>>();
        for (int i = 0; i < mesh.Mesh.Positions.Count; i++)
            if (!byExactPoint.ContainsKey(exactMap[i])) byExactPoint.Add(exactMap[i], new HashSet<int>());
        for (int i = 0; i < mesh.Mesh.Triangles.Count; i++)
        {
            var t = mesh.Mesh.Triangles[i]; var g = mesh.Mesh.TrianglesEx[i].GroupId;
            byExactPoint[exactMap[t.A]].Add(g);
            byExactPoint[exactMap[t.B]].Add(g);
            byExactPoint[exactMap[t.C]].Add(g);
        }
        var answer = new Dictionary<int, List<int>>();
        for (int i = 0; i < mesh.Mesh.Positions.Count; i++)
        {
            var groups = byExactPoint[exactMap[i]].ToList();
            groups.Sort();
            answer.Add(i, groups);
        }
        return answer;
    }

    internal static void ValidateOffset(MeshNormalUV source, MeshNormalUV cavity, bool requireClosedVolume)
    {
        for (int i = 0; i < cavity.Triangles.Count; i++)
        {
            var t = cavity.Triangles[i];
            // CSG intersections can create exact vertices closer than double
            // spacing. Approximate display points must not decide collapse or
            // winding when authoritative rational positions are available.
            var offsetNormal = Rat3Hybrid.Cross(cavity.PrecisionPositions[t.B] - cavity.PrecisionPositions[t.A],
                cavity.PrecisionPositions[t.C] - cavity.PrecisionPositions[t.A]);
            if (offsetNormal.IsZero())
                throw new InvalidOperationException($"Shell offset collapsed triangle {i}.");
            var s = source.Triangles[i];
            var sourceNormal = Rat3Hybrid.Cross(source.PrecisionPositions[s.B] - source.PrecisionPositions[s.A],
                source.PrecisionPositions[s.C] - source.PrecisionPositions[s.A]);
            if (Rat3Hybrid.Dot(sourceNormal, offsetNormal).Sign() <= 0)
                throw new InvalidOperationException($"Shell offset inverted triangle {i}; the requested thickness changes topology.");
        }
        if (requireClosedVolume && (!MeshAnalysis.IsWatertightMesh(cavity.PrecisionPositions, cavity.Triangles, true) ||
            MeshAnalysis.CompareSignedMeshVolume(cavity.PrecisionPositions, cavity.Triangles, new(0)) <= 0))
            throw new InvalidOperationException("Shell offset collapsed or changed topology.");
    }

}
