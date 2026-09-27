using GeoCore;
using GeoMeta;

namespace Geo.Surfaces;

internal static class SurfaceSewing
{
    private readonly record struct EdgeUse(int Surface, bool Forward);

    internal static AnchorMesh Sew(GeoAPI api, IReadOnlyList<AnchorMesh> surfaces,
        bool makeSolid, string name)
    {
        if (surfaces == null || surfaces.Count == 0)
            throw new ArgumentException("Sew requires at least one surface.", nameof(surfaces));

        var exactToGlobal = new Dictionary<Rat3Hybrid, int>();
        var exact = new List<Rat3Hybrid>();
        var positions = new List<Vec3D>();
        var mappedTriangles = new List<List<Tri>>(surfaces.Count);
        var usedGroupsBySurface = new List<HashSet<int>>(surfaces.Count);
        var edgeUses = new Dictionary<(int, int), List<EdgeUse>>();
        for (int surfaceIndex = 0; surfaceIndex < surfaces.Count; surfaceIndex++)
        {
            AnchorMesh surface = surfaces[surfaceIndex] ??
                throw new ArgumentException($"Surface {surfaceIndex} is null.", nameof(surfaces));
            if (surface.IsVolume)
                throw new ArgumentException($"'{surface.Name}' is a solid, not a surface.", nameof(surfaces));
            if (surface.Mesh.Triangles.Count == 0 ||
                surface.Mesh.PrecisionPositions.Count != surface.Mesh.Positions.Count)
                throw new ArgumentException($"'{surface.Name}' has no complete exact surface topology.", nameof(surfaces));

            var vertexMap = new int[surface.Mesh.PrecisionPositions.Count];
            for (int i = 0; i < vertexMap.Length; i++)
            {
                Rat3Hybrid sourcePoint = surface.Mesh.PrecisionPositions[i];
                Rat3Hybrid point = new(in sourcePoint);
                point.Simplify();
                if (!exactToGlobal.TryGetValue(point, out int global))
                {
                    global = exact.Count;
                    exactToGlobal.Add(point, global);
                    exact.Add(point);
                    positions.Add(api.Converter.Convert(point));
                }
                vertexMap[i] = global;
            }

            var triangles = new List<Tri>(surface.Mesh.Triangles.Count);
            var usedGroups = new HashSet<int>();
            for (int triangleIndex = 0; triangleIndex < surface.Mesh.Triangles.Count; triangleIndex++)
            {
                Tri source = surface.Mesh.Triangles[triangleIndex];
                var triangle = new Tri(vertexMap[source.A], vertexMap[source.B], vertexMap[source.C]);
                if (triangle.ContainsDuplicateIndex())
                    throw new ArgumentException($"Exact sewing collapses a triangle in '{surface.Name}'.", nameof(surfaces));
                triangles.Add(triangle);
                usedGroups.Add(surface.Mesh.TrianglesEx[triangleIndex].GroupId);
                AddEdge(triangle.A, triangle.B, surfaceIndex);
                AddEdge(triangle.B, triangle.C, surfaceIndex);
                AddEdge(triangle.C, triangle.A, surfaceIndex);
            }
            mappedTriangles.Add(triangles);
            usedGroupsBySurface.Add(usedGroups);
        }

        var relations = new List<(int Other, bool Different)>[surfaces.Count];
        for (int i = 0; i < relations.Length; i++) relations[i] = new();
        foreach (var pair in edgeUses)
        {
            if (pair.Value.Count > 2)
                throw new InvalidOperationException("Sew rejected a non-manifold edge shared by more than two triangles.");
            if (pair.Value.Count != 2) continue;
            EdgeUse a = pair.Value[0], b = pair.Value[1];
            bool different = a.Forward == b.Forward;
            if (a.Surface == b.Surface)
            {
                if (different)
                    throw new InvalidOperationException($"Surface '{surfaces[a.Surface].Name}' is inconsistently oriented.");
                continue;
            }
            relations[a.Surface].Add((b.Surface, different));
            relations[b.Surface].Add((a.Surface, different));
        }

        var flip = new bool?[surfaces.Count];
        for (int seed = 0; seed < surfaces.Count; seed++)
        {
            if (flip[seed].HasValue) continue;
            flip[seed] = false;
            var queue = new Queue<int>();
            queue.Enqueue(seed);
            while (queue.Count > 0)
            {
                int current = queue.Dequeue();
                foreach (var relation in relations[current])
                {
                    bool required = flip[current].Value ^ relation.Different;
                    if (flip[relation.Other].HasValue)
                    {
                        if (flip[relation.Other].Value != required)
                            throw new InvalidOperationException("Surface seam orientations are contradictory.");
                    }
                    else
                    {
                        flip[relation.Other] = required;
                        queue.Enqueue(relation.Other);
                    }
                }
            }
        }

        int patchCount = 0;
        for (int i = 0; i < surfaces.Count; i++) patchCount += usedGroupsBySurface[i].Count;
        int firstGroup = GeoAPI.ReserveGroupIds(patchCount);
        int nextGroup = firstGroup;
        var names = new Dictionary<int, string>();
        var metadata = new Dictionary<string, SurfaceMetaData>();
        var usedNames = new HashSet<string>(StringComparer.Ordinal);
        var trianglesOut = new List<Tri>();
        var cornersOut = new List<MeshTriangle<TriangleVertexNormalUV>>();
        for (int surfaceIndex = 0; surfaceIndex < surfaces.Count; surfaceIndex++)
        {
            AnchorMesh surface = surfaces[surfaceIndex];
            var remap = new Dictionary<int, int>();
            var sourceGroups = usedGroupsBySurface[surfaceIndex].ToList();
            sourceGroups.Sort();
            foreach (int sourceGroup in sourceGroups)
            {
                int outputGroup = nextGroup++;
                remap.Add(sourceGroup, outputGroup);
                string baseName = surface.groupIdToExtendedName[sourceGroup];
                string patchName = baseName;
                int suffix = 1;
                while (!usedNames.Add(patchName))
                    patchName = EntityNaming.FormatPatchComponentName(baseName, suffix++);
                names.Add(outputGroup, patchName);
                if (surface.surfaceMetaData != null &&
                    surface.surfaceMetaData.TryGetValue(baseName, out var data) && data != null)
                    metadata.Add(patchName, data.Clone());
                else metadata.Add(patchName, new SurfaceMetaData(SurfaceType.Unknown));
            }

            for (int i = 0; i < mappedTriangles[surfaceIndex].Count; i++)
            {
                Tri triangle = mappedTriangles[surfaceIndex][i];
                var corners = surface.Mesh.TrianglesEx[i];
                int group = remap[corners.GroupId];
                if (flip[surfaceIndex].Value)
                {
                    trianglesOut.Add(new Tri(triangle.A, triangle.C, triangle.B));
                    cornersOut.Add(Reverse(corners, group));
                }
                else
                {
                    trianglesOut.Add(triangle);
                    corners.GroupId = group;
                    cornersOut.Add(corners);
                }
            }
        }

        bool watertight = MeshAnalysis.IsWatertightMesh(exact, trianglesOut);
        if (makeSolid && !watertight)
            throw new InvalidOperationException("Sewn surfaces do not form a watertight solid.");
        if (!MeshAnalysis.AreTrianglesConsistentlyOriented(exact, trianglesOut))
            throw new InvalidOperationException("Sewn surfaces are inconsistently oriented.");
        if (makeSolid) OrientClosedComponentsOutward(exact, trianglesOut, cornersOut);

        var mesh = new MeshNormalUV {
            Positions = positions, PrecisionPositions = exact,
            Triangles = trianglesOut, TrianglesEx = cornersOut
        };
        return new AnchorMesh(name, mesh, names, metadata,
            deferCoplanarPostProcess: true, isVolume: makeSolid);

        void AddEdge(int start, int end, int surface)
        {
            var key = start < end ? (start, end) : (end, start);
            if (!edgeUses.TryGetValue(key, out var uses))
                edgeUses.Add(key, uses = new List<EdgeUse>(2));
            uses.Add(new EdgeUse(surface, start < end));
        }
    }

    internal static void OrientClosedComponentsOutward(List<Rat3Hybrid> points, List<Tri> triangles,
        List<MeshTriangle<TriangleVertexNormalUV>> corners)
    {
        foreach (var component in MeshConnectivity.FindExactPositionComponents(points, triangles))
        {
            var componentTriangles = new List<Tri>(component.Count);
            foreach (int index in component) componentTriangles.Add(triangles[index]);
            int sign = MeshAnalysis.ComputeSignedMeshVolume(points, componentTriangles).Sign();
            if (sign == 0)
                throw new InvalidOperationException("A sewn closed component has zero exact volume.");
            if (sign > 0) continue;
            foreach (int index in component)
            {
                Tri triangle = triangles[index];
                triangles[index] = new Tri(triangle.A, triangle.C, triangle.B);
                corners[index] = Reverse(corners[index], corners[index].GroupId);
            }
        }
    }

    private static MeshTriangle<TriangleVertexNormalUV> Reverse(
        MeshTriangle<TriangleVertexNormalUV> source, int group)
    {
        var a = source.V0; var b = source.V1; var c = source.V2;
        a.Normal = -a.Normal; b.Normal = -b.Normal; c.Normal = -c.Normal;
        return new MeshTriangle<TriangleVertexNormalUV> { V0 = a, V1 = c, V2 = b, GroupId = group };
    }
}
