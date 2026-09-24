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
    private readonly CoordinateConverter _converter;
    private readonly ShellSurfaceRegistry _surfaces;

    internal ShellTopologyBuilder(CoordinateConverter converter, ShellSurfaceRegistry surfaces)
    { _converter = converter; _surfaces = surfaces; }

    internal AnchorMesh BuildOffset(AnchorMesh source, HashSet<int> openingGroups,
        double retainedOffset, double openingOffset, int firstGroupId, bool exterior)
    {
        var support = new Dictionary<int, IShellSurfaceSupport>();
        foreach (var (id, patch) in source.groupIdToExtendedName)
        {
            if (!source.surfaceMetaData.TryGetValue(patch, out var meta) ||
                !source.TryGetTopologySurface(patch, out var uv))
                throw new ArgumentException($"Shell requires topology metadata for patch '{patch}'.");
            double amount = openingGroups.Contains(id) ? openingOffset : retainedOffset;
            var adapter = _surfaces.Resolve(meta, uv, amount);
            if (adapter == null)
                throw new NotSupportedException($"Shell does not support surface patch '{patch}' ({meta.SurfaceType}).");
            int triangleIndex = source.Mesh.TrianglesEx.FindIndex(t => t.GroupId == id);
            var tri = source.Mesh.Triangles[triangleIndex];
            var outward = Vec3DOps.Cross(source.Mesh.Positions[tri.B] - source.Mesh.Positions[tri.A],
                source.Mesh.Positions[tri.C] - source.Mesh.Positions[tri.A]);
            support[id] = adapter.CreateOffsetSupport(meta, uv, outward.Normalized(), amount, _converter);
        }

        var incident = BuildIncidentGroups(source);
        var points = new List<Vec3D>(source.Mesh.Positions.Count);
        foreach (var vertex in Enumerable.Range(0, source.Mesh.Positions.Count))
        {
            var groups = incident[vertex];
            // Boolean meshes may retain unreferenced points. They carry no
            // topology and therefore need no offset constraint.
            if (groups.Count == 0) { points.Add(source.Mesh.Positions[vertex]); continue; }
            var supports = groups.Select(id => support[id]).ToList();
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

        var cavityGroups = new Dictionary<int, string>();
        var cavityMeta = new Dictionary<string, SurfaceMetaData>();
        var remap = new Dictionary<int, int>();
        int next = firstGroupId;
        foreach (var (id, patch) in source.groupIdToExtendedName)
        {
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
            Positions = points, PrecisionPositions = points.Select(_converter.Convert).Select(p => new Rat3Hybrid(p.X, p.Y, p.Z)).ToList(),
            Triangles = new List<Tri>(source.Mesh.Triangles), TrianglesEx = corners
        };
        ValidateOffset(source.Mesh, cavity);
        return new AnchorMesh(source.Name + "_shellOffset", cavity, cavityGroups, cavityMeta,
            deferCoplanarPostProcess: true, isVolume: true);
    }

    private static Dictionary<int, List<int>> BuildIncidentGroups(AnchorMesh mesh)
    {
        var answer = Enumerable.Range(0, mesh.Mesh.Positions.Count).ToDictionary(i => i, _ => new HashSet<int>());
        for (int i = 0; i < mesh.Mesh.Triangles.Count; i++)
        {
            var t = mesh.Mesh.Triangles[i]; var g = mesh.Mesh.TrianglesEx[i].GroupId;
            answer[t.A].Add(g); answer[t.B].Add(g); answer[t.C].Add(g);
        }
        return answer.ToDictionary(x => x.Key, x => x.Value.OrderBy(v => v).ToList());
    }

    private static void ValidateOffset(MeshNormalUV source, MeshNormalUV cavity)
    {
        for (int i = 0; i < cavity.Triangles.Count; i++)
        {
            var t = cavity.Triangles[i];
            Vec3D offsetNormal = Vec3DOps.Cross(cavity.Positions[t.B] - cavity.Positions[t.A],
                cavity.Positions[t.C] - cavity.Positions[t.A]);
            if (offsetNormal.LengthSquared() == 0)
                throw new InvalidOperationException($"Shell offset collapsed triangle {i}.");
            var s = source.Triangles[i];
            Vec3D sourceNormal = Vec3DOps.Cross(source.Positions[s.B] - source.Positions[s.A],
                source.Positions[s.C] - source.Positions[s.A]);
            if (Vec3DOps.Dot(sourceNormal, offsetNormal) <= 0)
                throw new InvalidOperationException($"Shell offset inverted triangle {i}; the requested thickness changes topology.");
        }
        if (!MeshAnalysis.IsWatertightMesh(cavity.PrecisionPositions, cavity.Triangles, true) ||
            MeshAnalysis.ComputeSignedMeshVolume(cavity.Positions, cavity.Triangles) <= 0)
            throw new InvalidOperationException("Shell offset collapsed or changed topology.");
    }

}
