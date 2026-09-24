using CSG;
using GeoCore;
using GeoMeta;

namespace Geo.Shelling;

internal sealed class ShellOperation
{
    internal AnchorMesh Run(GeoAPI api, AnchorMesh source, List<string> faces, double thickness, string name, bool outward)
    {
        if (source == null) throw new ArgumentNullException(nameof(source));
        if (!source.IsVolume) throw new ArgumentException("Shell requires a watertight solid volume.", nameof(source));
        if (thickness <= 0 || !double.IsFinite(thickness)) throw new ArgumentOutOfRangeException(nameof(thickness));
        faces ??= new List<string>();
        source.EnsureCoplanarPostProcessed();
        var selected = new HashSet<int>();
        foreach (var face in faces)
        {
            if (!source.extendedNameToGroupId.TryGetValue(face, out var id))
                throw new ArgumentException($"Shell face '{face}' does not exist on '{source.Name}'.", nameof(faces));
            if (!selected.Add(id)) throw new ArgumentException($"Shell face '{face}' was selected more than once.", nameof(faces));
        }
        var builder = new ShellTopologyBuilder(api.Converter, ShellSurfaceRegistry.AnalyticV1);
        int ReserveGroups(int count)
        {
            int first = GeoAPI.GetBaseGroupIndex();
            GeoAPI.IncrementBaseGroupIndex(count);
            return first;
        }
        double overrun = Math.Max(thickness * 1e-5, api.Converter.SmallestUnit() * 4);
        var exterior = outward
            ? ExpandExterior(source)
            : source;
        AnchorMesh cavity;
        if (outward)
            cavity = selected.Count == 0 ? source
                : builder.BuildOffset(source, selected, 0, -thickness - overrun,
                    ReserveGroups(source.groupIdToExtendedName.Count), exterior: false);
        else
            cavity = builder.BuildOffset(source, selected, thickness, -overrun,
                ReserveGroups(source.groupIdToExtendedName.Count), exterior: false);
        if (outward)
        {
            // A mitered offset is valid only if it contains the entire source.
            // This exact CSG difference also catches offsets that cross a
            // concave wall without visibly inverting a local triangle.
            var escaped = MeshNormalUV.BooleanOperation(source.Mesh, exterior.Mesh,
                BooleanOp.Subtract, api.Converter);
            double unit = api.Converter.SmallestUnit();
            double escapedVolume = MeshAnalysis.ComputeSignedMeshVolume(escaped.Positions, escaped.Triangles);
            if (!double.IsFinite(escapedVolume) || Math.Abs(escapedVolume) > 8 * unit * unit * unit)
                throw new InvalidOperationException("Shell outward offset intersects the source volume.");
        }
        var mesh = MeshNormalUV.BooleanOperation(exterior.Mesh, cavity.Mesh, BooleanOp.Subtract, api.Converter);
        if (!MeshAnalysis.IsWatertightMesh(mesh.PrecisionPositions, mesh.Triangles, true))
            throw new InvalidOperationException("Shell produced a non-watertight result (offset self-intersection or collapse).");
        if (MeshAnalysis.ComputeSignedMeshVolume(mesh.Positions, mesh.Triangles) <= 0)
            throw new InvalidOperationException("Shell produced an empty or inverted result.");
        var names = new Dictionary<int, string>(exterior.groupIdToExtendedName);
        var meta = SurfaceMetaData.CloneDictionary(exterior.surfaceMetaData);
        // CSG retains the annular portion of an opening support as the sharp
        // topological rim. Promote that ordinary patch to an explicit,
        // selectable shell rim name (its adjacent group edges remain filletable).
        foreach (var id in selected)
        {
            string sourcePatch = source.groupIdToExtendedName[id];
            string rim = EntityNaming.ShellRim(sourcePatch);
            int exteriorId = outward ? exterior.extendedNameToGroupId[sourcePatch] : id;
            names[exteriorId] = rim;
            if (meta.Remove(sourcePatch, out var sourceMeta)) meta[rim] = sourceMeta;
        }
        foreach (var item in cavity.groupIdToExtendedName)
        {
            string patch = outward && selected.Count == 0 ? EntityNaming.ShellInner(item.Value) : item.Value;
            names[item.Key] = patch;
            meta[patch] = cavity.surfaceMetaData[item.Value].Clone();
        }
        return new AnchorMesh(name, mesh, names, meta, deferCoplanarPostProcess: false, isVolume: true);

        AnchorMesh ExpandExterior(AnchorMesh solid)
        {
            solid.EnsureCoplanarPostProcessed();
            try
            {
                return builder.BuildOffset(solid, new HashSet<int>(), -thickness, 0,
                    ReserveGroups(solid.groupIdToExtendedName.Count), exterior: true);
            }
            catch (ArgumentException ex) when (solid.UnionOperands != null &&
                ex.Message.StartsWith("Shell offset changes topology or uses an unsupported surface junction", StringComparison.Ordinal))
            {
                var (first, second) = solid.UnionOperands.Value;
                // Union operands may have been moved or edited since this solid
                // was created. Recheck their set equality before using history.
                var rebuilt = MeshNormalUV.BooleanOperation(first.Mesh, second.Mesh, BooleanOp.Union, api.Converter);
                double unit = api.Converter.SmallestUnit();
                double tolerance = 8 * unit * unit * unit;
                foreach (var (left, right) in new[] { (solid.Mesh, rebuilt), (rebuilt, solid.Mesh) })
                {
                    var difference = MeshNormalUV.BooleanOperation(left, right, BooleanOp.Subtract, api.Converter);
                    double volume = MeshAnalysis.ComputeSignedMeshVolume(difference.Positions, difference.Triangles);
                    if (!double.IsFinite(volume) || Math.Abs(volume) > tolerance)
                        throw new InvalidOperationException("Shell union operands no longer match the source solid.");
                }
                var a = ExpandExterior(first);
                var b = ExpandExterior(second);
                var united = MeshNormalUV.BooleanOperation(a.Mesh, b.Mesh, BooleanOp.Union, api.Converter);
                if (!MeshAnalysis.IsWatertightMesh(united.PrecisionPositions, united.Triangles, true) ||
                    MeshAnalysis.ComputeSignedMeshVolume(united.Positions, united.Triangles) <= 0)
                    throw new InvalidOperationException("Shell expanded union is not watertight.");
                var used = united.TrianglesEx.Select(t => t.GroupId).ToHashSet();
                var groups = new Dictionary<int, string>();
                var metadata = new Dictionary<string, SurfaceMetaData>();
                void Add(AnchorMesh operand, AnchorMesh expanded)
                {
                    foreach (var (id, patch) in expanded.groupIdToExtendedName)
                    {
                        if (!used.Contains(id)) continue;
                        string canonical = operand.extendedNameToGroupId.TryGetValue(patch, out int originalId) &&
                            solid.groupIdToExtendedName.TryGetValue(originalId, out string mapped) ? mapped : patch;
                        groups.Add(id, canonical);
                        metadata.Add(canonical, expanded.surfaceMetaData[patch].Clone());
                    }
                }
                Add(first, a);
                Add(second, b);
                return new AnchorMesh(solid.Name + "_shellOffset", united, groups, metadata,
                    deferCoplanarPostProcess: true, isVolume: true);
            }
        }
    }
}
