using CSG;
using GeoCore;
using GeoMeta;

namespace Geo.Shelling;

internal sealed class ShellOperation
{
    internal AnchorMesh Run(GeoAPI api, AnchorMesh source, List<string> faces, double thickness, string name)
    {
        if (source == null) throw new ArgumentNullException(nameof(source));
        if (!source.IsVolume) throw new ArgumentException("Shell requires a watertight solid volume.", nameof(source));
        if (thickness <= 0 || !double.IsFinite(thickness)) throw new ArgumentOutOfRangeException(nameof(thickness));
        if (faces == null || faces.Count == 0) throw new ArgumentException("Shell requires at least one face.", nameof(faces));
        source.EnsureCoplanarPostProcessed();
        var selected = new HashSet<int>();
        foreach (var face in faces)
        {
            if (!source.extendedNameToGroupId.TryGetValue(face, out var id))
                throw new ArgumentException($"Shell face '{face}' does not exist on '{source.Name}'.", nameof(faces));
            if (!selected.Add(id)) throw new ArgumentException($"Shell face '{face}' was selected more than once.", nameof(faces));
        }
        int first = GeoAPI.GetBaseGroupIndex();
        GeoAPI.IncrementBaseGroupIndex(source.groupIdToExtendedName.Count);
        var cavity = new ShellTopologyBuilder(api.Converter, ShellSurfaceRegistry.AnalyticV1)
            .BuildCavity(source, selected, thickness, first);
        var mesh = MeshNormalUV.BooleanOperation(source.Mesh, cavity.Mesh, BooleanOp.Difference, api.Converter);
        if (!MeshAnalysis.IsWatertightMesh(mesh.PrecisionPositions, mesh.Triangles, true))
            throw new InvalidOperationException("Shell produced a non-watertight result (offset self-intersection or collapse).");
        var names = new Dictionary<int, string>(source.groupIdToExtendedName);
        var meta = SurfaceMetaData.CloneDictionary(source.surfaceMetaData);
        // CSG retains the annular portion of an opening support as the sharp
        // topological rim.  Promote that ordinary planar patch to an explicit,
        // selectable shell rim name (its adjacent group edges remain filletable).
        foreach (var id in selected)
        {
            string sourcePatch = source.groupIdToExtendedName[id];
            string rim = EntityNaming.ShellRim(sourcePatch);
            names[id] = rim;
            if (meta.Remove(sourcePatch, out var sourceMeta)) meta[rim] = sourceMeta;
        }
        foreach (var item in cavity.groupIdToExtendedName) { names[item.Key] = item.Value; meta[item.Value] = cavity.surfaceMetaData[item.Value].Clone(); }
        return new AnchorMesh(name, mesh, names, meta, deferCoplanarPostProcess: false, isVolume: true);
    }
}
