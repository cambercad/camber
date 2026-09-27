using GeoCore;

namespace Geo;

/// <summary>A recursively placed part in an assembly's current world pose.</summary>
public sealed record AssemblyLeaf(string Path, AssemblyPart Part, Transform Pose)
{
    /// <summary>Return an unregistered geometry snapshot transformed to this leaf's pose.</summary>
    public AnchorMesh Snapshot()
    {
        CoordinateConverter converter = Part.OccurrenceContext?.Parent.Converter ?? Part.Assembly.Converter;
        return Part.Mesh.SnapshotForInspection(converter, Pose, Part.Mesh.Name);
    }
}
