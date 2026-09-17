namespace Geo;

/// <summary>Immutable creation and separating-boundary ancestry, independent of surface geometry.</summary>
internal sealed class FaceLineage
{
    public IReadOnlyList<string> Roots { get; }
    public IReadOnlyList<string> Separators { get; }
    public bool Split { get; }
    public bool Supported { get; }
    public string RootKey { get; }
    public string Reference { get; }

    public FaceLineage(IEnumerable<string> roots, IEnumerable<string> separators = null,
        bool split = false, bool supported = true)
    {
        Roots = Array.AsReadOnly(roots.Distinct(StringComparer.Ordinal).Order(StringComparer.Ordinal).ToArray());
        Separators = Array.AsReadOnly((separators ?? Array.Empty<string>()).Distinct(StringComparer.Ordinal)
            .Order(StringComparer.Ordinal).ToArray());
        Split = split;
        Supported = supported;
        RootKey = string.Concat(Roots.Select(root => root.Length + ":" + root));
        Reference = Split ? GeoMeta.EntityNaming.FormatFaceProvenance(Roots, Separators) : string.Join("&", Roots);
    }

    public FaceLineage Remap(Func<string, string> rename) =>
        new(Roots.Select(rename), Separators.Select(rename), Split, Supported);

    public static FaceLineage Merge(IEnumerable<FaceLineage> faces)
    {
        var values = faces.Distinct().ToArray();
        if (values.Length == 1) return values[0];
        return new(values.SelectMany(face => face.Roots), values.SelectMany(face => face.Separators),
            values.Any(face => face.Split), values.All(face => face.Supported));
    }
}
