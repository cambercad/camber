using GeoCore;

namespace CSG;

/// <summary>Transient classification data; observers must not retain the mesh arrays.</summary>
public sealed class BooleanFragments
{
    public IReadOnlyList<Tri> Triangles { get; }
    public IReadOnlyList<SourceTriangle> Sources { get; }
    public IReadOnlyList<bool> Retained { get; }

    internal BooleanFragments(IReadOnlyList<Tri> triangles, IReadOnlyList<SourceTriangle> sources,
        IReadOnlyList<bool> retained)
    {
        Triangles = triangles;
        Sources = sources;
        Retained = retained;
    }
}
