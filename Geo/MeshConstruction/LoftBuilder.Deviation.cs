using GeoCore;
using NURBS;

namespace Geo;

public static partial class LoftBuilder
{
    /// <summary>
    /// Reuse the NURBS surface tessellator's accepted leaf cells. Taking the
    /// union of their U/V boundaries makes every emitted loft cell a subset of
    /// an accepted patch, while retaining the existing cap and crease emitter.
    /// </summary>
    private static (double[] U, double[] V) SurfaceLoftParameters(
        Func<INurbsSurface> supportFactory, double deviation)
    {
        var surface = (BSplineSurface)supportFactory();
        _ = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface, out var patches, out _,
            maxDeviation: deviation, noNormals: true);
        var u = new SortedSet<double> { 0, 1 };
        var v = new SortedSet<double> { 0, 1 };
        foreach (var patch in patches)
        {
            if (!patch.IsLeave) continue;
            u.Add(patch.MinU);
            u.Add(patch.MaxU);
            v.Add(patch.MinV);
            v.Add(patch.MaxV);
        }
        return (u.ToArray(), v.ToArray());
    }
}
