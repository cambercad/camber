using DotWrap;
using Geo;
namespace GeoPy;
[DotWrapExpose]
public class NativeSection
{
    readonly SectionView _inner;
    internal NativeSection(SectionView section) { _inner=section; }
    public string Raycast(double ox,double oy,double oz,double dx,double dy,double dz)
        => PackHit(_inner.Raycast(new GeoCore.Vec3D(ox,oy,oz),new GeoCore.Vec3D(dx,dy,dz)));

    internal static string PackHit(RayMeshHit hit)
    {
        if (hit == null) return string.Empty;
        var inv = System.Globalization.CultureInfo.InvariantCulture;
        return string.Join(" ",
            hit.Point.X.ToString("G17", inv),
            hit.Point.Y.ToString("G17", inv),
            hit.Point.Z.ToString("G17", inv),
            hit.GeometricNormal.X.ToString("G17", inv),
            hit.GeometricNormal.Y.ToString("G17", inv),
            hit.GeometricNormal.Z.ToString("G17", inv),
            hit.ParameterT.ToString("G17", inv),
            hit.TriangleIndex.ToString(inv),
            hit.GroupId.ToString(inv));
    }

    public string DumpDisplay() => DisplayPack.PackMeshes(_inner.Meshes);
}
