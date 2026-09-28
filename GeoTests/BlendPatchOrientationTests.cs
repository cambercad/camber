using Geo;
using GeoCore;

namespace GeoTests;

public class BlendPatchOrientationTests
{
    [Fact]
    public void IndependentBlendSheetsUseTheirOwnOutwardOrientation()
    {
        var patches = new List<UVSurface>
        {
            Triangle(0, 0, 1, 0, 1, 1, reverse: true),
            Triangle(0, 0, 1, 1, 0, 1, reverse: false),
            Triangle(3, 0, 4, 0, 4, 1, reverse: false),
            Triangle(3, 0, 4, 1, 3, 1, reverse: true)
        };

        BlendCorner.OrientBlendPatches(patches);

        foreach (var patch in patches)
        {
            var tri = patch.Triangles[0];
            var normal = Vec3DOps.Cross(patch.Points[tri.B] - patch.Points[tri.A],
                patch.Points[tri.C] - patch.Points[tri.A]);
            Assert.True(normal.Z > 0);
        }
    }

    private static UVSurface Triangle(int ax, int ay, int bx, int by, int cx, int cy, bool reverse)
    {
        var points = new List<Vec3D> { new(ax, ay, 0), new(bx, by, 0), new(cx, cy, 0) };
        var precise = new List<Rat3Hybrid> { new(ax, ay, 0), new(bx, by, 0), new(cx, cy, 0) };
        var normals = new List<Vec3D> { new(0, 0, 1), new(0, 0, 1), new(0, 0, 1) };
        var uv = new List<Vec2D> { new(0, 0), new(1, 0), new(0, 1) };
        return new UVSurface(points, normals, uv,
            new List<Tri> { reverse ? new Tri(0, 2, 1) : new Tri(0, 1, 2) }, precise);
    }
}
