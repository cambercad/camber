using Geo;
using GeoCore;

namespace GeoTests;

/// <summary>Planar UV from minimum-area rectangle (used by loft end caps).</summary>
public class AutoUvPlanarTests
{
    [Fact]
    public void MinimumAreaRectangle_OfAxisAlignedBox_MatchesExtents()
    {
        var pts = new List<Vec2D>
        {
            new Vec2D(0, 0),
            new Vec2D(4, 0),
            new Vec2D(4, 1),
            new Vec2D(0, 1)
        };
        var rect = AutoUV.FindMinimumAreaRectangle(pts);
        Assert.InRange(rect.Size.X * rect.Size.Y, 3.99, 4.01);
        Assert.True(Math.Min(rect.Size.X, rect.Size.Y) < 1.01);
        Assert.True(Math.Max(rect.Size.X, rect.Size.Y) > 3.99);
    }

    [Fact]
    public void PlanarUv_RotatedThinRectangle_FillsBothAxesNear01()
    {
        // 4×1 rectangle rotated 30° — AABB would waste UV space; MAR should fill [0,1]².
        double ang = Math.PI / 6.0;
        double c = Math.Cos(ang), s = Math.Sin(ang);
        var local = new[]
        {
            new Vec2D(-2, -0.5),
            new Vec2D(2, -0.5),
            new Vec2D(2, 0.5),
            new Vec2D(-2, 0.5)
        };
        var pts = new List<Vec2D>();
        foreach (var p in local)
            pts.Add(new Vec2D(p.X * c - p.Y * s, p.X * s + p.Y * c));

        Vec2D[] uv = AutoUV.ComputePlanarUvFromPoints2D(pts);
        double minU = uv.Min(v => v.X), maxU = uv.Max(v => v.X);
        double minV = uv.Min(v => v.Y), maxV = uv.Max(v => v.Y);
        Assert.InRange(minU, -1e-9, 1e-9);
        Assert.InRange(minV, -1e-9, 1e-9);
        Assert.InRange(maxU, 1.0 - 1e-6, 1.0 + 1e-6);
        Assert.InRange(maxV, 1.0 - 1e-6, 1.0 + 1e-6);
        // Longer side → U
        Assert.True(maxU - minU >= maxV - minV - 1e-9);
    }
}
