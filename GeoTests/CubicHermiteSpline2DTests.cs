using Curves;
using GeoCore;

namespace GeoTests;

public class CubicHermiteSpline2DTests
{
    [Fact]
    public void Evaluate_HitsKnotsAtEnds()
    {
        var pts = new[] { new Vec2D(0, 0), new Vec2D(1, 0), new Vec2D(1, 2) };
        var h = new CubicHermiteSpline2D(pts);
        Assert.True(Vec2DOps.DistanceSquared(h.EvaluateVertex(0).Position, pts[0]) < 1e-20);
        Assert.True(Vec2DOps.DistanceSquared(h.EvaluateVertex(1).Position, pts[^1]) < 1e-20);
    }

    [Fact]
    public void OptionalStartTangent_OnlyDirectionMatters()
    {
        var pts = new[] { new Vec2D(0, 0), new Vec2D(1, 0), new Vec2D(2, 1) };
        var a = new CubicHermiteSpline2D(pts, startTangent: new Vec2D(0, 1));
        var b = new CubicHermiteSpline2D(pts, startTangent: new Vec2D(0, 1e9));
        var va = a.EvaluateVertex(0.05);
        var vb = b.EvaluateVertex(0.05);
        Assert.True(Vec2DOps.DistanceSquared(va.Position, vb.Position) < 1e-16);
    }

    [Fact]
    public void Length_BentPath_ExceedsChord()
    {
        var pts = new[] { new Vec2D(0, 0), new Vec2D(1, 1), new Vec2D(2, 0) };
        var h = new CubicHermiteSpline2D(pts);
        double chord = (pts[1] - pts[0]).Length() + (pts[2] - pts[1]).Length();
        Assert.True(h.Length() > chord * 0.99);
    }

    [Fact]
    public void NaturalTangents_ZeroSecondDerivativeAtEnds()
    {
        var pts = new[] { new Vec2D(0, 0), new Vec2D(1, 0), new Vec2D(1, 1), new Vec2D(2, 1) };
        var h = new CubicHermiteSpline2D(pts);
        // Finite-diff second derivative of position vs uniform u near the ends should be small
        // relative to a chord-locked start (which has a kink into the next span).
        Vec2D p0 = h.EvaluateVertex(0).Position;
        Vec2D pA = h.EvaluateVertex(0.02).Position;
        Vec2D pB = h.EvaluateVertex(0.04).Position;
        Vec2D d1 = (pA - p0) / 0.02;
        Vec2D d2 = (pB - pA) / 0.02;
        double accel = (d2 - d1).Length() / 0.02;
        Assert.True(accel < 5.0, $"expected mild end curvature for natural spline, accel={accel}");
    }

    [Fact]
    public void SetThroughPoints_CanGrowKnotCount()
    {
        var h = new CubicHermiteSpline2D(new[] { new Vec2D(0, 0), new Vec2D(1, 0) });
        var grown = new[] { new Vec2D(0, 0), new Vec2D(1, 1), new Vec2D(2, 0) };
        h.SetThroughPoints(grown);

        Assert.Equal(3, h.Points.Count);
        Assert.True(Vec2DOps.DistanceSquared(h.EvaluateVertex(0).Position, grown[0]) < 1e-20);
        Assert.True(Vec2DOps.DistanceSquared(h.EvaluateVertex(1).Position, grown[^1]) < 1e-20);
        Assert.True(h.Tessellate(1e-4).Count >= 3);
    }

    [Fact]
    public void PlotterSketcher_Tessellate_RegistersHermiteMetadata()
    {
        var sk = new PlotterSketcher("s");
        var h = sk.AddCubicHermiteSpline(new[] { new Vec2D(0, 0), new Vec2D(1, 1), new Vec2D(2, 0) });
        Assert.False(string.IsNullOrEmpty(h.Name));

        sk.Tessellate(1e-4, out _, out _, out var meta, 1e-8, SketchTessellationFlags.AllowOpenContour);

        Assert.True(meta.TryGetValue(h.Name, out var md));
        Assert.Equal(CurveType.HermiteSpline2D, md.CurveType);
        Assert.Same(h, md.Instance);
    }
}
