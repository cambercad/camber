using GeoCore;
using NURBS;

namespace GeoTests;

public sealed class NurbsOneSidedNormalTests
{
    [Fact]
    public void CollapsedBoundaryHasTheLimitingSurfaceNormal()
    {
        var surface = new BSplineSurface(1, 1, new[] {
            new[] { new Vec3D(0), new Vec3D(0) },
            new[] { new Vec3D(1,0,0), new Vec3D(1,1,0) }
        }, new[] { 0.0,0,1,1 }, new[] { 0.0,0,1,1 });
        Assert.Equal(new Vec3D(0,0,1), surface.EvaluateNormal(0,.5).Normalized());
    }

    [Fact]
    public void KnotNormalsPreserveSharpCreaseWithoutMovingEvaluationPoint()
    {
        var surface = new BSplineSurface(1, 1, new[] {
            new[] { new Vec3D(0,0,0), new Vec3D(0,0,1) },
            new[] { new Vec3D(1,0,0), new Vec3D(1,0,1) },
            new[] { new Vec3D(1,1,0), new Vec3D(1,1,1) }
        }, new[] { 0.0,0,.5,1,1 }, new[] { 0.0,0,1,1 });

        Assert.Equal(new Vec3D(1,0,.25), surface.Evaluate(.5,.25));
        Assert.Equal(new Vec3D(0,-1,0), surface.EvaluateNormal(.5,.25,true).Normalized());
        Assert.Equal(new Vec3D(1,0,0), surface.EvaluateNormal(.5,.25,false).Normalized());
        // Span hints must not change which one-sided derivative is returned.
        Assert.Equal(new Vec3D(0,-1,0), surface.EvaluateNormal(.5,.25,true).Normalized());
    }
}
