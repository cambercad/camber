using Curves;
using GeoCore;

namespace GeoTests;

public class LoftProfileContinuityTests
{
    [Fact]
    public void TangentSegmentJoinsAreNotCreases()
    {
        var points = new List<List<Vec2D>> {
            new() { new(1, 0), new(0, 1) },
            new() { new(0, 1), new(-1, 0) },
            new() { new(-1, 0), new(0, -1) },
            new() { new(0, -1), new(1, 0) },
        };
        // Circular arc endpoint normals; all four joins, including closure,
        // are tangent despite being separate named sketch entities.
        SketchStripTessellator.FlattenStripWithCreases(points, points,
            out _, out _, out var creases, preserveTangentJoints: false);
        Assert.Empty(creases);
    }

    [Fact]
    public void SquareSegmentJoinsRemainCreases()
    {
        var points = new List<List<Vec2D>> {
            new() { new(0, 0), new(1, 0) },
            new() { new(1, 0), new(1, 1) },
            new() { new(1, 1), new(0, 1) },
            new() { new(0, 1), new(0, 0) },
        };
        var normals = new List<List<Vec2D>> {
            new() { new(0, -1), new(0, -1) },
            new() { new(1, 0), new(1, 0) },
            new() { new(0, 1), new(0, 1) },
            new() { new(-1, 0), new(-1, 0) },
        };
        SketchStripTessellator.FlattenStripWithCreases(points, normals,
            out _, out _, out var creases, preserveTangentJoints: false);
        Assert.Equal(new[] { 0, 1, 2, 3 }, creases.OrderBy(i => i));
    }
}
