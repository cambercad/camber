using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class LoftSampledCorrespondenceTests
{
    [Fact]
    public void EqualEndCurveCountsSplitAnUnsegmentedIntermediateContourByRelativeArcLength()
    {
        static PlotterSketcherCoordSys Section(string name, double z, Vec2D[] corners)
        {
            var sketch = new PlotterSketcherCoordSys(name,
                new CoordinateSystem(new Vec3D(0, 0, z)));
            for (int i = 0; i < corners.Length; i++)
            {
                var curve = sketch.AddLine(corners[i], corners[(i + 1) % corners.Length]);
                curve.Name = "edge" + i;
            }
            return sketch;
        }

        var start = Section("start", 0,
        [new(-5, -3), new(5, -3), new(5, 3), new(-5, 3)]);
        var middle = new PlotterSketcherCoordSys("middle",
            new CoordinateSystem(new Vec3D(0, 0, 5)));
        middle.AddEllipse(new Vec2D(0), new Vec2D(4, 0), 2.5);
        var end = Section("end", 10,
        [new(-3, -5), new(3, -5), new(3, 5), new(-3, 5)]);

        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(MeshTestHelpers.MakeConverter(),
            [start, middle, end], .02,
            new LoftOptions { Style = LoftStyle.Ruled, CapEnds = true },
            output, "matched_sampled", out var names);

        Assert.Equal(4, names.Count(name => name.Value.StartsWith("matched_sampled-Side-edge")));
        Assert.NotNull(output.LoftSideSupportFactory);
        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        Assert.All(Enumerable.Range(0, 4), group => Assert.Contains(group, output.TriangleGroups));
    }
}
