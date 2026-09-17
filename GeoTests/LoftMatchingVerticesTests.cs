using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class LoftMatchingVerticesTests
{
    [Theory]
    [InlineData(LoftStyle.Ruled)]
    [InlineData(LoftStyle.SmoothCatmullRom)]
    public void ChangingAspectRatiosKeepCornersAndSharpFaceNormals(LoftStyle style)
    {
        var converter = MeshTestHelpers.MakeConverter();
        Vec2D[][] outlines =
        [
            [new(-1, 0), new(1, 0), new(1, 12), new(-1, 12)],
            [new(-4, 0), new(4, 0), new(4, 4), new(-4, 4)]
        ];
        var sections = outlines.Select((points, i) =>
        {
            var frame = new CoordinateSystem(new Vec3D(0, 0, i * 10),
                new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
            var sketch = new PlotterSketcherCoordSys("section" + i, frame, points[0]);
            foreach (var point in points.Skip(1).Append(points[0])) sketch.AppendLine(point.X, point.Y);
            return sketch;
        }).ToList();
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(converter, sections, .02,
            new LoftOptions { CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices, Style = style },
            output, "matched", out _);
        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        var support = output.LoftSideSupportFactory();
        for (int corner = 0; corner < 4; corner++)
        {
            var expected = (outlines[0][corner] + outlines[1][corner]) / 2;
            Assert.InRange((support.Evaluate(corner / 4.0, .5) - new Vec3D(expected.X, expected.Y, 5)).Length(), 0, 1e-10);
        }
        for (int i = 0; i < output.Triangles.Count; i++)
        {
            if (output.TriangleGroups[i] >= 4) continue;
            var triangle = output.Triangles[i];
            var a = output.Vertices[triangle.A];
            var b = output.Vertices[triangle.B];
            var c = output.Vertices[triangle.C];
            var normal = Vec3DOps.Cross(b - a, c - a).Normalized();
            foreach (int vertex in new[] { triangle.A, triangle.B, triangle.C })
            {
                Assert.True(Vec3DOps.Dot(normal, output.Normals[vertex]) > .99999,
                    $"Face normal {normal} disagrees with corner normal {output.Normals[vertex]} at UV {output.UVs[vertex]}.");
                var uv = output.UVs[vertex];
                Assert.InRange((support.Evaluate(uv.X, uv.Y) - output.Vertices[vertex]).Length(),
                    0, 2 * Math.Sqrt(3) * converter.SmallestUnit());
            }
        }
    }
}
