using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class LoftDefaultAlignmentTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1e-10)]
    [InlineData(-1e-10)]
    public void SymmetricCapsules_KeepAuthoredOrientation_WhenNearestOriginFootChanges(double shift)
    {
        var sections = new List<PlotterSketcherCoordSys>();
        for (int i = 0; i < 4; i++)
        {
            double y = i % 2 == 0 ? shift : -shift;
            var frame = new CoordinateSystem(new Vec3D(0, 0, 4 * i),
                new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
            var sk = new PlotterSketcherCoordSys($"section{i}", frame, new Vec2D(-3, 2 + y));
            sk.AppendLine(3, 2 + y);
            sk.AddArc(new Vec2D(3, 2 + y), new Vec2D(5, y), new Vec2D(3, -2 + y));
            sk.AddLine(new Vec2D(3, -2 + y), new Vec2D(-3, -2 + y));
            sk.AddArc(new Vec2D(-3, -2 + y), new Vec2D(-5, y), new Vec2D(-3, 2 + y));
            sections.Add(sk);
        }
        var output = new MeshOutput();
        var converter = MeshTestHelpers.MakeConverter();
        LoftBuilder.GenerateLoftFromSketches(converter, sections,
            .01, new LoftOptions { CapEnds = false }, output, "capsules", out _);
        foreach (var point in output.Vertices)
        {
            // Every longitudinal section must retain the stadium boundary.
            // A seam flip folds that boundary into its interior between stations.
            double endDistance = Math.Max(0, Math.Abs(point.X) - 3);
            double radius = Math.Sqrt(endDistance * endDistance + point.Y * point.Y);
            // The authoritative construction points include frame and local
            // coordinate rounding, each bounded by one lattice diagonal.
            double rounding = 2 * Math.Sqrt(3) * converter.SmallestUnit();
            Assert.InRange(radius, 1.98 - rounding, 2 + Math.Abs(shift) + rounding);
        }
    }

    [Fact]
    public void DefaultLoft_PreservesIntentionallyRotatedAuthoredConnectors()
    {
        var first = new PlotterSketcherCoordSys("first", CoordinateSystem.Default, new Vec2D(-2, -1));
        first.AppendLine(2, -1); first.AppendLine(2, 1); first.AppendLine(-2, 1); first.AppendLine(-2, -1);
        var frame = new CoordinateSystem(new Vec3D(0, 0, 4),
            new Vec3D(0, 1, 0), new Vec3D(-1, 0, 0), new Vec3D(0, 0, 1));
        var second = new PlotterSketcherCoordSys("second", frame, new Vec2D(-2, -1));
        second.AppendLine(2, -1); second.AppendLine(2, 1); second.AppendLine(-2, 1); second.AppendLine(-2, -1);
        var output = new MeshOutput();
        var converter = MeshTestHelpers.MakeConverter();
        LoftBuilder.GenerateLoftFromSketches(converter, [first, second],
            .01, new LoftOptions { CapEnds = false, Style = LoftStyle.Ruled }, output, "rotated", out _);
        var start = new PreciseFrameTransform(converter, CoordinateSystem.Default).Transform(new Vec3D(-2, -1, 0));
        var end = new PreciseFrameTransform(converter, frame).Transform(new Vec3D(-2, -1, 0));
        Assert.Contains(start, output.PrecisePositions);
        Assert.Contains(Enumerable.Range(0, output.Vertices.Count), i =>
            output.UVs[i].X == 0 && output.PrecisePositions[i] == end);
        for (int i = 0; i < output.Vertices.Count; i++)
            Assert.Equal(converter.Convert(output.PrecisePositions[i]), output.Vertices[i]);
    }
}
