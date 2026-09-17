using Geo;
using GeoCore;

namespace GeoTests;

public class SmallRadiusBoxFilletTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void DegenerateBoundarySegmentOnlyContainsItsOwnPoint()
    {
        var method = typeof(BlendEdge).GetMethod("PointOnBoundaryPolyline",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Static)!;
        var start = new Rat3Hybrid(1, 2, 3);
        var repeated = new List<Rat3Hybrid> { start, start };
        bool Contains(List<Rat3Hybrid> curve, Rat3Hybrid point) =>
            (bool)method.Invoke(null, new object[] { curve, point })!;
        Assert.True(Contains(repeated, start));
        Assert.False(Contains(repeated, new Rat3Hybrid(9, 8, 7)));
        repeated.Add(new Rat3Hybrid(5, 2, 3));
        Assert.True(Contains(repeated, new Rat3Hybrid(3, 2, 3)));
        Assert.True(Contains(repeated, new Rat3Hybrid(5, 2, 3)));
        Assert.False(Contains(repeated, new Rat3Hybrid(6, 2, 3)));
        Assert.False(Contains(repeated, new Rat3Hybrid(3, 2, 4)));
    }

    [Theory]
    [InlineData(false, 0, 2)]
    [InlineData(false, 0, 3)]
    [InlineData(false, 0, 4)]
    [InlineData(true, 0, 2)]
    [InlineData(false, .4, 2)]
    [InlineData(true, .4, 2)]
    public void SmallRoundsOnNonOrthogonalSlopeStayClosed(bool ridge, double angle, int length)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-160, -160, -20), new Vec3D(160, 160, 80)), .02);
        var frame = new CoordinateSystem(new Vec3D(-3.9, 0, 0),
            new Vec3D(-Math.Sin(angle), Math.Cos(angle), 0), new Vec3D(0, 0, 1),
            new Vec3D(Math.Cos(angle), Math.Sin(angle), 0));
        double end = length * 4 - .1;
        var points = ridge
            ? new Vec2D[] { new(-7.9, 0), new(7.9, 0), new(7.9, 3.2), new(0, 9.6), new(-7.9, 3.2) }
            : new Vec2D[] { new(-end, 0), new(end, 0), new(end, 3.2), new(8-end, 9.6), new(-end, 9.6) };
        var sketch = new Curves.PlotterSketcherCoordSys("profile", frame, points[0]);
        for (int i = 1; i <= points.Length; i++)
            sketch.AppendLine(points[i % points.Length].X, points[i % points.Length].Y);
        var blank = api.Extrude(sketch, 7.8, name: "slope");
        blank.EnsureCoplanarPostProcessed();
        var edges = blank.GroupEdges.Select(edge => edge.Name).ToList();
        Assert.Equal(15, edges.Count);
        var rounded = api.Fillet(blank, edges, .08, .01, "rounded");
        Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));
        var strips = rounded.surfaceMetaData.Values.Where(meta => meta.CylinderParams != null).ToList();
        Assert.Equal(15, strips.Count);
        Assert.All(strips, meta => Assert.Equal(.08, meta.CylinderParams.Radius, 12));
        double removed = MeshAnalysis.ComputeSignedMeshVolume(blank.Mesh.Positions, blank.Mesh.Triangles)
            - MeshAnalysis.ComputeSignedMeshVolume(rounded.Mesh.Positions, rounded.Mesh.Triangles);
        Assert.InRange(removed, .1, 2);
    }

    [Theory]
    [InlineData(false, .025)]
    [InlineData(true, .025)]
    [InlineData(true, .01)]
    [InlineData(true, .005)]
    public void PhysicalSmallRoundsOnRectangularBlankStayClosed(bool allEdges, double deviation)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-160, -160, -20), new Vec3D(160, 160, 80)), .025);
        var blank = api.CreateCuboid(new Vec3D(-7.9, -15.9, 0), new Vec3D(7.9, 15.9, 9.6), "blank");
        blank.EnsureCoplanarPostProcessed();
        var edges = blank.GroupEdges.Select(edge => edge.Name).ToList();
        Assert.Equal(12, edges.Count);
        if (!allEdges) edges = edges.Take(1).ToList();
        var rounded = api.Fillet(blank, edges, .08, deviation, "rounded");
        Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));
        if (allEdges)
        {
            var cylindricalStrips = rounded.surfaceMetaData.Values.Where(meta => meta.CylinderParams != null).ToList();
            Assert.Equal(12, cylindricalStrips.Count);
            Assert.All(cylindricalStrips, meta => Assert.Equal(.08, meta.CylinderParams.Radius, 12));
            double removed = MeshAnalysis.ComputeSignedMeshVolume(blank.Mesh.Positions, blank.Mesh.Triangles)
                - MeshAnalysis.ComputeSignedMeshVolume(rounded.Mesh.Positions, rounded.Mesh.Triangles);
            Assert.InRange(removed, .2, 1);
        }
    }
}
