using Curves;
using Geo;
using GeoCore;
using GeoSolver;

namespace GeoTests;

public class ConvexArmFilletTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void DimensionedConvexArmRoundsEveryEdge(int side)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-160, -160, -20), new Vec3D(160, 160, 160)), .025);
        var frame = new CoordinateSystem(new Vec3D(0, 2.3, 0), new Vec3D(1, 0, 0),
            new Vec3D(0, 0, 1), new Vec3D(0, -1, 0));
        var sketch = api.GetConstraintSketcher(frame, "arm");
        sketch.SolveAfterEveryConstraint = false;
        var datum = sketch.AddCLine(new Vec2D(0, 0), new Vec2D(1, 0), CurveFlags.HelperGeometry);
        sketch.FixPoint(datum.CStart, new Vec2D(0, 0));
        sketch.FixPoint(datum.CEnd, new Vec2D(1, 0));
        var points = new Vec2D[] { new(6.7, 27.5), new(7.4, 29.4), new(9.6, 29.2),
            new(11, 25.3), new(12.1, 21.2), new(9.3, 20.4) };
        points = points.Select(p => new Vec2D(side * p.X, p.Y)).ToArray();
        if (side < 0) Array.Reverse(points);
        var lines = new List<CLine2D>();
        for (int i = 0; i < points.Length; i++)
        {
            var line = sketch.AddCLine(points[i], points[(i + 1) % points.Length]);
            if (i > 0) sketch.SetPointOnPoint(lines[^1].CEnd, line.CStart);
            if (i == points.Length - 1) sketch.SetPointOnPoint(line.CEnd, lines[0].CStart);
            sketch.SetHorizontalDistancePointPoint(datum.CStart, line.CStart, points[i].X);
            sketch.SetVerticalDistancePointPoint(datum.CStart, line.CStart, points[i].Y);
            lines.Add(line);
        }
        Assert.True(sketch.SolveConstraints() < 1e-8);
        var blank = api.Extrude(sketch, 4.6, name: "arm");
        blank.EnsureCoplanarPostProcessed();
        var edges = blank.GroupEdges.Select(edge => edge.Name).ToList();
        Assert.Equal(18, edges.Count);
        var rounded = api.Fillet(blank, edges, .3, .025, "rounded");
        Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));
        var strips = rounded.surfaceMetaData.Values.Where(meta => meta.CylinderParams != null).ToList();
        Assert.Equal(18, strips.Count);
        Assert.All(strips, meta => Assert.Equal(.3, meta.CylinderParams.Radius, 12));
        var self = api.Boolean(rounded, rounded, CSG.BooleanOp.Intersect, "self");
        self.EnsureCoplanarPostProcessed();
        Assert.True(MeshAnalysis.IsWatertightMesh(self.Mesh.PrecisionPositions, self.Mesh.Triangles));
    }
    [Fact]
    public void LaterTrimRetainsPartialArcsAcrossDifferentBoundarySubdivisions()
    {
        var points = new List<Rat3Hybrid> { new(0, 0, 0), new(4, 0, 0), new(4, 2, 0), new(0, 2, 0) };
        var surface = new UVSurface(points.Select(p => new Vec3D(p.X.ToDouble(), p.Y.ToDouble(), p.Z.ToDouble())).ToList(),
            Enumerable.Repeat(new Vec3D(0, 0, 1), 4).ToList(), Enumerable.Repeat(new Vec2D(0, 0), 4).ToList(),
            new List<Tri> { new(0, 1, 2), new(0, 2, 3) }, points);
        var partial = new List<Rat3Hybrid> { new(-2, 0, 0), new(1, 0, 0), new(6, 0, 0) };
        var removed = new List<Rat3Hybrid> { new(0, -1, 0), new(4, -1, 0) };
        var interior = new List<Rat3Hybrid> { points[0], points[2] };
        var degenerate = new List<Rat3Hybrid> { points[0], points[0] };
        var result = BlendEdge.RetainBoundaryArcs(surface, new[] { partial, removed, interior, degenerate });
        var retained = Assert.Single(result);
        Assert.Equal(3, retained.Count);
        Assert.Contains(retained, point => point == new Rat3Hybrid(0, 0, 0));
        Assert.Contains(retained, point => point == new Rat3Hybrid(1, 0, 0));
        Assert.Contains(retained, point => point == new Rat3Hybrid(4, 0, 0));
        Assert.Equal(new Rat3Hybrid(-2, 0, 0), partial[0]);
        Assert.Equal(new Rat3Hybrid(6, 0, 0), partial[^1]);
    }

}
