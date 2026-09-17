using Curves;
using Geo;
using GeoCore;
using GeoSolver;
using GeoSolver.Sketcher;

namespace GeoTests;

public class SketchRepeatTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private const double GeomTol = 1e-4;


    private static double SumCircleAreas(PlotterSketcher sketch)
    {
        double area = 0.0;
        foreach (var strip in sketch.GetCurves())
        {
            foreach (var curve in strip)
            {
                Circle2D circle = curve is TransformedSketchCurve2D wrapper
                    ? wrapper.Geometry as Circle2D
                    : curve as Circle2D;
                if (circle != null)
                    area += Math.PI * circle.Radius * circle.Radius;
            }
        }

        return area;
    }

    private static int CountWrappers(PlotterSketcher sketch)
    {
        int count = 0;
        foreach (var strip in sketch.GetCurves())
        {
            foreach (var curve in strip)
            {
                if (curve is TransformedSketchCurve2D)
                    count++;
            }
        }

        return count;
    }

    private static ConstrainedSketcher MakeConstrainedSketch(string name = "sk")
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1000), new Vec3D(1000)), 0.01);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, name);
        sketch.SolveAfterEveryConstraint = false;
        return sketch;
    }

    [Fact]
    public void RepeatGrid_PlotterSketcher_ThreeCircles()
    {
        var sketch = new PlotterSketcher("grid");
        var source = new List<Curve2D> { sketch.AddCircle(Vec2DOps.Zero, 1.0) };

        sketch.RepeatGrid(source, countX: 3, countY: 1, stepX: new Vec2D(3, 0), stepY: Vec2DOps.Zero);

        Assert.Equal(3.0 * Math.PI, SumCircleAreas(sketch), GeomTol);
    }

    [Fact]
    public void RepeatGrid_SkipsOriginalByDefault()
    {
        var sketch = new PlotterSketcher("grid");
        var source = new List<Curve2D> { sketch.AddCircle(Vec2DOps.Zero, 1.0) };

        var added = sketch.RepeatGrid(source, countX: 2, countY: 2, stepX: new Vec2D(5, 0), stepY: new Vec2D(0, 5));

        Assert.Equal(3, added.Count);
        Assert.Equal(3, CountWrappers(sketch));
    }

    [Fact]
    public void RepeatGrid_ReturnsWrapperType()
    {
        var sketch = new PlotterSketcher("grid");
        var source = new List<Curve2D> { sketch.AddCircle(Vec2DOps.Zero, 1.0) };

        var added = sketch.RepeatGrid(source, countX: 2, countY: 1, stepX: new Vec2D(4, 0), stepY: Vec2DOps.Zero);

        Assert.Single(added);
        Assert.IsType<TransformedSketchCurve2D>(added[0]);
    }

    [Fact]
    public void RepeatGrid_PreservesStripConnectivity()
    {
        var sketch = new PlotterSketcher("strip");
        sketch.SetStartPoint(Vec2DOps.Zero);
        var line1 = sketch.AddLine(Vec2DOps.Zero, new Vec2D(1, 0));
        var line2 = sketch.AppendLine(new Vec2D(1, 1));
        var source = new List<Curve2D> { line1, line2 };

        sketch.RepeatGrid(source, countX: 2, countY: 1, stepX: new Vec2D(5, 0), stepY: Vec2DOps.Zero);

        var strips = sketch.GetCurves().Where(strip => strip.Any(c => c is TransformedSketchCurve2D)).ToList();
        Assert.Single(strips);
        Assert.Equal(2, strips[0].Count);

        var first = (TransformedSketchCurve2D)strips[0][0];
        var second = (TransformedSketchCurve2D)strips[0][1];
        Assert.True((first.Geometry.EndPosition - second.Geometry.StartPosition).Length() < GeomTol);
    }

    [Fact]
    public void RepeatCircular_FourFold()
    {
        var sketch = new PlotterSketcher("circular");
        var source = new List<Curve2D> { sketch.AddCircle(new Vec2D(5, 0), 0.5) };

        sketch.RepeatCircular(source, center: Vec2DOps.Zero, count: 4);

        Assert.Equal(4.0 * Math.PI * 0.25, SumCircleAreas(sketch), GeomTol);
    }

    [Fact]
    public void RepeatGrid_UpdatesAfterSolve()
    {
        var sketch = MakeConstrainedSketch();
        var circle = sketch.AddCCircle(Vec2DOps.Zero, 2.0);
        sketch.FixPoint(circle.CCenter, Vec2DOps.Zero);
        var source = new List<Curve2D> { circle };

        var copies = sketch.RepeatGrid(source, countX: 2, countY: 1, stepX: new Vec2D(10, 0), stepY: Vec2DOps.Zero);
        Assert.Single(copies);

        sketch.SetRadius(circle, 3.0);
        sketch.SolveConstraints();

        var copyCircle = (Circle2D)((TransformedSketchCurve2D)copies[0]).Geometry;
        Assert.Equal(3.0, copyCircle.Radius, GeomTol);
        Assert.Equal(10.0, copyCircle.Center.X, GeomTol);
        Assert.Equal(0.0, copyCircle.Center.Y, GeomTol);
    }

    [Fact]
    public void RepeatCircular_UpdatesAfterSolve()
    {
        var sketch = MakeConstrainedSketch();
        var line = sketch.AddCLine(Vec2DOps.Zero, new Vec2D(2, 0));
        sketch.FixPoint(line.CStart, Vec2DOps.Zero);
        var source = new List<Curve2D> { line };

        var copies = sketch.RepeatCircular(source, center: Vec2DOps.Zero, count: 4);
        Assert.Equal(3, copies.Count);

        sketch.SetLength(line, 5.0);
        sketch.SolveConstraints();

        var copyLine = (Line2D)((TransformedSketchCurve2D)copies[0]).Geometry;
        Assert.Equal(5.0, copyLine.Length(), GeomTol);
    }

    [Fact]
    public void ConstrainedConcentricCircles_AreTwoStrips()
    {
        var sketch = MakeConstrainedSketch();
        sketch.AddCCircle(Vec2DOps.Zero, 10.0);
        sketch.AddCCircle(Vec2DOps.Zero, 4.0);

        var strips = sketch.GetCurves().Where(s => s.Count > 0).ToList();
        Assert.Equal(2, strips.Count);
        Assert.All(strips, s => Assert.Single(s));
    }

    [Fact]
    public void ConstrainedWasher_ExtrudesAsVolume()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), 0.01);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "washer");
        sketch.SolveAfterEveryConstraint = false;
        var outer = sketch.AddCCircle(Vec2DOps.Zero, 10.0);
        var inner = sketch.AddCCircle(Vec2DOps.Zero, 4.0);
        sketch.SetConcentric(outer, inner);
        sketch.SetRadius(outer, 10.0);
        sketch.SetRadius(inner, 4.0);
        sketch.SolveConstraints();

        const double height = 2.0;
        var solid = api.Extrude(sketch, height, 0.01, "washer");
        Assert.True(solid.IsVolume);
        double volume = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(solid.Mesh.Positions, solid.Mesh.Triangles));
        double expected = Math.PI * (10.0 * 10.0 - 4.0 * 4.0) * height;
        Assert.InRange(volume, expected * 0.97, expected * 1.03);
    }
}
