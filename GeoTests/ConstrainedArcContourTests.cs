using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class ConstrainedArcContourTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Theory]
    [InlineData(1)]
    [InlineData(10)]
    public void TangentCrownRemainsClosedAfterGeometryUpdate(double scale)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-4000), new Vec3D(4000)), .01);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY, "crown");
        sketch.SolveAfterEveryConstraint = false;
        Vec2D P(double x, double y) => new Vec2D(x * scale, y * scale);
        var left = sketch.AddCArc(P(-10.6, 314.1), P(-13, 319), P(-14, 325));
        var crown = sketch.AddCArc(P(-14, 325), P(0, 339), P(14, 325));
        var right = sketch.AddCArc(P(14, 325), P(13, 319), P(10.6, 314.1));
        var baseLine = sketch.AddCLine(P(10.6, 314.1), P(-10.6, 314.1));
        var centerLine = sketch.AddCLine(P(-20, 325), P(20, 325), CurveFlags.HelperGeometry);
        sketch.FixPoint(baseLine.CStart, P(10.6, 314.1));
        sketch.FixPoint(baseLine.CEnd, P(-10.6, 314.1));
        sketch.FixPoint(centerLine.CStart, P(-20, 325));
        sketch.FixPoint(centerLine.CEnd, P(20, 325));
        sketch.SetRadius(crown, 14 * scale);
        sketch.FixPoint(crown.CCenter, P(0, 325));
        sketch.SetPointOnLine(crown.CStart, centerLine);
        sketch.SetPointOnLine(crown.CEnd, centerLine);
        sketch.SetPointOnPoint(left.CEnd, crown.CStart);
        sketch.SetPointOnPoint(right.CStart, crown.CEnd);
        sketch.FixPoint(left.CStart, P(-10.6, 314.1));
        sketch.FixPoint(right.CEnd, P(10.6, 314.1));
        sketch.SetTangentCircles(crown, left);
        sketch.SetTangentCircles(crown, right);
        var result = sketch.SolveConstraintsDetailed();
        Assert.True(result.Converged, $"{result.Message}; SSE={result.SumOfSquaredErrors:R}");
        var solid = api.Extrude(sketch, 1, name: "closed_crown");
        MeshTestHelpers.AssertValidMesh(solid.Mesh.Positions, solid.Mesh.Triangles);
    }
}
