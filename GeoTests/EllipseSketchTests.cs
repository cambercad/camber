using Curves;
using Curves.Base;
using Geo;
using Geo.NurbsConstruction;
using GeoCore;
using GeoMeta;
using GeoSolver;
using GeoSolver.Sketcher;

namespace GeoTests;

public class EllipseSketchTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void AxisDimensionsUpdateTheLiveConic()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100),new Vec3D(100)),.01);
        var sketch = api.GetConstraintSketcher(DefaultPlanes.OriginXY,"ellipse");
        sketch.SolveAfterEveryConstraint = false;
        var e = Assert.IsType<CEllipse2D>(sketch.AddEllipse(new Vec2D(2,3),new Vec2D(8,1),5));
        Assert.Equal(5,e.Count());
        sketch.FixPoint(e.CCenter);
        sketch.SetVerticalDistance(e.CCenter,e.Evaluate(0),0);
        var major = new DistanceBetweenPoints2d(e.CCenter,e.Evaluate(0),10);
        var minor = new DistanceBetweenPoints2d(e.CCenter,e.Evaluate(.25),4);
        sketch.AddConstraint(major);
        sketch.AddConstraint(minor);
        Assert.True(sketch.SolveConstraints() < 1e-8);
        Assert.Equal(10,e.MajorAxisLength,7);
        Assert.Equal(4,e.MinorAxisLength,7);
        Assert.True(sketch.TryGetCPointOnEdge(e.Name+"@center",out var center));
        Assert.Same(e.CCenter,center);
        // Replacing a driving dimension edits the same ellipse and its live
        // quarter-point expressions; no contour is recreated.
        sketch.RemoveConstraint(minor);
        sketch.SetDistance(e.CCenter,e.Evaluate(.25),6);
        Assert.True(sketch.SolveConstraints() < 1e-8);
        Assert.Equal(6,e.MinorAxisLength,7);
        Assert.Equal(9,e.EvaluateVertex(.25).Position.Y,7);
        Assert.Equal(e.StartPosition,e.EndPosition);
        e.Freeze(); Assert.True(e.All(p=>p.Frozen));
        e.Unfreeze(); Assert.True(e.All(p=>!p.Frozen));
    }

    [Theory]
    [InlineData(0,2*Math.PI)]
    [InlineData(2*Math.PI,0)]
    [InlineData(.23,2.1)]
    [InlineData(2.1,.23)]
    public void RationalConversionRetainsConicAndArcEndpoints(double start,double end)
    {
        var e = new Ellipse2D(new Vec2D(7,-3),new Vec2D(8,6),4,start,end);
        var frame = CoordinateSystem.Default;
        var curve = Curve2DToBSpline.EllipseToBSpline(e,frame);
        Assert.Equal(2,curve.Degree);
        Assert.True((curve.Start-frame.PointTo3D(e.StartPosition)).Length()<1e-12);
        Assert.True((curve.End-frame.PointTo3D(e.EndPosition)).Length()<1e-12);
        for (int i=0;i<=200;i++)
        {
            var p=curve.EvaluateUniform(i/200.0);
            double dx=p.X-7,dy=p.Y+3;
            double x=(.8*dx+.6*dy)/10, y=(-.6*dx+.8*dy)/4;
            Assert.InRange(Math.Abs(x*x+y*y-1),0,2e-13);
        }
        if (e.IsFullEllipse)
            Assert.Equal(curve.Start,curve.End);
    }

    [Fact]
    public void ReflectedEllipseRetainsParameterPointsAndConstrainableType()
    {
        var e = new Ellipse2D(new Vec2D(2,3),new Vec2D(8,6),4,.2,2.4,CurveFlags.HelperGeometry);
        Vec2D Map(Vec2D p) => new Vec2D(-p.X+20,p.Y-4);
        var transformed = Assert.IsType<CEllipse2D>(SketchCurveTransform.TransformCurve(e,Map,new ConstrainableCurveFactory()));
        Assert.Equal(e.Flags,transformed.Flags);
        for(int i=0;i<=20;i++)
            Assert.True((Map(e.EvaluateVertex(i/20.0).Position)-transformed.Evaluate(i/20.0).Evaluate()).Length()<1e-12);
        Assert.Equal(e.Flags,e.GetCopy().Flags);
        Assert.Equal(e.Flags,e.Reverse().Flags);
    }

    [Theory]
    [InlineData(.02)]
    [InlineData(100)]
    public void ClosedTessellationHasExactSeamAndQuarterPoints(double deviation)
    {
        var e = new Ellipse2D(new Vec2D(2,3),new Vec2D(10,0),4);
        var vertices=e.Tessellate(deviation);
        Assert.Equal(0,(vertices.Count-1)%4);
        Assert.Equal(vertices[0].Position,vertices[^1].Position);
        Assert.Equal(12,vertices[0].Position.X,12);
        Assert.Equal(7,vertices[(vertices.Count-1)/4].Position.Y,12);
        var reversed=(Ellipse2D)e.Reverse();
        Assert.True(reversed.IsFullEllipse);
        Assert.Equal(reversed.StartPosition,reversed.EndPosition);
        Assert.True((reversed.StartTangent+e.StartTangent).Length()<1e-12);
    }

    [Theory]
    [InlineData(1e-12)]
    [InlineData(1)]
    [InlineData(1e6)]
    public void AnalyticEllipseDoesNotCollapseAtSmallScales(double scale)
    {
        var e = new Ellipse2D(new Vec2D(0),new Vec2D(10*scale,0),4*scale);
        for(int i=0;i<=16;i++)
        {
            var vertex=e.EvaluateVertex(i/16.0);
            double x=vertex.Position.X/(10*scale),y=vertex.Position.Y/(4*scale);
            Assert.InRange(Math.Abs(x*x+y*y-1),0,1e-14);
            Assert.Equal(1,vertex.Tangent.Length(),12);
            var gradient=new Vec2D(x/10,y/4);
            Assert.True(vertex.Normal.X*gradient.X+vertex.Normal.Y*gradient.Y > 0);
            double angle=2*Math.PI*i/16;
            var derivative=new Vec2D(-10*Math.Sin(angle),4*Math.Cos(angle));
            Assert.True(vertex.Tangent.X*derivative.X+vertex.Tangent.Y*derivative.Y > 0);
        }
        Assert.Equal(e.Tessellate(.01*scale)[0].Position,e.Tessellate(.01*scale)[^1].Position);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void InvalidDeviationIsRejected(double value)
        => Assert.Throws<ArgumentOutOfRangeException>(()=>new Ellipse2D(new Vec2D(0),new Vec2D(10,0),4).Tessellate(value));
}
