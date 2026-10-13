using Curves;
using Geo.NurbsConstruction;
using GeoCore;

namespace GeoTests;

public class ArcNurbsConversionTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void ASpanJustBeyondAQuadrantHasNoArtificialTailKnot(int direction)
    {
        double sweep=direction*(Math.PI/2+5e-7);
        Vec2D Point(double angle)=>new(3+8*Math.Cos(angle),4+8*Math.Sin(angle));
        var arc=new Arc2D(Point(0),Point(sweep/2),Point(sweep));
        var frame=new CoordinateSystem(new Vec3D(0));
        var curve=Curve2DToBSpline.ArcToBSpline(arc,frame);
        var knots=curve.Knots.Distinct().Order().ToArray();
        for(int i=1;i<knots.Length;i++) Assert.InRange(knots[i]-knots[i-1],.5,1);
        Assert.InRange((curve.EvaluateUniform(0)-frame.PointTo3D(arc.StartPosition)).Length(),0,1e-12);
        Assert.InRange((curve.EvaluateUniform(1)-frame.PointTo3D(arc.EndPosition)).Length(),0,1e-12);
        for(int i=0;i<=32;i++)
            Assert.InRange(Math.Abs((curve.EvaluateUniform(i/32.0)-frame.PointTo3D(arc.Center)).Length()-arc.Radius),0,1e-11);
        Assert.Equal(direction,Math.Sign(curve.EvaluateUniform(.5).Y-4));
    }
}
