using Geo;
using GeoCore;
using System.Reflection;
namespace GeoTests;
public class TangentCylinderPropagationTests
{
    [Theory]
    [InlineData(7,0,1,true)] // External tangency, R3 + R4.
    [InlineData(1,0,1,true)] // Internal tangency, R4 - R3.
    [InlineData(7.00001,0,1,false)]
    [InlineData(6.99999,0,1,false)]
    [InlineData(1.00001,0,1,false)]
    [InlineData(7,1,1,false)] // Skew axes cannot share a tangent generator.
    public void AnalyticCylinderTangencyRequiresTrueParallelContact(double distance,double axisX,double axisZ,bool expected)
    {
        var a=new SurfaceMetaData(SurfaceType.Unknown){CylinderParams=new CylinderSurfaceParams{Origin=new Vec3D(0),Axis=new Vec3D(0,0,1),Radius=3}};
        var b=new SurfaceMetaData(SurfaceType.Unknown){CylinderParams=new CylinderSurfaceParams{Origin=new Vec3D(distance,0,5),Axis=new Vec3D(axisX,0,axisZ),Radius=4}};
        var type=typeof(GeoAPI).Assembly.GetType("Geo.EdgeBlendPipeline");
        var method=type.GetMethod("AreAnalyticallyTangent",BindingFlags.NonPublic|BindingFlags.Static);
        Assert.Equal(expected,(bool)method.Invoke(null,new object[]{a,b}));
    }
}
