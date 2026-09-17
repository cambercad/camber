using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class BlendClosureAttributesTests
{
    [Fact]
    public void SplitClosurePreservesItsSupportingTriangleAttributesAndExactPoints()
    {
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-10),new Vec3D(10)));
        List<Rat3Hybrid> points = [new(0,0,0),new(6,0,0),new(0,6,0),new(6,6,3)];
        List<Vec3D> normals = [new(0,0,1),new(0,0,1),new Vec3D(0,1,2).Normalized(),new Vec3D(1,0,2).Normalized()];
        List<Vec2D> uv = [new(0,0),new(1,0),new(0,1),new(1,1)];
        var support = new UVSurface(converter.Convert(points),normals,uv,[new(0,1,2),new(1,3,2)],points);
        var quarter = new BigRationalHybrid(1,4);
        var weights = new[] {new Rat3Hybrid(2,1,1)*quarter,new Rat3Hybrid(1,2,1)*quarter,new Rat3Hybrid(1,1,2)*quarter};
        var splitPoints = weights.Select(w=>points[1]*w.X+points[3]*w.Y+points[2]*w.Z).ToList();
        var closure = new Surface([new Tri(0,1,2)],splitPoints,new List<int>{1});
        var result = BlendEdge.InterpolateClosureAttributes(closure,support,converter);
        Assert.Equal(splitPoints,result.PointsPrecise);
        Assert.Equal(closure.Triangles,result.Triangles);
        for(int i=0;i<weights.Length;i++)
        {
            var w=weights[i];
            var expectedNormal=(normals[1]*w.X.ToDouble()+normals[3]*w.Y.ToDouble()+normals[2]*w.Z.ToDouble()).Normalized();
            var expectedUv=uv[1]*w.X.ToDouble()+uv[3]*w.Y.ToDouble()+uv[2]*w.Z.ToDouble();
            Assert.InRange((result.Normals[i]-expectedNormal).Length(),0,1e-12);
            Assert.InRange((result.Uv[i]-expectedUv).Length(),0,1e-12);
            Assert.Equal(converter.Convert(splitPoints[i]),result.Points[i]);
        }
        Assert.True((result.Normals[0]-result.Normals[1]).Length()>.01);
    }
}
