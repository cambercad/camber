using CSG;
using GeoCore;
namespace GeoTests;
public class DisconnectedCavityTrimTests
{
    [Theory]
    [InlineData(false,3,false,true)]
    [InlineData(true,3,false,true)]
    [InlineData(false,7,false,true)]
    [InlineData(true,7,false,true)]
    [InlineData(false,3,false)]
    [InlineData(true,3,false)]
    [InlineData(false,7,false)]
    [InlineData(true,7,false)]
    [InlineData(false,2,false)]
    [InlineData(true,2,false)]
    [InlineData(false,5,false)]
    [InlineData(true,5,false)]
    [InlineData(false,8,false)]
    [InlineData(true,8,false)]
    [InlineData(false,8,true)]
    [InlineData(true,8,true)]
    [InlineData(false,8,true,true)]
    [InlineData(true,8,true,true)]
    [InlineData(false,5,false,true)]
    [InlineData(true,5,false,true)]
    public void PlanarVolumeTrimClassifiesUntouchedCavityShell(bool keepPositive,int cutX,bool separateIsland,bool tilted=false)
    {
        List<Rat3Hybrid> points=[];List<Tri> triangles=[];
        void Cube(int low,int high,bool inward)
        {
            int offset=points.Count;
            foreach(var (x,y,z) in new[]{(low,low,low),(high,low,low),(high,high,low),(low,high,low),
                (low,low,high),(high,low,high),(high,high,high),(low,high,high)})
                points.Add(new Rat3Hybrid(x,y,z));
            foreach(var (a,b,c) in new[]{(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
                (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)})
                triangles.Add(new Tri(offset+a,offset+(inward?c:b),offset+(inward?b:c)));
        }
        Cube(0,10,false);Cube(3,7,true);
        if(separateIsland)Cube(20,22,false);
        List<Rat3Hybrid> plane=[new(cutX,-1,-1),new(cutX,11,-1),new(cutX,11,11),new(cutX,-1,11)];
        if(tilted)
        {
            Rat3Hybrid Transform(Rat3Hybrid p) => new(
                p.X*new BigRationalHybrid(3,5)-p.Y*new BigRationalHybrid(4,5)+new BigRationalHybrid(101,103),
                p.X*new BigRationalHybrid(4,5)+p.Y*new BigRationalHybrid(3,5)-new BigRationalHybrid(107,109),
                p.Z+new BigRationalHybrid(113,127));
            points=points.Select(Transform).ToList();plane=plane.Select(Transform).ToList();
        }
        // A finite planar cutter contributes its region inside the source volume.
        // Reverse its winding to select the other physical half. Exterior concave
        // blend patches use KeepInNormalDirection instead; FrameJointFilletTests
        // covers that distinct contract and requires a positive material addition.
        Resolver.Resolve(BooleanOp.AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection,
            points,triangles,plane,keepPositive?[new Tri(0,2,1),new Tri(0,3,2)]:[new Tri(0,1,2),new Tri(0,2,3)],out var resultPoints,out var resultTriangles,out _);
        var volume=BigRationalHybrid.Zero;
        foreach(var t in resultTriangles) volume+=Rat3Hybrid.Dot(resultPoints[t.A],Rat3Hybrid.Cross(resultPoints[t.B],resultPoints[t.C]));
        int negativeVolume=cutX*100-Math.Clamp(cutX-3,0,4)*16;
        int expected=(keepPositive?936-negativeVolume:negativeVolume)+(separateIsland?8:0);
        Assert.True(volume == new BigRationalHybrid(expected*6));
        var edges=new Dictionary<(int,int),(int count,int winding)>();
        foreach(var t in resultTriangles)
            foreach(var (a,b) in new[]{(t.A,t.B),(t.B,t.C),(t.C,t.A)})
            {
                var key=(Math.Min(a,b),Math.Max(a,b));edges.TryGetValue(key,out var value);
                edges[key]=(value.count+1,value.winding+(a<b?1:-1));
            }
        Assert.All(edges.Values,value => {Assert.Equal(2,value.count);Assert.Equal(0,value.winding);});
    }
}
