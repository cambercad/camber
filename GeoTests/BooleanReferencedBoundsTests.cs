using System.Numerics;
using CSG;
using GeoCore;

namespace GeoTests;

public class BooleanReferencedBoundsTests
{
    [Theory]
    [InlineData(BooleanOp.Union,368)]
    [InlineData(BooleanOp.Difference,152)]
    [InlineData(BooleanOp.Intersect,64)]
    public void BoundsPreparationPreservesExactVolumesAndIgnoresUnusedVertices(BooleanOp operation,int sixTimesVolume)
    {
        foreach (bool addUnused in new[] {false,true})
        {
            List<Rat3Hybrid> a = [new(0,0,0),new(6,0,0),new(0,6,0),new(0,0,6)];
            List<Rat3Hybrid> b = [new(2,0,0),new(8,0,0),new(2,6,0),new(2,0,6)];
            List<Tri> faces = [new(0,2,1),new(0,1,3),new(0,3,2),new(1,2,3)];
            if (addUnused)
            {
                var huge = new BigRationalHybrid(BigInteger.One<<3000,7);
                a.Add(new Rat3Hybrid(huge,huge,huge));
                b.Add(new Rat3Hybrid(-huge,-huge,-huge));
            }
            Resolver.Resolve(operation,a,faces,b,new List<Tri>(faces),out var points,out var triangles,out _);
            var volume = BigRationalHybrid.Zero;
            var edges = new Dictionary<(int,int),(int Count,int Winding)>();
            foreach (var t in triangles)
            {
                volume += Rat3Hybrid.Dot(points[t.A],Rat3Hybrid.Cross(points[t.B],points[t.C]));
                foreach (var (start,end) in new[] {(t.A,t.B),(t.B,t.C),(t.C,t.A)})
                {
                    var key=(Math.Min(start,end),Math.Max(start,end));
                    edges.TryGetValue(key,out var value);
                    edges[key]=(value.Count+1,value.Winding+(start<end?1:-1));
                }
            }
            // The overlap is a right tetrahedron of side 4; each input has
            // side 6. Check analytic rational volume and closed oriented edges.
            Assert.Equal(0,volume.CompareTo(new BigRationalHybrid(sixTimesVolume)));
            Assert.NotEmpty(edges);
            Assert.All(edges.Values,value => {Assert.Equal(2,value.Count);Assert.Equal(0,value.Winding);});
        }
    }
}
