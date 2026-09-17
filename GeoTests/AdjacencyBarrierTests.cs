using GeoCore;
namespace GeoTests;
public class AdjacencyBarrierTests
{
    [Theory]
    [InlineData(0,3)]
    [InlineData(1,3)]
    [InlineData(2,3)]
    [InlineData(0,4)]
    [InlineData(1,4)]
    [InlineData(2,4)]
    public void AmbiguousEdgeIsAnAdjacencyBarrierForEveryLocalEdge(int localEdge,int incidentCount)
    {
        var triangles=Enumerable.Range(2,incidentCount).Select(i=>localEdge switch
        {
            0=>new Tri(0,1,i),
            1=>new Tri(i,0,1),
            _=>new Tri(1,i,0)
        }).ToList();
        var before=triangles.ToArray();
        Assert.Throws<Exception>(()=>Adjacency.BuildAdjacencyInformation(triangles));
        var adjacency=Adjacency.BuildAdjacencyInformation(triangles,out var edges,skipInvalidEdges:true);
        var shared=Assert.Single(edges,e=>e.NeighbourIndex1==-2);
        Assert.Equal(-2,shared.NeighbourIndex2);
        Assert.All(adjacency,a=>{Assert.Equal(-1,a.NeighbourAB);Assert.Equal(-1,a.NeighbourBC);Assert.Equal(-1,a.NeighbourCA);});
        var extended=Adjacency.BuildAdjacencyInformationEx(triangles,skipInvalidEdges:true);
        Assert.All(extended,a=>{Assert.Equal(-1,a.NeighbourAB);Assert.Equal(-1,a.NeighbourBC);Assert.Equal(-1,a.NeighbourCA);});
        Assert.Equal(before,triangles);
    }
}
