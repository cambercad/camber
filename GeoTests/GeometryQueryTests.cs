using GeoCore;
using Xunit;

public class GeometryQueryTests
{
    static GeometryQuery.Primitive Point(double x,double y,double z) => new(new(x,y,z),default,default,1);
    static GeometryQuery.Primitive Edge(Vec3D a,Vec3D b) => new(a,b,default,2);
    static GeometryQuery.Primitive Face(double z) => new(new(0,0,z),new(1,0,z),new(0,1,z),3);
    static GeometryQuery Query(params GeometryQuery.Primitive[] items) => new(items);
    [Fact]
    public void AllEntityCombinationsAndSymmetry()
    {
        var point=Point(.25,.25,3);var edge=Edge(new(0,0,2),new(1,0,2));var face=Face(0);
        Check(point,Point(.25,.25,7),4);Check(point,edge,Math.Sqrt(1.0625));Check(point,face,3);
        Check(edge,Edge(new(.5,-1,4),new(.5,1,4)),2);Check(edge,face,2);Check(face,Face(5),5);
        void Check(GeometryQuery.Primitive a,GeometryQuery.Primitive b,double expected)
        { Assert.Equal(expected,Math.Sqrt(Query(a).Closest(Query(b)).SquaredDistance),12);
            Assert.Equal(expected,Math.Sqrt(Query(b).Closest(Query(a)).SquaredDistance),12); }
    }
    [Fact]
    public void IntersectionsAndDegeneracy()
    {
        Assert.Equal(0,Query(Edge(new(.2,.2,-1),new(.2,.2,1))).Closest(Query(Face(0))).SquaredDistance);
        Assert.Equal(1,Query(Point(1,1,0)).Closest(Query(new GeometryQuery.Primitive(new(0,0,0),new(0,0,0),new(2,0,0),3))).SquaredDistance);
        Assert.True(Query(Edge(new(1+1e-12,0,-1),new(1+1e-12,0,1))).Closest(Query(Face(0))).SquaredDistance>0);
    }
    [Fact]
    public void ConservativeFloatBoundsAtLargeCoordinates()
    {
        var primitives=new GeometryQuery.Primitive[1000];
        for(int i=0;i<primitives.Length;i++) primitives[i]=Point(1e9+i*.01,0,0);
        var query=Query(primitives); var probe=Query(Point(1e9+6.5025,0,1));
        double expected=double.PositiveInfinity;
        foreach(var p in primitives) expected=Math.Min(expected,GeometryQuery.Distance(p,Point(1e9+6.5025,0,1)).SquaredDistance);
        Assert.Equal(expected,query.Closest(probe).SquaredDistance);
    }
    [Fact]
    public void NearestRayAndSectionPlane()
    {
        var primitives=new GeometryQuery.Primitive[4096];
        for(int i=0;i<primitives.Length;i++) primitives[i]=Face(i+1);
        var query=Query(primitives);
        Assert.Equal(0,query.Ray(new(.2,.2,0),new(0,0,2),3).Index);
        Assert.Equal(24,query.Ray(new(.2,.2,5000),new(0,0,-1),3,0,new(0,0,1),25).Index);
        Assert.Equal(-1,query.Ray(new(2,2,0),new(0,0,1),3).Index);
    }
    [Fact]
    public void ParallelSegmentHitsNearestClippedEndpoint()
    {
        var query=Query(Edge(new(0,0,-1),new(0,0,2)));
        Assert.Equal(3,query.Ray(new(0,0,5),new(0,0,-1),2,.1).Depth);
        Assert.Equal(5,query.Ray(new(0,0,5),new(0,0,-1),2,.1,new(0,0,1),0).Depth);
    }
    [Fact]
    public void EmptyAndInvalidQueries()
    {
        Assert.Equal(-1,Query().Ray(default,new(0,0,1),3).Index);
        Assert.Throws<ArgumentException>(()=>Query().Closest(Query(Point(0,0,0))));
        Assert.Throws<ArgumentException>(()=>Query(Point(double.NaN,0,0)));
    }
    [Fact]
    public void ImmutableQueriesCanBeSharedAcrossThreads()
    {
        var query=Query(Face(0));
        Parallel.For(0,100,i=>Assert.Equal(0,query.Ray(new(.2,.2,1),new(0,0,-1),3).Index));
    }
    [Fact]
    public void BvhDistancesMatchExhaustivePrimitivePairs()
    {
        var random=new Random(103);
        Vec3D Next() => new(random.NextDouble()*10-5,random.NextDouble()*10-5,random.NextDouble()*10-5);
        var a=new GeometryQuery.Primitive[60]; var b=new GeometryQuery.Primitive[40];
        for(int i=0;i<a.Length;i++) a[i]=new(Next(),Next(),Next(),1+i%3);
        for(int i=0;i<b.Length;i++) b[i]=new(Next(),Next(),Next(),1+i%3);
        double expected=double.PositiveInfinity;
        foreach(var x in a) foreach(var y in b) expected=Math.Min(expected,GeometryQuery.Distance(x,y).SquaredDistance);
        Assert.Equal(expected,Query(a).Closest(Query(b)).SquaredDistance);
    }
}
