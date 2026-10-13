using Curves;
using Geo;
using GeoCore;
using NURBS;

namespace GeoTests;

public sealed class LoftPointClosureTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);
    private static PlotterSketcherCoordSys Circle(double z)
    {
        var sketch = new PlotterSketcherCoordSys("circle"+z, new CoordinateSystem(new Vec3D(0,0,z)));
        sketch.AddCircle(new Vec2D(0), 2);
        return sketch;
    }
    private static MeshOutput Build(LoftOptions options)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20),new Vec3D(20)), .005);
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(api.Converter, [Circle(0), Circle(10)], .03,
            options, output, "closed", out _);
        return output;
    }

    [Theory]
    [InlineData(LoftPointClosure.Rounded)]
    [InlineData(LoftPointClosure.Sharp)]
    public void ExactPointCapsHaveOnePoleAndWatertightNondegenerateTriangles(LoftPointClosure closure)
    {
        var output = Build(new LoftOptions { StartPoint=new(0,0,-4), EndPoint=new(0,0,14),
            StartPointClosure=closure, EndPointClosure=closure });
        Assert.True(MeshAnalysis.IsWatertightMesh(output.PrecisePositions, output.Triangles));
        var indices=output.PrecisePositions.Distinct().Select((point,index)=>(point,index)).ToDictionary(p=>p.point,p=>p.index);
        var directed=new Dictionary<(int,int),int>();
        foreach(var triangle in output.Triangles)
        {
            int[] points=[indices[output.PrecisePositions[triangle.A]],indices[output.PrecisePositions[triangle.B]],indices[output.PrecisePositions[triangle.C]]];
            for(int i=0;i<3;i++)
            {
                int a=points[i],b=points[(i+1)%3];
                var edge=(Math.Min(a,b),Math.Max(a,b));
                directed[edge]=directed.GetValueOrDefault(edge)+(a<b?1:-1);
            }
        }
        Assert.All(directed.Values,winding=>Assert.Equal(0,winding));
        foreach (var triangle in output.Triangles)
            Assert.True(Vec3DOps.Cross(output.Vertices[triangle.B]-output.Vertices[triangle.A],
                output.Vertices[triangle.C]-output.Vertices[triangle.A]).LengthSquared() > 0);
        foreach (var cap in new[] { output.StartPointCap, output.EndPointCap })
        {
            Assert.Single(Enumerable.Range(0,output.Vertices.Count).Where(i =>
                output.UVs[i].Y==1 && (output.Vertices[i]-cap.Point).Length() < .005)
                .Select(i=>output.PrecisePositions[i]).Distinct());
            for (int i=0; i<=16; i++)
            {
                double u=i/16.0;
                Assert.InRange((cap.Evaluate(u,1)-cap.Point).Length(),0,1e-12);
                var normal=cap.EvaluateNormal(u,1).Normalized();
                Assert.True(double.IsFinite(normal.X) && double.IsFinite(normal.Y) && double.IsFinite(normal.Z));
                if (closure==LoftPointClosure.Rounded)
                    Assert.InRange((normal-cap.OutwardNormal).Length(),0,1e-12);
            }
        }
    }

    [Theory]
    [InlineData(LoftPointClosure.Rounded)]
    [InlineData(LoftPointClosure.Sharp)]
    public void PointCapUvCornersStayLocalAcrossSeamAndPole(LoftPointClosure closure)
    {
        var output=Build(new LoftOptions { StartPoint=new(0,0,-4),EndPoint=new(0,0,14),
            StartPointClosure=closure,EndPointClosure=closure });
        var mesh=new MeshNormalUV(new CoordinateConverter(new Box3D(new Vec3D(-20),new Vec3D(20))),
            output.Vertices,output.Normals,output.UVs,output.Triangles,output.TriangleGroups,output.PrecisePositions);
        Assert.Equal(output.PrecisePositions.Distinct().Count(),mesh.PrecisionPositions.Count);
        Assert.Equal(output.Triangles.Count,mesh.Triangles.Count);
        Assert.True(MeshAnalysis.IsWatertightMesh(mesh.PrecisionPositions,mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(mesh.PrecisionPositions,mesh.Triangles));
        int[] groups=output.TriangleGroups.Distinct().OrderBy(i=>i).ToArray();
        foreach(var (cap,group) in new[] { (output.StartPointCap,groups[^2]),(output.EndPointCap,groups[^1]) })
        {
            bool sawSeam=false,sawPole=false;
            for(int t=0;t<output.Triangles.Count;t++)
            {
                if(output.TriangleGroups[t]!=group) continue;
                var triangle=output.Triangles[t];
                int[] corners=[triangle.A,triangle.B,triangle.C];
                var uv=corners.Select(i=>output.UVs[i]).ToArray();
                Assert.InRange(uv.Max(p=>p.X)-uv.Min(p=>p.X),0,.125+1e-12);
                foreach(int i in corners)
                    Assert.InRange((cap.Evaluate(output.UVs[i].X,output.UVs[i].Y)-output.Vertices[i]).Length(),0,.005);
                sawSeam |= uv.Any(p=>p.X==1);
                var pole=uv.Where(p=>p.Y==1).ToArray();
                if(pole.Length==1)
                {
                    var rim=uv.Where(p=>p.Y<1).OrderBy(p=>p.X).ToArray();
                    Assert.Equal((rim[0].X+rim[1].X)*.5,pole[0].X,12);
                    sawPole=true;
                }
            }
            Assert.True(sawSeam && sawPole);
        }
    }

    [Fact]
    public void RoundedCapKeepsExactRimAndBoundaryTangentPlane()
    {
        var output=Build(new LoftOptions { EndPoint=new(0,0,14) });
        var side=Assert.IsType<BSplineSurface>(output.LoftSideSupportFactory());
        var cap=output.EndPointCap;
        foreach (double u in new[] { .02,.125,.31,.6,.95 })
        {
            Assert.InRange((cap.Evaluate(u,0)-side.Evaluate(u,1)).Length(),0,1e-11);
            Assert.InRange(Math.Abs(Vec3DOps.Dot(cap.EvaluateNormal(u,0).Normalized(),
                side.EvaluateNormal(u,1).Normalized())),1-1e-10,1+1e-10);
            Assert.InRange(Vec3DOps.Cross(cap.Surface.EvaluateDV(u,0),side.EvaluateDV(u,1)).Length(),0,1e-10);
        }
    }

    [Fact]
    public void InvalidPointClosureDoesNotFabricateASectionOrOpenShell()
    {
        Assert.Throws<ArgumentException>(() => Build(new LoftOptions { EndPoint=new(0,0,10) }));
        Assert.Throws<ArgumentException>(() => Build(new LoftOptions { EndPoint=new(0,0,9) }));
        Assert.Throws<ArgumentException>(() => Build(new LoftOptions { EndPoint=new(double.NaN,0,14) }));
        Assert.Throws<ArgumentException>(() => Build(new LoftOptions { EndPoint=new(0,0,14), CapEnds=false }));
        Assert.Throws<ArgumentException>(() => Build(new LoftOptions { EndPoint=new(20,20,14) }));
        Assert.Throws<ArgumentException>(() => Build(new LoftOptions { StartPointClosure=LoftPointClosure.Sharp }));
        Assert.Throws<ArgumentException>(() => Build(new LoftOptions { EndPointClosure=LoftPointClosure.Rounded }));
    }

    [Fact]
    public void RoundedRectangleSectionsWithDifferentArcWeightsCloseWithoutChangingAuthoredCurves()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-50),new Vec3D(50)),.005);
        var sections=new List<PlotterSketcherCoordSys>();
        for(int p=0;p<3;p++)
        {
            double a=15+p*5,b=9+p,r=6+p,m=Math.Round(r/Math.Sqrt(2),4);
            var sketch=new PlotterSketcherCoordSys("round"+p,new CoordinateSystem(new Vec3D(0,0,p*10)));
            sketch.AddLine(new(-a+r,-b),new(a-r,-b));
            sketch.AddArc(new(a-r,-b),new(a-r+m,-b+r-m),new(a,-b+r));
            sketch.AddLine(new(a,-b+r),new(a,b-r));
            sketch.AddArc(new(a,b-r),new(a-r+m,b-r+m),new(a-r,b));
            sketch.AddLine(new(a-r,b),new(-a+r,b));
            sketch.AddArc(new(-a+r,b),new(-a+r-m,b-r+m),new(-a,b-r));
            sketch.AddLine(new(-a,b-r),new(-a,-b+r));
            sketch.AddArc(new(-a,-b+r),new(-a+r-m,-b+r-m),new(-a+r,-b));
            sections.Add(sketch);
        }
        var options=new LoftOptions { CorrespondenceMode=LoftCorrespondenceMode.MatchingVertices,
            StartPoint=new(0,0,-8),EndPoint=new(0,0,28) };
        var output=new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(api.Converter,sections,.05,options,output,"board",out _);
        Assert.True(MeshAnalysis.IsWatertightMesh(output.PrecisePositions,output.Triangles));
        var side=Assert.IsType<BSplineSurface>(output.LoftSideSupportFactory());
        for(int p=0;p<3;p++)
        {
            var expected=Geo.NurbsConstruction.NurbsSurfaceFactory.ConcatenateCompatible(sections[p].GetCurves().Single()
                .Select(curve=>Geo.NurbsConstruction.Curve2DToBSpline.ToBSplineCurve(curve,sections[p].CoordinateSystem)).ToArray());
            for(int i=0;i<=64;i++)
                Assert.InRange((side.Evaluate(i/64.0,p/2.0)-expected.EvaluateUniform(i/64.0)).Length(),0,1e-9);
        }
    }

    [Fact]
    public void RealBoardRoundedEndsRetainTheirFiniteSectionsAndCloseSmoothly()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-50),new Vec3D(50)),.01);
        (double x,double z,double width,double height)[] data=[(-220,785,115,17.5),(-150,778,190,20),
            (0,778,220,20),(350,778,225,20),(570,778,180,18)];
        var sections=new List<PlotterSketcherCoordSys>();
        foreach(var (x,z,a,b) in data)
        {
            double r=8,m=5.6569;
            Vec2D P(double y,double height)=>new(Math.Round(y,4),Math.Round(height,4));
            var sketch=new PlotterSketcherCoordSys("board"+x,new CoordinateSystem(new(x,0,z),new(0,1,0),new(0,0,1),new(1,0,0)));
            sketch.AddLine(P(-a+r,-b),P(a-r,-b));
            sketch.AddArc(P(a-r,-b),P(a-r+m,-b+r-m),P(a,-b+r));
            sketch.AddLine(P(a,-b+r),P(a,b-r));
            sketch.AddArc(P(a,b-r),P(a-r+m,b-r+m),P(a-r,b));
            sketch.AddLine(P(a-r,b),P(-a+r,b));
            sketch.AddArc(P(-a+r,b),P(-a+r-m,b-r+m),P(-a,b-r));
            sketch.AddLine(P(-a,b-r),P(-a,-b+r));
            sketch.AddArc(P(-a,-b+r),P(-a+r-m,-b+r-m),P(-a+r,-b));
            sections.Add(sketch);
        }
        var solid=api.Loft(sections,new LoftOptions { CorrespondenceMode=LoftCorrespondenceMode.MatchingVertices,
            StartPoint=new(-250,0,790),EndPoint=new(650,0,780) },"board",.01);
        Assert.True(MeshAnalysis.IsWatertightMesh(solid.Mesh.PrecisionPositions,solid.Mesh.Triangles));
        Assert.InRange((solid.surfaceMetaData["board-StartCap"].NurbsSurface.Evaluate(.2,1)-new Vec3D(-250,0,790)).Length(),0,1e-10);
    }
}
