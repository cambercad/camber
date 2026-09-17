using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class LoftAnalyticProfileTests
{
    [Theory]
    [InlineData(LoftProfileSamplingSource.AnalyticCurveStrip)]
    [InlineData(LoftProfileSamplingSource.TessellatedPolyline)]
    public void TiltedLoftCapsAreExactlyPlanarAndShareTheirSideRim(LoftProfileSamplingSource sampling)
    {
        var axis = new Vec3D(-45, 0, 151).Normalized();
        var x = Vec3DOps.Cross(new Vec3D(0, 1, 0), axis).Normalized();
        var y = Vec3DOps.Cross(axis, x).Normalized();
        var profiles = new List<PlotterSketcherCoordSys>();
        foreach (double offset in new[] { 0.0, 153.7 })
        {
            var frame = new CoordinateSystem(new Vec3D(12.123, 5.456, 9.789) + axis * offset, x, y, axis);
            var sketch = new PlotterSketcherCoordSys("tilted" + offset, frame);
            sketch.AddEllipse(new Vec2D(0), new Vec2D(22, 0), 17);
            profiles.Add(sketch);
        }
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(MeshTestHelpers.MakeConverter(), profiles, .07,
            new LoftOptions { ProfileSampling = sampling }, output, "planar", out var names);
        foreach (int group in output.TriangleGroups.Distinct().Where(group => !names[group].Contains("Side")))
        {
            var capTriangles = output.Triangles.Where((triangle, index) => output.TriangleGroups[index] == group).ToList();
            var first = capTriangles[0];
            var a = output.PrecisePositions[first.A];
            var normal = Rat3Hybrid.Cross(output.PrecisePositions[first.B] - a, output.PrecisePositions[first.C] - a);
            normal.Simplify();
            Assert.True(normal.X.Sign() != 0 || normal.Y.Sign() != 0 || normal.Z.Sign() != 0);
            foreach (int vertex in capTriangles.SelectMany(triangle => new[] { triangle.A, triangle.B, triangle.C }).Distinct())
                Assert.Equal(0, Rat3Hybrid.Dot(normal, output.PrecisePositions[vertex] - a).Sign());
        }
        var edges = new Dictionary<(Rat3Hybrid, Rat3Hybrid), int>();
        foreach (var triangle in output.Triangles)
            foreach (var pair in new[] { (triangle.A, triangle.B), (triangle.B, triangle.C), (triangle.C, triangle.A) })
            {
                var edge = (output.PrecisePositions[pair.Item1], output.PrecisePositions[pair.Item2]);
                edges[edge] = edges.GetValueOrDefault(edge) + 1;
            }
        foreach (var edge in edges)
            Assert.Equal(edge.Value, edges.GetValueOrDefault((edge.Key.Item2, edge.Key.Item1)));
    }

    [Fact]
    public void CompositeStripEvaluatesInsideEverySegment()
    {
        var strip=CurveStrip2D.FromSegments(new Curve2D[]{
            new Line2D(new Vec2D(0,0),new Vec2D(1,0)),
            new Line2D(new Vec2D(1,0),new Vec2D(1,2)),
            new Line2D(new Vec2D(1,2),new Vec2D(4,2))});
        Assert.True((strip.EvaluateAtNormalizedArcLength(.25).Position-new Vec2D(1,.5)).Length()<1e-12);
        Assert.True((strip.EvaluateAtNormalizedArcLength(.75).Position-new Vec2D(2.5,2)).Length()<1e-12);
        Assert.True((strip.DerivativeAtNormalizedArcLength(.25)-new Vec2D(0,6)).Length()<1e-12);
    }

    [Theory]
    [InlineData(LoftStyle.Ruled)]
    [InlineData(LoftStyle.Hermite)]
    [InlineData(LoftStyle.SmoothCatmullRom)]
    public void TaperedTiltedLoftNormalsFollowActualSurfaceDerivatives(LoftStyle style)
    {
        var frames=new[]{CoordinateSystem.Default,
            new CoordinateSystem(new Vec3D(2,3,10),new Vec3D(1,0,0),new Vec3D(0,Math.Cos(.2),Math.Sin(.2)),new Vec3D(0,-Math.Sin(.2),Math.Cos(.2))),
            new CoordinateSystem(new Vec3D(5,-2,24),new Vec3D(Math.Cos(.3),0,Math.Sin(.3)),new Vec3D(0,1,0),new Vec3D(-Math.Sin(.3),0,Math.Cos(.3)))};
        double[] rx={10,17,7},ry={4,9,6};
        int count=style == LoftStyle.Ruled ? 2 : 3;
        var profiles=new List<PlotterSketcherCoordSys>();
        for(int i=0;i<count;i++)
        {
            var sk=new PlotterSketcherCoordSys("ellipse"+i,frames[i]);
            sk.AddEllipse(new Vec2D(0),new Vec2D(rx[i],0),ry[i]);profiles.Add(sk);
        }
        Vec3D Surface(double u,double v)
        {
            var points=Enumerable.Range(0,count).Select(i=>frames[i].PointTo3D(new Vec2D(rx[i]*Math.Cos(2*Math.PI*u),ry[i]*Math.Sin(2*Math.PI*u)))).ToArray();
            return style == LoftStyle.Ruled ? points[0]*(1-v)+points[1]*v : new CubicHermiteSpline3D(points).Evaluate(v).Origin;
        }
        var output=new MeshOutput();
        var converter = MeshTestHelpers.MakeConverter();
        LoftBuilder.GenerateLoftFromSketches(converter,profiles,.05,
            new LoftOptions {Style=style,CapEnds=false,VSubdivisionsPerSpan=3},output,"tilted",out _);
        var first=new Dictionary<double,Vec3D>();
        for(int i=0;i<output.Vertices.Count;i++)
            if(output.UVs[i].Y==0) first[output.UVs[i].X]=output.Vertices[i];
        for(int i=0;i<output.Vertices.Count;i++)
        {
            var p=first[output.UVs[i].X];
            double u=Math.Atan2(p.Y/ry[0],p.X/rx[0])/(2*Math.PI),v=output.UVs[i].Y;
            const double h=1e-6;
            Vec3D du=(Surface(u+h,v)-Surface(u-h,v))/(2*h);
            double lo=Math.Max(0,v-h),hi=Math.Min(1,v+h);
            Vec3D dv=(Surface(u,hi)-Surface(u,lo))/(hi-lo);
            var expected=Vec3DOps.Cross(du,dv).Normalized();
            Assert.InRange(Vec3DOps.Dot(expected,output.Normals[i]),1-1e-9,1+1e-12);
        }
    }

    [Theory]
    [InlineData(3)]
    [InlineData(-3)]
    public void FlatCapsHaveOutwardNormalsWithoutRoundingSideEdges(double height)
    {
        var profiles=new List<PlotterSketcherCoordSys>();
        foreach(double z in new[]{0.0,height})
        {
            var sk=new PlotterSketcherCoordSys("cap"+z,new CoordinateSystem(new Vec3D(0,0,z),new Vec3D(1,0,0),new Vec3D(0,1,0),new Vec3D(0,0,1)));
            sk.AddEllipse(new Vec2D(0),new Vec2D(2,0),1);profiles.Add(sk);
        }
        var output=new MeshOutput();
        var converter = MeshTestHelpers.MakeConverter();
        LoftBuilder.GenerateLoftFromSketches(converter,profiles,.05,
            new LoftOptions(),output,"capped",out var names);
        for(int t=0;t<output.Triangles.Count;t++)
        {
            var tri=output.Triangles[t];
            bool side=names[output.TriangleGroups[t]].Contains("Side");
            foreach(int i in new[]{tri.A,tri.B,tri.C})
            {
                var p=output.Vertices[i];
                var normal = output.Normals[i];
                if (side)
                {
                    // Recover the ellipse point from its outward analytic normal;
                    // only the emitted position has construction-lattice rounding.
                    double scale = Math.Sqrt(4 * normal.X * normal.X + normal.Y * normal.Y);
                    var onEllipse = new Vec3D(4 * normal.X / scale, normal.Y / scale, p.Z);
                    Assert.InRange((onEllipse - p).Length(), 0, 2 * Math.Sqrt(3) * converter.SmallestUnit());
                    Assert.InRange(Math.Abs(normal.Z), 0, 1e-12);
                    Assert.InRange(normal.Length(), 1 - 1e-12, 1 + 1e-12);
                }
                else
                {
                    bool start = names[output.TriangleGroups[t]].Contains("StartCap");
                    var expected = new Vec3D(0, 0, start ? -Math.Sign(height) : Math.Sign(height));
                    Assert.InRange(Vec3DOps.Dot(expected, normal), 1 - 1e-12, 1 + 1e-12);
                }
            }
        }
    }

    [Fact]
    public void AnalyticThreeSectionCylinderCapsAreClosed()
    {
        var sections=new List<PlotterSketcherCoordSys>();
        foreach(double z in new[]{0.0,1.5,3.0})
        {
            var frame=new CoordinateSystem(new Vec3D(0,0,z),new Vec3D(1,0,0),new Vec3D(0,1,0),new Vec3D(0,0,1));
            var sketch=new PlotterSketcherCoordSys("circle"+z,frame);
            sketch.AddCircle(new Vec2D(0),.5);sections.Add(sketch);
        }
        var output=new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(MeshTestHelpers.MakeConverter(),sections,.01,
            new LoftOptions {VSubdivisionsPerSpan=2},output,"cylinder",out _);
        bool closed=MeshAnalysis.IsWatertightMesh(output.Vertices,output.Triangles,out var edges);
        Assert.True(closed,string.Join("\n",edges.Select(p=>$"{p.X:R},{p.Y:R},{p.Z:R}")));
    }

    [Theory]
    [InlineData(10,10)]
    [InlineData(10,4)]
    public void DefaultLoftVerticesLieOnActualConic(double rx,double ry)
    {
        var profiles=new List<PlotterSketcherCoordSys>();
        foreach(double z in new[]{0.0,20.0})
        {
            var frame=new CoordinateSystem(new Vec3D(0,0,z),new Vec3D(1,0,0),new Vec3D(0,1,0),new Vec3D(0,0,1));
            var sketch=new PlotterSketcherCoordSys("section"+z,frame);
            sketch.AddEllipse(new Vec2D(0),new Vec2D(rx,0),ry);
            profiles.Add(sketch);
        }
        var output=new MeshOutput();
        // Deliberately insert samples between the coarse sketch vertices:
        // this distinguishes analytic evaluation from polygon interpolation.
        var options=new LoftOptions { CapEnds=false, ProfileSamplesU=32 };
        var converter = MeshTestHelpers.MakeConverter();
        LoftBuilder.GenerateLoftFromSketches(converter,profiles,1,options,output,"conic",out _);
        Assert.NotEmpty(output.Vertices);
        double rounding = 2 * Math.Sqrt(3) * converter.SmallestUnit();
        double residualBound = 2 * rounding * (1 / rx + 1 / ry) +
            rounding * rounding * (1 / (rx * rx) + 1 / (ry * ry));
        foreach(var p in output.Vertices)
            Assert.InRange(Math.Abs(p.X*p.X/(rx*rx)+p.Y*p.Y/(ry*ry)-1),0,residualBound);
        // The explicit polygon option remains available for mesh-authored work.
        options.ProfileSampling=LoftProfileSamplingSource.TessellatedPolyline;
        var polygon=new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(MeshTestHelpers.MakeConverter(),profiles,1,options,polygon,"polygon",out _);
        Assert.Contains(polygon.Vertices,p=>Math.Abs(p.X*p.X/(rx*rx)+p.Y*p.Y/(ry*ry)-1)>.01);
    }
}
