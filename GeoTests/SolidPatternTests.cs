using CSG;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public class SolidPatternTests
{
    static GeoAPI Api() => new(new Box3D(new Vec3D(-20), new Vec3D(20)), .001);
    static BigRationalHybrid Volume6(AnchorMesh solid)
    {
        var result = new BigRationalHybrid(0);
        foreach (var t in solid.Mesh.Triangles)
        {
            var p = solid.Mesh.PrecisionPositions;
            result += Rat3Hybrid.Dot(p[t.A], Rat3Hybrid.Cross(p[t.B], p[t.C]));
            result.Simplify();
        }
        return result;
    }
    static void ExactEqual(BigRationalHybrid a, BigRationalHybrid b) => Assert.True((a-b).Sign() == 0);

    [Fact]
    public void LinearIncludesUnchangedSeedAndPreservesExactGeometry()
    {
        var api = Api();
        var seed = api.CreateCuboid(new CoordinateSystem(new Vec3D(1,2,3)),new Vec3D(1,2,3),"seed");
        var positions = seed.Mesh.PrecisionPositions.ToArray();
        var copies = api.PatternLinear(seed,4,new Vec3D(.123456789,2.3,-.04),"row");
        Assert.Same(seed,copies[0]);
        for (int i=1;i<copies.Count;i++)
        {
            Assert.Equal("row_"+i,copies[i].Name);
            ExactEqual(Volume6(seed),Volume6(copies[i]));
            Assert.Equal(seed.Mesh.Triangles.Count,copies[i].Mesh.Triangles.Count);
            Assert.Empty(seed.groupIdToExtendedName.Values.Intersect(copies[i].groupIdToExtendedName.Values));
        }
        for(int i=0;i<positions.Length;i++) Assert.True(positions[i] == seed.Mesh.PrecisionPositions[i]);
    }

    [Theory]
    [InlineData(2*Math.PI,4)]
    [InlineData(-Math.PI,3)]
    [InlineData(.731,5)]
    public void CircularHasExactRigidVolumeAndExpectedSweep(double angle,int count)
    {
        var api=Api();
        var seed=api.CreateCuboid(new CoordinateSystem(new Vec3D(2,1,0)),new Vec3D(.4,.7,1.1),"seed");
        var axis=new CoordinateSystem(new Vec3D(.17,-.23,.49),new Vec3D(1,0,0),new Vec3D(0,.8,.6),new Vec3D(0,-.6,.8));
        var copies=api.PatternCircular(seed,count,axis,angle,"ring");
        double step=angle/(Math.Abs(angle)==2*Math.PI?count:count-1);
        for(int i=1;i<count;i++)
        {
            ExactEqual(Volume6(seed),Volume6(copies[i]));
            var q=new Quaternion(axis.Z.X*Math.Sin(i*step/2),axis.Z.Y*Math.Sin(i*step/2),axis.Z.Z*Math.Sin(i*step/2),Math.Cos(i*step/2));
            int first = -1;
            for(int j=0;j<seed.Mesh.Positions.Count;j++)
            {
                var point=seed.Mesh.Positions[j];
                var expected=axis.Origin+TransformMath.RotateVector(in q,point-axis.Origin);
                int found=copies[i].Mesh.Positions.FindIndex(p=>(p-expected).Length()<1e-11);
                Assert.True(found>=0);
                if(j==0) first=found;
                var before=seed.Mesh.PrecisionPositions[j]-seed.Mesh.PrecisionPositions[0];
                var after=copies[i].Mesh.PrecisionPositions[found]-copies[i].Mesh.PrecisionPositions[first];
                ExactEqual(Rat3Hybrid.Dot(before,before),Rat3Hybrid.Dot(after,after));
            }
        }
    }

    [Fact]
    public void MirrorIsExactInvolutionWithOutwardNormalsAndSourceMetadataUnchanged()
    {
        var api=Api();
        var source=api.CreateCylinder(new CoordinateSystem(new Vec3D(2,3,1)),.6,1.5,.03,"cylinder");
        var plane=new CoordinateSystem(new Vec3D(.13,-.27,.31),new Vec3D(1,0,0),new Vec3D(0,.8,.6),new Vec3D(0,-.6,.8));
        var saved=source.Mesh.PrecisionPositions.ToArray();
        var mirrored=api.Mirror(source,plane,"mirrored");
        var restored=api.Mirror(mirrored,plane,"restored");
        ExactEqual(Volume6(source),Volume6(mirrored));
        foreach(var p in saved) Assert.Contains(restored.Mesh.PrecisionPositions,q=>p==q);
        for(int i=0;i<saved.Length;i++) Assert.True(saved[i]==source.Mesh.PrecisionPositions[i]);
        for(int i=0;i<mirrored.Mesh.Triangles.Count;i++)
        {
            var triangle=mirrored.Mesh.Triangles[i];
            var positions=mirrored.Mesh.Positions;
            var normal=Vec3DOps.Cross(positions[triangle.B]-positions[triangle.A],positions[triangle.C]-positions[triangle.A]);
            Assert.True(Vec3DOps.Dot(normal,mirrored.Mesh.TrianglesEx[i].V0.Normal)>0);
        }
        foreach(var pair in source.surfaceMetaData)
        {
            var reflected=mirrored.surfaceMetaData[EntityNaming.RewriteMeshNameInEntity(pair.Key,source.Name,"mirrored")];
            Assert.NotSame(pair.Value,reflected);
            if(!pair.Value.HasNurbs) continue;
            var range=pair.Value.ParamRange ?? ParametricRange.UnitSquare;
            double u=(range.UMin+range.UMax)/2,v=(range.VMin+range.VMax)/2;
            var point=pair.Value.NurbsSurface.Evaluate(u,v);
            var expected=point-plane.Z*(2*Vec3DOps.Dot(plane.Z,point-plane.Origin));
            Assert.True((expected-reflected.NurbsSurface.Evaluate(u,v)).Length()<1e-10);
            var n=pair.Value.NurbsSurface.EvaluateNormal(u,v).Normalized();
            var rn=n-plane.Z*(2*Vec3DOps.Dot(plane.Z,n));
            Assert.True((rn-reflected.NurbsSurface.EvaluateNormal(u,v)).Length()<1e-10);
        }
    }

    [Fact]
    public void InvalidPatternsRejectBeforeReplacingExistingSolids()
    {
        var api=Api(); var seed=api.CreateCuboid(CoordinateSystem.Default,new Vec3D(1),"seed");
        Assert.Throws<ArgumentOutOfRangeException>(()=>api.PatternLinear(seed,0,new Vec3D(1)));
        Assert.Throws<ArgumentException>(()=>api.PatternLinear(seed,2,new Vec3D(double.NaN,0,0)));
        Assert.Throws<ArgumentOutOfRangeException>(()=>api.PatternCircular(seed,2,CoordinateSystem.Default,double.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(()=>api.PatternCircular(seed,2,CoordinateSystem.Default,7));
        Assert.Throws<ArgumentException>(()=>api.Mirror(seed,CoordinateSystem.Default,"seed"));
        Assert.Same(seed,api.GetMeshFromName("seed"));
        var blocker=api.CreateCuboid(new CoordinateSystem(new Vec3D(5)),new Vec3D(1),"row_2");
        Assert.Throws<ArgumentException>(()=>api.PatternLinear(seed,3,new Vec3D(2,0,0),"row"));
        Assert.Null(api.GetMeshFromName("row_1"));
        Assert.Same(blocker,api.GetMeshFromName("row_2"));
        Assert.Single(api.PatternLinear(seed,1,new Vec3D(0)));
        Assert.Single(api.PatternCircular(seed,1,CoordinateSystem.Default,0));
    }
}
