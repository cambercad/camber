using Geo;
using GeoCore;
namespace GeoTests;
public class SectionViewTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    static GeoAPI Api() => new(new Box3D(new Vec3D(-128),new Vec3D(128),0),.01,1_048_577);
    static AnchorMesh Cube(GeoAPI api) => api.CreateFromTriangles(
        new List<Vec3D> { new(0,0,0),new(2,0,0),new(2,2,0),new(0,2,0),new(0,0,2),new(2,0,2),new(2,2,2),new(0,2,2) },
        new List<Tri> { new(0,2,1),new(0,3,2),new(4,5,6),new(4,6,7),new(0,1,5),new(0,5,4),new(1,2,6),new(1,6,5),new(2,3,7),new(2,7,6),new(3,0,4),new(3,4,7) },"Cube");
    static double Volume(AnchorMesh solid) => Math.Abs(solid.Mesh.Triangles.Sum(t => {
        var p=solid.Mesh.Positions; return p[t.A].Dot(p[t.B].Cross(p[t.C]))/6; }));

    [Theory]
    [InlineData(-1,0)] [InlineData(0,0)] [InlineData(1,4)] [InlineData(2,8)] [InlineData(3,8)]
    public void HalfspaceBoundaryAndCaps(double z,double expected)
    {
        var api=Api(); var solid=Cube(api); var original=solid.Mesh.PrecisionPositions.ToArray();
        var section=new SectionView(new[]{solid},api.Converter,new CoordinateSystem(new Vec3D(0,0,z)));
        Assert.Equal(expected,section.Meshes.Sum(Volume),8);
        Assert.Equal(original,solid.Mesh.PrecisionPositions);
        foreach(var mesh in section.Meshes)
        {
            Assert.True(MeshAnalysis.IsWatertightMesh(mesh.Mesh.Positions,mesh.Mesh.Triangles));
            if(z==1)
            {
                var hit=section.Raycast(new Vec3D(1,1,3),new Vec3D(0,0,-1));
                Assert.NotNull(hit); Assert.Equal(1,hit.Point.Z,10); Assert.Equal(1,hit.GeometricNormal.Z,10);
                Assert.Contains("section_cap",mesh.groupIdToExtendedName.Values);
            }
        }
    }

    [Fact]
    public void ObliquePlaneRetainsHalfCubeWithoutMutatingSource()
    {
        var api=Api();var solid=Cube(api);var n=new Vec3D(1,1,1).Normalized();var x=new Vec3D(1,-1,0).Normalized();
        var frame=new CoordinateSystem(new Vec3D(1),x,n.Cross(x),n);
        var section=new SectionView(new[]{solid},api.Converter,frame);
        Assert.Equal(4,Volume(Assert.Single(section.Meshes)),7);
        Assert.Equal(8,Volume(solid),10);
        var hit=section.Raycast(new Vec3D(3),-n);Assert.NotNull(hit);Assert.Equal(1,hit.Point.X,8);
    }

    [Fact]
    public void HollowSectionKeepsBoreOpen()
    {
        var api=Api();var outer=api.CreateCylinder(CoordinateSystem.Default,2,4,.02,"outer");
        var bore=api.CreateCylinder(new CoordinateSystem(new Vec3D(0,0,-1)),1,6,.02,"bore");
        var tube=api.Boolean(outer,bore,CSG.BooleanOp.Difference,"tube");
        var section=new SectionView(new[]{tube},api.Converter,new CoordinateSystem(new Vec3D(0,0,2)));
        Assert.Null(section.Raycast(new Vec3D(0,0,5),new Vec3D(0,0,-1)));
        var wall=section.Raycast(new Vec3D(1.5,0,5),new Vec3D(0,0,-1));
        Assert.NotNull(wall);Assert.Equal(2,wall.Point.Z,8);
        Assert.Equal(Volume(tube)/2,Volume(Assert.Single(section.Meshes)),7);
    }
    [Fact]
    public void RepeatedNestedBodiesUseWorldPoseAndDoNotUpdateSharedGeometry()
    {
        var api=Api();var cube=Cube(api);var child=api.GetAssembly("child");
        child.AddPart(cube,new Vec3D(1,0,0));var root=api.GetAssembly("root");
        root.AddSubAssembly(child,new Vec3D(5,0,0),new Quaternion(0,0,1,1));
        var other=api.GetAssembly("other");other.AddPart(cube,new Vec3D(1,0,0));
        root.AddSubAssembly(other,new Vec3D(-5,0,0),new Quaternion(0,0,1,1));
        // Simulate a previously displayed occurrence: the source remains exactly
        // in this prior pose while the snapshot uses each occurrence's rest mesh.
        cube.Update(new Transform(new Vec3D(100,0,0),TransformMath.IdentityOrientation));
        var before=cube.Mesh.PrecisionPositions.ToArray();
        var section=root.Section(new CoordinateSystem(new Vec3D(0,0,1)));
        Assert.Equal(2,section.Meshes.Count);
        Assert.Equal(2,section.Meshes.Select(m=>m.Name).Distinct().Count());
        Assert.All(section.Meshes,m=>Assert.Equal(4,Volume(m),8));
        Assert.NotNull(section.Raycast(new Vec3D(4,2,5),new Vec3D(0,0,-1)));
        Assert.NotNull(section.Raycast(new Vec3D(-6,2,5),new Vec3D(0,0,-1)));
        Assert.Equal(before,cube.Mesh.PrecisionPositions);
    }
}
