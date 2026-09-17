using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class BooleanSharedPatchTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void CavityCanBeCheckedAgainstItsOriginalTool()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-20),new Vec3D(20)),.01);
        var blank=api.CreateCuboid(new CoordinateSystem(new Vec3D(0)),new Vec3D(8,8,8),"Blank");
        var tool=api.CreateCuboid(new CoordinateSystem(new Vec3D(2,2,-1)),new Vec3D(3,3,10),"Tool");
        var cavity=api.Boolean(blank,tool,BooleanOp.Difference,"Cavity");
        var overlap=api.Boolean(cavity,tool,BooleanOp.Intersect,"Clearance");
        Assert.Empty(overlap.Mesh.Triangles);
        var restored=api.Boolean(cavity,tool,BooleanOp.Union,"Restored");
        Assert.True(MeshAnalysis.IsWatertightMesh(restored.Mesh.Positions,restored.Mesh.Triangles));
    }
}
