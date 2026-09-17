using CSG;
using Geo;
using GeoCore;
namespace GeoTests;
public class TorusInterfaceTests
{
    static double Volume(AnchorMesh m) => Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(m.Mesh.Positions,m.Mesh.Triangles));
    [Theory]
    [InlineData(312, 0, false)]
    [InlineData(312, .7, true)]
    [InlineData(-312, .7, false)]
    [InlineData(12, 0, true)]
    public void NestedCircularRevolveHasInwardCavityAndBoundedIntersection(double radial, double angle, bool holeFirst)
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-400),new Vec3D(400)),.1);
        var frame=new CoordinateSystem(new Vec3D(3,5,7),
            new Vec3D(Math.Sin(angle),0,Math.Cos(angle)),
            new Vec3D(Math.Cos(angle),0,-Math.Sin(angle)),new Vec3D(0,1,0));
        var shell=api.GetPlotterSketcher(frame,"shell_section");
        shell.AddCircle(new Vec2D(0,radial),holeFirst ? .65 : 2);
        shell.AddCircle(new Vec2D(0,radial),holeFirst ? 2 : .65);
        var a=api.Revolve(shell,2*Math.PI,.15,"shell");
        AnchorMesh Torus(double radius,double deviation,string name)
        {
            var sketch=api.GetPlotterSketcher(frame,name+"_section");
            sketch.AddCircle(new Vec2D(0,radial),radius);
            return api.Revolve(sketch,2*Math.PI,deviation,name);
        }
        var outside=Torus(2,.15,"outside");
        var coarseCore=Torus(.65,.15,"coarse_core");
        // Independent homogeneous solids verify that a nested contour removes
        // material, even though every individual shell is closed either way.
        Assert.InRange(Math.Abs(Volume(a)-(Volume(outside)-Volume(coarseCore))),0,1e-6);
        var b=Torus(.65,.02,"core");
        var c=api.Boolean(a,b,BooleanOp.Intersect,"contact");
        Assert.InRange(Volume(c),0,Math.Min(Volume(a),Volume(b)));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(c.Mesh.PrecisionPositions,c.Mesh.Triangles,true));
        c.EnsureCoplanarPostProcessed();
        Assert.InRange(Volume(c),0,Math.Min(Volume(a),Volume(b)));
        MeshTestHelpers.AssertValidMesh(c.Mesh.Positions,c.Mesh.Triangles);
    }
    [Fact]
    public void ReversedMeridianKeepsSourceFaceNames()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-10),new Vec3D(10)),1e-4);
        var sketch=api.GetPlotterSketcher(DefaultPlanes.OriginXY,"section");
        sketch.AddLine(new Vec2D(-1,2),new Vec2D(-1,4));
        sketch.AddLine(new Vec2D(-1,4),new Vec2D(1,4));
        sketch.AddLine(new Vec2D(1,4),new Vec2D(1,2));
        var inner=sketch.AddLine(new Vec2D(1,2),new Vec2D(-1,2));
        var ring=api.Revolve(sketch,2*Math.PI,.01,"ring");
        int face=ring.extendedNameToGroupId["ring-"+inner.Name];
        var vertices=ring.Mesh.Triangles.Where((t,index)=>ring.Mesh.TrianglesEx[index].GroupId==face)
            .SelectMany(t=>new[]{t.A,t.B,t.C}).Distinct().Select(i=>ring.Mesh.Positions[i]).ToList();
        Assert.NotEmpty(vertices);
        Assert.All(vertices,p=>Assert.InRange(Math.Sqrt(p.Y*p.Y+p.Z*p.Z),1.9999,2.0001));
    }

    [Fact]
    public void OppositeMaterialSidesUseIdenticalTorusTriangles()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-350,-350,-30),new Vec3D(350,350,30)),.1);
        var frame=new CoordinateSystem(new Vec3D(0),new Vec3D(0,0,1),new Vec3D(1,0,0),new Vec3D(0,1,0));
        var section=api.GetPlotterSketcher(frame,"section");
        section.AddCircle(new Vec2D(-9.65,312.05),2);
        var cavity=section.AddCircle(new Vec2D(-9.65,312.05),.65);
        var shell=api.Revolve(section,2*Math.PI,.15,"shell");
        var coreSection=api.GetPlotterSketcher(frame,"core_section");
        var coreCurve=coreSection.AddCircle(new Vec2D(-9.65,312.05),.65);
        var core=api.Revolve(coreSection,2*Math.PI,.15,"core");
        static string VertexKey(Rat3Hybrid point) => $"{point.X};{point.Y};{point.Z}";
        static HashSet<string> Facets(AnchorMesh solid,int group) => solid.Mesh.Triangles
            .Where((tri,index)=>solid.Mesh.TrianglesEx[index].GroupId==group)
            .Select(tri=>string.Join("/",new[]{tri.A,tri.B,tri.C}
                .Select(i=>VertexKey(solid.Mesh.PrecisionPositions[i])).OrderBy(key=>key,StringComparer.Ordinal)))
            .ToHashSet();
        Assert.True(Facets(shell,shell.extendedNameToGroupId["shell-"+cavity.Name])
            .SetEquals(Facets(core,core.extendedNameToGroupId["core-"+coreCurve.Name])),
            "Reversed material interfaces must have the same exact triangles, not just matching vertices.");
        var contact=api.Boolean(shell,core,BooleanOp.Intersect,"contact");
        contact.EnsureCoplanarPostProcessed();
        Assert.Empty(contact.Mesh.Triangles);
    }

}
