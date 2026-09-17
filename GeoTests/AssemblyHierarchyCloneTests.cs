using Geo;
using GeoCore;
namespace GeoTests;
public class AssemblyHierarchyCloneTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void CloneKeepsNestedOccurrenceFixAndExplicitPoseWithoutFixingOtherBodies()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-50),new Vec3D(50)),.01);
        var root = api.GetAssembly("source"); root.SolveAfterEveryConstraint=false;
        var child=api.GetAssembly("child"); child.SolveAfterEveryConstraint=false;
        var body=api.CreateCube(new CoordinateSystem(new Vec3D(0)),1,"cube");
        var leaf=child.AddPart(body,new Vec3D(2,3,4));
        child.FixPart(leaf,new Vec3D(3,4,5),new Quaternion(0,0,0,1));
        var occurrence=root.AddSubAssembly(child,new Vec3D(10,0,0));
        root.FixSubAssembly(occurrence);
        var free=root.AddPart(body,new Vec3D(0,7,0));
        root.SetConcentric(leaf.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)),
            free.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)));
        var before=leaf.EvaluatePose();
        var copy=root.CloneHierarchy("copy");
        Assert.Single(copy.GetParts());Assert.Single(copy.GetSubAssemblies());
        var copiedChild=copy.GetSubAssemblies()[0].Child;
        Assert.NotSame(child,copiedChild);Assert.Same(body,copiedChild.GetParts()[0].Mesh);
        Assert.Equal(before,copiedChild.GetParts()[0].EvaluatePose());
        Assert.Equal(occurrence.EvaluatePose(),copy.GetSubAssemblies()[0].EvaluatePose());
        Assert.Equal(root.GetMateRecords().Select(m=>m.Kind),copy.GetMateRecords().Select(m=>m.Kind));
        Assert.Same(copy.GetSubAssemblies()[0],copy.GetMateRecords()[0].FixedOccurrence);
        Assert.Single(copiedChild.GetMateRecords());
        Assert.Same(root,child.Parent);Assert.Equal(before,leaf.EvaluatePose());
        Assert.True(copiedChild.SolveConstraintsDetailed().Converged);
    }
    [Fact]
    public void ReplayPreservesDirectedPlanesAndMixedExplicitNamedReferences()
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-20),new Vec3D(20)),.01);
        var source=api.GetAssembly("directed");source.SolveAfterEveryConstraint=false;
        var body=api.CreateCube(new CoordinateSystem(new Vec3D(0)),1,"block");
        var a=source.AddPart(body,new Vec3D(0));var b=source.AddPart(body,new Vec3D(0));
        source.FixPart(a);source.FixPart(b);
        source.SetCoincidentOriented(a.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)),b.AddPlaneDatum("block-ExtrudeBottom"),true);
        source.SetContact(a.AddPointDatumAt(new Vec3D(0)),b.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)));
        var copy=source.CloneHierarchy("copy");
        Assert.Equal(-1,copy.GetMateRecords()[2].Scalar);
        Assert.Equal(source.GetMateRecords()[2].Entities,copy.GetMateRecords()[2].Entities);
        Assert.Equal(source.GetMateResiduals().Select(m=>m.MaxResidual),copy.GetMateResiduals().Select(m=>m.MaxResidual));
        Assert.Throws<ArgumentException>(()=>source.CloneHierarchy("copy"));
        Assert.Equal("copy",copy.Name);
        var reflected=source.CloneHierarchy("reflected",true);
        var reflectedB=reflected.GetParts()[1];
        string entity=reflected.GetMateRecords()[2].Entities.Single();
        Assert.StartsWith(reflectedB.Mesh.Name + ":" + reflectedB.Mesh.Name,entity);
        var named=reflectedB.AddPlaneDatum(entity.Substring(entity.IndexOf(':')+1));
        Assert.Equal(reflected.GetMateRecords()[2].DirB,named.LocalNormal);
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EveryMateKindRetainsItsEquationValuesAcrossClone(bool mirrored)
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-20),new Vec3D(20)),.01);
        var source=api.GetAssembly("all_mates");source.SolveAfterEveryConstraint=false;
        var body=api.CreateCube(new CoordinateSystem(new Vec3D(0)),1,"all_body");
        var a=source.AddPart(body,new Vec3D(1,2,3));var b=source.AddPart(body,new Vec3D(2,3,5));
        source.FixPart(a);source.FixPart(b);
        var pa=a.AddPointDatumAt(new Vec3D(1,0,0));var pb=b.AddPointDatumAt(new Vec3D(0,1,0));
        // An explicit datum sets a common normalization length, independent of
        // source display bounds versus reflected exact-rest mesh bounds.
        var aa=a.AddAxisDatumAt(new Vec3D(0,0,100),new Vec3D(0,0,1));var ab=b.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0));
        var fa=a.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1));var fb=b.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,-1));
        source.SetCoincident(pa,pb);source.SetCoincident(aa,ab);source.SetCoincident(fa,fb);
        source.SetCoincidentOriented(fa,fb,true);source.SetParallel(aa,ab);source.SetParallel(fa,fb);
        source.SetPerpendicular(aa,ab);source.SetPerpendicular(fa,fb);source.SetConcentric(aa,ab);
        source.SetDistance(pa,pb,4);source.SetDistance(fa,fb,-2);source.SetAngle(aa,ab,.7);
        source.SetPointOnPlane(pa,fb);source.SetContact(pa,fb);
        var copy=source.CloneHierarchy("all_copy",mirrored);
        Assert.Equal(Enum.GetValues<AssemblyMateKind>().Length,copy.GetMateRecords().Select(m=>m.Kind).Distinct().Count());
        var original=source.GetMateResiduals();var cloned=copy.GetMateResiduals();
        Assert.Equal(original.Count,cloned.Count);
        for(int j=0;j<original.Count;j++)
        {
            Assert.Equal(original[j].Kind,cloned[j].Kind);
            // Reflection may change individual signed basis equations, but preserves their norm.
            Assert.True(Math.Abs(original[j].Residuals.Sum(v=>v*v)-cloned[j].Residuals.Sum(v=>v*v))<1e-10, $"{original[j].Kind}: source length {source.CharacteristicLength}, clone {copy.CharacteristicLength}; {original[j].MaxResidual} vs {cloned[j].MaxResidual}");
        }
    }

}
