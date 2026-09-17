using Geo;
using GeoCore;

namespace GeoTests;

public class AssemblyMirrorTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    static GeoAPI Api() => new(new Box3D(new Vec3D(-100),new Vec3D(100)),.001);
    static Vec3D Reflect(Vec3D point,CoordinateSystem plane)
        => point-plane.Z*(2*Vec3DOps.Dot(plane.Z,point-plane.Origin));
    static List<Vec3D> WorldPoints(Assembly root,AssemblyPart part)
        => part.Mesh.SnapshotRigidPose(root.WorldPoseOf(part)).Positions;
    static void AssertPoints(IEnumerable<Vec3D> expected,IEnumerable<Vec3D> actual)
    {
        foreach(var point in expected) Assert.Contains(actual,p=>(point-p).Length()<1e-8);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MirroredSolvedBodyUsesRestGeometryAndKeepsCorrectWorldHandedness(bool tilted)
    {
        var api=Api();
        var mesh=api.CreateCuboid(new CoordinateSystem(new Vec3D(1,2,3)),new Vec3D(1,2,4),"asymmetric");
        var assembly=api.GetAssembly("parent");assembly.SolveAfterEveryConstraint=false;
        var seed=assembly.AddPart(mesh,new Vec3D(7,-5,9),new Quaternion(0,0,Math.Sin(.23),Math.Cos(.23)));
        assembly.FixPart(seed);Assert.True(assembly.SolveConstraintsDetailed().Converged);
        var before=WorldPoints(assembly,seed);
        var displayedBefore=seed.Mesh.Mesh.Positions.ToArray();
        var plane=tilted?new CoordinateSystem(new Vec3D(3,2,1),new Vec3D(1,0,0),new Vec3D(0,.8,.6),new Vec3D(0,-.6,.8))
            :new CoordinateSystem(new Vec3D(0,0,2));
        var copy=assembly.Mirror(seed,plane,"opposite");
        Assert.True(assembly.SolveConstraintsDetailed().Converged);
        Assert.NotSame(seed.Mesh,copy.Mesh);
        AssertPoints(before,WorldPoints(assembly,seed));
        AssertPoints(displayedBefore,seed.Mesh.Mesh.Positions);
        AssertPoints(before.Select(p=>Reflect(p,plane)),WorldPoints(assembly,copy));
        AssertPoints(displayedBefore.Select(p=>Reflect(p,plane)),copy.Mesh.Mesh.Positions);
        var local=copy.Mesh.SnapshotRigidPose(new Transform(new Vec3D(0),TransformMath.IdentityOrientation));
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(local.Positions,local.Triangles)>0);
        Assert.Equal(4,assembly.GetMateRecords().Count);
        Assert.Equal(new[]{AssemblyMateKind.Concentric,AssemblyMateKind.CoincidentPlanes,AssemblyMateKind.CoincidentPlanes},
            assembly.GetMateRecords().Skip(1).Select(m=>m.Kind));
        foreach(var meta in copy.Mesh.surfaceMetaData.Values)
            if(meta.PlaneParams != null) Assert.True(double.IsFinite(meta.PlaneParams.Origin.X));
        var restored=assembly.Mirror(copy,plane,"restored");
        Assert.True(assembly.SolveConstraintsDetailed().Converged);
        AssertPoints(before,WorldPoints(assembly,restored));
        AssertPoints(displayedBefore,restored.Mesh.Mesh.Positions);
    }

    [Fact]
    public void MirroredHierarchyPreservesRevoluteMatesAndIndependentChildMotion()
    {
        var api=Api();
        var mesh=api.CreateCuboid(new CoordinateSystem(new Vec3D(1,0,0)),new Vec3D(1,2,3),"body");
        var support=api.GetAssembly("support");support.SolveAfterEveryConstraint=false;
        var fixedBody=support.AddPart(mesh,new Vec3D(0));support.FixPart(fixedBody);
        var mechanism=api.GetAssembly("mechanism");mechanism.SolveAfterEveryConstraint=false;
        var housing=mechanism.AddSubAssembly(support,new Vec3D(0));mechanism.FixSubAssembly(housing);
        var rotor=mechanism.AddPart(mesh,new Vec3D(0,0,5));
        mechanism.SetConcentric(fixedBody.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)),rotor.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)));
        mechanism.SetDistance(rotor.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)),fixedBody.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)),5);
        Assert.True(mechanism.SolveConstraintsDetailed().Converged);
        var parent=api.GetAssembly("parent");parent.SolveAfterEveryConstraint=false;
        var seed=parent.AddSubAssembly(mechanism,new Vec3D(10,2,4));parent.FixSubAssembly(seed);
        Assert.True(parent.SolveConstraintsDetailed().Converged);
        var sourcePoints=WorldPoints(parent,rotor);
        var mirror=parent.Mirror(seed,CoordinateSystem.Default,"opposite_mechanism");
        Assert.Single(mirror.GetSubAssemblies());
        Assert.NotSame(mechanism,mirror.Child);
        Assert.Equal(mechanism.GetMateRecords().Select(m=>m.Kind),mirror.Child.GetMateRecords().Select(m=>m.Kind));
        Assert.True(parent.SolveConstraintsDetailed().Converged);
        var mirroredRotor=mirror.GetParts()[0];
        AssertPoints(sourcePoints.Select(p=>new Vec3D(p.X,p.Y,-p.Z)),WorldPoints(parent,mirroredRotor));
        var mirroredHousing=mirror.GetSubAssemblies()[0].GetParts()[0];
        var axisMate=mirror.Child.GetMateRecords().Single(m=>m.Kind==AssemblyMateKind.Concentric);
        Assert.Equal(new Vec3D(0,0,-1),axisMate.DirA);
        Assert.Equal(5,mirror.Child.GetMateRecords().Single(m=>m.Kind==AssemblyMateKind.DistancePlanes).Scalar);
        mirror.Child.SetAngle(mirroredHousing.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),
            mirroredRotor.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),.4);
        Assert.True(mirror.Child.SolveConstraintsDetailed().Converged);
        Assert.InRange(Math.Abs(TransformMath.TransformDirection(mirroredRotor.EvaluatePose(),new Vec3D(1,0,0)).X-Math.Cos(.4)),0,1e-7);
        Assert.Equal(new Vec3D(1,0,0),TransformMath.TransformDirection(rotor.EvaluatePose(),new Vec3D(1,0,0)));
    }

    [Fact]
    public void EmptyOrForeignSeedsRejectBeforeAddingMirrorInstances()
    {
        var api=Api();var parent=api.GetAssembly("parent");
        var empty=parent.AddSubAssembly(api.GetAssembly("empty"),new Vec3D(0));
        Assert.Throws<ArgumentException>(()=>parent.Mirror(empty,CoordinateSystem.Default));
        Assert.Single(parent.GetSubAssemblies());
        var foreign=api.GetAssembly("foreign");
        var part=foreign.AddPart(api.CreateCube(CoordinateSystem.Default,1,"seed"),new Vec3D(0));
        Assert.Throws<ArgumentException>(()=>parent.Mirror(part,CoordinateSystem.Default));
        Assert.Empty(parent.GetParts());Assert.Empty(parent.GetMateRecords());
    }
}
