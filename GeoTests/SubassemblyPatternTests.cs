using Geo;
using GeoCore;
namespace GeoTests;
public class SubassemblyPatternTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PatternsPreserveNestedRevoluteMatesAndIndependentBodies(bool circular)
    {
        var api=new GeoAPI(new Box3D(new Vec3D(-100),new Vec3D(100)),.01);
        var housingMesh=api.CreateCube(CoordinateSystem.Default,2,"housing");
        var rotorMesh=api.CreateCube(CoordinateSystem.Default,1,"rotor");
        var housing=api.GetAssembly("housing_group");housing.SolveAfterEveryConstraint=false;
        var fixedPart=housing.AddPart(housingMesh,new Vec3D(0));housing.FixPart(fixedPart);
        var mechanism=api.GetAssembly("mechanism");mechanism.SolveAfterEveryConstraint=false;
        var housingOccurrence=mechanism.AddSubAssembly(housing,new Vec3D(0));mechanism.FixSubAssembly(housingOccurrence);
        var rotor=mechanism.AddPart(rotorMesh,new Vec3D(0));
        mechanism.SetConcentric(fixedPart.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)),rotor.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)));
        mechanism.SetCoincidentOriented(fixedPart.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)),rotor.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)),false);
        Assert.True(mechanism.SolveConstraintsDetailed().Converged);
        var parent=api.GetAssembly("parent");parent.SolveAfterEveryConstraint=false;
        var seed=parent.AddSubAssembly(mechanism,new Vec3D(10,0,0));
        var copies=circular?parent.PatternCircular(seed,3,CoordinateSystem.Default,Math.PI):parent.PatternLinear(seed,3,new Vec3D(10,0,0));
        Assert.Same(seed,copies[0]);Assert.Same(parent,mechanism.Parent);
        Assert.NotSame(mechanism,copies[1].Child);
        Assert.Single(copies[1].GetSubAssemblies());
        Assert.Equal(mechanism.GetMateRecords().Select(m=>m.Kind),copies[1].Child.GetMateRecords().Select(m=>m.Kind));
        parent.FixSubAssembly(seed);Assert.True(parent.SolveConstraintsDetailed().Converged);
        var expected=circular?new Vec3D(-10,0,0):new Vec3D(30,0,0);
        Assert.InRange((copies[2].EvaluatePose().Position-expected).Length(),0,1e-6);
        var clone=copies[1].Child;var clonedRotor=clone.GetParts()[0];var clonedHousing=clone.GetSubAssemblies()[0].GetParts()[0];
        clone.SetAngle(clonedHousing.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),clonedRotor.AddAxisDatumAt(new Vec3D(0),new Vec3D(1,0,0)),.3);
        Assert.True(clone.SolveConstraintsDetailed().Converged);
        var direction=TransformMath.TransformDirection(clonedRotor.EvaluatePose(),new Vec3D(1,0,0));
        Assert.InRange(Math.Abs(direction.X-Math.Cos(.3)),0,1e-7);
        Assert.Equal(new Vec3D(1,0,0),TransformMath.TransformDirection(rotor.EvaluatePose(),new Vec3D(1,0,0)));
    }

    [Fact]
    public void SameSubAssemblyDefinitionCanBePlacedAndMatedIndependently()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01);
        var mesh = api.CreateCube(CoordinateSystem.Default, 2, "module_body");
        var module = api.GetAssembly("module");
        var modulePart = module.AddPart(mesh, new Vec3D(0));
        module.FixPart(modulePart);

        var parent = api.GetAssembly("parent");
        parent.SolveAfterEveryConstraint = false;
        var left = parent.AddSubAssembly(module, new Vec3D(10, 0, 0));
        var right = parent.AddSubAssembly(module, new Vec3D(30, 0, 0));

        AssemblyPart leftPart = Assert.Single(left.GetParts());
        AssemblyPart rightPart = Assert.Single(right.GetParts());
        Assert.Same(module, left.Child);
        Assert.Same(module, right.Child);
        Assert.NotSame(leftPart, rightPart);
        Assert.Same(leftPart.Mesh, rightPart.Mesh);

        parent.FixPart(leftPart);
        parent.FixPart(rightPart, new Vec3D(50, 0, 0), TransformMath.IdentityOrientation);
        Assert.True(parent.SolveConstraintsDetailed().Converged);
        Assert.Throws<ArgumentException>(() => parent.WorldPoseOf(modulePart));

        Assert.Equal(10, parent.WorldPoseOf(leftPart).Position.X, 8);
        Assert.Equal(50, parent.WorldPoseOf(rightPart).Position.X, 8);
        Assert.Equal(10, leftPart.EvaluatePose().Position.X, 8);
        Assert.Equal(50, rightPart.EvaluatePose().Position.X, 8);

        AssemblyLeaf[] leaves = parent.GetLeaves().ToArray();
        Assert.Equal(2, leaves.Length);
        Assert.NotEqual(leaves[0].Path, leaves[1].Path);
        Assert.Equal(new[] { 10d, 50d }, leaves.Select(leaf => leaf.Pose.Position.X).Order().ToArray());
        Assert.Equal(2, parent.Section(new CoordinateSystem(new Vec3D(0, 0, 1))).Meshes.Count);
    }

    [Fact]
    public void NestedPartsRemainOccurrenceScopedThroughRepeatedSubassemblies()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01);
        var mesh = api.CreateCube(CoordinateSystem.Default, 1, "nested_body");
        var component = api.GetAssembly("component");
        var part = component.AddPart(mesh, new Vec3D(2, 0, 0));
        component.FixPart(part);

        var module = api.GetAssembly("module");
        var nested = module.AddSubAssembly(component, new Vec3D(3, 0, 0));
        module.FixSubAssembly(nested);

        var parent = api.GetAssembly("parent");
        parent.SolveAfterEveryConstraint = false;
        var left = parent.AddSubAssembly(module, new Vec3D(10, 0, 0));
        var right = parent.AddSubAssembly(module, new Vec3D(30, 0, 0));
        AssemblyPart leftPart = Assert.Single(Assert.Single(left.GetSubAssemblies()).GetParts());
        AssemblyPart rightPart = Assert.Single(Assert.Single(right.GetSubAssemblies()).GetParts());

        parent.FixPart(leftPart);
        parent.FixPart(rightPart, new Vec3D(60, 0, 0), TransformMath.IdentityOrientation);
        Assert.True(parent.SolveConstraintsDetailed().Converged);
        Assert.Equal(15, parent.WorldPoseOf(leftPart).Position.X, 8);
        Assert.Equal(60, parent.WorldPoseOf(rightPart).Position.X, 8);
        Assert.Equal(new[] { 15d, 60d }, parent.GetLeaves().Select(leaf => leaf.Pose.Position.X).Order().ToArray());
    }

    [Fact]
    public void NestedRepeatedDefinitionsKeepDistinctOccurrencePaths()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01);
        var mesh = api.CreateCube(CoordinateSystem.Default, 1, "nested_body");
        var component = api.GetAssembly("component");
        var part = component.AddPart(mesh, new Vec3D(2, 0, 0));
        component.FixPart(part);

        var module = api.GetAssembly("module");
        var first = module.AddSubAssembly(component, new Vec3D(3, 0, 0));
        var second = module.AddSubAssembly(component, new Vec3D(6, 0, 0));
        module.FixSubAssembly(first);
        module.FixSubAssembly(second);

        var parent = api.GetAssembly("parent");
        parent.SolveAfterEveryConstraint = false;
        var left = parent.AddSubAssembly(module, new Vec3D(10, 0, 0));
        var right = parent.AddSubAssembly(module, new Vec3D(30, 0, 0));
        AssemblyPart[] leftParts = left.GetSubAssemblies().SelectMany(occ => occ.GetParts()).ToArray();
        AssemblyPart[] rightParts = right.GetSubAssemblies().SelectMany(occ => occ.GetParts()).ToArray();

        Assert.Equal(2, leftParts.Length);
        Assert.Equal(2, rightParts.Length);
        parent.FixPart(leftParts[0]);
        parent.FixPart(rightParts[1], new Vec3D(60, 0, 0), TransformMath.IdentityOrientation);
        Assert.True(parent.SolveConstraintsDetailed().Converged);
        Assert.Equal(15, parent.WorldPoseOf(leftParts[0]).Position.X, 8);
        Assert.Equal(18, parent.WorldPoseOf(leftParts[1]).Position.X, 8);
        Assert.Equal(57, parent.WorldPoseOf(rightParts[0]).Position.X, 8);
        Assert.Equal(60, parent.WorldPoseOf(rightParts[1]).Position.X, 8);
        Assert.Equal(new[] { 15d, 18d, 57d, 60d },
            parent.GetLeaves().Select(leaf => leaf.Pose.Position.X).Order().ToArray());

        Assembly cloned = module.CloneHierarchy("repeated_module_clone");
        Assert.Equal(new[] { 5d, 8d }, cloned.GetLeaves()
            .Select(leaf => leaf.Pose.Position.X).Order().ToArray());
    }

    [Fact]
    public void FlexibleOccurrencesHaveIndependentInternalMatesAndGeometry()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01);
        var baseMesh = api.CreateCube(CoordinateSystem.Default, 2, "flex_base");
        var rotorMesh = api.CreateCube(CoordinateSystem.Default, 1, "flex_rotor");
        var mechanism = api.GetAssembly("flex_mechanism");
        mechanism.SolveAfterEveryConstraint = false;
        var basePart = mechanism.AddPart(baseMesh, new Vec3D(0));
        var rotor = mechanism.AddPart(rotorMesh, new Vec3D(0));
        mechanism.FixPart(basePart);
        mechanism.SetConcentric(basePart.AddAxisDatumAt(new Vec3D(0), new Vec3D(0, 0, 1)),
            rotor.AddAxisDatumAt(new Vec3D(0), new Vec3D(0, 0, 1)));
        mechanism.SetCoincidentOriented(basePart.AddPlaneDatumAt(new Vec3D(0), new Vec3D(0, 0, 1)),
            rotor.AddPlaneDatumAt(new Vec3D(0), new Vec3D(0, 0, 1)), false);
        Assert.True(mechanism.SolveConstraintsDetailed().Converged);
        api.GetAssembly("rigid_parent").AddSubAssembly(mechanism, new Vec3D(0));

        var parent = api.GetAssembly("flex_parent");
        parent.SolveAfterEveryConstraint = false;
        var first = parent.AddSubAssembly(mechanism, new Vec3D(10, 0, 0), flexible: true);
        var second = parent.AddSubAssembly(mechanism, new Vec3D(30, 0, 0), flexible: true);
        Assert.True(first.IsFlexible);
        Assert.True(second.IsFlexible);
        Assert.Equal("flex_mechanism", first.Name);
        Assert.Equal(first.Name, second.Name);
        Assert.NotSame(mechanism, first.Child);
        Assert.NotSame(first.Child, second.Child);
        Assert.NotSame(first.GetParts()[0].Mesh, second.GetParts()[0].Mesh);

        AssemblyPart firstBase = first.GetParts()[0];
        AssemblyPart firstRotor = first.GetParts()[1];
        first.Child.SetAngle(firstBase.AddAxisDatumAt(new Vec3D(0), new Vec3D(1, 0, 0)),
            firstRotor.AddAxisDatumAt(new Vec3D(0), new Vec3D(1, 0, 0)), .4);
        Assert.True(first.Child.SolveConstraintsDetailed().Converged);

        Vec3D firstX = TransformMath.TransformDirection(firstRotor.EvaluatePose(), new Vec3D(1, 0, 0));
        Vec3D secondX = TransformMath.TransformDirection(second.GetParts()[1].EvaluatePose(), new Vec3D(1, 0, 0));
        Vec3D definitionX = TransformMath.TransformDirection(rotor.EvaluatePose(), new Vec3D(1, 0, 0));
        Assert.InRange(Math.Abs(firstX.Y - Math.Sin(.4)), 0, 1e-7);
        Assert.InRange(Math.Abs(secondX.Y), 0, 1e-7);
        Assert.InRange(Math.Abs(definitionX.Y), 0, 1e-7);
        Assert.Equal(new[] { 10d, 30d }, parent.GetLeaves()
            .Where(leaf => leaf.Part.Mesh.Name.Contains("flex_base"))
            .Select(leaf => leaf.Pose.Position.X).Order().ToArray());
    }
}
