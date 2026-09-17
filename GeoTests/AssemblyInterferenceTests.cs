using Geo;
using GeoCore;
using GeoSolver;

namespace GeoTests;

public class AssemblyInterferenceTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    readonly Xunit.Abstractions.ITestOutputHelper output;
    public AssemblyInterferenceTests(Xunit.Abstractions.ITestOutputHelper output) => this.output = output;
    // Binary lattice spacing makes nominal contact exact in these fixtures.
    static GeoAPI Api() => new(new Box3D(new Vec3D(-128),new Vec3D(128),0),.01,1_048_577);

    static AnchorMesh Cube(GeoAPI api) => api.CreateFromTriangles(
        new List<Vec3D> { new(0,0,0),new(2,0,0),new(2,2,0),new(0,2,0),
                         new(0,0,2),new(2,0,2),new(2,2,2),new(0,2,2) },
        new List<Tri> { new(0,2,1),new(0,3,2),new(4,5,6),new(4,6,7),
                        new(0,1,5),new(0,5,4),new(1,2,6),new(1,6,5),
                        new(2,3,7),new(2,7,6),new(3,0,4),new(3,4,7) }, "Cube");

    static BigRationalHybrid SixVolume(MeshNormalUV mesh)
    {
        BigRationalHybrid result = new(0);
        var p = mesh.PrecisionPositions;
        if (p.Count == 0) return result;
        var origin = p[0];
        foreach (var t in mesh.Triangles)
            result += Rat3Hybrid.Dot(p[t.A] - origin,
                Rat3Hybrid.Cross(p[t.B] - origin, p[t.C] - origin));
        result.Simplify();
        return result;
    }

    [Fact]
    public void RotatedShaftInsideClearBoreHasNoInterference()
    {
        var api = Api();
        var outer = api.CreateCylinder(CoordinateSystem.Default, 2, 3, .005, "Outer");
        var bore = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, -1)), 1.5, 5, .005, "Bore");
        var ring = api.Boolean(outer, bore, CSG.BooleanOp.Difference, "Ring");
        var shaft = api.CreateCylinder(CoordinateSystem.Default, 1, 3, .005, "Shaft");
        var pose = new Transform(new Vec3D(13, -7, 4), new Quaternion(.31, .57, .11, .75));
        var assembly = api.GetAssembly("Bearing");
        assembly.AddPart(ring, pose.Position, pose.Orientation);
        assembly.AddPart(shaft, pose.Position, pose.Orientation);
        var parts = new List<AssemblyPart>();
        var poses = new List<Transform>();
        assembly.CollectLeafWorldPoses(parts, poses);
        pose = poses[0];
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var direct = MeshNormalUV.BooleanOperation(ring.SnapshotRigidPose(pose, 0),
            shaft.SnapshotRigidPose(pose, 1), CSG.BooleanOp.Intersect, api.Converter);
        double directMs = clock.Elapsed.TotalMilliseconds;
        clock.Restart();
        Assert.Empty(assembly.Interferences());
        double localMs = clock.Elapsed.TotalMilliseconds;
        Assert.Equal(0, SixVolume(direct).Sign());
        output.WriteLine($"Same geometry: world CSG {directMs:F2} ms; shared-pose audit {localMs:F2} ms.");
    }

    [Theory]
    [InlineData(0.31, 0.57, 0.11, 0.75)]
    [InlineData(0, 0, 0.5, 0.8660254037844386)]
    public void CommonRigidPosePreservesExactIntersectionVolumeAndWorldGeometry(
        double x, double y, double z, double w)
    {
        var api = Api();
        var a = Cube(api);
        var b = api.CreateFromTriangles(a.Mesh.Positions.Select(p => p + new Vec3D(1, 0, 0)).ToList(),
            new List<Tri>(a.Mesh.Triangles), "ShiftedCube");
        var pose = new Transform(new Vec3D(13, -7, 4), new Quaternion(x, y, z, w));
        var assembly = api.GetAssembly("CommonPose");
        assembly.AddPart(a, pose.Position, pose.Orientation);
        assembly.AddPart(b, pose.Position, pose.Orientation);
        var parts = new List<AssemblyPart>();
        var poses = new List<Transform>();
        assembly.CollectLeafWorldPoses(parts, poses);
        pose = poses[0];
        var before = a.Mesh.PrecisionPositions.ToArray();
        var hit = Assert.Single(assembly.Interferences());
        var direct = MeshNormalUV.BooleanOperation(a.SnapshotRigidPose(pose, 0),
            b.SnapshotRigidPose(pose, 1), CSG.BooleanOp.Intersect, api.Converter);
        Assert.Equal(SixVolume(direct), SixVolume(hit.Geometry.Mesh));
        Assert.Equal(4, hit.Volume, 8);
        Assert.True(MeshAnalysis.IsWatertightMesh(hit.Geometry.Mesh.Positions, hit.Geometry.Mesh.Triangles));
        // The transformed overlap must occupy the same world bounds as direct CSG.
        foreach (var coordinate in new Func<Rat3Hybrid, BigRationalHybrid>[] { p => p.X, p => p.Y, p => p.Z })
        {
            var expected = direct.Triangles.SelectMany(t => new[] { t.A, t.B, t.C })
                .Select(i => coordinate(direct.PrecisionPositions[i])).OrderBy(v => v, Comparer<BigRationalHybrid>.Create((a, b) => a.CompareTo(b))).ToArray();
            var actual = hit.Geometry.Mesh.Triangles.SelectMany(t => new[] { t.A, t.B, t.C })
                .Select(i => coordinate(hit.Geometry.Mesh.PrecisionPositions[i])).OrderBy(v => v, Comparer<BigRationalHybrid>.Create((a, b) => a.CompareTo(b))).ToArray();
            Assert.Equal(expected.First(), actual.First());
            Assert.Equal(expected.Last(), actual.Last());
        }
        Assert.Equal(before, a.Mesh.PrecisionPositions);
    }

    [Fact]
    public void UnusedToolVerticesDoNotChangePlacedSurfaceIntersections()
    {
        var api = Api();
        var cube = Cube(api);
        var unused = new Vec3D(120, 120, 120);
        cube.Mesh.Positions.Add(unused);
        var lattice = api.Converter.Convert(unused);
        cube.Mesh.PrecisionPositions.Add(new Rat3Hybrid(lattice.X, lattice.Y, lattice.Z));
        var assembly = api.GetAssembly("UnusedToolVertices");
        assembly.AddPart(cube, new Vec3D(0));
        assembly.AddPart(cube, new Vec3D(1, 0, 0));
        assembly.AddPart(cube, new Vec3D(10, 0, 0));
        var before = cube.Mesh.PrecisionPositions.ToArray();

        var hit = Assert.Single(assembly.Interferences());

        Assert.Equal(4, hit.Volume, 7);
        Assert.Equal(before, cube.Mesh.PrecisionPositions);
    }

    [Fact]
    public void SmallPositiveOverlapIsVisibleUnlessExplicitlyFiltered()
    {
        var api=Api(); var cube=Cube(api); var assembly=api.GetAssembly("SmallOverlap");
        assembly.AddPart(cube,new Vec3D(0));
        assembly.AddPart(cube,new Vec3D(2-1e-6,0,0));
        var hit=Assert.Single(assembly.Interferences());
        Assert.InRange(hit.Volume,3.99e-6,4.01e-6);
        Assert.Empty(assembly.Interferences(1e-5));
    }

    [Fact]
    public void RepeatedSolidOccurrencesAreSortedFilteredAndReadOnly()
    {
        var api=Api();
        var cube=Cube(api);
        var assembly=api.GetAssembly("Repeated");
        assembly.SolveAfterEveryConstraint=false;
        assembly.AddPart(cube,new Vec3D(0));
        assembly.AddPart(cube,new Vec3D(1,0,0));
        assembly.AddPart(cube,new Vec3D(1.75,0,0));
        var before=cube.Mesh.PrecisionPositions.ToArray();
        var hits=assembly.Interferences();
        Assert.Equal(new[]{5d,4d,1d},hits.Select(x=>Math.Round(x.Volume,8)).ToArray());
        Assert.Equal(3,hits.SelectMany(x=>new[]{x.First,x.Second}).Distinct().Count());
        Assert.Equal(before,cube.Mesh.PrecisionPositions);
        Assert.Single(assembly.Interferences(4));
        Assert.Empty(assembly.Interferences(5));
        foreach(var hit in hits)
        {
            Assert.True(MeshAnalysis.IsWatertightMesh(hit.Geometry.Mesh.Positions,hit.Geometry.Mesh.Triangles));
            Assert.Equal(hit.Volume,Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(hit.Geometry.Mesh.Positions,hit.Geometry.Mesh.Triangles)),7);
        }
    }

    [Fact]
    public void SolvedRepeatedOccurrencesUseRestGeometryWithoutMovingTheSource()
    {
        var api = Api();
        var cube = Cube(api);
        var assembly = api.GetAssembly("Solved");
        assembly.SolveAfterEveryConstraint = false;
        assembly.FixPart(assembly.AddPart(cube, new Vec3D(10, 0, 0)));
        assembly.FixPart(assembly.AddPart(cube, new Vec3D(11, 0, 0)));
        assembly.SolveConstraints();
        var before = cube.Mesh.PrecisionPositions.ToArray();

        var hit = Assert.Single(assembly.Interferences());

        Assert.Equal(4, hit.Volume, 7);
        var mesh = hit.Geometry.Mesh;
        var vertices = mesh.Triangles.SelectMany(t => new[] { t.A, t.B, t.C })
            .Distinct().Select(i => mesh.Positions[i]).ToArray();
        Assert.InRange(vertices.Min(p => p.X), 10.999999, 11.000001);
        Assert.InRange(vertices.Max(p => p.X), 11.999999, 12.000001);
        Assert.Equal(before, cube.Mesh.PrecisionPositions);
    }

    [Fact]
    public void NestedRotationUsesComposedPose()
    {
        var api=Api();
        var cube=Cube(api);
        var child=api.GetAssembly("Child"); child.AddPart(cube,new Vec3D(1,0,0));
        var root=api.GetAssembly("Root");
        root.AddSubAssembly(child,new Vec3D(4,0,0),new Quaternion(0,0,1,1));
        root.AddPart(cube,new Vec3D(3,2,0));
        var hit=Assert.Single(root.Interferences());
        Assert.Equal(2,hit.Volume,7);
        Assert.Contains("Child[1]/Cube[1]",hit.Second);
    }

    [Fact]
    public void ContactSeparationAndEmptyAssembliesHaveNoInterference()
    {
        var api=Api(); var assembly=api.GetAssembly("Contact");
        Assert.Empty(assembly.Interferences());
        var cube=Cube(api);
        assembly.AddPart(cube,new Vec3D(0));
        assembly.AddPart(cube,new Vec3D(2,0,0));
        assembly.AddPart(cube,new Vec3D(5,0,0));
        Assert.Empty(assembly.Interferences());
        Assert.Throws<ArgumentOutOfRangeException>(()=>assembly.Interferences(-1));
        Assert.Throws<ArgumentOutOfRangeException>(()=>assembly.Interferences(double.NaN));
    }
}
