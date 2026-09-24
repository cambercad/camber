using Geo;
using GeoCore;

namespace GeoTests;

[Collection("GlobalCadState")]
public sealed class ShellTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlanarCuboid_ClosedShellWithoutOpening(bool passEmptyList)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .001);
        var block = api.CreateCuboid(new Vec3D(0, 0, 0), new Vec3D(10, 8, 6), "block");
        var result = passEmptyList ? api.Shell(block, 1, new List<string>(), name: "closedShell")
            : api.Shell(block, 1, name: "closedShell");

        Assert.True(result.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles),
            287.9, 288.1); // 10*8*6 - 8*6*4
        Assert.DoesNotContain(result.extendedNameToGroupId.Keys, name => name.StartsWith("ShellRim_"));
        Assert.Contains("ShellInner_block-ExtrudeTop", result.extendedNameToGroupId.Keys);
    }

    [Fact]
    public void PlanarCuboid_ShellsThroughNamedTopFace()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .001);
        var block = api.CreateCuboid(new Vec3D(0, 0, 0), new Vec3D(10, 8, 6), "block");
        var result = api.Shell(block, 1, new List<string> { "block-ExtrudeTop" }, name: "shell");

        Assert.True(result.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        Assert.InRange(Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles)),
            239.9, 240.1); // 10*8*6 - 8*6*5
        Assert.Contains("ShellInner_block-ExtrudeBottom", result.extendedNameToGroupId.Keys);
        Assert.Contains("ShellRim_block-ExtrudeTop", result.extendedNameToGroupId.Keys);
    }

    [Theory]
    [InlineData(false, 480)]
    [InlineData(true, 400)]
    public void PlanarCuboid_OutwardShellPreservesOriginalCavity(bool openTop, double expectedVolume)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .001);
        var block = api.CreateCuboid(new Vec3D(0, 0, 0), new Vec3D(10, 8, 6), "block");
        var faces = openTop ? new List<string> { "block-ExtrudeTop" } : new List<string>();

        var result = api.Shell(block, 1, faces, name: "outward", outward: true);

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles, true));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles),
            expectedVolume - .01, expectedVolume + .01);
        Assert.Contains("ShellInner_block-ExtrudeBottom", result.extendedNameToGroupId.Keys);
        Assert.Equal(openTop, result.extendedNameToGroupId.ContainsKey("ShellRim_block-ExtrudeTop"));
        var used = result.Mesh.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct().Select(i => result.Mesh.Positions[i]).ToArray();
        Assert.InRange(used.Min(p => p.X), -1.001, -.999);
        Assert.InRange(used.Max(p => p.X), 10.999, 11.001);
        Assert.InRange(used.Min(p => p.Z), -1.001, -.999);
        Assert.InRange(used.Max(p => p.Z), 6.999, 7.001);
    }

    [Fact]
    public void PlanarCuboid_OutwardShellWithAdjacentOpeningsHasNoInternalRim()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .001);
        var block = api.CreateCuboid(new Vec3D(0, 0, 0), new Vec3D(10, 8, 6), "block");
        string side = block.surfaceMetaData.Single(pair => pair.Value.PlaneParams != null &&
            pair.Value.PlaneParams.Origin.X > 9.9 && Math.Abs(pair.Value.PlaneParams.Normal.X) > .9).Key;

        var result = api.Shell(block, 1, new List<string> { "block-ExtrudeTop", side }, outward: true);

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles, true));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles), 343.99, 344.01);
        Assert.Contains("ShellRim_block-ExtrudeTop", result.extendedNameToGroupId.Keys);
        Assert.Contains("ShellRim_" + side, result.extendedNameToGroupId.Keys);
        Assert.DoesNotContain(result.extendedNameToGroupId.Keys, name => name.StartsWith("ShellOpening_"));
    }

    [Fact]
    public void AnalyticCylinder_OutwardShellExpandsWallAndPreservesRadius()
    {
        const double radius = 5, height = 8, thickness = .75;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var cylinder = api.CreateCylinder(CoordinateSystem.Default, radius, height, .005, "cyl");

        var result = api.Shell(cylinder, thickness, new List<string> { "cyl-ExtrudeTop" }, outward: true);

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles, true));
        var outerWall = Assert.Single(result.surfaceMetaData, pair =>
            !pair.Key.StartsWith("ShellInner_") && pair.Value.CylinderParams != null);
        Assert.Equal(radius + thickness, outerWall.Value.CylinderParams.Radius, 10);
        var innerWall = Assert.Single(result.surfaceMetaData, pair =>
            pair.Key.StartsWith("ShellInner_") && pair.Value.CylinderParams != null);
        Assert.Equal(radius, innerWall.Value.CylinderParams.Radius, 10);
        double sourceVolume = MeshAnalysis.ComputeSignedMeshVolume(cylinder.Mesh.Positions, cylinder.Mesh.Triangles);
        double expected = sourceVolume * (Math.Pow((radius + thickness) / radius, 2) *
            ((height + 2 * thickness) / height) - (height + thickness) / height);
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles),
            expected - .03, expected + .03);
    }

    [Fact]
    public void ConcavePlanarBoolean_OutwardShellKeepsMiteredCornersWatertight()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .001);
        var horizontal = api.CreateCuboid(new Vec3D(0, 0, 0), new Vec3D(10, 4, 6), "horizontal");
        var vertical = api.CreateCuboid(new Vec3D(0, 0, 0), new Vec3D(4, 10, 6), "vertical");
        var lShape = api.Boolean(horizontal, vertical, CSG.BooleanOp.Union, "lShape");

        var result = api.Shell(lShape, 1, outward: true);

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles, true));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles), 479.99, 480.01);
    }

    [Fact]
    public void OutwardShellRejectsCollapsedBore()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .01);
        var outer = api.CreateCylinder(CoordinateSystem.Default, 5, 6, .01, "outer");
        var bore = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, -1)), 1, 8, .01, "bore");
        var ring = api.Boolean(outer, bore, CSG.BooleanOp.Subtract, "ring");

        Assert.Throws<ArgumentException>(() => api.Shell(ring, 1.5, name: "invalidShell", outward: true));
        Assert.Null(api.GetMeshFromName("invalidShell"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Sphere_OutwardShellSupportsAnalyticAndMeshFallback(bool removeMetadata)
    {
        const double radius = 5, thickness = .4;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .01);
        var sphere = api.CreateSphere(CoordinateSystem.Default, radius, .01, "sphere");
        double sourceVolume = MeshAnalysis.ComputeSignedMeshVolume(sphere.Mesh.Positions, sphere.Mesh.Triangles);
        if (removeMetadata)
            foreach (string patch in sphere.surfaceMetaData.Keys.ToArray())
                sphere.surfaceMetaData[patch] = new SurfaceMetaData(SurfaceType.Unknown);

        var result = api.Shell(sphere, thickness, outward: true);

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles, true));
        double expected = sourceVolume * (Math.Pow((radius + thickness) / radius, 3) - 1);
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles),
            expected - .05, expected + .05);
        Assert.Equal(removeMetadata ? SurfaceType.Unknown : SurfaceType.Spherical,
            Assert.Single(result.surfaceMetaData, pair => pair.Key.StartsWith("ShellInner_")).Value.SurfaceType);
    }

    [Fact]
    public void UnionSeam_OutwardShellRebuildsTopologyWithCsg()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20, -20, -5), new Vec3D(20, 20, 40)), .001);
        var body = api.CreateCuboid(new Vec3D(0, -3, 0), new Vec3D(10, 3, 30), "body");
        var neck = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 28)), 3, 4, .001, "neck");
        var joined = api.Boolean(body, neck, CSG.BooleanOp.Union, "joined");

        var shell = api.Shell(joined, .3, new List<string> { "neck-ExtrudeTop" }, name: "shell", outward: true);

        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        Assert.Contains("ShellRim_neck-ExtrudeTop", shell.extendedNameToGroupId.Keys);
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(shell.Mesh.Positions, shell.Mesh.Triangles) > 0);
    }

    [Fact]
    public void AnalyticCylinder_OffsetsWallRadiusAndPreservesMetadata()
    {
        const double radius = 5, height = 8, thickness = 0.75;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var cylinder = api.CreateCylinder(CoordinateSystem.Default, radius, height, .005, "cyl");
        double sourceVolume = MeshAnalysis.ComputeSignedMeshVolume(cylinder.Mesh.Positions, cylinder.Mesh.Triangles);

        var result = api.Shell(cylinder, thickness, new List<string> { "cyl-ExtrudeTop" }, name: "cup");

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        var innerWall = Assert.Single(result.surfaceMetaData.Where(pair =>
            pair.Key.StartsWith("ShellInner_", StringComparison.Ordinal) && pair.Value.CylinderParams != null));
        Assert.Equal(radius - thickness, innerWall.Value.CylinderParams.Radius, 10);
        Assert.InRange(innerWall.Value.CylinderParams.Height, height - thickness, height - thickness + .01);
        double expected = sourceVolume * (1 - Math.Pow((radius - thickness) / radius, 2) * ((height - thickness) / height));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles),
            expected - .02, expected + .02);
    }

    [Fact]
    public void BooleanAnnulus_OffsetsConvexAndConcaveCylindersInOppositeDirections()
    {
        const double outerRadius = 5, boreRadius = 2, height = 6, thickness = .5;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .01);
        var outer = api.CreateCylinder(CoordinateSystem.Default, outerRadius, height, .01, "outer");
        var bore = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, -1)), boreRadius, height + 2, .01, "bore");
        var annulus = api.Boolean(outer, bore, CSG.BooleanOp.Subtract, "annulus");
        annulus.EnsureCoplanarPostProcessed();
        int topTriangle = Enumerable.Range(0, annulus.Mesh.Triangles.Count)
            .OrderByDescending(i => {
                var triangle = annulus.Mesh.Triangles[i];
                return annulus.Mesh.Positions[triangle.A].Z + annulus.Mesh.Positions[triangle.B].Z + annulus.Mesh.Positions[triangle.C].Z;
            }).First();
        string top = annulus.groupIdToExtendedName[annulus.Mesh.TrianglesEx[topTriangle].GroupId];

        var result = api.Shell(annulus, thickness, new List<string> { top }, name: "shelledAnnulus");

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        var radii = result.surfaceMetaData
            .Where(pair => pair.Key.StartsWith("ShellInner_", StringComparison.Ordinal) && pair.Value.CylinderParams != null)
            .Select(pair => pair.Value.CylinderParams.Radius).OrderBy(value => value).ToArray();
        Assert.Equal(new[] { boreRadius + thickness, outerRadius - thickness }, radii);
        double expected = Math.PI * ((outerRadius * outerRadius - boreRadius * boreRadius) * height -
            (Math.Pow(outerRadius - thickness, 2) - Math.Pow(boreRadius + thickness, 2)) * (height - thickness));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles),
            expected - .75, expected + .75);
    }

    [Fact]
    public void BooleanHemisphere_UsesExactSpherePlaneJunction()
    {
        const double radius = 5, thickness = .5;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .01);
        var sphere = api.CreateSphere(CoordinateSystem.Default, radius, .01, "sphere");
        var upperHalfSpace = api.CreateCuboid(new Vec3D(-10, -10, 0), new Vec3D(10, 10, 10), "clip");
        var hemisphere = api.Boolean(sphere, upperHalfSpace, CSG.BooleanOp.Intersect, "hemisphere");
        hemisphere.EnsureCoplanarPostProcessed();
        int capTriangle = Enumerable.Range(0, hemisphere.Mesh.Triangles.Count)
            .OrderBy(i => {
                var triangle = hemisphere.Mesh.Triangles[i];
                return hemisphere.Mesh.Positions[triangle.A].Z + hemisphere.Mesh.Positions[triangle.B].Z + hemisphere.Mesh.Positions[triangle.C].Z;
            }).First();
        string cap = hemisphere.groupIdToExtendedName[hemisphere.Mesh.TrianglesEx[capTriangle].GroupId];

        var result = api.Shell(hemisphere, thickness, new List<string> { cap }, name: "sphereBowl");

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        var innerSphere = Assert.Single(result.surfaceMetaData, pair =>
            pair.Key.StartsWith("ShellInner_", StringComparison.Ordinal) && pair.Value.SphereParams != null);
        Assert.Equal(radius - thickness, innerSphere.Value.SphereParams.Radius, 10);
        double expected = 2 * Math.PI / 3 * (Math.Pow(radius, 3) - Math.Pow(radius - thickness, 3));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles),
            expected - 1, expected + 1);
    }

    [Fact]
    public void BooleanHemisphere_WithoutSurfaceMetadata_UsesWeldedMeshOffset()
    {
        const double radius = 5, thickness = .4;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .01);
        var sphere = api.CreateSphere(CoordinateSystem.Default, radius, .01, "meshSphere");
        var clip = api.CreateCuboid(new Vec3D(-10, -10, 0), new Vec3D(10, 10, 10), "meshClip");
        var hemisphere = api.Boolean(sphere, clip, CSG.BooleanOp.Intersect, "meshHemisphere");
        hemisphere.EnsureCoplanarPostProcessed();
        string curvedPatch = hemisphere.surfaceMetaData.Single(pair => pair.Value.SphereParams != null).Key;
        int capTriangle = Enumerable.Range(0, hemisphere.Mesh.Triangles.Count)
            .OrderBy(i => {
                var triangle = hemisphere.Mesh.Triangles[i];
                return hemisphere.Mesh.Positions[triangle.A].Z + hemisphere.Mesh.Positions[triangle.B].Z + hemisphere.Mesh.Positions[triangle.C].Z;
            }).First();
        string cap = hemisphere.groupIdToExtendedName[hemisphere.Mesh.TrianglesEx[capTriangle].GroupId];
        foreach (string patch in hemisphere.surfaceMetaData.Keys.ToArray())
            hemisphere.surfaceMetaData[patch] = new SurfaceMetaData(SurfaceType.Unknown);

        var result = api.Shell(hemisphere, thickness, new List<string> { cap }, name: "meshBowl");

        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles));
        string innerName = "ShellInner_" + curvedPatch;
        Assert.Equal(SurfaceType.Unknown, result.surfaceMetaData[innerName].SurfaceType);
        Assert.True(result.TryGetSurface(innerName, out var inner));
        var radii = inner.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct()
            .Select(i => inner.Points[i].Length()).ToArray();
        Assert.InRange(radii.Min(), radius - thickness - .03, radius - thickness + .03);
        Assert.InRange(radii.Max(), radius - thickness - .03, radius - thickness + .03);
    }
}
