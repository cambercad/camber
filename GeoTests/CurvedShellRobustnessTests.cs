using CSG;
using Curves;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

[Collection("GlobalCadState")]
public sealed class CurvedShellRobustnessTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(true, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, true)]
    public void OffEquatorSphereOpeningPreservesWallAndRim(bool outward, bool eraseMetadata, bool transformed)
    {
        const double radius = 5, cut = 1, thickness = .4;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var sphere = api.CreateSphere(CoordinateSystem.Default, radius, .005, "sphere");
        var clip = api.CreateCuboid(new Vec3D(-10, -10, cut), new Vec3D(10), "clip");
        var cap = api.Boolean(sphere, clip, BooleanOp.Intersect, "cap");
        cap.EnsureCoplanarPostProcessed();
        string opening = "";
        foreach (var pair in cap.groupIdToExtendedName)
            if (cap.surfaceMetaData.TryGetValue(pair.Value, out var meta) && meta.PlaneParams != null)
                opening = pair.Value;
        Assert.NotEmpty(opening);
        var frame = transformed ? new CoordinateSystem(new Vec3D(2, -3, 1),
            new Vec3D(.6, .8, 0), new Vec3D(-.48, .36, .8), new Vec3D(.64, -.48, .6)) : CoordinateSystem.Default;
        if (transformed)
        {
            cap = api.CopyMesh(cap, CoordinateSystem.Default, frame, "posed_", SurfacePatchNameAffix.Prefix, "posed");
            opening = "posed_" + opening;
        }
        Assert.True(cap.TryGetSurface(opening, out var sourceRim));
        Assert.True(sourceRim.IsSurfacePlanar(), "A rigid transform must preserve exact cap planarity.");
        if (eraseMetadata) cap.surfaceMetaData.Clear();

        var shell = api.Shell(cap, thickness, new List<string> { opening }, outward: outward);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        double lower = outward ? cut - thickness : cut;
        double outerRadius = outward ? radius + thickness : radius;
        double innerRadius = outward ? radius : radius - thickness;
        double expected = CapVolume(outerRadius, lower) - CapVolume(innerRadius, lower);
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(shell.Mesh.Positions, shell.Mesh.Triangles),
            expected - 1, expected + 1);
        double Station(Vec3D point) => Vec3DOps.Dot(point - frame.Origin, frame.Z);
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, frame.Origin + frame.Z * cut,
            frame.Z, out var inside));
        Assert.InRange(Station(inside.Point), innerRadius - .02, innerRadius + .02);
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, frame.Origin + frame.Z * 10,
            -frame.Z, out var outside));
        Assert.InRange(Station(outside.Point) - Station(inside.Point), thickness - .02, thickness + .02);

        Assert.True(shell.TryGetSurface("ShellRim_" + opening, out var rim));
        foreach (var triangle in rim.Triangles)
            foreach (int index in new[] { triangle.A, triangle.B, triangle.C })
                Assert.InRange(Math.Abs(Station(rim.Points[index]) - lower), 0, 4 * api.Converter.SmallestUnit());
    }

    private static double CapVolume(double radius, double lower) =>
        Math.PI * (2 * radius * radius * radius / 3 - radius * radius * lower + lower * lower * lower / 3);

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ParallelCylinderIntersectionKeepsBothCurvedWalls(bool outward, bool eraseMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var first = api.CreateCylinder(new CoordinateSystem(new Vec3D(-2, 0, -4)), 5, 8, .005, "first");
        var second = api.CreateCylinder(new CoordinateSystem(new Vec3D(2, 0, -4)), 5, 8, .005, "second");
        var lens = api.Boolean(first, second, BooleanOp.Intersect, "lens");
        lens.EnsureCoplanarPostProcessed();
        var openings = new List<string>();
        foreach (var pair in lens.groupIdToExtendedName)
            if (lens.TryGetSurface(pair.Value, out var surface) && surface.IsSurfacePlanar() &&
                surface.Points[surface.Triangles[0].A].Z > 3)
                openings.Add(lens.Name + ":" + pair.Value);
        Assert.NotEmpty(openings);
        Assert.Null(lens.UnionOperands);
        if (eraseMetadata) lens.surfaceMetaData.Clear();
        var shell = api.Shell(lens, .2, openings, outward: outward);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0), new Vec3D(0, 1, 0), out var hit));
        double innerRadius = outward ? 5 : 4.8;
        Assert.InRange(hit.Point.Y, Math.Sqrt(innerRadius * innerRadius - 4) - .015,
            Math.Sqrt(innerRadius * innerRadius - 4) + .015);
        if (!outward)
        {
            double Area(double radius) => 2 * radius * radius * Math.Acos(2 / radius) - 4 * Math.Sqrt(radius * radius - 4);
            double expected = 8 * Area(5) - 7.8 * Area(4.8);
            Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(shell.Mesh.Positions, shell.Mesh.Triangles),
                expected - .15, expected + .15);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SphereCylinderUnionKeepsNeckAndChamber(bool outward, bool eraseMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var sphere = api.CreateSphere(CoordinateSystem.Default, 5, .005, "sphere");
        var neck = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 3)), 2, 5, .005, "neck");
        var blank = api.Boolean(sphere, neck, BooleanOp.Union, "vessel");
        blank.EnsureCoplanarPostProcessed();
        var openings = new List<string>();
        foreach (var pair in blank.groupIdToExtendedName)
            if (blank.TryGetSurface(pair.Value, out var surface) && surface.IsSurfacePlanar() &&
                surface.Points[surface.Triangles[0].A].Z > 7)
                openings.Add(pair.Value);
        Assert.Single(openings);
        if (eraseMetadata) blank.surfaceMetaData.Clear();
        var shell = api.Shell(blank, .2, openings, outward: outward);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        foreach (var (station, radius) in new[] { (0.0, outward ? 5 : 4.8), (6.0, outward ? 2 : 1.8) })
        {
            Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0, 0, station), new Vec3D(1, 0, 0), out var hit));
            Assert.InRange(hit.Point.X, radius - .015, radius + .015);
        }
    }

    [Theory]
    [InlineData(false, .2)]
    [InlineData(true, .2)]
    [InlineData(false, .6)]
    [InlineData(true, .6)]
    [InlineData(false, .9)]
    [InlineData(true, .9)]
    public void ReentrantNeckCavityFollowsOffsetSupports(bool eraseMetadata, double thickness) =>
        CheckReentrantNeck(eraseMetadata, thickness, 1, false);

    [Theory]
    [InlineData(.6, .1)]
    [InlineData(.9, .1)]
    [InlineData(.6, 10)]
    [InlineData(.9, 10)]
    public void MetadataFreeReentrantNeckIsInvariantUnderPoseAndScale(double thickness, double scale) =>
        CheckReentrantNeck(true, thickness, scale, true);

    private static void CheckReentrantNeck(bool eraseMetadata, double thickness, double scale, bool transformed)
    {
        thickness *= scale;
        var api = new GeoAPI(new Box3D(new Vec3D(-20 * scale), new Vec3D(20 * scale)), .005 * scale);
        var sphere = api.CreateSphere(CoordinateSystem.Default, 5 * scale, .005 * scale, "sphere");
        var neck = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 3 * scale)), 2 * scale, 5 * scale, .005 * scale, "neck");
        var blank = api.Boolean(sphere, neck, BooleanOp.Union, "vessel");
        blank.EnsureCoplanarPostProcessed();
        string opening = "neck-ExtrudeTop";
        if (transformed)
        {
            var frame = new CoordinateSystem(new Vec3D(2, -3, 1) * scale,
                new Vec3D(.6, .8, 0), new Vec3D(-.48, .36, .8), new Vec3D(.64, -.48, .6));
            blank = api.CopyMesh(blank, CoordinateSystem.Default, frame, "posed_", SurfacePatchNameAffix.Prefix, "posed");
            opening = "posed_" + opening;
        }
        var metadata = SurfaceMetaData.CloneDictionary(blank.surfaceMetaData);
        if (eraseMetadata) blank.surfaceMetaData.Clear();
        var shell = api.Shell(blank, thickness, new List<string> { opening });
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        foreach (var (patch, surface) in metadata)
        {
            if (surface.CylinderParams == null && surface.SphereParams == null) continue;
            Assert.True(shell.TryGetSurface(EntityNaming.ShellInner(patch), out var inner));
            foreach (var triangle in inner.Triangles)
                foreach (int vertex in new[] { triangle.A, triangle.B, triangle.C })
                {
                    var point = inner.Points[vertex];
                    double radius;
                    if (surface.SphereParams != null) radius = (point - surface.SphereParams.Center).Length();
                    else
                    {
                        var cylinder = surface.CylinderParams;
                        var axis = cylinder.Axis.Normalized(); var radial = point - cylinder.Origin;
                        radius = (radial - axis * Vec3DOps.Dot(radial, axis)).Length();
                    }
                    double expected = (surface.SphereParams?.Radius ?? surface.CylinderParams.Radius) - thickness;
                    Assert.InRange(radius, expected - .01 * scale, expected + .01 * scale);
                }
        }
    }

    [Theory]
    [InlineData(.2)]
    [InlineData(.6)]
    [InlineData(.9)]
    public void ReentrantOffsetIntersectionRequiresMissingSupportDomain(double thickness)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var sphere = api.CreateSphere(CoordinateSystem.Default, 5, .005, "sphere");
        var neck = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, 3)), 2, 5, .005, "neck");
        var blank = api.Boolean(sphere, neck, BooleanOp.Union, "vessel");
        blank.EnsureCoplanarPostProcessed();
        Assert.True(blank.TryGetTopologySurface("sphere-Arc1", out var spherePatch));
        Assert.True(blank.TryGetTopologySurface("neck-Circle1", out var neckPatch));
        blank.surfaceMetaData.Clear();
        double expectedSeamHeight = Math.Sqrt(Math.Pow(5 - thickness, 2) - Math.Pow(2 - thickness, 2));
        var sphereOffset = spherePatch.GetOffsetSurface(-thickness, api.Converter);
        var neckOffset = neckPatch.GetOffsetSurface(-thickness, api.Converter);
        double sphereTop = double.NegativeInfinity, neckBottom = double.PositiveInfinity;
        foreach (var triangle in sphereOffset.Triangles)
            foreach (int vertex in new[] { triangle.A, triangle.B, triangle.C })
                sphereTop = Math.Max(sphereTop, sphereOffset.Points[vertex].Z);
        foreach (var triangle in neckOffset.Triangles)
            foreach (int vertex in new[] { triangle.A, triangle.B, triangle.C })
                neckBottom = Math.Min(neckBottom, neckOffset.Points[vertex].Z);
        // The desired seam lies outside BOTH available offset patches. An
        // intersection solver alone cannot recover discarded support geometry.
        Assert.True(sphereTop < expectedSeamHeight - .01);
        Assert.True(neckBottom > expectedSeamHeight + .01);
    }

    [Theory]
    [InlineData(false, .1, false)]
    [InlineData(true, .1, false)]
    [InlineData(false, .2, false)]
    [InlineData(true, .2, false)]
    [InlineData(false, .6, true)]
    [InlineData(true, .6, true)]
    public void HistoryFreeNestedCurvedIntersectionsHaveStableCavityNames(bool eraseMetadata, double thickness, bool transformed) =>
        CheckNestedCurvedIntersections(eraseMetadata, thickness, transformed);

    private void CheckNestedCurvedIntersections(bool eraseMetadata, double thickness, bool transformed)
    {
        const double deviation = .01;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), deviation);
        var vertical = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, -6)), 5, 12, deviation, "vertical");
        var acrossFrame = new CoordinateSystem(new Vec3D(-6, 0, 0), new Vec3D(0, 1, 0),
            new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
        var across = api.CreateCylinder(acrossFrame, 5, 12, deviation, "across");
        var intersection = api.Boolean(vertical, across, BooleanOp.Intersect, "cross");
        var sphere = api.CreateSphere(CoordinateSystem.Default, 6, deviation, "sphere");
        var blank = api.Boolean(intersection, sphere, BooleanOp.Intersect, "blank");
        var frame = transformed ? new CoordinateSystem(new Vec3D(2, -3, 1),
            new Vec3D(.6, .8, 0), new Vec3D(-.48, .36, .8), new Vec3D(.64, -.48, .6)) : CoordinateSystem.Default;
        if (transformed)
            blank = api.CopyMesh(blank, CoordinateSystem.Default, frame, "posed_", SurfacePatchNameAffix.Prefix, "posed");
        blank.EnsureCoplanarPostProcessed();
        var sourceMetadata = SurfaceMetaData.CloneDictionary(blank.surfaceMetaData);
        Assert.Null(blank.UnionOperands);
        if (eraseMetadata) blank.surfaceMetaData.Clear();
        var shell = api.Shell(blank, thickness);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, frame.Origin, frame.Y, out var hit));
        Assert.InRange(Vec3DOps.Dot(hit.Point - frame.Origin, frame.Y), 5 - thickness - .02, 5 - thickness + .01);
        foreach (var (id, patch) in blank.groupIdToExtendedName)
        {
            if (!blank.TryGetTopologySurface(patch, out var surface) || surface.Triangles.Count == 0) continue;
            Assert.True(shell.extendedNameToGroupId.ContainsKey(EntityNaming.ShellInner(patch)),
                "Missing " + EntityNaming.ShellInner(patch) + ": " + string.Join(", ", shell.extendedNameToGroupId.Keys));
            Assert.True(shell.TryGetSurface(EntityNaming.ShellInner(patch), out var inner));
            var metadata = sourceMetadata[patch];
            foreach (var triangle in inner.Triangles)
                foreach (int vertex in new[] { triangle.A, triangle.B, triangle.C })
                {
                    var point = inner.Points[vertex];
                    if (metadata.CylinderParams is { } cylinder)
                    {
                        var axis = cylinder.Axis.Normalized();
                        var v = point - cylinder.Origin;
                        Assert.InRange((v - axis * Vec3DOps.Dot(v, axis)).Length(),
                            cylinder.Radius - thickness - .015, cylinder.Radius - thickness + .015);
                    }
                    else if (metadata.SphereParams is { } ball)
                        Assert.InRange((point - ball.Center).Length(), ball.Radius - thickness - .015,
                            ball.Radius - thickness + .015);
                }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HistoryFreePerpendicularCylinderIntersection(bool eraseMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var vertical = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, -6)), 5, 12, .005, "vertical");
        var across = api.CreateCylinder(new CoordinateSystem(new Vec3D(-6, 0, 0),
            new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0)), 5, 12, .005, "across");
        var blank = api.Boolean(vertical, across, BooleanOp.Intersect, "cross");
        Assert.Null(blank.UnionOperands);
        if (eraseMetadata) blank.surfaceMetaData.Clear();
        var shell = api.Shell(blank, .2);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0), new Vec3D(0, 1, 0), out var hit));
        Assert.InRange(hit.Point.Y, 4.78, 4.81);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LoftCylinderIntersectionSupportsNonanalyticPatches(bool eraseMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (var (height, radius) in new[] { (-4.0, 4.0), (0.0, 5.0), (4.0, 4.0) })
        {
            var section = new PlotterSketcherCoordSys("profile" + sections.Count,
                new CoordinateSystem(new Vec3D(0, 0, height)), new Vec2D(0));
            section.AddCircle(new Vec2D(0), radius);
            sections.Add(section);
        }
        var loft = api.Loft(sections, new LoftOptions(), name: "bulge");
        var across = api.CreateCylinder(new CoordinateSystem(new Vec3D(-6, 0, 0),
            new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0)), 4, 12, .005, "across");
        var blank = api.Boolean(loft, across, BooleanOp.Intersect, "blank");
        blank.EnsureCoplanarPostProcessed();
        if (eraseMetadata) blank.surfaceMetaData.Clear();
        var shell = api.Shell(blank, .15);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0), new Vec3D(1, 0, 0), out var x));
        Assert.InRange(x.Point.X, 4.82, 4.88);
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0), new Vec3D(0, 1, 0), out var y));
        Assert.InRange(y.Point.Y, 3.83, 3.87);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ConcaveLoftTransitionKeepsItsWaist(bool outward, bool eraseMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (var (height, radius) in new[] { (0.0, 5.0), (4.0, 3.0), (8.0, 5.0) })
        {
            var section = new PlotterSketcherCoordSys("section" + sections.Count,
                new CoordinateSystem(new Vec3D(0, 0, height)), new Vec2D(0));
            section.AddCircle(new Vec2D(0), radius);
            sections.Add(section);
        }
        var blank = api.Loft(sections, new LoftOptions(), name: "waist");
        if (eraseMetadata) blank.surfaceMetaData.Clear();
        var shell = api.Shell(blank, .15, new List<string> { "waist-EndCap" }, outward: outward);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0, 0, 4), new Vec3D(1, 0, 0), out var hit));
        Assert.InRange(hit.Point.X, outward ? 2.98 : 2.83, outward ? 3.02 : 2.87);
    }

    [Fact]
    public void CurvedIntersectionShellUsesAuthoritativeMeshNotOldOperands()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var first = api.CreateCylinder(new CoordinateSystem(new Vec3D(-2, 0, -4)), 5, 8, .005, "first");
        var second = api.CreateCylinder(new CoordinateSystem(new Vec3D(2, 0, -4)), 5, 8, .005, "second");
        var lens = api.Boolean(first, second, BooleanOp.Intersect, "lens");
        var moved = api.CopyMesh(first, CoordinateSystem.Default, new CoordinateSystem(new Vec3D(1, 0, 0)),
            "moved_", SurfacePatchNameAffix.Prefix, "moved");
        first.Mesh = moved.Mesh;
        var shell = api.Shell(lens, .2);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0), new Vec3D(0, 1, 0), out var hit));
        Assert.InRange(hit.Point.Y, Math.Sqrt(4.8 * 4.8 - 4) - .015, Math.Sqrt(4.8 * 4.8 - 4) + .015);
    }

    [Fact]
    public void CurvedIntersectionRejectsCollapsedCavityWithoutRegisteringShell()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var first = api.CreateCylinder(new CoordinateSystem(new Vec3D(-2, 0, -4)), 5, 8, .005, "first");
        var second = api.CreateCylinder(new CoordinateSystem(new Vec3D(2, 0, -4)), 5, 8, .005, "second");
        var lens = api.Boolean(first, second, BooleanOp.Intersect, "lens");
        int registered = api.GetMeshes().Count;
        var error = Assert.Throws<InvalidOperationException>(() => api.Shell(lens, 3.1));
        Assert.Contains("collapsed", error.Message);
        Assert.Equal(registered, api.GetMeshes().Count);
    }

    [Fact]
    public void ShellFaceReferencesRejectUnknownAndDuplicateAliases()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var cube = api.CreateCuboid(new Vec3D(-5), new Vec3D(5), "cube");
        int registered = api.GetMeshes().Count;
        Assert.Throws<ArgumentException>(() => api.Shell(cube, .2, new List<string> { "missing" }));
        string face = cube.groupIdToExtendedName.Values.First();
        var duplicate = Assert.Throws<ArgumentException>(() => api.Shell(cube, .2,
            new List<string> { face, cube.Name + ":" + face }));
        Assert.Contains("more than once", duplicate.Message);
        Assert.Equal(registered, api.GetMeshes().Count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConicalLoftOpeningHasUniformNormalThickness(bool eraseMetadata)
    {
        const double thickness = .4, height = 8, slope = .25;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var lower = new PlotterSketcherCoordSys("lower", CoordinateSystem.Default, new Vec2D(0));
        lower.AddCircle(new Vec2D(0), 5);
        var upper = new PlotterSketcherCoordSys("upper", new CoordinateSystem(new Vec3D(0, 0, height)), new Vec2D(0));
        upper.AddCircle(new Vec2D(0), 3);
        var blank = api.Loft(new List<PlotterSketcherCoordSys> { lower, upper }, new LoftOptions(), name: "cone");
        if (eraseMetadata) blank.surfaceMetaData.Clear();
        var shell = api.Shell(blank, thickness, new List<string> { "cone-EndCap" });
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        double inset = thickness * Math.Sqrt(1 + slope * slope);
        foreach (double z in new[] { 1.0, 4, 7.95 })
        {
            Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0, 0, z),
                new Vec3D(1, 0, 0), out var hit));
            Assert.InRange(hit.Point.X, 5 - slope * z - inset - .015, 5 - slope * z - inset + .015);
        }
        double Integral(double z) => Math.Pow(5 - inset - slope * z, 3) / (-3 * slope);
        double expected = Math.PI * height / 3 * (25 + 15 + 9) - Math.PI * (Integral(height) - Integral(thickness));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(shell.Mesh.Positions, shell.Mesh.Triangles), expected - 1, expected + 1);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void ClosedSphereHasUniformWallWithoutRequiringMetadata(bool outward, bool eraseMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var sphere = api.CreateSphere(CoordinateSystem.Default, 5, .005, "sphere");
        double volume = MeshAnalysis.ComputeSignedMeshVolume(sphere.Mesh.Positions, sphere.Mesh.Triangles);
        if (eraseMetadata) sphere.surfaceMetaData.Clear();
        var shell = api.Shell(sphere, .4, outward: outward);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        double expected = volume * (outward ? Math.Pow(1.08, 3) - 1 : 1 - Math.Pow(.92, 3));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(shell.Mesh.Positions, shell.Mesh.Triangles), expected - .05, expected + .05);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphericalBandHasTwoOpenings(bool eraseMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var sphere = api.CreateSphere(CoordinateSystem.Default, 5, .005, "sphere");
        var clip = api.CreateCuboid(new Vec3D(-10, -10, -1), new Vec3D(10, 10, 1), "clip");
        var band = api.Boolean(sphere, clip, BooleanOp.Intersect, "band");
        band.EnsureCoplanarPostProcessed();
        var openings = new List<string>();
        foreach (var pair in band.groupIdToExtendedName)
            if (band.surfaceMetaData.TryGetValue(pair.Value, out var meta) && meta.PlaneParams != null)
                openings.Add(pair.Value);
        Assert.Equal(2, openings.Count);
        if (eraseMetadata) band.surfaceMetaData.Clear();
        var shell = api.Shell(band, .4, openings);
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        double expected = 2 * Math.PI * (25 - 4.6 * 4.6);
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(shell.Mesh.Positions, shell.Mesh.Triangles), expected - 1, expected + 1);
        Assert.False(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0), new Vec3D(0, 0, 1), out _));
        Assert.False(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(0), new Vec3D(0, 0, -1), out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcaveCurvedExtrusionDoesNotTrimAwayEitherArm(bool eraseMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var profile = new PlotterSketcherCoordSys("profile", CoordinateSystem.Default, new Vec2D(0));
        profile.AppendLine(10, 0);
        profile.AppendArc(new Vec2D(10 + Math.Sqrt(2), 2 - Math.Sqrt(2)), new Vec2D(12, 2));
        profile.AppendLine(12, 4);
        profile.AppendLine(4, 4);
        profile.AppendLine(4, 10);
        profile.AppendLine(0, 10);
        profile.AppendLine(0, 0);
        var blank = api.Extrude(profile, 6, name: "blank");
        if (eraseMetadata) blank.surfaceMetaData.Clear();
        var shell = api.Shell(blank, .4, new List<string> { "blank-ExtrudeTop" });
        Assert.True(MeshAnalysis.IsWatertightMesh(shell.Mesh.PrecisionPositions, shell.Mesh.Triangles, true));
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(2, 7, 3), new Vec3D(1, 0, 0), out var vertical));
        Assert.InRange(vertical.Point.X, 3.599, 3.601);
        Assert.True(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(8, 2, 3), new Vec3D(0, 1, 0), out var horizontal));
        Assert.InRange(horizontal.Point.Y, 3.599, 3.601);
        Assert.False(RayMeshExact.TryCast(shell, api.Converter, new Vec3D(8, 7, 10), new Vec3D(0, 0, -1), out _));
    }
}
