using CSG;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public class PlanarFaceLineageTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static GeoAPI Api() => new(new Box3D(new Vec3D(-10), new Vec3D(20)), .01);
    private static AnchorMesh Slot(GeoAPI api, string name, double x) =>
        api.CreateCuboid(new Vec3D(x, -1, -1), new Vec3D(x + 1, 5, 3), name);
    private static AnchorMesh Hole(GeoAPI api, string name, double x) =>
        api.CreateCylinder(new CoordinateSystem(new Vec3D(x, 2, -1)), .3, 4, .02, name);
    private static AnchorMesh Blank(GeoAPI api) =>
        api.CreateCuboid(new Vec3D(0), new Vec3D(12, 4, 2), "base");
    private const string Right = "base-ExtrudeTop{main_slot-Line2}";

    private static void AssertRight(AnchorMesh result, double start = 5)
    {
        result.EnsureCoplanarPostProcessed();
        Assert.True(result.TryGetSurface(Right, out var surface), string.Join(";", result.groupIdToExtendedName.Values));
        var active = surface.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct();
        var x = active.Select(i => surface.Points[i].X).ToArray();
        Assert.InRange(x.Min(), start - .001, start + .001);
        Assert.InRange(x.Max(), 11.999, 12.001);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(result.Mesh);
    }

    [Theory]
    [InlineData(false, false, 4)]
    [InlineData(true, false, 4)]
    [InlineData(true, true, 4)]
    [InlineData(true, false, 4.2)]
    public void OptionalCutOrderAndParameterChangesKeepRightFace(bool optional, bool reversed, double mainStart)
    {
        var api = Api();
        var result = Blank(api);
        if (optional && !reversed) result = api.Boolean(result, Slot(api, "optional_slot", 1), BooleanOp.Subtract);
        result = api.Boolean(result, Slot(api, "main_slot", mainStart), BooleanOp.Subtract);
        if (optional && reversed) result = api.Boolean(result, Slot(api, "optional_slot", 1), BooleanOp.Subtract);
        AssertRight(result, mainStart + 1);
        var body = api.GetAssembly("reference_check").AddPart(result, new Vec3D(0));
        Assert.Throws<NameCollisionException>(() => body.AddPlaneDatum("base-ExtrudeTop"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NonseparatingHoleDoesNotRenameRightFace(bool before)
    {
        var api = Api(); var result = Blank(api);
        var hole = Hole(api, "hole", 9);
        if (before) result = api.Boolean(result, hole, BooleanOp.Subtract);
        result = api.Boolean(result, Slot(api, "main_slot", 4), BooleanOp.Subtract);
        if (!before) result = api.Boolean(result, hole, BooleanOp.Subtract);
        AssertRight(result);
    }

    [Fact]
    public void CompoundTwoHolesOnSeparateRegionsAreNotSeparators()
    {
        var api = Api();
        var result = api.Boolean(Blank(api), Slot(api, "main_slot", 4), BooleanOp.Subtract);
        var tool = api.Boolean(Hole(api, "left_hole", 2), Hole(api, "right_hole", 9), BooleanOp.Union, "compound_holes");
        result = api.Boolean(result, tool, BooleanOp.Subtract);
        AssertRight(result);
    }

    [Fact]
    public void OneFeatureContainingSlotAndHoleUsesOnlySeparatingSlotBoundary()
    {
        var api = Api();
        var tool = api.Boolean(Slot(api, "main_slot", 4), Hole(api, "hole", 9), BooleanOp.Union, "compound_tool");
        AssertRight(api.Boolean(Blank(api), tool, BooleanOp.Subtract));
    }

    [Fact]
    public void AnnularRemovedRegionDistinguishesItsInnerAndOuterSurvivors()
    {
        var api = Api();
        var outer = api.CreateCylinder(new CoordinateSystem(new Vec3D(6, 2, -1)), 1.5, 4, .02, "annulus_outer");
        var inner = api.CreateCylinder(new CoordinateSystem(new Vec3D(6, 2, -2)), .7, 6, .02, "annulus_inner");
        var tool = api.Boolean(outer, inner, BooleanOp.Subtract, "annulus");
        var result = api.Boolean(Blank(api), tool, BooleanOp.Subtract);
        result.EnsureCoplanarPostProcessed();
        var faces = result.FaceLineages.Where(pair => pair.Value.Roots.SequenceEqual(new[] { "base-ExtrudeTop" })).ToArray();
        Assert.Equal(2, faces.Length);
        Assert.All(faces, face => Assert.Single(face.Value.Separators));
        Assert.NotEqual(faces[0].Value.Reference, faces[1].Value.Reference);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(result.Mesh);
    }

    [Fact]
    public void IndistinguishableRepeatedBoundaryRootsKeepCurrentPatchNamesUsable()
    {
        var api = Api();
        AnchorMesh tool = null;
        foreach (int x in new[] { 2, 5, 8 })
        {
            var slot = Slot(api, "instance" + x, x);
            // Repeated occurrences of one authored tool share face creation
            // roots, so current components must receive distinct names.
            foreach (var pair in slot.groupIdToExtendedName)
                slot.FaceLineages[pair.Key] = new FaceLineage(new[] { pair.Value.Replace("instance" + x, "repeated_slot") });
            tool = tool == null ? slot : api.Boolean(tool, slot, BooleanOp.Union);
        }
        var result = api.Boolean(Blank(api), tool, BooleanOp.Subtract);
        result.EnsureCoplanarPostProcessed();
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(result.Mesh);
        var ambiguous = "base-ExtrudeTop{repeated_slot-Line2&repeated_slot-Line4}";
        var body = api.GetAssembly("ambiguous").AddPart(result, new Vec3D(0));
        // Select the current component, not its potentially ambiguous ancestor.
        int patch = result.FaceLineages.First(pair => pair.Value.Reference == ambiguous).Key;
        string currentPatch = result.GetCurrentPatchName(patch);
        Assert.DoesNotContain("#current=", currentPatch);
        Assert.DoesNotContain("unresolved:", currentPatch);
        Assert.NotNull(body.AddPlaneDatum(currentPatch));
        foreach (var copy in new[] {
            api.CopyMeshAsInstance(result, "ambiguous_instance"),
            api.PatternLinear(result, 2, new Vec3D(0, 0, 4), "ambiguous_pattern")[1],
            api.Mirror(result, CoordinateSystem.Default, "ambiguous_mirror") })
        {
            Assert.Equal(copy.groupIdToExtendedName.Count,
                copy.groupIdToExtendedName.Values.Distinct().Count());
            var repeated = copy.FaceLineages.GroupBy(pair => pair.Value.Reference)
                .Where(group => group.Count() > 1 && group.Any(item => item.Value.Split)).ToArray();
            Assert.NotEmpty(repeated);
            foreach (var group in repeated)
            {
                // Every visible current patch name remains valid and selectable.
                foreach (var item in group)
                {
                    Assert.DoesNotContain("#current=", copy.GetCurrentPatchName(item.Key));
                    copy.ValidateEntityReference(copy.groupIdToExtendedName[item.Key]);
                }
            }
            MeshPipelineTestHelpers.AssertWatertightAllowTouch(copy.Mesh);
        }
    }

    [Fact]
    public void CoplanarFusionKeepsEveryOriginalNameUntilASplit()
    {
        var api = Api();
        var fused = api.Boolean(
            api.CreateCuboid(new Vec3D(0), new Vec3D(6, 4, 2), "left"),
            api.CreateCuboid(new Vec3D(6, 0, 0), new Vec3D(12, 4, 2), "right"),
            BooleanOp.Union);
        fused.EnsureCoplanarPostProcessed();
        const string canonical = "left-ExtrudeTop&right-ExtrudeTop";
        Assert.Equal(canonical, fused.ResolveLocalPatchNamePublic("left-ExtrudeTop"));
        Assert.Equal(canonical, fused.ResolveLocalPatchNamePublic("right-ExtrudeTop"));
        Assert.True(fused.TryGetSurface("left-ExtrudeTop", out _));
        Assert.True(fused.TryGetSurface("right-ExtrudeTop", out _));
        Assert.Equal(fused.extendedNameToGroupId[canonical], fused.extendedNameToGroupId[fused.ResolveLocalPatchNamePublic("right-ExtrudeTop")]);
        string edge = fused.GroupEdges.Select(item => item.Name).First(name => name.Contains(canonical));
        Assert.Equal(edge, Assert.Single(FaceLineageEdges.Resolve(fused, new[] { edge.Replace(canonical, "left-ExtrudeTop") })));
        Assert.Throws<NameCollisionException>(() =>
            FaceLineageEdges.Resolve(fused, new[] { "[left-ExtrudeTop,right-ExtrudeTop]" }));
        var opened = api.Shell(fused, .2, new List<string> { "left-ExtrudeTop", "right-ExtrudeTop" });
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(opened.Mesh);

        var cut = api.Boolean(fused, Slot(api, "main_slot", 4), BooleanOp.Subtract);
        cut.EnsureCoplanarPostProcessed();
        var top = "left-ExtrudeTop&right-ExtrudeTop{main_slot-Line2}";
        Assert.True(cut.TryGetSurface(top, out _), string.Join(";", cut.groupIdToExtendedName.Values));
        var body = api.GetAssembly("fused_split").AddPart(cut, new Vec3D(0));
        var leftSplit = Assert.Throws<NameCollisionException>(() => body.AddPlaneDatum("left-ExtrudeTop"));
        var rightSplit = Assert.Throws<NameCollisionException>(() => body.AddPlaneDatum("right-ExtrudeTop"));
        Assert.Contains("ambiguous among current faces", leftSplit.Message);
        Assert.Contains(top, leftSplit.Message);
        Assert.Contains("ambiguous among current faces", rightSplit.Message);
        Assert.Contains(top, rightSplit.Message);
        Assert.Contains("main_slot-Line4", leftSplit.Message);
    }

    [Fact]
    public void ThreeCoplanarPatchesRemainOneFaceUnderEveryOriginalName()
    {
        var api = Api();
        var fused = api.Boolean(
            api.Boolean(
                api.CreateCuboid(new Vec3D(0), new Vec3D(4, 4, 2), "a"),
                api.CreateCuboid(new Vec3D(4, 0, 0), new Vec3D(8, 4, 2), "b"),
                BooleanOp.Union),
            api.CreateCuboid(new Vec3D(8, 0, 0), new Vec3D(12, 4, 2), "c"),
            BooleanOp.Union);
        fused.EnsureCoplanarPostProcessed();
        const string canonical = "a-ExtrudeTop&b-ExtrudeTop&c-ExtrudeTop";
        foreach (string alias in new[] { "a-ExtrudeTop", "b-ExtrudeTop", "c-ExtrudeTop", canonical })
            Assert.Equal(canonical, fused.ResolveLocalPatchNamePublic(alias));
        const string bottom = "a-ExtrudeBottom&b-ExtrudeBottom&c-ExtrudeBottom";
        Assert.Equal(bottom, fused.ResolveLocalPatchNamePublic("b-ExtrudeBottom"));
        string end = fused.groupIdToExtendedName.Values.First(name =>
            name.StartsWith("a-", StringComparison.Ordinal) && !name.Contains('&'));
        Assert.Equal(end, fused.ResolveLocalPatchNamePublic(end));
        Assert.NotEqual(canonical, end);
    }

    [Fact]
    public void CopiedAndRenamedFusionKeepsEveryAlias()
    {
        var api = Api();
        var fused = api.Boolean(
            api.CreateCuboid(new Vec3D(0), new Vec3D(6, 4, 2), "left"),
            api.CreateCuboid(new Vec3D(6, 0, 0), new Vec3D(12, 4, 2), "right"),
            BooleanOp.Union, "left");
        fused.EnsureCoplanarPostProcessed();
        var copy = api.CopyMeshAsInstance(fused, "copied");
        const string copied = "copied-ExtrudeTop&right-ExtrudeTop";
        Assert.Equal(copied, copy.ResolveLocalPatchNamePublic("copied-ExtrudeTop"));
        Assert.Equal(copied, copy.ResolveLocalPatchNamePublic("right-ExtrudeTop"));
        Assert.True(copy.TryGetSurface("right-ExtrudeTop", out _));
        fused.Rename("renamed");
        const string renamed = "renamed-ExtrudeTop&right-ExtrudeTop";
        Assert.Equal(renamed, fused.ResolveLocalPatchNamePublic("renamed-ExtrudeTop"));
        Assert.Equal(renamed, fused.ResolveLocalPatchNamePublic("right-ExtrudeTop"));
    }

    [Fact]
    public void RepeatingTheSameShellFaceStillFails()
    {
        var api = Api();
        var block = api.CreateCuboid(new Vec3D(0), new Vec3D(4, 4, 2), "block");
        var error = Assert.Throws<ArgumentException>(() =>
            api.Shell(block, .2, new List<string> { "block-ExtrudeTop", "block-ExtrudeTop" }));
        Assert.Contains("more than once", error.Message);
    }

    [Fact]
    public void FusedFacesRetainBothCreationRootsAndSnapshotsKeepLineage()
    {
        var api = Api();
        var left = api.CreateCuboid(new Vec3D(0), new Vec3D(6, 4, 2), "left");
        var right = api.CreateCuboid(new Vec3D(6, 0, 0), new Vec3D(12, 4, 2), "right");
        var blank = api.Boolean(left, right, BooleanOp.Union);
        blank.EnsureCoplanarPostProcessed();
        var result = api.Boolean(blank, Slot(api, "main_slot", 4), BooleanOp.Subtract);
        result.EnsureCoplanarPostProcessed();
        var reference = "left-ExtrudeTop&right-ExtrudeTop{main_slot-Line2}";
        Assert.True(result.TryGetSurface(reference, out _));
        var assembly = api.GetAssembly("snapshot");
        assembly.AddPart(result, new Vec3D(0));
        var snapshot = result.SnapshotRigidDefinition("copied");
        Assert.Equal(result.FaceLineages.OrderBy(pair => pair.Key), snapshot.FaceLineages.OrderBy(pair => pair.Key));
        Assert.Throws<NameCollisionException>(() => AssemblyDatumResolver.ResolvePlane(snapshot, "left-ExtrudeTop", api.Converter));
    }

    [Fact]
    public void ProvenanceNamesWorkInEdgesQualifiedDatumsAndMirroredCopies()
    {
        var api = Api();
        var result = api.Boolean(Blank(api), Slot(api, "main_slot", 4), BooleanOp.Subtract, "finished");
        AssertRight(result);
        var graph = new EdgeGraph(result.Mesh.Triangles, result.Mesh.GetTriangleGroups(), result.Mesh.Positions,
            result.Mesh.PrecisionPositions, result.groupIdToExtendedName);
        string edge = EntityNaming.FormatGroupEdgeName(Right, "main_slot-Line2");
        Assert.True(graph.TryGetEdge(edge, out _));
        Assert.True(EntityNaming.TryParseGroupEdgeAddress(edge, out var address, requireFullMatch: true));
        Assert.Equal(Right, address.PatchA);
        var assembly = api.GetAssembly("qualified");
        var body = assembly.AddPart(result, new Vec3D(0));
        Assert.Equal(body.AddPlaneDatum(Right).LocalNormal, body.AddPlaneDatum("finished:" + Right).LocalNormal);
        var mirrored = api.Mirror(result, CoordinateSystem.Default, "mirrored");
        Assert.Contains(mirrored.FaceLineages.Values, lineage => lineage.Reference == Right);
        var mirrorBody = assembly.AddPart(mirrored, new Vec3D(0));
        Assert.Throws<NameCollisionException>(() => mirrorBody.AddPlaneDatum("base-ExtrudeTop"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CopyKeepsRenamedProvenanceAfterLaterUnrelatedHole(bool mirror)
    {
        var api = Api();
        var result = api.Boolean(Blank(api), Slot(api, "main_slot", 4), BooleanOp.Subtract, "base");
        result.EnsureCoplanarPostProcessed();
        var copy = mirror ? api.Mirror(result, CoordinateSystem.Default, "copy") : api.CopyMeshAsInstance(result, "copy");
        string reference = "copy-ExtrudeTop{main_slot-Line2}";
        Assert.True(copy.TryGetSurface(reference, out _));
        var hole = api.CreateCylinder(new CoordinateSystem(new Vec3D(9, 2, -4)), .3, 8, .02, "later_hole");
        var after = api.Boolean(copy, hole, BooleanOp.Subtract);
        after.EnsureCoplanarPostProcessed();
        Assert.True(after.TryGetSurface(reference, out _));
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(after.Mesh);
    }

    [Fact]
    public void SupportPairRequiresOneEdgeAndDoesNotRelaxFiniteFaceDatums()
    {
        var api = Api();
        var result = api.Boolean(Blank(api), Slot(api, "main_slot", 4), BooleanOp.Subtract);
        result.EnsureCoplanarPostProcessed();
        var unique = FaceLineageEdges.Resolve(result, new[] { "[base-ExtrudeTop,main_slot-Line2]" });
        Assert.Single(unique);
        Assert.Contains(Right, unique[0]);
        // The front support intersects both disconnected top regions. Picking
        // its first current edge would silently restore the ordinal-name bug.
        Assert.Throws<NameCollisionException>(() => FaceLineageEdges.Resolve(result,
            new[] { "[base-ExtrudeTop,base-Line1]" }));
        var body = api.GetAssembly("strict_face").AddPart(result, new Vec3D(0));
        Assert.Throws<NameCollisionException>(() => body.AddPlaneDatum("base-ExtrudeTop"));
        Assert.Throws<NameCollisionException>(() => result.TryGetPointOnSurface("[base-ExtrudeTop]@0.5,0.5", out _));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void IndexedAncestorSupportPairCannotRetargetCurrentEdge(int index)
    {
        var api = Api();
        var result = api.Boolean(Blank(api), Slot(api, "main_slot", 4), BooleanOp.Subtract);
        result.EnsureCoplanarPostProcessed();
        const string supportPair = "[base-ExtrudeTop,main_slot-Line2]";
        Assert.Single(FaceLineageEdges.Resolve(result, new[] { supportPair }));
        Assert.Throws<NameCollisionException>(() =>
            FaceLineageEdges.Resolve(result, new[] { supportPair + "_" + index }));
    }

    [Fact]
    public void ExplicitFeatureRenameRemapsLineageBeforeLaterCut()
    {
        var api = Api();
        var result = api.Boolean(Blank(api), Slot(api, "main_slot", 4), BooleanOp.Subtract, "base");
        result.EnsureCoplanarPostProcessed();
        result.Rename("renamed");
        var after = api.Boolean(result, Hole(api, "hole", 9), BooleanOp.Subtract);
        after.EnsureCoplanarPostProcessed();
        Assert.True(after.TryGetSurface("renamed-ExtrudeTop{main_slot-Line2}", out _));
    }

    [Fact]
    public void TwoSidedExtrudedHoleKeepsItsUnsplitSketchElementName()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-500), new Vec3D(500)), .01);
        var profile = api.GetPlotterSketcher(Geo.DefaultPlanes.OriginXY, "plate_profile");
        profile.AddRectangleFromCorners(new Vec2D(0), new Vec2D(200, 40));
        var blank = api.ExtrudeTwoSides(profile, 5, 5, name: "plate_solid");
        var holes = api.GetPlotterSketcher(Geo.DefaultPlanes.OriginXY, "holes");
        holes.AddCircle(new Vec2D(20, 20), 5);
        holes.AddCircle(new Vec2D(180, 20), 5);
        var tool = api.ExtrudeTwoSides(holes, 5, 5, name: "plate_cutter");
        var result = api.Boolean(blank, tool, BooleanOp.Subtract, "plateA");
        result.EnsureCoplanarPostProcessed();
        Assert.Contains("plate_cutter-Circle1", result.groupIdToExtendedName.Values);
        var body = api.GetAssembly("holes").AddPart(result, new Vec3D(0));
        Assert.NotNull(body.AddAxisDatum("Circle1"));

        var laterHole = api.GetPlotterSketcher(Geo.DefaultPlanes.OriginXY, "later_hole");
        laterHole.AddCircle(new Vec2D(100, 20), 3);
        var laterTool = api.ExtrudeTwoSides(laterHole, 5, 5, name: "later_cutter");
        var later = api.Boolean(result, laterTool, BooleanOp.Subtract, "plateB");
        later.EnsureCoplanarPostProcessed();
        int circle = later.extendedNameToGroupId["plate_cutter-Circle1"];
        Assert.False(later.FaceLineages[circle].Split);
        Assert.True(later.FaceLineages[circle].Supported);
    }
}
