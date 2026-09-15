using GeoCore;
using GeoMeta;

namespace GeoTests;

public sealed class EntityNamingTests
{
    [Fact]
    public void PatchFormatters_MatchGoldenStrings()
    {
        Assert.Equal("pipe-ExtrudeBottom", EntityNaming.ExtrudeBottom("pipe"));
        Assert.Equal("pipe-ExtrudeTop", EntityNaming.ExtrudeTop("pipe"));
        Assert.Equal("pipe-Line1", EntityNaming.ExtrudeSide("pipe", "Line1"));
        Assert.Equal("pipe-CircleYZ-CLine0", EntityNaming.ExtrudeSideWithGuide("pipe", "CircleYZ", "CLine0"));

        Assert.Equal("rev-Line1", EntityNaming.RevolveSide("rev", "Line1"));
        Assert.Equal("rev-StartCap", EntityNaming.RevolveStartCap("rev"));
        Assert.Equal("rev-EndCap", EntityNaming.RevolveEndCap("rev"));

        Assert.Equal("loft-Side", EntityNaming.LoftSide("loft"));
        Assert.Equal("loft-StartCap", EntityNaming.LoftStartCap("loft"));
        Assert.Equal("loft-EndCap", EntityNaming.LoftEndCap("loft"));

        Assert.Equal("BlendEdge_[A,B]", EntityNaming.BlendEdge("[A,B]"));
        Assert.Equal("BlendCorner_0", EntityNaming.BlendCorner(0));
        Assert.Equal("ChamferEdge_[A,B]", EntityNaming.ChamferEdge("[A,B]"));
        Assert.Equal("ChamferCorner_0", EntityNaming.ChamferCorner(0));
        Assert.Equal("ObjAutoGroup_3", EntityNaming.ImportAutoGroup(EntityNaming.ObjAutoGroupPrefix, 3));

        Assert.Equal("hole_z-Circle1", EntityNaming.FormatPatchComponentName("hole_z-Circle1", 0));
        Assert.Equal("hole_z-Circle1_1", EntityNaming.FormatPatchComponentName("hole_z-Circle1", 1));
        Assert.Equal("hole_z-Circle1_2", EntityNaming.FormatPatchComponentName("hole_z-Circle1", 2));
    }

    [Fact]
    public void RenameMeshPrefix_DoesNotRewriteLongerPrefix()
    {
        Assert.Equal("pipe10-Line1", EntityNaming.RenameMeshPrefix("pipe10-Line1", "pipe1", "pipe2"));
    }

    [Fact]
    public void RewriteMeshNameInEntity_UpdatesBlendAndBracketNames()
    {
        Assert.Equal("pipe2-south", EntityNaming.RewriteMeshNameInEntity("pipe1-south", "pipe1", "pipe2"));
        Assert.Equal("[pipe2-south,pipe2-west]",
            EntityNaming.RewriteMeshNameInEntity("[pipe1-south,pipe1-west]", "pipe1", "pipe2"));
        Assert.Equal("BlendEdge_[pipe2-south,pipe2-west]",
            EntityNaming.RewriteMeshNameInEntity("BlendEdge_[pipe1-south,pipe1-west]", "pipe1", "pipe2"));
        Assert.Equal("BlendEdge_[pipe10-south,pipe10-west]",
            EntityNaming.RewriteMeshNameInEntity("BlendEdge_[pipe10-south,pipe10-west]", "pipe1", "pipe2"));
    }

    [Fact]
    public void RenameBracketedEdgeName_UpdatesPatchPrefixes()
    {
        Assert.Equal("[pipe-CircleYZ,pipe-ExtrudeTop]",
            EntityNaming.RenameBracketedEdgeName(
                "[ExtrudeAlongCurve1-CircleYZ,ExtrudeAlongCurve1-ExtrudeTop]",
                "ExtrudeAlongCurve1",
                "pipe"));
    }

    [Fact]
    public void ValidateContourSegmentName_ThrowsForReservedNames()
    {
        Assert.Throws<Exception>(() => EntityNaming.ValidateContourSegmentName(EntityNaming.ExtrudeTopSegment));
        Assert.Throws<Exception>(() => EntityNaming.ValidateContourSegmentName(EntityNaming.ExtrudeBottomSegment));
        EntityNaming.ValidateContourSegmentName("Line1");
    }

    [Fact]
    public void MergePatchNameMaps_ThrowsOnConflict()
    {
        var a = new Dictionary<string, int> { { "patch", 1 } };
        var b = new Dictionary<string, int> { { "patch", 2 } };
        Assert.Throws<NameCollisionException>(() => EntityNaming.MergePatchNameMaps(a, b));

        var c = new Dictionary<string, int> { { "other", 2 } };
        EntityNaming.MergePatchNameMaps(a, c);
        Assert.Equal(2, a.Count);
        Assert.Equal(2, a["other"]);
    }

    [Fact]
    public void SurfacePointAddress_RoundTripAndFlexibleDecimals()
    {
        string formatted = EntityNaming.FormatSurfacePointAddress("pipe-Line1", 0.25, 0.5);
        Assert.True(EntityNaming.TryParseSurfacePointAddress(formatted, out var parsed));
        Assert.Equal("pipe-Line1", parsed.PatchName);
        Assert.Equal(0.25, parsed.UniformX, 3);
        Assert.Equal(0.5, parsed.UniformY, 3);

        Assert.True(EntityNaming.TryParseSurfacePointAddress("[pipe-Line1]@0.5,0.25", out var flexible));
        Assert.Equal(0.5, flexible.UniformX, 3);
        Assert.Equal(0.25, flexible.UniformY, 3);
    }

    [Fact]
    public void EdgePointAddress_RoundTrip()
    {
        string formatted = EntityNaming.FormatEdgePointAddress("A", "B", 0.5);
        Assert.True(EntityNaming.TryParseEdgePointAddress(formatted, out var parsed));
        Assert.Equal("A", parsed.PatchA);
        Assert.Equal("B", parsed.PatchB);
        Assert.Equal(0.5, parsed.Uniform, 3);
    }

    [Fact]
    public void GroupEdgeAddress_ParseFullMatch()
    {
        Assert.True(EntityNaming.TryParseGroupEdgeAddress("[A,B]", out var bare, requireFullMatch: true));
        Assert.Equal("A", bare.PatchA);
        Assert.Equal("B", bare.PatchB);
        Assert.Equal(0, bare.EdgeIndex);

        Assert.True(EntityNaming.TryParseGroupEdgeAddress("[A,B]_1", out var indexed, requireFullMatch: true));
        Assert.Equal(1, indexed.EdgeIndex);

        Assert.False(EntityNaming.TryParseGroupEdgeAddress("[A,B]extra", out _, requireFullMatch: true));
    }

    [Fact]
    public void SketchCurveAddress_RoundTrip()
    {
        Assert.True(EntityNaming.TryParseSketchCurveAddress("Line1@0.5", out var param));
        Assert.Equal("Line1", param.CurveName);
        Assert.False(param.IsCenter);
        Assert.False(param.IsControlVertex);
        Assert.Equal(0.5, param.Uniform, 3);

        Assert.True(EntityNaming.TryParseSketchCurveAddress("Circle1@center", out var center));
        Assert.True(center.IsCenter);
        Assert.False(center.IsControlVertex);

        Assert.True(EntityNaming.TryParseSketchCurveAddress("Bezier1@cv2", out var cv));
        Assert.Equal("Bezier1", cv.CurveName);
        Assert.True(cv.IsControlVertex);
        Assert.Equal(2, cv.ControlVertexIndex);
        Assert.False(cv.IsCenter);

        Assert.Equal("north@out_offset", EntityNaming.FormatSketchOffsetCurve("north", true));
        Assert.Equal("north@in_offset", EntityNaming.FormatSketchOffsetCurve("north", false));
        Assert.Equal("north@out_offset_2", EntityNaming.FormatSketchOffsetCurve("north", true, 1));
        Assert.Equal("h@start_cap", EntityNaming.FormatSketchOffsetEndCap("h", true));
        Assert.Equal("h@end_cap", EntityNaming.FormatSketchOffsetEndCap("h", false));
        Assert.Equal("h@end_cap_2", EntityNaming.FormatSketchOffsetEndCap("h", false, 1));
        Assert.Equal("cap[h@out_offset,v@out_offset]",
            EntityNaming.FormatSketchOffsetJoinCap("v@out_offset", "h@out_offset"));
        Assert.Equal("cap[h@out_offset,v@out_offset]_2",
            EntityNaming.FormatSketchOffsetJoinCap("h@out_offset", "v@out_offset", 1));
        Assert.True(EntityNaming.IsSketchOffsetChildName("north@out_offset"));
        Assert.True(EntityNaming.IsSketchOffsetChildName("h@end_cap"));
        Assert.True(EntityNaming.IsSketchOffsetChildName("h@start_cap_2"));
        Assert.True(EntityNaming.IsSketchOffsetChildName("cap[h@out_offset,v@out_offset]"));
        Assert.False(EntityNaming.IsSketchOffsetChildName("north"));
        Assert.True(EntityNaming.TryParseSketchCurveAddress("north@out_offset@0.5", out var offsetPt));
        Assert.Equal("north@out_offset", offsetPt.CurveName);
        Assert.Equal(0.5, offsetPt.Uniform, 3);
        Assert.True(EntityNaming.TryParseSketchCurveAddress("h@end_cap@0.5", out var capPt));
        Assert.Equal("h@end_cap", capPt.CurveName);
        Assert.True(EntityNaming.TryParseSketchCurveAddress("cap[h@out_offset,v@out_offset]@0.5", out var joinPt));
        Assert.Equal("cap[h@out_offset,v@out_offset]", joinPt.CurveName);
        Assert.False(EntityNaming.TryParseSketchCurveAddress("north@out_offset", out _));
    }

    [Fact]
    public void ParseQualifiedCurveName_SplitsSketchPrefix()
    {
        var qualified = EntityNaming.ParseQualifiedCurveName("Sketch1:Line1");
        Assert.Equal("Sketch1", qualified.SketchName);
        Assert.Equal("Line1", qualified.CurveName);
        Assert.True(qualified.HasSketchQualifier);

        var bare = EntityNaming.ParseQualifiedCurveName("Line1");
        Assert.Null(bare.SketchName);
        Assert.Equal("Line1", bare.CurveName);
    }

    [Fact]
    public void ParseQualifiedSketchCurveAddress_SplitsSketchPrefix()
    {
        Assert.True(EntityNaming.TryParseQualifiedSketchCurveAddress("sk1:Line1@0.5", out var qualified));
        Assert.Equal("sk1", qualified.SketchName);
        Assert.Equal("Line1@0.5", qualified.LocalAddress);
        Assert.True(qualified.HasSketchQualifier);

        Assert.True(EntityNaming.TryParseQualifiedSketchCurveAddress("Line1@0.5", out var bare));
        Assert.False(bare.HasSketchQualifier);
        Assert.Equal("Line1@0.5", bare.LocalAddress);

        Assert.False(EntityNaming.TryParseQualifiedSketchCurveAddress("sk1:Line1", out _));
    }

    [Fact]
    public void GenerateGlobalName_IncrementsPerPrefix()
    {
        EntityNaming.ResetGlobalNameCounters();
        Assert.Equal("Part1", EntityNaming.GenerateGlobalName("Part"));
        Assert.Equal("Part2", EntityNaming.GenerateGlobalName("Part"));
        Assert.Equal("Extrude1", EntityNaming.GenerateGlobalName("Extrude"));
    }

    [Fact]
    public void BuildGroupIdMaps_AreInverses()
    {
        var nameToId = new Dictionary<string, int> { { "a", 1 }, { "b", 2 } };
        var idToName = EntityNaming.BuildGroupIdToName(nameToId);
        var roundTrip = EntityNaming.BuildNameToGroupId(idToName);
        Assert.Equal(nameToId["a"], roundTrip["a"]);
        Assert.Equal(nameToId["b"], roundTrip["b"]);
    }

    [Fact]
    public void MatchesGroupEdgeName_SupportsIndexedSuffix()
    {
        Assert.True(EntityNaming.MatchesGroupEdgeName("[A,B]_0", "[A,B]"));
        Assert.True(EntityNaming.MatchesGroupEdgeName("[A,B]", "[A,B]"));
        Assert.False(EntityNaming.MatchesGroupEdgeName("[A,B]_1", "[A,C]"));
    }

    [Fact]
    public void LegacyEdgePointAddress_ParsesWithKnownPatches()
    {
        var patches = new[] { "pipe-Line1", "pipe-ExtrudeTop" };
        string legacy = EntityNaming.FormatLegacyEdgePointName("pipe-Line1", "pipe-ExtrudeTop", 0, 0.5);
        Assert.True(EntityNaming.TryParseLegacyEdgePointAddress(legacy, patches, out var parsed));
        Assert.Equal("pipe-Line1", parsed.PatchA);
        Assert.Equal("pipe-ExtrudeTop", parsed.PatchB);
        Assert.Equal(0.5, parsed.Uniform, 3);
    }

    [Fact]
    public void QualifiedMeshAddress_SplitsMeshPrefixForBracketAnchors()
    {
        bool IsMesh(string n) => n == "pipe" || n == "cube";
        Assert.True(EntityNaming.TryParseQualifiedMeshAddress("pipe:[A,B]@0.5", IsMesh, out var q));
        Assert.Equal("pipe", q.MeshName);
        Assert.Equal("[A,B]@0.5", q.LocalAddress);

        Assert.False(EntityNaming.TryParseQualifiedMeshAddress("Sketch1:Line1", IsMesh, out _));
    }

    [Fact]
    public void GetSketchCurveTypePrefix_MapsConstrainedTypes()
    {
        Assert.Equal("Line", EntityNaming.GetSketchCurveTypePrefix("CLine2D"));
        Assert.Equal("Circle", EntityNaming.GetSketchCurveTypePrefix("Circle2D"));
    }

    [Fact]
    public void SketchHandleAddresses_MatchInteractiveRoles()
    {
        Assert.Equal("Line1@0.000", EntityNaming.FormatSketchHandleAddress("Line1", "line", 0));
        Assert.Equal("Line1@1.000", EntityNaming.FormatSketchHandleAddress("Line1", "line", 1));
        Assert.Equal("Line1@0.500", EntityNaming.FormatSketchHandleAddress("Line1", "line", 3));
        Assert.Equal("Circle1@center", EntityNaming.FormatSketchHandleAddress("Circle1", "circle", 2));
        Assert.Equal("Circle1@0.000", EntityNaming.FormatSketchHandleAddress("Circle1", "circle", 3));
        Assert.Equal("Circle1@0.250", EntityNaming.FormatSketchHandleAddress("Circle1", "circle", 4));
        Assert.Equal("Circle1@0.500", EntityNaming.FormatSketchHandleAddress("Circle1", "circle", 5));
        Assert.Equal("Circle1@0.750", EntityNaming.FormatSketchHandleAddress("Circle1", "circle", 6));
        Assert.Equal("Arc1@center", EntityNaming.FormatSketchHandleAddress("Arc1", "arc", 2));
        Assert.Equal("", EntityNaming.FormatSketchHandleAddress("Line1", "line", 2));
    }

    [Fact]
    public void QualifyAndSketchCurveAddress_RoundTrip()
    {
        string local = EntityNaming.FormatSketchCurveAddress("Line1", 0.5);
        string qualified = EntityNaming.Qualify("Sketch1", local);
        Assert.Equal("Sketch1:Line1@0.500", qualified);
        Assert.True(EntityNaming.TryParseQualifiedSketchCurveAddress(qualified, out var parsed));
        Assert.Equal("Sketch1", parsed.SketchName);
        Assert.Equal("Line1@0.500", parsed.LocalAddress);
        Assert.Equal("Line1", EntityNaming.Qualify("", "Line1"));
    }

    [Fact]
    public void SketchCurveDump_IncludesCurveName()
    {
        Assert.Equal("L\tLine1\t0\t0\t2\t0", SketchCurveDump.Line("Line1", 0, 0, 2, 0));
        Assert.Equal("C\tCircle1\t1\t2\t0.5", SketchCurveDump.Circle("Circle1", 1, 2, 0.5));
    }
}
