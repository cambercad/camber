using Geo;
using GeoCore;
using Curves;

namespace GeoTests;

[Collection("GlobalCadState")]
public sealed class PartialCornerBlendTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TransformedPolygonLoftDoesNotMistakeRealCreasesForSmoothSeams(bool chamfer)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-40), new Vec3D(40)), .01);
        var sections = new List<PlotterSketcherCoordSys>();
        for (int i = 0; i < 3; i++)
        {
            var sketch = new PlotterSketcherCoordSys("section" + i,
                new CoordinateSystem(new Vec3D(0,0,i*2)), new Vec2D(0));
            sketch.AddRectangleFromCorners(new Vec2D(0), new Vec2D(20,12));
            sections.Add(sketch);
        }
        var blank = api.Loft(sections,
            new LoftOptions { CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices }, "boxloft");
        blank = api.CopyMesh(blank, CoordinateSystem.Default,
            new CoordinateSystem(new Vec3D(2,-3,1), new Vec3D(.6,.8,0),
                new Vec3D(-.48,.36,.8), new Vec3D(.64,-.48,.6)),
            "posed_", SurfacePatchNameAffix.Prefix, "posed");
        var edges = new List<string>();
        foreach (var edge in blank.GroupEdges)
            if (edge.Name.Contains("EndCap") &&
                (edge.Name.Contains("Side-Line1") || edge.Name.Contains("Side-Line2"))) edges.Add(edge.Name);
        Assert.Equal(2, edges.Count);
        var result = chamfer ? api.Chamfer(blank, edges, .75, .01, "treated")
            : api.Fillet(blank, edges, .75, .01, "treated");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles, true));
        int strips = 0;
        foreach (string name in result.groupIdToExtendedName.Values)
            if (name.StartsWith(chamfer ? "ChamferEdge_" : "BlendEdge_")) strips++;
        Assert.Equal(2, strips);
    }

    [Theory]
    [InlineData(false, 1)]
    [InlineData(false, 2)]
    [InlineData(false, 3)]
    [InlineData(false, 4)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    [InlineData(true, 3)]
    [InlineData(true, 4)]
    public void TopEdgeTreatmentsLeaveOtherIncidentEdgesSharp(bool chamfer, int selectedCount)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), .01);
        var box = api.CreateCuboid(new Vec3D(0), new Vec3D(20,12,4), "box");
        var edges = new List<string>();
        for (int i = 1; i <= selectedCount; i++) edges.Add($"[box-Line{i},box-ExtrudeTop]");
        var result = chamfer ? api.Chamfer(box, edges, .75, .01, "treated")
            : api.Fillet(box, edges, .75, .01, "treated");
        Assert.True(MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions, result.Mesh.Triangles, true));
        Assert.InRange(MeshAnalysis.ComputeSignedMeshVolume(result.Mesh.Positions, result.Mesh.Triangles), 920, 959.99);
        Assert.Contains("box-ExtrudeBottom", result.extendedNameToGroupId.Keys);

        var used = new HashSet<Rat3Hybrid>();
        foreach (var triangle in result.Mesh.Triangles)
        {
            used.Add(result.Mesh.PrecisionPositions[triangle.A]);
            used.Add(result.Mesh.PrecisionPositions[triangle.B]);
            used.Add(result.Mesh.PrecisionPositions[triangle.C]);
        }
        // All four sharp bottom corners survive exactly, not merely within
        // the tessellation tolerance. No vertical/bottom edges were selected.
        foreach (var point in box.Mesh.PrecisionPositions)
            if (api.Converter.Convert(point).Z < .001) Assert.Contains(point, used);
    }
}
