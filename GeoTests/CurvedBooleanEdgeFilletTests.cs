using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public sealed class CurvedBooleanEdgeFilletTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SphereTrimmedBeamCanFilletBeamEdgesOrAllEdges(bool allEdges, bool eraseSurfaceMetadata)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-22, -16, -16), new Vec3D(22, 16, 16)), .01);
        // The 40 mm beam is cut back to the sphere's roughly 15 mm axial extent.
        var beam = api.CreateCuboid(new Vec3D(-20, -5, -5), new Vec3D(20, 5, 5), "beam");
        var sphere = api.CreateSphere(CoordinateSystem.Default, 15, .01, "sphere");

        var result = api.Boolean(beam, sphere, BooleanOp.Intersect, "beam_sphere");
        if (eraseSurfaceMetadata)
        {
            bool foundSpherePatch = false;
            foreach (var (groupId, patchName) in result.groupIdToExtendedName)
                if (patchName.Contains("sphere", StringComparison.OrdinalIgnoreCase))
                {
                    result.surfaceMetaData[patchName] = new SurfaceMetaData(SurfaceType.Unknown);
                    foundSpherePatch = true;
                }
            Assert.True(foundSpherePatch, "Expected the boolean result to retain a spherical patch.");
        }

        AssertEdgesCanBeFilleted(api, result, allEdges);
    }

    [Fact]
    public void SphereTrimmedBeamCanFilletOneBeamEdge()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-22, -16, -16), new Vec3D(22, 16, 16)), .01);
        var beam = api.CreateCuboid(new Vec3D(-20, -5, -5), new Vec3D(20, 5, 5), "beam");
        var sphere = api.CreateSphere(CoordinateSystem.Default, 15, .01, "sphere");
        var result = api.Boolean(beam, sphere, BooleanOp.Intersect, "beam_sphere");

        AssertEdgesCanBeFilleted(api, result, allEdges: false, singleEdge: true);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderTrimmedBeamCanFilletBeamEdgesOrAllEdges(bool allEdges)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-22, -16, -16), new Vec3D(22, 16, 16)), .05);
        // Its axial span covers the beam thickness, leaving the cylindrical side
        // to trim both long ends of the beam.
        var beam = api.CreateCuboid(new Vec3D(-20, -5, -5), new Vec3D(20, 5, 5), "beam");
        var cylinder = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, -8)),
            15, 16, .05, "cylinder");

        var result = api.Boolean(beam, cylinder, BooleanOp.Intersect, "beam_cylinder");

        AssertEdgesCanBeFilleted(api, result, allEdges);
    }

    private static void AssertEdgesCanBeFilleted(GeoAPI api, AnchorMesh solid, bool allEdges, bool singleEdge = false)
    {
        solid.EnsureCoplanarPostProcessed();
        var edges = new List<string>();
        for (int i = 0; i < solid.GroupEdges.Count; i++)
        {
            var edge = solid.GroupEdges[i];
            string first = solid.groupIdToExtendedName[edge.GroupIdA];
            string second = solid.groupIdToExtendedName[edge.GroupIdB];
            bool beamEdge = first.Contains("beam") && second.Contains("beam");
            if (allEdges || beamEdge)
                edges.Add(solid.GetEdgeReference(i));
        }
        Assert.NotEmpty(edges);
        if (singleEdge && edges.Count > 1)
            edges.RemoveRange(1, edges.Count - 1);
        if (!allEdges)
            Assert.True(singleEdge || edges.Count >= 4, string.Join(Environment.NewLine,
            solid.GroupEdges.Select(edge => edge.Name)));

        var usedPositions = new HashSet<int>();
        foreach (var triangle in solid.Mesh.Triangles)
        {
            usedPositions.Add(triangle.A);
            usedPositions.Add(triangle.B);
            usedPositions.Add(triangle.C);
        }
        Assert.InRange(usedPositions.Min(index => solid.Mesh.Positions[index].X), -16, -13);
        Assert.InRange(usedPositions.Max(index => solid.Mesh.Positions[index].X), 13, 16);

        double before = Volume(solid);
        var rounded = api.Fillet(solid, edges, .25, .05, allEdges ? "rounded_all" : "rounded_beam");

        Assert.True(MeshAnalysis.IsWatertightMesh(rounded.Mesh.PrecisionPositions, rounded.Mesh.Triangles));
        Assert.InRange(Volume(rounded), 0, before);
        Assert.True(Volume(rounded) < before);
    }

    private static double Volume(AnchorMesh mesh) => Math.Abs(mesh.Mesh.Triangles.Sum(triangle =>
        Vec3DOps.Dot(mesh.Mesh.Positions[triangle.A],
            Vec3DOps.Cross(mesh.Mesh.Positions[triangle.B], mesh.Mesh.Positions[triangle.C]))) / 6);
}
