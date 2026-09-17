using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class MatchingSweepInterfacesTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MatchingCurvedSleevesHaveIdenticalOppositelyOrientedInterface(bool innerFirst)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .03);
        var guide = new CubicHermiteSpline3D(
            new[] { new Vec3D(0), new Vec3D(12, 4, 25), new Vec3D(20, -5, 50) },
            new Vec3D?[] { new Vec3D(0, 0, 1), new Vec3D(1, 0, 2), new Vec3D(0, 0, 1) });
        AnchorMesh Sleeve(string name, double outer, double inner)
        {
            var profile = api.GetPlotterSketcher(CoordinateSystem.Default, name + "_profile");
            profile.AddCircle(new Vec2D(0), innerFirst ? inner : outer);
            profile.AddCircle(new Vec2D(0), innerFirst ? outer : inner);
            return api.ExtrudeAlongCurve(profile, guide, maxDeviation: .03, name: name);
        }
        var tube = Sleeve("tube", 5, 3);
        var cover = Sleeve("cover", 7, 5);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(tube.Mesh);
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(cover.Mesh);
        var overlap = api.Boolean(tube, cover, BooleanOp.Intersect, "matching_interface");
        Assert.Empty(overlap.Mesh.Triangles);
        var reverse = api.Boolean(cover, tube, BooleanOp.Intersect, "reverse_interface");
        Assert.Empty(reverse.Mesh.Triangles);
        var combined = api.Boolean(tube, cover, BooleanOp.Union, "combined");
        var expected = MeshAnalysis.ComputeSignedMeshVolume(tube.Mesh.PrecisionPositions, tube.Mesh.Triangles)
            + MeshAnalysis.ComputeSignedMeshVolume(cover.Mesh.PrecisionPositions, cover.Mesh.Triangles);
        Assert.Equal(expected, MeshAnalysis.ComputeSignedMeshVolume(combined.Mesh.PrecisionPositions, combined.Mesh.Triangles));
    }
}
