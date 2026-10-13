using Geo;
using GeoCore;

namespace GeoTests;

public sealed class ThreadPrecisionTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(100, false)]
    [InlineData(2000, true)]
    [InlineData(4000, false)]
    public void InternalThreadHelpersRemainDistinctOnThePartsCoordinateLattice(double span, bool downward)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-span / 4, -span / 2, span > 100 ? -100 : -50),
            new Vec3D(span * .75, span / 2, span > 100 ? 1200 : 50)), .01);
        var frame = new CoordinateSystem(new Vec3D(span > 100 ? 165 : 0, -28, -8),
            new Vec3D(1, 0, 0), new Vec3D(0, downward ? -1 : 1, 0), new Vec3D(0, 0, downward ? -1 : 1));
        double length = downward ? 14 : 4;
        var cutter = api.CreateMetricThreadForNutNegative(frame, 8, 1.25, length,
            name: "mast_thread", includeBoreChamfers: false);
        Assert.True(cutter.IsVolume);
        Assert.True(MeshAnalysis.IsWatertightMesh(cutter.Mesh.PrecisionPositions, cutter.Mesh.Triangles));
        Assert.True(MeshAnalysis.AreTrianglesConsistentlyOriented(cutter.Mesh.PrecisionPositions, cutter.Mesh.Triangles));
        Assert.True(MeshAnalysis.ComputeSignedMeshVolume(cutter.Mesh.PrecisionPositions, cutter.Mesh.Triangles).Sign() > 0);
        var used = cutter.Mesh.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct();
        var heights = used.Select(i => Vec3DOps.Dot(cutter.Mesh.Positions[i] - frame.Origin, frame.Z)).ToList();
        double tolerance = 4 * api.Converter.SmallestUnit();
        Assert.InRange(heights.Min(), -tolerance, tolerance);
        Assert.InRange(heights.Max(), length - tolerance, length + tolerance);
    }
}
