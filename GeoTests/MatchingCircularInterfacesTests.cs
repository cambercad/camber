using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public class MatchingCircularInterfacesTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Theory]
    [InlineData(4, .035, false)]
    [InlineData(8, .035, false)]
    [InlineData(4, .2, false)]
    [InlineData(8, .035, true)]
    public void RevolvedJournalMatchesExtrudedBoreDespiteLargerShoulder(double radius, double deviation, bool nearFullTurn)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), deviation);
        var meridian = api.GetPlotterSketcher(new CoordinateSystem(new Vec3D(0),
            new Vec3D(0, 1, 0), new Vec3D(1, 0, 0), new Vec3D(0, 0, -1)), "Meridian");
        var points = new[] { new Vec2D(0, 0), new Vec2D(0, radius*2), new Vec2D(4, radius*2),
            new Vec2D(4, radius), new Vec2D(20, radius), new Vec2D(20, 0) };
        for (int i = 0; i < points.Length; i++)
            meridian.AddLine(points[i], points[(i+1)%points.Length]);
        var shaft = api.Revolve(meridian, nearFullTurn ? 2*Math.PI-5e-11 : 2*Math.PI, name: "Shaft");
        // Same axis; profile axes differ by a quarter turn, as in the pedal.
        var section = api.GetPlotterSketcher(new CoordinateSystem(new Vec3D(0, 8, 0),
            new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0)), "BearingSection");
        section.AddCircle(new Vec2D(0), radius+.8);
        section.AddCircle(new Vec2D(0), radius);
        var bearing = api.Extrude(section, 5, name: "Bearing");
        var assembly = api.GetAssembly("BearingFit");
        assembly.FixPart(assembly.AddPart(shaft, new Vec3D(0)));
        assembly.FixPart(assembly.AddPart(bearing, new Vec3D(0)));
        var overlap = api.Boolean(shaft, bearing, BooleanOp.Intersect, "Overlap");
        Assert.Empty(overlap.Mesh.Triangles);
        var united = api.Boolean(shaft, bearing, BooleanOp.Union, "United");
        united.EnsureCoplanarPostProcessed();
        Assert.True(MeshAnalysis.IsWatertightMesh(united.Mesh.Positions, united.Mesh.Triangles));
    }
}
