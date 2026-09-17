using Geo;
using GeoCore;

namespace GeoTests;

public class AssemblyExactGeometryTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    static AnchorMesh RotatedPlate(GeoAPI api)
    {
        double angle = .731;
        var x = new Vec3D(Math.Cos(angle), Math.Sin(angle), 0);
        var y = new Vec3D(-Math.Sin(angle), Math.Cos(angle), 0);
        var sketch = api.GetPlotterSketcher(new CoordinateSystem(new Vec3D(.123, .456, .789),
            x, y, new Vec3D(0, 0, 1)), "PlateSketch");
        sketch.AddRectangleFromCorners(new Vec2D(-12, -9), new Vec2D(12, 9));
        return api.Extrude(sketch, 3, name: "Plate");
    }

    static BigRationalHybrid SixVolume(AnchorMesh solid)
    {
        var p = solid.Mesh.PrecisionPositions;
        var result = BigRationalHybrid.Zero;
        foreach (var t in solid.Mesh.Triangles)
            result += Rat3Hybrid.Dot(p[t.A], Rat3Hybrid.Cross(p[t.B], p[t.C]));
        return result;
    }

    [Fact]
    public void AssemblyUpdatesPreserveExactVolumeAndRestGeometry()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .03);
        var plate = RotatedPlate(api);
        var rest = plate.Mesh.PrecisionPositions.ToArray();
        var volume = SixVolume(plate);
        var assembly = api.GetAssembly("Test");
        assembly.FixPart(assembly.AddPart(plate, new Vec3D(0)));
        for (int i = 0; i < rest.Length; i++)
            Assert.Equal(rest[i], plate.Mesh.PrecisionPositions[i]);
        for (int i = 1; i <= 12; i++)
        {
            double a = i*.17;
            // Quaternion homogeneous coordinates do not require an irrational
            // normalization: the exact rotation divides by their squared norm.
            var pose = new Transform(new Vec3D(i*.123, -i*.234, i*.345),
                new Quaternion(Math.Sin(a), 0, 0, Math.Cos(a)));
            plate.Update(pose);
            Assert.True(volume == SixVolume(plate), "Rigid motion must preserve exact signed volume");
        }
        plate.Update(new Transform(new Vec3D(0), TransformMath.IdentityOrientation));
        for (int i = 0; i < rest.Length; i++)
            Assert.Equal(rest[i], plate.Mesh.PrecisionPositions[i]);
    }

    [Fact]
    public void ReusingAnAlreadySolvedSolidDoesNotRecaptureItsWorldPose()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .03);
        var plate = RotatedPlate(api);
        var rest = plate.Mesh.Positions[0];
        var first = api.GetAssembly("First");
        first.FixPart(first.AddPart(plate, new Vec3D(10, 0, 0)));
        var second = api.GetAssembly("Second");
        second.FixPart(second.AddPart(plate, new Vec3D(20, 0, 0)));
        Assert.InRange((plate.Mesh.Positions[0] - rest - new Vec3D(20, 0, 0)).Length(), 0, 1e-10);
    }
}
