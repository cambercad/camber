using Geo;
using GeoCore;

namespace GeoTests;

public class NamedModelReferenceTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(4)]
    [InlineData(9)]
    public void FaceReferenceUsesCurrentConstructionPlane(double height)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01);
        var body = api.CreateCuboid(new Vec3D(0), new Vec3D(2, 3, height), "block");
        var top = AssemblyDatumResolver.ResolvePlaneFrame(body, "block:block-ExtrudeTop", api.Converter);
        Assert.InRange(Math.Abs(top.Origin.Z - height), 0, api.Converter.SmallestUnit());
        Assert.InRange((top.Z - new Vec3D(0, 0, 1)).Length(), 0, 1e-12);
        var bottom = AssemblyDatumResolver.ResolvePlaneFrame(body, "block:block-ExtrudeBottom", api.Converter);
        Assert.InRange((bottom.Z - new Vec3D(0, 0, -1)).Length(), 0, 1e-12);
    }

    [Fact]
    public void CurvedFaceDoesNotBecomeAnApproximateSketchPlane()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01);
        var body = api.CreateCylinder(CoordinateSystem.Default, 2, 3, .01, "round");
        Assert.Throws<ArgumentException>(() =>
            AssemblyDatumResolver.ResolvePlaneFrame(body, "round:round-Circle1", api.Converter));
    }
}
