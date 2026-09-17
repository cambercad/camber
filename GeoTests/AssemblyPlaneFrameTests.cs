using Geo;
using GeoCore;

namespace GeoTests;

public class AssemblyPlaneFrameTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(false, false, true)]
    public void LocalFramePreservesAuthoredAxisOrUsesStableFallbackAfterDisplay(
        bool missingMetadata, bool degenerateDirection, bool nullMetadata = false)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-100), new Vec3D(100)), .01);
        var solid = api.CreateCuboid(new Vec3D(0), new Vec3D(2, 3, 4), "block");
        const string topName = "block-ExtrudeTop";
        var authoredDirection = new Vec3D(.6, .8, 0);
        solid.surfaceMetaData[topName].PlaneParams.RefDir =
            degenerateDirection ? new Vec3D(0, 0, 1) : authoredDirection;
        if (missingMetadata) solid.surfaceMetaData.Remove(topName);
        if (nullMetadata) solid.surfaceMetaData[topName] = null;
        var assembly = api.GetAssembly("frames");
        var first = assembly.AddPart(solid, new Vec3D(10, 20, 30));
        var before = first.GetPlaneFrame(topName);
        var pose = new Transform(new Vec3D(10, 20, 30), new Quaternion(0, Math.Sqrt(.5), 0, Math.Sqrt(.5)));
        solid.Update(pose);
        var second = assembly.AddPart(solid, new Vec3D(-20, 10, 5));
        foreach (var body in new[] { first, second })
        {
            var after = body.GetPlaneFrame(topName);
            Assert.Equal(before.Origin, after.Origin);
            Assert.Equal(before.X, after.X);
            Assert.Equal(before.Y, after.Y);
            Assert.Equal(before.Z, after.Z);
            Assert.InRange((after.Z - new Vec3D(0, 0, 1)).Length(), 0, 1e-12);
            Assert.InRange(Math.Abs(after.X.LengthSquared() - 1), 0, 1e-12);
            Assert.InRange(Math.Abs(Vec3DOps.Dot(after.X, after.Z)), 0, 1e-12);
            Assert.InRange((Vec3DOps.Cross(after.X, after.Y) - after.Z).Length(), 0, 1e-12);
            if (!missingMetadata && !degenerateDirection && !nullMetadata)
                Assert.InRange((after.X - authoredDirection).Length(), 0, 1e-12);
            var bottom = body.GetPlaneFrame("block-ExtrudeBottom");
            Assert.InRange((bottom.Z - new Vec3D(0, 0, -1)).Length(), 0, 1e-12);
        }
    }
}
