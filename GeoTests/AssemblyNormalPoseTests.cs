using Geo;
using GeoCore;

namespace GeoTests;

public class AssemblyNormalPoseTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void SharedPartNormalsFollowEachPoseWithoutAccumulatingRotations()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(20)), .01);
        var block = api.CreateCuboid(new Vec3D(0), new Vec3D(1, 2, 3), "block");
        api.GetAssembly("poses").AddPart(block, new Vec3D(0));
        var rotated = new Transform(new Vec3D(2, 0, 0),
            new Quaternion(0, 0, Math.Sqrt(.5), Math.Sqrt(.5)));
        var identity = new Transform(new Vec3D(0), new Quaternion(0, 0, 0, 1));

        block.Update(rotated);
        AssertAligned(block.Mesh);
        block.Update(new Transform(new Vec3D(4, -3, 1), rotated.Orientation));
        AssertAligned(block.Mesh);
        block.Update(identity);
        AssertAligned(block.Mesh);
        block.Update(rotated);
        AssertAligned(block.Mesh);
        AssertAligned(block.SnapshotRigidPose(identity));
        AssertAligned(block.SnapshotRigidPose(rotated));
    }

    private static void AssertAligned(MeshNormalUV mesh)
    {
        for (int i = 0; i < mesh.Triangles.Count; i++)
        {
            var tri = mesh.Triangles[i];
            var a = mesh.Positions[tri.A];
            var b = mesh.Positions[tri.B];
            var c = mesh.Positions[tri.C];
            var geometric = Vec3DOps.Cross(b - a, c - a);
            if (geometric.LengthSquared() < 1e-15) continue;
            var corner = mesh.TrianglesEx[i];
            Assert.True(Vec3DOps.Dot(geometric, corner.V0.Normal) > 0);
            Assert.True(Vec3DOps.Dot(geometric, corner.V1.Normal) > 0);
            Assert.True(Vec3DOps.Dot(geometric, corner.V2.Normal) > 0);
        }
    }
}
