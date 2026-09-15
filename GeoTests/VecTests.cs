using GeoCore;

namespace GeoTests;

public class VecTests
{
    private const double Eps = 1e-9;

    [Fact]
    public void Vec2D_BasicOperations_Work()
    {
        var a = new Vec2D(3, 4);
        var b = new Vec2D(1, 2);

        var sum = a + b;
        var diff = a - b;
        var scaled = 2.0 * b;

        Assert.Equal(4, sum.X, Eps);
        Assert.Equal(6, sum.Y, Eps);
        Assert.Equal(2, diff.X, Eps);
        Assert.Equal(2, diff.Y, Eps);
        Assert.Equal(2, scaled.X, Eps);
        Assert.Equal(4, scaled.Y, Eps);
        Assert.Equal(11, Vec2DOps.Dot(a, b), Eps);
        Assert.Equal(2, Vec2DOps.Cross(a, b), Eps);
        Assert.Equal(5, a.Length(), Eps);
    }

    [Fact]
    public void Vec3D_BasicOperations_Work()
    {
        var x = new Vec3D(1, 0, 0);
        var y = new Vec3D(0, 1, 0);
        var z = Vec3DOps.Cross(x, y);

        Assert.Equal(0, Vec3DOps.Dot(x, y), Eps);
        Assert.Equal(0, z.X, Eps);
        Assert.Equal(0, z.Y, Eps);
        Assert.Equal(1, z.Z, Eps);
        Assert.Equal(1, z.Length(), Eps);
    }

    [Fact]
    public void CoordinateSystem_PointTransform_RoundTrips()
    {
        var cs = new CoordinateSystem(
            new Vec3D(10, 20, 30),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(0, 0, 1));

        var local = new Vec3D(1.5, -2.5, 3.5);
        var world = cs.PointFromCoordSysToWorld(local);
        var localRoundtrip = cs.PointFromWorldToCoordSys(world);

        Assert.Equal(local.X, localRoundtrip.X, Eps);
        Assert.Equal(local.Y, localRoundtrip.Y, Eps);
        Assert.Equal(local.Z, localRoundtrip.Z, Eps);
    }

    [Fact]
    public void CoordinateSystem_OrthonormalCheck_RejectsNonUnitAxis()
    {
        var ex = Assert.Throws<Exception>(() => _ = new CoordinateSystem(
            new Vec3D(0),
            new Vec3D(1.0005, 0, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(0, 0, 1)));
        Assert.Contains("not orthonormal", ex.Message);
        Assert.Contains(nameof(CoordinateSystem.X), ex.Message);
    }

    [Fact]
    public void CoordinateSystem_OrthonormalCheck_RejectsNonPerpendicularAxes()
    {
        // Unit vectors in the XY plane separated by 45° → X·Y = cos(45°) = 1/√2
        double a = 1.0 / Math.Sqrt(2);
        var ex = Assert.Throws<Exception>(() => _ = new CoordinateSystem(
            new Vec3D(0),
            new Vec3D(1, 0, 0),
            new Vec3D(a, a, 0),
            new Vec3D(0, 0, 1)));
        Assert.Contains("not orthonormal", ex.Message);
        Assert.Contains("perpendicular", ex.Message);
    }

    [Fact]
    public void CoordinateSystem_OrthonormalCheck_CanBeDisabled()
    {
        var cs = new CoordinateSystem(
            new Vec3D(0),
            new Vec3D(2, 0, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(0, 0, 0.5),
            checkRightHanded: false,
            checkOrthoNormal: false);
        Assert.Equal(2.0, cs.X.X, Eps);
    }

    [Fact]
    public void CoordinateSystem_LeftHanded_Orthonormal_Throws()
    {
        var ex = Assert.Throws<Exception>(() => _ = new CoordinateSystem(
            new Vec3D(0),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(0, 0, -1)));
        Assert.Equal("left handed", ex.Message);
    }

    [Fact]
    public void CoordinateSystem_ToLocal_ToGlobal_PreservesOrthonormalFrame()
    {
        var world = new CoordinateSystem(
            new Vec3D(1, 2, 3),
            new Vec3D(1, 0, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(0, 0, 1));
        var local = new CoordinateSystem(
            new Vec3D(0.5, -1, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(1, 0, 0));
        var backInWorld = world.ToGlobal(local);
        var roundtrip = world.ToLocal(backInWorld);
        Assert.Equal(local.Origin.X, roundtrip.Origin.X, Eps);
        Assert.Equal(local.Origin.Y, roundtrip.Origin.Y, Eps);
        Assert.Equal(local.Origin.Z, roundtrip.Origin.Z, Eps);
    }

    [Fact]
    public void Box3D_ContainsAndOverlap_Work()
    {
        var a = new Box3D(new Vec3D(0, 0, 0), new Vec3D(1, 1, 1), 0);
        var b = new Box3D(new Vec3D(0.5, 0.5, 0.5), new Vec3D(2, 2, 2), 0);
        var c = new Box3D(new Vec3D(2.1, 2.1, 2.1), new Vec3D(3, 3, 3), 0);

        Assert.True(a.ContainsPoint(new Vec3D(0.25, 0.25, 0.25)));
        Assert.False(a.ContainsPoint(new Vec3D(1.1, 0.25, 0.25)));
        Assert.True(Box3D.Overlap(a, b));
        Assert.False(Box3D.Overlap(a, c));
    }

    [Fact]
    public void Mat4D_Inverse_AndTransformPoint_Work()
    {
        var translation = new Mat4D
        {
            M11 = 1,
            M22 = 1,
            M33 = 1,
            M44 = 1,
            M14 = 5,
            M24 = -3,
            M34 = 2
        };

        var p = new Vec3D(1, 2, 3);
        var transformed = translation.TransformPoint(p);
        Assert.Equal(6, transformed.X, Eps);
        Assert.Equal(-1, transformed.Y, Eps);
        Assert.Equal(5, transformed.Z, Eps);

        Assert.True(Mat4DOps.Inverse(translation, out var inverse));
        var roundtrip = inverse.TransformPoint(transformed);

        Assert.Equal(p.X, roundtrip.X, Eps);
        Assert.Equal(p.Y, roundtrip.Y, Eps);
        Assert.Equal(p.Z, roundtrip.Z, Eps);
    }
}
