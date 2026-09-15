using GeoCore;
using Xunit;

namespace GeoTests;

public class TransformMathTests
{
    [Fact]
    public void Compose_with_identity_is_unchanged()
    {
        var local = new Transform(new Vec3D(3, -2, 5), new Quaternion(0, 0, 0, 1));
        var identity = new Transform(default, TransformMath.IdentityOrientation);
        Transform composed = TransformMath.Compose(identity, local);
        AssertNear(local.Position, composed.Position);
        AssertNear(local.Orientation, composed.Orientation);
    }

    [Fact]
    public void Inverse_undoes_compose()
    {
        var parent = new Transform(new Vec3D(10, 20, 0), AxisAngle(new Vec3D(0, 0, 1), 0.5 * Math.PI));
        var local = new Transform(new Vec3D(1, 0, 0), AxisAngle(new Vec3D(0, 1, 0), 0.25 * Math.PI));
        Transform world = TransformMath.Compose(parent, local);
        Transform back = TransformMath.Compose(TransformMath.Inverse(parent), world);
        AssertNear(local.Position, back.Position);
        AssertNear(local.Orientation, back.Orientation);
    }

    private static Quaternion AxisAngle(Vec3D axis, double radians)
    {
            axis = axis.Normalized();
        double half = 0.5 * radians;
        double s = Math.Sin(half);
        return new Quaternion(axis.X * s, axis.Y * s, axis.Z * s, Math.Cos(half));
    }

    private static void AssertNear(Vec3D a, Vec3D b, double tol = 1e-9)
    {
        Assert.True(
            Math.Abs(a.X - b.X) < tol && Math.Abs(a.Y - b.Y) < tol && Math.Abs(a.Z - b.Z) < tol,
            $"expected {a.X},{a.Y},{a.Z} but got {b.X},{b.Y},{b.Z}");
    }

    private static void AssertNear(Quaternion a, Quaternion b, double tol = 1e-9)
    {
        bool same = Math.Abs(a.X - b.X) < tol && Math.Abs(a.Y - b.Y) < tol
            && Math.Abs(a.Z - b.Z) < tol && Math.Abs(a.W - b.W) < tol;
        bool flipped = Math.Abs(a.X + b.X) < tol && Math.Abs(a.Y + b.Y) < tol
            && Math.Abs(a.Z + b.Z) < tol && Math.Abs(a.W + b.W) < tol;
        Assert.True(same || flipped, $"expected quat {a.X},{a.Y},{a.Z},{a.W} but got {b.X},{b.Y},{b.Z},{b.W}");
    }
}
