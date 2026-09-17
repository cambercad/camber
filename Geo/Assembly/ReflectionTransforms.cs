using GeoCore;

namespace Geo;

/// <summary>Handedness maps shared by mirrored bodies and their assembly datums.</summary>
internal static class ReflectionTransforms
{
    // Every opposite-handed body definition uses the same canonical XY mirror.
    public static Vec3D LocalPoint(Vec3D point) => new(point.X, point.Y, -point.Z);

    public static Transform LocalPose(Transform pose)
    {
        var q = TransformMath.NormalizeDefault(pose.Orientation);
        return new Transform(LocalPoint(pose.Position), new Quaternion(-q.X, -q.Y, q.Z, q.W));
    }

    /// <summary>Proper placement M*T*J for a body whose local geometry is J-mirrored.</summary>
    public static Transform Placement(Transform pose, CoordinateSystem plane)
    {
        _ = PatternTransforms.Circular(1, plane, 0); // shared finite/orthonormal frame validation
        var n = plane.Z.Normalized();
        var point = pose.Position - n * (2 * Vec3DOps.Dot(n, pose.Position - plane.Origin));
        // M*J is the product of two reflections, hence an ordinary rotation.
        var planeRotation = new Quaternion(-n.Y, n.X, 0, n.Z);
        var local = LocalPose(pose);
        return new Transform(point, TransformMath.ComposeOrientations(planeRotation, local.Orientation));
    }
}
