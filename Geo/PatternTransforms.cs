using GeoCore;

namespace Geo;

/// <summary>Shared world-space placements for solid and assembly patterns.</summary>
internal static class PatternTransforms
{
    internal static void ValidateCount(int count)
    {
        if (count < 1) throw new ArgumentOutOfRangeException(nameof(count), "Pattern count includes the seed and must be positive.");
    }

    static void Finite(Vec3D value, string name)
    {
        if (!double.IsFinite(value.X) || !double.IsFinite(value.Y) || !double.IsFinite(value.Z))
            throw new ArgumentException("Pattern coordinates must be finite.", name);
    }

    public static IReadOnlyList<Transform> Linear(int count, Vec3D step)
    {
        ValidateCount(count);
        Finite(step, nameof(step));
        var result = new Transform[count];
        for (int i = 0; i < count; i++)
        {
            var position = step * i;
            Finite(position, nameof(step));
            result[i] = new Transform(position, TransformMath.IdentityOrientation);
        }
        return result;
    }

    public static IReadOnlyList<Transform> Circular(int count, CoordinateSystem axis, double angle)
    {
        ValidateCount(count);
        Finite(axis.Origin, nameof(axis));
        Finite(axis.X, nameof(axis)); Finite(axis.Y, nameof(axis)); Finite(axis.Z, nameof(axis));
        _ = new CoordinateSystem(axis.Origin, axis.X, axis.Y, axis.Z);
        if (!double.IsFinite(angle) || Math.Abs(angle) > 2 * Math.PI)
            throw new ArgumentOutOfRangeException(nameof(angle), "Circular sweep must be finite and between -2π and 2π radians.");
        var direction = axis.Z.Normalized();
        var result = new Transform[count];
        result[0] = new Transform(new Vec3D(0), TransformMath.IdentityOrientation);
        // Exact ±2π denotes a full circle; other sweeps include both ends.
        double increment = count == 1 ? 0 : angle / (Math.Abs(angle) == 2 * Math.PI ? count : count - 1);
        for (int i = 1; i < count; i++)
        {
            double half = increment * i / 2;
            double s = Math.Sin(half);
            var q = new Quaternion(direction.X * s, direction.Y * s, direction.Z * s, Math.Cos(half));
            var offset = axis.Origin - TransformMath.RotateVector(in q, axis.Origin);
            Finite(offset, nameof(axis));
            result[i] = new Transform(offset, q);
        }
        return result;
    }
}
