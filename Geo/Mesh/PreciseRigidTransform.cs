using GeoCore;

namespace Geo;

/// <summary>
/// Exact affine transform in kernel coordinates. The input quaternion is
/// interpreted as rational homogeneous rotation coordinates, giving an exactly
/// orthogonal rotation without rounding each transformed vertex independently.
/// </summary>
internal sealed class PreciseRigidTransform
{
    readonly Rat3Hybrid rowX, rowY, rowZ, offset;

    static BigRationalHybrid Exact(double value)
    {
        var rational = new BigRational(value);
        var result = new BigRationalHybrid(rational.Numerator, rational.Denominator);
        result.Simplify();
        return result;
    }

    public PreciseRigidTransform(CoordinateConverter converter, in Transform transform)
    {
        var q = TransformMath.NormalizeDefault(transform.Orientation);
        var x = Exact(q.X); var y = Exact(q.Y); var z = Exact(q.Z); var w = Exact(q.W);
        var n = x*x + y*y + z*z + w*w;
        var two = new BigRationalHybrid(2);
        rowX = new Rat3Hybrid((n - two*(y*y+z*z))/n, two*(x*y-w*z)/n, two*(x*z+w*y)/n);
        rowY = new Rat3Hybrid(two*(x*y+w*z)/n, (n - two*(x*x+z*z))/n, two*(y*z-w*x)/n);
        rowZ = new Rat3Hybrid(two*(x*z-w*y)/n, two*(y*z+w*x)/n, (n - two*(x*x+y*y))/n);
        rowX.Simplify(); rowY.Simplify(); rowZ.Simplify();
        var min = converter.OperatingSpace.Min;
        var origin = new Rat3Hybrid(Exact(min.X), Exact(min.Y), Exact(min.Z));
        var t = transform.Position;
        var translation = new Rat3Hybrid(Exact(t.X), Exact(t.Y), Exact(t.Z));
        offset = (Rotate(origin) + translation - origin) / Exact(converter.SmallestUnit());
        offset.Simplify();
    }

    /// <summary>Rotation about a world-space pivot, without rounding its affine offset.</summary>
    public PreciseRigidTransform(CoordinateConverter converter, Quaternion orientation, Vec3D pivot)
        : this(converter, new Transform(new Vec3D(0), orientation))
    {
        var min = converter.OperatingSpace.Min;
        var origin = new Rat3Hybrid(Exact(min.X), Exact(min.Y), Exact(min.Z));
        var point = new Rat3Hybrid(Exact(pivot.X), Exact(pivot.Y), Exact(pivot.Z));
        offset = (Rotate(origin - point) + point - origin) / Exact(converter.SmallestUnit());
        offset.Simplify();
    }

    /// <summary>Exact reflection in the plane through Origin with normal Z.</summary>
    public PreciseRigidTransform(CoordinateConverter converter, CoordinateSystem plane)
    {
        var n = new Rat3Hybrid(Exact(plane.Z.X), Exact(plane.Z.Y), Exact(plane.Z.Z));
        var denominator = Rat3Hybrid.Dot(n, n);
        if (denominator.Sign() == 0) throw new ArgumentException("Mirror plane needs a nonzero normal.", nameof(plane));
        var two = new BigRationalHybrid(2);
        rowX = new Rat3Hybrid(new BigRationalHybrid(1) - two*n.X*n.X/denominator, -two*n.X*n.Y/denominator, -two*n.X*n.Z/denominator);
        rowY = new Rat3Hybrid(-two*n.Y*n.X/denominator, new BigRationalHybrid(1) - two*n.Y*n.Y/denominator, -two*n.Y*n.Z/denominator);
        rowZ = new Rat3Hybrid(-two*n.Z*n.X/denominator, -two*n.Z*n.Y/denominator, new BigRationalHybrid(1) - two*n.Z*n.Z/denominator);
        rowX.Simplify(); rowY.Simplify(); rowZ.Simplify();
        var min = converter.OperatingSpace.Min;
        var origin = new Rat3Hybrid(Exact(min.X), Exact(min.Y), Exact(min.Z));
        var p = plane.Origin;
        var point = new Rat3Hybrid(Exact(p.X), Exact(p.Y), Exact(p.Z));
        offset = (Rotate(origin - point) + point - origin) / Exact(converter.SmallestUnit());
        offset.Simplify();
    }

    Rat3Hybrid Rotate(Rat3Hybrid point) => new(
        Rat3Hybrid.Dot(rowX, point), Rat3Hybrid.Dot(rowY, point), Rat3Hybrid.Dot(rowZ, point));

    public Rat3Hybrid Apply(Rat3Hybrid point)
    {
        var result = Rotate(point) + offset;
        result.Simplify();
        return result;
    }
}
