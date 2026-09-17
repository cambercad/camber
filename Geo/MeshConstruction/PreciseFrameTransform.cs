using GeoCore;

namespace Geo;

/// <summary>
/// Quantizes coordinates in the construction frame, then solves the three
/// frame-plane equations exactly. Independent world-vertex rounding tilts
/// nominally shared planes and can turn face contact into edge contact.
/// Plane offsets use the existing lattice unit; directions use 40 binary bits.
/// This retains the existing operating-space scale and makes parallel construction planes exact.
/// </summary>
internal sealed class PreciseFrameTransform
{
    const long DirectionScale = 1L << 40;
    readonly Vec3D[] directions;
    readonly Vec3D[] normals;
    readonly Rat3Hybrid[] inverseColumns;
    readonly double[] originOffsets;
    readonly double scaling;

    static BigRationalHybrid Quantize(double value, long scale) =>
        new BigRationalHybrid(checked((long)Math.Round(value * scale)), scale);

    public PreciseFrameTransform(CoordinateConverter converter, CoordinateSystem frame)
    {
        directions = new[] { frame.X, frame.Y, frame.Z };
        normals = new Vec3D[3];
        var exactNormals = new Rat3Hybrid[3];
        originOffsets = new double[3];
        scaling = 1 / converter.SmallestUnit();
        for (int i = 0; i < 3; i++)
        {
            Vec3D direction = directions[i];
            double pivot = Math.Abs(direction.X) >= Math.Abs(direction.Y) && Math.Abs(direction.X) >= Math.Abs(direction.Z)
                ? direction.X : Math.Abs(direction.Y) >= Math.Abs(direction.Z) ? direction.Y : direction.Z;
            // Sign and scale canonicalization makes opposite and permuted
            // frame axes use the same plane representation.
            Vec3D n = direction / pivot;
            exactNormals[i] = new Rat3Hybrid(Quantize(n.X, DirectionScale),
                Quantize(n.Y, DirectionScale), Quantize(n.Z, DirectionScale));
            normals[i] = n;
            originOffsets[i] = Vec3DOps.Dot(n, frame.Origin - converter.OperatingSpace.Min) * scaling;
        }
        var a = Rat3Hybrid.Cross(exactNormals[1], exactNormals[2]);
        var b = Rat3Hybrid.Cross(exactNormals[2], exactNormals[0]);
        var c = Rat3Hybrid.Cross(exactNormals[0], exactNormals[1]);
        var determinant = Rat3Hybrid.Dot(exactNormals[0], a);
        inverseColumns = new[] { a / determinant, b / determinant, c / determinant };
    }

    public static void Apply(CoordinateConverter converter, CoordinateSystem frame,
        List<Vec3D> vertices, List<Vec3D> vertexNormals, List<Rat3Hybrid> precise)
    {
        var transform = new PreciseFrameTransform(converter, frame);
        precise.Clear();
        for (int i = 0; i < vertices.Count; i++)
        {
            precise.Add(transform.Transform(vertices[i]));
            vertices[i] = frame.PointFromCoordSysToWorld(vertices[i]);
        }
        for (int i = 0; i < vertexNormals.Count; i++)
            vertexNormals[i] = frame.DirectionFromCoordSysToWorld(vertexNormals[i]);
    }

    public Rat3Hybrid Transform(Vec3D local)
    {
        var coordinates = new[] { local.X, local.Y, local.Z };
        var result = new Rat3Hybrid(0, 0, 0);
        for (int i = 0; i < 3; i++)
        {
            double offset = originOffsets[i] +
                Vec3DOps.Dot(normals[i], directions[i]) * coordinates[i] * scaling;
            result += inverseColumns[i] * new BigRationalHybrid(checked((long)offset));
        }
        result.Simplify();
        return result;
    }
}
