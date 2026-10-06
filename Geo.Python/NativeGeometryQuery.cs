using System.Globalization;
using DotWrap;
using GeoCore;

namespace GeoPy;

/// <summary>One immutable viewer query snapshot. Binary packing avoids per-triangle interop calls.</summary>
[DotWrapExpose]
public class NativeGeometryQuery
{
    readonly GeometryQuery query;
    public NativeGeometryQuery(string packed)
    {
        using var stream = new MemoryStream(Convert.FromBase64String(packed));
        using var reader = new BinaryReader(stream);
        int count = reader.ReadInt32();
        if (count < 0 || count > (stream.Length - 4) / 25) throw new ArgumentException("Invalid geometry payload.");
        var primitives = new GeometryQuery.Primitive[count];
        for (int i = 0; i < count; i++)
        {
            int n = reader.ReadByte(); if (n < 1 || n > 3) throw new ArgumentException("Invalid primitive.");
            Vec3D Read() => new(reader.ReadDouble(), reader.ReadDouble(), reader.ReadDouble());
            var a = Read(); var b = n > 1 ? Read() : a; var c = n > 2 ? Read() : a;
            primitives[i] = new(a, b, c, n);
        }
        if (stream.Position != stream.Length) throw new ArgumentException("Trailing geometry data.");
        query = new(primitives);
    }
    static string Numbers(params double[] values) => string.Join(" ", Array.ConvertAll(values, v => v.ToString("G17", CultureInfo.InvariantCulture)));
    public string Closest(NativeGeometryQuery other)
    { var p = query.Closest(other.query); return Numbers(p.First.X, p.First.Y, p.First.Z, p.Second.X, p.Second.Y, p.Second.Z, Math.Sqrt(p.SquaredDistance)); }
    public string Ray(string packedRay, int count, double radius, string packedPlane)
    {
        var values = Array.ConvertAll(packedRay.Split(' ', StringSplitOptions.RemoveEmptyEntries), v => double.Parse(v, CultureInfo.InvariantCulture));
        if (values.Length != 6) throw new ArgumentException("Expected ray origin and direction.");
        Vec3D? normal = null; double offset = 0;
        if (!string.IsNullOrEmpty(packedPlane))
        {
            var plane = Array.ConvertAll(packedPlane.Split(' ', StringSplitOptions.RemoveEmptyEntries), v => double.Parse(v, CultureInfo.InvariantCulture));
            if (plane.Length != 4) throw new ArgumentException("Expected section plane.");
            normal = new(plane[0], plane[1], plane[2]); offset = plane[3];
        }
        var hit = query.Ray(new(values[0], values[1], values[2]), new(values[3], values[4], values[5]), count, radius, normal, offset);
        return Numbers(hit.Index, hit.Depth, hit.Distance);
    }
}
