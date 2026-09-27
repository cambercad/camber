using CSG;
using Curves;
using GeoCore;
using GeoMeta;

namespace Geo;

public partial class GeoAPI
{
    [APIDescription(@"IntersectionCurves(surface: AnchorMesh, other: AnchorMesh, name: str = None) -> IReadOnlyList[Curve3D]
Returns one sampled 3D curve for each connected intersection of an open surface with another surface or a solid boundary. Intersections are exact for the input triangle meshes; curved inputs are represented by their existing tessellation. Coplanar overlap regions do not define a unique intersection curve and are omitted.")]
    public IReadOnlyList<Curve3D> IntersectionCurves(AnchorMesh surface, AnchorMesh other,
        string name = null)
    {
        ArgumentNullException.ThrowIfNull(surface);
        ArgumentNullException.ThrowIfNull(other);
        if (surface.IsVolume)
            throw new ArgumentException("The first input must be an open surface.", nameof(surface));

        var strips = new List<List<IntersectionSegmentEx>>();
        Resolver.Resolve(BooleanOp.NoOpIntersectionContourOnly,
            surface.Mesh.PrecisionPositions, surface.Mesh.Triangles,
            other.Mesh.PrecisionPositions, other.Mesh.Triangles,
            out _, out _, out _, intersectionStrips: strips);

        var curves = new List<Curve3D>(strips.Count);
        if (strips.Count == 0) return curves;

        string prefix = string.IsNullOrWhiteSpace(name) ? GenerateName("IntersectionCurve") : name;
        for (int i = 0; i < strips.Count; i++)
        {
            List<IntersectionSegmentEx> strip = strips[i];
            if (strip.Count == 0) continue;
            curves.Add(BuildIntersectionCurve(surface.Mesh, strip,
                strips.Count == 1 ? prefix : $"{prefix}_{i + 1}"));
        }

        lock (_meshRegistryLock) curves3D.AddRange(curves);
        return curves;
    }

    private Curve3D BuildIntersectionCurve(MeshNormalUV source,
        IReadOnlyList<IntersectionSegmentEx> segments, string name)
    {
        var exactPositions = new List<Rat3Hybrid>(segments.Count + 1)
        {
            segments[0].StartPoint
        };
        var positions = new List<Vec3D>(segments.Count + 1)
        {
            converter.Convert(exactPositions[0])
        };
        var normals = new List<Vec3D>(segments.Count);
        int previousEnd = segments[0].Start;
        Rat3Hybrid previousPoint = segments[0].StartPoint;
        for (int i = 0; i < segments.Count; i++)
        {
            IntersectionSegmentEx segment = segments[i];
            if (segment.Start != previousEnd || segment.StartPoint != previousPoint)
                throw new InvalidOperationException("Resolver returned a disconnected intersection curve strip.");
            exactPositions.Add(segment.EndPoint);
            positions.Add(converter.Convert(segment.EndPoint));
            normals.Add(TriangleNormal(source, segment.TriIdA));
            previousEnd = segment.End;
            previousPoint = segment.EndPoint;
        }

        bool closed = segments[0].Start == segments[^1].End &&
            segments[0].StartPoint == segments[^1].EndPoint;
        int uniqueCount = closed ? positions.Count - 1 : positions.Count;
        if (uniqueCount < 2) throw new InvalidOperationException("Intersection curve has fewer than two distinct points.");

        var vertices = new List<CurveVertex3D>(positions.Count);
        int initialVertexCount = closed ? uniqueCount : uniqueCount - 1;
        for (int i = 0; i < initialVertexCount; i++)
        {
            int beforeSegment = i > 0 ? i - 1 : closed ? segments.Count - 1 : 0;
            int afterSegment = i < segments.Count ? i : segments.Count - 1;
            Vec3D tangent = CurveTangent(positions, i, uniqueCount, closed);
            Vec3D normal = normals[beforeSegment] + normals[afterSegment];
            if (normal.LengthSquared() <= 1e-24) normal = normals[afterSegment];
            Vec3D up = normal - tangent * Vec3DOps.Dot(normal, tangent);
            if (up.LengthSquared() <= 1e-24) up = Perpendicular(tangent);
            else up = up.Normalized();
            vertices.Add(new CurveVertex3D(positions[i], tangent, up,
                i / (double)(positions.Count - 1)));
        }
        if (closed)
        {
            CurveVertex3D first = vertices[0];
            vertices.Add(new CurveVertex3D(positions[^1], first.Tangent, first.Up, 1));
        }
        else
        {
            Vec3D tangent = CurveTangent(positions, positions.Count - 1, uniqueCount, false);
            Vec3D up = normals[^1] - tangent * Vec3DOps.Dot(normals[^1], tangent);
            if (up.LengthSquared() <= 1e-24) up = Perpendicular(tangent);
            else up = up.Normalized();
            vertices.Add(new CurveVertex3D(positions[^1], tangent, up, 1));
        }
        return new PolylineCurve3D(vertices, exactPositions, converter, name);
    }

    private static Vec3D TriangleNormal(MeshNormalUV mesh, int triangleIndex)
    {
        Tri triangle = mesh.Triangles[triangleIndex];
        Vec3D a = mesh.Positions[triangle.A];
        Vec3D b = mesh.Positions[triangle.B];
        Vec3D c = mesh.Positions[triangle.C];
        Vec3D normal = Vec3DOps.Cross(b - a, c - a);
        if (normal.LengthSquared() <= 1e-24)
            throw new InvalidOperationException("Intersection curve references a degenerate source triangle.");
        return normal.Normalized();
    }

    private static Vec3D CurveTangent(IReadOnlyList<Vec3D> positions, int index,
        int uniqueCount, bool closed)
    {
        Vec3D tangent;
        if (closed)
        {
            int previous = (index + uniqueCount - 1) % uniqueCount;
            int next = (index + 1) % uniqueCount;
            tangent = positions[next] - positions[previous];
            if (tangent.LengthSquared() <= 1e-24)
                tangent = positions[next] - positions[index];
        }
        else if (index == 0)
        {
            tangent = positions[1] - positions[0];
        }
        else if (index == uniqueCount - 1)
        {
            tangent = positions[index] - positions[index - 1];
        }
        else
        {
            tangent = positions[index + 1] - positions[index - 1];
            if (tangent.LengthSquared() <= 1e-24)
                tangent = positions[index + 1] - positions[index];
        }
        if (!double.IsFinite(tangent.LengthSquared()) || tangent.LengthSquared() <= 1e-24)
            throw new InvalidOperationException("Intersection curve contains a degenerate tangent.");
        return tangent.Normalized();
    }

    private static Vec3D Perpendicular(Vec3D tangent)
    {
        Vec3D axis = Math.Abs(tangent.X) <= Math.Abs(tangent.Y) && Math.Abs(tangent.X) <= Math.Abs(tangent.Z)
            ? new Vec3D(1, 0, 0)
            : Math.Abs(tangent.Y) <= Math.Abs(tangent.Z)
                ? new Vec3D(0, 1, 0)
                : new Vec3D(0, 0, 1);
        return Vec3DOps.Cross(tangent, axis).Normalized();
    }
}
