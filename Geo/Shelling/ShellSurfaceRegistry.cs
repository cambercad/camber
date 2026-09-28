using GeoCore;

namespace Geo.Shelling;

public interface IShellSurfaceSupport
{
    SurfaceMetaData Metadata { get; }
}

/// <summary>The only surface-specific seam used by shell topology.
/// Offset distance is signed: positive inward, negative outward.</summary>
public interface IShellSurfaceAdapter
{
    bool Supports(SurfaceMetaData metadata, UVSurface surface, double inwardThickness);
    IShellSurfaceSupport CreateOffsetSupport(
        SurfaceMetaData metadata, UVSurface surface, Vec3D outwardAtSample,
        double inwardThickness, CoordinateConverter converter);
    bool TryResolveVertex(int sourceVertex, Vec3D sourcePoint,
        IReadOnlyList<IShellSurfaceSupport> supports, out Vec3D point);
}

/// <summary>Ordered adapters also own exact solvers for the support combinations they introduce.</summary>
public sealed class ShellSurfaceRegistry
{
    private readonly List<IShellSurfaceAdapter> _adapters = new();

    public ShellSurfaceRegistry Register(IShellSurfaceAdapter adapter)
    {
        _adapters.Add(adapter ?? throw new ArgumentNullException(nameof(adapter)));
        return this;
    }

    public IShellSurfaceAdapter Resolve(SurfaceMetaData metadata, UVSurface surface, double thickness) =>
        _adapters.FirstOrDefault(a => a.Supports(metadata, surface, thickness));

    public bool TryResolveVertex(int sourceVertex, Vec3D sourcePoint,
        IReadOnlyList<IShellSurfaceSupport> supports, double tolerance, out Vec3D point)
    {
        foreach (var adapter in _adapters)
            if (adapter.TryResolveVertex(sourceVertex, sourcePoint, supports, out point)) return true;
        // A Boolean seam can put more than three analytic patches at one vertex.
        // Solve a determined subset, but accept it only if every other exact
        // support passes through the same point. Inconsistent miters still fail.
        if (supports.Count > 3 && supports.All(s => s is PlanarShellSupport or
                CylindricalShellSupport or SphericalShellSupport))
        {
            for (int size = 2; size <= 3; size++)
                for (int i = 0; i < supports.Count; i++)
                    for (int j = i + 1; j < supports.Count; j++)
                        for (int k = size == 2 ? supports.Count : j + 1; k <= supports.Count; k++)
                        {
                            if (size == 3 && k == supports.Count) continue;
                            IShellSurfaceSupport[] subset = size == 2
                                ? new[] { supports[i], supports[j] }
                                : new[] { supports[i], supports[j], supports[k] };
                            foreach (var adapter in _adapters)
                                if (adapter.TryResolveVertex(sourceVertex, sourcePoint, subset, out var candidate) &&
                                    supports.All(s => OnSupport(s, candidate, tolerance)))
                                { point = candidate; return true; }
                        }
        }
        point = default;
        return false;
    }

    private static bool OnSupport(IShellSurfaceSupport support, Vec3D point, double tolerance)
    {
        if (support is PlanarShellSupport plane)
            return Math.Abs(Vec3DOps.Dot(point - plane.Plane.Origin, plane.Plane.Normal.Normalized())) <= tolerance;
        if (support is CylindricalShellSupport cylinder)
        {
            var axis = cylinder.Cylinder.Axis.Normalized();
            var v = point - cylinder.Cylinder.Origin;
            return Math.Abs((v - axis * Vec3DOps.Dot(v, axis)).Length() - cylinder.Cylinder.Radius) <= tolerance;
        }
        if (support is SphericalShellSupport sphere)
            return Math.Abs((point - sphere.Sphere.Center).Length() - sphere.Sphere.Radius) <= tolerance;
        return false;
    }

    public static ShellSurfaceRegistry AnalyticV1 { get; } = new ShellSurfaceRegistry()
        .Register(new SphericalShellSurfaceAdapter())
        .Register(new CylindricalShellSurfaceAdapter())
        .Register(new PlanarShellSurfaceAdapter())
        .Register(new MeshShellSurfaceAdapter());
}

public sealed class PlanarShellSupport : IShellSurfaceSupport
{
    public PlaneSurfaceParams Plane { get; }
    public SurfaceMetaData Metadata { get; }

    internal PlanarShellSupport(PlaneSurfaceParams plane)
    {
        Plane = plane;
        Metadata = new SurfaceMetaData(SurfaceType.Planar) { PlaneParams = plane.Clone() };
    }
}

/// <summary>Exact analytic translation and three-plane intersection.</summary>
public sealed class PlanarShellSurfaceAdapter : IShellSurfaceAdapter
{
    public bool Supports(SurfaceMetaData metadata, UVSurface surface, double inwardThickness) =>
        metadata?.SurfaceType == SurfaceType.Planar && metadata.PlaneParams != null &&
        surface != null && surface.IsSurfacePlanar() && double.IsFinite(inwardThickness);

    public IShellSurfaceSupport CreateOffsetSupport(SurfaceMetaData metadata, UVSurface surface,
        Vec3D outwardAtSample, double inwardThickness, CoordinateConverter converter)
    {
        var source = metadata.PlaneParams;
        var normal = source.Normal.Normalized();
        if (Vec3DOps.Dot(normal, outwardAtSample) < 0) normal = -normal;
        return new PlanarShellSupport(new PlaneSurfaceParams {
            Origin = source.Origin - normal * inwardThickness,
            Normal = normal,
            RefDir = source.RefDir
        });
    }

    public bool TryResolveVertex(int sourceVertex, Vec3D sourcePoint,
        IReadOnlyList<IShellSurfaceSupport> supports, out Vec3D point)
    {
        point = default;
        if (supports.Count != 3 || supports.Any(s => s is not PlanarShellSupport)) return false;
        var a = ((PlanarShellSupport)supports[0]).Plane;
        var b = ((PlanarShellSupport)supports[1]).Plane;
        var c = ((PlanarShellSupport)supports[2]).Plane;
        var n1 = a.Normal.Normalized(); var n2 = b.Normal.Normalized(); var n3 = c.Normal.Normalized();
        var n2n3 = Vec3DOps.Cross(n2, n3);
        double det = Vec3DOps.Dot(n1, n2n3);
        if (Math.Abs(det) < 1e-10) return false;
        double d1 = Vec3DOps.Dot(n1, a.Origin), d2 = Vec3DOps.Dot(n2, b.Origin), d3 = Vec3DOps.Dot(n3, c.Origin);
        point = (n2n3 * d1 + Vec3DOps.Cross(n3, n1) * d2 + Vec3DOps.Cross(n1, n2) * d3) / det;
        return IsFinite(point);
    }

    internal static bool IsFinite(Vec3D p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z);
}

public sealed class CylindricalShellSupport : IShellSurfaceSupport
{
    public CylinderSurfaceParams Cylinder { get; }
    public SurfaceMetaData Metadata { get; }

    internal CylindricalShellSupport(CylinderSurfaceParams cylinder)
    {
        Cylinder = cylinder;
        Metadata = new SurfaceMetaData(SurfaceType.Cylindrical) { CylinderParams = cylinder.Clone() };
    }

    internal void UpdateTrim(IEnumerable<Vec3D> points)
    {
        var axis = Cylinder.Axis.Normalized();
        var stations = points.Select(point => Vec3DOps.Dot(point - Cylinder.Origin, axis)).ToArray();
        if (stations.Length == 0) return;
        double min = stations.Min(), max = stations.Max();
        Cylinder.Origin += axis * min;
        Cylinder.Height = max - min;
        Metadata.CylinderParams = Cylinder.Clone();
    }
}

/// <summary>Exact constant-radius offset for analytic cylindrical patches.</summary>
public sealed class CylindricalShellSurfaceAdapter : IShellSurfaceAdapter
{
    public bool Supports(SurfaceMetaData metadata, UVSurface surface, double inwardThickness) =>
        metadata?.SurfaceType == SurfaceType.Cylindrical && metadata.CylinderParams != null &&
        surface != null && double.IsFinite(inwardThickness);

    public IShellSurfaceSupport CreateOffsetSupport(SurfaceMetaData metadata, UVSurface surface,
        Vec3D outwardAtSample, double inwardThickness, CoordinateConverter converter)
    {
        var cylinder = metadata.CylinderParams.Clone();
        var axis = cylinder.Axis.Normalized();
        Vec3D sample = surface.Points[surface.Triangles[0].A];
        Vec3D radial = sample - cylinder.Origin - axis * Vec3DOps.Dot(sample - cylinder.Origin, axis);
        if (radial.LengthSquared() == 0)
            throw new ArgumentException("Cylindrical shell support contains a point on its axis.");
        double orientation = Vec3DOps.Dot(radial.Normalized(), outwardAtSample) >= 0 ? 1 : -1;
        cylinder.Radius -= orientation * inwardThickness;
        if (!(cylinder.Radius > converter.SmallestUnit()))
            throw new ArgumentException("Shell thickness collapses a cylindrical support.");
        return new CylindricalShellSupport(cylinder);
    }

    public bool TryResolveVertex(int sourceVertex, Vec3D sourcePoint,
        IReadOnlyList<IShellSurfaceSupport> supports, out Vec3D point)
    {
        point = default;
        var cylinders = supports.OfType<CylindricalShellSupport>().ToList();
        if (cylinders.Count != 1 || supports.Count != cylinders.Count + supports.OfType<PlanarShellSupport>().Count())
            return false;
        var cylinder = cylinders[0].Cylinder;
        var axis = cylinder.Axis.Normalized();
        var planes = supports.OfType<PlanarShellSupport>().Select(s => s.Plane).ToArray();
        if (planes.Length == 2)
        {
            // Two planes meet in a line. Intersect that line with the cylinder
            // and choose the branch local to the source vertex.
            var n0 = planes[0].Normal.Normalized();
            var n1 = planes[1].Normal.Normalized();
            var direction = Vec3DOps.Cross(n0, n1);
            double lengthSquared = direction.LengthSquared();
            if (lengthSquared < 1e-20) return false;
            double d0 = Vec3DOps.Dot(n0, planes[0].Origin - sourcePoint);
            double d1 = Vec3DOps.Dot(n1, planes[1].Origin - sourcePoint);
            var basePoint = sourcePoint + (Vec3DOps.Cross(n1, direction) * d0 +
                Vec3DOps.Cross(direction, n0) * d1) / lengthSquared;
            var fromAxis = basePoint - cylinder.Origin;
            var baseRadial = fromAxis - axis * Vec3DOps.Dot(fromAxis, axis);
            var radialDirection = direction - axis * Vec3DOps.Dot(direction, axis);
            double a = radialDirection.LengthSquared();
            double c = baseRadial.LengthSquared() - cylinder.Radius * cylinder.Radius;
            if (a < 1e-20)
            {
                // If the plane-intersection line is a cylinder generator, the
                // supports share that line rather than an isolated point. The
                // closest point on it to the source vertex is the stable miter.
                if (Math.Abs(c) > 1e-10 * Math.Max(1, cylinder.Radius * cylinder.Radius)) return false;
                point = basePoint;
                return PlanarShellSurfaceAdapter.IsFinite(point);
            }
            double b = 2 * Vec3DOps.Dot(baseRadial, radialDirection);
            double discriminant = b * b - 4 * a * c;
            if (discriminant < 0) return false;
            double root = Math.Sqrt(discriminant);
            var first = basePoint + direction * ((-b + root) / (2 * a));
            var second = basePoint + direction * ((-b - root) / (2 * a));
            point = (first - sourcePoint).LengthSquared() <= (second - sourcePoint).LengthSquared() ? first : second;
            return PlanarShellSurfaceAdapter.IsFinite(point);
        }
        Vec3D fromOrigin = sourcePoint - cylinder.Origin;
        Vec3D radial = fromOrigin - axis * Vec3DOps.Dot(fromOrigin, axis);
        if (radial.LengthSquared() == 0) return false;
        point = cylinder.Origin + axis * Vec3DOps.Dot(fromOrigin, axis) + radial.Normalized() * cylinder.Radius;
        foreach (var support in supports.OfType<PlanarShellSupport>())
        {
            var plane = support.Plane;
            var normal = plane.Normal.Normalized();
            double denominator = Vec3DOps.Dot(normal, axis);
            if (Math.Abs(denominator) < 1e-10) return false;
            point += axis * ((Vec3DOps.Dot(normal, plane.Origin) - Vec3DOps.Dot(normal, point)) / denominator);
        }
        return PlanarShellSurfaceAdapter.IsFinite(point);
    }
}

public sealed class SphericalShellSupport : IShellSurfaceSupport
{
    public SphereSurfaceParams Sphere { get; }
    public SurfaceMetaData Metadata { get; }

    internal SphericalShellSupport(SphereSurfaceParams sphere)
    {
        Sphere = sphere;
        Metadata = new SurfaceMetaData(SurfaceType.Spherical) { SphereParams = sphere.Clone() };
    }
}

/// <summary>Exact constant-radius offset for analytic spherical patches.</summary>
public sealed class SphericalShellSurfaceAdapter : IShellSurfaceAdapter
{
    public bool Supports(SurfaceMetaData metadata, UVSurface surface, double inwardThickness) =>
        metadata?.SurfaceType == SurfaceType.Spherical && metadata.SphereParams != null &&
        surface != null && double.IsFinite(inwardThickness);

    public IShellSurfaceSupport CreateOffsetSupport(SurfaceMetaData metadata, UVSurface surface,
        Vec3D outwardAtSample, double inwardThickness, CoordinateConverter converter)
    {
        var sphere = metadata.SphereParams.Clone();
        Vec3D sample = surface.Points[surface.Triangles[0].A];
        Vec3D radial = sample - sphere.Center;
        if (radial.LengthSquared() == 0)
            throw new ArgumentException("Spherical shell support contains a point at its center.");
        double orientation = Vec3DOps.Dot(radial.Normalized(), outwardAtSample) >= 0 ? 1 : -1;
        sphere.Radius -= orientation * inwardThickness;
        if (!(sphere.Radius > converter.SmallestUnit()))
            throw new ArgumentException("Shell thickness collapses a spherical support.");
        return new SphericalShellSupport(sphere);
    }

    public bool TryResolveVertex(int sourceVertex, Vec3D sourcePoint,
        IReadOnlyList<IShellSurfaceSupport> supports, out Vec3D point)
    {
        point = default;
        var spheres = supports.OfType<SphericalShellSupport>().ToList();
        var planes = supports.OfType<PlanarShellSupport>().ToList();
        if (spheres.Count != 1 || supports.Count != spheres.Count + planes.Count || planes.Count > 1)
            return false;
        var sphere = spheres[0].Sphere;
        if (planes.Count == 0)
        {
            Vec3D radial = sourcePoint - sphere.Center;
            if (radial.LengthSquared() == 0) return false;
            point = sphere.Center + radial.Normalized() * sphere.Radius;
            return PlanarShellSurfaceAdapter.IsFinite(point);
        }

        var plane = planes[0].Plane;
        Vec3D normal = plane.Normal.Normalized();
        double signedCenterDistance = Vec3DOps.Dot(normal, plane.Origin - sphere.Center);
        double circleRadiusSquared = sphere.Radius * sphere.Radius - signedCenterDistance * signedCenterDistance;
        if (circleRadiusSquared <= 0) return false;
        Vec3D circleCenter = sphere.Center + normal * signedCenterDistance;
        Vec3D tangent = sourcePoint - sphere.Center - normal * Vec3DOps.Dot(sourcePoint - sphere.Center, normal);
        if (tangent.LengthSquared() == 0) return false;
        point = circleCenter + tangent.Normalized() * Math.Sqrt(circleRadiusSquared);
        return PlanarShellSurfaceAdapter.IsFinite(point);
    }
}

public sealed class MeshShellSupport : IShellSurfaceSupport
{
    private readonly Dictionary<int, (Vec3D Point, Vec3D Normal)> _vertices;
    private readonly Dictionary<Vec3D, (Vec3D Point, Vec3D Normal)> _positions;
    public SurfaceMetaData Metadata { get; } = new(SurfaceType.Unknown);

    internal MeshShellSupport(UVSurface source, UVSurface offset, double normalSense)
    {
        _vertices = new Dictionary<int, (Vec3D, Vec3D)>();
        _positions = new Dictionary<Vec3D, (Vec3D, Vec3D)>();
        foreach (int index in source.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
        {
            Vec3D normal = source.Normals[index] * normalSense;
            if (normal.LengthSquared() == 0)
                throw new ArgumentException($"Mesh shell support has no normal at vertex {index}.");
            var value = (offset.Points[index], normal.Normalized());
            _vertices[index] = value;
            _positions[source.Points[index]] = value;
        }
    }

    internal bool TryGet(int sourceVertex, Vec3D sourcePoint, out Vec3D point, out Vec3D normal)
    {
        if (_vertices.TryGetValue(sourceVertex, out var value) || _positions.TryGetValue(sourcePoint, out value))
        {
            point = value.Point; normal = value.Normal; return true;
        }
        point = default; normal = default; return false;
    }
}

/// <summary>
/// Fallback for any consistently oriented triangle patch, including patches with
/// no analytic or NURBS metadata. Boundaries are mitered by intersecting local
/// tangent supports; invalid or topology-changing offsets are rejected downstream.
/// </summary>
public sealed class MeshShellSurfaceAdapter : IShellSurfaceAdapter
{
    public bool Supports(SurfaceMetaData metadata, UVSurface surface, double inwardThickness) =>
        surface?.Triangles?.Count > 0 && surface.Normals != null && double.IsFinite(inwardThickness);

    public IShellSurfaceSupport CreateOffsetSupport(SurfaceMetaData metadata, UVSurface surface,
        Vec3D outwardAtSample, double inwardThickness, CoordinateConverter converter)
    {
        int sampleIndex = surface.Triangles[0].A;
        Vec3D sampleNormal = surface.Normals[sampleIndex];
        if (sampleNormal.LengthSquared() == 0)
            throw new ArgumentException("Mesh shell support has a zero sample normal.");
        double sense = Vec3DOps.Dot(sampleNormal, outwardAtSample) >= 0 ? 1 : -1;
        var offset = surface.GetOffsetSurface(-sense * inwardThickness, converter);
        return new MeshShellSupport(surface, offset, sense);
    }

    public bool TryResolveVertex(int sourceVertex, Vec3D sourcePoint,
        IReadOnlyList<IShellSurfaceSupport> supports, out Vec3D point)
    {
        point = default;
        if (supports.Count == 0) return false;
        var planes = new List<(Vec3D Point, Vec3D Normal)>();
        foreach (var support in supports)
            if (!TryTangent(support, sourceVertex, sourcePoint, out var tangent)) return false;
            else planes.Add(tangent);

        if (planes.Count == 1)
        {
            var plane = planes[0];
            double displacement = Vec3DOps.Dot(plane.Normal, plane.Point - sourcePoint);
            point = sourcePoint + plane.Normal * displacement;
            return PlanarShellSurfaceAdapter.IsFinite(point);
        }
        if (planes.Count == 2)
        {
            var a = planes[0]; var b = planes[1];
            double c = Vec3DOps.Dot(a.Normal, b.Normal);
            double determinant = 1 - c * c;
            if (determinant < 1e-12) return false;
            double da = Vec3DOps.Dot(a.Normal, a.Point - sourcePoint);
            double db = Vec3DOps.Dot(b.Normal, b.Point - sourcePoint);
            double alpha = (da - c * db) / determinant;
            double beta = (db - c * da) / determinant;
            point = sourcePoint + a.Normal * alpha + b.Normal * beta;
            return PlanarShellSurfaceAdapter.IsFinite(point);
        }

        if (planes.Count == 3)
            return TryIntersect(planes[0], planes[1], planes[2], out point);

        // A vertex can have more than three incident patches (for example, an
        // icosahedron vertex). Solve the best-conditioned triple, then require
        // every remaining tangent support to contain that same point. Do not
        // pick an arbitrary triple and silently create a corner gap.
        double bestDeterminant = 0;
        Vec3D candidate = default;
        for (int i = 0; i < planes.Count - 2; i++)
            for (int j = i + 1; j < planes.Count - 1; j++)
                for (int k = j + 1; k < planes.Count; k++)
                {
                    var cross = Vec3DOps.Cross(planes[j].Normal, planes[k].Normal);
                    double determinant = Vec3DOps.Dot(planes[i].Normal, cross);
                    if (Math.Abs(determinant) <= Math.Abs(bestDeterminant) ||
                        !TryIntersect(planes[i], planes[j], planes[k], out var intersection)) continue;
                    bestDeterminant = determinant;
                    candidate = intersection;
                }
        if (bestDeterminant == 0) return false;

        double scale = Math.Max(1, sourcePoint.Length());
        double tolerance = 1e-8 * scale;
        foreach (var plane in planes)
            if (Math.Abs(Vec3DOps.Dot(plane.Normal, candidate - plane.Point)) > tolerance)
                return false;
        point = candidate;
        return true;
    }

    private static bool TryIntersect((Vec3D Point, Vec3D Normal) a,
        (Vec3D Point, Vec3D Normal) b, (Vec3D Point, Vec3D Normal) c,
        out Vec3D point)
    {
        var bCrossC = Vec3DOps.Cross(b.Normal, c.Normal);
        double determinant = Vec3DOps.Dot(a.Normal, bCrossC);
        if (Math.Abs(determinant) < 1e-10)
        {
            point = default;
            return false;
        }
        double da = Vec3DOps.Dot(a.Normal, a.Point);
        double db = Vec3DOps.Dot(b.Normal, b.Point);
        double dc = Vec3DOps.Dot(c.Normal, c.Point);
        point = (bCrossC * da + Vec3DOps.Cross(c.Normal, a.Normal) * db +
            Vec3DOps.Cross(a.Normal, b.Normal) * dc) / determinant;
        return PlanarShellSurfaceAdapter.IsFinite(point);
    }

    private static bool TryTangent(IShellSurfaceSupport support, int sourceVertex, Vec3D sourcePoint,
        out (Vec3D Point, Vec3D Normal) tangent)
    {
        if (support is MeshShellSupport mesh && mesh.TryGet(sourceVertex, sourcePoint, out var point, out var normal))
        { tangent = (point, normal); return true; }
        if (support is PlanarShellSupport plane)
        { tangent = (plane.Plane.Origin, plane.Plane.Normal.Normalized()); return true; }
        if (support is CylindricalShellSupport cylinder)
        {
            var c = cylinder.Cylinder; var axis = c.Axis.Normalized();
            Vec3D fromOrigin = sourcePoint - c.Origin;
            Vec3D radial = fromOrigin - axis * Vec3DOps.Dot(fromOrigin, axis);
            if (radial.LengthSquared() == 0) { tangent = default; return false; }
            normal = radial.Normalized();
            tangent = (c.Origin + axis * Vec3DOps.Dot(fromOrigin, axis) + normal * c.Radius, normal);
            return true;
        }
        if (support is SphericalShellSupport sphere)
        {
            Vec3D radial = sourcePoint - sphere.Sphere.Center;
            if (radial.LengthSquared() == 0) { tangent = default; return false; }
            normal = radial.Normalized();
            tangent = (sphere.Sphere.Center + normal * sphere.Sphere.Radius, normal);
            return true;
        }
        tangent = default; return false;
    }
}
