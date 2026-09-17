using GeoCore;

namespace Geo;

public partial class GeoAPI
{
    /// <summary>Independent solid copies at equal world-space steps, including the unchanged seed.</summary>
    public IReadOnlyList<AnchorMesh> PatternLinear(AnchorMesh seed, int count, Vec3D step, string name = null)
        => PatternSolids(seed, PatternTransforms.Linear(count, step), name);

    /// <summary>Independent copies about axis.Z; full circles omit the duplicate endpoint.</summary>
    public IReadOnlyList<AnchorMesh> PatternCircular(AnchorMesh seed, int count,
        CoordinateSystem axis, double angle = 2 * Math.PI, string name = null)
        => PatternSolids(seed, PatternTransforms.Circular(count, axis, angle), name, axis.Origin);

    private void ValidatePatternSeed(AnchorMesh seed)
    {
        if (seed == null) throw new ArgumentNullException(nameof(seed));
        if (!ReferenceEquals(GetMeshFromName(seed.Name), seed))
            throw new ArgumentException("Solid must belong to this part.", nameof(seed));
        seed.EnsureCoplanarPostProcessed();
    }

    private IReadOnlyList<AnchorMesh> PatternSolids(AnchorMesh seed,
        IReadOnlyList<Transform> placements, string name, Vec3D? pivot = null)
    {
        ValidatePatternSeed(seed);
        name = string.IsNullOrEmpty(name) ? GenerateName(seed.Name + "_pattern") : name;
        for (int i = 1; i < placements.Count; i++)
            if (GetMeshFromName(name + "_" + i) != null)
                throw new ArgumentException("Pattern output name already exists: " + name + "_" + i, nameof(name));
        var result = new List<AnchorMesh> { seed };
        for (int i = 1; i < placements.Count; i++)
        {
            var placement = placements[i];
            var exact = pivot.HasValue
                ? new PreciseRigidTransform(converter, placement.Orientation, pivot.Value)
                : new PreciseRigidTransform(converter, in placement);
            result.Add(CopyExactTransform(seed, TransformMath.ToMat4D(in placement), exact,
                name + "_" + i, reverseWinding: false));
        }
        return result;
    }

    /// <summary>Reflect a solid in plane.Z, preserving outward winding and analytic metadata.</summary>
    public AnchorMesh Mirror(AnchorMesh source, CoordinateSystem plane, string name = null)
    {
        ValidatePatternSeed(source);
        return MirrorGeometry(source, plane, name);
    }

    // Assembly geometry is defined in the captured body frame, even after a
    // display or solve has materialized a different occurrence pose.
    internal AnchorMesh MirrorRigidRestBody(AnchorMesh source, string name = null)
    {
        ValidatePatternSeed(source);
        var rest = source.SnapshotRigidDefinition(source.Name);
        return MirrorGeometry(rest, CoordinateSystem.Default, name);
    }

    private AnchorMesh MirrorGeometry(AnchorMesh source, CoordinateSystem plane, string name)
    {
        // Reuse the frame validation of circular placement without constructing a rotation.
        _ = PatternTransforms.Circular(1, plane, 0);
        name = string.IsNullOrEmpty(name) ? GenerateName(source.Name + "_mirror") : name;
        if (GetMeshFromName(name) != null)
            throw new ArgumentException("Mirror output name already exists: " + name, nameof(name));
        var n = plane.Z.Normalized();
        var d = 2 * Vec3DOps.Dot(n, plane.Origin);
        var matrix = new Mat4D(
            1-2*n.X*n.X, -2*n.X*n.Y, -2*n.X*n.Z, d*n.X,
            -2*n.Y*n.X, 1-2*n.Y*n.Y, -2*n.Y*n.Z, d*n.Y,
            -2*n.Z*n.X, -2*n.Z*n.Y, 1-2*n.Z*n.Z, d*n.Z,
            0, 0, 0, 1);
        return CopyExactTransform(source, matrix, new PreciseRigidTransform(converter, plane), name, reverseWinding: true);
    }

    private AnchorMesh CopyExactTransform(AnchorMesh source, Mat4D matrix,
        PreciseRigidTransform exact, string name, bool reverseWinding)
    {
        var mesh = source.Mesh;
        var positions = new Vec3D[mesh.PrecisionPositions.Count];
        var precise = new Rat3Hybrid[positions.Length];
        for (int i = 0; i < positions.Length; i++)
        {
            precise[i] = exact.Apply(mesh.PrecisionPositions[i]);
            // Keep the source's continuous display coordinates, as CopyMesh does.
            // PrecisionPositions remain the authoritative exact topology.
            positions[i] = matrix.TransformPoint(mesh.Positions[i]);
        }
        return CopyTransformedMesh(source, matrix, positions, precise,
            name + "_", SurfacePatchNameAffix.Prefix, name, reverseWinding, rewriteNames: true);
    }
}
