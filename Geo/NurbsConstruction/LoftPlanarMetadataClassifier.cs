using Curves;
using GeoCore;
using GeoMeta;

namespace Geo;

/// <summary>
/// Conservatively marks ruled loft side patches as planar when the exact mesh
/// lattice proves coplanarity. Smooth lofts and curved profiles are excluded:
/// finitely many surface samples cannot prove their continuous NURBS support.
/// </summary>
internal static class LoftPlanarMetadataClassifier
{
    internal static bool HasOnlyLineProfiles(IReadOnlyList<PlotterSketcherCoordSys> sketches)
    {
        if (sketches == null || sketches.Count < 2)
            return false;

        foreach (var sketch in sketches)
        {
            if (sketch == null)
                return false;
            var curves = sketch.GetCurves();
            if (curves.Count == 0)
                return false;
            foreach (var curve in curves.SelectMany(strip => strip))
                if (!curve.IsHelperGeometry && curve is not Line2D)
                    return false;
        }
        return true;
    }

    internal static void ClassifyRuledLineSides(
        IReadOnlyList<Rat3Hybrid> precisePositions,
        IReadOnlyList<Tri> triangles,
        IReadOnlyList<int> triangleGroups,
        IReadOnlyDictionary<int, string> groupNames,
        IDictionary<string, SurfaceMetaData> metadata,
        CoordinateConverter converter,
        string operationName)
    {
        string sidePrefix = EntityNaming.LoftSide(operationName);
        var sideIds = groupNames
            .Where(pair => pair.Value == sidePrefix || pair.Value.StartsWith(sidePrefix + "-", StringComparison.Ordinal))
            .Select(pair => pair.Key)
            .ToArray();

        foreach (int groupId in sideIds)
        {
            string patchName = groupNames[groupId];
            if (!metadata.TryGetValue(patchName, out var patch) || patch.SurfaceType != SurfaceType.Unknown)
                continue;

            var vertexIndices = new HashSet<int>();
            for (int i = 0; i < triangles.Count; i++)
            {
                if (triangleGroups[i] != groupId)
                    continue;
                var triangle = triangles[i];
                vertexIndices.Add(triangle.A);
                vertexIndices.Add(triangle.B);
                vertexIndices.Add(triangle.C);
            }

            if (!TryGetExactPlane(precisePositions, vertexIndices, out var origin, out var exactNormal, out var exactDirection))
                continue;

            Vec3D normal = new(exactNormal.X.ToDouble(), exactNormal.Y.ToDouble(), exactNormal.Z.ToDouble());
            Vec3D refDir = new(exactDirection.X.ToDouble(), exactDirection.Y.ToDouble(), exactDirection.Z.ToDouble());
            if (!IsFinite(normal) || !IsFinite(refDir) || normal.LengthSquared() == 0 || refDir.LengthSquared() == 0)
                continue;

            normal.Normalize();
            refDir.Normalize();
            patch.PlaneParams = new PlaneSurfaceParams
            {
                Origin = converter.Convert(origin),
                Normal = normal,
                RefDir = refDir
            };
            patch.SurfaceType = SurfaceType.Planar;
        }
    }

    private static bool TryGetExactPlane(
        IReadOnlyList<Rat3Hybrid> positions,
        HashSet<int> vertexIndices,
        out Rat3Hybrid origin,
        out Rat3Hybrid normal,
        out Rat3Hybrid refDirection)
    {
        origin = default;
        normal = default;
        refDirection = default;
        if (vertexIndices.Count < 3)
            return false;

        using var indices = vertexIndices.GetEnumerator();
        indices.MoveNext();
        origin = positions[indices.Current];

        Rat3Hybrid firstDirection = default;
        bool hasDirection = false;
        foreach (int index in vertexIndices)
        {
            var direction = positions[index] - origin;
            if (direction.IsZero())
                continue;
            if (!hasDirection)
            {
                firstDirection = direction;
                hasDirection = true;
                continue;
            }

            var candidateNormal = Rat3Hybrid.Cross(firstDirection, direction);
            if (candidateNormal.IsZero())
                continue;

            normal = candidateNormal;
            refDirection = firstDirection;
            break;
        }

        if (normal.IsZero())
            return false;

        foreach (int index in vertexIndices)
            if (Rat3Hybrid.Dot(normal, positions[index] - origin).Sign() != 0)
                return false;

        return true;
    }

    private static bool IsFinite(Vec3D vector) =>
        double.IsFinite(vector.X) && double.IsFinite(vector.Y) && double.IsFinite(vector.Z);
}
