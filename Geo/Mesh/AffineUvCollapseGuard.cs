using GeoCore;
using Remeshing;

namespace Geo;

// Removing a collinear geometric vertex must not remove a break in its
// parameterization. Surviving triangles retain their original affine UV map.
internal sealed class AffineUvCollapseGuard(
    List<Rat3Hybrid> positions,
    List<Tri> triangles,
    List<MeshTriangle<TriangleVertexNormalUV>> attributes,
    ISet<int> affineGroups = null)
{
    private Dictionary<(int Group, Rat3Hybrid Point), Vec2D?> values;

    private static Rat3Hybrid Key(Rat3Hybrid point)
    {
        var result = new Rat3Hybrid(in point);
        result.Simplify();
        return result;
    }

    private Vec2D? Find(int group, Rat3Hybrid point)
    {
        if (values == null)
        {
            values = new();
            for (int i = 0; i < triangles.Count; i++)
            {
                var t = triangles[i];
                var a = attributes[i];
                if (affineGroups?.Contains(a.GroupId) == true) continue;
                Add(a.GroupId, positions[t.A], a.V0.UV);
                Add(a.GroupId, positions[t.B], a.V1.UV);
                Add(a.GroupId, positions[t.C], a.V2.UV);
            }
        }
        return values.GetValueOrDefault((group, Key(point)));
    }

    private void Add(int group, Rat3Hybrid point, Vec2D uv)
    {
        var key = (group, Key(point));
        if (!values.TryGetValue(key, out var existing)) values.Add(key, uv);
        else if (existing != uv) values[key] = null;
    }

    internal bool Validate(Dictionary<int, FullTriangle> candidates)
    {
        foreach (var (index, candidate) in candidates)
        {
            if (affineGroups?.Contains(attributes[index].GroupId) == true) continue;
            var triangle = triangles[index];
            var a = positions[triangle.A];
            var b = positions[triangle.B];
            var c = positions[triangle.C];
            if (candidate.A == a && candidate.B == b && candidate.C == c) continue;
            var data = attributes[index];
            if (!Check(candidate.A) || !Check(candidate.B) || !Check(candidate.C)) return false;

            bool Check(Rat3Hybrid point)
            {
                if (point == a || point == b || point == c) return true;
                var actual = Find(data.GroupId, point);
                if (!actual.HasValue) return false;
                var weights = InterpolationHelpers.GetBarycentricWeights(point, triangle, positions);
                double wa = weights.X, wb = weights.Y, wc = weights.Z;
                return Coordinate(data.V0.UV.X, data.V1.UV.X, data.V2.UV.X, actual.Value.X) &&
                    Coordinate(data.V0.UV.Y, data.V1.UV.Y, data.V2.UV.Y, actual.Value.Y);

                bool Coordinate(double u, double v, double w, double target)
                {
                    double expected = wa * u + wb * v + wc * w;
                    double scale = Math.Max(1, Math.Max(Math.Abs(target),
                        Math.Abs(wa * u) + Math.Abs(wb * v) + Math.Abs(wc * w)));
                    return Math.Abs(expected - target) <= 128 * 2.2204460492503131e-16 * scale;
                }
            }
        }
        return true;
    }
}
