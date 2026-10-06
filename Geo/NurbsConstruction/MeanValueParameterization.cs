using GeoCore;

namespace Geo.NurbsConstruction;

/// <summary>
/// Floater mean-value coordinates on an oriented manifold disk. The boundary is
/// mapped by arc length to a convex circle; positive interior weights give a
/// one-to-one embedding in exact arithmetic. The floating solve is accepted only
/// after convergence and exact orientation checks on its represented coordinates.
/// This constructs a chart, not a reconstruction of unknown surface geometry.
/// </summary>
internal static class MeanValueParameterization
{
    public static bool TryCreate(UVSurface patch, out Dictionary<int, Vec2D> uv)
    {
        uv = null;
        if (patch?.Triangles == null || patch.Points == null || patch.Triangles.Count == 0)
            return false;

        var rows = new Dictionary<int, Dictionary<int, double>>();
        var edges = new Dictionary<(int, int), (int From, int To, int First, int Second)>();
        var incident = new Dictionary<int, List<int>>();
        var adjacent = new List<int>[patch.Triangles.Count];
        for (int i = 0; i < adjacent.Length; i++) adjacent[i] = new List<int>(3);
        for (int i = 0; i < patch.Triangles.Count; i++)
        {
            var t = patch.Triangles[i];
            if (!AddCorner(t.A, t.B, t.C, i) || !AddCorner(t.B, t.C, t.A, i) ||
                !AddCorner(t.C, t.A, t.B, i)) return false;
            if (!AddEdge(t.A, t.B, i) || !AddEdge(t.B, t.C, i) ||
                !AddEdge(t.C, t.A, i)) return false;
        }

        var next = new Dictionary<int, int>();
        var incoming = new HashSet<int>();
        foreach (var edge in edges.Values)
            if (edge.Second < 0 && (!next.TryAdd(edge.From, edge.To) || !incoming.Add(edge.To)))
                return false;
        if (next.Count < 3 || rows.Count - edges.Count + patch.Triangles.Count != 1)
            return false;
        var boundary = new List<int>(next.Count);
        var boundaryIterator = next.Keys.GetEnumerator(); boundaryIterator.MoveNext();
        int start = boundaryIterator.Current, current = start;
        do
        {
            boundary.Add(current);
            if (!next.TryGetValue(current, out current) || boundary.Count > next.Count) return false;
        } while (current != start);
        if (boundary.Count != next.Count) return false; // Holes/multiple components need seams first.

        // Edge valence alone does not exclude a pinched vertex. Every vertex must
        // have one connected triangle fan, and the patch one connected component.
        if (Reach(0, adjacent, null) != patch.Triangles.Count) return false;
        foreach (var fan in incident.Values)
            if (Reach(fan[0], adjacent, new HashSet<int>(fan)) != fan.Count) return false;

        var result = new Dictionary<int, Vec2D>(rows.Count);
        foreach (int id in rows.Keys) result.Add(id, new Vec2D(.5, .5));
        double perimeter = 0;
        var lengths = new double[boundary.Count];
        for (int i = 0; i < boundary.Count; i++)
        {
            lengths[i] = (patch.Points[boundary[(i + 1) % boundary.Count]] -
                patch.Points[boundary[i]]).Length();
            perimeter += lengths[i];
        }
        if (!double.IsFinite(perimeter) || perimeter <= 0) return false;
        double distance = 0;
        for (int i = 0; i < boundary.Count; i++)
        {
            double angle = 2 * Math.PI * distance / perimeter;
            result[boundary[i]] = new Vec2D(.5 + .5 * Math.Cos(angle), .5 + .5 * Math.Sin(angle));
            distance += lengths[i];
        }
        for (int i = 0; i < boundary.Count; i++)
            if (Orientation(result[boundary[i]], result[boundary[(i + 1) % boundary.Count]],
                result[boundary[(i + 2) % boundary.Count]]) <= 0) return false;

        var interior = new List<int>();
        foreach (var (id, row) in rows)
        {
            if (next.ContainsKey(id)) continue;
            double sum = 0;
            foreach (double weight in row.Values) sum += weight;
            if (!double.IsFinite(sum) || sum <= 0) return false;
            foreach (int neighbor in row.Keys) row[neighbor] /= sum;
            interior.Add(id);
        }
        bool converged = interior.Count == 0;
        // Sparse Gauss-Seidel: no dense vertex-by-vertex matrix or new dependency.
        // Failure to converge is an explicit failure, never a folded fallback chart.
        for (int iteration = 0; iteration < 20000 && !converged; iteration++)
        {
            double change = 0;
            foreach (int id in interior)
            {
                var value = new Vec2D(0, 0);
                foreach (var (neighbor, weight) in rows[id]) value += result[neighbor] * weight;
                var previous = result[id];
                change = Math.Max(change, Math.Max(Math.Abs(value.X - previous.X), Math.Abs(value.Y - previous.Y)));
                result[id] = value;
            }
            if (!double.IsFinite(change)) return false;
            converged = change <= 1e-12;
        }
        if (!converged || !HasConsistentOrientation(patch.Triangles, result, requiredSign: 1)) return false;
        uv = result;
        return true;

        bool AddCorner(int a, int b, int c, int triangle)
        {
            if ((uint)a >= patch.Points.Count || (uint)b >= patch.Points.Count ||
                (uint)c >= patch.Points.Count || a == b || a == c || b == c) return false;
            var u = patch.Points[b] - patch.Points[a];
            var v = patch.Points[c] - patch.Points[a];
            double lu = u.Length(), lv = v.Length();
            if (!double.IsFinite(lu) || !double.IsFinite(lv) || lu <= 0 || lv <= 0) return false;
            u /= lu; v /= lv;
            double sine = Vec3DOps.Cross(u, v).Length(), cosine = Vec3DOps.Dot(u, v);
            // Avoid cancellation in 1+cos(theta) near a straight angle.
            double tangent = cosine >= 0 ? sine / (1 + cosine) : (1 - cosine) / sine;
            if (!double.IsFinite(tangent) || tangent <= 0) return false;
            if (!rows.TryGetValue(a, out var row))
            {
                rows.Add(a, row = new Dictionary<int, double>());
                incident.Add(a, new List<int>());
            }
            incident[a].Add(triangle);
            row[b] = row.GetValueOrDefault(b) + tangent / lu;
            row[c] = row.GetValueOrDefault(c) + tangent / lv;
            return true;
        }

        bool AddEdge(int a, int b, int triangle)
        {
            var key = (Math.Min(a, b), Math.Max(a, b));
            if (!edges.TryGetValue(key, out var edge))
                edges.Add(key, (a, b, triangle, -1));
            else
            {
                if (edge.Second >= 0 || edge.From != b || edge.To != a) return false;
                adjacent[triangle].Add(edge.First); adjacent[edge.First].Add(triangle);
                edges[key] = (edge.From, edge.To, edge.First, triangle);
            }
            return true;
        }
    }

    // Checks local injectivity only. For TryCreate, disk topology and the convex
    // boundary additionally establish global injectivity. Arbitrary supplied UVs
    // also need overlap/boundary validation before use as an extension chart.
    internal static bool HasConsistentOrientation(IReadOnlyList<Tri> triangles,
        IReadOnlyDictionary<int, Vec2D> uv, int requiredSign = 0)
    {
        var exact = new Dictionary<int, Rat2Hybrid>(uv.Count);
        foreach (var (id, p) in uv)
        {
            if (!Finite(p)) return false;
            exact.Add(id, Exact(p));
        }
        foreach (var t in triangles)
        {
            if (!exact.TryGetValue(t.A, out var a) || !exact.TryGetValue(t.B, out var b) ||
                !exact.TryGetValue(t.C, out var c)) return false;
            int sign = Rat2Hybrid.Orient2DSign(a, b, c);
            if (sign == 0 || (requiredSign != 0 && sign != requiredSign)) return false;
            requiredSign = sign;
        }
        return triangles.Count > 0;
    }

    private static bool Finite(Vec2D p) => double.IsFinite(p.X) && double.IsFinite(p.Y);

    private static int Orientation(Vec2D a, Vec2D b, Vec2D c) =>
        Rat2Hybrid.Orient2DSign(Exact(a), Exact(b), Exact(c));

    private static Rat2Hybrid Exact(Vec2D p)
    {
        // Exact sign of the stored doubles, not an epsilon-area heuristic.
        var x = new BigRational(p.X); var y = new BigRational(p.Y);
        return new Rat2Hybrid(new BigRationalHybrid(x.Numerator, x.Denominator),
            new BigRationalHybrid(y.Numerator, y.Denominator));
    }

    private static int Reach(int start, List<int>[] adjacency, HashSet<int> allowed)
    {
        var seen = new HashSet<int> { start };
        var pending = new Stack<int>(); pending.Push(start);
        while (pending.TryPop(out int current))
            foreach (int neighbor in adjacency[current])
                if ((allowed == null || allowed.Contains(neighbor)) && seen.Add(neighbor)) pending.Push(neighbor);
        return seen.Count;
    }
}
