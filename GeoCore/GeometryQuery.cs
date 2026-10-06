namespace GeoCore;

/// <summary>Immutable, CPU-side queries on displayed points, segments and triangles.
/// Accuracy is that of the input tessellation; analytic metadata is not required.</summary>
public sealed class GeometryQuery
{
    public readonly record struct Primitive(Vec3D A, Vec3D B, Vec3D C, int Count);
    public readonly record struct Pair(Vec3D First, Vec3D Second, double SquaredDistance);
    public readonly record struct Hit(int Index, double Depth, double Distance);
    readonly Primitive[] items;
    readonly Lazy<BVHNode[]> tree;
    public GeometryQuery(Primitive[] primitives)
    {
        items = (Primitive[])primitives.Clone();
        foreach (var p in items)
            if (p.Count < 1 || p.Count > 3 || !Finite(p.A) || !Finite(p.B) || !Finite(p.C))
                throw new ArgumentException("Expected finite points, segments or triangles.");
        tree = new Lazy<BVHNode[]>(() => items.Length == 0 ? [] :
            Tree.BuildTreeFast(i => Bounds(items[i]), items.Length));
    }
    static bool Finite(Vec3D p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z);
    static double Dot(Vec3D a, Vec3D b) => Vec3DOps.Dot(a, b);
    static Vec3D Cross(Vec3D a, Vec3D b) => Vec3DOps.Cross(a, b);
    static double Size(Box3F b) => (double)(b.Max.X - b.Min.X) * (b.Max.X - b.Min.X)
        + (double)(b.Max.Y - b.Min.Y) * (b.Max.Y - b.Min.Y) + (double)(b.Max.Z - b.Min.Z) * (b.Max.Z - b.Min.Z);
    // The shared BVH uses float boxes. Round OUTWARD so conversion cannot hide
    // double-precision geometry and incorrectly prune a closest pair or ray hit.
    static float Lower(double v) { float f = (float)v; return f > v ? float.BitDecrement(f) : f; }
    static float Upper(double v) { float f = (float)v; return f < v ? float.BitIncrement(f) : f; }
    static Box3F Bounds(Primitive p)
    {
        var lo = p.A; var hi = p.A;
        void Extend(Vec3D v)
        {
            lo = new(Math.Min(lo.X, v.X), Math.Min(lo.Y, v.Y), Math.Min(lo.Z, v.Z));
            hi = new(Math.Max(hi.X, v.X), Math.Max(hi.Y, v.Y), Math.Max(hi.Z, v.Z));
        }
        if (p.Count > 1) Extend(p.B); if (p.Count > 2) Extend(p.C);
        return new Box3F { Min = new(Lower(lo.X), Lower(lo.Y), Lower(lo.Z)), Max = new(Upper(hi.X), Upper(hi.Y), Upper(hi.Z)) };
    }
    static double BoxDistance(Box3F a, Box3F b)
    {
        double x = Math.Max(0, Math.Max((double)a.Min.X - b.Max.X, (double)b.Min.X - a.Max.X));
        double y = Math.Max(0, Math.Max((double)a.Min.Y - b.Max.Y, (double)b.Min.Y - a.Max.Y));
        double z = Math.Max(0, Math.Max((double)a.Min.Z - b.Max.Z, (double)b.Min.Z - a.Max.Z));
        return x * x + y * y + z * z;
    }
    public Pair Closest(GeometryQuery other)
    {
        if (items.Length == 0 || other.items.Length == 0) throw new ArgumentException("Entities must have nonempty geometry.");
        var a = tree.Value; var b = other.tree.Value;
        var queue = new PriorityQueue<(int, int), double>(); queue.Enqueue((0, 0), BoxDistance(a[0].Bounds, b[0].Bounds));
        var best = new Pair(default, default, double.PositiveInfinity);
        while (queue.TryDequeue(out var pair, out double lower))
        {
            if (lower >= best.SquaredDistance) break;
            var x = a[pair.Item1]; var y = b[pair.Item2];
            if (x.IsLeaveNode && y.IsLeaveNode)
            {
                var result = Distance(items[x.ChildB], other.items[y.ChildB]);
                if (result.SquaredDistance < best.SquaredDistance) best = result;
                continue;
            }
            bool split = !x.IsLeaveNode && (y.IsLeaveNode || Size(x.Bounds) >= Size(y.Bounds));
            Enqueue(split ? (x.ChildA, pair.Item2) : (pair.Item1, y.ChildA));
            Enqueue(split ? (x.ChildB, pair.Item2) : (pair.Item1, y.ChildB));
            void Enqueue((int, int) next)
            {
                double bound = BoxDistance(a[next.Item1].Bounds, b[next.Item2].Bounds);
                if (bound < best.SquaredDistance) queue.Enqueue(next, bound);
            }
        }
        return best;
    }
    static Pair Make(Vec3D a, Vec3D b) => new(a, b, Dot(a - b, a - b));
    static Vec3D PointSegment(Vec3D p, Vec3D a, Vec3D b)
    { var d = b - a; double den = Dot(d, d); return den == 0 ? a : a + d * Math.Clamp(Dot(p - a, d) / den, 0, 1); }
    static Vec3D PointTriangle(Vec3D p, Primitive t)
    {
        var ab = t.B - t.A; var ac = t.C - t.A; var ap = p - t.A;
        double d1 = Dot(ab, ap), d2 = Dot(ac, ap);
        var normal = Cross(ab, ac);
        if (Dot(normal, normal) == 0) return Degenerate();
        if (d1 <= 0 && d2 <= 0) return t.A;
        var bp = p - t.B; double d3 = Dot(ab, bp), d4 = Dot(ac, bp);
        if (d3 >= 0 && d4 <= d3) return t.B;
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) return t.A + ab * (d1 / (d1 - d3));
        var cp = p - t.C; double d5 = Dot(ab, cp), d6 = Dot(ac, cp);
        if (d6 >= 0 && d5 <= d6) return t.C;
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) return t.A + ac * (d2 / (d2 - d6));
        double va = d3 * d6 - d5 * d4;
        if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) return t.B + (t.C - t.B) * ((d4 - d3) / (d4 - d3 + d5 - d6));
        double total = va + vb + vc;
        return total == 0 ? Degenerate() : t.A + ab * (vb / total) + ac * (vc / total);
        Vec3D Degenerate()
        {
            var q = PointSegment(p, t.A, t.B); var r = PointSegment(p, t.B, t.C); var s = PointSegment(p, t.C, t.A);
            if (Dot(p - r, p - r) < Dot(p - q, p - q)) q = r;
            return Dot(p - s, p - s) < Dot(p - q, p - q) ? s : q;
        }
    }
    static Pair Segments(Vec3D a, Vec3D b, Vec3D c, Vec3D d)
    {
        var u = b - a; var v = d - c; var w = a - c;
        double aa = Dot(u, u), bb = Dot(u, v), cc = Dot(v, v), dd = Dot(u, w), ee = Dot(v, w);
        if (aa == 0) return Make(a, PointSegment(a, c, d));
        if (cc == 0) return Make(PointSegment(c, a, b), c);
        var n = Cross(u, v); double den = Dot(n, n);
        double s = den > 0 ? Math.Clamp(Dot(Cross(v, w), n) / den, 0, 1) : 0;
        double t = (bb * s + ee) / cc;
        if (t < 0) { t = 0; s = Math.Clamp(-dd / aa, 0, 1); }
        else if (t > 1) { t = 1; s = Math.Clamp((bb - dd) / aa, 0, 1); }
        return Make(a + u * s, c + v * t);
    }
    static Pair SegmentTriangle(Vec3D a, Vec3D b, Primitive t)
    {
        var n = Cross(t.B - t.A, t.C - t.A); double den = Dot(n, b - a);
        if (den != 0)
        {
            double u = Dot(n, t.A - a) / den; var p = a + (b - a) * u;
            if (u >= 0 && u <= 1 && Dot(Cross(t.B - t.A, p - t.A), n) >= 0
                && Dot(Cross(t.C - t.B, p - t.B), n) >= 0 && Dot(Cross(t.A - t.C, p - t.C), n) >= 0) return Make(p, p);
        }
        var best = Make(a, PointTriangle(a, t)); Consider(Make(b, PointTriangle(b, t)));
        Consider(Segments(a, b, t.A, t.B)); Consider(Segments(a, b, t.B, t.C)); Consider(Segments(a, b, t.C, t.A));
        return best;
        void Consider(Pair p) { if (p.SquaredDistance < best.SquaredDistance) best = p; }
    }
    public static Pair Distance(Primitive a, Primitive b)
    {
        if (a.Count == 1) return Make(a.A, b.Count == 1 ? b.A : b.Count == 2 ? PointSegment(a.A, b.A, b.B) : PointTriangle(a.A, b));
        if (b.Count == 1) { var p = Distance(b, a); return new(p.Second, p.First, p.SquaredDistance); }
        if (a.Count == 2) return b.Count == 2 ? Segments(a.A, a.B, b.A, b.B) : SegmentTriangle(a.A, a.B, b);
        if (b.Count == 2) { var p = SegmentTriangle(b.A, b.B, a); return new(p.Second, p.First, p.SquaredDistance); }
        var best = SegmentTriangle(a.A, a.B, b); Consider(SegmentTriangle(a.B, a.C, b)); Consider(SegmentTriangle(a.C, a.A, b));
        Reverse(SegmentTriangle(b.A, b.B, a)); Reverse(SegmentTriangle(b.B, b.C, a)); Reverse(SegmentTriangle(b.C, b.A, a));
        return best;
        void Consider(Pair p) { if (p.SquaredDistance < best.SquaredDistance) best = p; }
        void Reverse(Pair p) => Consider(new(p.Second, p.First, p.SquaredDistance));
    }
    static double RayBox(Vec3D o, Vec3D d, Box3F b, double radius)
    {
        double near = 0, far = double.PositiveInfinity;
        bool Axis(double origin, double direction, double lo, double hi)
        {
            if (direction == 0) return origin >= lo - radius && origin <= hi + radius;
            double a = (lo - radius - origin) / direction, c = (hi + radius - origin) / direction;
            near = Math.Max(near, Math.Min(a, c)); far = Math.Min(far, Math.Max(a, c)); return near <= far;
        }
        return Axis(o.X, d.X, b.Min.X, b.Max.X) && Axis(o.Y, d.Y, b.Min.Y, b.Max.Y)
            && Axis(o.Z, d.Z, b.Min.Z, b.Max.Z) ? near : double.PositiveInfinity;
    }
    /// <summary>Near-first ray traversal. Count selects points, segments or triangles;
    /// radius is the screen-derived world tolerance for overlay picking.</summary>
    public Hit Ray(Vec3D origin, Vec3D direction, int count, double radius = 0, Vec3D? clipNormal = null, double clipOffset = 0)
    {
        if (count < 1 || count > 3 || double.IsNaN(radius) || radius < 0 || !Finite(origin) || !Finite(direction)
            || (clipNormal is Vec3D cn && (!Finite(cn) || !double.IsFinite(clipOffset))))
            throw new ArgumentException("Invalid ray query.");
        var best = new Hit(-1, double.PositiveInfinity, double.PositiveInfinity);
        double front = double.PositiveInfinity;
        List<Hit> candidates = count == 3 ? null : new();
        double length = Math.Sqrt(Dot(direction, direction)); if (length == 0 || items.Length == 0) return best;
        direction = direction * (1 / length); var nodes = tree.Value;
        var queue = new PriorityQueue<int, double>(); queue.Enqueue(0, RayBox(origin, direction, nodes[0].Bounds, radius));
        while (queue.TryDequeue(out int index, out double near))
        {
            if (near > front + radius || double.IsPositiveInfinity(near)) break;
            var node = nodes[index];
            if (!node.IsLeaveNode)
            { Enqueue(node.ChildA); Enqueue(node.ChildB); continue; }
            var p = items[node.ChildB]; if (p.Count != count) continue;
            double depth, distance;
            if (count == 3)
            {
                var e1 = p.B - p.A; var e2 = p.C - p.A; var v = Cross(direction, e2); double det = Dot(e1, v);
                if (det == 0) continue;
                var s = origin - p.A; double u = Dot(s, v) / det; var q = Cross(s, e1); double w = Dot(direction, q) / det;
                depth = Dot(e2, q) / det; distance = 0;
                if (u < 0 || w < 0 || u + w > 1 || depth <= 0) continue;
                if (clipNormal is Vec3D n && Dot(origin + direction * depth, n) > clipOffset) continue;
            }
            else
            {
                var a = p.A; var b = count == 1 ? a : p.B;
                if (clipNormal is Vec3D n)
                {
                    double da = Dot(a, n) - clipOffset, db = Dot(b, n) - clipOffset;
                    if (da > 0 && db > 0) continue;
                    if (da > 0) a = a + (b - a) * (da / (da - db)); else if (db > 0) b = a + (b - a) * (da / (da - db));
                }
                var v = b - a; var w = origin - a; double vv = Dot(v, v), dv = Dot(direction, v), dw = Dot(direction, w), vw = Dot(v, w);
                double den = Dot(Cross(direction, v), Cross(direction, v));
                double t = den > 0 ? Math.Clamp((vw - dv * dw) / den, 0, 1) : 0;
                if (den == 0 && dv != 0) t = Math.Clamp(dw / dv, 0, 1);
                depth = Math.Max(0, dv * t - dw);
                if (depth == 0) t = vv == 0 ? 0 : Math.Clamp(vw / vv, 0, 1);
                distance = Math.Sqrt(Dot(origin + direction * depth - (a + v * t), origin + direction * depth - (a + v * t)));
                if (distance >= radius) continue;
            }
            front = Math.Min(front, depth);
            if (count == 3) { if (depth < best.Depth) best = new(node.ChildB, depth, distance); }
            else candidates.Add(new(node.ChildB, depth, distance));
            void Enqueue(int child)
            {
                double entry = RayBox(origin, direction, nodes[child].Bounds, radius);
                if (entry <= front + radius) queue.Enqueue(child, entry);
            }
        }
        if (candidates != null)
            foreach (var hit in candidates)
                if (hit.Depth <= front + radius && (hit.Distance < best.Distance
                    || (hit.Distance == best.Distance && (hit.Depth < best.Depth
                    || (hit.Depth == best.Depth && hit.Index < best.Index))))) best = hit;
        return best;
    }
}
