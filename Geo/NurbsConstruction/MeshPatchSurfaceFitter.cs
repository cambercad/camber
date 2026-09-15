using GeoCore;
using NURBS;

namespace Geo.NurbsConstruction
{
    public sealed class FittedMeshSurface
    {
        public BSplineSurface Surface { get; set; }
        public bool InventedUv { get; set; }
        public Dictionary<int, Vec2D> VertexUv { get; set; }
        public Vec2D UvMin { get; set; }
        public Vec2D UvSpan { get; set; }
        public bool NormalizeOriginalUv { get; set; }
        public int ControlCountU { get; set; }
        public int ControlCountV { get; set; }
        public double MaxError { get; set; }
    }

    /// <summary>
    /// Fits a compact bicubic (or bilinear) B-spline to a triangulated patch.
    /// Control-point count grows only until the vertex error meets <c>tolerance</c>.
    /// </summary>
    public static class MeshPatchSurfaceFitter
    {
        public const int MaxControlPointsPerDir = 12;
        private const int MaxSamples = 320;
        private const double MinUvSpan = 1e-9;

        public static bool TryFit(UVSurface patch, double tolerance, out FittedMeshSurface fitted)
        {
            fitted = null;
            if (patch == null || patch.Triangles == null || patch.Triangles.Count == 0)
                return false;
            if (tolerance <= 0)
                tolerance = InferTolerance(patch);

            var vertexIds = new List<int>();
            var seen = new HashSet<int>();
            for (int t = 0; t < patch.Triangles.Count; t++)
            {
                var tri = patch.Triangles[t];
                AddVertex(seen, vertexIds, tri.A);
                AddVertex(seen, vertexIds, tri.B);
                AddVertex(seen, vertexIds, tri.C);
            }
            if (vertexIds.Count < 3)
                return false;

            var world = new List<Vec3D>(vertexIds.Count);
            var uv = new List<Vec2D>(vertexIds.Count);
            bool invented;
            Vec2D uvMin;
            Vec2D uvSpan;
            Dictionary<int, Vec2D> vertexUv;
            CollectUv(patch, vertexIds, world, uv, out invented, out uvMin, out uvSpan, out vertexUv);
            if (world.Count < 3)
                return false;

            Subsample(uv, world, MaxSamples);

            int[] sizes = { 2, 4, 5, 6, 8, 10, MaxControlPointsPerDir };
            BSplineSurface best = null;
            double bestErr = double.MaxValue;
            int bestU = 0;
            int bestV = 0;

            for (int s = 0; s < sizes.Length; s++)
            {
                int n = sizes[s];
                if (n * n > world.Count)
                    break;
                int degree = n <= 2 ? 1 : 3;
                if (n < degree + 1)
                    continue;

                var candidate = BSplineSurfaceFitter.FitScattered(uv, world, n, n, degree, degree);
                if (candidate == null)
                    continue;
                double err = BSplineSurfaceFitter.MaxError(candidate, uv, world);
                if (err < bestErr)
                {
                    best = candidate;
                    bestErr = err;
                    bestU = n;
                    bestV = n;
                }
                if (err <= tolerance)
                    break;
            }

            if (best == null)
                return false;

            fitted = new FittedMeshSurface
            {
                Surface = best,
                InventedUv = invented,
                VertexUv = vertexUv,
                UvMin = uvMin,
                UvSpan = uvSpan,
                NormalizeOriginalUv = !invented,
                ControlCountU = bestU,
                ControlCountV = bestV,
                MaxError = bestErr
            };
            return true;
        }

        public static double InferTolerance(UVSurface patch)
        {
            double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;
            for (int t = 0; t < patch.Triangles.Count; t++)
            {
                var tri = patch.Triangles[t];
                AccBBox(patch.Points[tri.A], ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
                AccBBox(patch.Points[tri.B], ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
                AccBBox(patch.Points[tri.C], ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ);
            }
            double dx = maxX - minX;
            double dy = maxY - minY;
            double dz = maxZ - minZ;
            double diag = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            return Math.Max(diag * 1e-3, 1e-6);
        }

        public static Vec2D NormalizeUv(Vec2D uv, Vec2D min, Vec2D span)
        {
            double u = span.X > MinUvSpan ? (uv.X - min.X) / span.X : 0.0;
            double v = span.Y > MinUvSpan ? (uv.Y - min.Y) / span.Y : 0.0;
            if (u < 0) u = 0;
            if (u > 1) u = 1;
            if (v < 0) v = 0;
            if (v > 1) v = 1;
            return new Vec2D(u, v);
        }

        private static void CollectUv(
            UVSurface patch,
            List<int> vertexIds,
            List<Vec3D> world,
            List<Vec2D> uv,
            out bool invented,
            out Vec2D uvMin,
            out Vec2D uvSpan,
            out Dictionary<int, Vec2D> vertexUv)
        {
            uvMin = new Vec2D(double.MaxValue, double.MaxValue);
            var uvMax = new Vec2D(double.MinValue, double.MinValue);
            bool hasUv = patch.Uv != null && patch.Uv.Count >= patch.Points.Count;
            if (hasUv)
            {
                for (int i = 0; i < vertexIds.Count; i++)
                {
                    var t = patch.Uv[vertexIds[i]];
                    if (t.X < uvMin.X) uvMin.X = t.X;
                    if (t.Y < uvMin.Y) uvMin.Y = t.Y;
                    if (t.X > uvMax.X) uvMax.X = t.X;
                    if (t.Y > uvMax.Y) uvMax.Y = t.Y;
                }
            }

            uvSpan = new Vec2D(uvMax.X - uvMin.X, uvMax.Y - uvMin.Y);
            invented = !hasUv || (uvSpan.X < MinUvSpan && uvSpan.Y < MinUvSpan) || uvSpan.X * uvSpan.Y < 1e-16;
            vertexUv = invented ? new Dictionary<int, Vec2D>(vertexIds.Count) : null;

            if (invented)
                InventPlanarUv(patch, vertexIds, vertexUv, out uvMin, out uvSpan);

            for (int i = 0; i < vertexIds.Count; i++)
            {
                int id = vertexIds[i];
                world.Add(patch.Points[id]);
                if (invented)
                    uv.Add(vertexUv[id]);
                else
                    uv.Add(NormalizeUv(patch.Uv[id], uvMin, uvSpan));
            }
        }

        private static void InventPlanarUv(
            UVSurface patch,
            List<int> vertexIds,
            Dictionary<int, Vec2D> vertexUv,
            out Vec2D uvMin,
            out Vec2D uvSpan)
        {
            var pts = new List<Vec3D>(vertexIds.Count);
            for (int i = 0; i < vertexIds.Count; i++)
                pts.Add(patch.Points[vertexIds[i]]);

            var (normal, origin) = PlaneFitter.FitPlane(pts);
            var hint = Math.Abs(normal.X) < 0.9 ? new Vec3D(1, 0, 0) : new Vec3D(0, 1, 0);
            var x = Vec3DOps.Cross(normal, hint);
            if (x.LengthSquared() < 1e-16)
                x = Vec3DOps.Cross(normal, new Vec3D(0, 1, 0));
            x = x.Normalized();
            var y = Vec3DOps.Cross(normal, x).Normalized();

            uvMin = new Vec2D(double.MaxValue, double.MaxValue);
            var uvMax = new Vec2D(double.MinValue, double.MinValue);
            var raw = new Vec2D[vertexIds.Count];
            for (int i = 0; i < vertexIds.Count; i++)
            {
                var d = pts[i] - origin;
                raw[i] = new Vec2D(Vec3DOps.Dot(d, x), Vec3DOps.Dot(d, y));
                if (raw[i].X < uvMin.X) uvMin.X = raw[i].X;
                if (raw[i].Y < uvMin.Y) uvMin.Y = raw[i].Y;
                if (raw[i].X > uvMax.X) uvMax.X = raw[i].X;
                if (raw[i].Y > uvMax.Y) uvMax.Y = raw[i].Y;
            }

            uvSpan = new Vec2D(uvMax.X - uvMin.X, uvMax.Y - uvMin.Y);
            if (uvSpan.X < MinUvSpan) uvSpan.X = 1.0;
            if (uvSpan.Y < MinUvSpan) uvSpan.Y = 1.0;
            for (int i = 0; i < vertexIds.Count; i++)
                vertexUv[vertexIds[i]] = NormalizeUv(raw[i], uvMin, uvSpan);
            uvMin = new Vec2D(0, 0);
            uvSpan = new Vec2D(1, 1);
        }

        private static void Subsample(List<Vec2D> uv, List<Vec3D> world, int maxSamples)
        {
            if (uv.Count <= maxSamples)
                return;

            int bins = 16;
            var keep = new bool[uv.Count];
            var occupied = new HashSet<int>();
            int kept = 0;
            for (int i = 0; i < uv.Count; i++)
            {
                int bx = (int)(Clamp01(uv[i].X) * (bins - 1));
                int by = (int)(Clamp01(uv[i].Y) * (bins - 1));
                int key = bx * bins + by;
                if (occupied.Add(key))
                {
                    keep[i] = true;
                    kept++;
                }
            }

            if (kept < maxSamples)
            {
                int stride = Math.Max(1, uv.Count / (maxSamples - kept + 1));
                for (int i = 0; i < uv.Count && kept < maxSamples; i += stride)
                {
                    if (keep[i])
                        continue;
                    keep[i] = true;
                    kept++;
                }
            }

            int w = 0;
            for (int i = 0; i < uv.Count; i++)
            {
                if (!keep[i])
                    continue;
                uv[w] = uv[i];
                world[w] = world[i];
                w++;
            }
            uv.RemoveRange(w, uv.Count - w);
            world.RemoveRange(w, world.Count - w);
        }

        private static void AddVertex(HashSet<int> seen, List<int> ids, int v)
        {
            if (seen.Add(v))
                ids.Add(v);
        }

        private static void AccBBox(Vec3D p, ref double minX, ref double minY, ref double minZ, ref double maxX, ref double maxY, ref double maxZ)
        {
            if (p.X < minX) minX = p.X;
            if (p.Y < minY) minY = p.Y;
            if (p.Z < minZ) minZ = p.Z;
            if (p.X > maxX) maxX = p.X;
            if (p.Y > maxY) maxY = p.Y;
            if (p.Z > maxZ) maxZ = p.Z;
        }

        private static double Clamp01(double t)
        {
            if (t < 0) return 0;
            if (t > 1) return 1;
            return t;
        }
    }
}
