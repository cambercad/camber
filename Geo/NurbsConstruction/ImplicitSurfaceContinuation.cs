using GeoCore;

namespace Geo.NurbsConstruction;

/// <summary>
/// One implicit polynomial guide per mesh patch, fitted to positions and normal
/// directions. No sphere/cylinder recognition, UV requirement, or invented
/// analytic metadata. The authoritative triangles are never replaced. Fits are
/// checked at every source sample; that is not a certified bound on unknown
/// geometry beyond the trim. A failed fit is not a usable continuation.
/// </summary>
internal sealed class ImplicitSurfaceContinuation
{
    private readonly Vec3D _origin;
    private readonly double _scale;
    private readonly (int X, int Y, int Z)[] _powers;
    private readonly double[] _coefficients;

    private ImplicitSurfaceContinuation(Vec3D origin, double scale,
        (int, int, int)[] powers, double[] coefficients)
    { _origin = origin; _scale = scale; _powers = powers; _coefficients = coefficients; }

    internal static bool TryFit(IReadOnlyList<(Vec3D Point, Vec3D Normal)> samples,
        double tolerance, out ImplicitSurfaceContinuation fit)
    {
        fit = null;
        if (samples.Count < 10 || !(tolerance > 0) || !double.IsFinite(tolerance)) return false;
        var origin = samples[0].Point;
        double scale = 0;
        foreach (var sample in samples)
        {
            if (!Finite(sample.Point) || !Finite(sample.Normal) || sample.Normal.LengthSquared() == 0) return false;
            scale = Math.Max(scale, (sample.Point - origin).Length());
        }
        if (!(scale > 0) || !double.IsFinite(scale)) return false;
        double spacing = scale / Math.Sqrt(samples.Count);
        for (int degree = 2; degree <= 4; degree++)
        {
            var powers = new List<(int, int, int)>();
            for (int total = 0; total <= degree; total++)
                for (int x = 0; x <= total; x++)
                    for (int y = 0; y <= total - x; y++) powers.Add((x, y, total - x - y));
            int count = powers.Count;
            if (samples.Count < count) continue;
            var matrix = new double[count + 1, count + 1];
            var rhs = new double[count + 1]; rhs[count] = 1;
            var row = new double[count];
            var dx = new double[count]; var dy = new double[count]; var dz = new double[count];
            // Bound fitting cost independently of tessellation density. Validation
            // below still visits EVERY referenced vertex, not just this subsample.
            int selected = Math.Min(samples.Count, 320);
            for (int s = 0; s < selected; s++)
            {
                var sample = samples[s * (samples.Count - 1) / (selected - 1)];
                var p = (sample.Point - origin) / scale; var n = sample.Normal.Normalized();
                for (int i = 0; i < count; i++)
                {
                    row[i] = Term(powers[i], p);
                    dx[i] = Term(powers[i], p, 1, 0, 0);
                    dy[i] = Term(powers[i], p, 0, 1, 0);
                    dz[i] = Term(powers[i], p, 0, 0, 1);
                }
                Add();
                double weight = spacing / scale;
                for (int i = 0; i < count; i++) row[i] = (dy[i] * n.Z - dz[i] * n.Y) * weight;
                Add();
                for (int i = 0; i < count; i++) row[i] = (dz[i] * n.X - dx[i] * n.Z) * weight;
                Add();
                for (int i = 0; i < count; i++) row[i] = (dx[i] * n.Y - dy[i] * n.X) * weight;
                Add();
            }
            void Add()
            {
                for (int i = 0; i < count; i++)
                    for (int j = 0; j < count; j++) matrix[i, j] += row[i] * row[j];
            }
            // Fix the homogeneous polynomial's scale by its outward derivative
            // at the seed; the zero polynomial must never qualify as a fit.
            var seedNormal = samples[0].Normal.Normalized();
            for (int i = 0; i < count; i++)
                matrix[i, count] = matrix[count, i] =
                    Term(powers[i], new Vec3D(0), 1, 0, 0) * seedNormal.X +
                    Term(powers[i], new Vec3D(0), 0, 1, 0) * seedNormal.Y +
                    Term(powers[i], new Vec3D(0), 0, 0, 1) * seedNormal.Z;
            var coefficients = new double[count + 1];
            if (!GeoSolver.NewtonSolver.SolveLinearSystem(coefficients, matrix, rhs, count + 1)) continue;
            bool valid = true;
            foreach (double c in coefficients) if (!double.IsFinite(c)) valid = false;
            var candidate = new ImplicitSurfaceContinuation(origin, scale, powers.ToArray(), coefficients);
            foreach (var sample in samples)
            {
                var p = (sample.Point - origin) / scale;
                var g = candidate.Gradient(p); double length = g.Length();
                if (!(length > 0) || Math.Abs(candidate.Evaluate(p)) * scale / length > tolerance ||
                    (g / length - sample.Normal.Normalized()).Length() * spacing > tolerance)
                { valid = false; break; }
            }
            if (!valid) continue;
            fit = candidate;
            return true;
        }
        return false;
    }

    internal bool TryOffsetTangent(Vec3D candidate, double distance,
        out (Vec3D Point, Vec3D Normal) tangent, double maxExtension = 0)
    {
        tangent = default;
        if (!Finite(candidate) || !double.IsFinite(distance)) return false;
        var target = (candidate - _origin) / _scale;
        // Only a bounded neighborhood of the measured patch is usable.
        double limit = 1 + 2 * Math.Max(Math.Abs(distance), maxExtension) / _scale;
        if (target.Length() > limit) return false;
        var p = target;
        double lambda = 0;
        var gradient = new double[3];
        var hessian = new double[3, 3];
        var matrix = new double[4, 4];
        var rhs = new double[4];
        var step = new double[4];
        for (int iteration = 0; iteration < 32; iteration++)
        {
            double value = EvaluateJet(p, gradient, hessian);
            var g = new Vec3D(gradient[0], gradient[1], gradient[2]);
            double length = g.Length();
            if (!(length > 0) || !double.IsFinite(length)) return false;
            var residual = p - target + g * lambda;
            if (residual.Length() < 1e-11 && Math.Abs(value) / length < 1e-11)
            {
                var normal = g / length;
                var x = Vec3DOps.GetOrthoNormal(normal); var y = Vec3DOps.Cross(normal, x).Normalized();
                Vec3D Multiply(Vec3D v) => new(
                    hessian[0, 0] * v.X + hessian[0, 1] * v.Y + hessian[0, 2] * v.Z,
                    hessian[1, 0] * v.X + hessian[1, 1] * v.Y + hessian[1, 2] * v.Z,
                    hessian[2, 0] * v.X + hessian[2, 1] * v.Y + hessian[2, 2] * v.Z);
                double amount = distance / (_scale * length);
                double xx = 1 + amount * Vec3DOps.Dot(x, Multiply(x));
                double xy = amount * Vec3DOps.Dot(x, Multiply(y));
                double yy = 1 + amount * Vec3DOps.Dot(y, Multiply(y));
                if (!(xx > 0) || !(xx * yy - xy * xy > 0)) return false;
                tangent = (_origin + p * _scale + normal * distance, normal);
                return Finite(tangent.Point);
            }
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++) matrix[i, j] = (i == j ? 1 : 0) + lambda * hessian[i, j];
                matrix[i, 3] = matrix[3, i] = gradient[i];
            }
            matrix[3, 3] = 0;
            rhs[0] = -residual.X; rhs[1] = -residual.Y; rhs[2] = -residual.Z; rhs[3] = -value;
            if (!GeoSolver.NewtonSolver.SolveLinearSystem(step, matrix, rhs, 4)) return false;
            p += new Vec3D(step[0], step[1], step[2]); lambda += step[3];
            if (!Finite(p) || p.Length() > limit) return false;
        }
        return false;
    }

    private Vec3D Gradient(Vec3D p) => new(Evaluate(p, 1, 0, 0), Evaluate(p, 0, 1, 0), Evaluate(p, 0, 0, 1));
    private double[,] Hessian(Vec3D p) => new double[,] {
        { Evaluate(p, 2, 0, 0), Evaluate(p, 1, 1, 0), Evaluate(p, 1, 0, 1) },
        { Evaluate(p, 1, 1, 0), Evaluate(p, 0, 2, 0), Evaluate(p, 0, 1, 1) },
        { Evaluate(p, 1, 0, 1), Evaluate(p, 0, 1, 1), Evaluate(p, 0, 0, 2) } };

    private double EvaluateJet(Vec3D p, double[] gradient, double[,] hessian)
    {
        Span<double> x = stackalloc double[5] { 1, p.X, p.X * p.X, 0, 0 };
        Span<double> y = stackalloc double[5] { 1, p.Y, p.Y * p.Y, 0, 0 };
        Span<double> z = stackalloc double[5] { 1, p.Z, p.Z * p.Z, 0, 0 };
        for (int i = 3; i < 5; i++)
        {
            x[i] = x[i - 1] * p.X;
            y[i] = y[i - 1] * p.Y;
            z[i] = z[i - 1] * p.Z;
        }
        gradient[0] = gradient[1] = gradient[2] = 0;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) hessian[i, j] = 0;

        double value = 0;
        for (int i = 0; i < _powers.Length; i++)
        {
            var (px, py, pz) = _powers[i];
            double c = _coefficients[i];
            double monomial = x[px] * y[py] * z[pz];
            value += c * monomial;
            if (px > 0) gradient[0] += c * px * x[px - 1] * y[py] * z[pz];
            if (py > 0) gradient[1] += c * py * x[px] * y[py - 1] * z[pz];
            if (pz > 0) gradient[2] += c * pz * x[px] * y[py] * z[pz - 1];
            if (px > 1) hessian[0, 0] += c * px * (px - 1) * x[px - 2] * y[py] * z[pz];
            if (py > 1) hessian[1, 1] += c * py * (py - 1) * x[px] * y[py - 2] * z[pz];
            if (pz > 1) hessian[2, 2] += c * pz * (pz - 1) * x[px] * y[py] * z[pz - 2];
            if (px > 0 && py > 0)
            {
                double mixed = c * px * py * x[px - 1] * y[py - 1] * z[pz];
                hessian[0, 1] += mixed; hessian[1, 0] += mixed;
            }
            if (px > 0 && pz > 0)
            {
                double mixed = c * px * pz * x[px - 1] * y[py] * z[pz - 1];
                hessian[0, 2] += mixed; hessian[2, 0] += mixed;
            }
            if (py > 0 && pz > 0)
            {
                double mixed = c * py * pz * x[px] * y[py - 1] * z[pz - 1];
                hessian[1, 2] += mixed; hessian[2, 1] += mixed;
            }
        }
        return value;
    }

    private double Evaluate(Vec3D p, int dx = 0, int dy = 0, int dz = 0)
    {
        double value = 0;
        for (int i = 0; i < _powers.Length; i++) value += _coefficients[i] * Term(_powers[i], p, dx, dy, dz);
        return value;
    }
    private static double Term((int X, int Y, int Z) power, Vec3D p, int dx = 0, int dy = 0, int dz = 0)
    {
        if (power.X < dx || power.Y < dy || power.Z < dz) return 0;
        double coefficient = 1;
        for (int i = 0; i < dx; i++) coefficient *= power.X - i;
        for (int i = 0; i < dy; i++) coefficient *= power.Y - i;
        for (int i = 0; i < dz; i++) coefficient *= power.Z - i;
        return coefficient * Math.Pow(p.X, power.X - dx) * Math.Pow(p.Y, power.Y - dy) * Math.Pow(p.Z, power.Z - dz);
    }
    private static bool Finite(Vec3D p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z);
}
