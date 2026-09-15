using System;
using GeoCore;

namespace NURBS
{
    /// <summary>
    /// Scattered least-squares B-spline surface fit. Control-net size is independent of
    /// the sample count (not one node per mesh vertex).
    /// </summary>
    public static class BSplineSurfaceFitter
    {
        public static BSplineSurface FitScattered(
            IReadOnlyList<Vec2D> uv,
            IReadOnlyList<Vec3D> points,
            int numU,
            int numV,
            int degreeU,
            int degreeV)
        {
            if (uv == null || points == null || uv.Count != points.Count || uv.Count == 0)
                throw new ArgumentException("UV and points must be non-empty and the same length.");
            if (numU < degreeU + 1 || numV < degreeV + 1)
                throw new ArgumentException("Need at least degree+1 control points per direction.");

            double[] knotsU = BSplineCurve.UniformKnotVector(degreeU, numU);
            double[] knotsV = BSplineCurve.UniformKnotVector(degreeV, numV);
            int nCp = numU * numV;
            int nSamp = uv.Count;

            var ata = new double[nCp * nCp];
            var atx = new double[nCp];
            var aty = new double[nCp];
            var atz = new double[nCp];
            var nu = new double[degreeU + 1];
            var nv = new double[degreeV + 1];
            var left = new double[Math.Max(degreeU, degreeV) + 1];
            var right = new double[Math.Max(degreeU, degreeV) + 1];

            for (int s = 0; s < nSamp; s++)
            {
                double u = Clamp01(uv[s].X);
                double v = Clamp01(uv[s].Y);
                int spanU = FindSpan(numU - 1, degreeU, u, knotsU);
                int spanV = FindSpan(numV - 1, degreeV, v, knotsV);
                BasisFuns(spanU, u, degreeU, knotsU, nu, left, right);
                BasisFuns(spanV, v, degreeV, knotsV, nv, left, right);

                var p = points[s];
                int i0 = spanU - degreeU;
                int j0 = spanV - degreeV;
                for (int a = 0; a <= degreeU; a++)
                {
                    int i = i0 + a;
                    for (int b = 0; b <= degreeV; b++)
                    {
                        int j = j0 + b;
                        int col = i * numV + j;
                        double w = nu[a] * nv[b];
                        atx[col] += w * p.X;
                        aty[col] += w * p.Y;
                        atz[col] += w * p.Z;

                        for (int c = 0; c <= degreeU; c++)
                        {
                            int i2 = i0 + c;
                            for (int d = 0; d <= degreeV; d++)
                            {
                                int j2 = j0 + d;
                                int row = i2 * numV + j2;
                                ata[row * nCp + col] += (nu[c] * nv[d]) * w;
                            }
                        }
                    }
                }
            }

            double ridge = 1e-10;
            for (int i = 0; i < nCp; i++)
                ata[i * nCp + i] += ridge;

            if (!FactorCholesky(ata, nCp))
                return null;
            SolveCholesky(ata, nCp, atx);
            SolveCholesky(ata, nCp, aty);
            SolveCholesky(ata, nCp, atz);

            var net = new Vec3D[numU][];
            for (int i = 0; i < numU; i++)
            {
                net[i] = new Vec3D[numV];
                for (int j = 0; j < numV; j++)
                {
                    int k = i * numV + j;
                    net[i][j] = new Vec3D(atx[k], aty[k], atz[k]);
                }
            }

            return new BSplineSurface(degreeU, degreeV, net, knotsU, knotsV);
        }

        public static double MaxError(
            BSplineSurface surface,
            IReadOnlyList<Vec2D> uv,
            IReadOnlyList<Vec3D> points)
        {
            double max = 0;
            for (int i = 0; i < points.Count; i++)
            {
                var s = surface.Evaluate(Clamp01(uv[i].X), Clamp01(uv[i].Y));
                double e = (s - points[i]).Length();
                if (e > max)
                    max = e;
            }
            return max;
        }

        private static double Clamp01(double t)
        {
            if (t < 0.0) return 0.0;
            if (t > 1.0) return 1.0;
            return t;
        }

        private static int FindSpan(int n, int p, double u, double[] U)
        {
            if (u >= U[n + 1])
                return n;
            if (u <= U[p])
                return p;
            int low = p;
            int high = n + 1;
            int mid = (low + high) / 2;
            while (u < U[mid] || u >= U[mid + 1])
            {
                if (u < U[mid])
                    high = mid;
                else
                    low = mid;
                mid = (low + high) / 2;
            }
            return mid;
        }

        private static void BasisFuns(int i, double u, int p, double[] U, double[] N, double[] left, double[] right)
        {
            N[0] = 1.0;
            for (int j = 1; j <= p; j++)
            {
                left[j] = u - U[i + 1 - j];
                right[j] = U[i + j] - u;
                double saved = 0.0;
                for (int r = 0; r < j; r++)
                {
                    double denom = right[r + 1] + left[j - r];
                    double temp = Math.Abs(denom) < 1e-30 ? 0.0 : N[r] / denom;
                    N[r] = saved + right[r + 1] * temp;
                    saved = left[j - r] * temp;
                }
                N[j] = saved;
            }
        }

        private static bool FactorCholesky(double[] a, int n)
        {
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    double sum = a[i * n + j];
                    for (int k = 0; k < j; k++)
                        sum -= a[i * n + k] * a[j * n + k];
                    if (i == j)
                    {
                        if (sum <= 1e-18)
                            return false;
                        a[i * n + j] = Math.Sqrt(sum);
                    }
                    else
                        a[i * n + j] = sum / a[j * n + j];
                }
                for (int j = i + 1; j < n; j++)
                    a[i * n + j] = 0;
            }
            return true;
        }

        private static void SolveCholesky(double[] l, int n, double[] b)
        {
            for (int i = 0; i < n; i++)
            {
                double sum = b[i];
                int row = i * n;
                for (int k = 0; k < i; k++)
                    sum -= l[row + k] * b[k];
                b[i] = sum / l[row + i];
            }
            for (int i = n - 1; i >= 0; i--)
            {
                double sum = b[i];
                for (int k = i + 1; k < n; k++)
                    sum -= l[k * n + i] * b[k];
                b[i] = sum / l[i * n + i];
            }
        }
    }
}
