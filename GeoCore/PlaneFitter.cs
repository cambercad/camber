using System.Numerics;

namespace GeoCore
{
    public static class PlaneFitter
    {
        public static (Vec3D normal, Vec3D pointOnPlane) FitPlane(List<Vec3D> points)
        {
            if (points == null || points.Count < 3)
                throw new ArgumentException("Need at least 3 points to fit a plane");

            // --- 1. Compute centroid ---
            Vec3D centroid = new Vec3D(0);
            foreach (var p in points)
                centroid += p;
            centroid /= points.Count;

            // --- 2. Build covariance matrix ---
            double[,] cov = new double[3, 3];
            foreach (var p in points)
            {
                var d = p - centroid;
                cov[0, 0] += d.X * d.X; cov[0, 1] += d.X * d.Y; cov[0, 2] += d.X * d.Z;
                cov[1, 0] += d.Y * d.X; cov[1, 1] += d.Y * d.Y; cov[1, 2] += d.Y * d.Z;
                cov[2, 0] += d.Z * d.X; cov[2, 1] += d.Z * d.Y; cov[2, 2] += d.Z * d.Z;
            }

            // --- 3. Find eigenvector with smallest eigenvalue ---
            var normal = SmallestEigenVector(cov);

            return (normal.Normalized(), centroid);
        }

        public static (Vec3D normal, Vec3D pointOnPlane) FitPlane(List<Vec3D> points, IEnumerable<Tri> triangles)
        {
            if (triangles == null)
                throw new ArgumentException("Triangles cannot be null");

            // --- 1. Compute area-weighted centroid ---
            double totalArea = 0;
            Vec3D centroid = new Vec3D(0);
            int triangleCount = 0;

            foreach (var tri in triangles)
            {
                var a = points[tri.A];
                var b = points[tri.B];
                var c = points[tri.C];

                var area = 0.5 * Vec3DOps.Cross(b - a, c - a).Length();
                var triCentroid = (a + b + c) * (1.0 / 3.0);
                centroid += triCentroid * area;
                totalArea += area;
                triangleCount++;
            }

            if (triangleCount == 0)
                throw new ArgumentException("Need at least one triangle");

            centroid /= totalArea;

            // --- 2. Build area-weighted covariance of triangle vertices ---
            // Centroids alone are rank-deficient for a 2-triangle rectangle (collinear),
            // so PCA can return an in-plane vector instead of the face normal.
            double[,] cov = new double[3, 3];

            foreach (var tri in triangles)
            {
                var a = points[tri.A];
                var b = points[tri.B];
                var c = points[tri.C];

                var area = 0.5 * Vec3DOps.Cross(b - a, c - a).Length();
                double w = area / 3.0;
                Accumulate(cov, a - centroid, w);
                Accumulate(cov, b - centroid, w);
                Accumulate(cov, c - centroid, w);
            }

            // --- 3. Find eigenvector with smallest eigenvalue ---
            var normal = SmallestEigenVector(cov);

            return (normal.Normalized(), centroid);
        }

        private static void Accumulate(double[,] cov, Vec3D d, double w)
        {
            cov[0, 0] += w * d.X * d.X; cov[0, 1] += w * d.X * d.Y; cov[0, 2] += w * d.X * d.Z;
            cov[1, 0] += w * d.Y * d.X; cov[1, 1] += w * d.Y * d.Y; cov[1, 2] += w * d.Y * d.Z;
            cov[2, 0] += w * d.Z * d.X; cov[2, 1] += w * d.Z * d.Y; cov[2, 2] += w * d.Z * d.Z;
        }

        //Finds eigenvector of smallest eigenvalue of a symmetric 3x3 matrix
        public static Vec3D SmallestEigenVector(double[,] m, int maxIter = 50)
        {
            // Initialize eigenvectors as identity
            double[,] v = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
            double[,] a = (double[,])m.Clone();

            for (int iter = 0; iter < maxIter; iter++)
            {
                // Find largest off-diagonal element
                int p = 0, q = 1;
                double max = Math.Abs(a[0, 1]);
                if (Math.Abs(a[0, 2]) > max) { max = Math.Abs(a[0, 2]); p = 0; q = 2; }
                if (Math.Abs(a[1, 2]) > max) { max = Math.Abs(a[1, 2]); p = 1; q = 2; }
                if (max < 1e-12) break; // converged

                double app = a[p, p], aqq = a[q, q], apq = a[p, q];
                double phi = 0.5 * Math.Atan2(2 * apq, aqq - app);
                double c = Math.Cos(phi), s = Math.Sin(phi);

                // Rotate in p-q plane
                for (int k = 0; k < 3; k++)
                {
                    double aip = a[k, p], aiq = a[k, q];
                    a[k, p] = c * aip - s * aiq;
                    a[k, q] = s * aip + c * aiq;
                }
                for (int k = 0; k < 3; k++)
                {
                    double apk = a[p, k], aqk = a[q, k];
                    a[p, k] = c * apk - s * aqk;
                    a[q, k] = s * apk + c * aqk;
                }
                a[p, q] = a[q, p] = 0.0;

                // Update eigenvectors
                for (int k = 0; k < 3; k++)
                {
                    double vip = v[k, p], viq = v[k, q];
                    v[k, p] = c * vip - s * viq;
                    v[k, q] = s * vip + c * viq;
                }
            }

            // Find smallest diagonal value
            int minIndex = 0;
            if (a[1, 1] < a[minIndex, minIndex]) minIndex = 1;
            if (a[2, 2] < a[minIndex, minIndex]) minIndex = 2;

            return new Vec3D(
                v[0, minIndex],
                v[1, minIndex],
                v[2, minIndex]
            ).Normalized();
        }
    }

    // TODO: Not yet tested
    public static class CylinderFitter
    {
        private class Ray
        {
            public Vec3D Origin;
            public Vec3D Direction;

            public Ray(Vec3D origin, Vec3D direction)
            {
                Origin = origin;
                Direction = direction;
                Direction.Normalize();
            }
        }

        public static (Vec3D axisDir, Vec3D pointOnAxis, double radius) FitCylinder(List<Vec3D> points, List<Vec3D> normals, IEnumerable<Tri> triangles)
        {
            List<Ray> rays = new List<Ray>();
            HashSet<int> duplicateAvoider = new HashSet<int>();
            foreach(var tri in triangles)
            {
                if(!duplicateAvoider.Contains(tri.A))
                {
                    rays.Add(new Ray(points[tri.A], normals[tri.A]));
                    duplicateAvoider.Add(tri.A);
                }
                if (!duplicateAvoider.Contains(tri.B))
                {
                    rays.Add(new Ray(points[tri.B], normals[tri.B]));
                    duplicateAvoider.Add(tri.B);
                }
                if (!duplicateAvoider.Contains(tri.C))
                {
                    rays.Add(new Ray(points[tri.C], normals[tri.C]));
                    duplicateAvoider.Add(tri.C);
                }
            }

            if (rays == null || rays.Count < 2)
                throw new ArgumentException("At least two rays are required.");

            int n = rays.Count;

            // Step 1: Compute centroid of ray origins
            Vec3D centroid = new Vec3D(0);
            foreach (var r in rays)
                centroid += r.Origin;
            centroid /= n;

            // Step 2: Build 3x3 "moment" matrix = Σ (I - d dᵀ)
            double[,] A = new double[3, 3];
            foreach (var ray in rays)
            {
                var d = ray.Direction;
                A[0, 0] += 1 - d.X * d.X; A[0, 1] += -d.X * d.Y; A[0, 2] += -d.X * d.Z;
                A[1, 0] += -d.Y * d.X; A[1, 1] += 1 - d.Y * d.Y; A[1, 2] += -d.Y * d.Z;
                A[2, 0] += -d.Z * d.X; A[2, 1] += -d.Z * d.Y; A[2, 2] += 1 - d.Z * d.Z;
            }

            // Step 3: Find the eigenvector of A with the smallest eigenvalue
            // (using inverse power iteration — simple for 3x3)
            Vec3D v = PlaneFitter.SmallestEigenVector(A, 50);

            // Step 4: Solve for the best-fit point on the axis
            double[,] B = new double[3, 3];
            Vec3D b = new Vec3D(0);

            foreach (var ray in rays)
            {
                var d = ray.Direction;
                var o = ray.Origin;
                double dx = d.X, dy = d.Y, dz = d.Z;
                double[,] M = new double[3, 3]
                {
                { 1 - dx*dx, -dx*dy,   -dx*dz },
                { -dy*dx,    1 - dy*dy, -dy*dz },
                { -dz*dx,    -dz*dy,   1 - dz*dz }
                };

                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        B[r, c] += M[r, c];

                b += new Vec3D(
                    M[0, 0] * o.X + M[0, 1] * o.Y + M[0, 2] * o.Z,
                    M[1, 0] * o.X + M[1, 1] * o.Y + M[1, 2] * o.Z,
                    M[2, 0] * o.X + M[2, 1] * o.Y + M[2, 2] * o.Z
                );
            }


            Vec3D p = Solve3x3(B, b);

            double radius = 0.0;
            foreach (var ray in rays)
            {
                var o = ray.Origin;
                var d2 = GeometricAlgorithms.DistancePointLineSquared(o, p, v);
                radius += Math.Sqrt(d2);
            }
            radius /= rays.Count;

            return (v, p, radius);
        }


        // Basic 3×3 solver via Cramer’s rule
        private static Vec3D Solve3x3(double[,] A, Vec3D b)
        {
            double detA =
                A[0, 0] * (A[1, 1] * A[2, 2] - A[1, 2] * A[2, 1]) -
                A[0, 1] * (A[1, 0] * A[2, 2] - A[1, 2] * A[2, 0]) +
                A[0, 2] * (A[1, 0] * A[2, 1] - A[1, 1] * A[2, 0]);

            if (Math.Abs(detA) < 1e-8f)
                throw new InvalidOperationException("Matrix is singular or nearly singular.");

            double[,] A1 = (double[,])A.Clone();
            double[,] A2 = (double[,])A.Clone();
            double[,] A3 = (double[,])A.Clone();

            A1[0, 0] = b.X; A1[1, 0] = b.Y; A1[2, 0] = b.Z;
            A2[0, 1] = b.X; A2[1, 1] = b.Y; A2[2, 1] = b.Z;
            A3[0, 2] = b.X; A3[1, 2] = b.Y; A3[2, 2] = b.Z;

            double det1 =
                A1[0, 0] * (A1[1, 1] * A1[2, 2] - A1[1, 2] * A1[2, 1]) -
                A1[0, 1] * (A1[1, 0] * A1[2, 2] - A1[1, 2] * A1[2, 0]) +
                A1[0, 2] * (A1[1, 0] * A1[2, 1] - A1[1, 1] * A1[2, 0]);

            double det2 =
                A2[0, 0] * (A2[1, 1] * A2[2, 2] - A2[1, 2] * A2[2, 1]) -
                A2[0, 1] * (A2[1, 0] * A2[2, 2] - A2[1, 2] * A2[2, 0]) +
                A2[0, 2] * (A2[1, 0] * A2[2, 1] - A2[1, 1] * A2[2, 0]);

            double det3 =
                A3[0, 0] * (A3[1, 1] * A3[2, 2] - A3[1, 2] * A3[2, 1]) -
                A3[0, 1] * (A3[1, 0] * A3[2, 2] - A3[1, 2] * A3[2, 0]) +
                A3[0, 2] * (A3[1, 0] * A3[2, 1] - A3[1, 1] * A3[2, 0]);

            return new Vec3D(det1 / detA, det2 / detA, det3 / detA);
        }
    }

    // TODO: Not yet tested
    public static class SphereFitter
    {
        public static bool FitSphere(IList<Vec3D> points, out Vec3D center, out double radius)
        {
            center = new Vec3D(0, 0, 0);
            radius = 0.0;

            int n = points.Count;
            if (n < 4) return false; // need at least 4 points

            // Normal equations in absolute world coordinates lose the small
            // radius through cancellation on translated parts. Fit in a local,
            // unit-sized frame and transform the result back afterward.
            var origin = points[0];
            double scale = points.Max(p => (p - origin).Length());
            if (!(scale > 0) || !double.IsFinite(scale)) return false;

            Mat4D A = new Mat4D();
            Vec4D B = new Vec4D(0, 0, 0, 0);

            for (int i = 0; i < n; i++)
            {
                Vec3D p = (points[i] - origin) / scale;
                double s = p.LengthSquared();

                // Accumulate normal equations
                A.M11 += 4 * p.X * p.X;
                A.M12 += 4 * p.X * p.Y;
                A.M13 += 4 * p.X * p.Z;
                A.M14 += 2 * p.X;

                A.M22 += 4 * p.Y * p.Y;
                A.M23 += 4 * p.Y * p.Z;
                A.M24 += 2 * p.Y;

                A.M33 += 4 * p.Z * p.Z;
                A.M34 += 2 * p.Z;

                A.M44 += 1;

                B.X += 2 * p.X * s;
                B.Y += 2 * p.Y * s;
                B.Z += 2 * p.Z * s;
                B.W += s;
            }

            // Symmetrize A
            A.M21 = A.M12;
            A.M31 = A.M13;
            A.M32 = A.M23;
            A.M41 = A.M14;
            A.M42 = A.M24;
            A.M43 = A.M34;

            // Solve A * P = B using inverse
            if (!Mat4DOps.Inverse(A, out Mat4D AInv))
                return false;

            Vec4D P = AInv * B;

            var localCenter = new Vec3D(P.X, P.Y, P.Z);
            double c = P.W;
            radius = Math.Sqrt(Math.Max(0.0, localCenter.LengthSquared() + c)) * scale;
            center = origin + localCenter * scale;

            return true;
        }
    }
}
