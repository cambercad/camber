using System;
using System.Collections.Generic;

namespace GeoCore
{
    /// <summary>
    /// Axis-from-normals cylinder fit. Used at import / AutoUV when the
    /// tessellation is already a cylinder — not a general surface fitter.
    /// </summary>
    public static class CylinderAxisFit
    {
        public static bool TryFit(
            IList<Vec3D> points,
            IList<Vec3D> normals,
            HashSet<int> used,
            out Vec3D axis,
            out Vec3D pointOnAxis,
            out double radius,
            out double minHeight,
            out double maxHeight)
        {
            axis = new Vec3D(0, 0, 1);
            pointOnAxis = new Vec3D(0, 0, 0);
            radius = 0;
            minHeight = 0;
            maxHeight = 0;
            if (points == null || normals == null || used == null || used.Count < 8)
                return false;

            double[,] cov = new double[3, 3];
            var centroid = new Vec3D(0);
            int n = 0;
            foreach (int i in used)
            {
                if (i < 0 || i >= points.Count || i >= normals.Count)
                    continue;
                var d = normals[i];
                double len = d.Length();
                if (len < 1e-12)
                    continue;
                d *= 1.0 / len;
                cov[0, 0] += d.X * d.X; cov[0, 1] += d.X * d.Y; cov[0, 2] += d.X * d.Z;
                cov[1, 0] += d.Y * d.X; cov[1, 1] += d.Y * d.Y; cov[1, 2] += d.Y * d.Z;
                cov[2, 0] += d.Z * d.X; cov[2, 1] += d.Z * d.Y; cov[2, 2] += d.Z * d.Z;
                centroid += points[i];
                n++;
            }
            if (n < 8)
                return false;
            centroid *= 1.0 / n;
            axis = PlaneFitter.SmallestEigenVector(cov, 50);
            if (axis.LengthSquared() < 1e-16)
                return false;
            axis = axis.Normalized();
            pointOnAxis = centroid;

            minHeight = double.MaxValue;
            maxHeight = double.MinValue;
            foreach (int i in used)
            {
                var d = points[i] - centroid;
                double h = Vec3DOps.Dot(d, axis);
                radius += (d - axis * h).Length();
                if (h < minHeight) minHeight = h;
                if (h > maxHeight) maxHeight = h;
            }
            radius /= n;
            if (radius < 1e-9)
                return false;

            double maxErr = 0;
            foreach (int i in used)
            {
                var d = points[i] - centroid;
                double e = Math.Abs((d - axis * Vec3DOps.Dot(d, axis)).Length() - radius);
                if (e > maxErr)
                    maxErr = e;
            }
            if (maxErr > Math.Max(1e-4, radius * 1e-3))
                return false;
            return maxHeight - minHeight >= Math.Max(1e-4, radius * 1e-3);
        }
    }
}
