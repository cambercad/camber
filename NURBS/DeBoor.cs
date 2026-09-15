using System;
using GeoCore;

namespace NURBS
{
    public static class DeBoor
    {
        public static int KnotIndex(double w, int degree, double[] knots)
        {
            // Largest k in [degree, length-1] with knots[k] <= w, then clamp to valid span index.
            int lo = degree;
            int hi = knots.Length - 1;
            int k = degree - 1;
            while (lo <= hi)
            {
                int mid = (lo + hi) >> 1;
                if (knots[mid] <= w)
                {
                    k = mid;
                    lo = mid + 1;
                }
                else
                {
                    hi = mid - 1;
                }
            }
            return Math.Max(degree, Math.Min(k, knots.Length - degree - 2));
        }

        /// <summary>
        /// Like <see cref="KnotIndex"/> but reuses <paramref name="hint"/> when the parameter
        /// stays in/near the previous knot span (typical for sequential grid evaluation).
        /// </summary>
        public static int KnotIndexFromHint(ref int hint, double w, int degree, double[] knots)
        {
            int min = degree;
            int max = knots.Length - degree - 2;
            if (max < min)
                return min;

            if (hint < min || hint > max)
                return hint = KnotIndex(w, degree, knots);

            if (w >= knots[hint + 1])
            {
                while (hint < max && w >= knots[hint + 1])
                    hint++;
            }
            else if (w < knots[hint])
            {
                while (hint > min && w < knots[hint])
                    hint--;
            }
            return hint;
        }

        public static Vec3D Evaluate(int degree, double parameter, Vec3D[] controlPoints, double[] knots)
        {
            return Evaluate(degree, KnotIndex(parameter, degree, knots), parameter, controlPoints, knots);
        }

        public unsafe static Vec3D Evaluate(int p, int k, double w, Vec3D[] controlPoints, double[] knots)
        {
            Vec3D* buffer = stackalloc Vec3D[p + 1];

            int offset = k - p;
            for (int i = offset; i <= k; ++i)
                buffer[i - offset] = controlPoints[i];

            return Evaluate(p, k, w, buffer, offset, knots);
        }

        public unsafe static Vec3D Evaluate(int p, int k, double w, Vec3D* buffer, int offset, double[] knots)
        {
            for (int r = 1; r <= p; ++r)
            {
                for (int i = k; i >= k - p + r; --i)
                {
                    double a = (w - knots[i]) / (knots[i + p - r + 1] - knots[i]);
                    buffer[i - offset] = (1 - a) * buffer[i - offset - 1] + a * buffer[i - offset];
                }
            }
            return buffer[k - offset];
        }
      

        public static Vec3D Project(Vec4D v)
        {
            if (v.W != 0)
            {
                double s = 1.0 / v.W;
                return new Vec3D(v.X * s, v.Y * s, v.Z * s);
            }
            else
                return new Vec3D(v.X, v.Y, v.Z);
        }

        /*public static Vector3d Project(Vector4d v)
        {
            if (v.W > -1e-16 && v.W > 1e-16)
                throw new DivideByZeroException();
            double s = 1.0 / v.W;
            return new Vector3d(v.X * s, v.Y * s, v.Z * s);
        }*/

        public unsafe static Vec4D EvaluateRational(int degree, double parameter, Vec4D[] controlPoints, double[] knots)
        {
            int k = KnotIndex(parameter, degree, knots);
            Vec4D* buffer = stackalloc Vec4D[degree + 1];
            return EvaluateRational(degree, k, parameter, controlPoints, knots, buffer);
        }

        public unsafe static Vec4D EvaluateRational(int degree, double parameter, Vec4D[] controlPoints, double[] knots, ref int knotHint)
        {
            int k = KnotIndexFromHint(ref knotHint, parameter, degree, knots);
            Vec4D* buffer = stackalloc Vec4D[degree + 1];
            return EvaluateRational(degree, k, parameter, controlPoints, knots, buffer);
        }

        public unsafe static Vec4D EvaluateRational(int p, int k, double w, Vec4D[] controlPoints, double[] knots, Vec4D* buffer)
        {
            int offset = k - p;
            for (int i = offset; i <= k; ++i)
                buffer[i - offset] = controlPoints[i];

            return EvaluateRational(p, k, w, buffer, offset, knots);
        }

        public unsafe static Vec4D EvaluateRational(int p, int k, double w, Vec4D* buffer, int offset, double[] knots)
        {
            for (int r = 1; r <= p; ++r)
            {
                for (int i = k; i >= k - p + r; --i)
                {
                    double a = (w - knots[i]) / (knots[i + p - r + 1] - knots[i]);
                    buffer[i - offset] = (1 - a) * buffer[i - offset - 1] + a * buffer[i - offset];
                }
            }
            return buffer[k - offset];
        }
    }

    //public static class DeBoor2
    //{
    //    public static int KnotIndex(double w, int degree, double[] knots)
    //    {
    //        //if (w < 0 || w > 1)
    //        //    throw new Exception("Parameter is not in the valid range");

    //        int k = knots.Length - 1;
    //        while (k >= degree)
    //        {
    //            if (knots[k] <= w)
    //                break;
    //            --k;
    //        }
    //        k = Math.Max(degree, Math.Min(k, knots.Length - degree - 2));
    //        return k;
    //    }

    //    public static Vector3d Evaluate(int degree, double parameter, Vector3d[] controlPoints, double[] knots)
    //    {
    //        return Evaluate(degree, KnotIndex(parameter, degree, knots), parameter, controlPoints, knots);
    //    }

    //    public unsafe static Vector3d Evaluate(int p, int k, double w, Vector3d[] controlPoints, double[] knots)
    //    {
    //        Vector3d* buffer = stackalloc Vector3d[p + 1];

    //        int offset = k - p;
    //        for (int i = offset; i <= k; ++i)
    //            buffer[i - offset] = controlPoints[i];

    //        return Evaluate(p, k, w, buffer, offset, knots);
    //    }

    //    public unsafe static Vector3d Evaluate(int p, int k, double w, Vector3d* buffer, int offset, double[] knots)
    //    {
    //        for (int r = 1; r <= p; ++r)
    //        {
    //            for (int i = k; i >= k - p + r; --i)
    //            {
    //                double a = (w - knots[i]) / (knots[i + p - r + 1] - knots[i]);
    //                buffer[i - offset] = (1 - a) * buffer[i - offset - 1] + a * buffer[i - offset];
    //            }
    //        }
    //        return buffer[k - offset];
    //    }

    //    public static Vector3d EvaluateRational(int degree, double parameter, Vector4d[] controlPoints, double[] knots)
    //    {
    //        Vector3d[] points = new Vector3d[controlPoints.Length];
    //        double[] weights = new double[controlPoints.Length];
    //        for (int i = 0; i < controlPoints.Length; ++i)
    //        {
    //            Vector4d v = controlPoints[i];
    //            points[i] = new Vector3d(v.X , v.Y , v.Z );
    //            weights[i] = v.W;
    //        }

    //        double weight;
    //        return EvaluateRational(degree, KnotIndex(parameter, degree, knots), parameter, points, controlPoints, weights, knots, out weight);
    //    }

    //    public unsafe static Vector3d EvaluateRational(int p, int k, double w, Vector3d[] controlPoints, Vector4d[] test, double[] weights, double[] knots, out double weight)
    //    {
    //        Vector3d* buffer = stackalloc Vector3d[p + 1];
    //        double* weightBuffer = stackalloc double[p + 1];

    //        Vector4d* tmp = stackalloc Vector4d[p + 1];

    //        int offset = k - p;
    //        for (int i = offset; i <= k; ++i)
    //        {
    //            buffer[i - offset] = controlPoints[i]; //Scaling with weight-factors
    //            weightBuffer[i - offset] = weights[i];
    //        }

    //        weight = EvaluateRational(p, k, w, weightBuffer, offset, knots);
    //        return Evaluate(p, k, w, buffer, offset, knots) / weight;
    //    }

    //    public unsafe static double EvaluateRational(int p, int k, double w, double* buffer, int offset, double[] knots)
    //    {
    //        for (int r = 1; r <= p; ++r)
    //        {
    //            for (int i = k; i >= k - p + r; --i)
    //            {
    //                double a = (w - knots[i]) / (knots[i + p - r + 1] - knots[i]);
    //                buffer[i - offset] = (1 - a) * buffer[i - offset - 1] + a * buffer[i - offset];
    //            }
    //        }
    //        return buffer[k - offset];
    //    }

    //    public unsafe static Vector4d EvaluateRational(int p, int k, double w, Vector4d* buffer, int offset, double[] knots)
    //    {
    //        for (int r = 1; r <= p; ++r)
    //        {
    //            for (int i = k; i >= k - p + r; --i)
    //            {
    //                double a = (w - knots[i]) / (knots[i + p - r + 1] - knots[i]);
    //                buffer[i - offset] = (1 - a) * buffer[i - offset - 1] + a * buffer[i - offset];
    //            }
    //        }
    //        return buffer[k - offset];
    //    }
    //}
}
