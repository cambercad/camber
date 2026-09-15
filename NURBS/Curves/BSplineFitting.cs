using System;
using System.Collections.Generic;
using GeoCore;

namespace NURBS
{
    public class BSplineFitting
    {
        //The NURBS book page 68
        private static int FindSpan(int n, int p, double u, double[] U)
        {/* Determine the knot span index */
            /* Input: n,p,u,U */
            /* Return: the knot span index */
            if (u == U[n + 1]) return (n); /* Special case */
            int low = p; int high = n + 1; /* Do binary search */
            int mid = (low + high) / 2;
            while (u < U[mid] || u >= U[mid + 1])
            {
                if (u < U[mid]) high = mid;
                else low = mid;
                mid = (low + high) / 2;
            }
            return mid;
        }

        //The NURBS book page 70
        private static void BasisFuns(int i, double u, int p, double[] U, double[] N) //Length of N: p+1
        {
            /* Compute the nonvanishing basis functions */
            /* Input: i,u,p,U */
            /* Output: N */
            N[0] = 1.0;
            double[] left = new double[p + 1];
            double[] right = new double[p + 1];
            for (int j = 1; j <= p; ++j)
            {
                left[j] = u - U[i + 1 - j];
                right[j] = U[i + j] - u;
                double saved = 0.0;
                for (int r = 0; r < j; ++r)
                {
                    double temp = N[r] / (right[r + 1] + left[j - r]);
                    N[r] = saved + right[r + 1] * temp;
                    saved = left[j - r] * temp;
                }
                N[j] = saved;
            }
        }

        private unsafe static double OneBasisFun(int p, double[] U, int i, double u)
        {
            /* Compute the basis function Nip */
            /* Input: p,m,U,i,u */
            /* Output: Nip */
            int m = U.Length - 1;
            if ((i == 0 && u == U[0]) || (i == m - p - 1 && u == U[m])) /* Special cases */
                return 1.0;

            if (u < U[i] || u >= U[i + p + 1]) /* Local property */
                return 0.0;

            double* N = stackalloc double[p + 1];
            for (int j = 0; j <= p; ++j) /* Initialize zeroth-degree functs */
                if (u >= U[i + j] && u < U[i + j + 1]) N[j] = 1.0;
                else N[j] = 0.0;

            for (int k = 1; k <= p; ++k) /* Compute triangular table */
            {
                double saved;
                if (N[0] == 0.0) saved = 0.0;
                else saved = ((u - U[i]) * N[0]) / (U[i + k] - U[i]);
                for (int j = 0; j < p - k + 1; j++)
                {
                    double Uleft = U[i + j + 1];
                    double Uright = U[i + j + k + 1];
                    if (N[j + 1] == 0.0)
                    {
                        N[j] = saved; saved = 0.0;
                    }
                    else
                    {
                        double temp = N[j + 1] / (Uright - Uleft);
                        N[j] = saved + (Uright - u) * temp;
                        saved = (u - Uleft) * temp;
                    }
                }
            }
            return N[0];
        }


        public static BSplineCurve CubicSplineInterpolation(IList<Vec3D> points)
        {
            return CubicSplineInterpolation(points, 0.5 * (points[1] - points[0]), 0.5 * (points[points.Count - 1] - points[points.Count - 2]));
        }

        public static BSplineCurve CubicSplineInterpolation(IList<Vec3D> points, Vec3D derivativeStart, Vec3D derivativeEnd)
        {
            //const int degree = 3;
            int n = points.Count - 1;

            double[] knots = new double[n + 7];
            knots[0] = 0;
            knots[1] = 0;
            knots[2] = 0;
            knots[3] = 0;
            double s = 1.0 / (knots.Length - 7);
            for (int i = 4; i < knots.Length - 4; ++i)
                knots[i] = (i - 3) * s;
            knots[knots.Length - 4] = 1;
            knots[knots.Length - 3] = 1;
            knots[knots.Length - 2] = 1;
            knots[knots.Length - 1] = 1;

            Vec3D[] controlPoints = new Vec3D[n + 3];
            controlPoints[0] = points[0];
            controlPoints[1] = (knots[4] / 3.0) * derivativeStart + controlPoints[0];
            controlPoints[n + 2] = points[n];
            controlPoints[n + 1] = controlPoints[n + 2] - ((1 - knots[n + 2]) / 3.0) * derivativeEnd;
            SolveTridiagonal(n, points, knots, controlPoints);

            return new BSplineCurve(3, controlPoints, knots, false);
        }


        //Second derivative is not continuous but cures do not overshoot much
        public static BSplineCurve PiecewiseCubicInterpolation(IList<Vec3D> points, Vec3D derivativeStart, Vec3D derivativeEnd)
        {
            return null;
            //Vec3D[] dys = new Vec3D[points.Count - 1];
            //double[] dxs = new double[points.Count - 1];
            //Vec3D[] ms = new Vec3D[points.Count - 1];
            //// Get consecutive differences and slopes
            //for (int i = 0; i < points.Count - 1; i++)
            //{
            //    var dy = points[i + 1] - points[i];
            //    var dx = dy.Length; // xs[i + 1] - xs[i]; //TODO
            //    dxs[i] = dx; dys[i] = dy; ms[i] = dy / dx;
            //}

            //// Get degree-1 coefficients
            //var c1s = [ms[0]];
            //for (int i = 0; i < points.Count - 1; i++)
            //{
            //    var m = ms[i];
            //    var mNext = ms[i + 1];
            //    if (m * mNext <= 0)
            //    {
            //        c1s.push(0);
            //    }
            //    else
            //    {
            //        var dx_ = dxs[i];
            //        var dxNext = dxs[i + 1];
            //        var common = dx_ + dxNext;
            //        c1s.push(3 * common / ((common + dxNext) / m + (common + dx_) / mNext));
            //    }
            //}
            //c1s.push(ms[ms.length - 1]);





            //Vec3D[] controlPoints = new Vec3D[2 + 2 * (points.Count - 1)];
            //controlPoints[0] = points[0];
            //controlPoints[controlPoints.Length - 1] = points[points.Count - 1];

            //double[] uBar = new double[points.Count];
            //double uPrev = 0.0;
            //int baseIndex = 1;
            //uBar[0] = uPrev;
            //for (int i = 1; i < points.Count; ++i)
            //{
            //    uPrev += 3 * LocalCubic(baseIndex, controlPoints, points[i - 1], t0, points[i], t3);
            //    baseIndex += 2;
            //    uBar[i] = uPrev;
            //}

            //double[] knots = new double[2 * (points.Count - 2) + 8];
            //knots[0] = 0;
            //knots[1] = 0;
            //knots[2] = 0;
            //knots[3] = 0;
            //double scale = 1.0 / uBar[uBar.Length - 1];
            //for (int i = 4; i < knots.Length - 4; i += 2)
            //{
            //    int j = i / 2 + 1;
            //    double u = uBar[j] * scale;
            //    knots[i] = knots[i + 1] = u;
            //}
            //knots[knots.Length - 4] = 1;
            //knots[knots.Length - 3] = 1;
            //knots[knots.Length - 2] = 1;
            //knots[knots.Length - 1] = 1;

            //return new BSplineCurve(3, controlPoints, knots, false);
        }

        //https://de.wikipedia.org/wiki/Harmonisches_Mittel
        //https://blogs.mathworks.com/cleve/2012/07/16/splines-and-pchips/
        //https://en.wikipedia.org/wiki/Monotone_cubic_interpolation
        public static double MonotoneTangent(Vec3D prev, Vec3D center, Vec3D next)
        {
            return 0;
        }

        private static double ComputeAlpha(Vec3D P0, Vec3D T0, Vec3D P3, Vec3D T3)
        {
            double a = 16 - (T0 + T3).LengthSquared();
            double b = 12 * Vec3DOps.Dot(P3 - P0, T0 + T3);
            double c = -36 * (P3 - P0).LengthSquared();
            double alpha1, alpha2;
            int solvable = Algorithms.SolveQuadraticEquation(a, b, c, out alpha1, out alpha2);
            if (solvable != 0)
                throw new Exception();
            if (alpha1 < alpha2)
                throw new Exception();
            return alpha1;
        }

        private static double LocalCubic(int baseIndex, Vec3D[] controlPoints, Vec3D P0, Vec3D T0, Vec3D P3, Vec3D T3)
        {
            var alpha = ComputeAlpha(P0, T0, P3, T3);
            var P1 = P0 + (1.0 / 3.0) * alpha * T0;
            var P2 = P3 - (1.0 / 3.0) * alpha * T3;
            return (P1 - P0).Length();
        }

        //The NURBS book page 373
        private static void SolveTridiagonal(int n, IList<Vec3D> Q, double[] U, Vec3D[] P)
        { /* Solve tridiagonal system for C2 cubic spline */
            /* Input: n,Q,U,P[0],P[1],P[n+1],P[n+2]*/
            /* Output: P */

            Vec3D[] R = new Vec3D[n + 1];
            double[] dd = new double[n + 1];
            double[] abc = new double[4];

            for (int i = 3; i < n; ++i)
                R[i] = Q[i - 1];
            BasisFuns(4, U[4], 3, U, abc);
            double den = abc[1];
            P[2] = (Q[1] - abc[0] * P[1]) / den;
            for (int i = 3; i < n; ++i)
            {
                dd[i] = abc[2] / den;
                BasisFuns(i + 2, U[i + 2], 3, U, abc);
                den = abc[1] - abc[0] * dd[i];
                P[i] = (R[i] - abc[0] * P[i - 1]) / den;
            }
            dd[n] = abc[2] / den;
            BasisFuns(n + 2, U[n + 2], 3, U, abc);
            den = abc[1] - abc[0] * dd[n];
            P[n] = (Q[n - 1] - abc[2] * P[n + 1] - abc[0] * P[n - 1]) / den;
            for (int i = n - 1; i >= 2; --i)
                P[i] = P[i] - dd[i + 1] * P[i + 1];
        }





        //public static int KnotIndex(double w, int degree, double[] knots)
        //{
        //    //if (w < 0 || w > 1)
        //    //    throw new Exception("Parameter is not in the valid range");

        //    int k = knots.Length - 1;
        //    while (k >= degree)
        //    {
        //        if (knots[k] <= w)
        //            break;
        //        --k;
        //    }
        //    k = Math.Max(degree, Math.Min(k, knots.Length - degree - 2));
        //    return k;
        //}

        //public static int DeBoorIndex(double w, int degree, double[] knots)
        //{
        //    //if (w < 0 || w > 1)
        //    //    throw new Exception("Parameter is not in the valid range");

        //    int k = knots.Length - 1;
        //    while (k >= degree)
        //    {
        //        if (knots[k] <= w)
        //            break;
        //        --k;
        //    }
        //    k = Math.Max(degree, Math.Min(k, knots.Length - degree - 2));
        //    return k;
        //}

        //http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/de-Boor.html
        public unsafe static double EvaluateDeBoorBasis(double u, int degree, double[] knots)
        {
            int k = DeBoor.KnotIndex(u, degree, knots);
            double* evalBuffer = stackalloc double[degree + 1];
            int l = knots.Length - degree - 1;
            double* degreeZeroBasisFunctions = stackalloc double[l];
            for (int i = 0; i < l; ++i) degreeZeroBasisFunctions[i] = 0;
            degreeZeroBasisFunctions[k] = 1;
            return DeBoorBasis(degree, k, u, degreeZeroBasisFunctions, knots, evalBuffer);
        }

        public unsafe static double DeBoorBasis(int p, int k, double w, double* controlPoints, double[] knots, double* buffer)
        {
            int offset = k - p;
            for (int i = offset; i <= k; ++i)
                buffer[i - offset] = controlPoints[i];

            return DeBoorBasis(p, k, w, buffer, offset, knots);
        }

        public unsafe static double DeBoorBasis(int p, int k, double w, double* buffer, int offset, double[] knots)
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

        ////The NURBS book page 50
        //public double EvaluateBasisFunction(double u, int degreeP, double[] knotVectorU)
        //{
        //    int l = knotVectorU.Length;

        //    int knotVectorIndexI = 0;
        //    while (knotVectorIndexI < l - 1)
        //    {
        //        if (knotVectorU[knotVectorIndexI] <= u && u < knotVectorU[knotVectorIndexI++])
        //            break;
        //        ++knotVectorIndexI;
        //    }

        //    double[] buffer = new double[degreeP + 1]; //TODO: use stackalloc
        //    for (int i = 0; i <= degreeP; ++i)            
        //        buffer[i] = knotVectorU[i] <= u && u < knotVectorU[i + 1] ? 1 : 0;


        //    for (int j = 0; j < degreeP; ++j)
        //    {
        //        buffer[j] = buffer[j] + buffer[j + 1];
        //    }
        //}

        //private double Ni_0(double u, int i, double[] knotVectorU)
        //{
        //    return knotVectorU[i] <= u && u < knotVectorU[i + 1] ? 1 : 0;


        //}




        private static double Distance3D(Vec3D a, Vec3D b)
        {
            double dx = a.X - b.X; double dy = a.Y - b.Y; double dz = a.Z - b.Z;
            return dx * dx + dy * dy + dz * dz;
        }

        //TODO: Needs to be tested...
        //The NURBS book page 377-378
        //ALGORITHM A9.3
        private void SurfMeshParams(int n, int m, Vec3D[][] Q, double[] uk)//, dynamic vl)
        { /* Compute parameters for */
            /* global surface interpolation */
            /* Input: n,m,Q */
            /* Output: uk,vl */
            /* First get the uk */
            double[] cds = new double[Math.Max(n, m) + 1];
            int num = m + 1; /* number of nondegenerate rows */
            uk[0] = 0.0; uk[n] = 1.0;
            for (int k = 1; k < n; k++) uk[k] = 0.0;
            for (int l = 0; l <= m; l++)
            {
                double total = 0.0; /* total chord length of row */
                for (int k = 1; k <= n; k++)
                {
                    cds[k] = Distance3D(Q[k][l], Q[k - l][l]);
                    total = total + cds[k];
                }
                if (total == 0.0) num = num - 1;
                else
                {
                    double d = 0.0;
                    for (int k = 1; k < n; k++)
                    {
                        d = d + cds[k];
                        uk[k] = uk[k] + d / total;
                    }
                }
            }
            if (num == 0) throw new Exception();
            for (int k = 1; k < n; k++) uk[k] = uk[k] / num;
            /* Now do the same for vl */
        }



        ////The NURBS book page 380
        ////ALGORITHM A9.4
        //private void GlobalSurfInterp(int n, int m, Vector3d[][] Q, int p, int q, double[] U, double[] V, dynamic P)
        //{ /* Global surface interpolation */
        //    /* Input: n,m,Q,p,q */
        //    /* Output: U,V,P */
        //    SurfMeshParams(n, m, Q, uk, vl); /* get parameters */
        //    //Compute U using Eq.(9.8);
        //    //Compute V using Eq.(9.8);
        //    for (int l = 0; l <= m; l++)
        //    {
        //        //Do curve interpolation through Q[O] [l], ... ,Q[n] [l]j
        //        //This yields R[O] [l], ... ,R[n] [l]j
        //    }
        //    for (int i = 0; i <= n; i++)
        //    {
        //        //Do curve interpolation through R[i] [O], ... ,R[i] [m]j
        //        //This yields P[i] [0], ... ,P[i] [m]j
        //    }
        //}



    }
}
