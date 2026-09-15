//using GeoCore;
//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;

//namespace NURBS
//{
//    //Filling n-sided regions with G1 triangular Coons B-spline patches
//    public class PolygonCoons
//    {
//        // O denotes the zeroth-order derivative, or the boundary position
//        // v denotes the 1st-order crossboundary derivative
//        public static void Evaluate(List<BSplineCurve> D_o, List<BSplineCurve> D_v)
//        {
//            // j = (i+1)%n
//            // k = (i+n-1)%n

//            int n = D_o.Count;

//            Vec3D O = Vec3D.Zero;
//            for (int i = 0; i < n; ++i)
//            {
//                int j = (i + 1) % n;
//                int k = (i + n - 1) % n;

//                double l = 0.25 * ((D_o[j].Evaluate(1) - D_o[j].Evaluate(0)).Length + (D_o[k].Evaluate(1) + D_o[k].Evaluate(0)).Length);
//                O += l * D_v[i].Evaluate(0.5).Normalized() + D_o[i].Evaluate(0.5);
//            }
//            O = (1.0 / n) * O;


//            Vec3D N = Vec3D.Zero;
//            for (int i = 0; i < n; ++i)
//            {
//                int j = (i + 1) % n;
//                N += Vec3DOps.Cross(D_o[i].Evaluate(0.5) - O, D_o[j].Evaluate(0.5) - O);
//            }
//            N = (1.0 / n) * N;


//            Func<int, double, Vec3D> D_u = (id, u) => D_o[id].EvaluateDU(u);


//            Func<int, int, Vec3D> _D_ = (i, j) => (-D_u(i, 1).Normalized() + D_u(j, 0).Normalized()).Normalized();

//            Func<int, int, Vec3D> C_ = (i, j) => D_o[i].Evaluate(1);

//            Func<int, int, Vec3D> D_ = (i, j) => _D_(i, j) * Vec3DOps.Dot(_D_(i, j), O - C_(i, j));

//            Func<int, int, Vec3D> V_ = (i, j) => (O - C_(i, j)) - N * Vec3DOps.Dot(O - C_(i, j), N);

//            Func<int, int, Vec3D> I_1 = (i, j) => O;
//            Func<int, int, Vec3D> I_0 = (i, j) => D_o[i].Evaluate(1);

//            Func<int, int, Vec3D> Idash_0 = (i, j) => D_(i, j);


            
//        }


//        // i - knot span(from FindSpan() )
//        // u - parametric point 
//        // p - spline degree 
//        // U - knot sequence
//        public static void basisfun(int i, double u, int p, double[] U, double[] N)
//        {
//            int j, r;
//            double saved, temp;

//            // work space 
//            double[] left = new double[p + 1];
//            double[] right = new double[p + 1];

//            N[0] = 1.0;
//            for (j = 1; j <= p; j++)
//            {
//                left[j] = u - U[i + 1 - j];
//                right[j] = U[i + j] - u;
//                saved = 0.0;

//                for (r = 0; r < j; r++)
//                {
//                    temp = N[r] / (right[r + 1] + left[j - r]);
//                    N[r] = saved + right[r + 1] * temp;
//                    saved = left[j - r] * temp;
//                }

//                N[j] = saved;
//            }

//            //mxFree(left);
//            //mxFree(right);
//        }

//        int bspderiv(int d, double[] c, int mc, int nc, double[] k, int nk, double[] dc,
//                     double[] dk)
//        {
//            int ierr = 0;
//            int i, j;
//            double tmp;

//            // control points 
//            double[][] ctrl = vec2mat(c, mc, nc);

//            // control points of the derivative 
//            double[][] dctrl = vec2mat(dc, mc, nc - 1);

//            for (i = 0; i < nc - 1; i++)
//            {
//                tmp = d / (k[i + d + 1] - k[i + 1]);
//                for (j = 0; j < mc; j++)
//                {
//                    dctrl[i][j] = tmp * (ctrl[i + 1][j] - ctrl[i][j]);
//                }
//            }

//            j = 0;
//            for (i = 1; i < nk - 1; i++)
//                dk[j++] = k[i];

//            //freevec2mat(dctrl);
//            //freevec2mat(ctrl);

//            return ierr;
//        }

//        public static int findspan(int n, int p, double u, double[] U)
//        {
//            int low, high, mid;
//            // special case 
//            if (u == U[n + 1]) return (n);

//            // do binary search 
//            low = p;
//            high = n + 1;
//            mid = (low + high) >> 1;// / 2;
//            while (u < U[mid] || u >= U[mid + 1])
//            {
//                if (u < U[mid])
//                    high = mid;
//                else
//                    low = mid;

//                mid = (low + high) >> 1; // / 2;
//            }

//            return (mid);
//        }

//        public static int bspeval(int d, double[] c, int mc, int nc, double[] k, int nk, double[] u, int nu, double[] p)
//        {
//            int ierr = 0;
//            int i, s, tmp1, row, col;
//            double tmp2;

//            // Construct the control points 
//            double[][] ctrl = vec2mat(c, mc, nc);

//            // Construct the evaluated points 
//            double[][] pnt = vec2mat(p, mc, nu);

//            // space for the basis functions 
//            double[] N = new double[d + 1];

//            // for each parametric point i 
//            for (col = 0; col < nu; col++)
//            {
//                // find the span of u[col] 
//                s = findspan(nc - 1, d, u[col], k);
//                basisfun(s, u[col], d, k, N);

//                tmp1 = s - d;
//                for (row = 0; row < mc; row++)
//                {
//                    tmp2 = 0.0;
//                    for (i = 0; i <= d; i++)
//                        tmp2 += N[i] * ctrl[tmp1 + i][row];

//                    pnt[col][row] = tmp2;
//                }
//            }

//            //mxFree(N);
//            //freevec2mat(pnt);
//            //freevec2mat(ctrl);

//            return ierr;
//        }









//    }
//}
