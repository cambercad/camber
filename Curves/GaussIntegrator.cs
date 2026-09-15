namespace Curves
{

    public class GaussIntegrator
    {
        private int _order;
        private double[] _x;
        private double[] _w;

        private GaussIntegrator(int order)
        {
            _order = order;
            /*lock (_locations)
            {
                if (!_locations.ContainsKey(order))                
                    ComputePointsAndWeights(order);
                
                _x = _locations[order];
                _w = _weights[order];
            }*/
            ComputePointsAndWeights(order, out _x, out _w);
        }

        public double Integrate(double a, double b, Func<double, double> f)
        {
            double integral = 0;
            for (int i = 0; i < _order; ++i)
                integral += _w[i] * f((a * (1 - _x[i]) + b * (1 + _x[i])) * 0.5);
            return (b - a) * integral;
        }

        public static void Test()
        {
            const double eps = 1e-10;
            GaussIntegrator gi = new GaussIntegrator(6);
            double v = gi.Integrate(2.3, 2.598, delegate (double x) { return x * x * Math.Exp(-x + 3 * Math.Sin(x)); });
            if (Math.Abs(v - 1.0635743000767) > eps)
                throw new Exception();
            //TODO: extend

            gi = new GaussIntegrator(2);
        }


        #region static

        public static GaussIntegrator GetIntegrator(int order)
        {
            lock (_integrators)
            {
                GaussIntegrator gi;
                if (_integrators.TryGetValue(order, out gi))
                    return gi;
                else
                {
                    gi = new GaussIntegrator(order);
                    _integrators.Add(order, gi);
                    return gi;
                }
            }
        }

        //http://www.mathworks.com/matlabcentral/fileexchange/4540-legendre-gauss-quadrature-weights-and-nodes
        private static double _eps;
        //private static Dictionary<int, double[]> _locations;
        //private static Dictionary<int, double[]> _weights;
        private static Dictionary<int, GaussIntegrator> _integrators;
        static GaussIntegrator()
        {
            _eps = 1.0;
            do
            {
                _eps = _eps * 0.5;
                // If next epsilon yields 1, then break, because current
                // epsilon is the machine epsilon.
            }
            while (1.0 + (_eps * 0.5) != 1.0);
            //_locations = new Dictionary<int, double[]>();
            //_weights = new Dictionary<int, double[]>();
            _integrators = new Dictionary<int, GaussIntegrator>();
        }

        private static void ComputePointsAndWeights(int N, out double[] locations, out double[] weights)
        {
            switch (N)
            {
                case 0:
                    throw new Exception("This is not supported");
                    break;
                //http://de.wikipedia.org/wiki/Gauß-Quadratur
                case 1:
                    locations = new double[] { 0 };
                    weights = new double[] { 1 };
                    break;
                case 2:
                    double a = Math.Sqrt(1.0 / 3.0);
                    locations = new double[] { a, -a };
                    weights = new double[] { 0.5, 0.5 };
                    break;
                case 3:
                    double b = Math.Sqrt(3.0 / 5.0);
                    locations = new double[] { b, 0, -b };
                    weights = new double[] { 5.0 / 18.0, 8.0 / 18.0, 5.0 / 18.0 };
                    break;
                case 4:
                    double c = Math.Sqrt(3.0 / 7.0 + 2.0 / 7.0 * Math.Sqrt(6.0 / 5.0));
                    double d = Math.Sqrt(3.0 / 7.0 - 2.0 / 7.0 * Math.Sqrt(6.0 / 5.0));
                    locations = new double[] { c, d, -d, -c };
                    double e = (18 - Math.Sqrt(30)) / 72.0;
                    double f = (18 + Math.Sqrt(30)) / 72.0;
                    weights = new double[] { e, f, f, e };
                    break;
                case 5:
                    double g = 1.0 / 3.0 * Math.Sqrt(5 + 2 * Math.Sqrt(10.0 / 7.0));
                    double h = 1.0 / 3.0 * Math.Sqrt(5 - 2 * Math.Sqrt(10.0 / 7.0));
                    locations = new double[] { g, h, 0, -h, -g };
                    double u = (322 - 13 * Math.Sqrt(70)) / 1800.0;
                    double v = (322 + 13 * Math.Sqrt(70)) / 1800.0;
                    weights = new double[] { u, v, 128.0 / 225.0, v, u };
                    break;
                default:
                    // lgwt.m
                    //
                    // This script is for computing definite integrals using Legendre-Gauss 
                    // Quadrature. Computes the Legendre-Gauss nodes and weights  on an interval
                    // [a,b] with truncation order N
                    //
                    // Suppose you have a continuous function f(x) which is defined on [a,b]
                    // which you can evaluate at any x in [a,b]. Simply evaluate it at all of
                    // the values contained in the x vector to obtain a vector f. Then compute
                    // the definite integral using sum(f.*w);
                    //
                    // Written by Greg von Winckel - 02/25/2004

                    N = N - 1;
                    int N1 = N + 1; int N2 = N + 2;

                    int n = (N1 + 1) / 2;

                    // Initial guess
                    double[] y = new double[n];
                    for (int i = 0; i < n; ++i)
                        y[i] = Math.Cos((2 * i + 1) * Math.PI / (2 * N + 2)) + (0.27 / N1) * Math.Sin((Math.PI * ((2.0 * i) / (N1 - 1) - 1) * N) / N2);

                    // Legendre-Gauss Vandermonde Matrix

                    // Derivative of LGVM

                    // Compute the zeros of the N+1 Legendre Polynomial
                    // using the recursion relation and the Newton-Raphson method

                    //double[] y0= new double[N1];
                    double maxDelta = 1;
                    double[] L1 = new double[n];
                    double[] L2 = new double[n];

                    // Iterate until new points are uniformly within epsilon of old points
                    while (maxDelta > _eps)
                    {
                        for (int i = 0; i < n; ++i)
                        {
                            L1[i] = 1;
                            L2[i] = y[i];
                        }
                        for (int k = 2; k <= N1; ++k)
                        {
                            for (int i = 0; i < n; ++i)
                            {
                                double tmp = L2[i];
                                L2[i] = ((2 * k - 1) * y[i] * L2[i] - (k - 1) * L1[i]) / k;
                                L1[i] = tmp;
                            }
                        }
                        for (int i = 0; i < n; ++i)
                            L1[i] = (N2) * (L1[i] - y[i] * L2[i]) / (1 - y[i] * y[i]);

                        maxDelta = 0;
                        for (int i = 0; i < n; ++i)
                        {
                            double delta = L2[i] / L1[i];
                            y[i] = y[i] - delta;
                            if (delta < 0) delta = -delta;
                            if (delta > maxDelta)
                                maxDelta = delta;
                        }
                    }

                    // Linear map from[-1,1] to [a,b]
                    double[] x = new double[N1];
                    for (int i = 0; i < n; ++i)
                    {
                        x[i] = y[i];// (a * (1 - y[i]) + b * (1 + y[i])) * 0.5;
                        x[N1 - i - 1] = -y[i];// (a * (1 + y[i]) + b * (1 - y[i])) * 0.5;
                    }

                    // Compute the weights
                    double[] w = new double[N1];
                    for (int i = 0; i < n; ++i)
                    {
                        w[i] = (/*(b - a)*/(N2 * N2)) / ((N1 * N1) * (1 - y[i] * y[i]) * L1[i] * L1[i]);
                        w[N1 - i - 1] = w[i];
                    }

                    locations = x;
                    weights = w;
                    break;
            }
        }
        #endregion
    }
}
