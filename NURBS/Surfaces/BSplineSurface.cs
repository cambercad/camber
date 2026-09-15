using System;
using System.Collections.Generic;
using GeoCore;

namespace NURBS
{
    public partial class BSplineSurface : INurbsSurface
    {
        protected readonly int _degreeU;
        protected readonly int _degreeV;
        protected readonly Vec4D[][] _controlPoints; public Vec4D[][] ControlPoints { get { return _controlPoints; } } // = new List<List<Vector3d>>();//The inner lists are in u-Direction
        protected readonly double[] _knotsU; public double[] KnotsU { get { return _knotsU; } }
        protected readonly double[] _knotsV; public double[] KnotsV { get { return _knotsV; } }

        //Since there are not that many splines in memory, just buffer everything for fast evaluation
        protected BSplineSurface _dU; //First derivative wrt u
        protected BSplineSurface _dV; //First derivative wrt v
        protected BSplineSurface _ddU; //Second derivative wrt u
        protected BSplineSurface _ddV; //Second derivative wrt v
        protected BSplineSurface _dUdV; //Mixed derivative, first derived wrt u followed by another derivation wrt v (the order of u,v does not matter)

        protected Vec3D[][] _samples;
        protected double[] _sampleParametersU;
        protected double[] _sampleParametersV;
        protected GaussIntegrator _integratorU;
        protected GaussIntegrator _integratorV;
        protected int _uSpanHint = -1;
        protected int _vSpanHint = -1;

        public int DegreeU { get { return _degreeU; } }
        public int DegreeV { get { return _degreeV; } }

        public int NumKnotsU { get { return _knotsU.Length; } }
        public int NumControlPointsU { get { return _controlPoints.Length; } }
        public int NumKnotsV { get { return _knotsV.Length; } }
        public int NumControlPointsV { get { return _controlPoints[0].Length; } }

               
        public BSplineSurface(int degreeU, int degreeV, Vec3D[][] controlPoints, double[] knotsU, double[] knotsV)
            : this(degreeU, degreeV, controlPoints, Ones(controlPoints.Length, controlPoints[0].Length), knotsU, knotsV)
        { }
        public BSplineSurface(int degreeU, int degreeV, Vec3D[][] controlPoints, double[][] weights, double[] knotsU, double[] knotsV)
        {
            _degreeU = degreeU;
            _degreeV = degreeV;
            _controlPoints = ComputeControlPoints(controlPoints, weights);
            _knotsU = knotsU;
            _knotsV = knotsV;

            Init();
        }
        public BSplineSurface(int degreeU, int degreeV, Vec4D[][] controlPoints, double[] knotsU, double[] knotsV)
        {
            _degreeU = degreeU;
            _degreeV = degreeV;
            _controlPoints = controlPoints;
            _knotsU = knotsU;
            _knotsV = knotsV;

            Init();
        }

        public BSplineSurface(int degreeU, IList<BSplineCurve> vCurves)
            : this(degreeU, vCurves, BSplineCurve.Ones(vCurves.Count), BSplineCurve.UniformKnotVector(degreeU, vCurves.Count))
        { }
        public BSplineSurface(int degreeU, IList<BSplineCurve> vCurves, double[] knotsU)
            : this(degreeU, vCurves, BSplineCurve.Ones(vCurves.Count), knotsU)
        { }

        public BSplineSurface(int degreeU, IList<BSplineCurve> vCurves, double[] weights, double[] knotsU)
        {
            _degreeU = degreeU;
            _knotsU = knotsU;

            BSplineCurve c = vCurves[0];
            int numVPoints = c.ControlPoints.Length;
            _degreeV = c.Degree;
            for (int i = 1; i < vCurves.Count; ++i)
            {
                c = vCurves[i];
                if (c.ControlPoints.Length != numVPoints)
                    throw new Exception("Not all v-Curves have the same number of control-points");
                if (c.Degree != _degreeV)
                    throw new Exception("Not all v-Curves have the same degree");
                //TODO: Check knot vectors
            }

            _knotsV = vCurves[0].Knots;
            _controlPoints = new Vec4D[vCurves.Count][];
            for (int i = 0; i < vCurves.Count; ++i)
            {
                double w = weights[i];
                BSplineCurve curve = vCurves[i];
                Vec4D[] points = new Vec4D[curve.ControlPoints.Length];
                for (int j = 0; j < points.Length; ++j)
                    points[j] = w * curve.ControlPoints[j];
                _controlPoints[i] = points;
            }

            Init();
        }



        public BSplineSurface(int degreeU, int degreeV, List<List<Vec3D>> controlPoints, List<double> knotsU, List<double> knotsV)
            : this(degreeU, degreeV, controlPoints, OnesList(controlPoints.Count, controlPoints[0].Count), knotsU, knotsV)
        { }
        public BSplineSurface(int degreeU, int degreeV, List<List<Vec3D>> controlPoints, List<List<double>> weights, List<double> knotsU, List<double> knotsV)
        {
            _degreeU = degreeU;
            _degreeV = degreeV;
            int l = controlPoints.Count;
            _controlPoints = ComputeControlPoints(controlPoints, weights);
            _knotsU = knotsU.ToArray();
            _knotsV = knotsV.ToArray();

            //_bufferU = new Vector3d[_degreeU + 1];
            //_bufferV = new Vector3d[_degreeV + 1];

            /*string s = "";
            s += degreeU.ToString()+Environment.NewLine;
            s += degreeV.ToString() + Environment.NewLine;
            s += controlPoints.Count.ToString() + Environment.NewLine;
            for (int i = 0; i < controlPoints.Count; ++i)
            {
                for (int j = 0; j < controlPoints[i].Count; ++j)
                {
                    Vector3d v = controlPoints[i][j];
                    s += v.X.ToString() + " " + v.Y.ToString() + " " + v.Z.ToString() + ", ";
                }
                s += Environment.NewLine;
            }
            s += knotsU.Count.ToString() + Environment.NewLine;
            for (int i = 0; i < knotsU.Count; ++i)            
                s += knotsU[i].ToString() + Environment.NewLine;
            
            s += knotsV.Count.ToString() + Environment.NewLine;
            for (int i = 0; i < knotsV.Count; ++i)
                s += knotsV[i].ToString() + Environment.NewLine;*/

            Init();

            //Vector3d test = EvaluateDeBoor(0.5, 0.5);
        }

        private Vec4D[][] ComputeControlPoints(Vec3D[][] controlPoints, double[][] weights)
        {
            Vec4D[][] points = new Vec4D[controlPoints.Length][];
            for (int i = 0; i < controlPoints.Length; ++i)
            {
                Vec3D[] s = controlPoints[i];
                double[] we = weights[i];
                Vec4D[] p = new Vec4D[s.Length];
                for (int j = 0; j < s.Length; ++j)
                {
                    Vec3D v = s[j];
                    double w = we[j];
                    p[j] = new Vec4D(v.X * w, v.Y * w, v.Z * w, w);
                }
                points[i] = p;
            }
            return points;
        }

        private Vec4D[][] ComputeControlPoints(List<List<Vec3D>> controlPoints, List<List<double>> weights)
        {
            Vec4D[][] points = new Vec4D[controlPoints.Count][];
            for (int i = 0; i < controlPoints.Count; ++i)
            {
                List<Vec3D> s = controlPoints[i];
                List<double> we = weights[i];
                Vec4D[] p = new Vec4D[s.Count];
                for (int j = 0; j < s.Count; ++j)
                {
                    Vec3D v = s[j];
                    double w = we[j];
                    p[j] = new Vec4D(v.X * w, v.Y * w, v.Z * w, w);
                }
                points[i] = p;
            }
            return points;
        }

        //public BSplineSurface(int degreeU, int degreeV, Vector3d[][] controlPoints, double[] knotsU, double[] knotsV)
        //    : this(degreeU, degreeV, controlPoints, Ones(controlPoints.Length, controlPoints[0].Length), knotsU, knotsV, true)
        //{ }

        //public BSplineSurface(int degreeU, int degreeV, Vector3d[][] controlPoints, double[][] weights, double[] knotsU, double[] knotsV)
        //    : this(degreeU, degreeV, controlPoints, weights, knotsU, knotsV, true)
        //{ }

        private BSplineSurface(int degreeU, int degreeV, Vec4D[][] controlPoints, double[] knotsU, double[] knotsV, bool init)
        {
            _degreeU = degreeU;
            _degreeV = degreeV;
            _controlPoints = controlPoints;
            _knotsU = knotsU;
            _knotsV = knotsV;

            //_bufferU = new Vector3d[_degreeU + 1];
            //_bufferV = new Vector3d[_degreeV + 1];

            if (init)
                Init();
        }

        private static double[][] Ones(int rowCount, int colCount)
        {
            double[][] ones = new double[rowCount][];
            for (int i = 0; i < rowCount; ++i)
            {
                double[] row = new double[colCount];
                for (int j = 0; j < colCount; ++j)
                {
                    row[j] = 1;
                }
                ones[i] = row;
            }
            return ones;
        }

        private static List<List<double>> OnesList(int rowCount, int colCount)
        {
            List<List<double>> ones = new List<List<double>>(rowCount);
            for (int i = 0; i < rowCount; ++i)
            {
                List<double> row = new List<double>(colCount);
                for (int j = 0; j < colCount; ++j)
                {
                    row.Add(1);
                }
                ones.Add(row);
            }
            return ones;
        }

        public double GetKnotU(int index) { return _knotsU[index]; }
        public double GetKnotV(int index) { return _knotsV[index]; }

        /*public BSplineCurve ExtractSplineU(int vIndex)
        {
            Vector3d[] controlPoints = new Vector3d[_controlPoints.Length];
            double[] weights = new double[_controlPoints.Length];
            for (int i = 0; i < _controlPoints.Length; ++i)
            {
                controlPoints[i] = _controlPoints[i][vIndex];
                weights[i] = _weights[i][vIndex];
            }

            return new BSplineCurve(_degreeU, controlPoints, weights, _knotsU.Copy(), false); //TODO: might be closed...
        }*/


        /*public BSplineCurve ExtractSplineV(int uIndex)
        {
            return new BSplineCurve(_degreeV, _controlPoints[uIndex].Copy(), _weights[uIndex].Copy(), _knotsV.Copy(), false); //TODO: might be closed...
        }*/

        public Vec3D EvaluateDU(double u, double v)
        {
            return FirstDerivative(EvaluateH(u, v), _dU.EvaluateH(u, v));
            //return _dU.Evaluate(u, v); 
        }
        public Vec3D EvaluateDV(double u, double v)
        {
            return FirstDerivative(EvaluateH(u, v), _dV.EvaluateH(u, v));
            //return _dV.Evaluate(u, v); 
        }
        public Vec3D EvaluateDDU(double u, double v)
        {
            return SecondDerivative(EvaluateH(u, v), _dU.EvaluateH(u, v), _ddU.EvaluateH(u, v));
            //return _ddU.Evaluate(u, v);
        }
        public Vec3D EvaluateDDV(double u, double v)
        {
            return SecondDerivative(EvaluateH(u, v), _dV.EvaluateH(u, v), _ddV.EvaluateH(u, v));
            //return _ddV.Evaluate(u, v);
        }
        public Vec3D EvaluateDUDV(double u, double v)
        {
            Vec4D f = EvaluateH(u, v);
            Vec4D du = _dU.EvaluateH(u, v);
            Vec4D dv = _dV.EvaluateH(u, v);
            Vec4D duv = _dUdV.EvaluateH(u, v);

            double fw2 = f.W * f.W;
            return (XYZ(duv) * f.W + XYZ(du) * dv.W - (duv.W * XYZ(f) + du.W * XYZ(dv)) / fw2) - (2 * dv.W * (XYZ(du) * f.W - du.W * XYZ(f))) / (fw2 * f.W);
            //return (((XYZ(duv) * f.W + XYZ(du) * dv.W - (duv.W * XYZ(f) + du.W * XYZ(dv))) * f.W * f.W) - ((2 * f.W * dv.W) * (XYZ(du) * f.W - du.W * XYZ(f)))) / (f.W * f.W * f.W * f.W);

            //return _dUdV.Evaluate(u, v); 
        }


        private Vec3D FirstDerivative(Vec4D f, Vec4D d1) { return XYZ(d1) / f.W - XYZ(f) * d1.W / (f.W * f.W); }
        private Vec3D SecondDerivative(Vec4D f, Vec4D d1, Vec4D d2)
        {
            double fw2 = f.W * f.W;
            return ((XYZ(d2) / f.W - XYZ(f) * (d2.W / fw2)) - ((XYZ(d1) / fw2 - XYZ(f) * (d1.W / (fw2 * f.W))) * 2 * d1.W));
        }
        private Vec3D XYZ(Vec4D v) { return new Vec3D(v.X, v.Y, v.Z); }


        private double ArcLengthU(double v)
        {
            int l = _sampleParametersU.Length;
            double length = 0;
            for (int i = 1; i < l; ++i)
                length = length + _integratorU.Integrate(_sampleParametersU[i - 1], _sampleParametersU[i], delegate (double a) { return SpeedU(a, v); });
            return length;
        }

        private double ArcLengthV(double u)
        {
            int l = _sampleParametersV.Length;
            double length = 0;
            for (int i = 1; i < l; ++i)
                length = length + _integratorV.Integrate(_sampleParametersV[i - 1], _sampleParametersV[i], delegate (double a) { return SpeedV(u, a); });
            return length;
        }

        private double EstimateMaxCurvatureU()
        {
            return Math.Max(ComputeMaxCurvatureU(0), Math.Max(ComputeMaxCurvatureU(0.5), ComputeMaxCurvatureU(1)));
        }

        private double ComputeMaxCurvatureU(double v)
        {
            int l = _sampleParametersU.Length;
            double maxCurvatureSquared = 0;
            for (int i = 0; i < l; ++i)
            {
                Vec3D c = _ddU.Evaluate(_sampleParametersU[i], v);
                double lengthSquared = c.LengthSquared();
                if (lengthSquared > maxCurvatureSquared)
                    maxCurvatureSquared = lengthSquared;
            }
            return Math.Sqrt(maxCurvatureSquared);
        }

        private double EstimateMaxCurvatureV()
        {
            return Math.Max(ComputeMaxCurvatureV(0), Math.Max(ComputeMaxCurvatureV(0.5), ComputeMaxCurvatureV(1)));
        }

        private double ComputeMaxCurvatureV(double u)
        {
            int l = _sampleParametersV.Length;
            double maxCurvatureSquared = 0;
            for (int i = 0; i < l; ++i)
            {
                Vec3D c = _ddV.Evaluate(u, _sampleParametersV[i]);
                double lengthSquared = c.LengthSquared();
                if (lengthSquared > maxCurvatureSquared)
                    maxCurvatureSquared = lengthSquared;
            }
            return Math.Sqrt(maxCurvatureSquared);
        }

        private void Init()
        {
            _dU = DerivativeU();
            _dV = DerivativeV();
            _ddU = _dU.DerivativeU();
            _ddV = _dV.DerivativeV();
            _dUdV = _dU.DerivativeV();

            int lu = BSplineCurve.NumberOfDistinctKnots(_degreeU, _knotsU);
            int lv = BSplineCurve.NumberOfDistinctKnots(_degreeV, _knotsV);
            int numSplitsBetweenKnotsU = 2 * _degreeU; //Heuristics inspired by Shannon Theorem   
            int numSplitsBetweenKnotsV = 2 * _degreeV; //Heuristics inspired by Shannon Theorem  
            int numU = (lu - 1) * numSplitsBetweenKnotsU + 1;
            int numV = (lv - 1) * numSplitsBetweenKnotsV + 1;
            _samples = new Vec3D[numU][];
            //_arcLengthU = new double[numU][];
            //_arcLengthV = new double[numU][];
            for (int i = 0; i < numU; ++i)
            {
                _samples[i] = new Vec3D[numV];
                //_arcLengthU[i] = new double[numV];
                //_arcLengthV[i] = new double[numV];
            }
            _integratorU = GaussIntegrator.GetIntegrator(_degreeU);
            _integratorV = GaussIntegrator.GetIntegrator(_degreeV);
            _sampleParametersU = new double[numU];
            _sampleParametersV = new double[numV];

            double leftU, rightU;
            double leftV, rightV;
            int indexerU = 0;
            int jStart = 0;
            lu = _knotsU.Length - _degreeU - 1;
            lv = _knotsV.Length - _degreeV - 1;
            for (int i = _degreeU; i < lu; ++i)
            {
                leftU = _knotsU[i];
                rightU = _knotsU[i + 1];
                if (leftU != rightU)
                {
                    double spacingU = (rightU - leftU) / numSplitsBetweenKnotsU;
                    for (int j = jStart; j <= numSplitsBetweenKnotsU; ++j)
                    {
                        double u = leftU + j * spacingU;
                        int indexerV = 0;
                        int lStart = 0;
                        for (int k = _degreeV; k < lv; ++k)
                        {
                            leftV = _knotsV[k];
                            rightV = _knotsV[k + 1];
                            if (leftV != rightV)
                            {
                                double spacingV = (rightV - leftV) / numSplitsBetweenKnotsV;
                                for (int l = lStart; l <= numSplitsBetweenKnotsV; ++l)
                                {
                                    double v = leftV + l * spacingV;
                                    _samples[indexerU][indexerV] = Evaluate(u, v);
                                    _sampleParametersV[indexerV] = v;

                                    /*if (indexerU > 0)
                                        _arcLengthU[indexerU][indexerV] = _arcLengthU[indexerU - 1][indexerV] + _integratorU.Integrate(_sampleParametersU[indexerU - 1], u, delegate(double a) { return SpeedU(a, v); });
                                    else
                                        _arcLengthU[0][indexerV] = 0;

                                    if (indexerV > 0)
                                        _arcLengthV[indexerU][indexerV] = _arcLengthV[indexerU][indexerV - 1] + _integratorV.Integrate(_sampleParametersV[indexerV - 1], v, delegate(double a) { return SpeedV(u, a); });
                                    else
                                        _arcLengthV[indexerU][0] = 0;*/

                                    ++indexerV;
                                }
                                lStart = 1;
                            }
                        }
                        _sampleParametersU[indexerU] = u;
                        ++indexerU;
                    }
                    jStart = 1;
                }
            }

            /*_arcLengthMeasureU = 0; //Use maximum norm
            for (int i = 0; i < numV; ++i)
            {
                double l = _arcLengthU[numU - 1][i];
                if (l > _arcLengthMeasureU)
                    _arcLengthMeasureU = l;
            }
            _arcLengthMeasureV = 0; //Use maximum norm
            for (int i = 0; i < numU; ++i)
            {
                double l = _arcLengthV[i][numV - 1];
                if (l > _arcLengthMeasureV)
                    _arcLengthMeasureV = l;
            }*/
            //_arcLengthMeasureU = Math.Max(ArcLengthU(0), Math.Max(ArcLengthU(0.5), ArcLengthU(1)));
            //_arcLengthMeasureV = Math.Max(ArcLengthV(0), Math.Max(ArcLengthV(0.5), ArcLengthV(1)));
        }

        private double SpeedU(double u, double v)
        {
            Vec3D s = _dU.Evaluate(u, v);
            return Math.Sqrt(s.X * s.X + s.Y * s.Y + s.Z * s.Z);
        }

        private double SpeedV(double u, double v)
        {
            Vec3D s = _dV.Evaluate(u, v);
            return Math.Sqrt(s.X * s.X + s.Y * s.Y + s.Z * s.Z);
        }

        //http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
        private BSplineSurface DerivativeU()
        {
            int l = _controlPoints.Length - 1;
            int m = _controlPoints[0].Length;
            Vec4D[][] derivativeControlPoints;

            if (_degreeU == 0)
            {
                //TODO: Is this correct???
                //TODO: Valid parameter range might not always be 0...1
                derivativeControlPoints = new Vec4D[1][];
                derivativeControlPoints[0] = new Vec4D[m];

                for (int i = 0; i < m; ++i)
                    derivativeControlPoints[0][i] = new Vec4D(0, 0, 0, 0);

                return new BSplineSurface(0, _degreeV, derivativeControlPoints, new double[] { 0, 1 }, _knotsV, false);
                //throw new Exception("Derivative of this spline not computable using the code below");
            }

            derivativeControlPoints = new Vec4D[l][];
            for (int j = 0; j < l; ++j)
                derivativeControlPoints[j] = new Vec4D[m];

            for (int j = 0; j < m; ++j)
                for (int i = 0; i < l; ++i)
                {
                    double d = _degreeU / (_knotsU[i + _degreeU + 1] - _knotsU[i + 1]);
                    derivativeControlPoints[i][j] = d * (_controlPoints[i + 1][j] - _controlPoints[i][j]);
                }


            l = _knotsU.Length - 2;
            double[] knotsU = new double[l];
            Array.Copy(_knotsU, 1, knotsU, 0, l);

            return new BSplineSurface(_degreeU - 1, _degreeV, derivativeControlPoints, knotsU, _knotsV, false);
        }

        //http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
        private BSplineSurface DerivativeV()
        {
            int l = _controlPoints.Length;
            Vec4D[][] derivativeControlPoints = new Vec4D[l][];

            if (_degreeV == 0)
            {
                //TODO: Is this correct???
                //TODO: Valid parameter range might not always be 0...1

                for (int i = 0; i < l; ++i)
                    derivativeControlPoints[i] = new Vec4D[] { new Vec4D(0, 0, 0, 0) };

                return new BSplineSurface(_degreeU, 0, derivativeControlPoints, _knotsU, new double[] { 0, 1 }, false);
                //throw new Exception("Derivative of this spline not computable using the code below");
            }

            for (int i = 0; i < l; ++i)
                derivativeControlPoints[i] = BSplineCurve.DerivativeControlPoints(_degreeV, _controlPoints[i], _knotsV);

            l = _knotsV.Length - 2;
            double[] knotsV = new double[l];
            Array.Copy(_knotsV, 1, knotsV, 0, l);

            return new BSplineSurface(_degreeU, _degreeV - 1, derivativeControlPoints, _knotsU, knotsV, false);
        }

        //The NURBS book, page 232
        public bool GetParameterOfClosestPointOnSurface(Vec3D p, bool pOnSurface, out double u, out double v)
        {
            const double eps1 = 1e-3; //measure of Eucledian distance
            const double eps2 = 1e-6; //zero cosine measure

            const double eps1Squared = eps1 * eps1;
            const double eps2Squared = eps2 * eps2;


            //Find initial condition for following Newton Iteration         
            double minDistanceSquared = double.MaxValue;
            double dist;
            int lu = _sampleParametersU.Length;
            int lv = _sampleParametersV.Length;
            int indexU = -1;
            int indexV = -1;
            for (int i = 0; i < lu; ++i)
            {
                Vec3D[] buffer = _samples[i];
                for (int j = 0; j < lv; ++j)
                {
                    dist = (buffer[j] - p).LengthSquared();
                    if (dist < minDistanceSquared)
                    {
                        minDistanceSquared = dist;
                        indexU = i;
                        indexV = j;
                    }
                }
            }

            //Newton Iteration
            u = _sampleParametersU[indexU];
            v = _sampleParametersV[indexV];

            bool converged = false;

            double uNext;
            double vNext;

            int counter = 0;
            while (true)
            {
                Vec3D su = _dU.Evaluate(u, v);
                Vec3D sv = _dV.Evaluate(u, v);

                Vec3D r = Evaluate(u, v) - p;
                if (r.LengthSquared() <= eps1Squared)
                {
                    converged = true;
                    break;
                }

                //Kappa
                double f = Vec3DOps.Dot(r, su);
                double g = Vec3DOps.Dot(r, sv);
                double suLengthSquared = su.LengthSquared();
                double svLengthSquared = sv.LengthSquared();
                if (!pOnSurface)
                {
                    double rLengthSquared = r.LengthSquared();
                    double dotConditionU = (f * f) / (suLengthSquared * rLengthSquared);
                    double dotConditionV = (g * g) / (svLengthSquared * rLengthSquared);
                    if (dotConditionU <= eps2Squared && dotConditionV <= eps2Squared)
                    //if ((dotConditionU <= eps2Squared /*|| u == 0 || u == 1*/) && (dotConditionV <= eps2Squared /*|| v == 0 || v == 1*/)) //Experimental
                    {
                        converged = true;
                        break;
                    }
                }

                Vec3D suu = _ddU.Evaluate(u, v);
                Vec3D svv = _ddV.Evaluate(u, v);
                Vec3D suv = _dUdV.Evaluate(u, v);

                double j11 = suLengthSquared + Vec3DOps.Dot(r, suu);
                double j12 = Vec3DOps.Dot(su, sv) + Vec3DOps.Dot(r, suv);
                //double j21 = j12;
                double j22 = svLengthSquared + Vec3DOps.Dot(r, svv);

                //Invert j-matrix to get delta
                double scaling = 1.0 / (j11 * j22 - j12 * j12);
                double deltaU = scaling * (j12 * g - j22 * f); // == uNext - u
                double deltaV = scaling * (j12 * f - j11 * g); // == vNext - v

                uNext = u + 0.2 * deltaU;
                vNext = v + 0.2 * deltaV;

                //TODO: Valid parameter range might not always be 0...1
                /*if (uNext < 0)
                    uNext = 0;
                if (uNext > 1)
                    uNext = 1;
                if (vNext < 0)
                    vNext = 0;
                if (vNext > 1)
                    vNext = 1;*/

                if ((deltaU * su + deltaV * sv).LengthSquared() <= 0/*eps1Squared*/)
                {
                    converged = false;
                    break;
                }

                u = uNext;
                v = vNext;
                ++counter;
                if (counter > 1000)
                {
                    throw new Exception();
                }
            }
            return converged;
        }

        public Vec3D EvaluateNormal(double u, double v)
        {
            return EvaluateNormalFromH(EvaluateH(u, v), u, v);
        }

        public Vec3D EvaluatePointAndNormal(double u, double v, out Vec3D normal)
        {
            Vec4D f = EvaluateH(u, v);
            normal = EvaluateNormalFromH(f, u, v);
            return DeBoor.Project(f);
        }

        private Vec3D EvaluateNormalFromH(Vec4D f, double u, double v)
        {
            double step = 1e-8;
            Vec3D du = FirstDerivative(f, _dU.EvaluateH(u, v));
            double s = step;
            while (du.LengthSquared() < 1e-12)
            {
                double upper = v + s;
                double lower = v - s;
                if (upper <= 1 && lower >= 0)
                    du = 0.5 * (FirstDerivative(EvaluateH(u, upper), _dU.EvaluateH(u, upper))
                        + FirstDerivative(EvaluateH(u, lower), _dU.EvaluateH(u, lower)));
                else if (upper <= 1)
                    du = FirstDerivative(EvaluateH(u, upper), _dU.EvaluateH(u, upper));
                else if (lower >= 0)
                    du = FirstDerivative(EvaluateH(u, lower), _dU.EvaluateH(u, lower));
                s *= 2;
            }
            Vec3D dv = FirstDerivative(f, _dV.EvaluateH(u, v));
            s = step;
            while (dv.LengthSquared() < 1e-12)
            {
                double upper = u + s;
                double lower = u - s;
                if (upper <= 1 && lower >= 0)
                    dv = 0.5 * (FirstDerivative(EvaluateH(upper, v), _dU.EvaluateH(upper, v))
                        + FirstDerivative(EvaluateH(lower, v), _dU.EvaluateH(lower, v)));
                else if (upper <= 1)
                    dv = FirstDerivative(EvaluateH(upper, v), _dU.EvaluateH(upper, v));
                else if (lower >= 0)
                    dv = FirstDerivative(EvaluateH(lower, v), _dU.EvaluateH(lower, v));
                s *= 2;
            }
            return Vec3DOps.Cross(du, dv);
        }

        //http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/surface/bspline-de-boor.html
        public unsafe Vec3D Evaluate(double u, double v)
        {
            return DeBoor.Project(EvaluateH(u, v));
        }

        protected unsafe virtual Vec4D EvaluateH(double u, double v)
        {
            int uIndex = DeBoor.KnotIndexFromHint(ref _uSpanHint, u, _degreeU, _knotsU);
            int vIndex = DeBoor.KnotIndexFromHint(ref _vSpanHint, v, _degreeV, _knotsV);

            Vec4D* bufferU = stackalloc Vec4D[_degreeU + 1];
            Vec4D* bufferV = stackalloc Vec4D[_degreeV + 1];

            int offset = uIndex - _degreeU;
            for (int i = offset; i <= uIndex; ++i)
                bufferU[i - offset] = DeBoor.EvaluateRational(_degreeV, vIndex, v, _controlPoints[i], _knotsV, bufferV);

            return DeBoor.EvaluateRational(_degreeU, uIndex, u, bufferU, offset, _knotsU);
        }

        //TODO: Not tested yet
        public void SplitV(double vIsoValue, out BSplineSurface lower, out BSplineSurface upper, bool init = true)
        {
            int l = _controlPoints.Length;
            Vec4D[][] lowerPoints = new Vec4D[l][];
            Vec4D[][] upperPoints = new Vec4D[l][];
            BSplineCurve low = null;
            BSplineCurve up = null;
            for (int i = 0; i < l; ++i)
            {
                BSplineCurve.Split(vIsoValue, _degreeV, _knotsV, _controlPoints[i], false, out low, out up, false);
                /*int m = low.ControlPoints.Length;
                lowerPoints[i] = new Vector4d[m]; Array.Copy(low.ControlPoints, lowerPoints, m);
                m = up.ControlPoints.Length;
                upperPoints[i] = new Vector4d[m]; Array.Copy(up.ControlPoints, upperPoints, m);*/
                lowerPoints[i] = low.ControlPoints;
                upperPoints[i] = up.ControlPoints;
            }

            l = _knotsU.Length;
            double[] lowerKnotsU = new double[l]; Array.Copy(_knotsU, lowerKnotsU, l);
            double[] upperKnotsU = new double[l]; Array.Copy(_knotsU, upperKnotsU, l);
            lower = new BSplineSurface(_degreeU, _degreeV, lowerPoints, lowerKnotsU, low.Knots, init);
            upper = new BSplineSurface(_degreeU, _degreeV, upperPoints, upperKnotsU, up.Knots, init);
        }

        //TODO: Not tested yet
        public void SplitU(double uIsoValue, out BSplineSurface lower, out BSplineSurface upper, bool init = true)
        {
            int l = _controlPoints.Length;
            int m = _controlPoints[0].Length;

            Vec4D[] buffer = new Vec4D[l];
            BSplineCurve low, up;
            for (int j = 0; j < l; ++j)
                buffer[j] = _controlPoints[j][0];
            BSplineCurve.Split(uIsoValue, _degreeU, _knotsU, buffer, false, out low, out up, false);
            int lc = low.ControlPoints.Length;
            Vec4D[][] lowerPoints = new Vec4D[lc][];
            for (int i = 0; i < lc; ++i)
                lowerPoints[i] = new Vec4D[m];

            int uc = up.ControlPoints.Length;
            Vec4D[][] upperPoints = new Vec4D[uc][];
            for (int i = 0; i < uc; ++i)
                upperPoints[i] = new Vec4D[m];

            //double[] lowerKnotsU = low.Knots; // new double[low.Knots.Length]; Array.Copy(low.Knots, lowerKnotsU, lowerKnotsU.Length);
            //double[] upperKnotsU = new double[up.Knots.Length]; Array.Copy(up.Knots, upperKnotsU, upperKnotsU.Length);

            for (int j = 0; j < lc; ++j)
                lowerPoints[j][0] = low.ControlPoints[j];
            for (int j = 0; j < uc; ++j)
                upperPoints[j][0] = up.ControlPoints[j];

            for (int i = 1; i < m; ++i)
            {
                for (int j = 0; j < l; ++j)
                    buffer[j] = _controlPoints[j][i];
                BSplineCurve.Split(uIsoValue, _degreeU, _knotsU, buffer, false, out low, out up, false);

                for (int j = 0; j < lc; ++j)
                    lowerPoints[j][i] = low.ControlPoints[j];
                for (int j = 0; j < uc; ++j)
                    upperPoints[j][i] = up.ControlPoints[j];
            }

            l = _knotsV.Length;
            double[] lowerKnotsV = new double[l]; Array.Copy(_knotsV, lowerKnotsV, l);
            double[] upperKnotsV = new double[l]; Array.Copy(_knotsV, upperKnotsV, l);
            lower = new BSplineSurface(_degreeU, _degreeV, lowerPoints, low.Knots, lowerKnotsV, init);
            upper = new BSplineSurface(_degreeU, _degreeV, upperPoints, up.Knots, upperKnotsV, init);
        }


        /*private BSplineSurface DerivativeU()
        {
            int l = _controlPoints.Length - 1;
            int m = _controlPoints[0].Length;
            Vector4d[][] derivativeControlPoints;

            if (_degreeU == 0)
            {
                //TODO: Is this correct???
                //TODO: Valid parameter range might not always be 0...1
                derivativeControlPoints = new Vector4d[1][];
                derivativeControlPoints[0] = new Vector4d[m];

                for (int i = 0; i < m; ++i)
                    derivativeControlPoints[0][i] = new Vector4d(0, 0, 0, 0);

                return new BSplineSurface(0, _degreeV, derivativeControlPoints, new double[] { 0, 1 }, _knotsV, false);
                //throw new Exception("Derivative of this spline not computable using the code below");
            }

            derivativeControlPoints = new Vector4d[l][];
            for (int j = 0; j < l; ++j)
                derivativeControlPoints[j] = new Vector4d[m];

            for (int j = 0; j < m; ++j)
                for (int i = 0; i < l; ++i)
                {
                    double d = _degreeU / (_knotsU[i + _degreeU + 1] - _knotsU[i + 1]);
                    derivativeControlPoints[i][j] = d * (_controlPoints[i + 1][j] - _controlPoints[i][j]);
                }


            l = _knotsU.Length - 2;
            double[] knotsU = new double[l];
            Array.Copy(_knotsU, 1, knotsU, 0, l);

            return new BSplineSurface(_degreeU - 1, _degreeV, derivativeControlPoints, knotsU, _knotsV, false);
        }*/

        public TriangulatedGeometry Triangulate(List<List<Vec2D>> uvLoops, double maxDeviation = 0.1, double minimalBoundaryPointDistance = 1e-2, double tol = 1e-8, double eps = 1e-12)
        {
            /*if (_borderLoops.Count == 0)
            {
                CurveStrip cs = new CurveStrip();
                cs.Add(new BSplineLine(new Vector3d(0, 0, 0), new Vector3d(1, 0, 0)));
                cs.Add(new BSplineLine(new Vector3d(1, 0, 0), new Vector3d(1, 1, 0)));
                cs.Add(new BSplineLine(new Vector3d(1, 1, 0), new Vector3d(0, 1, 0)));
                cs.Add(new BSplineLine(new Vector3d(0, 1, 0), new Vector3d(0, 0, 0)));
                _borderLoops.Add(cs);
            }*/

            //TODO: Extract boundary as uv-polyline
            return AdaptiveSurfaceSplitter.TriangulateAdaptive(this, uvLoops, maxDeviation, minimalBoundaryPointDistance, tol, eps); // NurbsTessellator.Triangulate(this, maxDeviation, minimalBoundaryPointDistance, tol, eps);
        }

        //TODO: Not sure if this is correct for rational splines...
        //public void ApplyTransformation(Double4x4 trafo)
        //{
        //    for (int i = 0; i < _controlPoints.Length; ++i)
        //    {
        //        var arr = _controlPoints[i];

        //        for (int j = 0; j < arr.Length; ++j)
        //        {
        //            var cp = arr[j];
        //            var p = cp.Xyz * (1.0 / cp.W);

        //            p = trafo.TransformPoint(p);

        //            cp.Xyz = p * cp.W;
        //            arr[j] = cp;
        //        }
        //    }
        //}
    }
}
