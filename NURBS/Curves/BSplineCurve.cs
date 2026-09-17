using System;
using System.Collections.Generic;
using GeoCore;


namespace NURBS
{
    /*public enum BSplineCurveType
    {
        General,
        Line,
        Circle,
        Arc
    }*/

    
    public class BSplineCurve //: IUniformCurve3, IArcLengthCurve3
    {
        
        protected readonly int _degree; public int Degree { get { return _degree; } } //p   
        
        protected readonly Vec4D[] _controlPoints; public Vec4D[] ControlPoints { get { return _controlPoints; } } //d
        
        protected readonly double[] _knots; public double[] Knots { get { return _knots; } }
        
        protected readonly bool _closedCurve; public bool ClosedCurve { get { return _closedCurve; } } //Just additional information, means that some of the knots and control points from the beginning of their lists are repeated at the end

        //Since there are not that many splines in memory, just buffer everything for fast evaluation
        //Initialization of all acceleration quantities is done with lazy evaluation - makes storing splines easier
        protected BSplineCurve _firstPseudoDerivative;
        protected BSplineCurve _secondPseudoDerivative;
        protected Vec3D[] _samples;
        protected double[] _sampleParameters;
        protected double[] _arcLength;
        protected GaussIntegrator _integrator;
        protected double _totalArcLength;
        protected int _spanHint = -1;

        //private double _maxCurvature = -1;

        public virtual Vec3D Start { get { return EvaluateUniform(0); } } //TODO: Override in derived classes
        public virtual Vec3D End { get { return EvaluateUniform(1); } }

        public Vec3D EvaluateUniformDU(double u)
        {
            InitIfRequired();

            Vec4D f = EvaluateH(u);
            Vec4D d1 = _firstPseudoDerivative.EvaluateH(u);

            return XYZ(d1) / f.W - XYZ(f) * (d1.W / (f.W * f.W));
            //    return (XYZ(d2) * f.W * f.W - 2 * XYZ(d1) * f.W * d1.W + XYZ(f) * (2 * d1.W * d1.W - f.W * d2.W)) / (f.W * f.W * f.W);
            //return _firstPseudoDerivative.Evaluate(u);
        }
        public Vec3D EvaluateDDU(double u)
        {
            InitIfRequired();

            Vec4D f = EvaluateH(u);
            Vec4D d1 = _firstPseudoDerivative.EvaluateH(u);
            Vec4D d2 = _secondPseudoDerivative.EvaluateH(u);

            double fw2 = f.W * f.W;
            return ((XYZ(d2) / f.W - XYZ(f) * (d2.W / fw2)) - ((XYZ(d1) / fw2 - XYZ(f) * (d1.W / (fw2 * f.W))) * 2 * d1.W));
            //return _secondPseudoDerivative.Evaluate(u); 
        }
        private Vec3D XYZ(Vec4D v) { return new Vec3D(v.X, v.Y, v.Z); }
        //numKnots = numControlPoints+_degree+1???

        private BSplineCurve() { }

        public BSplineCurve(int degree, List<Vec3D> controlPoints, bool closedCurve = false)
            : this(degree, controlPoints.ToArray(), Ones(controlPoints.Count), UniformKnotVector(degree, controlPoints.Count), closedCurve)
        { }
        public BSplineCurve(int degree, List<Vec3D> controlPoints, List<double> knots, bool closedCurve = false)
            : this(degree, controlPoints.ToArray(), Ones(controlPoints.Count), knots.ToArray(), closedCurve)
        { }
        public BSplineCurve(int degree, List<Vec3D> controlPoints, List<double> weights, List<double> knots, bool closedCurve)
            : this(degree, controlPoints.ToArray(), weights.ToArray(), knots.ToArray(), closedCurve = false)
        { }
        public BSplineCurve(int degree, Vec3D[] controlPoints, double[] knots, bool closedCurve = false)
            : this(degree, controlPoints, Ones(controlPoints.Length), knots, closedCurve)
        { }



        public BSplineCurve(int degree, Vec3D[] controlPoints, double[] weights, double[] knots, bool closedCurve = false)
            : this(degree, BuildControlPoints(controlPoints, weights), knots, closedCurve)
        { }

        public BSplineCurve(int degree, Vec4D[] controlPoints, double[] knots, bool closedCurve = false)
        {
            //Make sure knots are properly sorted
            for (int i = 1; i < knots.Length; ++i)
                if (knots[i] < knots[i - 1])
                    throw new Exception("Knots are not sorted in ascending order but this is required by the algorithms provided by this class!");

            /*for (int i = 1; i <= degree; ++i)
                if (knots[i] != knots[i - 1])
                    throw new Exception("This indicates that the parameter range is not 0...1");
            for (int i = knots.Count - degree; i < knots.Count; ++i)            
                if (knots[i] != knots[i - 1])
                    throw new Exception("This indicates that the parameter range is not 0...1");  */

            //Make sure the valid parameter range is 0...1
            if (knots[degree] != 0)
            {
                const double eps = 1e-10;
                if (Math.Abs(knots[degree]) < eps)
                {
                    for (int i = 0; i <= degree; ++i)
                    {
                        if (Math.Abs(knots[i]) < eps)
                            knots[i] = 0;
                    }
                }
                else
                    throw new Exception("Assumption of parameter range from 0...1 does not hold and this may lead to misfunction of some of the algoritms of this class");
            }
            if (knots[knots.Length - 1 - degree] != 1)
            {
                double scaling = 1.0 / knots[knots.Length - 1 - degree];
                for (int i = degree + 1; i < knots.Length - 1 - degree; ++i)
                    knots[i] = knots[i] * scaling;
                for (int i = knots.Length - 1 - degree; i < knots.Length; ++i)
                    knots[i] = 1;
            }

            if (knots.Length != controlPoints.Length + degree + 1)
                throw new Exception("The length of the arrays (either knots or controlPoints) does not match the required ratio for the specified degree");

            _degree = degree;
            _controlPoints = controlPoints;
            _knots = knots;
            _closedCurve = closedCurve;
            //_evalBuffer = new Vector3d[degree + 1];

            //Init();
        }

        private static Vec4D[] BuildControlPoints(Vec3D[] controlPoints, double[] weights)
        {
            int l = controlPoints.Length;
            Vec4D[] points = new Vec4D[l];
            for (int i = 0; i < l; ++i)
            {
                double w = weights[i];
                Vec3D v = controlPoints[i];
                points[i] = new Vec4D(w * v.X, w * v.Y, w * v.Z, w);
            }
            return points;
        }

        private BSplineCurve(int degree, Vec4D[] controlPoints, double[] knots, bool closedCurve, bool init)
        {
            //Make sure the valid parameter range is 0...1
            if (knots[degree] != 0 || knots[knots.Length - 1 - degree] != 1)
                throw new Exception("Assumption of parameter range from 0...1 does not hold and this may lead to misfunction of some of the algoritms of this class");

            for (int i = 1; i < knots.Length; ++i)
                if (knots[i] < knots[i - 1])
                    throw new Exception("Knots are not sorted in ascending order but this is required by the algorithms provided by this class!");

            if (knots.Length != controlPoints.Length + degree + 1)
                throw new Exception("The length of the arrays (ether knots or controlPoints) does not match the required ratio for the specified degree");

            _degree = degree;
            _controlPoints = controlPoints;
            _knots = knots;
            _closedCurve = closedCurve;
            //_evalBuffer = new Vector3d[degree + 1];

            if (init)
                Init();
        }

        public static double[] Ones(int count)
        {
            double[] ones = new double[count];
            for (int i = 0; i < count; ++i)
                ones[i] = 1;
            return ones;
        }

        private double Speed(double u)
        {
            InitIfRequired();

            // Differentiate the projected rational curve, not the homogeneous
            // control-point derivative (whose weight is itself a derivative).
            return EvaluateUniformDU(u).Length();
        }

        private void InitIfRequired()
        {
            if (_firstPseudoDerivative == null)
                Init();
        }

        //TODO: Add more inteligence in case of multiple knots at the same location!
        private void Init()
        {
            _firstPseudoDerivative = Derivative();
            _secondPseudoDerivative = _firstPseudoDerivative.Derivative();

            int l = NumberOfDistinctKnots(_degree, _knots);
            int numSplitsBetweenKnots = 2 * _degree; //Heuristics inspired by Shannon Theorem            
            _samples = new Vec3D[(l - 1) * numSplitsBetweenKnots + 1];
            _sampleParameters = new double[(l - 1) * numSplitsBetweenKnots + 1];
            _arcLength = new double[(l - 1) * numSplitsBetweenKnots + 1];
            // Arc speed is not a polynomial of the spline degree, especially
            // for rational weights. Reuse higher-order Gaussian quadrature.
            _integrator = GaussIntegrator.GetIntegrator(Math.Max(8, _degree));

            double left, right;
            int indexer = 0;
            int jStart = 0;
            l = _knots.Length - _degree - 1;
            for (int i = _degree; i < l; ++i)
            {
                left = _knots[i];
                right = _knots[i + 1];
                if (left != right)
                {
                    double spacing = (right - left) / numSplitsBetweenKnots;
                    for (int j = jStart; j <= numSplitsBetweenKnots; ++j)
                    {
                        double u = left + j * spacing;
                        _samples[indexer] = EvaluateUniform(u);
                        _sampleParameters[indexer] = u;
                        if (indexer > 0)
                            _arcLength[indexer] = _arcLength[indexer - 1] + _integrator.Integrate(_sampleParameters[indexer - 1], u, Speed);
                        else
                            _arcLength[0] = 0;
                        ++indexer;
                    }
                    jStart = 1; //Jump over the first node next time because it was already added in this iteation
                }
            }
            //_samples[indexer] = EvaluateDeBoor(right);
            //_parameters[indexer] = right;
            _totalArcLength = _arcLength[_arcLength.Length - 1];
        }


        public CoordinateSystem FrenetFrame(double u)
        {
            InitIfRequired();

            Vec3D tangent = EvaluateUniformDU(u); tangent.Normalize();
            Vec3D ddu = EvaluateDDU(u);
            Vec3D y = Vec3DOps.Cross(tangent, ddu);
            y.Normalize();
            Vec3D x = Vec3DOps.Cross(y, tangent);
            x.Normalize();
            Vec3D point = EvaluateUniform(u);
            return new CoordinateSystem(point, x, y, tangent);
        }

        public double MaxCurvatureInRange(double startParam, double endParam)
        {
            InitIfRequired();

            Vec3D v = _secondPseudoDerivative.EvaluateUniform(startParam);
            double maxCurvature = v.LengthSquared();
            v = _secondPseudoDerivative.EvaluateUniform(endParam);
            double lengthSquared = v.LengthSquared();
            if (lengthSquared > maxCurvature)
                maxCurvature = lengthSquared;

            int l = _sampleParameters.Length;
            for (int i = 0; i < l; ++i)
            {
                double param = _sampleParameters[i];

                if (param > startParam)
                {
                    if (param < endParam)
                    {
                        v = _secondPseudoDerivative.EvaluateUniform(param);
                        lengthSquared = v.LengthSquared();
                        if (lengthSquared > maxCurvature)
                            maxCurvature = lengthSquared;
                    }
                    else
                        break;
                }
            }
            return Math.Sqrt(maxCurvature);
        }

        /*public double MaxCurvature
        {
            get
            {
                if (_maxCurvature == -1)
                    _maxCurvature = ComputeMaxCurvature();
                return _maxCurvature;
            }
        }*/

        //float tmin, tmax; // The curve parameter interval [tmin,tmax].
        //Point Y(float t); // The position Y(t), tmin <= t <= tmax.
        //Point DY(float t); // The derivative dY(t)/dt, tmin <= t <= tmax.
        //float Speed(float t) { return Length(DY(t)); }
        //float ArcLength(float t) { return Integral(tmin, t, Speed()); }
        //float L = ArcLength(tmax); // The total length of the curve.

        public double ArcLength(double uStart, double uEnd)
        {
            return Math.Abs(ArcLength(uStart) - ArcLength(uEnd));
        }

        public double ArcLength(Vec3D p, bool pointOnCurve)
        {
            double u;
            if (!GetParameterOfClosestPointOnCurve(p, pointOnCurve, out u))
                throw new Exception();
            return ArcLength(u);
        }

        public double ArcLength(double u)
        {
            InitIfRequired();

            double lastU = _sampleParameters[_sampleParameters.Length - 1];
            double currentU;
            for (int i = _sampleParameters.Length - 2; i >= 0; --i)
            {
                currentU = _sampleParameters[i];
                if (u > currentU && u <= lastU)
                {
                    return _arcLength[i] + _integrator.Integrate(currentU, u, Speed);
                }
                lastU = currentU;
            }
            return 0;
        }

        public Vec3D EvaluateByArcLength(double arcLength)
        {
            double curveParam;
            return EvaluateByArcLength(arcLength, out curveParam);
        }

        public Vec3D EvaluateByArcLength(double arcLength, out double curveParam)
        {
            curveParam = GetCurveParameter(arcLength);
            double l = ArcLength(curveParam);
            if (Math.Abs(l - arcLength) > 1e-4)
                throw new Exception("GetCurveParameter might not have converged");
            return EvaluateUniform(curveParam);
        }

        public Vec3D EvaluateDerivativeByArcLength(double arcLength)
        {
            double curveParam;
            return EvaluateDerivativeByArcLength(arcLength, out curveParam);
        }

        public Vec3D EvaluateDerivativeByArcLength(double arcLength, out double curveParam)
        {
            curveParam = GetCurveParameter(arcLength);
            double l = ArcLength(curveParam);
            if (Math.Abs(l - arcLength) > 1e-4)
                throw new Exception("GetCurveParameter might not have converged");
            return EvaluateUniformDU(curveParam);
        }

        public double TotalArcLength
        {
            get
            {
                InitIfRequired();
                return _totalArcLength;
            }
        }

        //Taken from http://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf
        public double GetCurveParameter(double arcLength) // 0 <= s <= L, output is t
        {
            InitIfRequired();
            return GeometricAlgorithms.GetCurveParameter(arcLength, _totalArcLength, ArcLength, Speed);
        }


        public static int NumberOfDistinctKnots(int degree, double[] knots)
        {
            int l = knots.Length;
            int counter = 1;
            double last = knots[0];
            double current;
            for (int i = 1; i < l; ++i)
            {
                current = knots[i];
                if (current != last)
                    ++counter;
                last = current;
            }
            return counter;
        }

        //http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
        private BSplineCurve Derivative()
        {
            if (_degree == 0)
            {
                //TODO: Is this correct???
                //TODO: Valid parameter range might not always be 0...1
                return new BSplineCurve(0, new Vec4D[] { new Vec4D(0, 0, 0, 0) }, new double[] { 0, 1 }, _closedCurve, false);
                //throw new Exception("Derivative of this spline not computable using the code below");
            }

            Vec4D[] derivativeControlPoints = DerivativeControlPoints(_degree, _controlPoints, _knots);

            int l = _knots.Length - 2;
            double[] knots = new double[l];
            Array.Copy(_knots, 1, knots, 0, l);

            return new BSplineCurve(_degree - 1, derivativeControlPoints, knots, _closedCurve, false);
        }

        public static Vec4D[] DerivativeControlPoints(int degree, Vec4D[] controlPoints, double[] knots)
        {
            int l = controlPoints.Length - 1;
            Vec4D[] derivativeControlPoints = new Vec4D[l];
            for (int i = 0; i < l; ++i)
            {
                double d = degree / (knots[i + degree + 1] - knots[i + 1]);
                derivativeControlPoints[i] = d * (controlPoints[i + 1] - controlPoints[i]);
            }
            return derivativeControlPoints;
        }

        public List<Vec3D> SampleRange(Vec3D start, Vec3D end, out List<double> parameterValues)
        {
            //TODO: Improve
            double s, e;
            if (!GetParameterOfClosestPointOnCurve(start, true, out s))
                throw new Exception("Start point seems not to lie on the curve...");
            if (!GetParameterOfClosestPointOnCurve(end, true, out e))
                throw new Exception("End point seems not to lie on the curve...");

            List<Vec3D> samples = SampleRange(s, e, out parameterValues);
            samples[0] = start;
            samples[samples.Count - 1] = end;
            return samples;
        }

        public BSplineCurve ExtractRange(double start, double end, bool init = true, double eps = 1e-16)
        {
            if (start > end)
            {
                double tmp = start;
                start = end;
                end = tmp;
            }
            BSplineCurve lower, upper;
            if (start == 0)
                upper = this;
            else
                Split(start, out lower, out upper, false, eps);
            double e = (end - start) / (1 - start);
            if (end == 1)
                lower = upper;
            else
                upper.Split(e, out lower, out upper, false, eps);
            if (init)
                lower.Init();
            return lower;
        }

        public virtual void Split(double param, out BSplineCurve lower, out BSplineCurve upper, bool init = true, double eps = 1e-16)
        {
            Split(param, _degree, _knots, _controlPoints, _closedCurve, out lower, out upper, init, eps);
        }

        public static void Split(double param, int degree, double[] knots, Vec4D[] controlPoints, bool closedCurve,
            out BSplineCurve lower, out BSplineCurve upper, bool init = true, double eps = 1e-16)
        {
            // eps remains in the public signature for source compatibility.
            // Span membership and multiplicity require exact knot comparisons.
            int existingKnotMultiplicity;
            int k = GetKnotInsertionIndex(param, knots, out existingKnotMultiplicity);
            BSplineCurve buffer = InsertKnot(degree, knots, controlPoints, closedCurve, param, k, degree - existingKnotMultiplicity, false);

            int order = degree + 1;
            int numInnerPointsLower = k - order + 1 /*- existingKnotMultiplicity*/;
            double[] lowerKnots = new double[2 * order + numInnerPointsLower];
            for (int i = 0; i < order; ++i)
                lowerKnots[i] = 0;
            for (int i = 0; i < numInnerPointsLower; ++i)
                lowerKnots[i + order] = knots[i + order] / param;
            for (int i = order + numInnerPointsLower; i < lowerKnots.Length; ++i)
                lowerKnots[i] = 1;

            Vec4D[] lowerCP = new Vec4D[numInnerPointsLower + order];
            for (int i = 0; i < lowerCP.Length; ++i)
                lowerCP[i] = buffer._controlPoints[i];


            int numInnerPointsUpper = (knots.Length - k - 1) - order - existingKnotMultiplicity;
            double[] upperKnots = new double[2 * order + numInnerPointsUpper];
            for (int i = 0; i < order; ++i)
                upperKnots[i] = 0;
            for (int i = 0; i < numInnerPointsUpper; ++i)
                upperKnots[i + order] = (knots[i + order + numInnerPointsLower + existingKnotMultiplicity] - param) / (1 - param);
            for (int i = order + numInnerPointsUpper; i < upperKnots.Length; ++i)
                upperKnots[i] = 1;

            Vec4D[] upperCP = new Vec4D[numInnerPointsUpper + order];
            for (int i = 0; i < upperCP.Length; ++i)
                upperCP[i] = buffer._controlPoints[i + lowerCP.Length - 1 /*- existingKnotMultiplicity*/];


            lower = new BSplineCurve(degree, lowerCP, lowerKnots, closedCurve, init);
            upper = new BSplineCurve(degree, upperCP, upperKnots, closedCurve, init);
        }

        private static int GetKnotInsertionIndex(double knotLocation, double[] knots)
        {
            int existingKnotMultiplicity;
            return GetKnotInsertionIndex(knotLocation, knots, out existingKnotMultiplicity);
        }

        private static int GetKnotInsertionIndex(double knotLocation, double[] knots, out int existingKnotMultiplicity)
        {
            existingKnotMultiplicity = 0;
            int k = -1;
            int numKnots = knots.Length;
            for (int i = 1; i < numKnots; ++i)
            //for (int i = numKnots - 1; i > 0; --i)
            {
                // Knot ordering is topological: nearby but distinct doubles must
                // remain distinct. A tolerance here can insert a larger knot
                // before a smaller one and corrupt both the span and multiplicity.
                double lower = knots[i - 1];
                double upper = knots[i];
                if (knotLocation >= lower && knotLocation <= upper)
                {
                    k = i - 1;

                    for (int j = 0; j < numKnots; ++j)
                    {
                        if (knots[j] == knotLocation)
                            ++existingKnotMultiplicity;
                    }

                    break;
                }
            }
            if (k < 0)
                throw new Exception();

            return k;
        }

        ////TODO: Handle case when knot to insert lies exactly on an existing knot that can have multiplicity>1
        //private static int GetLowerKnotInsertionIndex(double knotLocation, double[] knots, double eps = 1e-16)
        //{
        //    int k = -1;
        //    int numKnots = knots.Length;
        //    for (int i = 1; i < numKnots; ++i)
        //    {
        //        double lower = knots[i - 1] - eps;
        //        double upper = knots[i] + eps;
        //        if (knotLocation >= lower && knotLocation <= upper)
        //        {
        //            k = i - 1;
        //            break;
        //        }
        //    }
        //    if (k < 0)
        //        throw new Exception();

        //    return k;
        //}

        //private static int GetUpperKnotInsertionIndex(double knotLocation, double[] knots, double eps = 1e-16)
        //{
        //    int k = -1;
        //    int numKnots = knots.Length;
        //    for (int i = numKnots - 1; i > 0; --i) //Just a reversed loop compared to GetLowerKnotInsertionIndex
        //    {
        //        double lower = knots[i - 1] - eps;
        //        double upper = knots[i] + eps;
        //        if (knotLocation >= lower && knotLocation <= upper)
        //        {
        //            k = i - 1;
        //            break;
        //        }
        //    }
        //    if (k < 0)
        //        throw new Exception();

        //    return k;
        //}

        public BSplineCurve InsertKnot(double knotLocation, int knotMultiplicity = 1, bool init = true)
        {
            int k = GetKnotInsertionIndex(knotLocation, _knots);
            return InsertKnot(_degree, _knots, _controlPoints, _closedCurve, knotLocation, k, knotMultiplicity, init);
        }

        private static BSplineCurve InsertKnot(int degree, double[] knots, Vec4D[] controlPoints, bool closedCurve,
            double knotLocation, int k, int knotMultiplicity = 1, bool init = true)
        {
            int numKnots = knots.Length;
            double[] newKnots = new double[numKnots + knotMultiplicity];
            for (int i = 0; i <= k; ++i)
                newKnots[i] = knots[i];
            for (int i = 0; i < knotMultiplicity; ++i)
                newKnots[k + i + 1] = knotLocation;
            for (int i = k + 1; i < numKnots; ++i)
                newKnots[i + knotMultiplicity] = knots[i];

            int numControlPoints = controlPoints.Length;
            Vec4D[] newControlPoints = new Vec4D[numControlPoints + knotMultiplicity];
            for (int i = 0; i <= k; ++i)
                newControlPoints[i] = controlPoints[i];
            for (int i = k + 1; i < numControlPoints; ++i)
                newControlPoints[i + knotMultiplicity] = controlPoints[i];

            for (int i = 0; i < knotMultiplicity; ++i)
            {
                //Store the point at the bottom to get space to store the first result of the following loop
                newControlPoints[k + knotMultiplicity - i] = newControlPoints[k];
                int lower = k - degree + i;
                for (int j = k; j > lower; --j)
                {
                    double alpha = (knotLocation - newKnots[j]) / (newKnots[j + degree + (knotMultiplicity - i)] - newKnots[j]);
                    newControlPoints[j] = (1 - alpha) * newControlPoints[j - 1] + alpha * newControlPoints[j];
                }
            }

            return new BSplineCurve(degree, newControlPoints, newKnots, closedCurve, init);
        }



        //TODO: Might require some verification...
        public List<Vec3D> SampleRange(double uStart, double uEnd, out List<double> parameterValues)
        {
            if (uStart > uEnd)
            {
                double tmp = uStart;
                uStart = uEnd;
                uEnd = tmp;
            }

            int l = _knots.Length;
            int s = DeBoor.KnotIndex(uStart, _degree, _knots);

            int e = 0;
            while (e < l)
            {
                if (_knots[e] >= uEnd)
                    break;
                ++e;
            }

            int numSplitsBetweenKnots = 2 * _degree; //Heuristics inspired by Shannon Theorem         
            List<Vec3D> samples = new List<Vec3D>();
            parameterValues = new List<double>();
            //Handle start
            samples.Add(EvaluateUniform(uStart));
            parameterValues.Add(uStart);
            double left = _knots[s];
            double right = _knots[s + 1];
            double spacing = (right - left) / numSplitsBetweenKnots;
            bool finished = false;
            for (int i = 0; i <= numSplitsBetweenKnots; ++i)
            {
                double u = left + i * spacing;
                if (u > uStart + spacing)
                {
                    if (u < uEnd - spacing)
                    {
                        samples.Add(EvaluateUniform(u));
                        parameterValues.Add(u);
                    }
                    else
                        finished = true;
                }
            }

            if (!finished)
            {
                l = _knots.Length - _degree - 1;
                for (int i = _degree; i < l; ++i)
                {
                    left = _knots[i];
                    right = _knots[i + 1];
                    spacing = (right - left) / numSplitsBetweenKnots;
                    if (left != right && left > uStart + spacing && right < uEnd - spacing)
                    {
                        for (int j = 1/*jStart*/; j <= numSplitsBetweenKnots; ++j)
                        {
                            double u = left + j * spacing;
                            samples.Add(EvaluateUniform(u));
                            parameterValues.Add(u);
                        }
                    }
                }

                left = _knots[e - 1];
                right = _knots[e];
                for (int i = 1; i <= numSplitsBetweenKnots; ++i)
                {
                    double u = left + i * spacing;
                    if (u < uEnd - spacing && u > uStart + spacing)
                    {
                        samples.Add(EvaluateUniform(u));
                        parameterValues.Add(u);
                    }
                }
            }
            //Handle end            
            samples.Add(EvaluateUniform(uEnd));
            parameterValues.Add(uEnd);

            return samples;
        }

        public bool GetParameterOfClosestPointOnCurve(Vec3D p, bool pOnCurve, out double u)
        {
            InitIfRequired();

            const double eps1 = 1e-3; //measure of Eucledian distance
            const double eps2 = 1e-6; //zero cosine measure

            const double eps1Squared = eps1 * eps1;
            const double eps2Squared = eps2 * eps2;

            //Find initial condition for following Newton Iteration         
            double minDistanceSquared = double.MaxValue;
            double dist;
            int l = _samples.Length;
            int index = -1;
            for (int i = 0; i < l; ++i)
            {
                dist = (_samples[i] - p).LengthSquared();
                if (dist < minDistanceSquared)
                {
                    minDistanceSquared = dist;
                    index = i;
                }
            }

            //Newton Iteration
            u = _sampleParameters[index];
            double uNext;

            bool converged = false;
            int counter = 0;
            while (true)
            {
                Vec3D cp = EvaluateUniform(u) - p;
                double cpLengthSquared = cp.LengthSquared();
                if (cpLengthSquared <= eps1Squared)
                {
                    converged = true;
                    break;
                }

                Vec3D dc = _firstPseudoDerivative.EvaluateUniform(u);
                double dcDotCp = Vec3DOps.Dot(dc, cp);
                double dcLengthSquared = dc.LengthSquared();
                if (!pOnCurve)
                {
                    double dotCondition = (dcDotCp * dcDotCp) / (dcLengthSquared * cpLengthSquared);
                    if (dotCondition <= eps2Squared)
                    {
                        converged = true;
                        break;
                    }
                }
                Vec3D ddc = _secondPseudoDerivative.EvaluateUniform(u);
                uNext = u - 0.1 * (dcDotCp / (Vec3DOps.Dot(ddc, cp) + dcLengthSquared));

                //TODO: Valid parameter range might not always be 0...1
                /*if (uNext < 0)
                    uNext = 0;
                if (uNext > 1)
                    uNext = 1;*/

                if (((uNext - u) * dc).LengthSquared() <= 0/*eps1Squared*/)
                {
                    converged = false;
                    break;
                }

                u = uNext;
                ++counter;
                if (counter > 1000)
                {
                }
            }
            return converged;
        }

        //http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/de-Boor.html
        public virtual Vec3D EvaluateUniform(double u)
        {
            return DeBoor.Project(EvaluateH(u));
            //Vector3d res = DeBoor.Project(v);

            //return DeBoor2.EvaluateRational(_degree, u, _controlPoints, _knots);
        }

        protected virtual Vec4D EvaluateH(double u)
        {
            return DeBoor.EvaluateRational(_degree, u, _controlPoints, _knots, ref _spanHint);
        }

        /*public Vector3d EvaluateDeBoorRational(double u, double[] weights)
        {
            return DeBoor.EvaluateRational(_degree, u, _controlPoints, weights, _knots);
        }  */

        public List<Vec3D> Tessellate(int numPoints)
        {
            double s = 1.0 / (numPoints - 1);
            List<Vec3D> points = new List<Vec3D>(numPoints);

            points.Add(EvaluateUniform(0));
            int l = numPoints - 1;
            for (int i = 1; i < l; ++i)
                points.Add(EvaluateUniform(i * s));
            points.Add(EvaluateUniform(1));

            return points;
        }

        public List<Vec3D> Tessellate(double maxDeviation)
        {
            List<double> parameters;
            List<Vec3D> points = AdaptiveSurfaceSplitter.Tessellate(this, out parameters, tol: maxDeviation);
            return points;
        }

        public List<Vec3D> Tessellate(double maxDeviation, out List<double> parameters)
        {
            List<Vec3D> points = AdaptiveSurfaceSplitter.Tessellate(this, out parameters, tol: maxDeviation);
            return points;
        }

        //TODO: Not sure if this is correct for rational splines...
        //TODO: Apply to derivatives...
        //public void ApplyTransformation(Double4x4 trafo)
        //{
        //    if (trafo != Double4x4.Identity)
        //    {

        //    }

        //    var a = EvaluateUniform(0);
        //    var b = EvaluateUniform(0.2);
        //    var c = EvaluateUniform(1);

        //    for (int i = 0; i < _controlPoints.Length; ++i)
        //    {
        //        var cp = _controlPoints[i];
        //        var p = cp.Xyz * (1.0 / cp.W);

        //        p = trafo.TransformPoint(p);

        //        cp.Xyz = p * cp.W;
        //        _controlPoints[i] = cp;
        //    }

        //    var aa = EvaluateUniform(0);
        //    var bb = EvaluateUniform(0.2);
        //    var cc = EvaluateUniform(1);

        //    var aaa = trafo.TransformPoint(a);
        //    var bbb = trafo.TransformPoint(b);
        //    var ccc = trafo.TransformPoint(c);
        //}

        public static double[] UniformKnotVector(int curveDegree, int numControlPoints)
        {
            int numKnots = numControlPoints + curveDegree + 1;
            double[] result = new double[numKnots];

            for (int i = 0; i <= curveDegree; ++i)
            {
                result[i] = 0;
            }

            int numInterKnots = numKnots - 2 * (curveDegree + 1);
            if (numInterKnots > 0)
            {
                int l = numInterKnots + 1;
                double scaling = 1.0 / l;
                for (int i = 1; i < l; ++i)
                {
                    result[i + curveDegree] = i * scaling;
                }
            }

            for (int i = result.Length - curveDegree - 1; i < result.Length; ++i)
            {
                result[i] = 1;
            }

            return result;
        }
    }
}
