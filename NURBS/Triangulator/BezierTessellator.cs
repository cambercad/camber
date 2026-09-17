using GeoCore;
using System.Collections.Generic;

namespace NURBS
{
    public class BezierTessellator
    {
        public static unsafe Vec2D EvaluateBezierDeCasteljau(IList<Vec2D> controlPoints, double u)
        {
            double u2 = 1 - u;
            int l = controlPoints.Count - 1;
            double* bufferX = stackalloc double[l];
            double* bufferY = stackalloc double[l];
            for (int i = 0; i < l; ++i)
            {
                Vec2D a = controlPoints[i];
                Vec2D b = controlPoints[i + 1];
                bufferX[i] = u2 * a.X + u * b.X;
                bufferY[i] = u2 * a.Y + u * b.Y;
            }
            for (int i = l - 2; i >= 0; --i)
            {
                for (int j = 0; j <= i; ++j)
                {
                    bufferX[j] = u2 * bufferX[j] + u * bufferX[j + 1];
                    bufferY[j] = u2 * bufferY[j] + u * bufferY[j + 1];
                }
            }
            return new Vec2D(bufferX[0], bufferY[0]);
        }

        public static unsafe Vec2D EvaluateBezierDerivativeDeCasteljau(IList<Vec2D> controlPoints, double u)
        {
            double u2 = 1 - u;
            int l = controlPoints.Count - 1;
            double* bufferX = stackalloc double[l];
            double* bufferY = stackalloc double[l];
            for (int i = 0; i < l; ++i)
            {
                Vec2D a = controlPoints[i];
                Vec2D b = controlPoints[i + 1];
                bufferX[i] = l * (b.X - a.X);
                bufferY[i] = l * (b.Y - a.Y);
            }
            for (int i = l - 2; i >= 0; --i)
            {
                for (int j = 0; j <= i; ++j)
                {
                    bufferX[j] = u2 * bufferX[j] + u * bufferX[j + 1];
                    bufferY[j] = u2 * bufferY[j] + u * bufferY[j + 1];
                }
            }
            return new Vec2D(bufferX[0], bufferY[0]);
        }

        public static void Test()
        {
            List<Vec2D> result = Tessellate(0.01, new Vec2D[] { new Vec2D(0, 0), new Vec2D(2, 0), new Vec2D(0, 2), new Vec2D(2, 2) });
        }

        public static List<Vec2D> Tessellate(double maxDeviation, IList<Vec2D> controlPoints)
        {
            List<double> uValues;
            return Tessellate(maxDeviation, controlPoints, out uValues);
        }

        //http://www.malinc.se/m/DeCasteljauAndBezier.php
        public static List<Vec2D> Tessellate(double maxDeviation, IList<Vec2D> controlPoints, out List<double> uValues)
        {
            double max2 = maxDeviation * maxDeviation;

            List<Vec2D> result = new List<Vec2D>();
            uValues = new List<double>();
            Stack<IList<Vec2D>> stack = new Stack<IList<Vec2D>>();
            stack.Push(new List<Vec2D>(controlPoints));
            Stack<Vec2D> uStack = new Stack<Vec2D>();
            uStack.Push(new Vec2D(0, 1));

            int length = controlPoints.Count;
            int l = length - 1;

            while (stack.Count > 0)
            {
                IList<Vec2D> cp = stack.Pop();
                Vec2D u = uStack.Pop();

                Vec2D start = cp[0];
                Vec2D end = cp[l];
                double max = 0;
                for (int i = 1; i < l; ++i)
                {
                    double d = GeometricAlgorithms.DistancePointSegmentSquared(cp[i], start, end);

                    if (d > max)
                        max = d;
                }

                if (max > max2)
                {
                    //Split
                    Vec2D[] pointsA = new Vec2D[length];
                    Vec2D[] pointsB = new Vec2D[length];

                    pointsA[0] = cp[0];
                    pointsB[l] = cp[l];
                    int k = 1;
                    //De Casteljau Algorithm
                    for (int i = l - 1; i >= 0; --i)
                    {
                        for (int j = 0; j <= i; ++j)
                        {
                            cp[j] = 0.5 * (cp[j] + cp[j + 1]);
                        }
                        pointsA[k] = cp[0];
                        pointsB[l - k] = cp[i];
                        ++k;
                    }

                    //Order matters on a stack!
                    stack.Push(pointsB);
                    stack.Push(pointsA);

                    double v = 0.5 * (u.X + u.Y);
                    uStack.Push(new Vec2D(v, u.Y));
                    uStack.Push(new Vec2D(u.X, v));
                }
                else
                {
                    result.Add(cp[0]);//Add the start point of the local bezier
                    uValues.Add(u.X);
                }
            }

            result.Add(controlPoints[controlPoints.Count - 1]); //Add the end point of the global bezier
            uValues.Add(1);
            return result;
        }

        public static List<Vec3D> Tessellate(double maxDeviation, params Vec3D[] controlPoints)
        {
            return Tessellate(maxDeviation, (IList<Vec3D>)controlPoints);
        }
        public static List<Vec3D> Tessellate(double maxDeviation, IList<Vec3D> controlPoints)
        {
            List<double> uValues;
            return Tessellate(maxDeviation, controlPoints, out uValues);
        }
        //http://www.malinc.se/m/DeCasteljauAndBezier.php
        public static List<Vec3D> Tessellate(double maxDeviation,
            IList<Vec3D> controlPoints, out List<double> uValues)
        {
            double max2 = maxDeviation * maxDeviation;

            List<Vec3D> result = new List<Vec3D>();
            uValues = new List<double>();
            Stack<IList<Vec3D>> stack = new Stack<IList<Vec3D>>();
            stack.Push(new List<Vec3D>(controlPoints));
            Stack<Vec2D> uStack = new Stack<Vec2D>();
            uStack.Push(new Vec2D(0, 1));

            int length = controlPoints.Count;
            int l = length - 1;

            while (stack.Count > 0)
            {
                IList<Vec3D> cp = stack.Pop();
                Vec2D u = uStack.Pop();

                Vec3D start = cp[0];
                Vec3D end = cp[l];
                double max = 0;
                for (int i = 1; i < l; ++i)
                {
                    double d = GeometricAlgorithms.DistancePointSegmentSquared(cp[i], start, end);

                    if (d > max)
                        max = d;
                }

                if (max > max2)
                {
                    //Split
                    Vec3D[] pointsA = new Vec3D[length];
                    Vec3D[] pointsB = new Vec3D[length];

                    pointsA[0] = cp[0];
                    pointsB[l] = cp[l];
                    int k = 1;
                    //De Casteljau Algorithm
                    for (int i = l - 1; i >= 0; --i)
                    {
                        for (int j = 0; j <= i; ++j)
                        {
                            cp[j] = 0.5 * (cp[j] + cp[j + 1]);
                        }
                        pointsA[k] = cp[0];
                        pointsB[l - k] = cp[i];
                        ++k;
                    }

                    //Order matters on a stack!
                    stack.Push(pointsB);
                    stack.Push(pointsA);

                    double v = 0.5 * (u.X + u.Y);
                    uStack.Push(new Vec2D(v, u.Y));
                    uStack.Push(new Vec2D(u.X, v));
                }
                else
                {
                    result.Add(cp[0]);//Add the start point of the local bezier
                    uValues.Add(u.X);
                }
            }

            result.Add(controlPoints[controlPoints.Count - 1]); //Add the end point of the global bezier
            uValues.Add(1);
            return result;
        }

        //https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-sub.html
        public static void Subdivide(IList<Vec3D> controlPoints, double uSplit,
            out List<Vec3D> controlPointsA, out List<Vec3D> controlPointsB)
        {
            controlPointsA = new List<Vec3D>(controlPoints.Count);
            controlPointsB = new List<Vec3D>(controlPoints.Count);

            controlPointsA.Add(controlPoints[0]);
            controlPointsB.Add(controlPoints[controlPoints.Count - 1]);

            Vec3D[] buffer = new Vec3D[controlPoints.Count];
            for (int i = 0; i < controlPoints.Count; ++i)
                buffer[i] = controlPoints[i];

            int l = controlPoints.Count - 1;
            for (int i = l; i >= 0; --i)
            {
                for (int j = 0; j < l; ++j)
                {
                    buffer[j] = (1 - uSplit) * buffer[j] + uSplit * buffer[j + 1];
                }
                controlPointsA.Add(buffer[0]);
                controlPointsB.Add(buffer[l - 1]);
            }

            controlPointsB.Reverse();
        }

        public static void Subdivide(IList<Vec2D> controlPoints, double uSplit,
            out List<Vec2D> controlPointsA, out List<Vec2D> controlPointsB)
        {
            controlPointsA = new List<Vec2D>(controlPoints.Count);
            controlPointsB = new List<Vec2D>(controlPoints.Count);

            controlPointsA.Add(controlPoints[0]);
            controlPointsB.Add(controlPoints[controlPoints.Count - 1]);

            Vec2D[] buffer = new Vec2D[controlPoints.Count];
            for (int i = 0; i < controlPoints.Count; ++i)
                buffer[i] = controlPoints[i];

            int l = controlPoints.Count - 1;
            for (int i = l; i >= 0; --i)
            {
                for (int j = 0; j < l; ++j)
                {
                    buffer[j] = (1 - uSplit) * buffer[j] + uSplit * buffer[j + 1];
                }
                controlPointsA.Add(buffer[0]);
                controlPointsB.Add(buffer[l - 1]);
            }

            controlPointsB.Reverse();
        }
    }
}
