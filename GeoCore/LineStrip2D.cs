using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GeoCore
{
    public class LineStrip2D
    {
        private IList<Vec2D> points; public IList<Vec2D> Points { get { return points; } }
        private IList<double> distances;


        public LineStrip2D(IList<Vec2D> points)
        {
            this.points = points;
            this.distances = BuildDistanceBuffer(points);
        }

        public LineStrip2D(IList<Vec2D> points, IList<double> distances)
        {
            this.points = points;
            this.distances = distances;
        }

        public double GetDistanceFromBuffer(int index)
        {
            return distances[index];
        }

        public Vec2D EvaluateUniform(double uniform)
        {
            return EvaluateAtDistance(points, distances, uniform * distances[distances.Count - 1]);
        }

        public Vec2D Evaluate(double distance)
        {
            return EvaluateAtDistance(points, distances, distance);
        }

        public Vec2D EvaluateUniform(double uniform, out double index)
        {
            return EvaluateAtDistance(points, distances, uniform * distances[distances.Count - 1], out index);
        }

        public Vec2D Evaluate(double distance, out double index)
        {
            return EvaluateAtDistance(points, distances, distance, out index);
        }

        public double TotalLength { get { return distances[distances.Count - 1]; } }

        public static double DistanceSquared(IList<Vec2D> lineStrip, Vec2D p) { return DistancePointToStripSquared(lineStrip, p); }


        //Static methods
        public static double[] BuildDistanceBuffer(IList<Vec2D> points)
        {
            int l = points.Count;

            double[] result = new double[l];
            result[0] = 0;
            double length = 0;
            for (int i = 1; i < l; ++i)
            {
                Vec2D prev = points[i - 1];
                Vec2D current = points[i];
                double dx = current.X - prev.X;
                double dy = current.Y - prev.Y;
                length += Math.Sqrt(dx * dx + dy * dy);
                result[i] = length;
            }

            return result;
        }

        public static Vec2D EvaluateAtDistance(IList<Vec2D> pointBuffer, IList<double> distanceBuffer, double distance, double startSearchIndex = 0, bool cyclic = false)
        {
            double index;
            return EvaluateAtDistance(pointBuffer, distanceBuffer, distance, out index, startSearchIndex, cyclic);
        }

        public static Vec2D EvaluateAtDistance(IList<Vec2D> pointBuffer, IList<double> distanceBuffer, double distance, out double index, double startSearchIndex = 0, bool cyclic = false)
        {
            if (distance >= distanceBuffer[distanceBuffer.Count - 1])
            {
                //TODO: The cyclic case needs to be tested
                if (cyclic)
                {
                    Vec2D first = pointBuffer[0];
                    Vec2D last = pointBuffer[pointBuffer.Count - 1];

                    double dx = last.X - first.X;
                    double dy = last.Y - first.Y;
                    double distanceLastToFirstPoint = Math.Sqrt(dx * dx + dy * dy);

                    double closedLoopLength = distanceBuffer[distanceBuffer.Count - 1] + distanceLastToFirstPoint;

                    int numFullCycles = (int)(distance / closedLoopLength);
                    double mappedLength = distance - numFullCycles * closedLoopLength;

                    double d = distanceBuffer[distanceBuffer.Count - 1];
                    if (mappedLength >= d)
                    {
                        // Walk the closing chord in perimeter order: last vertex → first (open chain ends at last).
                        if (distanceLastToFirstPoint <= 1e-30)
                        {
                            index = distanceBuffer.Count - 1;
                            return last;
                        }

                        double w = (mappedLength - d) / distanceLastToFirstPoint;
                        if (w > 1)
                            w = 1;
                        index = distanceBuffer.Count - 1 + w;
                        return new Vec2D(
                            last.X * (1 - w) + w * first.X,
                            last.Y * (1 - w) + w * first.Y);
                    }
                    distance = mappedLength;
                }
                else
                {
                    index = distanceBuffer.Count;
                    return pointBuffer[pointBuffer.Count - 1];
                }
            }

            if (distance <= 0)
            {
                index = 0;
                return pointBuffer[0];
            }

            int l = distanceBuffer.Count;
            int start = (int)(startSearchIndex);
            if (start >= l || distanceBuffer[start] > distance)
                start = 1;

            for (int i = start/*1*/; i < l; ++i)
            {
                if (distanceBuffer[i] > distance)
                {
                    double delta = distanceBuffer[i] - distanceBuffer[i - 1];
                    double w;
                    if (delta < 1e-8)
                        w = 0;
                    else
                        w = (distance - distanceBuffer[i - 1]) / delta;

                    Vec2D a = pointBuffer[i - 1];
                    Vec2D b = pointBuffer[i];

                    index = i - 1 + w;

                    return new Vec2D(
                        a.X * (1 - w) + w * b.X,
                        a.Y * (1 - w) + w * b.Y);
                }
            }

            //if (startSearchIndex != 0)
            //    return EvaluateAtDistance(pointBuffer, distanceBuffer, distance, out index, 0, cyclic);

            throw new Exception("Code should never reach this line");
        }

        public static double DistancePointToStripSquared(IList<Vec2D> lineStrip, Vec2D p)
        {
            Vec2D s = lineStrip[0];
            double dX = p.X - s.X;
            double dY = p.Y - s.Y;

            double minDistSquared = dX * dX + dY * dY;

            int l = lineStrip.Count;
            for (int i = 1; i < l; ++i)
            {
                s = lineStrip[i - 1];
                Vec2D e = lineStrip[i];

                dX = e.X - s.X;
                dY = e.Y - s.Y;

                double a = p.X - s.X; double b = p.Y - s.Y;
                double t = (a * dX + b * dY) / (dX * dX + dY * dY);

                if (t > 1)
                {
                    dX = p.X - e.X;
                    dY = p.Y - e.Y;
                    double d2 = dX * dX + dY * dY;
                    if (d2 < minDistSquared)
                        minDistSquared = d2;
                }
                else if (t >= 0)
                {
                    a = t * dX - a;
                    b = t * dY - b;

                    double d2 = a * a + b * b;
                    if (d2 < minDistSquared)
                        minDistSquared = d2;
                }
            }

            //double debug = Debug(lineStrip, p);

            return minDistSquared;
        }
    }
}
