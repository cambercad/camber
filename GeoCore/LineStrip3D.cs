namespace GeoCore
{
    public class LineStrip3D
    {
        private IList<Vec3D> points; public IList<Vec3D> Points { get { return points; } }
        private IList<double> distances;


        public LineStrip3D(IList<Vec3D> points)
        {
            this.points = points;
            this.distances = BuildDistanceBuffer(points);
        }

        public LineStrip3D(IList<Vec3D> points, IList<double> distances)
        {
            this.points = points;
            this.distances = distances;
        }

        public double GetDistanceFromBuffer(int index)
        {
            return distances[index];
        }

        public bool IsClosed(double tolerance = 1e-8)
        {
            var first = EvaluateUniform(0);
            var last = EvaluateUniform(1);
            var d2 = Vec3DOps.DistanceSquared(first, last);
            return d2 < tolerance * tolerance;
        }

        public Vec3D EvaluateUniform(double uniform)
        {
            return EvaluateAtDistance(points, distances, uniform * distances[distances.Count - 1]);
        }

        public Vec3D Evaluate(double distance)
        {
            return EvaluateAtDistance(points, distances, distance);
        }

        public Vec3D EvaluateUniform(double uniform, out double index)
        {
            return EvaluateAtDistance(points, distances, uniform * distances[distances.Count - 1], out index);
        }

        public Vec3D Evaluate(double distance, out double index)
        {
            return EvaluateAtDistance(points, distances, distance, out index);
        }

        public double TotalLength { get { return distances[distances.Count - 1]; } }

        /// <summary>
        /// Finds the uniform parameter value (0 to 1) of the closest point on the line strip to the given point.
        /// </summary>
        /// <param name="p">The point to find the closest point to</param>
        /// <returns>The uniform parameter value (0 to 1) where the closest point lies</returns>
        public double GetClosestPointUniform(Vec3D p)
        {
            double closestDistance;
            int segmentIndex;
            FindClosestPointOnStrip(points, distances, p, out closestDistance, out segmentIndex);
            
            double totalLength = distances[distances.Count - 1];
            if (totalLength < 1e-10)
                return 0.0;
            
            return closestDistance / totalLength;
        }

        /// <summary>
        /// Evaluates the unit tangent vector at the given uniform parameter value (0 to 1).
        /// If the uniform value hits a point directly, returns the tangent of the segment after the point.
        /// For the last point, returns the tangent of the segment before it.
        /// </summary>
        /// <param name="uniform">The uniform parameter value (0 to 1)</param>
        /// <returns>The unit tangent vector at the given uniform parameter</returns>
        public Vec3D EvaluateTangentUniform(double uniform)
        {
            double distance = uniform * distances[distances.Count - 1];
            return EvaluateTangentAtDistance(points, distances, distance);
        }

        /// <summary>
        /// Evaluates the unit tangent vector at the given distance along the line strip.
        /// </summary>
        /// <param name="pointBuffer">The points of the line strip</param>
        /// <param name="distanceBuffer">The distance buffer</param>
        /// <param name="distance">The distance along the line strip</param>
        /// <returns>The unit tangent vector at the given distance</returns>
        private static Vec3D EvaluateTangentAtDistance(IList<Vec3D> pointBuffer, IList<double> distanceBuffer, double distance)
        {
            // Handle edge cases
            if (distance <= 0)
            {
                // At the start, use the first segment
                Vec3D tangent = pointBuffer[1] - pointBuffer[0];
                double length = tangent.Length();
                return length > 1e-10 ? tangent / length : new Vec3D(1, 0, 0);
            }
            
            if (distance >= distanceBuffer[distanceBuffer.Count - 1])
            {
                // At the end, use the last segment
                int lastIdx = pointBuffer.Count - 1;
                Vec3D tangent = pointBuffer[lastIdx] - pointBuffer[lastIdx - 1];
                double length = tangent.Length();
                return length > 1e-10 ? tangent / length : new Vec3D(1, 0, 0);
            }
            
            // Find the segment containing this distance
            int l = distanceBuffer.Count;
            for (int i = 1; i < l; ++i)
            {
                if (distanceBuffer[i] >= distance)
                {
                    // We're in segment [i-1, i]
                    Vec3D tangent = pointBuffer[i] - pointBuffer[i - 1];
                    double length = tangent.Length();
                    return length > 1e-10 ? tangent / length : new Vec3D(1, 0, 0);
                }
            }
            
            // Fallback (should not reach here)
            return new Vec3D(1, 0, 0);
        }

        public static double DistanceSquared(IList<Vec3D> lineStrip, Vec3D p) { return DistancePointToStripSquared(lineStrip, p); }


        //Static methods
        public static double[] BuildDistanceBuffer(IList<Vec3D> points)
        {
            int l = points.Count;

            double[] result = new double[l];
            result[0] = 0;
            double length = 0;
            for (int i = 1; i < l; ++i)
            {
                Vec3D prev = points[i - 1];
                Vec3D current = points[i];
                double dx = current.X - prev.X;
                double dy = current.Y - prev.Y;
                double dz = current.Z - prev.Z;
                length += Math.Sqrt(dx * dx + dy * dy + dz * dz);
                result[i] = length;
            }

            return result;
        }

        public static Vec3D EvaluateAtDistance(IList<Vec3D> pointBuffer, IList<double> distanceBuffer, double distance, double startSearchIndex = 0, bool cyclic = false)
        {
            double index;
            return EvaluateAtDistance(pointBuffer, distanceBuffer, distance, out index, startSearchIndex, cyclic);
        }

        public static Vec3D EvaluateAtDistance(IList<Vec3D> pointBuffer, IList<double> distanceBuffer, double distance, out double index, double startSearchIndex = 0, bool cyclic = false)
        {
            if (distance >= distanceBuffer[distanceBuffer.Count - 1])
            {
                //TODO: The cyclic case needs to be tested
                if (cyclic)
                {
                    Vec3D first = pointBuffer[0];
                    Vec3D last = pointBuffer[pointBuffer.Count - 1];

                    double dx = last.X - first.X;
                    double dy = last.Y - first.Y;
                    double dz = last.Z - first.Z;
                    double distanceLastToFirstPoint = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                    double closedLoopLength = distanceBuffer[distanceBuffer.Count - 1] + distanceLastToFirstPoint;

                    int numFullCycles = (int)(distance / closedLoopLength);
                    double mappedLength = distance - numFullCycles * closedLoopLength;

                    double d = distanceBuffer[distanceBuffer.Count - 1];
                    if (mappedLength >= d)
                    {
                        if (distanceLastToFirstPoint <= 1e-30)
                        {
                            index = distanceBuffer.Count - 1;
                            return last;
                        }

                        double w = (mappedLength - d) / distanceLastToFirstPoint;
                        if (w > 1)
                            w = 1;
                        index = distanceBuffer.Count - 1 + w;
                        return new Vec3D(
                            last.X * (1 - w) + w * first.X,
                            last.Y * (1 - w) + w * first.Y,
                            last.Z * (1 - w) + w * first.Z);
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

                    Vec3D a = pointBuffer[i - 1];
                    Vec3D b = pointBuffer[i];

                    index = i - 1 + w;

                    return new Vec3D(
                        a.X * (1 - w) + w * b.X,
                        a.Y * (1 - w) + w * b.Y,
                        a.Z * (1 - w) + w * b.Z);
                }
            }

            //if (startSearchIndex != 0)
            //    return EvaluateAtDistance(pointBuffer, distanceBuffer, distance, out index, 0, cyclic);

            throw new Exception("Code should never reach this line");
        }

        /// <summary>
        /// Finds the closest point on the line strip to the given point and returns information about it.
        /// </summary>
        /// <param name="lineStrip">The line strip points</param>
        /// <param name="distanceBuffer">The distance buffer for the line strip</param>
        /// <param name="p">The point to find the closest point to</param>
        /// <param name="closestDistance">Output: The distance along the line strip to the closest point</param>
        /// <param name="segmentIndex">Output: The segment index where the closest point lies</param>
        /// <returns>The squared distance to the closest point</returns>
        private static double FindClosestPointOnStrip(IList<Vec3D> lineStrip, IList<double> distanceBuffer, Vec3D p, 
            out double closestDistance, out int segmentIndex)
        {
            Vec3D s = lineStrip[0];
            double dX = p.X - s.X;
            double dY = p.Y - s.Y;
            double dZ = p.Z - s.Z;

            double minDistSquared = dX * dX + dY * dY + dZ * dZ;
            closestDistance = 0.0;
            segmentIndex = 0;

            int l = lineStrip.Count;
            for (int i = 1; i < l; ++i)
            {
                s = lineStrip[i - 1];
                Vec3D e = lineStrip[i];

                dX = e.X - s.X;
                dY = e.Y - s.Y;
                dZ = e.Z - s.Z;

                double a = p.X - s.X; double b = p.Y - s.Y; double c = p.Z - s.Z;
                double segmentLengthSq = dX * dX + dY * dY + dZ * dZ;
                double t = (a * dX + b * dY + c * dZ) / segmentLengthSq;

                if (t > 1)
                {
                    dX = p.X - e.X;
                    dY = p.Y - e.Y;
                    dZ = p.Z - e.Z;
                    double d2 = dX * dX + dY * dY + dZ * dZ;
                    if (d2 < minDistSquared)
                    {
                        minDistSquared = d2;
                        closestDistance = distanceBuffer[i];
                        segmentIndex = i;
                    }
                }
                else if (t >= 0)
                {
                    a = t * dX - a;
                    b = t * dY - b;
                    c = t * dZ - c;

                    double d2 = a * a + b * b + c * c;
                    if (d2 < minDistSquared)
                    {
                        minDistSquared = d2;
                        // Interpolate distance along the segment
                        closestDistance = distanceBuffer[i - 1] + t * (distanceBuffer[i] - distanceBuffer[i - 1]);
                        segmentIndex = i - 1;
                    }
                }
            }

            return minDistSquared;
        }

        public static double DistancePointToStripSquared(IList<Vec3D> lineStrip, Vec3D p)
        {
            Vec3D s = lineStrip[0];
            double dX = p.X - s.X;
            double dY = p.Y - s.Y;
            double dZ = p.Z - s.Z;

            double minDistSquared = dX * dX + dY * dY + dZ * dZ;

            int l = lineStrip.Count;
            for (int i = 1; i < l; ++i)
            {
                s = lineStrip[i - 1];
                Vec3D e = lineStrip[i];

                dX = e.X - s.X;
                dY = e.Y - s.Y;
                dZ = e.Z - s.Z;

                double a = p.X - s.X; double b = p.Y - s.Y; double c = p.Z - s.Z;
                double t = (a * dX + b * dY + c * dZ) / (dX * dX + dY * dY + dZ * dZ);

                if (t > 1)
                {
                    dX = p.X - e.X;
                    dY = p.Y - e.Y;
                    dZ = p.Z - e.Z;
                    double d2 = dX * dX + dY * dY + dZ * dZ;
                    if (d2 < minDistSquared)
                        minDistSquared = d2;
                }
                else if (t >= 0)
                {
                    a = t * dX - a;
                    b = t * dY - b;
                    c = t * dZ - c;

                    double d2 = a * a + b * b + c * c;
                    if (d2 < minDistSquared)
                        minDistSquared = d2;
                }
            }

            //double debug = Debug(lineStrip, p);

            return minDistSquared;
        }
    }
}
