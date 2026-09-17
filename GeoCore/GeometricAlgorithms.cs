namespace GeoCore
{
    public static class GeometricAlgorithms
    {//Book Real-Time Collision Detection Page 183
        public static bool IntersectBoxLineSegment(double boxMinX, double boxMaxX,
            double boxMinY, double boxMaxY, Vec2D p0, Vec2D p1, double eps = 1e-12)
        {
            //Point c = (b.min + b.max) * 0.5f; // Box center-point
            //Vector e = b.max - c; // Box halflength extents
            //Point m = (p0 + p1) * 0.5f; // Segment midpoint
            //Vector d = p1 - m; // Segment halflength vector
            //m = m - c; // Translate box and segment to origin
            //           // Try world coordinate axes as separating axes

            double eX = boxMaxX - boxMinX;
            double eY = boxMaxY - boxMinY;
            double dX = p1.X - p0.X;
            double dY = p1.Y - p0.Y;
            double mX = p0.X + p1.X - boxMinX - boxMaxX;
            double mY = p0.Y + p1.Y - boxMinY - boxMaxY;

            double adx = Math.Abs(dX);
            if (Math.Abs(mX) > eX + adx) return false;
            double ady = Math.Abs(dY);
            if (Math.Abs(mY) > eY + ady) return false;

            // Add in an epsilon term to counteract arithmetic errors when segment is
            // (near) parallel to a coordinate axis (see text for detail)
            adx += eps; ady += eps;
            // Try cross products of segment direction vector with coordinate axes
            if (Math.Abs(mX * dY - mY * dX) > eX * ady + eY * adx) return false;
            // No separating axis found; segment must be overlapping AABB
            return true;
        }

        public static double SquaredDistancePointLine(Vec3D p, Vec3D o, Vec3D d, out double s)
        {
            double x = p.X - o.X;
            double y = p.Y - o.Y;
            double z = p.Z - o.Z;
            s = (x * d.X + y * d.Y + z * d.Z) / (d.X * d.X + d.Y * d.Y + d.Z * d.Z);

            //Compute the coordinates of the intersection point and subtract p
            //a = o.X + s * d.X - p.X;
            //b = o.Y + s * d.Y - p.Y;
            x = s * d.X - x;
            y = s * d.Y - y;
            z = s * d.Z - z;

            return x * x + y * y + z * z;
        }


        //Taken from http://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf
        public static double GetCurveParameter(double arcLength, double totalArcLength, Func<double, double> arcLengthEval, Func<double, double> speedEval) // 0 <= s <= L, output is t
        {
            double tmin = 0;
            double tmax = 1;
            double L = totalArcLength;
            int imax = 1000;
            double epsilon = 1e-8;

            // Initial guess for Newton's method.
            double t = tmin + arcLength * (tmax - tmin) / L;
            // Initial root-bounding interval for bisection.
            double lower = tmin, upper = tmax;
            for (int i = 0; i < imax; i++) // `imax' is application-specified
            {
                double F = arcLengthEval(t) - arcLength;
                if (Math.Abs(F) < epsilon) // `epsilon' is application-specified
                {
                    // |F(t)| is close enough to zero, report t as the time at
                    // which length s is attained.
                    return t;
                }
                // Generate a candidate for Newton's method.
                double DF = speedEval(t);
                double tCandidate = t - F / DF;
                // Update the root-bounding interval and test for containment of
                // the candidate.
                if (F > 0)
                {
                    upper = t;
                    if (tCandidate <= lower)
                    {
                        // Candidate is outside the root-bounding interval. Use
                        // bisection instead.
                        t = 0.5 * (upper + lower);
                    }
                    else
                    {
                        // There is no need to compare to 'upper' because the tangent
                        // line has positive slope, guaranteeing that the t-axis
                        // intercept is smaller than 'upper'.
                        t = tCandidate;
                    }
                }
                else
                {
                    lower = t;
                    if (tCandidate >= upper)
                    {
                        // Candidate is outside the root-bounding interval. Use
                        // bisection instead.
                        t = 0.5 * (upper + lower);
                    }
                    else
                    {
                        // There is no need to compare to 'lower' because the tangent
                        // line has positive slope, guaranteeing that the t-axis
                        // intercept is larger than 'lower'.
                        t = tCandidate;
                    }
                }
            }
            // A root was not found according to the specified number of iterations
            // and tolerance. You might want to increase iterations or tolerance or
            // integration accuracy. However, in this application it is likely that
            // the time values are oscillating, due to the limited numerical
            // precision of 32-bit doubles. It is safe to use the last computed time.
            return t;
        }

        //https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/smoothstep.xhtml
        public static double SmoothStep(double edge0, double edge1, double x)
        {
            double t = Algorithms.Clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
            return t * t * (3.0 - 2.0 * t);
        }

        //From book 'Real-Time Collision Detection'
        //Not yet optimized for speed
        // Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and
        // S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
        // distance between between S1(s) and S2(t)
        // Real-Time Collision Detection Page 149
        public static double ClosestPtSegmentSegment(Vec3D p1, Vec3D q1, Vec3D p2, Vec3D q2,
            out double s, out double t, out Vec3D c1, out Vec3D c2, double eps)
        {
            Vec3D d1 = q1 - p1; // Direction vector of segment S1
            Vec3D d2 = q2 - p2; // Direction vector of segment S2
            Vec3D r = p1 - p2;
            double a = d1.LengthSquared(); // Squared length of segment S1, always nonnegative
            double e = d2.LengthSquared(); // Squared length of segment S2, always nonnegative
            double f = Vec3DOps.Dot(d2, r);
            // Check if either or both segments degenerate into points
            if (a <= eps && e <= eps)
            {
                // Both segments degenerate into points
                s = t = 0.0;
                c1 = p1;
                c2 = p2;
                Vec3D diff = c1 - c2;
                return diff.LengthSquared();
            }
            if (a <= eps)
            {
                // First segment degenerates into a point
                s = 0.0;
                t = f / e; // s = 0 => t = (b*s + f) / e = f / e
                t = Clamp(t, 0.0, 1.0);
            }
            else
            {
                double c = Vec3DOps.Dot(d1, r);
                if (e <= eps)
                {
                    // Second segment degenerates into a point
                    t = 0.0;
                    s = Clamp(-c / a, 0.0, 1.0); // t = 0 => s = (b*t - c) / a = -c / a
                }
                else
                {
                    // The general nondegenerate case starts here
                    double b = Vec3DOps.Dot(d1, d2);
                    double denom = a * e - b * b; // Always nonnegative
                                                  // If segments not parallel, compute closest point on L1 to L2 and
                                                  // clamp to segment S1. Else pick arbitrary s (here 0)
                    if (denom != 0.0)
                    {
                        s = Clamp((b * f - c * e) / denom, 0.0, 1.0);
                    }
                    else s = 0.0;
                    // Compute point on L2 closest to S1(s) using
                    // t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e
                    t = (b * s + f) / e;
                    // If t in [0,1] done. Else clamp t, recompute s for the new value
                    // of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a
                    // and clamp s to [0, 1]
                    if (t < 0.0)
                    {
                        t = 0.0;
                        s = Clamp(-c / a, 0.0, 1.0);
                    }
                    else if (t > 1.0)
                    {
                        t = 1.0;
                        s = Clamp((b - c) / a, 0.0, 1.0);
                    }
                }
            }
            c1 = p1 + d1 * s;
            c2 = p2 + d2 * t;
            return Vec3DOps.Dot(c1 - c2, c1 - c2);
        }
        private static double Clamp(double n, double min, double max)
        {
            if (n < min) return min;
            if (n > max) return max;
            return n;
        }


        public static Vec3D ClosestPointOnLineSegment(Vec3D p, Vec3D start, Vec3D end, out double distSquared)
        {
            //Vector2d dir = end - start;
            double dirX = end.X - start.X;
            double dirY = end.Y - start.Y;
            double dirZ = end.Z - start.Z;

            //Vector2d originToPoint = p - start;
            double originToPointX = p.X - start.X;
            double originToPointY = p.Y - start.Y;
            double originToPointZ = p.Z - start.Z;

            //double t = Vector2d.Dot(originToPoint, dir);
            double t = dirX * originToPointX + dirY * originToPointY + dirZ * originToPointZ;

            double dirLengthSquared = dirX * dirX + dirY * dirY + dirZ * dirZ;

            //double lengthSquared;
            if (t >= 0 && t <= dirLengthSquared)
            {
                t = t / dirLengthSquared;
                Vec3D pt = new Vec3D(start.X + t * dirX,
                    start.Y + t * dirY,
                    start.Z + t * dirZ);

                double dx = pt.X - p.X;
                double dy = pt.Y - p.Y;
                double dz = pt.Z - p.Z;
                distSquared = dx * dx + dy * dy + dz * dz;

                return pt;
            }
            else if (t < 0)
            {
                distSquared = originToPointX * originToPointX + originToPointY * originToPointY + originToPointZ * originToPointZ;
                return start;
            }
            else
            {
                double dx = end.X - p.X;
                double dy = end.Y - p.Y;
                double dz = end.Z - p.Z;
                distSquared = dx * dx + dy * dy + dz * dz;

                return end;
            }
        }

        public static bool ClosestPointsLineLine(Vec3D o1, Vec3D d1, Vec3D o2, Vec3D d2, double eps, out double s, out double t)
        {
            return ClosestPoints(o1, d1, o2, d2, eps, out s, out t);
        }

        // Real-Time Collision Detection Page 147              
        public static bool ClosestPoints(Vec3D o1, Vec3D d1, Vec3D o2, Vec3D d2, double eps, out double s, out double t)
        {
            double rX = o1.X - o2.X;
            double rY = o1.Y - o2.Y;
            double rZ = o1.Z - o2.Z;

            double a = d1.X * d1.X + d1.Y * d1.Y + d1.Z * d1.Z;
            double b = d1.X * d2.X + d1.Y * d2.Y + d1.Z * d2.Z;
            double c = d1.X * rX + d1.Y * rY + d1.Z * rZ;
            double e = d2.X * d2.X + d2.Y * d2.Y + d2.Z * d2.Z;
            double f = d2.X * rX + d2.Y * rY + d2.Z * rZ;

            double d = a * e - b * b;
            if (d < eps && d > -eps)
            {
                s = 0;
                t = 0;
                return false; //Lines are parallel
            }

            double invD = 1.0f / d;
            s = (b * f - c * e) * invD;
            t = (a * f - b * c) * invD;
            return true;
        }

        public static Vec3D TriangleCenter(Tri t, List<Vec3D> pos)
        {
            return (1.0 / 3.0) * (pos[t.A] + pos[t.B] + pos[t.C]);
        }

        public static Vec3D ProjectPointOntoLine(Vec3D p, Vec3D o, Vec3D d)
        {
            double x = p.X - o.X;
            double y = p.Y - o.Y;
            double z = p.Z - o.Z;
            double t = (x * d.X + y * d.Y + z * d.Z) / (d.X * d.X + d.Y * d.Y + d.Z * d.Z);
            return new Vec3D(o.X + t * d.X, o.Y + t * d.Y, o.Z + t * d.Z);
        }

        public static bool IsPolygonCCW(List<Vec2D> polygon)
        {
            if (polygon == null || polygon.Count < 3)
                throw new ArgumentException("A polygon must have at least 3 vertices.");

            double sum = 0;
            int n = polygon.Count;

            for (int i = 0; i < n; i++)
            {
                Vec2D current = polygon[i];
                Vec2D next = polygon[(i + 1) % n]; // wrap around to first vertex
                sum += (next.X - current.X) * (next.Y + current.Y);
            }

            // If sum > 0, polygon is clockwise; CCW if sum < 0
            return sum < 0;
        }

        public static double SignedDistancePointPlane(Vec3D point, Vec3D normal, Vec3D pointOnPlane)
        {
            return point.X * normal.X + point.Y * normal.Y + point.Z * normal.Z - Vec3DOps.Dot(pointOnPlane, normal);
        }

        public static double SignedDistancePointPlane(Vec3D point, Vec3D normal, double planeD)
        {
            return point.X * normal.X + point.Y * normal.Y + point.Z * normal.Z + planeD;
        }

        /// <summary>Squared distance to the finite segment, including coincident endpoints.</summary>
        public static double DistancePointSegmentSquared(Vec3D point, Vec3D start, Vec3D end)
        {
            var chord = end - start;
            var delta = point - start;
            double length2 = chord.Dot(chord);
            double t = length2 == 0 ? 0 : Math.Clamp(delta.Dot(chord) / length2, 0, 1);
            var residual = delta - t * chord;
            return residual.Dot(residual);
        }

        /// <summary>Squared distance to the finite segment, including coincident endpoints.</summary>
        public static double DistancePointSegmentSquared(Vec2D point, Vec2D start, Vec2D end)
        {
            var chord = end - start;
            var delta = point - start;
            double length2 = chord.X * chord.X + chord.Y * chord.Y;
            double t = length2 == 0 ? 0 : Math.Clamp((delta.X * chord.X + delta.Y * chord.Y) / length2, 0, 1);
            var residual = delta - t * chord;
            return residual.X * residual.X + residual.Y * residual.Y;
        }

        public static double DistancePointLineSquared(Vec3D p, Vec3D o, Vec3D d)
        {
            double t;
            return DistancePointLineSquared(p, o, d, out t);
        }


        public static double DistancePointLineSquared(Vec3D p, Vec3D o, Vec3D d, out double t)
        {
            double a = p.X - o.X; double b = p.Y - o.Y; double c = p.Z - o.Z;
            t = (a * d.X + b * d.Y + c * d.Z) / (d.X * d.X + d.Y * d.Y + d.Z * d.Z);

#if DEBUG
            if (double.IsNaN(t))
                throw new Exception();
#endif

            //Compute the coordinates of the intersection point and subtract p
            //a = o.X + t * d.X - p.X;
            //b = o.Y + t * d.Y - p.Y;
            //c = o.Z + t * d.Z - p.Z;
            a = t * d.X - a;
            b = t * d.Y - b;
            c = t * d.Z - c;

            return a * a + b * b + c * c;
        }

        public static double DistancePointLineSquared(Vec2D p, Vec2D o, Vec2D d)
        {
            double t;
            return DistancePointLineSquared(p, o, d, out t);
        }

        public static double DistancePointLineSquared(Vec2D p, Vec2D o, Vec2D d, out double t)
        {
            double a = p.X - o.X;
            double b = p.Y - o.Y;
            t = (a * d.X + b * d.Y) / (d.X * d.X + d.Y * d.Y);
            double denom = d.X * d.X + d.Y * d.Y;

            //Compute the coordinates of the intersection point and subtract p
            a = t * d.X - a;
            b = t * d.Y - b;

            return a * a + b * b;
        }

        //http://paulbourke.net/geometry/circlesphere/
        public static bool CircleCircleIntersection(Vec2D circleCenterA, double radiusA,
            Vec2D circleCenterB, double radiusB, out Vec2D p1, out Vec2D p2)
        {
            Vec2D d = circleCenterB - circleCenterA;

            double dist = d.Length();
            if (dist > radiusA + radiusB)
            {
                //Circles are separate
                p1 = default(Vec2D);
                p2 = default(Vec2D);
                return false;
            }
            if (dist < Math.Abs(radiusB - radiusA))
            {
                //One circle is fully contained inside the oder
                p1 = default(Vec2D);
                p2 = default(Vec2D);
                return false;
            }

            double a = (radiusA * radiusA - radiusB * radiusB + dist * dist) / (2 * dist);

            double h = Math.Sqrt(Math.Max(0, radiusA * radiusA - a * a));

            d = d * (1.0 / dist);
            double x2 = circleCenterA.X + a * d.X;
            double y2 = circleCenterA.Y + a * d.Y;

            p1 = new Vec2D(x2 + h * d.Y, y2 - h * d.X);
            p2 = new Vec2D(x2 - h * d.Y, y2 + h * d.X);
            return true;
        }

        public static Vec2D ProjectPointOntoLine(Vec2D p, Vec2D o, Vec2D d)
        {
            double x = p.X - o.X;
            double y = p.Y - o.Y;
            double t = (x * d.X + y * d.Y) / (d.X * d.X + d.Y * d.Y);
            return new Vec2D(o.X + t * d.X, o.Y + t * d.Y);
        }

        public static Vec3D ProjectPointOntoPlane(Vec3D p, Vec3D n, Vec3D pointOnPlane)
        {
            return ProjectPointOntoPlane(p, n, -(n.X * pointOnPlane.X + n.Y * pointOnPlane.Y + n.Z * pointOnPlane.Z));
        }

        public static Vec3D ProjectPointOntoPlane(Vec3D p, Vec3D n, double d)
        {
            //Plane equation used here is Vector3d.Dot(pointOnPlane, n) + d = 0   d is on the left side!       
            double t = (n.X * p.X + n.Y * p.Y + n.Z * p.Z + d) / (n.X * n.X + n.Y * n.Y + n.Z * n.Z);  //(Dot(n, p) + d) / Dot(n, n);          

            p.X = p.X - t * n.X;
            p.Y = p.Y - t * n.Y;
            p.Z = p.Z - t * n.Z;

            return p;
            //return p - t * n;
        }


        public const double TWO_PI = 2.0 * Math.PI;

        //Requires rangeStart < rangeEnd
        public static bool IsAngleInRange(double rangeStart, double rangeEnd, double angle)
        {
            if (rangeStart > rangeEnd)
            {
                while (angle < rangeEnd)
                    angle += TWO_PI;

                while (angle > rangeStart)
                    angle -= TWO_PI;

                return angle >= rangeEnd && angle <= rangeStart;
            }
            else
            {
                while (angle < rangeStart)
                    angle += TWO_PI;

                while (angle > rangeEnd)
                    angle -= TWO_PI;

                return angle >= rangeStart && angle <= rangeEnd;
            }
        }

        public static bool IntersectionLineLine(Vec2D o1, Vec2D d1, Vec2D o2, Vec2D d2, double eps, out double s, out double t)
        {
            double denom = d1.X * d2.Y - d1.Y * d2.X;

            if (denom > -eps && denom < eps)
            {
                s = 0; t = 0;
                return false;
            }

            double invDenom = 1.0 / denom;

            //u(s) = _start + _dir*s     this line
            //v(t) = start  + dir *t     the line described by the arguments            
            s = ((o1.Y - o2.Y) * d2.X + (o2.X - o1.X) * d2.Y) * invDenom;
            t = ((o1.Y - o2.Y) * d1.X + (o2.X - o1.X) * d1.Y) * invDenom;
            return true;
        }


        public static bool LineCircleIntersection(Vec2D o, Vec2D d,
            Vec2D circleCenter, double circleRadius, out double t1, out double t2)
        {
            //d.Normalize();

            double l2 = d.X * d.X + d.Y * d.Y;

            double a = circleCenter.X - o.X; double b = circleCenter.Y - o.Y;
            double t = (a * d.X + b * d.Y) / l2;

            a = t * d.X - a;
            b = t * d.Y - b;

            double distSquared = a * a + b * b;
            double r2 = circleRadius * circleRadius;
            if (distSquared > r2)
            {
                t1 = 0;
                t2 = 0;
                return false;
            }

            double scaling = 1.0 / Math.Sqrt(l2);
            double offset = Math.Sqrt(r2 - distSquared);
            t1 = t + offset * scaling;
            t2 = t - offset * scaling;
            return true;
        }


        public static bool IntersectSegmentSegment(Vec2D s1, Vec2D e1, Vec2D s2, Vec2D e2,
          out double s, out double t, out Vec2D p, double eps)
        {
            s = 0; t = 0;
            p = default(Vec2D);

            Vec2D d1 = e1 - s1;
            Vec2D d2 = e2 - s2;

            double denom = d1.X * d2.Y - d1.Y * d2.X;

            if (denom > -eps && denom < eps)
                return false;

            double invDenom = 1.0 / denom;

            //u(s) = _start + _dir*s     this line
            //v(t) = start  + dir *t     the line described by the arguments            
            s = ((s1.Y - s2.Y) * d2.X + (s2.X - s1.X) * d2.Y) * invDenom;
            if (s < 0 || s > 1)
                return false;

            t = ((s1.Y - s2.Y) * d1.X + (s2.X - s1.X) * d1.Y) * invDenom;
            if (t < 0 || t > 1)
                return false;

            p = s1 + s * d1;

            return true;
        }


        //Requires d to be normalized
        //Requires arcAngleStart < arcAngleEnd
        public static bool LineSegmentArcIntersection(Vec2D o, Vec2D end,
            Vec2D arcCenter, double arcRadius, double arcAngleStart, double arcAngleEnd, out double t)
        {
            t = 0;
            Vec2D d = end - o;

            double t1, t2;
            if (LineCircleIntersection(o, d, arcCenter, arcRadius, out t1, out t2))
            {
                Vec2D p1 = o + t1 * d;
                double angle1 = Math.Atan2(p1.Y - arcCenter.Y, p1.X - arcCenter.X);

                Vec2D p2 = o + t2 * d;
                double angle2 = Math.Atan2(p2.Y - arcCenter.Y, p2.X - arcCenter.X);

                bool b1 = t1 >= 0 && t1 <= 1 && IsAngleInRange(arcAngleStart, arcAngleEnd, angle1);
                bool b2 = t2 >= 0 && t2 <= 1 && IsAngleInRange(arcAngleStart, arcAngleEnd, angle2);

                if (b1 != b2)
                {
                    if (b1)
                    {
                        t = t1;
                        return true;
                    }
                    else
                    {
                        t = t2;
                        return true;
                    }
                }
                else if (b1)
                {
                    throw new NotImplementedException();
                }
                else
                    return false;
            }
            else
            {
                return false;
            }
        }

        public static Vec3D ComputeTriangleNormal(Vec3D a, Vec3D b, Vec3D c, bool normalize = true)
        {
            Vec3D normal = Vec3DOps.Cross(b - a, c - a);
            if (normalize)
                normal.Normalize();
            return normal;
        }


        //http://www.geometrictools.com/GTEngine/Include/Mathematics/GteDistPointTriangleExact.h
        public static double DistancePointTriangleSquared(Vec3D triA, Vec3D triB, Vec3D triC, Vec3D point, out double u, out double v, out double w)
        {
            Vec3D diff = point - triA;
            Vec3D edge0 = triB - triA;
            Vec3D edge1 = triC - triA;
            double a00 = Vec3DOps.Dot(edge0, edge0);
            double a01 = Vec3DOps.Dot(edge0, edge1);
            double a11 = Vec3DOps.Dot(edge1, edge1);
            double b0 = -Vec3DOps.Dot(diff, edge0);
            double b1 = -Vec3DOps.Dot(diff, edge1);
            double det = a00 * a11 - a01 * a01;
            double t0 = a01 * b1 - a11 * b0;
            double t1 = a01 * b0 - a00 * b1;

            if (t0 + t1 <= det)
            {
                if (t0 < 0)
                {
                    if (t1 < 0)  // region 4
                    {
                        if (b0 < 0)
                        {
                            t1 = 0;
                            if (-b0 >= a00)  // V0
                            {
                                t0 = 1;
                            }
                            else  // E01
                            {
                                t0 = -b0 / a00;
                            }
                        }
                        else
                        {
                            t0 = 0;
                            if (b1 >= 0)  // V0
                            {
                                t1 = 0;
                            }
                            else if (-b1 >= a11)  // V2
                            {
                                t1 = 1;
                            }
                            else  // E20
                            {
                                t1 = -b1 / a11;
                            }
                        }
                    }
                    else  // region 3
                    {
                        t0 = 0;
                        if (b1 >= 0)  // V0
                        {
                            t1 = 0;
                        }
                        else if (-b1 >= a11)  // V2
                        {
                            t1 = 1;
                        }
                        else  // E20
                        {
                            t1 = -b1 / a11;
                        }
                    }
                }
                else if (t1 < 0)  // region 5
                {
                    t1 = 0;
                    if (b0 >= 0)  // V0
                    {
                        t0 = 0;
                    }
                    else if (-b0 >= a00)  // V1
                    {
                        t0 = 1;
                    }
                    else  // E01
                    {
                        t0 = -b0 / a00;
                    }
                }
                else  // region 0, interior
                {
                    double invDet = 1.0 / det;
                    t0 *= invDet;
                    t1 *= invDet;
                }
            }
            else
            {
                double tmp0, tmp1, numer, denom;

                if (t0 < 0)  // region 2
                {
                    tmp0 = a01 + b0;
                    tmp1 = a11 + b1;
                    if (tmp1 > tmp0)
                    {
                        numer = tmp1 - tmp0;
                        denom = a00 - ((double)2) * a01 + a11;
                        if (numer >= denom)  // V1
                        {
                            t0 = 1;
                            t1 = 0;
                        }
                        else  // E12
                        {
                            t0 = numer / denom;
                            t1 = 1 - t0;
                        }
                    }
                    else
                    {
                        t0 = 0;
                        if (tmp1 <= 0)  // V2
                        {
                            t1 = 1;
                        }
                        else if (b1 >= 0)  // V0
                        {
                            t1 = 0;
                        }
                        else  // E20
                        {
                            t1 = -b1 / a11;
                        }
                    }
                }
                else if (t1 < 0)  // region 6
                {
                    tmp0 = a01 + b1;
                    tmp1 = a00 + b0;
                    if (tmp1 > tmp0)
                    {
                        numer = tmp1 - tmp0;
                        denom = a00 - 2.0 * a01 + a11;
                        if (numer >= denom)  // V2
                        {
                            t1 = 1;
                            t0 = 0;
                        }
                        else  // E12
                        {
                            t1 = numer / denom;
                            t0 = 1 - t1;
                        }
                    }
                    else
                    {
                        t1 = 0;
                        if (tmp1 <= 0)  // V1
                        {
                            t0 = 1;
                        }
                        else if (b0 >= 0)  // V0
                        {
                            t0 = 0;
                        }
                        else  // E01
                        {
                            t0 = -b0 / a00;
                        }
                    }
                }
                else  // region 1
                {
                    numer = a11 + b1 - a01 - b0;
                    if (numer <= 0)  // V2
                    {
                        t0 = 0;
                        t1 = 1;
                    }
                    else
                    {
                        denom = a00 - 2.0 * a01 + a11;
                        if (numer >= denom)  // V1
                        {
                            t0 = 1;
                            t1 = 0;
                        }
                        else  // 12
                        {
                            t0 = numer / denom;
                            t1 = 1 - t0;
                        }
                    }
                }
            }

            u = 1 - t0 - t1;
            v = t0;
            w = t1;


            double dx = diff.X - (t0 * edge0.X + t1 * edge1.X);
            double dy = diff.Y - (t0 * edge0.Y + t1 * edge1.Y);
            double dz = diff.Z - (t0 * edge0.Z + t1 * edge1.Z);
            return dx * dx + dy * dy + dz * dz;


            Vec3D closestPoint = triA + t0 * edge0 + t1 * edge1;

            //Debug
            //Vec3D debug = u * triA + v * triB + w * triC;
            //double debugDist = (closestPoint - debug).LengthSquared();

            Vec3D d = point - closestPoint;
            return d.X * d.X + d.Y * d.Y + d.Z * d.Z;
        }
    }
}
