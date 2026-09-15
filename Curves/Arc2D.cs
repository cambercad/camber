using GeoCore;


namespace Curves
{
    public class Arc2D : Curve2D
    {
        protected Vec2D _center; public Vec2D Center { get { return _center; } }
        protected double _radius; public double Radius { get { return _radius; } }
        protected double _angleStart; public double AngleStart { get { return _angleStart; } }
        protected double _angleEnd; public double AngleEnd { get { return _angleEnd; } }


        protected Arc2D() { }
        public Arc2D(Vec2D center, double radius, double angleStart, double angleEnd, CurveFlags curveFlags = CurveFlags.None)
        {
            _center = center;
            _radius = radius;
            _angleStart = angleStart;
            _angleEnd = angleEnd;
            base.Flags = curveFlags;
        }

        public Arc2D(Arc2D a)
        {
            _center = a._center;
            _radius = a._radius;
            _angleStart = a._angleStart;
            _angleEnd = a._angleEnd;
            base.Flags = a.Flags;
        }

        public Arc2D(Vec2D start, Vec2D pointOnArc, Vec2D end, CurveFlags curveFlags = CurveFlags.None)
        {
           Initialize(start, pointOnArc, end);
            base.Flags = curveFlags;
        }

        private void Initialize(Vec2D start, Vec2D pointOnArc, Vec2D end)
        {
            Vec2D ab = pointOnArc - start;
            Vec2D bc = end - pointOnArc;
            Vec2D ca = start - end;

            double a = bc.LengthSquared();
            double b = ca.LengthSquared();
            double c = ab.LengthSquared();

            double u = a * (b + c - a);
            double v = b * (c + a - b);
            double w = c * (a + b - c);

            _center = (u * start + v * pointOnArc + w * end) / (u + v + w);

            _angleStart = GetAngle(start, _center);
            _angleEnd = GetAngle(end, _center);

            if (_angleStart > _angleEnd)
                _angleEnd = _angleEnd + 2.0 * Math.PI;

            if ((pointOnArc.X - start.X) * (end.Y - start.Y) - (pointOnArc.Y - start.Y) * (end.X - start.X) < 0)
            {
                _angleStart = _angleStart + 2.0 * Math.PI;
            }

            bool debug = IsAngleOnArc(GetAngle(pointOnArc, _center));
            if (!debug)
            {
                throw new Exception();
            }

            _radius = (start - _center).Length();
        }

        public override List<Vec2D> ToReferencePoints()
        {
            return new List<Vec2D>() { StartPosition, OnCurveCenterPosition, EndPosition };
        }

        public override void UpdateFromReferencePoints(List<Vec2D> referencePoints)
        {
            Initialize(referencePoints[0], referencePoints[1], referencePoints[2]);
        }

        /// <summary>
        /// Returns defining points: [start, midpoint, end]
        /// </summary>
        public List<Vec2D> ToPoints()
        {
            return new List<Vec2D> { StartPosition, OnCurveCenterPosition, EndPosition };
        }

        /// <summary>
        /// Creates an Arc2D from defining points: [start, midpoint, end]
        /// </summary>
        public static Arc2D FromPoints(List<Vec2D> points, string name = null, CurveFlags flags = CurveFlags.None)
        {
            if (points == null || points.Count < 3)
                throw new ArgumentException("Arc2D requires 3 points: [start, midpoint, end]");
            var arc = new Arc2D(points[0], points[1], points[2], flags);
            arc.Name = name;
            return arc;
        }

        public override double Length()
        {
            double angleRange = Math.Abs(_angleEnd - _angleStart);
            return angleRange * _radius;
        }

        public bool IsAngleOnArc(double angle)
        {
            return GeometricAlgorithms.IsAngleInRange(_angleStart, _angleEnd, angle);
        }


        public Arc2D(Vec2D start, Vec2D end, Vec2D center, Vec2D pointOnArcDirection, CurveFlags curveFlags = CurveFlags.None)
            : this(start, /*center + (start - center).Length * pointOnArcDirection.Normalized()*/GetPointOnArc(start, end, center, pointOnArcDirection), end, curveFlags)
        { }

        private static Vec2D GetPointOnArc(Vec2D start, Vec2D end, Vec2D center, Vec2D pointOnArcDirection)
        {
            var dirStart = start - center;
            var dirEnd = end - center;
            var average = dirStart + dirEnd;
            average.Normalize();

            if(Vec2DOps.Dot(pointOnArcDirection, average)<0)
            {
                average = -average;
            }

            return center + dirStart.Length() * average;
        }

        public Arc2D(Vec2D start, Vec2D end, Vec2D center, bool shorter, CurveFlags curveFlags = CurveFlags.None)
            : this(start, center + (shorter ? 1 : -1) * (start - center).Length() * (0.5 * (start + end) - center).Normalized(), end, curveFlags)
        { }

        public static Arc2D GetArcWithDifferentRadius(Arc2D arc, double newRadius)
        {
            return new Arc2D(arc.Center, newRadius, arc.AngleStart, arc.AngleEnd);
        }

        //public static Arc2D From3Points(Vector2d p1, Vector2d p2, Vector2d v)
        //{
        //    Vector2d d1 = p1 - v;
        //    Vector2d d2 = p2 - v;

        //    double x = d1.X;
        //    d1.X = -d1.Y;
        //    d1.Y = x;
        //    x = d2.X;
        //    d2.X = -d2.Y;
        //    d2.Y = x;

        //    Vector2d o1 = 0.5 * (v + p1);
        //    Vector2d o2 = 0.5 * (v + p2);

        //    double s, t;
        //    if (GeometricAlgorithms.IntersectionLineLine(o1, d1, o2, d2, 1e-8, out s, out t))
        //    {
        //        //Vector2d center = o1 + s * d1;
        //        double cX = o1.X + s * d1.X;
        //        double cY = o1.Y + s * d1.Y;

        //        double dX = cX - p1.X;
        //        double dY = cY - p1.Y;

        //        arc.Center.SetXY(cX, cY);
        //        arc.SetRadius(Math.Sqrt(dX * dX + dY * dY));
        //        arc.SetAngleStart(CArc2d.GetAngle(p1.X, p1.Y, cX, cY));
        //        arc.SetAngleEnd(CArc2d.GetAngle(p2.X, p2.Y, cX, cY));

        //        if (arc.AngleStart > arc.AngleEnd)
        //            arc.SetAngleEnd(arc.AngleEnd + 2.0 * Math.PI);

        //        if ((v.X - p1.X) * (p2.Y - p1.Y) - (v.Y - p1.Y) * (p2.X - p1.X) < 0)
        //        {
        //            arc.SetAngleStart(arc.AngleStart + 2.0 * Math.PI);
        //        }
        //    }
        //}

        public static double GetAngle(Vec2D p, Vec2D center)
        {
            return Math.Atan2(p.Y - center.Y, p.X - center.X);
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
        {
            double angle = _angleStart + (_angleEnd - _angleStart) * uniform;
            Vec2D normal = new Vec2D(Math.Cos(angle), Math.Sin(angle));
            Vec2D position = new Vec2D(_center.X + normal.X * _radius, _center.Y + normal.Y * _radius);
            return new CurveVertex2D(position, normal, uniform);
        }

        private Vec2D GetPointFromAngle(double angle)
        {
            Vec2D normal = new Vec2D(Math.Cos(angle), Math.Sin(angle));
            return new Vec2D(_center.X + normal.X * _radius, _center.Y + normal.Y * _radius);
        }

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
        {
            double a = _angleEnd - _angleStart;
            if (a < 0)
            {
                //Debug
                a = Math.Abs(a);
            }
            int numSteps = Circle2D.NumberOfCirclePoints(maxDeviation, _radius, angle: a);

           return Tessellate(numSteps);
        }
        public override List<CurveVertex2D> Tessellate(int numSteps)
        {
            List<CurveVertex2D> points = new List<CurveVertex2D>(numSteps);
            double scaling = 1.0 / (numSteps - 1);
            for (int i = 0; i < numSteps; ++i)
            {
                var v = EvaluateVertex(i * scaling);                
                points.Add(v);
            }

            return points;
        }

        private Vec2D ProjectPointOntoArcCircle(Vec2D p)
        {
            double dist2 = Vec2DOps.DistanceSquared(p, _center);
            if (Math.Abs(dist2 - _radius * _radius) < 1e-24)
                return p;

            Vec2D d = p - _center;
            d.Normalize();
            return _center + _radius * d;
        }

        public void SetStart(Vec2D start)
        {
            start = ProjectPointOntoArcCircle(start);

            double halfAngle = 0.5 * (_angleStart + _angleEnd);
            Vec2D pointOnArc = GetPointFromAngle(halfAngle);
            var pointOnArcDirection = pointOnArc - GeometricAlgorithms.ProjectPointOntoLine(pointOnArc, StartPosition, EndPosition - StartPosition);

            //Arc2D debug = new Arc2D(StartPosition, EndPosition, _center, pointOnArcDirection);

            Arc2D tmp = new Arc2D(start, EndPosition, _center, pointOnArcDirection);
            _angleStart = tmp._angleStart;
            _angleEnd = tmp._angleEnd;

            //double angle = GetAngle(start, _center);
            //if (_angleStart < _angleEnd)
            //{
            //    while (angle > _angleEnd)
            //        angle -= 2.0 * Math.PI;
            //    _angleStart = angle;
            //}
            //else
            //{
            //    while (angle < _angleEnd)
            //        angle += 2.0 * Math.PI;
            //    _angleStart = angle;
            //}
        }

        public void SetEnd(Vec2D end)
        {
            end = ProjectPointOntoArcCircle(end);

            double halfAngle = 0.5 * (_angleStart + _angleEnd);
            Vec2D pointOnArc = GetPointFromAngle(halfAngle);
            var pointOnArcDirection = pointOnArc - GeometricAlgorithms.ProjectPointOntoLine(pointOnArc, StartPosition, EndPosition - StartPosition);

            Arc2D tmp = new Arc2D(StartPosition, end, _center, pointOnArcDirection);
            _angleStart = tmp._angleStart;
            _angleEnd = tmp._angleEnd;

            //double angle = GetAngle(end, _center);
            //if (_angleStart < _angleEnd)
            //{
            //    while (angle > _angleStart)
            //        angle -= 2.0 * Math.PI;
            //    _angleEnd = angle;
            //}
            //else
            //{
            //    while (angle < _angleStart)
            //        angle += 2.0 * Math.PI;
            //    _angleEnd = angle;
            //}

            /*double angle = GetAngle(end, _center);
            while (angle < _angleEnd)
                angle += 2.0 * Math.PI;
            _angleEnd = angle;*/
        }

        public override Curve2D GetCopy() { return new Arc2D(this); }

        public override Curve2D Reverse()
        {
            // Use the 3-point constructor to properly reverse the arc
            Vec2D start = StartPosition;
            Vec2D end = EndPosition;
            Vec2D mid = OnCurveCenterPosition;
            
            // Reverse by swapping start and end
            return new Arc2D(end, mid, start);
        }

        public void FlipDirection()
        {
            double tmp = _angleStart;
            _angleStart = _angleEnd;
            _angleEnd = tmp; // +2 * Math.PI;
        }
    }
}