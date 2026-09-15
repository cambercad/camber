using GeoCore;


namespace Curves
{
    public class Line2D : Curve2D
    {
        protected Vec2D _start; //public Vector2d Start { get { return _start; } }
        protected Vec2D _end; //public Vector2d End { get { return _end; } }

        public Vec2D DirectionNormalized { get { return (_end - _start).Normalized(); } }
        public override double Length() { return (_end - _start).Length();  }
        public Vec2D Center { get { return 0.5 * (_start + _end); } }



        protected Line2D() { }
        public Line2D(Vec2D start, Vec2D end, CurveFlags curveFlags = CurveFlags.None)
        {
            _start = start;
            _end = end;
            base.Flags = curveFlags;
        }

        public Line2D(Line2D l)
        {
            _start = l._start;
            _end = l._end;
            base.Flags = l.Flags;
        }

        public Line2D(double startX, double startY, double endX, double endY)
            : this(new Vec2D(startX, startY), new Vec2D(endX, endY))
        { }

        public void SetStart(Vec2D start) { _start = start; }
        public void SetEnd(Vec2D end) { _end = end; }

        public override List<Vec2D> ToReferencePoints()
        {
            return new List<Vec2D>() { StartPosition, EndPosition };
        }

        public override void UpdateFromReferencePoints(List<Vec2D> referencePoints)
        {
            _start = referencePoints[0];
            _end = referencePoints[1];
        }

        /// <summary>
        /// Returns defining points: [start, end]
        /// </summary>
        public List<Vec2D> ToPoints()
        {
            return new List<Vec2D> { _start, _end };
        }

        /// <summary>
        /// Creates a Line2D from defining points: [start, end]
        /// </summary>
        public static Line2D FromPoints(List<Vec2D> points, string name = null, CurveFlags flags = CurveFlags.None)
        {
            if (points == null || points.Count < 2)
                throw new ArgumentException("Line2D requires 2 points: [start, end]");
            var line = new Line2D(points[0], points[1], flags);
            line.Name = name;
            return line;
        }

        public static Line2D GetTranslated(Line2D line, Vec2D translation)
        {
            return new Line2D(line.StartPosition + translation, line.EndPosition + translation);
        }


        //https://en.wikipedia.org/wiki/Tangent_lines_to_circles
        //Does only return 2 out of 4 solutions at the moment (no "cross solutions")
        public static Line2D[] TangentialToTwoCircles(Circle2D circleStart, Circle2D circleEnd)
        {
            if (circleStart.Radius < circleEnd.Radius)
            {
                double r = circleEnd.Radius - circleStart.Radius;
                Line2D[] result = TangentialToCircleThroughPoint(new Circle2D(circleEnd.Center, circleEnd.Radius - r), circleStart.Center);

                Vec2D offset = r * (result[0].EndPosition - circleEnd.Center).Normalized();
                result[0] = new Line2D(result[0].StartPosition + offset, result[0].EndPosition + offset);
                offset = r * (result[1].EndPosition - circleEnd.Center).Normalized();
                result[1] = new Line2D(result[1].StartPosition + offset, result[1].EndPosition + offset);
                return result;
            }
            else
            {
                double r = circleStart.Radius - circleEnd.Radius;
                Line2D[] result = TangentialToCircleThroughPoint(new Circle2D(circleStart.Center, circleStart.Radius - r), circleEnd.Center);

                Vec2D offset = r * (result[0].EndPosition - circleStart.Center).Normalized();
                result[0] = new Line2D(result[0].EndPosition + offset, result[0].StartPosition + offset);
                offset = r * (result[1].EndPosition - circleStart.Center).Normalized();
                result[1] = new Line2D(result[1].EndPosition + offset, result[1].StartPosition + offset);
                return result;
            }
        }

        public static Line2D[] TangentialToCircleThroughPoint(Circle2D circle, Vec2D pointOnLine)
        {
            double pointToCenterLengthSquared = (circle.Center - pointOnLine).LengthSquared();
            double radius = Math.Sqrt(pointToCenterLengthSquared - circle.Radius * circle.Radius);

            Vec2D p1, p2;
            if (!GeometricAlgorithms.CircleCircleIntersection(circle.Center, circle.Radius, pointOnLine, radius, out p1, out p2))
                throw new Exception();

            return new Line2D[] { new Line2D(pointOnLine, p1), new Line2D(pointOnLine, p2) };
        }


        public Vec2D EvaluatePoint(double uniform)
        {
            return (1 - uniform) * _start + uniform * _end; 
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
        {
            Vec2D dir = _end - _start;
            Vec2D normal = new Vec2D(dir.Y, -dir.X);
            normal.Normalize();

            Vec2D position = (1 - uniform) * _start + uniform * _end;

            return new CurveVertex2D(position, normal, uniform);
        }

        public override List<CurveVertex2D> Tessellate(double tolerance)
        {
            Vec2D dir = _end - _start;
            Vec2D normal = new Vec2D(dir.Y, -dir.X);
            normal.Normalize();
            return new List<CurveVertex2D>() { new CurveVertex2D(_start, normal, 0), new CurveVertex2D(_end, normal, 1) };
        }
        public override List<CurveVertex2D> Tessellate(int numPoints)
        {
            Vec2D dir = _end - _start;
            Vec2D normal = new Vec2D(dir.Y, -dir.X);
            normal.Normalize();
            return new List<CurveVertex2D>() { new CurveVertex2D(_start, normal, 0), new CurveVertex2D(_end, normal, 1) };
        }

        public override Curve2D GetCopy() { return new Line2D(this); }

        public override Curve2D Reverse()
        {
            return new Line2D(_end, _start);
        }
    }
}