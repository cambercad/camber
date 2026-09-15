using GeoCore;


namespace Curves
{

    public class Circle2D : Curve2D
    {
        protected Vec2D _center; public Vec2D Center { get { return _center; } }
        protected double _radius; public double Radius { get { return _radius; } }
        public double Diameter { get { return 2.0 * _radius; } }


        protected Circle2D() { }
        public Circle2D(double centerX, double centerY, double radius, CurveFlags curveFlags = CurveFlags.None)
        {
            _center = new Vec2D(centerX, centerY);
            _radius = radius;
            base.Flags = curveFlags;
        }

        public Circle2D(Vec2D center, double radius, CurveFlags curveFlags = CurveFlags.None)
        {
            _center = center;
            _radius = radius;
            base.Flags = curveFlags;
        }

        public Circle2D(Circle2D c)
        {
            _center = c._center;
            _radius = c._radius;
            base.Flags = c.Flags;
        }

        public override List<Vec2D> ToReferencePoints()
        {
            return new List<Vec2D>() { _center, StartPosition };
        }

        public override void UpdateFromReferencePoints(List<Vec2D> referencePoints)
        {
            _center = referencePoints[0];
            _radius = (referencePoints[1] - _center).Length();
        }

        /// <summary>
        /// Returns defining points: [center, zeroAnglePoint, quarterPoint]
        /// The zero angle point is at angle 0 (on the positive X direction from center).
        /// The quarter point is at angle 90° (on the positive Y direction from center).
        /// </summary>
        public List<Vec2D> ToPoints()
        {
            Vec2D quarterPoint = EvaluateVertex(0.25).Position;
            return new List<Vec2D> { _center, StartPosition, quarterPoint };
        }

        /// <summary>
        /// Creates a Circle2D from defining points: [center, zeroAnglePoint, quarterPoint]
        /// The radius is derived from the distance between center and zeroAnglePoint.
        /// Note: In 2D the quarterPoint is not strictly needed but is accepted for consistency with Circle3D.
        /// </summary>
        public static Circle2D FromPoints(List<Vec2D> points, string name = null, CurveFlags flags = CurveFlags.None)
        {
            if (points == null || points.Count < 2)
                throw new ArgumentException("Circle2D requires at least 2 points: [center, pointOnCircle]");
            double radius = (points[1] - points[0]).Length();
            var circle = new Circle2D(points[0], radius, flags);
            circle.Name = name;
            return circle;
        }

        public override double Length()
        {
            return (2.0 * Math.PI) * _radius;
        }

        public static double GetAngle(Vec2D point, Vec2D center)
        {
            Vec2D dir = point - center;
            double angle = Math.Atan2(dir.Y, dir.X);
            if (angle < 0)
                angle += 2 * Math.PI;
            return angle;
        }

        public override CurveVertex2D EvaluateVertex(double uniform)
        {
            double angle = 2 * Math.PI * uniform;
            Vec2D normal = new Vec2D(Math.Cos(angle), Math.Sin(angle));
            Vec2D position = new Vec2D(_center.X + normal.X * _radius, _center.Y + normal.Y * _radius);
            return new CurveVertex2D(position, normal, uniform);
        }

        public static int NumberOfCirclePoints(double maxDeviation, double radius, int minNumSegments = 6, double angle = 2 * Math.PI)
        {
            double maxAngleStep = 2 * Math.Acos(1 - maxDeviation / radius);
            int numSteps = Math.Max(minNumSegments, (int)(angle / maxAngleStep) + 1);
            return numSteps + 1;
        }

        public override List<CurveVertex2D> Tessellate(double maxDeviation)
        {
            //double maxAngleStep = 2 * Math.Acos(1 - maxDeviation / _radius);
            //int numSteps = Math.Max(2, (int)(2 * Math.PI / maxAngleStep) + 1);
            int numSteps = NumberOfCirclePoints(maxDeviation, _radius);

            return Tessellate(numSteps);
        }
        public override List<CurveVertex2D> Tessellate(int numSteps)
        {
            List<CurveVertex2D> points = new List<CurveVertex2D>(numSteps);
            double scaling = 1.0 / (numSteps - 1);
            for (int i = 0; i < numSteps; ++i)
                points.Add(EvaluateVertex(i * scaling));

            return points;
        }

        public override Curve2D GetCopy() { return new Circle2D(this); }

        public override Curve2D Reverse()
        {
            // A full circle has no direction, so return a copy
            return new Circle2D(this);
        }
    }
}