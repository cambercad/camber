using GeoCore;
using System.Xml.Linq;

namespace Curves
{
    public class Line3D : Curve3D
    {
        private Vec3D _start; //public Vec3D Start { get { return _start; } }
        private Vec3D _end; //public Vec3D End { get { return _end; } }

        public Vec3D DirectionNormalized { get { return (_end - _start).Normalized(); } }
        public double Length() { return (_end - _start).Length(); }
        public Vec3D Center { get { return 0.5 * (_start + _end); } }


        protected Line3D() : base(null) { }
        public Line3D(Vec3D start, Vec3D end, string name = null) : base(name)
        {
            _start = start;
            _end = end;
        }

        public Line3D(Line3D l) : base(l.Name)
        {
            _start = l._start;
            _end = l._end;
        }

        public Line3D(double startX, double startY, double startZ, double endX, double endY, double endZ)
            : this(new Vec3D(startX, startY, startZ), new Vec3D(endX, endY, endZ))
        { }

        public void SetStart(Vec3D start) { _start = start; }
        public void SetEnd(Vec3D end) { _end = end; }

        public List<Vec3D> ToReferencePoints()
        {
            return new List<Vec3D>() { Start, End };
        }

        public void UpdateFromReferencePoints(List<Vec3D> referencePoints)
        {
            _start = referencePoints[0];
            _end = referencePoints[1];
        }

        /// <summary>
        /// Returns defining points: [start, end]
        /// </summary>
        public List<Vec3D> ToPoints()
        {
            return new List<Vec3D> { _start, _end };
        }

        /// <summary>
        /// Creates a Line3D from defining points: [start, end]
        /// </summary>
        public static Line3D FromPoints(List<Vec3D> points, string name = null)
        {
            if (points == null || points.Count < 2)
                throw new ArgumentException("Line3D requires 2 points: [start, end]");
            return new Line3D(points[0], points[1], name);
        }

        public Vec3D EvaluatePoint(double uniform)
        {
            return (1 - uniform) * _start + uniform * _end;
        }

        public override CurveVertex3D Evaluate(double uniform)
        {
            Vec3D dir = _end - _start;
            Vec3D tangent = dir.Normalized();
            
            // Choose up vector perpendicular to tangent
            Vec3D up;
            if (Math.Abs(tangent.Z) < 0.9) // If not parallel to Z-axis
            {
                up = new Vec3D(0, 0, 1); // Use Z-up
                up = (up - Vec3DOps.Dot(up, tangent) * tangent).Normalized(); // Make perpendicular
            }
            else
            {
                up = new Vec3D(1, 0, 0); // Use X-axis if parallel to Z
                up = (up - Vec3DOps.Dot(up, tangent) * tangent).Normalized(); // Make perpendicular
            }

            Vec3D position = (1 - uniform) * _start + uniform * _end;

            return new CurveVertex3D(position, tangent, up, uniform);
        }

        public override List<CurveVertex3D> Tessellate(double tolerance)
        {
            Vec3D dir = _end - _start;
            Vec3D tangent = dir.Normalized();
            
            // Choose up vector perpendicular to tangent
            Vec3D up;
            if (Math.Abs(tangent.Z) < 0.9) // If not parallel to Z-axis
            {
                up = new Vec3D(0, 0, 1); // Use Z-up
                up = (up - Vec3DOps.Dot(up, tangent) * tangent).Normalized(); // Make perpendicular
            }
            else
            {
                up = new Vec3D(1, 0, 0); // Use X-axis if parallel to Z
                up = (up - Vec3DOps.Dot(up, tangent) * tangent).Normalized(); // Make perpendicular
            }
            
            return new List<CurveVertex3D>() { new CurveVertex3D(_start, tangent, up, 0), new CurveVertex3D(_end, tangent, up, 1) };
        }

        public Curve3D GetCopy() { return new Line3D(this); }
    }
}
