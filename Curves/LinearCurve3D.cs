using System.Collections.Generic;
using GeoCore;

namespace Curves
{
    public class LinearCurve3D : Curve3D
    {
        private Vec3D _start; //public Vector2d Start { get { return _start; } }
        private Vec3D _end; //public Vector2d End { get { return _end; } }
        private Vec3D _up;


        protected LinearCurve3D(string name = null) : base(name) { }
        public LinearCurve3D(Vec3D start, Vec3D end, Vec3D up, string name = null) : base(name)
        {
            _start = start;
            _end = end;
            _up = up;
        }

        public override CurveVertex3D Evaluate(double uniform)
        {
            Vec3D dir = _end - _start;
            dir.Normalize();

            Vec3D position = (1 - uniform) * _start + uniform * _end;

            return new CurveVertex3D(position, dir, _up, uniform);
        }

        public override List<CurveVertex3D> Tessellate(double tolerance)
        {
            Vec3D dir = _end - _start;
            dir.Normalize();
            return new List<CurveVertex3D>() { new CurveVertex3D(_start, dir, _up, 0), new CurveVertex3D(_end, dir, _up, 1) };
        }
    }
}