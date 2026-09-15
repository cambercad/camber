using System.Collections.Generic;
using GeoCore;

namespace Curves
{

    public abstract class Curve3D : IName
    {
        private string _name; public string Name { get { return _name; } set { _name = value; } }

        public abstract CurveVertex3D Evaluate(double uniform);
        public abstract List<CurveVertex3D> Tessellate(double maxDeviation/*, out List<Vector2d> normals, out List<double> uv*/);

        public Vec3D Start { get { return Evaluate(0).Origin; } }
        public Vec3D End { get { return Evaluate(1).Origin; } }

        protected Curve3D(string name = null)
        {
            // Name is set by the parent GeoAPI/container, not auto-generated here
            _name = name;
        }
    }
}