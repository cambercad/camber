using GeoCore;
using System.Xml.Linq;

namespace Curves
{
    public class Plane3D
    {
        private string _name; public string Name { get { return _name; } set { _name = value; } }
        private Vec3D _origin; public Vec3D Origin { get { return _origin; } }
        private Vec3D _normal; public Vec3D Normal { get { return _normal; } }
        private Vec3D _x; public Vec3D X { get { return _x; } }
        private Vec3D _y; public Vec3D Y { get { return _y; } }

        public Plane3D(string name = null)
        {
            // Name is set by the parent GeoAPI/container, not auto-generated here
            _name = name;
            _origin = new Vec3D(0);
            _normal = new Vec3D(0,0,1);
            _x = new Vec3D(1, 0, 0);
            _y = new Vec3D(0, 1, 0);
        }
        public Plane3D(Vec3D origin, Vec3D normal, Vec3D x, Vec3D y, string name = null)
        {
            // Name is set by the parent GeoAPI/container, not auto-generated here
            _name = name;
            _origin = origin;
            _normal = normal;
            _x = x;
            _y = y;

            // Todo: Validate (frame should be orthonormal)
        }
        public Plane3D(CoordinateSystem cs, string name = null) : this(cs.Origin, cs.Z, cs.X, cs.Y, name)
        {
        }

        public Vec3D To3D(Vec2D xy)
        {
            return _origin + xy.X * _x + xy.Y * _y;
        }

        public Vec2D To2D(Vec3D xyz)
        {
            xyz = GeometricAlgorithms.ProjectPointOntoPlane(xyz, _normal, _origin);
            xyz -= _origin;
            return new Vec2D(Vec3DOps.Dot(xyz, _x), Vec3DOps.Dot(xyz, _y));
        }

        public CoordinateSystem GetCoordinateSystem()
        {
            return new CoordinateSystem(_origin, _x, _y, _normal);
        }
    }
}
