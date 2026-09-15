using GeoCore;

namespace Curves
{

    public struct CurveVertex3D
    {
        public Vec3D Origin;
        public Vec3D Tangent;
        public Vec3D Up; // Equivalent to Y
        public double Uniform;

        public Vec3D Y { get { return Up; } }
        public Vec3D X
        {
            get
            {
                return Vec3DOps.Cross(Up, Tangent);
            }
        }
        public Vec3D Z { get{ return Tangent; } }

        public CurveVertex3D(Vec3D origin, Vec3D tangent, Vec3D up, double uniform)
        {
            Origin = origin; Tangent = tangent; Up = up; Uniform = uniform;
        }

        public CurveVertex3D(CoordinateSystem cs, double uniform)
        {
            Origin = cs.Origin;
            Tangent = cs.Z;
            Up = cs.Y;
            Uniform = uniform;
        }

        public Vec3D PointTo3D(Vec2D point)
        {
            return Origin + point.X * X + point.Y * Y;
        }

        public Vec3D DirectionTo3D(Vec2D point)
        {
            return point.X * X + point.Y * Y;
        }

        public CoordinateSystem GetCoordinateSystem()
        {
            return new CoordinateSystem(Origin, X, Y, Tangent);
        }

        public override string ToString()
        {
            return $"Origin: {Origin}, Tangent: {Tangent}, Up: {Up}, Uniform: {Uniform:F3}";
        }
    }
}