using GeoCore;

namespace Curves
{

    public struct CurveVertex2D
    {
        public Vec2D Position;
        public Vec2D Normal;
        public double Uniform;

        public Vec2D Tangent { get { return new Vec2D(-Normal.Y, Normal.X); } }

        public CurveVertex2D(Vec2D position, Vec2D normal, double uniform)
        {
            Position = position; Normal = normal; Uniform = uniform;
        }
    }
}