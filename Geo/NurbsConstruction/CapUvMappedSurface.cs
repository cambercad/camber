using GeoCore;
using NURBS;

namespace Geo.NurbsConstruction
{
    /// <summary>
    /// Maps mesh cap UV (isotropic scale 1/max(span)) to BSplinePlane [0,1]² parameterization.
    /// </summary>
    public sealed class CapUvMappedSurface : INurbsSurface
    {
        public BSplinePlane Plane { get; }
        public Vec2D UvMin { get; }
        public Vec2D UvMax { get; }
        public double Scale { get; }

        public CapUvMappedSurface(BSplinePlane plane, Vec2D uvMin, Vec2D uvMax, double scale)
        {
            Plane = plane;
            UvMin = uvMin;
            UvMax = uvMax;
            Scale = scale;
        }

        public Vec2D MapMeshUv(double u, double v)
        {
            double spanX = UvMax.X - UvMin.X;
            double spanY = UvMax.Y - UvMin.Y;
            double x = UvMin.X + u / Scale;
            double y = UvMin.Y + v / Scale;
            double nu = spanX > 1e-15 ? (x - UvMin.X) / spanX : 0;
            double nv = spanY > 1e-15 ? (y - UvMin.Y) / spanY : 0;
            return new Vec2D(nu, nv);
        }

        public Vec3D Evaluate(double u, double v)
        {
            var nurbsUv = MapMeshUv(u, v);
            return Plane.Evaluate(nurbsUv.X, nurbsUv.Y);
        }

        public Vec3D EvaluateNormal(double u, double v)
        {
            var nurbsUv = MapMeshUv(u, v);
            return Plane.EvaluateNormal(nurbsUv.X, nurbsUv.Y);
        }
    }
}
