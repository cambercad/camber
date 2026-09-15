using GeoCore;

namespace NURBS
{
    public class BSplineSphere : BSplineSurfaceOfRevolution
    {
        protected Vec3D _center;
        protected double _radius;

        public BSplineSphere(Vec3D center, double radius)
            : base(center, new Vec3D(0, 1, 0), new BSplineArc(center, new Vec3D(0, 0, 1), radius, new Vec3D(0, 1, 0), 0, 0.5), new Vec3D(1, 0, 0))
        {
            _center = center;
            _radius = radius;
        }
    }
}
