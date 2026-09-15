using GeoCore;

namespace NURBS
{
    public class MinCurvatureHermiteSpline
    {
        public static void ScaleTangents(Vec3D p0, Vec3D p1, ref Vec3D d0, ref Vec3D d1)
        {
            d0.Normalize();
            d1.Normalize();

            Vec3D deltaP0 = p1 - p0;

            var c01 = Vec3DOps.Cross(d0, d1).Length();
            var c0 = Vec3DOps.Cross(d0, deltaP0).Length();
            var c1 = Vec3DOps.Cross(d1, deltaP0).Length();

            var alpha0 = -2.0 * c1 / c01;
            var alpha1 = 2.0 * c0 / c01;

            d0 *= alpha0;
            d1 *= alpha1;
        }
    }
}
