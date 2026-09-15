using GeoCore;
using NURBS;

namespace Curves
{
    /// <summary>
    /// One cubic Hermite span (parameter s ∈ [0,1]) is exactly a cubic Bézier, hence a clamped cubic <see cref="BSplineCurve"/>.
    /// </summary>
    public static class CubicHermiteSpanBspline
    {
        private static readonly double[] ClampedUnitKnots = { 0, 0, 0, 0, 1, 1, 1, 1 };

        public static BSplineCurve ToBSplineCurve(Vec3D p0, Vec3D m0, Vec3D p1, Vec3D m1)
        {
            const double third = 1.0 / 3.0;
            Vec3D[] cp =
            {
                p0,
                p0 + m0 * third,
                p1 - m1 * third,
                p1
            };
            return new BSplineCurve(3, cp, ClampedUnitKnots, closedCurve: false);
        }

        /// <summary>Planar span in z = 0; tessellation uses only x/y (see <see cref="NURBS.BSplineCurve"/>).</summary>
        public static BSplineCurve ToBSplineCurve(Vec2D p0, Vec2D m0, Vec2D p1, Vec2D m1) =>
            ToBSplineCurve(
                new Vec3D(p0.X, p0.Y, 0),
                new Vec3D(m0.X, m0.Y, 0),
                new Vec3D(p1.X, p1.Y, 0),
                new Vec3D(m1.X, m1.Y, 0));
    }
}
