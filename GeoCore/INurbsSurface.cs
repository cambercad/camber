namespace GeoCore
{
    /// <summary>
    /// Minimal analytic surface API stored on surface patches. Implemented by NURBS.BSplineSurface.
    /// </summary>
    public interface INurbsSurface
    {
        Vec3D Evaluate(double u, double v);
        Vec3D EvaluateNormal(double u, double v);
    }

    public struct ParametricRange
    {
        public double UMin;
        public double UMax;
        public double VMin;
        public double VMax;

        public ParametricRange(double uMin, double uMax, double vMin, double vMax)
        {
            UMin = uMin;
            UMax = uMax;
            VMin = vMin;
            VMax = vMax;
        }

        public static ParametricRange UnitSquare => new ParametricRange(0, 1, 0, 1);

        public Vec2D MapMeshUvToNurbs(Vec2D meshUv)
        {
            double u = UMin + meshUv.X * (UMax - UMin);
            double v = VMin + meshUv.Y * (VMax - VMin);
            return new Vec2D(u, v);
        }
    }
}
