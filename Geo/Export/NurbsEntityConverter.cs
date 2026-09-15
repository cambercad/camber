using GeoCore;
using NURBS;

namespace Geo.Export
{
    public static class NurbsEntityConverter
    {
        public static (List<double> uniqueKnots, List<int> multiplicities) CompressKnotVector(double[] knots, int degree)
        {
            var unique = new List<double>();
            var mults = new List<int>();
            if (knots == null || knots.Length == 0)
                return (unique, mults);

            double current = knots[0];
            int count = 1;
            for (int i = 1; i < knots.Length; i++)
            {
                if (Math.Abs(knots[i] - current) < 1e-12)
                    count++;
                else
                {
                    unique.Add(current);
                    mults.Add(count);
                    current = knots[i];
                    count = 1;
                }
            }
            unique.Add(current);
            mults.Add(count);
            return (unique, mults);
        }

        public static void DehomogenizeControlPoint(Vec4D v, out Vec3D point, out double weight)
        {
            weight = v.W;
            if (Math.Abs(weight) < 1e-15)
            {
                point = new Vec3D(v.X, v.Y, v.Z);
                weight = 1;
                return;
            }
            point = new Vec3D(v.X / weight, v.Y / weight, v.Z / weight);
        }

        public static bool IsRational(BSplineCurve curve)
        {
            foreach (var cp in curve.ControlPoints)
            {
                if (Math.Abs(cp.W - 1.0) > 1e-9)
                    return true;
            }
            return false;
        }

        public static bool IsRational(BSplineSurface surface)
        {
            foreach (var row in surface.ControlPoints)
            {
                foreach (var cp in row)
                {
                    if (Math.Abs(cp.W - 1.0) > 1e-9)
                        return true;
                }
            }
            return false;
        }

        public static List<Vec3D> FlattenSurfaceControlPoints(BSplineSurface surface, out List<double> weights)
        {
            int nu = surface.NumControlPointsU;
            int nv = surface.NumControlPointsV;
            var points = new List<Vec3D>(nu * nv);
            weights = new List<double>(nu * nv);
            for (int v = 0; v < nv; v++)
            {
                for (int u = 0; u < nu; u++)
                {
                    DehomogenizeControlPoint(surface.ControlPoints[u][v], out var p, out var w);
                    points.Add(p);
                    weights.Add(w);
                }
            }
            return points;
        }

        public static List<Vec3D> CurveControlPoints(BSplineCurve curve, out List<double> weights)
        {
            weights = new List<double>(curve.ControlPoints.Length);
            var points = new List<Vec3D>(curve.ControlPoints.Length);
            foreach (var cp in curve.ControlPoints)
            {
                DehomogenizeControlPoint(cp, out var p, out var w);
                points.Add(p);
                weights.Add(w);
            }
            return points;
        }
    }
}
