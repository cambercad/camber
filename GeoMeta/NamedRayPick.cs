using System.Collections.Generic;

namespace GeoMeta
{
    /// <summary>
    /// Closest-hit ray pick against named points and polylines.
    /// Points win over curves when both are within their radii; empty string is a miss.
    /// </summary>
    public static class NamedRayPick
    {
        public const string KindPoint = "point";
        public const string KindCurve = "curve";

        public static string Closest(
            double originX, double originY, double originZ,
            double dirX, double dirY, double dirZ,
            IReadOnlyList<NamedPickTarget> targets,
            double pointRadius,
            double curveRadius)
        {
            if (targets == null || targets.Count == 0)
                return "";
            double dirLen = Math.Sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
            if (dirLen < 1e-15)
                return "";
            dirX /= dirLen;
            dirY /= dirLen;
            dirZ /= dirLen;
            if (pointRadius <= 0.0)
                pointRadius = 1e-9;
            if (curveRadius <= 0.0)
                curveRadius = 1e-9;

            string bestPoint = "";
            double bestPointD = pointRadius;
            string bestCurve = "";
            double bestCurveD = curveRadius;

            for (int i = 0; i < targets.Count; i++)
            {
                NamedPickTarget target = targets[i];
                if (target == null || string.IsNullOrEmpty(target.Name))
                    continue;
                string kind = target.Kind ?? "";
                if (kind == KindPoint)
                {
                    double d = RayPointDistance(
                        originX, originY, originZ, dirX, dirY, dirZ,
                        target.X, target.Y, target.Z);
                    if (d < bestPointD)
                    {
                        bestPointD = d;
                        bestPoint = target.Name;
                    }
                    continue;
                }
                if (kind != KindCurve || target.Polyline == null || target.Polyline.Count < 2)
                    continue;
                double curveD = RayPolylineDistance(
                    originX, originY, originZ, dirX, dirY, dirZ,
                    target.Polyline, target.Closed);
                if (curveD < bestCurveD)
                {
                    bestCurveD = curveD;
                    bestCurve = target.Name;
                }
            }

            if (bestPoint.Length > 0)
                return bestPoint;
            return bestCurve;
        }

        public static double RayPointDistance(
            double ox, double oy, double oz,
            double dx, double dy, double dz,
            double px, double py, double pz)
        {
            double wx = px - ox;
            double wy = py - oy;
            double wz = pz - oz;
            double t = wx * dx + wy * dy + wz * dz;
            if (t < 0.0)
                return double.PositiveInfinity;
            return Distance(px, py, pz, ox + t * dx, oy + t * dy, oz + t * dz);
        }

        static double RayPolylineDistance(
            double ox, double oy, double oz,
            double dx, double dy, double dz,
            List<NamedPickPoint> polyline,
            bool closed)
        {
            double best = double.PositiveInfinity;
            int last = polyline.Count - 1;
            for (int i = 0; i < last; i++)
            {
                double d = RaySegmentDistance(ox, oy, oz, dx, dy, dz, polyline[i], polyline[i + 1]);
                if (d < best)
                    best = d;
            }
            if (closed && polyline.Count > 2)
            {
                double d = RaySegmentDistance(ox, oy, oz, dx, dy, dz, polyline[last], polyline[0]);
                if (d < best)
                    best = d;
            }
            return best;
        }

        static double RaySegmentDistance(
            double ox, double oy, double oz,
            double dx, double dy, double dz,
            NamedPickPoint a, NamedPickPoint b)
        {
            double ex = b.X - a.X;
            double ey = b.Y - a.Y;
            double ez = b.Z - a.Z;
            double wx = ox - a.X;
            double wy = oy - a.Y;
            double wz = oz - a.Z;
            double aa = ex * ex + ey * ey + ez * ez;
            double ad = ex * dx + ey * dy + ez * dz;
            double aw = ex * wx + ey * wy + ez * wz;
            double dw = dx * wx + dy * wy + dz * wz;
            double denom = aa - ad * ad;
            double s;
            double t;
            if (aa < 1e-18)
            {
                return RayPointDistance(ox, oy, oz, dx, dy, dz, a.X, a.Y, a.Z);
            }
            if (Math.Abs(denom) < 1e-18)
            {
                double d0 = RayPointDistance(ox, oy, oz, dx, dy, dz, a.X, a.Y, a.Z);
                double d1 = RayPointDistance(ox, oy, oz, dx, dy, dz, b.X, b.Y, b.Z);
                return Math.Min(d0, d1);
            }
            s = (aw - ad * dw) / denom;
            if (s < 0.0)
                s = 0.0;
            else if (s > 1.0)
                s = 1.0;
            t = s * ad - dw;
            if (t < 0.0)
                return double.PositiveInfinity;
            return Distance(
                a.X + s * ex, a.Y + s * ey, a.Z + s * ez,
                ox + t * dx, oy + t * dy, oz + t * dz);
        }

        static double Distance(double ax, double ay, double az, double bx, double by, double bz)
        {
            double dx = ax - bx;
            double dy = ay - by;
            double dz = az - bz;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }
    }
}
