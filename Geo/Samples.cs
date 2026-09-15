using Curves;
using GeoCore;
using GeoSolver.Sketcher;

namespace Geo
{
    public class Samples
    {        
        public static ConstrainedSketcher CogWheel(GeoAPI api, string name, double module, int numTeeth, double innerRadiusOffsetModuleFactor = 1.166)
        {
            double m = module;
            int z = numTeeth;
            double p = m * Math.PI;

            double r = GetRadius(module, numTeeth);
            double hf = innerRadiusOffsetModuleFactor * m; // 1.25 * m;
            double hk = m;
            double rf = r - hf;
            double rk = r + hk;

            //double phi = 3 * Math.Sqrt(4 * z - 1) / (2 * z - 5);
            double phi = Math.Sqrt(rk * rk - rf * rf) / rf;
            Vec2D evolvent = EvaluateEvolvent(phi, rf);
            double gamma = Math.Atan2(evolvent.Y, evolvent.X);
            double alpha = p / r; //Angle betwen two teeth
            double angleBetweenTeeth = alpha - 2 * gamma;

            double beta = angleBetweenTeeth / 3;
            double delta = angleBetweenTeeth - beta;

            int numPoints = 20;
           
            double angleStart = gamma;
            double angleEnd = angleStart + beta;

            double mirrorAxisAngle = gamma + 0.5 * beta;
            Vec2D mirrorAxisOrigin = new Vec2D(0, 0);
            Vec2D mirrorAxis = new Vec2D(Math.Cos(mirrorAxisAngle), Math.Sin(mirrorAxisAngle));


            //List<Vector2d> points = new List<Vector2d>();
            List<SampledCurve> curves = new List<SampledCurve>();
     
            double scaling = 1.0 / (numPoints - 1);
            SampledCurve curve = new SampledCurve(numPoints);
            var curveMirrored = new SampledCurve(numPoints);
            for (int i = 0; i < numPoints; ++i)
            {
                var pt = EvaluateEvolvent(i * scaling * phi, rf);
                var n = EvaluateEvolventNormalNormalized(i * scaling * phi, rf);
                curve.Add(pt, n);

                //curveMirrored.Add(MirrorPoint(pt, mirrorAxisOrigin, mirrorAxis), MirrorPoint(n, new Vec2D(0), mirrorAxis));
            }
            for (int i = 0; i < numPoints; ++i)
            {
                var pt = EvaluateEvolvent((numPoints-i-1) * scaling * phi, rf);
                var n = EvaluateEvolventNormalNormalized((numPoints - i - 1) * scaling * phi, rf);
                

                curveMirrored.Add(MirrorPoint(pt, mirrorAxisOrigin, mirrorAxis), MirrorPoint(n, new Vec2D(0), mirrorAxis));
            }
            curves.Add(curve);
            //curveMirrored.Reverse();
       

            numPoints = 5;
            scaling = 1.0 / (numPoints - 1);

            curve = new SampledCurve(numPoints);
            //var curveMirrored = new SampledCurve(numPoints);
            for (int i = 0; i < numPoints; ++i)
            {
                double angle = angleStart + (i * scaling) * (angleEnd - angleStart);
                //Vector2d pt = new Vector2d(rk * Math.Cos(angle), rk * Math.Sin(angle));
                Vec2D n = new Vec2D(Math.Cos(angle), Math.Sin(angle));

                curve.Add(rk * n, n);

                //curveMirrored.Add(MirrorPoint(rk * n, mirrorAxisOrigin, mirrorAxis), MirrorPoint(n, new Vec2D(0), mirrorAxis));
            }
            
            curves.Add(curve);
            

            curves.Add(curveMirrored);


            //for (int i = 0; i < points.Count; ++i)
            //    points.Add(MirrorPoint(points[i], mirrorAxisOrigin, mirrorAxis));
            //curves.Add(((SampledCurve)curves[0].GetMirrored(mirrorAxisOrigin, mirrorAxis)).Reverse());

            angleStart = 2 * gamma + beta;
            angleEnd = angleStart + delta;
            curve = new SampledCurve(numPoints);
            for (int i = 0; i < numPoints; ++i)
            {
                double angle = angleStart + (i * scaling) * (angleEnd - angleStart);
                //Vector2d pt = new Vector2d(rf * Math.Cos(angle), rf * Math.Sin(angle));
                Vec2D n = new Vec2D(Math.Cos(angle), Math.Sin(angle));
                //points.Add(pt);
                curve.Add(rf * n, n);
            }
            curves.Add(curve);

            List<Curve2D> allCurves = new List<Curve2D>(numTeeth * curves.Count);
            allCurves.AddRange(curves);
            double deltaAngle = (2 * Math.PI) / numTeeth;
            for (int i = 1; i < numTeeth; ++i)
            {
                for (int j = 0; j < curves.Count; ++j)
                {
                    SampledCurve c = curves[j];
                    allCurves.Add(c.GetRotated(i * deltaAngle));
                }
            }

            var s = api.GetPlotterSketcher(new Plane3D(), name);

            //Sketch s = new Sketch(new Double3(0, 0, 0), new Double3(1, 0, 0), new Double3(0, 1, 0));
            s.SetCurves(new List<List<Curve2D>>() { allCurves });
            return s;
        }

        public static Vec2D MirrorPoint(Vec2D p, Vec2D pointOnAxis, Vec2D axisDirection)
        {
            Vec2D l = GeometricAlgorithms.ProjectPointOntoLine(p, pointOnAxis, axisDirection);
            Vec2D delta = l - p;
            return l + delta;
        }

        public static double GetRadius(double module, int numTeeth)
        {
            return module * numTeeth * 0.5;
        }

        public static double GetInnerRadius(double module, int numTeeth, double innerRadiusOffsetModuleFactor = 1.166)
        {
            return GetRadius(module, numTeeth) - innerRadiusOffsetModuleFactor * module;
        }

        public static double GetOuterRadius(double module, int numTeeth)
        {
            return GetRadius(module, numTeeth) + module;
        }

        private static Vec2D EvaluateEvolvent(double phi, double baseCircleRadius = 1)
        {
            double cos = Math.Cos(phi);
            double sin = Math.Sin(phi);
            return new Vec2D(baseCircleRadius * (cos + phi * sin), baseCircleRadius * (sin - phi * cos));
        }

        private static Vec2D EvaluateEvolventTangent(double phi, double baseCircleRadius = 1)
        {
            return new Vec2D(baseCircleRadius * (phi * Math.Cos(phi)), baseCircleRadius * (phi * Math.Sin(phi)));
        }

        private static Vec2D EvaluateEvolventNormalNormalized(double phi, double baseCircleRadius = 1)
        {
            if (phi < 1e-12)
                return new Vec2D(0, -1);
            Vec2D t = EvaluateEvolventTangent(phi, baseCircleRadius);
            Vec2D n = new Vec2D(t.Y, -t.X);
            n.Normalize();
            return n;
        }
    }
}
