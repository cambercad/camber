using System;
using GeoCore;

namespace NURBS
{

    /*public class SketchCurve
    {
        
    }*/

    public class BSplineCurveExtruder
    {
        public static BSplineSurface Extrude(BSplineCurve curve, Vec3D extrudeDir, double extrudeLength)
        {
            extrudeDir.Normalize();
            extrudeDir = extrudeLength * extrudeDir;
            return Extrude(curve, extrudeDir);
        }
        public static BSplineSurface Extrude(BSplineCurve curve, Vec3D extrude)
        {
            switch (curve)
            {
                case BSplineLine line:
                    Vec3D dir = line.End - line.Start;
                    Vec3D normal = Vec3DOps.Cross(extrude, dir);
                    return new BSplinePlane(line.Start, normal, dir, extrude);
                /*case BSplineCurveType.Circle:
                    BSplineCircle circle = curve as BSplineCircle;
                    return new BSplineCylinder();
                    break;*/
                case BSplineCurve c:
                    /*BSplineCurve lower = new BSplineCurve(curve);
                    Vector3d[] controlPoints = new Vector3d[curve.ControlPoints.Length];
                    BSplineCurve upper = new BSplineCurve(curve.Degree, cp, knots, curve.ClosedCurve, curve.Type);
                    return new BSplineSurface(1, new BSplineCurve[] { lower, upper }, new double[] { 0, 0, 1, 1 });*/
                    return new BSplineSurface(1, curve.Degree, BSplineLinearExtrudeSurface.ControlPoints(curve.ControlPoints, extrude), new double[] { 0, 0, 1, 1 }, curve.Knots.Copy());
                
            }
            throw new Exception();
        }
    }
}