using System;
using GeoCore;

namespace NURBS
{
    public class BSplineLinearExtrudeSurface : BSplineSurfaceBounded
    {
        private Vec3D _extrudeDirection;
        private double _extrudeLength;
        private BSplineCurve _baseCurve;

        public BSplineLinearExtrudeSurface(BSplineCurve baseCurve, Vec3D extrudeDirection, double extrudeLength)
            : base(1, baseCurve.Degree, ControlPoints(baseCurve.ControlPoints, extrudeDirection, extrudeLength), new double[] { 0, 0, 1, 1 }, baseCurve.Knots.Copy())
        {            
            extrudeDirection.Normalize();

            _extrudeDirection = extrudeDirection;
            _extrudeLength = extrudeLength;
            _baseCurve = baseCurve;
        }

        public new static Vec4D[][] ControlPoints(Vec4D[] baseControlPoints, Vec3D extrudeDirection, double extrudeLength)
        {
            extrudeDirection.Normalize();
            extrudeDirection *= extrudeLength;

            return ControlPoints(baseControlPoints, extrudeDirection);
        }
        public new static Vec4D[][] ControlPoints(Vec4D[] baseControlPoints, Vec3D extrude)
        {
            int l = baseControlPoints.Length;
            Vec4D[][] controlPoints = new Vec4D[2][];

            Vec4D[] vPoints = new Vec4D[l];
            for (int j = 0; j < l; ++j)
                vPoints[j] = baseControlPoints[j];
            controlPoints[0] = vPoints;

            vPoints = new Vec4D[l];
            for (int j = 0; j < l; ++j)
            {
                Vec4D v = baseControlPoints[j];
                vPoints[j] = v + new Vec4D(v.W * extrude.X, v.W * extrude.Y, v.W * extrude.Z, 0);
            }
            controlPoints[1] = vPoints;

            return controlPoints;
        }
    }
}
