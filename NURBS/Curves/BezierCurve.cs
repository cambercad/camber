using GeoCore;


namespace NURBS
{
    
    public class BezierCurve : BSplineCurve
    {
        public BezierCurve(params Vec3D[] controlPoints)
            : base(controlPoints.Length - 1, controlPoints,
                  BSplineCurve.UniformKnotVector(controlPoints.Length - 1, controlPoints.Length), false)
        {
        }
    }
}
