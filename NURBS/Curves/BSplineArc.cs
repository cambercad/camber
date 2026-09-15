using GeoCore;


namespace NURBS
{
    
    public class BSplineArc : BSplineCircle
    {        
        public BSplineArc(Vec3D center, Vec3D axis, double radius, Vec3D centerToStart, double startParameter, double endParameter)
            : base(center, axis, radius, centerToStart, startParameter, endParameter)
        {
            //_startParameter = startParameter;
            //_endParameter = endParameter;
        }

        /*public override Vector3d Evaluate(double u)
        {
            return base.Evaluate(_startParameter + u * (_endParameter - _startParameter));
        }*/
    }
}
