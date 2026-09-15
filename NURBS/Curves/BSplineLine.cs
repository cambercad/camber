using GeoCore;


namespace NURBS
{
    
    public class BSplineLine : BSplineCurve
    {
        protected Vec3D _start; public override Vec3D Start { get { return _start; } }
        protected Vec3D _end; public override Vec3D End { get { return _end; } }

        public BSplineLine(Vec3D start, Vec3D end)
            : base(1, new Vec3D[] { start, end }, new double[] { 0, 0, 1, 1 }, false)
        {
            _start = start;
            _end = end;
        }
    }
}
