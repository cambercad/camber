using Curves;
using Curves.Base;
using GeoCore;

namespace GeoSolver.Sketcher
{
    public class ConstrainableCurveFactory : BaseCurveFactory
    {
        public override Line2D CreateLine2D(Vec2D start, Vec2D end, CurveFlags flags)
        {
            return new CLine2D(start, end, flags);
        }

        public override Circle2D CreateCircle2D(Vec2D center, double radius, CurveFlags flags)
        {
            return new CCircle2D(center, radius, flags);
        }

        public override Arc2D CreateArc2D(Vec2D start, Vec2D end, Vec2D center, bool shorter, CurveFlags flags)
        {
            return new CArc2D(start, end, center, shorter, flags);
        }

        public override Arc2D CreateArc2D(Vec2D start, Vec2D pointOnArc, Vec2D end, CurveFlags flags)
        {
            return new CArc2D(start, pointOnArc, end, flags);
        }
    }
}
