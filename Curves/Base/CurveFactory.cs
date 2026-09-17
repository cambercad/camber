using GeoCore;

namespace Curves.Base
{
    public abstract class BaseCurveFactory
    {
        public static BaseCurveFactory Instance = new DefaultCurveFactory();

        // 2D Curve factory methods
        public abstract Line2D CreateLine2D(Vec2D start, Vec2D end, CurveFlags flags);
        public abstract Ellipse2D CreateEllipse2D(Vec2D center, Vec2D majorAxis, double minorRadius, double startAngle, double endAngle, CurveFlags flags);
        public abstract Circle2D CreateCircle2D(Vec2D center, double radius, CurveFlags flags);
        public abstract Arc2D CreateArc2D(Vec2D start, Vec2D end, Vec2D center, bool shorter, CurveFlags flags);
        public abstract Arc2D CreateArc2D(Vec2D start, Vec2D pointOnArc, Vec2D end, CurveFlags flags);
    }

    public class DefaultCurveFactory : BaseCurveFactory
    {
        // 2D Curve implementations
        public override Line2D CreateLine2D(Vec2D start, Vec2D end, CurveFlags flags)
        {
            return new Line2D(start, end, flags);
        }

        public override Ellipse2D CreateEllipse2D(Vec2D center, Vec2D majorAxis, double minorRadius,
            double startAngle, double endAngle, CurveFlags flags)
            => new Ellipse2D(center,majorAxis,minorRadius,startAngle,endAngle,flags);

        public override Circle2D CreateCircle2D(Vec2D center, double radius, CurveFlags flags)
        {
            return new Circle2D(center, radius, flags);
        }

        public override Arc2D CreateArc2D(Vec2D start, Vec2D end, Vec2D center, bool shorter, CurveFlags flags)
        {
            return new Arc2D(start, end, center, shorter, flags);
        }

        public override Arc2D CreateArc2D(Vec2D start, Vec2D pointOnArc, Vec2D end, CurveFlags flags)
        {
            return new Arc2D(start, pointOnArc, end, flags);
        }
    }
}
