using System.Collections;
using Curves;
using GeoCore;

namespace GeoSolver;

/// <summary>A five-parameter ellipse. Axis dimensions use ordinary point distances.</summary>
public class CEllipse2D : Ellipse2D, IUpdate, IEnumerableParams, IEvaluateCPoint, IFreezable
{
    public CVec2D CCenter { get; }
    public CVec2D CMajorAxis { get; }
    public Expr CMinorRadius { get; }

    public CEllipse2D(Vec2D center, Vec2D majorAxis, double minorRadius,
        double startAngle = 0, double endAngle = 2*Math.PI, CurveFlags flags = CurveFlags.None)
        : base(center,majorAxis,minorRadius,startAngle,endAngle,flags)
    {
        CCenter = new CVec2D(center);
        CMajorAxis = new CVec2D(majorAxis);
        CMinorRadius = Expr.Parameter(minorRadius);
    }

    public void Update()
    {
        _center = CCenter.Evaluate();
        _majorAxis = CMajorAxis.Evaluate();
        _minorAxisLength = CMinorRadius.Evaluate();
    }

    public CVec2D Evaluate(double uniform)
    {
        double angle = StartAngle + (IsFullEllipse && uniform == 1 ? 0 : uniform)*(EndAngle-StartAngle);
        // Quarter points use exact coefficients so axis constraints do not
        // inherit the nonzero floating-point value of cos(pi/2).
        double quarter = angle/(Math.PI/2);
        double c = Math.Cos(angle), s = Math.Sin(angle);
        if (quarter == Math.Round(quarter))
        {
            int q = ((int)Math.Round(quarter)%4+4)%4;
            c = q == 0 ? 1 : q == 2 ? -1 : 0;
            s = q == 1 ? 1 : q == 3 ? -1 : 0;
        }
        var ratio = CMinorRadius / Expr.Sqrt(CMajorAxis.Ex*CMajorAxis.Ex + CMajorAxis.Ey*CMajorAxis.Ey);
        return new CVec2D(CCenter.Ex + c*CMajorAxis.Ex - s*ratio*CMajorAxis.Ey,
                          CCenter.Ey + c*CMajorAxis.Ey + s*ratio*CMajorAxis.Ex);
    }

    public IEnumerator<Param> GetEnumerator()
    {
        foreach (var p in CCenter) yield return p;
        foreach (var p in CMajorAxis) yield return p;
        foreach (var p in CMinorRadius) yield return p;
    }
    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    public void Freeze() { foreach (var p in this) p.Frozen = true; }
    public void Unfreeze() { foreach (var p in this) p.Frozen = false; }
    public bool IsFrozen => this.Any(p => p.Frozen);
}
