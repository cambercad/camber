using GeoCore;

namespace Curves;

/// <summary>A sketch-space adapter over the shared spatial NURBS evaluator.</summary>
public class BSpline2D : Curve2D
{
    private readonly NURBS.BSplineCurve _curve;
    public Vec2D[] ControlPoints { get; }
    public double[] Knots { get; }
    public double[] Weights { get; }
    public int Degree => _curve.Degree;
    public bool IsRational => Weights != null;

    public BSpline2D(Vec2D[] controlPoints, double[] knots, int degree)
        : this(controlPoints, knots, degree, null) { }

    public BSpline2D(Vec2D[] controlPoints, double[] knots, int degree, double[] weights)
    {
        ArgumentNullException.ThrowIfNull(controlPoints);
        ArgumentNullException.ThrowIfNull(knots);
        if (degree < 1 || degree >= controlPoints.Length)
            throw new ArgumentException("Degree must be positive and less than the control-point count.");
        if (knots.Length != controlPoints.Length + degree + 1)
            throw new ArgumentException("Invalid knot vector size.");
        if (weights != null && weights.Length != controlPoints.Length)
            throw new ArgumentException("Weights array must match control points.");
        foreach (var point in controlPoints)
            if (!double.IsFinite(point.X) || !double.IsFinite(point.Y))
                throw new ArgumentException("Control points must be finite.");
        if (weights != null)
            foreach (double weight in weights)
                if (!double.IsFinite(weight) || weight <= 0)
                    throw new ArgumentException("Weights must be finite and positive.");
        for (int i = 0; i < knots.Length; i++)
            if (!double.IsFinite(knots[i]) || i > 0 && knots[i] < knots[i-1])
                throw new ArgumentException("Knots must be finite and nondecreasing.");
        double start = knots[degree], span = knots[controlPoints.Length] - start;
        if (!(span > 0) || !double.IsFinite(span))
            throw new ArgumentException("The active knot domain must have positive finite length.");
        ControlPoints = (Vec2D[])controlPoints.Clone();
        Knots = (double[])knots.Clone();
        Weights = weights == null ? null : (double[])weights.Clone();
        var points = new Vec3D[controlPoints.Length];
        for (int i = 0; i < points.Length; i++)
            points[i] = new Vec3D(controlPoints[i].X, controlPoints[i].Y, 0);
        var normalized = new double[knots.Length];
        for (int i = 0; i < normalized.Length; i++) normalized[i] = (knots[i]-start)/span;
        _curve = weights == null
            ? new NURBS.BSplineCurve(degree, points, normalized)
            : new NURBS.BSplineCurve(degree, points, (double[])weights.Clone(), normalized);
    }

    public override CurveVertex2D EvaluateVertex(double uniform)
    {
        double u = Math.Clamp(uniform, 0, 1);
        var point = _curve.EvaluateUniform(u);
        var derivative = _curve.EvaluateUniformDU(u);
        var tangent = new Vec2D(derivative.X, derivative.Y);
        double length = tangent.Length();
        // Curve2D normals lie to the right of the directed tangent (Line2D,
        // Bezier2D and Hermite use this convention too).
        var normal = length > 0 ? new Vec2D(tangent.Y/length, -tangent.X/length) : new Vec2D(0,-1);
        return new CurveVertex2D(new Vec2D(point.X,point.Y), normal, u);
    }

    public override List<CurveVertex2D> Tessellate(double maxDeviation)
    {
        _ = _curve.Tessellate(maxDeviation, out var parameters);
        var result = new List<CurveVertex2D>(parameters.Count+1);
        foreach (double u in parameters) result.Add(EvaluateVertex(u));
        result.Add(EvaluateVertex(1));
        return result;
    }

    public override List<CurveVertex2D> Tessellate(int numPoints)
    {
        if (numPoints < 2) throw new ArgumentOutOfRangeException(nameof(numPoints));
        var result = new List<CurveVertex2D>(numPoints);
        for (int i = 0; i < numPoints; i++) result.Add(EvaluateVertex((double)i/(numPoints-1)));
        return result;
    }

    public override double Length() => _curve.ArcLength(0,1);

    /// <summary>Same analytic curve in a spatial frame, with its normalized active domain.</summary>
    public NURBS.BSplineCurve ToSpatialCurve(CoordinateSystem frame)
    {
        var points = new Vec3D[ControlPoints.Length];
        for (int i = 0; i < points.Length; i++) points[i] = frame.PointTo3D(ControlPoints[i]);
        var knots = (double[])_curve.Knots.Clone();
        return Weights == null
            ? new NURBS.BSplineCurve(Degree, points, knots)
            : new NURBS.BSplineCurve(Degree, points, (double[])Weights.Clone(), knots);
    }
    public override Curve2D GetCopy() => new BSpline2D(ControlPoints,Knots,Degree,Weights)
        { Flags = Flags, Name = Name };

    public override Curve2D Reverse()
    {
        var points = new Vec2D[ControlPoints.Length];
        var knots = new double[Knots.Length];
        var weights = Weights == null ? null : new double[Weights.Length];
        for (int i = 0; i < points.Length; i++)
        {
            points[i] = ControlPoints[^(i+1)];
            if (weights != null) weights[i] = Weights[^(i+1)];
        }
        double sum = Knots[0]+Knots[^1];
        for (int i = 0; i < knots.Length; i++) knots[i] = sum-Knots[^(i+1)];
        return new BSpline2D(points,knots,Degree,weights) { Flags = Flags, Name = Name };
    }
}

