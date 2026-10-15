namespace GeoSolver;

/// <summary>Symbolic point contacts authored by sketch constraints.</summary>
internal static class SketchConstraintIncidence
{
    internal static bool SamePoint(CVec2D a, CVec2D b)
        => a.Ex.Identical(b.Ex) && a.Ey.Identical(b.Ey);

    internal static IEnumerable<CVec2D> ConnectedPoints(CVec2D point, IList<IBaseEquation> constraints)
    {
        var points = new List<CVec2D> { point };
        for (int i = 0; i < points.Count; i++)
        {
            yield return points[i];
            foreach (var constraint in constraints)
            {
                if (constraint is not PointOnPoint2d join) continue;
                var other = SamePoint(points[i], join.Point1) ? join.Point2
                    : SamePoint(points[i], join.Point2) ? join.Point1 : null;
                if (other != null && !points.Any(known => SamePoint(known, other))) points.Add(other);
            }
        }
    }
}
