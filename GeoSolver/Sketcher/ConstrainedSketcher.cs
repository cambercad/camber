using Curves;
using Curves.Base;
using GeoCore;
using GeoMeta;

namespace GeoSolver.Sketcher
{
    [APIDescription(@"ConstrainedSketcher: 2D sketch with parametric constraint solver. Inherits all PlotterSketcher / PlotterSketcherCoordSys methods.
Returned by GeoAPI.GetConstraintSketcher(...). Default for recreating user sketch images and dimensioned drawings (standard CAD). Add curves with AddLine / AddCircle / AddArc / AddRectangle / AddRectangleFromCorners - the overrides return constraint-aware variants (CLine2D / CCircle2D / CArc2D) but typed as their base; cast or use AddCLine / AddCCircle / AddCArc to keep the constrainable type.

Constraint workflow:
  1. Add curves (call .AddLine, .AddCLine, ...).
  2. Add constraints (call Set* / Fix* methods).
  3. Call SolveConstraints() (or rely on the implicit solve ? see SolveAfterEveryConstraint).

Read-only landmarks (use as anchors in constraints):
  Origin: CVec2D constant at (0, 0).
  OriginUnitX: CLine2D from (0,0) to (1,0) (helper geometry).
  OriginUnitY: CLine2D from (0,0) to (0,1) (helper geometry).

Constraint endpoint accessors on returned CLine2D: .CStart, .CEnd (CVec2D). For points at parameter u on a curve, use ""<curveName>@<u>"" via part.Get(name) or sketch.TryGetCPointOnEdge(name, out CVec2D).

Property: SolveAfterEveryConstraint (bool, default True) ? when False, batch many constraints before calling SolveConstraints() once (much faster).")]
    public partial class ConstrainedSketcher : PlotterSketcherCoordSys
    {
        protected List<CVec2D> points;

        protected List<IBaseEquation> constraints;

        private bool _solveAfterEveryConstraint = true;

        [APIDescription(@"Origin: CVec2D
Constant constrained point (0, 0) in sketch coordinates. Use as a fixed anchor in constraints.")]
        public readonly CVec2D Origin = CVec2D.Constant(0, 0);
        [APIDescription(@"OriginUnitX: CLine2D
Constant unit line from (0,0) to (1,0) (helper geometry). Useful as a horizontal reference for SetPointOnLine, SetAngle, etc.")]
        public readonly CLine2D OriginUnitX = new CLine2D(CVec2D.Constant(0, 0), CVec2D.Constant(1, 0), CurveFlags.HelperGeometry);
        [APIDescription(@"OriginUnitY: CLine2D
Constant unit line from (0,0) to (0,1) (helper geometry). Useful as a vertical reference.")]
        public readonly CLine2D OriginUnitY = new CLine2D(CVec2D.Constant(0, 0), CVec2D.Constant(0, 1), CurveFlags.HelperGeometry);

        /// <summary>
        /// When true (default), constraints are solved immediately after being added.
        /// Set to false to batch multiple constraints before solving.
        /// </summary>
        [APIDescription(@"SolveAfterEveryConstraint (bool, default True)
When True, every Set*/Fix* call triggers an implicit SolveConstraints(). Set to False to batch many constraints, then call SolveConstraints() explicitly (much faster for large sketches).")]
        public bool SolveAfterEveryConstraint 
        { 
            get { return _solveAfterEveryConstraint; } 
            set { _solveAfterEveryConstraint = value; } 
        }

        public ConstrainedSketcher(string name, CoordinateSystem cs, Vec2D startPoint, Vec2D startNormal = default) 
            : base(name, cs, startPoint, startNormal, new ConstrainableCurveFactory())
        {
            points = new List<CVec2D>();
            constraints = new List<IBaseEquation>();
        }
        public ConstrainedSketcher(string name, CoordinateSystem cs, BaseCurveFactory curveFactory = null)
          : base(name, cs, curveFactory ?? new ConstrainableCurveFactory())
        {
            points = new List<CVec2D>();
            constraints = new List<IBaseEquation>();
        }

        [APIDescription(@"AddCLine(startPoint: Vec2D, endPoint: Vec2D, flags: CurveFlags = None) -> CLine2D
Adds a constrainable line. Returns CLine2D with .CStart/.CEnd accessors usable in constraints. Equivalent to AddLine for constrained sketchers but keeps the concrete CLine2D return type.")]
        public CLine2D AddCLine(Vec2D startPoint, Vec2D endPoint, CurveFlags flags = CurveFlags.None)
        {
            CLine2D line = _curveFactory.CreateLine2D(startPoint, endPoint, flags) as CLine2D;
            AddCurveToStrip(line); // AddCurveToStrip auto-assigns name if needed
            return line;
        }
        [APIDescription(@"AddCLineStrip(points: IList[Vec2D], closeStrip: bool = False, flags: CurveFlags = None) -> List[CLine2D]
Adds a polyline of constrainable lines through `points` and inserts implicit point-on-point constraints between consecutive segments. If closeStrip=True, also connects the last segment's end to the first segment's start.")]
        public List<CLine2D> AddCLineStrip(IList<Vec2D> points, bool closeStrip = false, CurveFlags flags = CurveFlags.None)
        {
            List<CLine2D> result = new List<CLine2D>();
            CLine2D prev = null;
            CLine2D first = null;
            for (int i = 1; i < points.Count; i++)
            {
                var start = points[i - 1];
                var end = points[i];
                var curr = AddCLine(start, end, flags);
                if(prev != null)
                {
                    SetPointOnPoint(prev.CEnd, curr.CStart);
                }
                prev = curr;
                if(i==1)
                    first = curr;

                result.Add(curr);
            }
            if(closeStrip)
            {
                var start = points[points.Count - 1];
                var end = points[0];
                var curr = AddCLine(start, end, flags);

                if (prev != null)
                {
                    SetPointOnPoint(prev.CEnd, curr.CStart);
                }
                if (first != null)
                {
                    SetPointOnPoint(curr.CEnd, first.CStart);
                }

                prev = curr;

                result.Add(curr);
            }

            return result;
        }

        [APIDescription(@"AddConstraint(constraint: IBaseEquation)
Adds a raw constraint equation directly. Most users should call the typed Set*/Fix* methods instead.")]
        public void AddConstraint(IBaseEquation constraint)
        {
            constraints.Add(constraint);
        }

        /// <summary>Removes a constraint equation from this sketch (no solve).</summary>
        public bool RemoveConstraint(IBaseEquation constraint)
        {
            if (constraint == null) return false;
            return constraints.Remove(constraint);
        }

        /// <summary>
        /// Removes a curve and any constraints that share its free parameters.
        /// Returns the constraints that were removed.
        /// </summary>
        public List<IBaseEquation> RemoveCurve(Curve2D curve)
        {
            var removedCons = new List<IBaseEquation>();
            if (curve == null) return removedCons;

            HashSet<Param> curveParams = new HashSet<Param>();
            if (curve is IEnumerableParams enumerable)
            {
                foreach (var p in enumerable)
                    curveParams.Add(p);
            }

            if (curveParams.Count > 0)
            {
                for (int i = constraints.Count - 1; i >= 0; i--)
                {
                    bool shared = false;
                    foreach (var p in constraints[i])
                    {
                        if (curveParams.Contains(p))
                        {
                            shared = true;
                            break;
                        }
                    }
                    if (shared)
                    {
                        removedCons.Add(constraints[i]);
                        constraints.RemoveAt(i);
                    }
                }
            }

            RemoveStoredCurve(curve);
            return removedCons;
        }

        /// <summary>
        /// Fixes a point at the specified position.
        /// </summary>
        [APIDescription(@"FixPoint(point: CVec2D, fixPos: Vec2D)
Adds a constraint that pins `point` to the world-fixed location `fixPos`.")]
        public void FixPoint(CVec2D point, Vec2D fixPos)
        {
            constraints.Add(new PointOnPoint2d(point, CVec2D.Constant(fixPos.X, fixPos.Y)));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        /// <summary>
        /// Fixes a point at its current position (locks it in place).
        /// </summary>
        [APIDescription(@"FixPoint(point: CVec2D)
Pins `point` at its current evaluated position (locks it in place).")]
        public void FixPoint(CVec2D point)
        {
            Vec2D currentPos = point.Evaluate();
            FixPoint(point, currentPos);
        }

        [APIDescription(@"SetHorizontal(line: CLine2D)
Constrains `line` to be horizontal in the sketch plane (Y_start == Y_end).")]
        public void SetHorizontal(CLine2D line)
        {
            constraints.Add(new Horizontal2d(line));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetPointOnPoint(a: CVec2D, b: CVec2D)
Constrains points `a` and `b` to be coincident (same x and y). Equivalent to SetCoincident.")]
        public void SetPointOnPoint(CVec2D a, CVec2D b)
        {
            constraints.Add(new PointOnPoint2d(a, b));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetLength(line: CLine2D, length: float)
Constrains the Euclidean length of `line` to `length`. Implemented as SetDistancePointPoint(line.CStart, line.CEnd, length).")]
        public void SetLength(CLine2D line, double length)
        {
            SetDistancePointPoint(line.CStart, line.CEnd, length);
        }

        [APIDescription(@"SetDistancePointPoint(a: CVec2D, b: CVec2D, length: float)
Constrains the distance between points `a` and `b` to be exactly `length`.")]
        public void SetDistancePointPoint(CVec2D a, CVec2D b, double length)
        {
            constraints.Add(new DistanceBetweenPoints2d(a, b, length) { LabelAnchorWrtCenter = new Vec2D(0, 0.05 * length) });
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetHorizontalDistancePointPoint(a: CVec2D, b: CVec2D, distance: float)
Constrains b.x - a.x = distance (signed horizontal offset).")]
        public void SetHorizontalDistancePointPoint(CVec2D a, CVec2D b, double distance)
        {
            constraints.Add(new HorizontalDistanceBetweenPoints2d(a, b, distance) { LabelAnchorWrtCenter = new Vec2D(0, 0.05 * distance) });
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetVerticalDistancePointPoint(a: CVec2D, b: CVec2D, distance: float)
Constrains b.y - a.y = distance (signed vertical offset).")]
        public void SetVerticalDistancePointPoint(CVec2D a, CVec2D b, double distance)
        {
            constraints.Add(new VerticalDistanceBetweenPoints2d(a, b, distance) { LabelAnchorWrtCenter = new Vec2D(0.05 * distance, 0) });
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetVertical(line: CLine2D)
Constrains `line` to be vertical in the sketch plane (X_start == X_end).")]
        public void SetVertical(CLine2D line)
        {
            constraints.Add(new Vertical2d(line));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetPerpendicular(line1: CLine2D, line2: CLine2D)
Constrains the two lines to be perpendicular (dot of their direction vectors = 0).")]
        public void SetPerpendicular(CLine2D line1, CLine2D line2)
        {
            constraints.Add(new Perpendicular2d(line1, line2));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetParallel(line1: CLine2D, line2: CLine2D)
Constrains the two lines to be parallel (cross product of direction vectors = 0).")]
        public void SetParallel(CLine2D line1, CLine2D line2)
        {
            constraints.Add(new Parallel2d(line1, line2));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetEqualLength(line1: CLine2D, line2: CLine2D)
Constrains the two lines to have the same length.")]
        public void SetEqualLength(CLine2D line1, CLine2D line2)
        {
            constraints.Add(new EqualLength2d(line1, line2));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetDistance(point1: CVec2D, point2: CVec2D, distance: float)
Constrains the Euclidean distance between two points (same constraint type as SetDistancePointPoint, no label offset).")]
        public void SetDistance(CVec2D point1, CVec2D point2, double distance)
        {
            constraints.Add(new DistanceBetweenPoints2d(point1, point2, distance));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetHorizontalDistance(point1: CVec2D, point2: CVec2D, distance: float)
Constrains point2.x - point1.x = distance.")]
        public void SetHorizontalDistance(CVec2D point1, CVec2D point2, double distance)
        {
            constraints.Add(new HorizontalDistanceBetweenPoints2d(point1, point2, distance));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetVerticalDistance(point1: CVec2D, point2: CVec2D, distance: float)
Constrains point2.y - point1.y = distance.")]
        public void SetVerticalDistance(CVec2D point1, CVec2D point2, double distance)
        {
            constraints.Add(new VerticalDistanceBetweenPoints2d(point1, point2, distance));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetCoincident(point1: CVec2D, point2: CVec2D)
Constrains the two points to share the same position. Same effect as SetPointOnPoint.")]
        public void SetCoincident(CVec2D point1, CVec2D point2)
        {
            constraints.Add(new PointOnPoint2d(point1, point2));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        //public void PointOnLine(CVec2D point, CLine2D line)
        //{
        //    constraints.Add(new PointOnLineConstraint2d(point, line));
        //    if (_solveAfterEveryConstraint)
        //        SolveConstraints();
        //}

        [APIDescription(@"SetMidpoint(point: CVec2D, line: CLine2D)
Constrains `point` to be the midpoint of `line` (point on line at parameter 0.5).")]
        public void SetMidpoint(CVec2D point, CLine2D line)
        {
            constraints.Add(new Midpoint2d(point, line));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        //public void PointOnCircle(CVec2D point, CCircle2D circle)
        //{
        //    constraints.Add(new PointOnCircle2d(point, circle));
        //    if (_solveAfterEveryConstraint)
        //        SolveConstraints();
        //}

        [APIDescription(@"SetPointOnCircle(point: CVec2D, circle: CCircle2D)
Constrains `point` to lie exactly on `circle`'s circumference.")]
        public void SetPointOnCircle(CVec2D point, CCircle2D circle)
        {
            constraints.Add(new PointOnCircle2d(point, circle));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetPointOnArc(point: CVec2D, arc: CArc2D)
Constrains `point` to lie on `arc` (alias for PointOnArc).")]
        public void SetPointOnArc(CVec2D point, CArc2D arc)
        {
            PointOnArc(point, arc);
        }

        [APIDescription(@"SetTangent(line: CLine2D, circular: ICircular2D)
Constrains `line` to be tangent to a circle or arc (`circular` can be CCircle2D or CArc2D).")]
        public void SetTangent(CLine2D line, ICircular2D circular)
        {
            constraints.Add(new TangentLineCircular2d(line, circular));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetConcentric(circular1: ICircular2D, circular2: ICircular2D)
Constrains two circles/arcs to share the same center.")]
        public void SetConcentric(ICircular2D circular1, ICircular2D circular2)
        {
            constraints.Add(new Concentric2d(circular1, circular2));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetEqualRadius(circular1: ICircular2D, circular2: ICircular2D)
Constrains two circles/arcs to have the same radius.")]
        public void SetEqualRadius(ICircular2D circular1, ICircular2D circular2)
        {
            constraints.Add(new EqualRadius2d(circular1, circular2));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetRadius(circular: ICircular2D, radius: float)
Constrains the radius of a circle or arc to `radius`.")]
        public void SetRadius(ICircular2D circular, double radius)
        {
            constraints.Add(new RadiusConstraint2d(circular, radius));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetSymmetric(point1: CVec2D, point2: CVec2D, symmetryLine: CLine2D)
Constrains `point1` and `point2` to be mirror images across `symmetryLine`.")]
        public void SetSymmetric(CVec2D point1, CVec2D point2, CLine2D symmetryLine)
        {
            constraints.Add(new SymmetricAboutLine2d(point1, point2, symmetryLine));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetAngle(line1: CLine2D, line2: CLine2D, angleRadians: float)
Constrains the (signed) angle between line1's and line2's direction vectors.")]
        public void SetAngle(CLine2D line1, CLine2D line2, double angleRadians)
        {
            constraints.Add(new AngleBetweenLines2d(line1, line2, angleRadians));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }


        [APIDescription(@"SetPointOnLine(point: CVec2D, line: CLine2D)
Constrains `point` to lie anywhere on the infinite line through `line.CStart` and `line.CEnd`.")]
        public void SetPointOnLine(CVec2D point, CLine2D line)
        {
            constraints.Add(new PointOnLineConstraint2d(point, line));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        /// <summary>
        /// Constrains the perpendicular distance from a point to a line.
        /// </summary>
        [APIDescription(@"SetDistancePointLine(point: CVec2D, line: CLine2D, distance: float)
Constrains the perpendicular distance from `point` to the infinite line of `line` to be `distance`.")]
        public void SetDistancePointLine(CVec2D point, CLine2D line, double distance)
        {
            constraints.Add(new DistancePointLine2d(point, line, distance));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"SetAngleDegrees(line1: CLine2D, line2: CLine2D, angleDegrees: float)
Same as SetAngle but `angleDegrees` is in degrees.")]
        public void SetAngleDegrees(CLine2D line1, CLine2D line2, double angleDegrees)
        {
            SetAngle(line1, line2, angleDegrees * Math.PI / 180.0);
        }

        /// <summary>
        /// Constrains two lines to be colinear (lie on the same infinite line).
        /// </summary>
        [APIDescription(@"SetColinear(line1: CLine2D, line2: CLine2D)
Constrains both lines to lie on the same infinite line (parallel + zero perpendicular distance).")]
        public void SetColinear(CLine2D line1, CLine2D line2)
        {
            constraints.Add(new Colinear2d(line1, line2));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        /// <summary>
        /// Constrains two circular entities (arcs or circles) to be tangent to each other.
        /// </summary>
        /// <param name="circular1">First circle or arc</param>
        /// <param name="circular2">Second circle or arc</param>
        /// <param name="tangentType">Type of tangency: Auto (default), External, or Internal.</param>
        [APIDescription(@"SetTangentCircles(circular1: ICircular2D, circular2: ICircular2D, tangentType: CircleTangentType = Auto)
Constrains two circles/arcs to be tangent.
  tangentType: CircleTangentType.Auto (decide from current geometry; both branches valid) | External (touch outside, distance = R1+R2) | Internal (one inside the other, distance = |R1-R2|).")]
        public void SetTangentCircles(ICircular2D circular1, ICircular2D circular2, CircleTangentType tangentType = CircleTangentType.Auto)
        {
            constraints.Add(new TangentCircularCircular2d(circular1, circular2, tangentType));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        /// <summary>
        /// Constrains a point to lie on an arc.
        /// </summary>
        [APIDescription(@"PointOnArc(point: CVec2D, arc: CArc2D)
Constrains `point` to lie on `arc` (within the arc's angular range, not the full circle).")]
        public void PointOnArc(CVec2D point, CArc2D arc)
        {
            constraints.Add(new PointOnArc2d(point, arc));
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        [APIDescription(@"AddCCircle(center: Vec2D, radius: float, flags: CurveFlags = None) -> CCircle2D
Adds a constrainable circle (returns CCircle2D for use in SetRadius / SetTangent / etc.).")]
        public CCircle2D AddCCircle(Vec2D center, double radius, CurveFlags flags = CurveFlags.None)
        {
            CCircle2D circle = _curveFactory.CreateCircle2D(center, radius, flags) as CCircle2D;
            AddCurveToStrip(circle); // names the curve and records it; contours come from connectivity
            return circle;
        }

        [APIDescription(@"AddCArc(start: Vec2D, pointOnArc: Vec2D, end: Vec2D, flags: CurveFlags = None) -> CArc2D
Adds a constrainable three-point arc. Internal unit-vector constraints for the arc's parameterization are added automatically.")]
        public CArc2D AddCArc(Vec2D start, Vec2D pointOnArc, Vec2D end, CurveFlags flags = CurveFlags.None)
        {
            CArc2D arc = _curveFactory.CreateArc2D(start, pointOnArc, end, flags) as CArc2D;
            AddCurveToStrip(arc); // AddCurveToStrip auto-assigns name if needed
            
            // Add internal constraints for the arc (unit vector normalization)
            foreach (var constraint in arc.GetInternalConstraints())
            {
                constraints.Add(constraint);
            }
            
            return arc;
        }

        [APIDescription(@"AddCArc(start: Vec2D, end: Vec2D, center: Vec2D, shorter: bool, flags: CurveFlags = None) -> CArc2D
Adds a constrainable arc by start/end and explicit center. shorter=True picks the shorter sweep around the circle.")]
        public CArc2D AddCArc(Vec2D start, Vec2D end, Vec2D center, bool shorter, CurveFlags flags = CurveFlags.None)
        {
            CArc2D arc = _curveFactory.CreateArc2D(start, end, center, shorter, flags) as CArc2D;
            AddCurveToStrip(arc); // AddCurveToStrip auto-assigns name if needed
            
            // Add internal constraints for the arc (unit vector normalization)
            foreach (var constraint in arc.GetInternalConstraints())
            {
                constraints.Add(constraint);
            }
            
            return arc;
        }

        /// <summary>
        /// Override AddArc to create a constrainable CArc2D instead of a plain Arc2D.
        /// Use this for constrained sketching where constraints like SetRadius will be applied.
        /// </summary>
        public override Arc2D AddArc(Vec2D start, Vec2D pointOnArc, Vec2D end, CurveFlags flags = CurveFlags.None)
        {
            return AddCArc(start, pointOnArc, end, flags);
        }

        /// <summary>
        /// Override AddLine to create a constrainable CLine2D instead of a plain Line2D.
        /// Use this for constrained sketching where constraints will be applied.
        /// </summary>
        public override Line2D AddLine(Vec2D startPoint, Vec2D endPoint, CurveFlags flags = CurveFlags.None)
        {
            return AddCLine(startPoint, endPoint, flags);
        }

        /// <summary>
        /// Override AddCircle to create a constrainable CCircle2D instead of a plain Circle2D.
        /// Use this for constrained sketching where constraints like SetRadius will be applied.
        /// </summary>
        public override Circle2D AddCircle(Vec2D center, double radius, CurveFlags flags = CurveFlags.None)
        {
            return AddCCircle(center, radius, flags);
        }

        /// <summary>
        /// Adds a constrained rectangle defined by two opposite corners.
        /// </summary>
        [APIDescription(@"AddCRectangleFromCorners(cornerA: Vec2D, cornerB: Vec2D, flags: CurveFlags = None) -> List[CLine2D]
Adds a constrained rectangle as 4 CLine2D edges (CCW from bottom-left). Auto-adds: corner point-on-point constraints, parallel constraints (top || bottom, left || right), and one perpendicular constraint to fix the rectangle shape. Corner order does not matter.")]
        public List<CLine2D> AddCRectangleFromCorners(Vec2D cornerA, Vec2D cornerB, CurveFlags flags = CurveFlags.None)
        {
            // Compute real min/max in case values are swapped
            Vec2D realMin = new Vec2D(Math.Min(cornerA.X, cornerB.X), Math.Min(cornerA.Y, cornerB.Y));
            Vec2D realMax = new Vec2D(Math.Max(cornerA.X, cornerB.X), Math.Max(cornerA.Y, cornerB.Y));

            Vec2D bottomLeft = realMin;
            Vec2D bottomRight = new Vec2D(realMax.X, realMin.Y);
            Vec2D topRight = realMax;
            Vec2D topLeft = new Vec2D(realMin.X, realMax.Y);

            // Temporarily disable solving to batch all operations
            bool wasSolving = _solveAfterEveryConstraint;
            _solveAfterEveryConstraint = false;

            var lines = new List<CLine2D>();
            var line1 = AddCLine(bottomLeft, bottomRight, flags);
            var line2 = AddCLine(bottomRight, topRight, flags);
            var line3 = AddCLine(topRight, topLeft, flags);
            var line4 = AddCLine(topLeft, bottomLeft, flags);

            lines.Add(line1);
            lines.Add(line2);
            lines.Add(line3);
            lines.Add(line4);

            // Connect the lines with point-on-point constraints
            constraints.Add(new PointOnPoint2d(line1.CEnd, line2.CStart));
            constraints.Add(new PointOnPoint2d(line2.CEnd, line3.CStart));
            constraints.Add(new PointOnPoint2d(line3.CEnd, line4.CStart));
            constraints.Add(new PointOnPoint2d(line4.CEnd, line1.CStart));

            // Add rectangle shape constraints: parallel opposite sides and one perpendicular
            constraints.Add(new Parallel2d(line1, line3)); // bottom parallel to top
            constraints.Add(new Parallel2d(line2, line4)); // right parallel to left
            constraints.Add(new Perpendicular2d(line1, line2)); // bottom perpendicular to right

            // Restore solving behavior and solve if needed
            _solveAfterEveryConstraint = wasSolving;
            if (_solveAfterEveryConstraint)
                SolveConstraints();

            return lines;
        }

        /// <summary>
        /// Override AddRectangleFromCorners to create constrainable CLine2D edges instead of plain Line2D.
        /// </summary>
        public override List<Line2D> AddRectangleFromCorners(Vec2D cornerA, Vec2D cornerB, CurveFlags flags = CurveFlags.None)
        {
            var clines = AddCRectangleFromCorners(cornerA, cornerB, flags);
            return clines.Cast<Line2D>().ToList();
        }

        /// <summary>
        /// Sets the end of line1 coincident with the start of line2.
        /// </summary>
        [APIDescription(@"ConnectLines(line1: CLine2D, line2: CLine2D)
Constrains line1.CEnd to be coincident with line2.CStart.")]
        public void ConnectLines(CLine2D line1, CLine2D line2)
        {
            SetCoincident(line1.CEnd, line2.CStart);
        }

        /// <summary>
        /// Connects all lines in sequence (end of each line to start of next).
        /// </summary>
        [APIDescription(@"ConnectLinesInSequence(*lines: CLine2D)
Constrains the end of each line to the start of the next, in order. Variadic argument list of CLine2D.")]
        public void ConnectLinesInSequence(params CLine2D[] lines)
        {
            bool wasSolving = _solveAfterEveryConstraint;
            _solveAfterEveryConstraint = false;
            
            for (int i = 0; i < lines.Length - 1; i++)
            {
                constraints.Add(new PointOnPoint2d(lines[i].CEnd, lines[i + 1].CStart));
            }
            
            _solveAfterEveryConstraint = wasSolving;
            if (_solveAfterEveryConstraint)
                SolveConstraints();
        }

        /// <summary>
        /// Closes a loop by connecting the end of the last line to the start of the first line.
        /// </summary>
        [APIDescription(@"CloseLoop(firstLine: CLine2D, lastLine: CLine2D)
Constrains lastLine.CEnd to be coincident with firstLine.CStart, closing a polyline loop.")]
        public void CloseLoop(CLine2D firstLine, CLine2D lastLine)
        {
            SetCoincident(lastLine.CEnd, firstLine.CStart);
        }

        [APIDescription(@"GetConstraints() -> IList[IBaseEquation]
Live list of constraint equations on this sketch.")]
        public IList<IBaseEquation> GetConstraints()
        {
            return constraints;
        }

        /// <summary>
        /// Gets a curve by name.
        /// </summary>
        [APIDescription(@"GetCurve(name: str) -> Curve2D
Returns the curve with this exact name in this sketch. Throws if not found.")]
        public Curve2D GetCurve(string name)
        {
            Curve2D found = FindStoredCurveByName(name);
            if (found != null)
                return found;
            throw new Exception($"Curve '{name}' not found in sketch '{_name}'");
        }

        /// <summary>
        /// Try to get a constrained point (CVec2D) on a curve edge by name.
        /// Name format: "curveName@uniform" where uniform is a value like 0.000, 0.500, or 1.000
        /// Or "curveName@center" for circles/arcs to get the center point.
        /// Example: "Line1@0.000" returns the start of Line1, "Line1@0.500" returns the midpoint
        /// The returned CVec2D is expression-based, meaning it depends on the curve's parameters
        /// and will update when the curve moves. For lines at exactly 0 or 1, returns the 
        /// actual CStart/CEnd parameters for direct constraint compatibility.
        /// </summary>
        [APIDescription(@"TryGetCPointOnEdge(name: str, out result: CVec2D) -> bool
Resolves a constrained point on a curve edge by name.
  Name: ""<curveName>@<u>"" with u in [0,1] (e.g. ""Line1@0.5"" = midpoint), or ""<curveName>@center"" for circle/arc center.
For CLine2D at exactly u=0 or u=1, returns the actual CStart/CEnd parameter (so the result can be used in further constraints). Otherwise returns an expression-based CVec2D that updates as the curve changes.")]
        public bool TryGetCPointOnEdge(string name, out CVec2D result)
        {
            result = default;
            if (!EntityNaming.TryParseSketchCurveAddress(name, out var address))
                return false;

            string curveName = address.CurveName;
            bool isCenter = address.IsCenter;
            double uniformParam = address.Uniform;

            Curve2D foundCurve = FindStoredCurveByName(curveName);

            if (foundCurve == null)
                return false;

            // Handle @center for circular curves
            if (isCenter)
            {
                if (foundCurve is ICircular2D circular)
                {
                    result = circular.CCenter;
                    return true;
                }
                return false; // @center only valid for circles/arcs
            }

            // B?zier @cvN addresses are geometric CVs, not constrainable CPoints.
            if (address.IsControlVertex)
                return false;

            const double tolerance = 1e-9;

            // For CLine2D, return the actual CStart/CEnd parameters at 0/1 for better constraint compatibility
            if (foundCurve is CLine2D cline)
            {
                if (Math.Abs(uniformParam) < tolerance)
                {
                    result = cline.CStart;
                    return true;
                }
                if (Math.Abs(uniformParam - 1.0) < tolerance)
                {
                    result = cline.CEnd;
                    return true;
                }
            }

            // Use IEvaluateCPoint interface for all other cases (intermediate params, arcs, circles)
            if (foundCurve is IEvaluateCPoint evaluatable)
            {
                result = evaluatable.Evaluate(uniformParam);
                return true;
            }
            
            // Non-constrained curve types don't have CPoints
            return false;
        }
        [APIDescription(@"SolveConstraints(preferMinimalMovement: bool = True) -> float
Runs the numeric constraint solver on this sketch and returns the residual error after the solve. Returned value near 0 means all constraints satisfied; large value means an over-constrained / inconsistent system.
  preferMinimalMovement: when True, the solver prefers solutions close to the current parameter values (gentler updates), which usually gives more intuitive results when multiple solutions exist.
External parameters referenced by constraints (e.g. points from another sketch) are temporarily frozen during the solve.")]
        public double SolveConstraints(bool preferMinimalMovement = true)
        {
            HashSet<Param> draggedParams = null;
            if (preferMinimalMovement)
            {
                draggedParams = new HashSet<Param>();
                foreach (var constraint in constraints)
                {
                    foreach (var param in constraint)
                        draggedParams.Add(param);
                }
            }

            return SolveInternal(draggedParams);
        }

        /// <summary>
        /// True when <paramref name="point"/> is a free parameter point that can be interactively dragged
        /// (both coordinates are independent parameters and not frozen). Constants / derived expressions cannot.
        /// </summary>
        public static bool CanDragPoint(CVec2D point)
        {
            if (point == null || point.IsFrozen)
                return false;
            if (point.Ex.Type != ExprType.Parameter || point.Ey.Type != ExprType.Parameter)
                return false;
            if (point.Ex.Value == null || point.Ey.Value == null)
                return false;
            return !point.Ex.Value.Frozen && !point.Ey.Value.Frozen;
        }

        /// <summary>
        /// Interactive drag: move a free sketch point to <paramref name="target"/> and resolve constraints,
        /// soft-pinning the point's parameters so underconstrained DOFs follow/leave it as needed.
        /// Fully constrained points snap back; returns false if the point cannot be dragged.
        /// </summary>
        public bool TryDragPoint(CVec2D point, Vec2D target)
        {
            if (!CanDragPoint(point))
                return false;

            point.Ex.SetValue(target.X);
            point.Ey.SetValue(target.Y);

            var draggedParams = new HashSet<Param> { point.Ex.Value, point.Ey.Value };
            SolveInternal(draggedParams);
            return true;
        }

        /// <summary>
        /// True when <paramref name="radius"/> is an independent unfrozen parameter (e.g. circle/arc radius).
        /// </summary>
        public static bool CanDragRadius(Expr radius)
        {
            if (radius == null || radius.Type != ExprType.Parameter || radius.Value == null)
                return false;
            return !radius.Value.Frozen;
        }

        /// <summary>
        /// Soft-pin a circle/arc radius and resolve constraints.
        /// </summary>
        public bool TryDragRadius(Expr radius, double newRadius)
        {
            if (!CanDragRadius(radius) || newRadius < 1e-9)
                return false;

            radius.SetValue(newRadius);
            SolveInternal(new HashSet<Param> { radius.Value });
            return true;
        }

        /// <summary>
        /// Translate several free points by the same delta (e.g. move a line via its midpoint).
        /// </summary>
        public bool TryTranslatePoints(Vec2D delta, params CVec2D[] points)
        {
            if (points == null || points.Length == 0)
                return false;

            var draggedParams = new HashSet<Param>();
            foreach (var point in points)
            {
                if (!CanDragPoint(point))
                    return false;

                Vec2D cur = point.Evaluate();
                point.Ex.SetValue(cur.X + delta.X);
                point.Ey.SetValue(cur.Y + delta.Y);
                draggedParams.Add(point.Ex.Value);
                draggedParams.Add(point.Ey.Value);
            }

            SolveInternal(draggedParams);
            return true;
        }

        /// <summary>
        /// Drag an arc endpoint marker: aim start/end direction at <paramref name="target"/> and
        /// soft-pin radius to the distance from center (when radius is free).
        /// </summary>
        public bool TryDragArcEnd(CArc2D arc, bool startEnd, Vec2D target)
        {
            if (arc == null)
                return false;

            CVec2D dir = startEnd ? arc.CStartDir : arc.CEndDir;
            if (!CanDragPoint(dir))
                return false;

            Vec2D center = arc.CCenter.Evaluate();
            Vec2D offset = target - center;
            double r = offset.Length();
            if (r < 1e-9)
                return false;

            Vec2D unit = offset / r;
            dir.Ex.SetValue(unit.X);
            dir.Ey.SetValue(unit.Y);

            var draggedParams = new HashSet<Param> { dir.Ex.Value, dir.Ey.Value };
            if (CanDragRadius(arc.CRadius))
            {
                arc.CRadius.SetValue(r);
                draggedParams.Add(arc.CRadius.Value);
            }

            SolveInternal(draggedParams);
            return true;
        }

        private double SolveInternal(HashSet<Param> draggedParams)
        {
            const double scaling = 1.0;

            HashSet<Param> sketchParams = new HashSet<Param>();
            foreach (var curve in EnumerateCurves())
            {
                if (curve is IEnumerableParams enumerable)
                {
                    foreach (var param in enumerable)
                        sketchParams.Add(param);
                }
            }

            List<Param> externalParams = new List<Param>();
            foreach (var constraint in constraints)
            {
                foreach (var param in constraint)
                {
                    if (!sketchParams.Contains(param) && !param.Frozen)
                    {
                        param.Frozen = true;
                        externalParams.Add(param);
                    }
                }
            }

            try
            {
                int numParams, numConstraints;
                double error = ConstraintSolver.Solve(constraints, draggedParams, scaling, out numParams, out numConstraints);
                UpdateAllObjects();
                return error;
            }
            finally
            {
                foreach (var param in externalParams)
                    param.Frozen = false;
            }
        }
        
        private void UpdateAllObjects()
        {
            foreach (var curve in EnumerateCurves())
            {
                if (curve is IUpdate updateable && curve is not IDependentSketchCurve)
                    updateable.Update();
            }

            foreach (var curve in EnumerateCurves())
            {
                if (curve is IDependentSketchCurve dependent)
                    dependent.Update();
            }
        }

        /// <summary>
        /// Freezes all curves in this sketch, preventing their parameters from being 
        /// modified by the solver. Use this when referencing curves from this sketch 
        /// in another sketch's constraints.
        /// </summary>
        [APIDescription(@"FreezeAllCurves()
Locks all curves in this sketch so their parameters cannot be modified by the solver. Call this when curves of this sketch are referenced by another sketch's constraints (so the other sketch's solve can't drag them around).")]
        public void FreezeAllCurves()
        {
            foreach (var curve in EnumerateCurves())
            {
                if (curve is IFreezable freezable)
                    freezable.Freeze();
            }
        }

        /// <summary>
        /// Unfreezes all curves in this sketch, allowing their parameters to be 
        /// modified by the solver again.
        /// </summary>
        [APIDescription(@"UnfreezeAllCurves()
Unlocks all curves frozen by FreezeAllCurves so they can be moved again.")]
        public void UnfreezeAllCurves()
        {
            foreach (var curve in EnumerateCurves())
            {
                if (curve is IFreezable freezable)
                    freezable.Unfreeze();
            }
        }

    }
}
