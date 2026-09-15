using Curves;
using GeoCore;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using System.Runtime.CompilerServices;

namespace GeoSolver
{
    public interface IBaseEquation : IEnumerableParams
    {
        public void GenerateEquations(List<Expr> equations, double scaling);
    }

    public interface IUpdate
    {
        public void Update();
    }
    public interface IEvaluateCPoint
    {
        public CVec2D Evaluate(double uniform);
    }

    /// <summary>
    /// Interface for circular geometry (circles and arcs) that have a center and radius.
    /// </summary>
    public interface ICircular2D : IEnumerableParams
    {
        CVec2D CCenter { get; }
        Expr CRadius { get; }
    }

    /// <summary>
    /// Interface for objects that can be frozen to prevent modification during constraint solving.
    /// </summary>
    public interface IFreezable
    {
        /// <summary>
        /// Freezes all parameters, preventing them from being modified by the solver.
        /// </summary>
        void Freeze();
        
        /// <summary>
        /// Unfreezes all parameters, allowing them to be modified by the solver.
        /// </summary>
        void Unfreeze();
        
        /// <summary>
        /// Gets whether this object is currently frozen.
        /// </summary>
        bool IsFrozen { get; }
    }

    /// <summary>
    /// Tangent type for circle-circle tangency constraints.
    /// </summary>
    public enum CircleTangentType
    {
        /// <summary>
        /// Automatically determine based on current geometry - both configurations are valid.
        /// </summary>
        Auto,
        /// <summary>
        /// External tangent: circles touch on the outside (distance = R1 + R2).
        /// </summary>
        External,
        /// <summary>
        /// Internal tangent: one circle is inside the other (distance = |R1 - R2|).
        /// </summary>
        Internal
    }

    public class CVec2D : IEnumerableParams, IFreezable
    {
        private Expr _x; public Expr Ex { get { return _x; } }
        private Expr _y; public Expr Ey { get { return _y; } }

        public CVec2D(double x, double y)
        {
            _x = Expr.Parameter(x);
            _y = Expr.Parameter(y);
        }
        public CVec2D(Expr x, Expr y)
        {
            _x = x;
            _y = y;
        }

        public static CVec2D Constant(double x, double y)
        {
            return new CVec2D(Expr.Constant(x), Expr.Constant(y));
        }

        public CVec2D(Vec2D v) : this(v.X, v.Y) { }

        public Vec2D Evaluate() { return new Vec2D(_x.Evaluate(), _y.Evaluate()); }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in _x) yield return p;
            foreach (Param p in _y) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }

        public override string ToString()
        {
            return $"({_x.Evaluate():F4}, {_y.Evaluate():F4})";
        }

        public static implicit operator Vec2D(CVec2D p)
        {
            return p.Evaluate();
        }

        /// <summary>
        /// Freezes all parameters, preventing them from being modified by the solver.
        /// </summary>
        public void Freeze()
        {
            foreach (Param p in this)
                p.Frozen = true;
        }

        /// <summary>
        /// Unfreezes all parameters, allowing them to be modified by the solver.
        /// </summary>
        public void Unfreeze()
        {
            foreach (Param p in this)
                p.Frozen = false;
        }

        /// <summary>
        /// Gets whether this point is currently frozen.
        /// </summary>
        public bool IsFrozen
        {
            get
            {
                foreach (Param p in this)
                    if (p.Frozen) return true;
                return false;
            }
        }
    }

    /// <summary>
    /// C prefix indicates "constrainable"
    /// </summary>
    public class CLine2D : Line2D, IUpdate, IEnumerableParams, IEvaluateCPoint, IFreezable
    {
        public CVec2D CStart { get; private set; }
        public CVec2D CEnd { get; private set; }

        public CLine2D(Vec2D start, Vec2D end, CurveFlags curveFlags = CurveFlags.None) : base(start, end, curveFlags)
        {
            CStart = new CVec2D(start);
            CEnd = new CVec2D(end);
        }

        public CLine2D(CVec2D cStart, CVec2D cEnd, CurveFlags curveFlags = CurveFlags.None) : base(cStart.Evaluate(), cEnd.Evaluate(), curveFlags)
        {
            CStart = cStart;
            CEnd = cEnd;
        }

        public void Update()
        {
            base._start = CStart.Evaluate();
            base._end = CEnd.Evaluate();
        }

        /// <summary>
        /// Evaluates a point on the line at the given uniform parameter.
        /// Returns CStart + t * (CEnd - CStart) as an expression-based CVec2D.
        /// </summary>
        public CVec2D Evaluate(double uniform)
        {
            var t = Expr.Constant(uniform);
            // P = Start + t * (End - Start)
            var x = CStart.Ex + t * (CEnd.Ex - CStart.Ex);
            var y = CStart.Ey + t * (CEnd.Ey - CStart.Ey);
            return new CVec2D(x, y);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in CStart) yield return p;
            foreach (Param p in CEnd) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }

        public override string ToString()
        {
            var s = CStart.Evaluate();
            var e = CEnd.Evaluate();
            double len = (e - s).Length();
            return $"Line: Start={CStart}, End={CEnd}, Length={len:F4}";
        }

        /// <summary>
        /// Freezes all parameters, preventing them from being modified by the solver.
        /// </summary>
        public void Freeze()
        {
            CStart.Freeze();
            CEnd.Freeze();
        }

        /// <summary>
        /// Unfreezes all parameters, allowing them to be modified by the solver.
        /// </summary>
        public void Unfreeze()
        {
            CStart.Unfreeze();
            CEnd.Unfreeze();
        }

        /// <summary>
        /// Gets whether this line is currently frozen.
        /// </summary>
        public bool IsFrozen => CStart.IsFrozen || CEnd.IsFrozen;
    }

    public class CCircle2D : Circle2D, IUpdate, IEnumerableParams, IEvaluateCPoint, ICircular2D, IFreezable
    {
        public CVec2D CCenter { get; private set; }
        public Expr CRadius { get; private set; }

        public CCircle2D(Vec2D center, double radius, CurveFlags curveFlags = CurveFlags.None) : base(center, radius, curveFlags)
        {
            CCenter = new CVec2D(center);
            CRadius = Expr.Parameter(radius);
        }

        public void Update()
        {
            base._center = CCenter.Evaluate();
            base._radius = CRadius.Evaluate();
        }

        /// <summary>
        /// Evaluates a point on the circle at the given uniform parameter.
        /// uniform=0 is at angle 0 (right), uniform=0.25 is at angle PI/2 (top), etc.
        /// Returns Center + Radius * (Cos(2*PI*uniform), Sin(2*PI*uniform)) as an expression-based CVec2D.
        /// </summary>
        public CVec2D Evaluate(double uniform)
        {
            var angle = Expr.Constant(2.0 * Math.PI * uniform);
            // P = Center + Radius * (Cos(angle), Sin(angle))
            var x = CCenter.Ex + CRadius * Expr.Cos(angle);
            var y = CCenter.Ey + CRadius * Expr.Sin(angle);
            return new CVec2D(x, y);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in CCenter) yield return p;
            foreach (Param p in CRadius) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }

        public override string ToString()
        {
            return $"Circle: Center={CCenter}, Radius={CRadius.Evaluate():F4}";
        }

        /// <summary>
        /// Freezes all parameters, preventing them from being modified by the solver.
        /// </summary>
        public void Freeze()
        {
            CCenter.Freeze();
            foreach (Param p in CRadius)
                p.Frozen = true;
        }

        /// <summary>
        /// Unfreezes all parameters, allowing them to be modified by the solver.
        /// </summary>
        public void Unfreeze()
        {
            CCenter.Unfreeze();
            foreach (Param p in CRadius)
                p.Frozen = false;
        }

        /// <summary>
        /// Gets whether this circle is currently frozen.
        /// </summary>
        public bool IsFrozen
        {
            get
            {
                if (CCenter.IsFrozen) return true;
                foreach (Param p in CRadius)
                    if (p.Frozen) return true;
                return false;
            }
        }
    }

    public class CArc2D : Arc2D, IUpdate, IEnumerableParams, IEvaluateCPoint, ICircular2D, IFreezable
    {
        public CVec2D CCenter { get; private set; }
        public Expr CRadius { get; private set; }
        
        // Use direction unit vectors instead of bare angles to prevent solver divergence
        // CStartDir and CEndDir store (cos(angle), sin(angle)) as separate parameters
        // with an implicit unit length constraint
        public CVec2D CStartDir { get; private set; }
        public CVec2D CEndDir { get; private set; }

        /// <summary>
        /// Start point of the arc. Derived from CCenter, CRadius, and CStartDir
        /// (same as Evaluate(0)); not independent degrees of freedom.
        /// </summary>
        public CVec2D CStart => Evaluate(0);

        /// <summary>
        /// End point of the arc. Derived from CCenter, CRadius, and CEndDir
        /// (same as Evaluate(1)); not independent degrees of freedom.
        /// </summary>
        public CVec2D CEnd => Evaluate(1);
        
        // Track whether the arc goes CCW (positive sweep) or CW (negative sweep)
        private bool _isCounterClockwise;

        public CArc2D(Vec2D center, double radius, double angleStart, double angleEnd, CurveFlags curveFlags = CurveFlags.None) : base(center, radius, angleStart, angleEnd, curveFlags)
        {
            Init();
        }

        public CArc2D(Vec2D start, Vec2D end, Vec2D center, bool shorter, CurveFlags curveFlags = CurveFlags.None)
            : base(start, end, center, shorter, curveFlags)
        {
            Init();
        }

        public CArc2D(Vec2D start, Vec2D pointOnArc, Vec2D end, CurveFlags curveFlags = CurveFlags.None) : base(start, pointOnArc, end, curveFlags)
        {
            Init();
        }

        private void Init()
        {
            CCenter = new CVec2D(base.Center);
            CRadius = Expr.Parameter(base.Radius);
            
            // Initialize direction vectors from angles
            // These are unit vectors: (cos(angle), sin(angle))
            double cosStart = Math.Cos(base.AngleStart);
            double sinStart = Math.Sin(base.AngleStart);
            double cosEnd = Math.Cos(base.AngleEnd);
            double sinEnd = Math.Sin(base.AngleEnd);
            
            CStartDir = new CVec2D(cosStart, sinStart);
            CEndDir = new CVec2D(cosEnd, sinEnd);
            
            // Determine arc direction from the original sweep
            _isCounterClockwise = (base.AngleEnd - base.AngleStart) >= 0;
        }

        public void Update()
        {
            base._center = CCenter.Evaluate();
            base._radius = CRadius.Evaluate();
            
            // Get direction vectors and normalize them (in case solver drifted)
            Vec2D startDir = CStartDir.Evaluate();
            Vec2D endDir = CEndDir.Evaluate();
            
            // Normalize the direction vectors
            double startLen = startDir.Length();
            double endLen = endDir.Length();
            
            if (startLen > 1e-10)
            {
                startDir = startDir / startLen;
                // Update the parameters to stay normalized
                CStartDir.Ex.Value.Value = startDir.X;
                CStartDir.Ey.Value.Value = startDir.Y;
            }
            if (endLen > 1e-10)
            {
                endDir = endDir / endLen;
                CEndDir.Ex.Value.Value = endDir.X;
                CEndDir.Ey.Value.Value = endDir.Y;
            }
            
            // Compute angles from direction vectors
            base._angleStart = Math.Atan2(startDir.Y, startDir.X);
            base._angleEnd = Math.Atan2(endDir.Y, endDir.X);
            
            // Ensure the sweep direction is preserved
            if (_isCounterClockwise)
            {
                // CCW: angleEnd should be greater than angleStart
                while (base._angleEnd < base._angleStart)
                    base._angleEnd += 2.0 * Math.PI;
            }
            else
            {
                // CW: angleEnd should be less than angleStart
                while (base._angleEnd > base._angleStart)
                    base._angleEnd -= 2.0 * Math.PI;
            }
        }

        /// <summary>
        /// Evaluates a point on the arc at the given uniform parameter.
        /// uniform=0 is the start point, uniform=1 is the end point.
        /// Uses direction vector interpolation for robust constraint solving.
        /// </summary>
        public CVec2D Evaluate(double uniform)
        {
            if (Math.Abs(uniform) < 1e-9)
            {
                // Start point: Center + Radius * StartDir
                var x = CCenter.Ex + CRadius * CStartDir.Ex;
                var y = CCenter.Ey + CRadius * CStartDir.Ey;
                return new CVec2D(x, y);
            }
            else if (Math.Abs(uniform - 1.0) < 1e-9)
            {
                // End point: Center + Radius * EndDir
                var x = CCenter.Ex + CRadius * CEndDir.Ex;
                var y = CCenter.Ey + CRadius * CEndDir.Ey;
                return new CVec2D(x, y);
            }
            else
            {
                // Intermediate point: compute angle by linear interpolation
                // We need to compute the angle from the direction vectors
                // This is less common, so we use a simpler approach
            var t = Expr.Constant(uniform);
                
                // Linear interpolation of direction vectors (then normalize)
                // This isn't perfect for arcs but works for intermediate sampling
                var dirX = CStartDir.Ex + t * (CEndDir.Ex - CStartDir.Ex);
                var dirY = CStartDir.Ey + t * (CEndDir.Ey - CStartDir.Ey);
                var len = Expr.Sqrt(dirX * dirX + dirY * dirY);
                
                var x = CCenter.Ex + CRadius * dirX / len;
                var y = CCenter.Ey + CRadius * dirY / len;
            return new CVec2D(x, y);
            }
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in CCenter) yield return p;
            foreach (Param p in CRadius) yield return p;
            foreach (Param p in CStartDir) yield return p;
            foreach (Param p in CEndDir) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }

        public override string ToString()
        {
            // Compute angles from direction vectors for display
            Vec2D startDir = CStartDir.Evaluate();
            Vec2D endDir = CEndDir.Evaluate();
            double startDeg = Math.Atan2(startDir.Y, startDir.X) * 180.0 / Math.PI;
            double endDeg = Math.Atan2(endDir.Y, endDir.X) * 180.0 / Math.PI;
            return $"Arc: Center={CCenter}, Radius={CRadius.Evaluate():F4}, Angles=[{startDeg:F2}° to {endDeg:F2}°]";
        }
        
        /// <summary>
        /// Returns the internal constraints that must be added to the solver for this arc.
        /// These include unit vector normalization constraints for the direction vectors.
        /// </summary>
        public IEnumerable<IBaseEquation> GetInternalConstraints()
        {
            yield return new UnitVector2d(CStartDir);
            yield return new UnitVector2d(CEndDir);
        }

        /// <summary>
        /// Freezes all parameters, preventing them from being modified by the solver.
        /// </summary>
        public void Freeze()
        {
            CCenter.Freeze();
            foreach (Param p in CRadius)
                p.Frozen = true;
            CStartDir.Freeze();
            CEndDir.Freeze();
        }

        /// <summary>
        /// Unfreezes all parameters, allowing them to be modified by the solver.
        /// </summary>
        public void Unfreeze()
        {
            CCenter.Unfreeze();
            foreach (Param p in CRadius)
                p.Frozen = false;
            CStartDir.Unfreeze();
            CEndDir.Unfreeze();
        }

        /// <summary>
        /// Gets whether this arc is currently frozen.
        /// </summary>
        public bool IsFrozen
        {
            get
            {
                if (CCenter.IsFrozen) return true;
                foreach (Param p in CRadius)
                    if (p.Frozen) return true;
                if (CStartDir.IsFrozen) return true;
                if (CEndDir.IsFrozen) return true;
                return false;
            }
        }
    }

    /// <summary>
    /// Constrains a 2D vector to have unit length: x² + y² = 1.
    /// Used internally for direction vector parameterization of arcs.
    /// </summary>
    public class UnitVector2d : IBaseEquation
    {
        public CVec2D Vector { get; private set; }

        public UnitVector2d(CVec2D vector)
        {
            Vector = vector;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // x² + y² - 1 = 0
            equations.Add(Vector.Ex * Vector.Ex + Vector.Ey * Vector.Ey - Expr.Constant(1.0));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Vector) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    public class PointOnPoint2d : IBaseEquation
    {
        public CVec2D Point1 { get; private set; }
        public CVec2D Point2 { get; private set; }
        /// <summary>
        /// Offset for the constraint label relative to the coincident point center.
        /// </summary>
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public PointOnPoint2d(CVec2D point1, CVec2D point2, Vec2D labelAnchorWrtCenter = default)
        {
            Point1 = point1;
            Point2 = point2;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            equations.Add(Point1.Ex - Point2.Ex);
            equations.Add(Point1.Ey - Point2.Ey);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Point2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Distance constraint between two 2D points.
    /// </summary>
    public class DistanceBetweenPoints2d : IBaseEquation
    {
        public CVec2D Point1 { get; private set; }
        public CVec2D Point2 { get; private set; }
        public Expr Distance { get; private set; }
        /// <summary>
        /// Offset for the dimension label. X = along line direction, Y = perpendicular offset.
        /// </summary>
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public DistanceBetweenPoints2d(CVec2D point1, CVec2D point2, double distance, Vec2D labelAnchorWrtCenter = default)
        {
            Point1 = point1;
            Point2 = point2;
            Distance = Expr.Constant(distance);
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Distance constraint: |P2 - P1|² = distance²
            var dx = Point2.Ex - Point1.Ex;
            var dy = Point2.Ey - Point1.Ey;
            var distanceSquared = dx * dx + dy * dy;
            equations.Add(distanceSquared - Distance * Distance);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Point2) yield return p;
            foreach (Param p in Distance) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains the horizontal distance (X difference) between two points.
    /// </summary>
    public class HorizontalDistanceBetweenPoints2d : IBaseEquation
    {
        public CVec2D Point1 { get; private set; }
        public CVec2D Point2 { get; private set; }
        public double Distance { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public HorizontalDistanceBetweenPoints2d(CVec2D point1, CVec2D point2, double distance, Vec2D labelAnchorWrtCenter = default)
        {
            Point1 = point1;
            Point2 = point2;
            Distance = distance;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Horizontal distance constraint: |X2 - X1| = distance
            // Using (X2 - X1)² = distance² to avoid sign issues
            var dx = Point2.Ex - Point1.Ex;
            equations.Add(dx * dx - Expr.Constant(Distance * Distance));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Point2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains the vertical distance (Y difference) between two points.
    /// </summary>
    public class VerticalDistanceBetweenPoints2d : IBaseEquation
    {
        public CVec2D Point1 { get; private set; }
        public CVec2D Point2 { get; private set; }
        public double Distance { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public VerticalDistanceBetweenPoints2d(CVec2D point1, CVec2D point2, double distance, Vec2D labelAnchorWrtCenter = default)
        {
            Point1 = point1;
            Point2 = point2;
            Distance = distance;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Vertical distance constraint: |Y2 - Y1| = distance
            // Using (Y2 - Y1)² = distance² to avoid sign issues
            var dy = Point2.Ey - Point1.Ey;
            equations.Add(dy * dy - Expr.Constant(Distance * Distance));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Point2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }
    public abstract class LineConstraint2d : IBaseEquation
    {       
        protected CLine2D _line; public CLine2D Line { get { return _line; } set { _line = value; } }
        /// <summary>
        /// Offset for the constraint label relative to the line center.
        /// X = along line direction, Y = perpendicular offset.
        /// </summary>
        public Vec2D LabelAnchorWrtCenter { get; set; }

        protected LineConstraint2d(CLine2D line, Vec2D labelAnchorWrtCenter = default) 
        { 
            Line = line; 
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Line) yield return p;
        }
        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }

        public abstract void GenerateEquations(List<Expr> equations, double scaling);
    }

    /// <summary>
    /// Constrains a point to lie on a line (extended infinitely).
    /// </summary>
    public class PointOnLineConstraint2d : IBaseEquation
    {
        public CVec2D Point { get; private set; }
        public CLine2D Line { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public PointOnLineConstraint2d(CVec2D point, CLine2D line, Vec2D labelAnchorWrtCenter = default)
        {
            Point = point;
            Line = line;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Point on line: cross product of (Point - LineStart) and (LineEnd - LineStart) = 0
            // (Px - Sx)(Ey - Sy) - (Py - Sy)(Ex - Sx) = 0
            var dx = Line.CEnd.Ex - Line.CStart.Ex;
            var dy = Line.CEnd.Ey - Line.CStart.Ey;
            var px = Point.Ex - Line.CStart.Ex;
            var py = Point.Ey - Line.CStart.Ey;
            equations.Add(px * dy - py * dx);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in Line) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }


    ///// <summary>
    ///// Constrains a line to have a specific length.
    ///// </summary>
    //public class LineLengthConstraint2d : LineConstraint2d
    //{
    //    private double _length;
    //    public double Length { get { return _length; } }

    //    public LineLengthConstraint2d(CLine2D line, double length, Vec2D labelAnchorWrtCenter = default) 
    //        : base(line, labelAnchorWrtCenter) 
    //    {
    //        _length = length;
    //    }

    //    public override void GenerateEquations(List<Expr> equations, double scaling)
    //    {
    //        // Length constraint: |End - Start|² = length²
    //        var dx = Line.CEnd.Ex - Line.CStart.Ex;
    //        var dy = Line.CEnd.Ey - Line.CStart.Ey;
    //        var distanceSquared = dx * dx + dy * dy;
    //        equations.Add(distanceSquared - Expr.Constant(_length * _length));
    //    }
    //}


    public class Horizontal2d : LineConstraint2d
    {
        public Horizontal2d(Line2D line, Vec2D labelAnchorWrtCenter = default) : base(line as CLine2D, labelAnchorWrtCenter) { }

        public override void GenerateEquations(List<Expr> equations, double scaling)
        {
            equations.Add(Line.CEnd.Ey - Line.CStart.Ey);
        }
    }


    public class Vertical2d : LineConstraint2d
    {
        public Vertical2d(Line2D line, Vec2D labelAnchorWrtCenter = default) : base(line as CLine2D, labelAnchorWrtCenter) { }

        public override void GenerateEquations(List<Expr> equations, double scaling)
        {
            equations.Add(Line.CEnd.Ex - Line.CStart.Ex);
        }
    }

    /// <summary>
    /// Constrains two lines to be perpendicular.
    /// </summary>
    public class Perpendicular2d : IBaseEquation
    {
        public CLine2D Line1 { get; private set; }
        public CLine2D Line2 { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public Perpendicular2d(CLine2D line1, CLine2D line2, Vec2D labelAnchorWrtCenter = default)
        {
            Line1 = line1;
            Line2 = line2;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Perpendicular: dot product of direction vectors = 0
            // (E1x - S1x)(E2x - S2x) + (E1y - S1y)(E2y - S2y) = 0
            var dx1 = Line1.CEnd.Ex - Line1.CStart.Ex;
            var dy1 = Line1.CEnd.Ey - Line1.CStart.Ey;
            var dx2 = Line2.CEnd.Ex - Line2.CStart.Ex;
            var dy2 = Line2.CEnd.Ey - Line2.CStart.Ey;
            equations.Add(dx1 * dx2 + dy1 * dy2);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Line1) yield return p;
            foreach (Param p in Line2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains two lines to be parallel.
    /// </summary>
    public class Parallel2d : IBaseEquation
    {
        public CLine2D Line1 { get; private set; }
        public CLine2D Line2 { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public Parallel2d(CLine2D line1, CLine2D line2, Vec2D labelAnchorWrtCenter = default)
        {
            Line1 = line1;
            Line2 = line2;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Parallel: cross product of direction vectors = 0
            // (E1x - S1x)(E2y - S2y) - (E1y - S1y)(E2x - S2x) = 0
            var dx1 = Line1.CEnd.Ex - Line1.CStart.Ex;
            var dy1 = Line1.CEnd.Ey - Line1.CStart.Ey;
            var dx2 = Line2.CEnd.Ex - Line2.CStart.Ex;
            var dy2 = Line2.CEnd.Ey - Line2.CStart.Ey;
            equations.Add(dx1 * dy2 - dy1 * dx2);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Line1) yield return p;
            foreach (Param p in Line2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains two lines to have equal length.
    /// </summary>
    public class EqualLength2d : IBaseEquation
    {
        public CLine2D Line1 { get; private set; }
        public CLine2D Line2 { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public EqualLength2d(CLine2D line1, CLine2D line2, Vec2D labelAnchorWrtCenter = default)
        {
            Line1 = line1;
            Line2 = line2;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Equal length: |Line1|² = |Line2|²
            var dx1 = Line1.CEnd.Ex - Line1.CStart.Ex;
            var dy1 = Line1.CEnd.Ey - Line1.CStart.Ey;
            var dx2 = Line2.CEnd.Ex - Line2.CStart.Ex;
            var dy2 = Line2.CEnd.Ey - Line2.CStart.Ey;
            equations.Add((dx1 * dx1 + dy1 * dy1) - (dx2 * dx2 + dy2 * dy2));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Line1) yield return p;
            foreach (Param p in Line2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains a point to be at the midpoint of a line.
    /// </summary>
    public class Midpoint2d : IBaseEquation
    {
        public CVec2D Point { get; private set; }
        public CLine2D Line { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public Midpoint2d(CVec2D point, CLine2D line, Vec2D labelAnchorWrtCenter = default)
        {
            Point = point;
            Line = line;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Midpoint: P = (Start + End) / 2
            // 2*Px = Sx + Ex, 2*Py = Sy + Ey
            equations.Add(Expr.Constant(2.0) * Point.Ex - Line.CStart.Ex - Line.CEnd.Ex);
            equations.Add(Expr.Constant(2.0) * Point.Ey - Line.CStart.Ey - Line.CEnd.Ey);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in Line) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains a point to lie on a circle.
    /// </summary>
    public class PointOnCircle2d : IBaseEquation
    {
        public CVec2D Point { get; private set; }
        public CCircle2D Circle { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public PointOnCircle2d(CVec2D point, CCircle2D circle, Vec2D labelAnchorWrtCenter = default)
        {
            Point = point;
            Circle = circle;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Point on circle: |Point - Center|² = Radius²
            var dx = Point.Ex - Circle.CCenter.Ex;
            var dy = Point.Ey - Circle.CCenter.Ey;
            equations.Add(dx * dx + dy * dy - Circle.CRadius * Circle.CRadius);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in Circle) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains a line to be tangent to a circular shape (circle or arc).
    /// </summary>
    public class TangentLineCircular2d : IBaseEquation
    {
        public CLine2D Line { get; private set; }
        public ICircular2D Circular { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public TangentLineCircular2d(CLine2D line, ICircular2D circular, Vec2D labelAnchorWrtCenter = default)
        {
            Line = line;
            Circular = circular;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Tangent: distance from circular center to line = radius
            // Using: |cross(P-S, E-S)| / |E-S| = r
            // Normalized: cross² / lengthSq = r² (length² units)
            var dx = Line.CEnd.Ex - Line.CStart.Ex;
            var dy = Line.CEnd.Ey - Line.CStart.Ey;
            var px = Circular.CCenter.Ex - Line.CStart.Ex;
            var py = Circular.CCenter.Ey - Line.CStart.Ey;
            var cross = px * dy - py * dx;
            var lengthSquared = dx * dx + dy * dy;
            equations.Add(cross * cross / lengthSquared - Circular.CRadius * Circular.CRadius);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Line) yield return p;
            foreach (Param p in Circular) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains two circular shapes (circles or arcs) to be concentric (same center).
    /// </summary>
    public class Concentric2d : IBaseEquation
    {
        public ICircular2D Circular1 { get; private set; }
        public ICircular2D Circular2 { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public Concentric2d(ICircular2D circular1, ICircular2D circular2, Vec2D labelAnchorWrtCenter = default)
        {
            Circular1 = circular1;
            Circular2 = circular2;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            equations.Add(Circular1.CCenter.Ex - Circular2.CCenter.Ex);
            equations.Add(Circular1.CCenter.Ey - Circular2.CCenter.Ey);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Circular1) yield return p;
            foreach (Param p in Circular2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains two circular shapes (circles or arcs) to have equal radius.
    /// </summary>
    public class EqualRadius2d : IBaseEquation
    {
        public ICircular2D Circular1 { get; private set; }
        public ICircular2D Circular2 { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public EqualRadius2d(ICircular2D circular1, ICircular2D circular2, Vec2D labelAnchorWrtCenter = default)
        {
            Circular1 = circular1;
            Circular2 = circular2;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            equations.Add(Circular1.CRadius - Circular2.CRadius);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Circular1) yield return p;
            foreach (Param p in Circular2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains a circular shape (circle or arc) to have a specific radius.
    /// </summary>
    public class RadiusConstraint2d : IBaseEquation
    {
        public ICircular2D Circular { get; private set; }
        public double Radius { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public RadiusConstraint2d(ICircular2D circular, double radius, Vec2D labelAnchorWrtCenter = default)
        {
            Circular = circular;
            Radius = radius;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            equations.Add(Circular.CRadius - Expr.Constant(Radius));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Circular) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains two points to be symmetric about a line.
    /// </summary>
    public class SymmetricAboutLine2d : IBaseEquation
    {
        public CVec2D Point1 { get; private set; }
        public CVec2D Point2 { get; private set; }
        public CLine2D SymmetryLine { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public SymmetricAboutLine2d(CVec2D point1, CVec2D point2, CLine2D symmetryLine, Vec2D labelAnchorWrtCenter = default)
        {
            Point1 = point1;
            Point2 = point2;
            SymmetryLine = symmetryLine;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Midpoint of P1-P2 lies on symmetry line
            var mx = (Point1.Ex + Point2.Ex) * Expr.Constant(0.5);
            var my = (Point1.Ey + Point2.Ey) * Expr.Constant(0.5);
            
            var ldx = SymmetryLine.CEnd.Ex - SymmetryLine.CStart.Ex;
            var ldy = SymmetryLine.CEnd.Ey - SymmetryLine.CStart.Ey;
            var pmx = mx - SymmetryLine.CStart.Ex;
            var pmy = my - SymmetryLine.CStart.Ey;
            
            // Midpoint on line: cross product = 0
            equations.Add(pmx * ldy - pmy * ldx);
            
            // P1-P2 perpendicular to symmetry line: dot product = 0
            var pdx = Point2.Ex - Point1.Ex;
            var pdy = Point2.Ey - Point1.Ey;
            equations.Add(pdx * ldx + pdy * ldy);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Point2) yield return p;
            foreach (Param p in SymmetryLine) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains the angle between two lines.
    /// </summary>
    public class AngleBetweenLines2d : IBaseEquation
    {
        public CLine2D Line1 { get; private set; }
        public CLine2D Line2 { get; private set; }
        public double AngleRadians { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public AngleBetweenLines2d(CLine2D line1, CLine2D line2, double angleRadians, Vec2D labelAnchorWrtCenter = default)
        {
            Line1 = line1;
            Line2 = line2;
            AngleRadians = angleRadians;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // cos(angle) = (d1 · d2) / (|d1| * |d2|)
            // Normalized form: dot² / (len1Sq * len2Sq) - cos²(angle) = 0
            // This is unitless (around magnitude 1), multiply by scaling² for length² units
            var dx1 = Line1.CEnd.Ex - Line1.CStart.Ex;
            var dy1 = Line1.CEnd.Ey - Line1.CStart.Ey;
            var dx2 = Line2.CEnd.Ex - Line2.CStart.Ex;
            var dy2 = Line2.CEnd.Ey - Line2.CStart.Ey;
            
            var dot = dx1 * dx2 + dy1 * dy2;
            var len1Sq = dx1 * dx1 + dy1 * dy1;
            var len2Sq = dx2 * dx2 + dy2 * dy2;
            
            double cosAngle = Math.Cos(AngleRadians);
            // Normalized to unitless, then scaled by scaling² to get length² units
            // This makes it comparable to distance constraints
            equations.Add((dot * dot / (len1Sq * len2Sq) - Expr.Constant(cosAngle * cosAngle)) * Expr.Constant(scaling * scaling));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Line1) yield return p;
            foreach (Param p in Line2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains two lines to be colinear (lie on the same infinite line).
    /// </summary>
    public class Colinear2d : IBaseEquation
    {
        public CLine2D Line1 { get; private set; }
        public CLine2D Line2 { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public Colinear2d(CLine2D line1, CLine2D line2, Vec2D labelAnchorWrtCenter = default)
        {
            Line1 = line1;
            Line2 = line2;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Two lines are colinear if:
            // 1. They are parallel (cross product of directions = 0)
            // 2. Start of Line2 lies on Line1
            
            var dx1 = Line1.CEnd.Ex - Line1.CStart.Ex;
            var dy1 = Line1.CEnd.Ey - Line1.CStart.Ey;
            var dx2 = Line2.CEnd.Ex - Line2.CStart.Ex;
            var dy2 = Line2.CEnd.Ey - Line2.CStart.Ey;
            
            // Parallel: dx1 * dy2 - dy1 * dx2 = 0
            equations.Add(dx1 * dy2 - dy1 * dx2);
            
            // Line2.Start on Line1: (Line2.Start - Line1.Start) × (Line1.End - Line1.Start) = 0
            var px = Line2.CStart.Ex - Line1.CStart.Ex;
            var py = Line2.CStart.Ey - Line1.CStart.Ey;
            equations.Add(px * dy1 - py * dx1);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Line1) yield return p;
            foreach (Param p in Line2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains two circular entities (circles or arcs) to be tangent to each other.
    /// </summary>
    public class TangentCircularCircular2d : IBaseEquation
    {
        public ICircular2D Circular1 { get; private set; }
        public ICircular2D Circular2 { get; private set; }
        public CircleTangentType TangentType { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public TangentCircularCircular2d(ICircular2D circular1, ICircular2D circular2, CircleTangentType tangentType = CircleTangentType.Auto, Vec2D labelAnchorWrtCenter = default)
        {
            Circular1 = circular1;
            Circular2 = circular2;
            TangentType = tangentType;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // External tangent: |C1 - C2| = R1 + R2
            // Internal tangent: |C1 - C2| = |R1 - R2|
            // Auto: (|C1 - C2| - R1 - R2) * (|C1 - C2| - |R1 - R2|) = 0
            //       This allows either configuration to satisfy the constraint
            
            var dx = Circular1.CCenter.Ex - Circular2.CCenter.Ex;
            var dy = Circular1.CCenter.Ey - Circular2.CCenter.Ey;
            var distSq = dx * dx + dy * dy;
            
            switch (TangentType)
            {
                case CircleTangentType.Internal:
                    {
                        // Internal: distance = |R1 - R2|
                        var radiusDiff = Circular1.CRadius - Circular2.CRadius;
                        equations.Add(distSq - radiusDiff * radiusDiff);
                    }
                    break;

                case CircleTangentType.External:
                    {
                        // External: distance = R1 + R2
                        var radiusSum = Circular1.CRadius + Circular2.CRadius;
                        equations.Add(distSq - radiusSum * radiusSum);
                    }
                    break;

                case CircleTangentType.Auto:
                default:
                    {
                        // Auto: either internal OR external is valid
                        // (distSq - (R1+R2)²) * (distSq - (R1-R2)²) = 0
                        // This is satisfied when distSq equals either (R1+R2)² or (R1-R2)²
                        var radiusSum = Circular1.CRadius + Circular2.CRadius;
                        var radiusDiff = Circular1.CRadius - Circular2.CRadius;
                        var externalError = distSq - radiusSum * radiusSum;
                        var internalError = distSq - radiusDiff * radiusDiff;
                        equations.Add(externalError * internalError);
                    }
                    break;
            }
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Circular1) yield return p;
            foreach (Param p in Circular2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains a point to lie on an arc.
    /// </summary>
    public class PointOnArc2d : IBaseEquation
    {
        public CVec2D Point { get; private set; }
        public CArc2D Arc { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public PointOnArc2d(CVec2D point, CArc2D arc, Vec2D labelAnchorWrtCenter = default)
        {
            Point = point;
            Arc = arc;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Point on arc: distance from point to center = radius
            // (Point - Center)² = Radius²
            var dx = Point.Ex - Arc.CCenter.Ex;
            var dy = Point.Ey - Arc.CCenter.Ey;
            equations.Add(dx * dx + dy * dy - Arc.CRadius * Arc.CRadius);
            
            // Note: This constrains the point to the circle, not strictly the arc segment.
            // For strict arc segment containment, additional angle constraints would be needed,
            // but those are typically not used in CAD constraint systems as they create
            // discontinuities that confuse the solver.
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in Arc) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Constrains the perpendicular distance from a point to a line.
    /// </summary>
    public class DistancePointLine2d : IBaseEquation
    {
        public CVec2D Point { get; private set; }
        public CLine2D Line { get; private set; }
        public double Distance { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public DistancePointLine2d(CVec2D point, CLine2D line, double distance, Vec2D labelAnchorWrtCenter = default)
        {
            Point = point;
            Line = line;
            Distance = distance;
            LabelAnchorWrtCenter = labelAnchorWrtCenter;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Perpendicular distance from point to line:
            // distance = |cross(P - LineStart, LineEnd - LineStart)| / |LineEnd - LineStart|
            // Rearranging: cross² / lineLength² = distance²
            // This keeps the equation in length² units for better numerical behavior
            
            // Line direction vector
            var ldx = Line.CEnd.Ex - Line.CStart.Ex;
            var ldy = Line.CEnd.Ey - Line.CStart.Ey;
            
            // Vector from line start to point
            var px = Point.Ex - Line.CStart.Ex;
            var py = Point.Ey - Line.CStart.Ey;
            
            // Cross product (2D cross = scalar)
            var cross = px * ldy - py * ldx;
            
            // Line length squared
            var lineLengthSq = ldx * ldx + ldy * ldy;
            
            // Constraint: cross² / lineLengthSq = distance² (length² units)
            equations.Add(cross * cross / lineLengthSq - Expr.Constant(Distance * Distance));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in Line) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }
}
