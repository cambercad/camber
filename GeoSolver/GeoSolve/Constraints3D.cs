using GeoCore;
using GeoSolver.Kinematics;
using System.Collections;

namespace GeoSolver
{
    public class CVec3D : IEnumerableParams
    {
        private Expr _x; public Expr Ex { get { return _x; } }
        private Expr _y; public Expr Ey { get { return _y; } }
        private Expr _z; public Expr Ez { get { return _z; } }

        public CVec3D(double x, double y, double z)
        {
            _x = Expr.Parameter(x);
            _y = Expr.Parameter(y);
            _z = Expr.Parameter(z);
        }

        public CVec3D(Vec3D v) : this(v.X, v.Y, v.Z) { }

        public CVec3D(Expr x, Expr y, Expr z)
        {
            _x = x;
            _y = y;
            _z = z;
        }

        public static CVec3D Constant(Vec3D v)
        {
            return new CVec3D(Expr.Constant(v.X), Expr.Constant(v.Y), Expr.Constant(v.Z));
        }

        public Vec3D Evaluate() { return new Vec3D(_x.Evaluate(), _y.Evaluate(), _z.Evaluate()); }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in _x) yield return p;
            foreach (Param p in _y) yield return p;
            foreach (Param p in _z) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    public class PointOnPoint3d : IBaseEquation
    {
        public CVec3D Point1 { get; private set; }
        public CVec3D Point2 { get; private set; }

        public PointOnPoint3d(CVec3D point1, CVec3D point2)
        {
            Point1 = point1;
            Point2 = point2;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            Expr invL = Expr.Constant(1.0 / (scaling > 1e-12 ? scaling : 1.0));
            equations.Add((Point1.Ex - Point2.Ex) * invL);
            equations.Add((Point1.Ey - Point2.Ey) * invL);
            equations.Add((Point1.Ez - Point2.Ez) * invL);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Point2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // 3D geometric primitives for constraints
    public class CLine3D : IUpdate, IEnumerableParams
    {
        public CVec3D CStart { get; private set; }
        public CVec3D CEnd { get; private set; }
        public CVec3D CDirection { get; private set; }

        public CLine3D(Vec3D start, Vec3D end)
        {
            CStart = new CVec3D(start);
            CEnd = new CVec3D(end);
            var dir = end - start;
            CDirection = new CVec3D(dir);
        }

        public CLine3D(Vec3D point, Vec3D direction, bool isDirection)
        {
            CStart = new CVec3D(point);
            if (isDirection)
            {
                CDirection = new CVec3D(direction);
                CEnd = new CVec3D(point.X + direction.X, point.Y + direction.Y, point.Z + direction.Z);
            }
            else
            {
                CEnd = new CVec3D(direction);
                var dir = direction - point;
                CDirection = new CVec3D(dir);
            }
        }

        public CLine3D(CVec3D point, CVec3D direction)
        {
            CStart = point;
            CDirection = direction;
            CEnd = new CVec3D(
                point.Ex + direction.Ex,
                point.Ey + direction.Ey,
                point.Ez + direction.Ez);
        }

        public void Update()
        {
            // Update direction vector from start and end points
            var start = CStart.Evaluate();
            var end = CEnd.Evaluate();
            var dir = end - start;
            CDirection = new CVec3D(dir);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in CStart) yield return p;
            foreach (Param p in CEnd) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    public class CPlane3D : IUpdate, IEnumerableParams
    {
        public CVec3D CPoint { get; private set; }
        public CVec3D CNormal { get; private set; }

        public CPlane3D(Vec3D point, Vec3D normal)
        {
            CPoint = new CVec3D(point);
            CNormal = new CVec3D(normal);
        }

        public CPlane3D(CVec3D point, CVec3D normal)
        {
            CPoint = point;
            CNormal = normal;
        }

        public void Update()
        {
            // Plane is defined by point and normal, no additional updates needed
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in CPoint) yield return p;
            foreach (Param p in CNormal) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Distance constraint between two 3D points
    public class DistanceBetweenPoints3d : IBaseEquation
    {
        public CVec3D Point1 { get; private set; }
        public CVec3D Point2 { get; private set; }
        public Expr Distance { get; private set; }
        public Vec2D LabelAnchorWrtCenter { get; set; }

        public DistanceBetweenPoints3d(CVec3D point1, CVec3D point2, double distance, Vec2D labelAnchorWrtCenter = default)
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
            var dz = Point2.Ez - Point1.Ez;
            var distanceSquared = dx * dx + dy * dy + dz * dz;
            double L = scaling > 1e-12 ? scaling : 1.0;
            Expr invL2 = Expr.Constant(1.0 / (L * L));
            equations.Add((distanceSquared - Distance * Distance) * invL2);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Point2) yield return p;
            foreach (Param p in Distance) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Parallel directions constraint
    public class ParallelDirections3d : IBaseEquation
    {
        public CVec3D Direction1 { get; private set; }
        public CVec3D Direction2 { get; private set; }

        public ParallelDirections3d(CVec3D direction1, CVec3D direction2)
        {
            Direction1 = direction1;
            Direction2 = direction2;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Two vectors are parallel if their cross product is zero
            // Cross product: (a × b) = (a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)
            var cross_x = Direction1.Ey * Direction2.Ez - Direction1.Ez * Direction2.Ey;
            var cross_y = Direction1.Ez * Direction2.Ex - Direction1.Ex * Direction2.Ez;
            var cross_z = Direction1.Ex * Direction2.Ey - Direction1.Ey * Direction2.Ex;
            AddTwoIndependentComponents(equations, cross_x, cross_y, cross_z, Direction2);
        }

        internal static void AddTwoIndependentComponents(
            List<Expr> equations, Expr x, Expr y, Expr z, CVec3D reference)
        {
            Vec3D axis = reference.Evaluate();
            double ax = Math.Abs(axis.X);
            double ay = Math.Abs(axis.Y);
            double az = Math.Abs(axis.Z);
            if (az >= ax && az >= ay)
            {
                equations.Add(x);
                equations.Add(y);
            }
            else if (ay >= ax)
            {
                equations.Add(x);
                equations.Add(z);
            }
            else
            {
                equations.Add(y);
                equations.Add(z);
            }
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Direction1) yield return p;
            foreach (Param p in Direction2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    public class DirectedParallelDirections3d : IBaseEquation
    {
        public CVec3D Direction1 { get; private set; }
        public CVec3D Direction2 { get; private set; }
        public bool Opposite { get; private set; }

        public DirectedParallelDirections3d(CVec3D direction1, CVec3D direction2, bool opposite)
        {
            Direction1 = direction1;
            Direction2 = direction2;
            Opposite = opposite;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // All three components of n1 − s n2. Dropping a Cartesian axis admits
            // spurious roots (residual along the dropped axis only). The extra row is
            // consistent for unit directions; kinematics uses Gauss–Newton.
            double sign = Opposite ? -1.0 : 1.0;
            equations.Add(Direction1.Ex - sign * Direction2.Ex);
            equations.Add(Direction1.Ey - sign * Direction2.Ey);
            equations.Add(Direction1.Ez - sign * Direction2.Ez);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Direction1) yield return p;
            foreach (Param p in Direction2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Perpendicular directions constraint
    public class PerpendicularDirections3d : IBaseEquation
    {
        public CVec3D Direction1 { get; private set; }
        public CVec3D Direction2 { get; private set; }

        public PerpendicularDirections3d(CVec3D direction1, CVec3D direction2)
        {
            Direction1 = direction1;
            Direction2 = direction2;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Two vectors are perpendicular if their dot product is zero
            var dotProduct = Direction1.Ex * Direction2.Ex + Direction1.Ey * Direction2.Ey + Direction1.Ez * Direction2.Ez;
            equations.Add(dotProduct);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Direction1) yield return p;
            foreach (Param p in Direction2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Point on plane constraint
    public class PointOnPlane3d : IBaseEquation
    {
        public CVec3D Point { get; private set; }
        public CPlane3D Plane { get; private set; }

        public PointOnPlane3d(CVec3D point, CPlane3D plane)
        {
            Point = point;
            Plane = plane;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Point lies on plane if: normal · (point - planePoint) = 0
            var diff_x = Point.Ex - Plane.CPoint.Ex;
            var diff_y = Point.Ey - Plane.CPoint.Ey;
            var diff_z = Point.Ez - Plane.CPoint.Ez;
            
            var dotProduct = Plane.CNormal.Ex * diff_x + Plane.CNormal.Ey * diff_y + Plane.CNormal.Ez * diff_z;
            Expr invL = Expr.Constant(1.0 / (scaling > 1e-12 ? scaling : 1.0));
            equations.Add(dotProduct * invL);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in Plane) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>
    /// Frictionless unilateral contact: point stays in the positive half-space of a plane
    /// (g = n · (p − a) ≥ 0) via a regularized Fischer–Burmeister complementarity residual.
    /// </summary>
    public class ContactHalfSpace3d : IBaseEquation
    {
        public const double DefaultEpsilon = 1e-10;

        public CVec3D Point { get; private set; }
        public CPlane3D Plane { get; private set; }
        public Param Lambda { get; private set; }
        public double Epsilon { get; private set; }

        public ContactHalfSpace3d(CVec3D point, CPlane3D plane, double epsilon = DefaultEpsilon)
        {
            Point = point;
            Plane = plane;
            Lambda = new Param(0.0) { ScaleKind = ParamScaleKind.ContactLambda };
            Epsilon = epsilon > 0 ? epsilon : DefaultEpsilon;
        }

        /// <summary>Signed gap g = n · (p − a). Positive means separation.</summary>
        public Expr SignedGapExpr()
        {
            var diff_x = Point.Ex - Plane.CPoint.Ex;
            var diff_y = Point.Ey - Plane.CPoint.Ey;
            var diff_z = Point.Ez - Plane.CPoint.Ez;
            return Plane.CNormal.Ex * diff_x + Plane.CNormal.Ey * diff_y + Plane.CNormal.Ez * diff_z;
        }

        public double EvaluateSignedGap()
        {
            Vec3D p = Point.Evaluate();
            Vec3D o = Plane.CPoint.Evaluate();
            Vec3D n = Plane.CNormal.Evaluate();
            return n.X * (p.X - o.X) + n.Y * (p.Y - o.Y) + n.Z * (p.Z - o.Z);
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            double L = scaling > 1e-12 ? scaling : 1.0;
            Expr gap = SignedGapExpr() / Expr.Constant(L);
            Expr lambda = Expr.Parameter(Lambda);
            // φ_ε(λ, ĝ) = λ + ĝ − sqrt(λ² + ĝ² + ε)
            Expr phi = lambda + gap - Expr.Sqrt(
                Expr.Pow2(lambda) + Expr.Pow2(gap) + Expr.Constant(Epsilon));
            equations.Add(phi);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in Plane) yield return p;
            yield return Lambda;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Point on line constraint
    public class PointOnLine3d : IBaseEquation
    {
        public CVec3D Point { get; private set; }
        public CLine3D Line { get; private set; }

        public PointOnLine3d(CVec3D point, CLine3D line)
        {
            Point = point;
            Line = line;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Point lies on line if: (point - lineStart) × lineDirection = 0
            var diff_x = Point.Ex - Line.CStart.Ex;
            var diff_y = Point.Ey - Line.CStart.Ey;
            var diff_z = Point.Ez - Line.CStart.Ez;
            
            var dir_x = Line.CEnd.Ex - Line.CStart.Ex;
            var dir_y = Line.CEnd.Ey - Line.CStart.Ey;
            var dir_z = Line.CEnd.Ez - Line.CStart.Ez;
            
            // Cross product of (point - lineStart) and lineDirection should be zero
            var cross_x = diff_y * dir_z - diff_z * dir_y;
            var cross_y = diff_z * dir_x - diff_x * dir_z;
            var cross_z = diff_x * dir_y - diff_y * dir_x;
            
            equations.Add(cross_x);
            equations.Add(cross_y);
            equations.Add(cross_z);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in Line) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    public class PointOnAxis3d : IBaseEquation
    {
        public CVec3D Point { get; private set; }
        public CVec3D AxisPoint { get; private set; }
        public CVec3D AxisDirection { get; private set; }

        public PointOnAxis3d(CVec3D point, CVec3D axisPoint, CVec3D axisDirection)
        {
            Point = point;
            AxisPoint = axisPoint;
            AxisDirection = axisDirection;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            Expr dx = Point.Ex - AxisPoint.Ex;
            Expr dy = Point.Ey - AxisPoint.Ey;
            Expr dz = Point.Ez - AxisPoint.Ez;
            equations.Add(dy * AxisDirection.Ez - dz * AxisDirection.Ey);
            equations.Add(dz * AxisDirection.Ex - dx * AxisDirection.Ez);
            equations.Add(dx * AxisDirection.Ey - dy * AxisDirection.Ex);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in AxisPoint) yield return p;
            foreach (Param p in AxisDirection) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    public class CoincidentAxes3d : IBaseEquation
    {
        public CVec3D Point1 { get; private set; }
        public CVec3D Direction1 { get; private set; }
        public CVec3D Point2 { get; private set; }
        public CVec3D Perpendicular2A { get; private set; }
        public CVec3D Perpendicular2B { get; private set; }

        public CoincidentAxes3d(
            CVec3D point1,
            CVec3D direction1,
            CVec3D point2,
            CVec3D perpendicular2A,
            CVec3D perpendicular2B)
        {
            Point1 = point1;
            Direction1 = direction1;
            Point2 = point2;
            Perpendicular2A = perpendicular2A;
            Perpendicular2B = perpendicular2B;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            CVec3D delta = new CVec3D(
                Point1.Ex - Point2.Ex,
                Point1.Ey - Point2.Ey,
                Point1.Ez - Point2.Ez);
            Expr invL = Expr.Constant(1.0 / (scaling > 1e-12 ? scaling : 1.0));
            equations.Add(Dot(delta, Perpendicular2A) * invL);
            equations.Add(Dot(delta, Perpendicular2B) * invL);
            equations.Add(Dot(Direction1, Perpendicular2A));
            equations.Add(Dot(Direction1, Perpendicular2B));
        }

        private static Expr Dot(CVec3D a, CVec3D b)
        {
            return a.Ex * b.Ex + a.Ey * b.Ey + a.Ez * b.Ez;
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Direction1) yield return p;
            foreach (Param p in Point2) yield return p;
            foreach (Param p in Perpendicular2A) yield return p;
            foreach (Param p in Perpendicular2B) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Angle between vectors constraint
    public class AngleBetweenVectors3d : IBaseEquation
    {
        public CVec3D Vector1 { get; private set; }
        public CVec3D Vector2 { get; private set; }
        public Expr Angle { get; private set; }

        public AngleBetweenVectors3d(CVec3D vector1, CVec3D vector2, double angle)
        {
            Vector1 = vector1;
            Vector2 = vector2;
            Angle = Expr.Constant(angle);
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // cos(angle) = (v1 · v2) / (|v1| * |v2|)
            // We'll use: v1 · v2 = |v1| * |v2| * cos(angle)
            var dot = Vector1.Ex * Vector2.Ex + Vector1.Ey * Vector2.Ey + Vector1.Ez * Vector2.Ez;
            var len1Sq = Vector1.Ex * Vector1.Ex + Vector1.Ey * Vector1.Ey + Vector1.Ez * Vector1.Ez;
            var len2Sq = Vector2.Ex * Vector2.Ex + Vector2.Ey * Vector2.Ey + Vector2.Ez * Vector2.Ez;
            
            // To avoid square roots, we use: (v1 · v2)² = |v1|² * |v2|² * cos²(angle)
            var cosAngle = Expr.Cos(Angle);
            equations.Add(dot * dot - len1Sq * len2Sq * cosAngle * cosAngle);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Vector1) yield return p;
            foreach (Param p in Vector2) yield return p;
            foreach (Param p in Angle) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Coplanar points constraint (four points lie in the same plane)
    public class CoplanarPoints3d : IBaseEquation
    {
        public CVec3D Point1 { get; private set; }
        public CVec3D Point2 { get; private set; }
        public CVec3D Point3 { get; private set; }
        public CVec3D Point4 { get; private set; }

        public CoplanarPoints3d(CVec3D point1, CVec3D point2, CVec3D point3, CVec3D point4)
        {
            Point1 = point1;
            Point2 = point2;
            Point3 = point3;
            Point4 = point4;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Four points are coplanar if the scalar triple product of vectors (P2-P1), (P3-P1), (P4-P1) is zero
            // Scalar triple product: a · (b × c) = det([a b c])
            var v1_x = Point2.Ex - Point1.Ex;
            var v1_y = Point2.Ey - Point1.Ey;
            var v1_z = Point2.Ez - Point1.Ez;
            
            var v2_x = Point3.Ex - Point1.Ex;
            var v2_y = Point3.Ey - Point1.Ey;
            var v2_z = Point3.Ez - Point1.Ez;
            
            var v3_x = Point4.Ex - Point1.Ex;
            var v3_y = Point4.Ey - Point1.Ey;
            var v3_z = Point4.Ez - Point1.Ez;
            
            // Calculate determinant: v1 · (v2 × v3)
            var cross_x = v2_y * v3_z - v2_z * v3_y;
            var cross_y = v2_z * v3_x - v2_x * v3_z;
            var cross_z = v2_x * v3_y - v2_y * v3_x;
            
            var scalarTripleProduct = v1_x * cross_x + v1_y * cross_y + v1_z * cross_z;
            equations.Add(scalarTripleProduct);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Point2) yield return p;
            foreach (Param p in Point3) yield return p;
            foreach (Param p in Point4) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Unit vector constraint (constrains a vector to have unit length)
    public class UnitVector3d : IBaseEquation
    {
        public CVec3D Vector { get; private set; }

        public UnitVector3d(CVec3D vector)
        {
            Vector = vector;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Unit vector constraint: |v|² = 1
            var lengthSquared = Vector.Ex * Vector.Ex + Vector.Ey * Vector.Ey + Vector.Ez * Vector.Ez;
            equations.Add(lengthSquared - Expr.Constant(1.0));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Vector) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Fixed point constraint (constrains a point to a specific location)
    public class FixedPoint3d : IBaseEquation
    {
        public CVec3D Point { get; private set; }
        public Vec3D FixedPosition { get; private set; }

        public FixedPoint3d(CVec3D point, Vec3D fixedPosition)
        {
            Point = point;
            FixedPosition = fixedPosition;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            equations.Add(Point.Ex - Expr.Constant(FixedPosition.X));
            equations.Add(Point.Ey - Expr.Constant(FixedPosition.Y));
            equations.Add(Point.Ez - Expr.Constant(FixedPosition.Z));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Symmetric points constraint (two points are symmetric about a plane)
    public class SymmetricPoints3d : IBaseEquation
    {
        public CVec3D Point1 { get; private set; }
        public CVec3D Point2 { get; private set; }
        public CPlane3D SymmetryPlane { get; private set; }

        public SymmetricPoints3d(CVec3D point1, CVec3D point2, CPlane3D symmetryPlane)
        {
            Point1 = point1;
            Point2 = point2;
            SymmetryPlane = symmetryPlane;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // For symmetric points about a plane:
            // 1. The midpoint of the two points lies on the plane
            // 2. The line connecting the points is perpendicular to the plane
            
            var midpoint_x = (Point1.Ex + Point2.Ex) * Expr.Constant(0.5);
            var midpoint_y = (Point1.Ey + Point2.Ey) * Expr.Constant(0.5);
            var midpoint_z = (Point1.Ez + Point2.Ez) * Expr.Constant(0.5);
            
            // Midpoint lies on plane: normal · (midpoint - planePoint) = 0
            var diff_x = midpoint_x - SymmetryPlane.CPoint.Ex;
            var diff_y = midpoint_y - SymmetryPlane.CPoint.Ey;
            var diff_z = midpoint_z - SymmetryPlane.CPoint.Ez;
            
            var dotProduct = SymmetryPlane.CNormal.Ex * diff_x + SymmetryPlane.CNormal.Ey * diff_y + SymmetryPlane.CNormal.Ez * diff_z;
            equations.Add(dotProduct);
            
            // Line P1-P2 is parallel to plane normal: (P2-P1) × normal = 0
            var line_x = Point2.Ex - Point1.Ex;
            var line_y = Point2.Ey - Point1.Ey;
            var line_z = Point2.Ez - Point1.Ez;
            
            var cross_x = line_y * SymmetryPlane.CNormal.Ez - line_z * SymmetryPlane.CNormal.Ey;
            var cross_y = line_z * SymmetryPlane.CNormal.Ex - line_x * SymmetryPlane.CNormal.Ez;
            var cross_z = line_x * SymmetryPlane.CNormal.Ey - line_y * SymmetryPlane.CNormal.Ex;
            
            equations.Add(cross_x);
            equations.Add(cross_y);
            equations.Add(cross_z);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point1) yield return p;
            foreach (Param p in Point2) yield return p;
            foreach (Param p in SymmetryPlane) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    /// <summary>Signed offset from <paramref name="point"/> to <paramref name="plane"/> along the plane normal.</summary>
    public class PlaneOffset3d : IBaseEquation
    {
        public CVec3D Point { get; private set; }
        public CPlane3D Plane { get; private set; }
        public Expr Offset { get; private set; }

        public PlaneOffset3d(CVec3D point, CPlane3D plane, double offset)
        {
            Point = point;
            Plane = plane;
            Offset = Expr.Constant(offset);
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            var diff_x = Point.Ex - Plane.CPoint.Ex;
            var diff_y = Point.Ey - Plane.CPoint.Ey;
            var diff_z = Point.Ez - Plane.CPoint.Ez;
            var signedDistance = Plane.CNormal.Ex * diff_x + Plane.CNormal.Ey * diff_y + Plane.CNormal.Ez * diff_z;
            equations.Add(signedDistance - Offset);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Point) yield return p;
            foreach (Param p in Plane) yield return p;
            foreach (Param p in Offset) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Lines parallel constraint (two lines are parallel)
    public class ParallelLines3d : IBaseEquation
    {
        public CLine3D Line1 { get; private set; }
        public CLine3D Line2 { get; private set; }

        public ParallelLines3d(CLine3D line1, CLine3D line2)
        {
            Line1 = line1;
            Line2 = line2;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Two lines are parallel if their direction vectors are parallel
            var dir1_x = Line1.CEnd.Ex - Line1.CStart.Ex;
            var dir1_y = Line1.CEnd.Ey - Line1.CStart.Ey;
            var dir1_z = Line1.CEnd.Ez - Line1.CStart.Ez;
            
            var dir2_x = Line2.CEnd.Ex - Line2.CStart.Ex;
            var dir2_y = Line2.CEnd.Ey - Line2.CStart.Ey;
            var dir2_z = Line2.CEnd.Ez - Line2.CStart.Ez;
            
            // Cross product of direction vectors should be zero
            var cross_x = dir1_y * dir2_z - dir1_z * dir2_y;
            var cross_y = dir1_z * dir2_x - dir1_x * dir2_z;
            var cross_z = dir1_x * dir2_y - dir1_y * dir2_x;
            
            equations.Add(cross_x);
            equations.Add(cross_y);
            equations.Add(cross_z);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Line1) yield return p;
            foreach (Param p in Line2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }

    // Lines perpendicular constraint (two lines are perpendicular)
    public class PerpendicularLines3d : IBaseEquation
    {
        public CLine3D Line1 { get; private set; }
        public CLine3D Line2 { get; private set; }

        public PerpendicularLines3d(CLine3D line1, CLine3D line2)
        {
            Line1 = line1;
            Line2 = line2;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
            // Two lines are perpendicular if their direction vectors are perpendicular
            var dir1_x = Line1.CEnd.Ex - Line1.CStart.Ex;
            var dir1_y = Line1.CEnd.Ey - Line1.CStart.Ey;
            var dir1_z = Line1.CEnd.Ez - Line1.CStart.Ez;
            
            var dir2_x = Line2.CEnd.Ex - Line2.CStart.Ex;
            var dir2_y = Line2.CEnd.Ey - Line2.CStart.Ey;
            var dir2_z = Line2.CEnd.Ez - Line2.CStart.Ez;
            
            // Dot product of direction vectors should be zero
            var dotProduct = dir1_x * dir2_x + dir1_y * dir2_y + dir1_z * dir2_z;
            equations.Add(dotProduct);
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Line1) yield return p;
            foreach (Param p in Line2) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }
}
