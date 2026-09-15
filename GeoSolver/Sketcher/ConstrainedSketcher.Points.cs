using Curves;
using GeoCore;
using GeoMeta;

namespace GeoSolver.Sketcher
{
    public partial class ConstrainedSketcher
    {
        [APIDescription(@"IsConstantPoint(point: CVec2D) -> bool
True when both coordinates are constants (projected model anchors, Origin). False for free or derived sketch handles.")]
        public static bool IsConstantPoint(CVec2D point)
        {
            if (point == null)
                return false;
            return point.Ex.Type == ExprType.Constant && point.Ey.Type == ExprType.Constant;
        }

        [APIDescription(@"ProjectWorldPointAsConstant(world: Vec3D) -> CVec2D
Projects a world point onto this sketch plane and returns a constant CVec2D. The solver cannot move it.")]
        public CVec2D ProjectWorldPointAsConstant(Vec3D world)
        {
            Vec2D uv = ProjectWorldPoint(world);
            return CVec2D.Constant(uv.X, uv.Y);
        }

        [APIDescription(@"GetConstraintPoint(name: str) -> CVec2D
Resolves any named point for use in Set*/Fix* constraints.
  Sketch handles (""Line1@0.000"", ""Circle1@center"", ""<thisSketch>:Line1@0.5"") stay live expressions.
  Origin / ""<thisSketch>:Origin"" is the constant (0, 0).
  Any other resolvable 3D model name (edge @u, surface @u,v, ""<mesh>:<anchor>"") is projected onto this plane and treated as a constant.
Throws if the name cannot be resolved.")]
        public CVec2D GetConstraintPoint(string name)
        {
            if (TryGetConstraintPoint(name, out CVec2D point))
                return point;
            throw new ArgumentException(
                "Point '" + name + "' cannot be resolved as a sketch constraint reference.");
        }

        [APIDescription(@"TryGetConstraintPoint(name: str, out point: CVec2D) -> bool
Non-throwing variant of GetConstraintPoint.")]
        public bool TryGetConstraintPoint(string name, out CVec2D point)
        {
            point = default;
            if (string.IsNullOrWhiteSpace(name))
                return false;
            string trimmed = name.Trim().Trim('"');
            if (IsSketchOriginName(trimmed))
            {
                point = Origin;
                return true;
            }

            string local = LocalSketchAddress(trimmed);
            if (local != null && TryGetCPointOnEdge(local, out point))
                return true;

            if (!TryGetWorldPoint(trimmed, out Vec3D world))
                return false;
            point = ProjectWorldPointAsConstant(world);
            return true;
        }

        bool IsSketchOriginName(string name)
        {
            if (string.Equals(name, EntityNaming.DefaultPoints.Origin, StringComparison.OrdinalIgnoreCase))
                return true;
            return string.Equals(
                name,
                EntityNaming.Qualify(Name, EntityNaming.DefaultPoints.Origin),
                StringComparison.OrdinalIgnoreCase);
        }

        string LocalSketchAddress(string name)
        {
            if (!EntityNaming.TryParseQualifiedSketchCurveAddress(name, out var qualified))
                return name;
            if (qualified.HasSketchQualifier && qualified.SketchName != Name)
                return null;
            return qualified.LocalAddress;
        }

        CVec2D PointRef(string name) => GetConstraintPoint(name);

        [APIDescription(@"FixPoint(point: str)
Pins the named point at its current (or projected) location.")]
        public void FixPoint(string point) => FixPoint(PointRef(point));

        [APIDescription(@"FixPoint(point: str, fixPos: Vec2D)
Pins the named point to fixPos.")]
        public void FixPoint(string point, Vec2D fixPos) => FixPoint(PointRef(point), fixPos);

        [APIDescription(@"SetPointOnPoint(a: str, b: str)
Coincident two named points. Either name may be a sketch handle or a projected model anchor.")]
        public void SetPointOnPoint(string a, string b) => SetPointOnPoint(PointRef(a), PointRef(b));
        public void SetPointOnPoint(CVec2D a, string b) => SetPointOnPoint(a, PointRef(b));
        public void SetPointOnPoint(string a, CVec2D b) => SetPointOnPoint(PointRef(a), b);

        [APIDescription(@"SetCoincident(point1: str, point2: str)
Same as SetPointOnPoint. Accepts any named sketch or model point.")]
        public void SetCoincident(string point1, string point2) => SetCoincident(PointRef(point1), PointRef(point2));
        public void SetCoincident(CVec2D point1, string point2) => SetCoincident(point1, PointRef(point2));
        public void SetCoincident(string point1, CVec2D point2) => SetCoincident(PointRef(point1), point2);

        public void SetDistancePointPoint(string a, string b, double length) =>
            SetDistancePointPoint(PointRef(a), PointRef(b), length);
        public void SetDistancePointPoint(CVec2D a, string b, double length) =>
            SetDistancePointPoint(a, PointRef(b), length);
        public void SetDistancePointPoint(string a, CVec2D b, double length) =>
            SetDistancePointPoint(PointRef(a), b, length);

        public void SetHorizontalDistancePointPoint(string a, string b, double distance) =>
            SetHorizontalDistancePointPoint(PointRef(a), PointRef(b), distance);
        public void SetHorizontalDistancePointPoint(CVec2D a, string b, double distance) =>
            SetHorizontalDistancePointPoint(a, PointRef(b), distance);
        public void SetHorizontalDistancePointPoint(string a, CVec2D b, double distance) =>
            SetHorizontalDistancePointPoint(PointRef(a), b, distance);

        public void SetVerticalDistancePointPoint(string a, string b, double distance) =>
            SetVerticalDistancePointPoint(PointRef(a), PointRef(b), distance);
        public void SetVerticalDistancePointPoint(CVec2D a, string b, double distance) =>
            SetVerticalDistancePointPoint(a, PointRef(b), distance);
        public void SetVerticalDistancePointPoint(string a, CVec2D b, double distance) =>
            SetVerticalDistancePointPoint(PointRef(a), b, distance);

        public void SetDistance(string point1, string point2, double distance) =>
            SetDistance(PointRef(point1), PointRef(point2), distance);
        public void SetDistance(CVec2D point1, string point2, double distance) =>
            SetDistance(point1, PointRef(point2), distance);
        public void SetDistance(string point1, CVec2D point2, double distance) =>
            SetDistance(PointRef(point1), point2, distance);

        public void SetHorizontalDistance(string point1, string point2, double distance) =>
            SetHorizontalDistance(PointRef(point1), PointRef(point2), distance);
        public void SetHorizontalDistance(CVec2D point1, string point2, double distance) =>
            SetHorizontalDistance(point1, PointRef(point2), distance);
        public void SetHorizontalDistance(string point1, CVec2D point2, double distance) =>
            SetHorizontalDistance(PointRef(point1), point2, distance);

        public void SetVerticalDistance(string point1, string point2, double distance) =>
            SetVerticalDistance(PointRef(point1), PointRef(point2), distance);
        public void SetVerticalDistance(CVec2D point1, string point2, double distance) =>
            SetVerticalDistance(point1, PointRef(point2), distance);
        public void SetVerticalDistance(string point1, CVec2D point2, double distance) =>
            SetVerticalDistance(PointRef(point1), point2, distance);

        public void SetMidpoint(string point, CLine2D line) => SetMidpoint(PointRef(point), line);

        public void SetPointOnCircle(string point, CCircle2D circle) => SetPointOnCircle(PointRef(point), circle);

        public void SetPointOnArc(string point, CArc2D arc) => SetPointOnArc(PointRef(point), arc);

        public void SetSymmetric(string point1, string point2, CLine2D symmetryLine) =>
            SetSymmetric(PointRef(point1), PointRef(point2), symmetryLine);
        public void SetSymmetric(CVec2D point1, string point2, CLine2D symmetryLine) =>
            SetSymmetric(point1, PointRef(point2), symmetryLine);
        public void SetSymmetric(string point1, CVec2D point2, CLine2D symmetryLine) =>
            SetSymmetric(PointRef(point1), point2, symmetryLine);

        public void SetPointOnLine(string point, CLine2D line) => SetPointOnLine(PointRef(point), line);

        public void SetDistancePointLine(string point, CLine2D line, double distance) =>
            SetDistancePointLine(PointRef(point), line, distance);
    }
}
