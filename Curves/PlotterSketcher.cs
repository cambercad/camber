using Curves.Base;
using GeoCore;
using GeoMeta;

namespace Curves
{
    public enum SketchTessellationFlags
    {
        None = 0,
        SkipValidation = 1,
        AllowOpenContour = 2,
        ExcludeHelperGeometry=4
    }

    [APIDescription(@"PlotterSketcherCoordSys: 2D sketch placed in a 3D world CoordinateSystem (Origin + X/Y/Z axes; sketch lives in XY of the CS).
Inherits all PlotterSketcher methods (AddLine, AddArc, AddCircle, AddRectangle, AddRectangleFromCorners, AddNaca4DigitAirfoil, Append*, ...). Returned by GeoAPI.GetPlotterSketcher(...) - prefer that factory over direct construction so the sketch is registered for visualization.
Property: CoordinateSystem (CoordinateSystem, read-only) - sketch placement frame in world space; sketch (u,v) -> world via Origin + u*X + v*Y.")]
    public class PlotterSketcherCoordSys : PlotterSketcher
    {
        private CoordinateSystem _coordinateSystem;
        public CoordinateSystem CoordinateSystem { get { return _coordinateSystem; } }


        protected Func<string, Vec3D> _globalPointQuery;

        public PlotterSketcherCoordSys(string name, CoordinateSystem cs, Vec2D startPoint, Vec2D startNormal = default, BaseCurveFactory curveFactory = null)
            : base(name, startPoint, startNormal, curveFactory)
        {
            _coordinateSystem = cs;
        }

        public PlotterSketcherCoordSys(string name, CoordinateSystem cs, BaseCurveFactory curveFactory = null)
            : base(name, curveFactory)
        {
            _coordinateSystem = cs;
        }

        [APIDescription(@"SetWorldPointQuery(query: (str) -> Vec3D)
Installs the lookup used to resolve named 3D model points (mesh anchors, edge @u, surface @u,v). GeoAPI wires this automatically.")]
        public void SetWorldPointQuery(Func<string, Vec3D> query)
        {
            _globalPointQuery = query;
        }

        [APIDescription(@"ProjectWorldPoint(world: Vec3D) -> Vec2D
Projects a world-space point onto this sketch plane and returns sketch UV. Off-plane points become the closest point on the plane.")]
        public Vec2D ProjectWorldPoint(Vec3D world)
        {
            Vec3D point = GeometricAlgorithms.ProjectPointOntoPlane(world, _coordinateSystem.Z, _coordinateSystem.Origin);
            return new Vec2D(
                Vec3DOps.Dot(point - _coordinateSystem.Origin, _coordinateSystem.X),
                Vec3DOps.Dot(point - _coordinateSystem.Origin, _coordinateSystem.Y));
        }

        [APIDescription(@"TryGetWorldPoint(name: str, out world: Vec3D) -> bool
Resolves a named 3D point via the world-point query. Returns False if no query is installed or the name is unknown.")]
        public bool TryGetWorldPoint(string name, out Vec3D world)
        {
            world = default;
            if (_globalPointQuery == null || string.IsNullOrWhiteSpace(name))
                return false;
            world = _globalPointQuery(name);
            return !double.IsNaN(world.X);
        }


        [APIDescription(@"MoveToPointAndStartNewCurveStrip(pointName: str) -> bool
Moves the plotter ""pen"" to a named 3D world point (projected onto this sketch's plane) and starts a new disconnected curve strip. Returns False if the named point cannot be resolved.")]
        public bool MoveToPointAndStartNewCurveStrip(string pointName)
        {
            if (!TryGetWorldPoint(pointName, out Vec3D point))
                return false;
            MoveToPointAndStartNewCurveStrip(ProjectWorldPoint(point));
            return true;
        }

        [APIDescription(@"MoveToPointAndStartNewCurveStrip(startPoint: Vec2D = default)
Lifts the plotter pen to startPoint (sketch coords). Contours/holes are grouped later by endpoint connectivity; this only resets turtle Append* state.")]
        public void MoveToPointAndStartNewCurveStrip(Vec2D startPoint = default)
        {
            StartNewCurveStripIfNonEmpty(startPoint);
        }

    }

    [APIDescription(@"PlotterSketcher: 2D sketch of curve strips (a sketch can hold multiple disconnected strips - e.g. outer contour + holes).
Unconstrained turtle-graphics style sketching (AppendLine, AddArc, ...). Returned by GeoAPI.GetPlotterSketcher(string|CoordinateSystem) ? not for dimensioned/parametric work.
For user sketch images or drawings with dimensions/constraints, use GeoAPI.GetConstraintSketcher ? ConstrainedSketcher instead (standard CAD workflow).
Curves are added in plotter style: ""Add*"" methods take explicit start/end coordinates (a new strip starts automatically when AddLine/AddArc/AddCircle does not continue the current strip end); ""Append*"" methods continue from the previous strip end. Each curve gets a unique auto-name (Line1, Arc2, Circle3, ...) unless you set Curve.Name yourself.

Properties:
  Name (str): sketch name.

Curve naming for queries (used by GeoAPI.Get(name) and PlotterSketcher.TryGetPointOnEdge):
  ""<curveName>@<u>"" - point at uniform parameter u in [0,1] on a curve (e.g. ""Line1@0.5"" = midpoint of Line1).
  ""<curveName>@center"" - center of a Circle2D / Arc2D.
  ""<curveName>@cvN"" - B?zier control vertex N (0-based; e.g. ""Bezier1@cv1"").

CurveFlags: None (default) | HelperGeometry (excluded from extrusion / loft / revolve via SketchTessellationFlags.ExcludeHelperGeometry; rendered as construction lines).")]
    public class PlotterSketcher
    {
        protected const double DegToRad = Math.PI / 180.0;

        /// <summary>Source of truth: every curve in add order. Strips are derived by connectivity.</summary>
        protected List<Curve2D> _curves;

        /// <summary>Turtle pen: last added curve, or null after MoveTo / SetStartPoint.</summary>
        protected Curve2D _penCurve;

        protected readonly List<SketchBitmapHelper> _bitmapHelpers = new();
        protected CurveVertex2D _startPoint = new CurveVertex2D(new Vec2D(double.NaN), new Vec2D(double.NaN), double.NaN);
        protected string _name; public string Name { get { return _name; } set { _name = value; } }

        protected BaseCurveFactory _curveFactory;
        protected double _defaultMaxDeviation;

        /// <summary>Set by GeoAPI when the sketch is registered; supplies default tessellation tolerances for OffsetStrip.</summary>
        public void SetDefaultMaxDeviation(double maxDeviation)
        {
            if (maxDeviation > 0)
                _defaultMaxDeviation = maxDeviation;
        }

        protected double ResolveDefaultMaxDeviation() =>
            _defaultMaxDeviation > 0 ? _defaultMaxDeviation : 0.01;

        protected SketchStripOffsetOptions ResolveOffsetOptions(SketchStripOffsetOptions options)
        {
            if (options.TessellationTolerance == 0 && options.CornerArcTolerance == 0)
                options = SketchStripOffsetOptions.Default;
            return SketchStripOffsetOptions.Resolve(options, ResolveDefaultMaxDeviation());
        }

        // Name counters for auto-generating curve names within this sketch
        private Dictionary<string, int> _curveNameCounters = new Dictionary<string, int>();

        public PlotterSketcher(string name, Vec2D startPoint, Vec2D startNormal = default, BaseCurveFactory curveFactory = null)
        {
            // Name should be provided by the caller (GeoAPI or user)
            _name = name ?? "Sketch";
            _curves = new List<Curve2D>();
            _startPoint = new CurveVertex2D(startPoint, startNormal, 1);

            _curveFactory = curveFactory != null ? curveFactory : BaseCurveFactory.Instance;
        }

        /// <summary>
        /// Generates a unique name for a curve within this sketch.
        /// </summary>
        /// <param name="prefix">The prefix for the name (e.g., "Line", "Arc", "Circle")</param>
        /// <returns>A unique name like "Line1", "Line2", etc.</returns>
        public string GenerateCurveName(string prefix) =>
            EntityNaming.GenerateScopedName(prefix, _curveNameCounters);

        /// <summary>
        /// Ensures the curve has a name. If the curve has no name, generates one based on its type.
        /// </summary>
        protected void EnsureCurveName(Curve2D curve)
        {
            if (string.IsNullOrEmpty(curve.Name))
            {
                string prefix = GetCurveTypePrefix(curve);
                curve.Name = GenerateCurveName(prefix);
            }
        }

        /// <summary>
        /// Gets the appropriate name prefix for a curve type.
        /// </summary>
        private static string GetCurveTypePrefix(Curve2D curve) =>
            EntityNaming.GetSketchCurveTypePrefix(curve.GetType().Name);

        /// <summary>
        /// Adds a curve to the sketch and moves the turtle pen to its end.
        /// Contours are not stored here — <see cref="GetCurves"/> groups by endpoint connectivity.
        /// </summary>
        protected void AddCurveToStrip(Curve2D curve)
        {
            EnsureCurveName(curve);
            if (_curves == null)
                _curves = new List<Curve2D>();
            _curves.Add(curve);
            _penCurve = curve;
        }

        /// <summary>
        /// Lifts the turtle pen (does not create a stored strip). Contours come from connectivity.
        /// </summary>
        protected void StartNewCurveStripIfNonEmpty(Vec2D startPoint = default)
        {
            _startPoint = new CurveVertex2D(startPoint, new Vec2D(double.NaN), double.NaN);
            _penCurve = null;
        }

        /// <summary>
        /// Lifts the pen when <paramref name="startPoint"/> is not the current pen end
        /// so the next <c>Append*</c> does not pretend the new curve continues the last one.
        /// </summary>
        protected void StartNewStripIfDisconnected(Vec2D startPoint)
        {
            if (_penCurve == null)
                return;
            double tol = SketchContourBuilder.DefaultTolerance;
            Vec2D end = _penCurve.EndPosition;
            if (Vec2DOps.DistanceSquared(end, startPoint) > tol * tol)
                StartNewCurveStripIfNonEmpty(startPoint);
        }

        protected IEnumerable<Curve2D> EnumerateCurves()
        {
            if (_curves == null)
                yield break;
            for (int i = 0; i < _curves.Count; i++)
                yield return _curves[i];
        }

        [APIDescription(@"GetAllCurves() -> List[Curve2D]
Flat add-order list of every curve in this sketch. Prefer GetCurves() for contours.")]
        public IReadOnlyList<Curve2D> GetAllCurves()
        {
            if (_curves == null)
                _curves = new List<Curve2D>();
            return _curves;
        }

        protected bool RemoveStoredCurve(Curve2D curve)
        {
            if (_curves == null || curve == null)
                return false;
            bool removed = _curves.Remove(curve);
            if (removed && ReferenceEquals(_penCurve, curve))
                _penCurve = _curves.Count > 0 ? _curves[_curves.Count - 1] : null;
            return removed;
        }

        protected Curve2D FindStoredCurveByName(string curveName)
        {
            foreach (var curve in EnumerateCurves())
            {
                if (curve.Name == curveName)
                    return curve;
            }
            return null;
        }

        /// <summary>
        /// Try to get a point on a curve edge by name.
        /// Name format: "curveName@uniform" where uniform is a value like 0.500
        /// Or "curveName@center" for circles/arcs to get the center point.
        /// Or "curveName@cvN" for a B?zier control vertex (0-based).
        /// Example: "Line1@0.500" returns the midpoint of Line1
        /// </summary>
        [APIDescription(@"TryGetPointOnEdge(name: str, out result: Vec2D) -> bool
Resolves a 2D point on a sketch curve. Name format: ""<curveName>@<uniform>"" (uniform in [0,1], e.g. ""Line1@0.5""), ""<curveName>@center"" (Circle2D / Arc2D), or ""<curveName>@cvN"" (Bezier2D control vertex). Returns False if not resolvable.")]
        public bool TryGetPointOnEdge(string name, out Vec2D result)
        {
            result = default;
            if (!EntityNaming.TryParseSketchCurveAddress(name, out var address))
                return false;

            string curveName = address.CurveName;
            bool isCenter = address.IsCenter;

            Curve2D foundCurve = FindStoredCurveByName(curveName);

            if (foundCurve == null)
                return false;

            // Handle @center for circular curves
            if (isCenter)
            {
                if (foundCurve is Circle2D circle)
                {
                    result = circle.Center;
                    return true;
                }
                if (foundCurve is Arc2D arc)
                {
                    result = arc.Center;
                    return true;
                }
                if (foundCurve is Ellipse2D ellipse)
                {
                    result = ellipse.Center;
                    return true;
                }
                return false; // @center only valid for curves with a center
            }

            if (address.IsControlVertex)
            {
                if (foundCurve is Bezier2D bezier &&
                    bezier.ControlPoints != null &&
                    address.ControlVertexIndex >= 0 &&
                    address.ControlVertexIndex < bezier.ControlPoints.Length)
                {
                    result = bezier.ControlPoints[address.ControlVertexIndex];
                    return true;
                }
                return false;
            }

            CurveVertex2D vertex = foundCurve.EvaluateVertex(address.Uniform);
            Vec2D point2D = vertex.Position;
            
            result = point2D;
            
            return true;
        }

        [APIDescription(@"SetCurves(allCurves: List[List[Curve2D]])
Replaces the sketch's curve content. Incoming strips are flattened; GetCurves() re-derives contours by endpoint connectivity.")]
        public void SetCurves(List<List<Curve2D>> allCurves)
        {
            _curves = new List<Curve2D>();
            if (allCurves != null)
            {
                for (int s = 0; s < allCurves.Count; s++)
                {
                    List<Curve2D> strip = allCurves[s];
                    if (strip == null)
                        continue;
                    for (int i = 0; i < strip.Count; i++)
                    {
                        if (strip[i] != null)
                            _curves.Add(strip[i]);
                    }
                }
            }
            _penCurve = _curves.Count > 0 ? _curves[_curves.Count - 1] : null;
        }

        /// <summary>
        /// Connected contours derived from endpoint connectivity (helpers grouped separately).
        /// </summary>
        [APIDescription(@"GetCurves() -> List[List[Curve2D]]
Connected curve strips derived from endpoint connectivity. Outer = disconnected contours (holes, islands); inner = ordered curves in a strip. Construction geometry is grouped separately so it does not weld to the profile. Offset-network loops stay separate even when they share T-junction vertices.")]
        public List<List<Curve2D>> GetCurves()
        {
            return SketchContourBuilder.Connect(_curves);
        }

        /// <summary>
        /// Display-only bitmap overlay on the sketch plane (textured quad). Not included in tessellation / solids.
        /// Provide <paramref name="width"/>, <paramref name="height"/>, or both; a missing dimension is derived from the image aspect ratio.
        /// </summary>
        [APIDescription(@"AddBitmapHelper(imagePath: str, center: Vec2D, width: float? = None, height: float? = None, opacity: float = 1) -> SketchBitmapHelper
Adds a display-only bitmap overlay centered at `center` in sketch coordinates. Provide width and/or height (sketch units); the omitted size is derived from the image aspect ratio. opacity multiplies texture alpha (0-1]. Not tessellated into solids.")]
        public virtual SketchBitmapHelper AddBitmapHelper(
            string imagePath,
            Vec2D center,
            double? width = null,
            double? height = null,
            float opacity = 1f)
        {
            var helper = new SketchBitmapHelper(imagePath, center, width, height, opacity);
            _bitmapHelpers.Add(helper);
            return helper;
        }

        [APIDescription(@"GetBitmapHelpers() -> IReadOnlyList[SketchBitmapHelper]
Returns the display-only bitmap overlays attached to this sketch.")]
        public IReadOnlyList<SketchBitmapHelper> GetBitmapHelpers() => _bitmapHelpers;

        public bool RemoveBitmapHelper(SketchBitmapHelper helper) =>
            helper != null && _bitmapHelpers.Remove(helper);

        public void ClearBitmapHelpers() => _bitmapHelpers.Clear();

        [APIDescription(@"AddText(text: str, origin: Vec2D, font: SketchFontSpec, flags: CurveFlags = None)
Adds text outlines to this sketch using SketchTextProvider.Instance. origin is the first line baseline in sketch coordinates. Each closed glyph contour is inserted as its own curve strip, so holes remain separate contours for extrusion.")]
        public void AddText(string text, Vec2D origin, SketchFontSpec font, CurveFlags flags = CurveFlags.None)
        {
            var contours = SketchTextProvider.Instance.GenerateAt(text, font, origin);
            AddContourStrips(contours, flags);
        }

        [APIDescription(@"AddText(text: str, layout: SketchTextLayoutOptions, flags: CurveFlags = None)
Adds text outlines laid out inside layout.Min/layout.Max using SketchTextProvider.Instance. Supports alignment, word wrapping, and AutoFit. Each closed glyph contour is inserted as its own sketch curve strip.")]
        public void AddText(string text, SketchTextLayoutOptions layout, CurveFlags flags = CurveFlags.None)
        {
            var result = SketchTextProvider.Instance.Layout(text, layout);
            AddContourStrips(result.Contours, flags);
        }

        protected void AddContourStrips(IReadOnlyList<IReadOnlyList<Curve2D>> contours, CurveFlags flags)
        {
            if (contours == null)
                return;

            foreach (var contour in contours)
                AddContourStrip(contour, flags);
        }

        protected void AddContourStrip(IReadOnlyList<Curve2D> contour, CurveFlags flags)
        {
            if (contour == null || contour.Count == 0)
                return;

            foreach (var curve in contour)
            {
                var copy = CopyCurveForStrip(curve);
                copy.Flags = curve.Flags | flags;
                AddCurveToStrip(copy);
            }
            StartNewCurveStripIfNonEmpty();
        }

        private void EnsureWritableCurrentStrip()
        {
            if (_curves == null)
                _curves = new List<Curve2D>();
        }

        private Curve2D CopyCurveForStrip(Curve2D curve)
        {
            if (curve is Line2D line)
                return _curveFactory.CreateLine2D(line.StartPosition, line.EndPosition, line.Flags);

            var copy = curve.GetCopy();
            copy.Flags = curve.Flags;
            return copy;
        }

        public PlotterSketcher(string name, BaseCurveFactory curveFactory = null)
        {
            _name = name;
            _curves = new List<Curve2D>();

            _curveFactory = curveFactory != null ? curveFactory : BaseCurveFactory.Instance;
        }


        private CurveVertex2D PrevCurveEndPoint()
        {
            if (_penCurve == null)
                return _startPoint;
            return _penCurve.EndVertex;
        }

        [APIDescription(@"SetStartPoint(startPoint: Vec2D, startNormal: Vec2D = default)
Sets the implicit ""pen"" start used by Append* methods (no curve is drawn). startNormal is the surface normal at that vertex (used for tangential append).")]
        public void SetStartPoint(Vec2D startPoint, Vec2D startNormal = default)
        {
            _startPoint = new CurveVertex2D(startPoint, startNormal, 1);
            _penCurve = null;
        }

        [APIDescription(@"AddLine(startPoint: Vec2D, endPoint: Vec2D, flags: CurveFlags = None) -> Line2D
Adds a 2D line segment. If startPoint is not the current strip end, a new disconnected strip is started (holes / separate contours). flags = CurveFlags.HelperGeometry to mark as construction (excluded from extrude/loft when ExcludeHelperGeometry is set).")]
        public virtual Line2D AddLine(Vec2D startPoint, Vec2D endPoint, CurveFlags flags = CurveFlags.None)
        {
            StartNewStripIfDisconnected(startPoint);
            Line2D line = _curveFactory.CreateLine2D(startPoint, endPoint, flags);
            AddCurveToStrip(line);
            return line;
        }
        public virtual Ellipse2D AddEllipse(Vec2D center, Vec2D majorAxis, double minorRadius, CurveFlags flags = CurveFlags.None)
        {
            var ellipse = _curveFactory.CreateEllipse2D(center,majorAxis,minorRadius,0,2*Math.PI,flags);
            AddCurveToStrip(ellipse);
            return ellipse;
        }

        [APIDescription(@"AddCircle(center: Vec2D, radius: float, flags: CurveFlags = None) -> Circle2D
Adds a full 2D circle. Each circle is its own closed contour (derived from start==end connectivity), so a second circle is a separate hole/profile without starting a stored strip.")]
        public virtual Circle2D AddCircle(Vec2D center, double radius, CurveFlags flags = CurveFlags.None)
        {
            Circle2D result = _curveFactory.CreateCircle2D(center, radius, flags);
            AddCurveToStrip(result);
            return result;
        }
        [APIDescription(@"AddArc(start: Vec2D, pointOnArc: Vec2D, end: Vec2D, flags: CurveFlags = None) -> Arc2D
Adds a 2D arc through three points (start, an interior point, end). Arc direction follows the order of the three points.")]
        public virtual Arc2D AddArc(Vec2D start, Vec2D pointOnArc, Vec2D end, CurveFlags flags = CurveFlags.None)
        {
            StartNewStripIfDisconnected(start);
            var result = _curveFactory.CreateArc2D(start, pointOnArc, end, flags);
            AddCurveToStrip(result);
            return result;
        }
        [APIDescription(@"AddRectangle(center: Vec2D, sizeX: float, sizeY: float, flags: CurveFlags = None)
Adds a centered axis-aligned rectangle as four Line2D segments (CCW from bottom-left). For a rectangle between two corners, use AddRectangleFromCorners instead.")]
        public void AddRectangle(Vec2D center, double sizeX, double sizeY, CurveFlags flags = CurveFlags.None)
        {
            double halfX = sizeX * 0.5;
            double halfY = sizeY * 0.5;

            Vec2D bottomLeft = new Vec2D(center.X - halfX, center.Y - halfY);
            Vec2D bottomRight = new Vec2D(center.X + halfX, center.Y - halfY);
            Vec2D topRight = new Vec2D(center.X + halfX, center.Y + halfY);
            Vec2D topLeft = new Vec2D(center.X - halfX, center.Y + halfY);

            AddCurveToStrip(_curveFactory.CreateLine2D(bottomLeft, bottomRight, flags));
            AddCurveToStrip(_curveFactory.CreateLine2D(bottomRight, topRight, flags));
            AddCurveToStrip(_curveFactory.CreateLine2D(topRight, topLeft, flags));
            AddCurveToStrip(_curveFactory.CreateLine2D(topLeft, bottomLeft, flags));
        }
        /// <summary>
        /// Adds a rectangle defined by two opposite corners. Handles swapped corner values by computing real bounds.
        /// </summary>
        [APIDescription(@"AddRectangleFromCorners(cornerA: Vec2D, cornerB: Vec2D, flags: CurveFlags = None) -> List[Line2D]
Adds an axis-aligned rectangle between two opposite corners (order does not matter). Returns the four CCW edges as Line2D, indexable by ""<sketchName>:Line1"" .. ""Line4"" or via Curve2D.Name. Not the same as AddRectangle(center, sizeX, sizeY), which is center-sized.")]
        public virtual List<Line2D> AddRectangleFromCorners(Vec2D cornerA, Vec2D cornerB, CurveFlags flags = CurveFlags.None)
        {
            // Compute real min/max in case values are swapped
            Vec2D realMin = new Vec2D(Math.Min(cornerA.X, cornerB.X), Math.Min(cornerA.Y, cornerB.Y));
            Vec2D realMax = new Vec2D(Math.Max(cornerA.X, cornerB.X), Math.Max(cornerA.Y, cornerB.Y));

            Vec2D bottomLeft = realMin;
            Vec2D bottomRight = new Vec2D(realMax.X, realMin.Y);
            Vec2D topRight = realMax;
            Vec2D topLeft = new Vec2D(realMin.X, realMax.Y);

            var lines = new List<Line2D>();
            lines.Add(AddLine(bottomLeft, bottomRight, flags));
            lines.Add(AddLine(bottomRight, topRight, flags));
            lines.Add(AddLine(topRight, topLeft, flags));
            lines.Add(AddLine(topLeft, bottomLeft, flags));
            return lines;
        }

        /// <summary>
        /// Adds a cubic Hermite spline through the given 2D points. Optional start/end tangents contribute direction only;
        /// magnitude is chosen like the default chord-based end derivatives (see <see cref="CubicHermiteSpline2D"/>).
        /// </summary>
        [APIDescription(@"AddCubicHermiteSpline(points: IReadOnlyList[Vec2D], startTangent: Vec2D? = None, endTangent: Vec2D? = None, flags: CurveFlags = None) -> CubicHermiteSpline2D
Cubic Hermite spline interpolating the given 2D points. With no end tangents, uses a natural cubic (C2 interiors, zero second derivative at ends). startTangent / endTangent override end directions only (magnitudes from the adjacent chord; interiors then Catmull-Rom). Pass None for natural auto ends.")]
        public virtual CubicHermiteSpline2D AddCubicHermiteSpline(
            IReadOnlyList<Vec2D> points,
            Vec2D? startTangent = null,
            Vec2D? endTangent = null,
            CurveFlags flags = CurveFlags.None)
        {
            var curve = new CubicHermiteSpline2D(points, startTangent, endTangent, flags);
            AddCurveToStrip(curve);
            return curve;
        }

        /// <summary>
        /// Hermite spline from the current strip end through the given points (last point is the spline end).
        /// </summary>
        [APIDescription(@"AppendCubicHermiteSpline(throughPoints: IReadOnlyList[Vec2D], startTangent: Vec2D? = None, endTangent: Vec2D? = None, flags: CurveFlags = None) -> CubicHermiteSpline2D
Hermite spline starting at the current strip end and passing through `throughPoints` (last point is the new strip end). Throws if `throughPoints` is empty.")]
        public CubicHermiteSpline2D AppendCubicHermiteSpline(
            IReadOnlyList<Vec2D> throughPoints,
            Vec2D? startTangent = null,
            Vec2D? endTangent = null,
            CurveFlags flags = CurveFlags.None)
        {
            if (throughPoints == null || throughPoints.Count < 1)
                throw new ArgumentException("At least one target point is required after the current position.", nameof(throughPoints));

            var v = PrevCurveEndPoint();
            var all = new List<Vec2D>(throughPoints.Count + 1) { v.Position };
            all.AddRange(throughPoints);
            return AddCubicHermiteSpline(all, startTangent, endTangent, flags);
        }

        /// <summary>
        /// Adds a NACA 4-digit airfoil as cubic Hermite splines: upper (LE?near-TE), a straight TE closure in the thickness direction,
        /// then lower (near-TE?LE), forming a closed loop. By default the last 1% of chord at the TE is omitted so the contour is not pinched to a knife edge.
        /// </summary>
        /// <param name="spec">Camber and thickness definition.</param>
        /// <param name="leadingEdge">Leading edge in sketch coordinates.</param>
        /// <param name="chordLength">Physical chord length.</param>
        /// <param name="chordAngleRadians">Angle from sketch +X to the chord direction (LE ? TE).</param>
        /// <param name="samplesPerSide">Cosine-spaced stations per surface; 32?48 is typical.</param>
        /// <param name="analyticEndTangents">When true, uses exact chordwise derivatives at each knot for Hermite directions (LE knots use chord defaults where the thickness law is singular).</param>
        /// <param name="trailingEdgeTrimChordFraction">Fraction of chord trimmed from the TE toward the LE; 0 uses the full analytic chord [0, 1] with no closure segment.</param>
        [APIDescription(@"AddNaca4DigitAirfoil(spec: Naca4DigitSpec, leadingEdge: Vec2D, chordLength: float, chordAngleRadians: float, samplesPerSide: int = 36, analyticEndTangents: bool = True, flags: CurveFlags = None, trailingEdgeTrimChordFraction: float = Naca4DigitAirfoil.DefaultTrailingEdgeTrimChordFraction) -> Naca4AirfoilSplines
Adds a closed NACA 4-digit airfoil contour as: upper Hermite spline (LE -> near-TE), straight TE closure (in thickness direction), lower Hermite spline (near-TE -> LE).
  spec: camber/thickness definition (use Naca4DigitSpec.FromCode(int) or .Parse(str) - or the string/int overloads below).
  leadingEdge: LE position in sketch coords; the airfoil chord runs from leadingEdge along chord direction.
  chordLength: physical chord length in sketch units.
  chordAngleRadians: angle from sketch +X to chord direction (LE -> TE), in radians.
  samplesPerSide: cosine-spaced stations per surface (32-48 typical).
  analyticEndTangents: use exact chordwise derivatives at each knot (LE knots use chord defaults due to thickness-law singularity).
  trailingEdgeTrimChordFraction: TE trim fraction (default avoids knife-edge); set to 0 for full chord [0,1] with no TE closure line.
Returns Naca4AirfoilSplines { Upper: CubicHermiteSpline2D, Lower: CubicHermiteSpline2D, TeClosure: Line2D? }.")]
        public virtual Naca4AirfoilSplines AddNaca4DigitAirfoil(
            Naca4DigitSpec spec,
            Vec2D leadingEdge,
            double chordLength,
            double chordAngleRadians,
            int samplesPerSide = 36,
            bool analyticEndTangents = true,
            CurveFlags flags = CurveFlags.None,
            double trailingEdgeTrimChordFraction = Naca4DigitAirfoil.DefaultTrailingEdgeTrimChordFraction)
        {
            Naca4DigitAirfoil.BuildHermiteSplineData(
                spec, leadingEdge, chordLength, chordAngleRadians, samplesPerSide, analyticEndTangents,
                trailingEdgeTrimChordFraction,
                out var upperPts, out var lowerPts,
                out var upperKnotDirs, out var lowerKnotDirs);

            CubicHermiteSpline2D upper;
            CubicHermiteSpline2D lower;
            if (upperKnotDirs != null && lowerKnotDirs != null)
            {
                upper = new CubicHermiteSpline2D(upperPts, upperKnotDirs, flags);
                lower = new CubicHermiteSpline2D(lowerPts, lowerKnotDirs, flags);
            }
            else
            {
                upper = new CubicHermiteSpline2D(upperPts, null, null, flags);
                lower = new CubicHermiteSpline2D(lowerPts, null, null, flags);
            }

            AddCurveToStrip(upper);
                Line2D teClosure = null;
            if (trailingEdgeTrimChordFraction > 0 && upperPts.Count > 0 && lowerPts.Count > 0)
            {
                Vec2D a = upperPts[^1];
                Vec2D b = lowerPts[0];
                double minLenSq = 1e-24 * chordLength * chordLength;
                if (minLenSq < 1e-30)
                    minLenSq = 1e-30;
                if ((a - b).LengthSquared() >= minLenSq)
                    teClosure = AddLine(a, b, flags);
            }

            AddCurveToStrip(lower);
            return new Naca4AirfoilSplines { Upper = upper, Lower = lower, TeClosure = teClosure };
        }

        /// <inheritdoc cref="AddNaca4DigitAirfoil(Naca4DigitSpec, Vec2D, double, double, int, bool, CurveFlags, double)"/>
        [APIDescription(@"AddNaca4DigitAirfoil(nacaCode: int, leadingEdge: Vec2D, chordLength: float, chordAngleRadians: float, samplesPerSide: int = 36, analyticEndTangents: bool = True, flags: CurveFlags = None, trailingEdgeTrimChordFraction: float = ...) -> Naca4AirfoilSplines
Convenience overload: nacaCode is the integer 4-digit code (e.g. 2412). Other parameters identical to the spec overload.")]
        public Naca4AirfoilSplines AddNaca4DigitAirfoil(
            int nacaCode,
            Vec2D leadingEdge,
            double chordLength,
            double chordAngleRadians,
            int samplesPerSide = 36,
            bool analyticEndTangents = true,
            CurveFlags flags = CurveFlags.None,
            double trailingEdgeTrimChordFraction = Naca4DigitAirfoil.DefaultTrailingEdgeTrimChordFraction) =>
            AddNaca4DigitAirfoil(Naca4DigitSpec.FromCode(nacaCode), leadingEdge, chordLength, chordAngleRadians, samplesPerSide, analyticEndTangents, flags, trailingEdgeTrimChordFraction);

        /// <inheritdoc cref="AddNaca4DigitAirfoil(Naca4DigitSpec, Vec2D, double, double, int, bool, CurveFlags, double)"/>
        [APIDescription(@"AddNaca4DigitAirfoil(nacaLabel: str, leadingEdge: Vec2D, chordLength: float, chordAngleRadians: float, samplesPerSide: int = 36, analyticEndTangents: bool = True, flags: CurveFlags = None, trailingEdgeTrimChordFraction: float = ...) -> Naca4AirfoilSplines
Convenience overload: nacaLabel is the 4-digit string (e.g. ""2412"", ""4412"").")]
        public Naca4AirfoilSplines AddNaca4DigitAirfoil(
            string nacaLabel,
            Vec2D leadingEdge,
            double chordLength,
            double chordAngleRadians,
            int samplesPerSide = 36,
            bool analyticEndTangents = true,
            CurveFlags flags = CurveFlags.None,
            double trailingEdgeTrimChordFraction = Naca4DigitAirfoil.DefaultTrailingEdgeTrimChordFraction) =>
            AddNaca4DigitAirfoil(Naca4DigitSpec.Parse(nacaLabel), leadingEdge, chordLength, chordAngleRadians, samplesPerSide, analyticEndTangents, flags, trailingEdgeTrimChordFraction);

        [APIDescription(@"AddInvolute(center: Vec2D, baseRadius: float, tStart: float, tEnd: float, rotation: float = 0, maxDeviation: float = -1, flags: CurveFlags = None) -> CubicHermiteSpline2D
Fits a natural cubic Hermite spline to a circle involute. tStart/tEnd are unroll angles (radians) from the tangent point on the base circle. Knot count is raised until the spline stays within maxDeviation (-1 → 0.01).")]
        public virtual CubicHermiteSpline2D AddInvolute(
            Vec2D center,
            double baseRadius,
            double tStart,
            double tEnd,
            double rotation = 0,
            double maxDeviation = -1,
            CurveFlags flags = CurveFlags.None)
        {
            var curve = InvoluteGear2D.FitSpline(center, baseRadius, tStart, tEnd, rotation, maxDeviation, flags);
            StartNewStripIfDisconnected(curve.StartPosition);
            AddCurveToStrip(curve);
            return curve;
        }

        [APIDescription(@"AddInvoluteGear(center: Vec2D, module: float, teeth: int, pressureAngleRadians: float = 0.3490658503988659, addendumFactor: float = 1, dedendumFactor: float = 1.25, maxDeviation: float = -1, flags: CurveFlags = None)
External involute spur gear (ISO 53-style 20° default). Flanks are Hermite fits of the involute; tip and root are arcs. Center on the sketch origin before a twist extrude. Do not put the bore in this sketch — a twisted extrude would helix the hole; cut a cylinder afterwards.")]
        public virtual List<Curve2D> AddInvoluteGear(
            Vec2D center,
            double module,
            int teeth,
            double pressureAngleRadians = 20.0 * Math.PI / 180.0,
            double addendumFactor = 1.0,
            double dedendumFactor = 1.25,
            double maxDeviation = -1,
            CurveFlags flags = CurveFlags.None)
        {
            List<Curve2D> curves = InvoluteGear2D.CreateGear(
                center, module, teeth, pressureAngleRadians, addendumFactor, dedendumFactor, maxDeviation);
            for (int i = 0; i < curves.Count; i++)
            {
                curves[i].Flags = flags;
                StartNewStripIfDisconnected(curves[i].StartPosition);
                AddCurveToStrip(curves[i]);
            }
            return curves;
        }

        [APIDescription(@"AddInvoluteInternalGear(center: Vec2D, module: float, teeth: int, pressureAngleRadians: float = 0.3490658503988659, addendumFactor: float = 1, dedendumFactor: float = 1.25, maxDeviation: float = -1, flags: CurveFlags = None)
Inner hole of an internal ring gear: an external involute with addendum/dedendum swapped. Extrude and subtract from a disc.")]
        public virtual List<Curve2D> AddInvoluteInternalGear(
            Vec2D center,
            double module,
            int teeth,
            double pressureAngleRadians = 20.0 * Math.PI / 180.0,
            double addendumFactor = 1.0,
            double dedendumFactor = 1.25,
            double maxDeviation = -1,
            CurveFlags flags = CurveFlags.None)
        {
            List<Curve2D> curves = InvoluteGear2D.CreateInternalGear(
                center, module, teeth, pressureAngleRadians, addendumFactor, dedendumFactor, maxDeviation);
            for (int i = 0; i < curves.Count; i++)
            {
                curves[i].Flags = flags;
                StartNewStripIfDisconnected(curves[i].StartPosition);
                AddCurveToStrip(curves[i]);
            }
            return curves;
        }

        private CurveMetaData MetaDataFromCurve(Curve2D curve)
        {
            int flags = (int)curve.Flags;

            if (curve is Line2D)
                return new CurveMetaData(CurveType.Line2D, curve, flags);
            if (curve is Circle2D)
                return new CurveMetaData(CurveType.Circle2D, curve, flags);
            if (curve is Arc2D)
                return new CurveMetaData(CurveType.Arc2D, curve, flags);
            if (curve is CubicHermiteSpline2D)
                return new CurveMetaData(CurveType.HermiteSpline2D, curve, flags);
            if (curve is Bezier2D)
                return new CurveMetaData(CurveType.Bezier2D, curve, flags);
            if (curve is Ellipse2D)
                return new CurveMetaData(CurveType.Ellipse2D, curve, flags);
            if (curve is BSpline2D)
                return new CurveMetaData(CurveType.BSpline2D, curve, flags);

            return new CurveMetaData(CurveType.Unknown, curve, flags);
        }

        public List<List<List<Vec2D>>> Tessellate(double maxDeviation, out List<List<List<Vec2D>>> allNormals, 
            out List<List<string>> names, out Dictionary<string, CurveMetaData> metaData, double curveMatchingTolerance = 1e-8,
            SketchTessellationFlags flags = SketchTessellationFlags.None)
        {
            var options = SketchStripTessellateOptions.FromSketchFlags(flags, curveMatchingTolerance);

            List<List<List<Vec2D>>> allPoints = new List<List<List<Vec2D>>>();
            allNormals = new List<List<List<Vec2D>>>();
            names = new List<List<string>>();
            metaData = new Dictionary<string, CurveMetaData>();

            List<List<Curve2D>> strips = GetCurves();
            for (int stripIndex = 0; stripIndex < strips.Count; stripIndex++)
            {
                var strip = strips[stripIndex];
                List<string> na = new List<string>();

                for (int i = 0; i < strip.Count; i++)
                {
                    var curve = strip[i];
                    if (!options.IncludeHelperGeometry && curve.IsHelperGeometry)
                        continue;
                    metaData[curve.Name] = MetaDataFromCurve(curve);
                }

                SketchStripTessellator.TessellateStrip(
                    strip,
                    maxDeviation,
                    options,
                    out var points,
                    out var normals,
                    na,
                    stripIndex);

                if (points.Count == 0)
                    continue;

                allPoints.Add(points);
                allNormals.Add(normals);
                names.Add(na);
            }

            return allPoints;
        }


        [APIDescription(@"AppendLine(endPointX: float, endPointY: float) -> Line2D
Adds a line from the current strip end to (endPointX, endPointY).")]
        public Line2D AppendLine(double endPointX, double endPointY)
        {
            return AppendLine(new Vec2D(endPointX, endPointY));
        }

        [APIDescription(@"AppendLineIncrement(deltaX: float, deltaY: float) -> Line2D
Adds a line from the current strip end by (+deltaX, +deltaY) in sketch coords.")]
        public Line2D AppendLineIncrement(double deltaX, double deltaY)
        {
            return AppendLine(PrevCurveEndPoint().Position + new Vec2D(deltaX, deltaY));
        }

        public Line2D AppendLineUntilIntersection(double dirX, double dirY, Circle2D terminationCircle)
        {
            return AppendLineUntilIntersection(new Vec2D(dirX, dirY), terminationCircle);
        }


        [APIDescription(@"AppendLineVertical(endPointY: float) -> Line2D
Vertical line from current end up/down to Y = endPointY.")]
        public Line2D AppendLineVertical(double endPointY)
        {
            return AppendLine(PrevCurveEndPoint().Position.X, endPointY);
        }
        [APIDescription(@"AppendLineHorizontal(endPointX: float) -> Line2D
Horizontal line from current end left/right to X = endPointX.")]
        public Line2D AppendLineHorizontal(double endPointX)
        {
            return AppendLine(endPointX, PrevCurveEndPoint().Position.Y);
        }


        [APIDescription(@"ApplyTangentialArcBetweenLastTwoCurves(radius: float, indexOffset: int = 0) -> Curve2D
Replaces the corner between the two most recently added curves with a tangential fillet arc of the given radius (Line/Line, Line/Arc and Arc/Line corners; Arc/Arc not supported). Trims the adjacent ends to the new arc. indexOffset shifts which two curves to use (0 = last two).")]
        public Curve2D ApplyTangentialArcBetweenLastTwoCurves(double radius, int indexOffset = 0)
        {
            int last = _curves.Count - 1 - indexOffset;
            Curve2D a = _curves[last - 1];
            Curve2D b = _curves[last];

            Curve2D result;
            if (a is Line2D && b is Line2D)
            {
                Line2D lineA = (Line2D)a;
                Line2D lineB = (Line2D)b;
                result = GetTangentialArcBetweenLineLine(lineA, lineB, radius);

                lineA.SetEnd(result.StartPosition);
                lineB.SetStart(result.EndPosition); //TODO: Check if Start/End modification leads to zero length, if yes, remove element

                EnsureCurveName(result);
                _curves.Insert(last, result);
                _penCurve = b;
                return result;
            }

            if (a is Line2D && b is Arc2D)
            {
                Line2D lineA = (Line2D)a;
                Arc2D arcB = (Arc2D)b;

                result = GetTangentialArcBetweenLineArc(lineA, arcB, radius);

                lineA.SetEnd(result.StartPosition);
                arcB.SetStart(result.EndPosition); //TODO: Check if Start/End modification leads to zero length, if yes, remove element

                EnsureCurveName(result);
                _curves.Insert(last, result);
                _penCurve = b;
                return result;
            }

            if (a is Arc2D && b is Line2D)
            {
                Arc2D arcA = (Arc2D)a;
                Line2D lineB = (Line2D)b;

                Arc2D arc = GetTangentialArcBetweenLineArc(lineB, arcA, radius);
                arc.FlipDirection();
                result = arc;

                arcA.SetEnd(result.StartPosition);
                lineB.SetStart(result.EndPosition); //TODO: Check if Start/End modification leads to zero length, if yes, remove element

                EnsureCurveName(result);
                _curves.Insert(last, result);
                _penCurve = b;
                return result;
            }

            if (a is Arc2D && b is Arc2D)
            {
                throw new NotImplementedException();
            }

            throw new Exception();
        }

        private Arc2D GetTangentialArcBetweenLineArc(Line2D lineA, Arc2D arcB, double radius, CurveFlags curveFlags = CurveFlags.None)
        {
            Vec2D offsetA = radius * lineA.DirectionNormalized.PerpendicularRight(); // Vec2D.GetPerpendicularVector(lineA.DirectionNormalized);

            Line2D a = Line2D.GetTranslated(lineA, offsetA);
            Arc2D offsetArc = Arc2D.GetArcWithDifferentRadius(arcB, arcB.Radius + radius);

            double t;
            if (GeometricAlgorithms.LineSegmentArcIntersection(a.StartPosition, a.EndPosition, offsetArc.Center, offsetArc.Radius,
                offsetArc.AngleStart, offsetArc.AngleEnd, out t))
            {
                Vec2D start = lineA.EvaluatePoint(t);
                Vec2D center = a.EvaluatePoint(t);
                Vec2D dir = center - arcB.Center; // arcB.Center - center;
                dir.Normalize();
                Vec2D end = arcB.Center + arcB.Radius * dir;

                return _curveFactory.CreateArc2D(start, end, center, true, curveFlags);
            }

            a = Line2D.GetTranslated(lineA, -offsetA);
            if (GeometricAlgorithms.LineSegmentArcIntersection(a.StartPosition, a.EndPosition, offsetArc.Center, offsetArc.Radius,
               offsetArc.AngleStart, offsetArc.AngleEnd, out t))
            {
                Vec2D start = lineA.EvaluatePoint(t);
                Vec2D center = a.EvaluatePoint(t);
                Vec2D dir = center - arcB.Center;
                dir.Normalize();
                Vec2D end = arcB.Center + arcB.Radius * dir;

                return _curveFactory.CreateArc2D(start, end, center, true, curveFlags);
            }


            a = Line2D.GetTranslated(lineA, offsetA);
            offsetArc = Arc2D.GetArcWithDifferentRadius(arcB, arcB.Radius - radius);
            if (GeometricAlgorithms.LineSegmentArcIntersection(a.StartPosition, a.EndPosition, offsetArc.Center, offsetArc.Radius,
                offsetArc.AngleStart, offsetArc.AngleEnd, out t))
            {
                Vec2D start = lineA.EvaluatePoint(t);
                Vec2D center = a.EvaluatePoint(t);
                Vec2D dir = arcB.Center - center;
                dir.Normalize();
                Vec2D end = arcB.Center + arcB.Radius * dir;

                return _curveFactory.CreateArc2D(start, end, center, true, curveFlags);
            }

            a = Line2D.GetTranslated(lineA, -offsetA);
            if (GeometricAlgorithms.LineSegmentArcIntersection(a.StartPosition, a.EndPosition, offsetArc.Center, offsetArc.Radius,
               offsetArc.AngleStart, offsetArc.AngleEnd, out t))
            {
                Vec2D start = lineA.EvaluatePoint(t);
                Vec2D center = a.EvaluatePoint(t);
                Vec2D dir = arcB.Center - center;
                dir.Normalize();
                Vec2D end = arcB.Center + arcB.Radius * dir;

                return _curveFactory.CreateArc2D(start, end, center, true, curveFlags);
            }

            throw new Exception();
        }

        private Arc2D GetTangentialArcBetweenLineLine(Line2D lineA, Line2D lineB, double radius, CurveFlags curveFlags = CurveFlags.None, double eps = 1e-8)
        {
            Vec2D offsetA = radius * lineA.DirectionNormalized.PerpendicularRight(); // Vec2D.GetPerpendicularVector(lineA.DirectionNormalized);
            Vec2D offsetB = radius * lineB.DirectionNormalized.PerpendicularRight(); // Vec2D.GetPerpendicularVector(lineB.DirectionNormalized);

            Line2D a = Line2D.GetTranslated(lineA, offsetA);
            Line2D b = Line2D.GetTranslated(lineB, offsetB);

            double s, t;
            Vec2D p;
            if (GeometricAlgorithms.IntersectSegmentSegment(a.StartPosition, a.EndPosition, b.StartPosition, b.EndPosition, out s, out t, out p, eps))
            {
                Vec2D start = lineA.EvaluatePoint(s);
                Vec2D end = lineB.EvaluatePoint(t);
                return _curveFactory.CreateArc2D(start, end, p, true, curveFlags);
            }

            a = Line2D.GetTranslated(lineA, offsetA);
            b = Line2D.GetTranslated(lineB, -offsetB);
            if (GeometricAlgorithms.IntersectSegmentSegment(a.StartPosition, a.EndPosition, b.StartPosition, b.EndPosition, out s, out t, out p, eps))
            {
                Vec2D start = lineA.EvaluatePoint(s);
                Vec2D end = lineB.EvaluatePoint(t);
                return _curveFactory.CreateArc2D(start, end, p, true, curveFlags);
            }

            a = Line2D.GetTranslated(lineA, -offsetA);
            b = Line2D.GetTranslated(lineB, offsetB);
            if (GeometricAlgorithms.IntersectSegmentSegment(a.StartPosition, a.EndPosition, b.StartPosition, b.EndPosition, out s, out t, out p, eps))
            {
                Vec2D start = lineA.EvaluatePoint(s);
                Vec2D end = lineB.EvaluatePoint(t);
                return _curveFactory.CreateArc2D(start, end, p, true, curveFlags);
            }

            a = Line2D.GetTranslated(lineA, -offsetA);
            b = Line2D.GetTranslated(lineB, -offsetB);
            if (GeometricAlgorithms.IntersectSegmentSegment(a.StartPosition, a.EndPosition, b.StartPosition, b.EndPosition, out s, out t, out p, eps))
            {
                Vec2D start = lineA.EvaluatePoint(s);
                Vec2D end = lineB.EvaluatePoint(t);
                return _curveFactory.CreateArc2D(start, end, p, true, curveFlags);
            }

            throw new Exception();
        }


        [APIDescription(@"AppendLineUntilIntersection(dir: Vec2D, terminationLine: Line2D, curveFlags: CurveFlags = None) -> Line2D
Adds a line from the current strip end in direction `dir` (auto-normalized) up to its intersection with `terminationLine`. Throws if the lines are parallel / non-intersecting.")]
        public Line2D AppendLineUntilIntersection(Vec2D dir, Line2D terminationLine, CurveFlags curveFlags = CurveFlags.None)
        {
            CurveVertex2D v = PrevCurveEndPoint();
            dir.Normalize();

            double s, t;
            if (!GeometricAlgorithms.IntersectionLineLine(v.Position, dir, terminationLine.StartPosition, terminationLine.DirectionNormalized, 1e-8, out s, out t))
                throw new Exception();

            Line2D line = _curveFactory.CreateLine2D(v.Position, v.Position + t * dir, curveFlags);
            AddCurveToStrip(line);
            return line;
        }


        [APIDescription(@"AppendLineUntilIntersection(dir: Vec2D, terminationCircle: Circle2D, curveFlags: CurveFlags = None) -> Line2D
Adds a line from the current strip end in direction `dir` up to the nearest forward intersection with `terminationCircle`. Throws if the ray does not hit the circle in the forward direction.")]
        public Line2D AppendLineUntilIntersection(Vec2D dir, Circle2D terminationCircle, CurveFlags curveFlags = CurveFlags.None)
        {
            CurveVertex2D v = PrevCurveEndPoint();
            dir.Normalize();

            double t1, t2;
            if (!GeometricAlgorithms.LineCircleIntersection(v.Position, dir, terminationCircle.Center, terminationCircle.Radius, out t1, out t2))
                throw new Exception();

            double t = t1;
            if (t < 0)
                t = t2;
            else if (t2 > 0 && t2 < t)
                t = t2;

            if (t < 0)
                throw new Exception();

            Line2D line = _curveFactory.CreateLine2D(v.Position, v.Position + t * dir, curveFlags);
            AddCurveToStrip(line);
            return line;
        }

        [APIDescription(@"AppendArcUntilIntersection(arcCircle: Circle2D, terminationLine: Line2D, curveFlags: CurveFlags = None) -> PlotterSketcher
Adds an arc on `arcCircle` from the current strip end to the intersection of `terminationLine` with `arcCircle` (closer of the two intersections to the current end). Returns the sketcher (chainable).")]
        public PlotterSketcher AppendArcUntilIntersection(Circle2D arcCircle, Line2D terminationLine, CurveFlags curveFlags = CurveFlags.None)
        {
            double l = terminationLine.Length();
            Vec2D dir = terminationLine.DirectionNormalized;
            double t1, t2;
            if (!GeometricAlgorithms.LineCircleIntersection(terminationLine.StartPosition, dir, arcCircle.Center, arcCircle.Radius, out t1, out t2))
                throw new Exception();

            Vec2D p1 = terminationLine.StartPosition + t1 * dir;
            Vec2D p2 = terminationLine.StartPosition + t2 * dir;

            Vec2D start = PrevCurveEndPoint().Position;
            Vec2D end;
            bool b1 = t1 >= 0 && t1 <= l;
            bool b2 = t2 >= 0 && t2 <= l;
            if (b1 == b2)
            {
                if ((p1 - start).LengthSquared() < (p2 - start).LengthSquared())
                    end = p1;
                else
                    end = p2;
            }
            else if (b1)
                end = p1;
            else // if (b2)
                end = p2;

            var c = _curveFactory.CreateArc2D(start, end, arcCircle.Center, true, curveFlags);
            //_prevCurve() = new Arc2D(arcCircle.Center, arcCircle.Radius, Arc2D.GetAngle(start, arcCircle.Center), Arc2D.GetAngle(end, arcCircle.Center));
            AddCurveToStrip(c);
            return this;
        }

        [APIDescription(@"AppendLine(endPoint: Vec2D, curveFlags: CurveFlags = None) -> Line2D
Adds a line from the current strip end to endPoint.")]
        public Line2D AppendLine(Vec2D endPoint, CurveFlags curveFlags = CurveFlags.None)
        {
            Line2D line = _curveFactory.CreateLine2D(PrevCurveEndPoint().Position, endPoint, curveFlags);
            AddCurveToStrip(line);
            return line;
        }

        [APIDescription(@"AppendLineTangential(lineLength: float, curveFlags: CurveFlags = None) -> Line2D
Adds a line of `lineLength` continuing the tangent direction of the previous curve at its end. Requires a previous curve in the strip with a defined tangent.")]
        public Line2D AppendLineTangential(double lineLength, CurveFlags curveFlags = CurveFlags.None)
        {
            CurveVertex2D v = PrevCurveEndPoint();
            Vec2D tangent = v.Tangent.Normalized();

            Line2D line = _curveFactory.CreateLine2D(PrevCurveEndPoint().Position, PrevCurveEndPoint().Position + lineLength * tangent, curveFlags);
            AddCurveToStrip(line);
            return line;
        }

        [APIDescription(@"AppendLineAtAngleLeft(lineLength: float, angle: float = pi/2, curveFlags: CurveFlags = None) -> Line2D
Left turn relative to strip travel (turtle on the curve): increases heading by `angle` radians from the end tangent. Not screen-left. Example: strip ends traveling -X (west) ? Left(pi/2) goes toward -Y; strip ends traveling +X (east) ? Left(pi/2) goes toward +Y. Same turn sense as AppendArcTangentialLeft.")]
        public Line2D AppendLineAtAngleLeft(double lineLength, double angle = Math.PI * 0.5, CurveFlags curveFlags = CurveFlags.None)
        {
            CurveVertex2D v = PrevCurveEndPoint();
            Vec2D dir = TravelTurnDirection(v.Tangent, angle);

            Line2D line = _curveFactory.CreateLine2D(v.Position, v.Position + lineLength * dir, curveFlags);
            AddCurveToStrip(line);
            return line;
        }

        [APIDescription(@"AppendLineAtAngleRight(lineLength: float, angle: float = pi/2) -> Line2D
Right turn relative to strip travel (clockwise by `angle`). Same turn sense as AppendArcTangentialRight.")]
        public Line2D AppendLineAtAngleRight(double lineLength, double angle = Math.PI * 0.5)
        {
            return AppendLineAtAngleLeft(lineLength, -angle);
        }

        [APIDescription(@"AppendLineAtAngleLeftDeg(lineLength: float, angleDegree: float = 90)
Same as AppendLineAtAngleLeft but angle is in degrees.")]
        public Line2D AppendLineAtAngleLeftDeg(double lineLength, double angleDegree = 90)
        {
            return AppendLineAtAngleLeft(lineLength, angleDegree * DegToRad);
        }

        [APIDescription(@"AppendLineAtAngleRightDeg(lineLength: float, angleDegree: float = 90)
Same as AppendLineAtAngleRight but angle is in degrees.")]
        public Line2D AppendLineAtAngleRightDeg(double lineLength, double angleDegree = 90)
        {
            return AppendLineAtAngleRight(lineLength, angleDegree * DegToRad);
        }


        public Line2D AppendTwoLinesAtAngleDeg(Vec2D direction, Vec2D endPoint, double angleDegree)
        {
            return AppendTwoLinesAtAngle(direction, endPoint, angleDegree * DegToRad);
        }
        [APIDescription(@"AppendTwoLinesAtAngle(direction: Vec2D, endPoint: Vec2D, angle: float, curveFlags: CurveFlags = None) -> Line2D
Adds two consecutive lines: one from the current end along `direction`, and a second from there to `endPoint` at angle `angle` (radians) relative to `direction`. Throws if the construction has no intersection. Returns the second line.")]
        public Line2D AppendTwoLinesAtAngle(Vec2D direction, Vec2D endPoint, double angle, CurveFlags curveFlags = CurveFlags.None)
        {
            CurveVertex2D v = PrevCurveEndPoint();
            Vec2D tangent = direction.Normalized();
            Vec2D rotTangent = Vec2DOps.Rotated(tangent, angle);

            double s, t;
            if (!GeometricAlgorithms.IntersectionLineLine(v.Position, tangent, endPoint, rotTangent, 1e-8, out s, out t))
                throw new Exception();

            Vec2D corner = v.Position + s * tangent;

            Line2D first = _curveFactory.CreateLine2D(v.Position, corner, curveFlags);
            Line2D second = _curveFactory.CreateLine2D(corner, endPoint, curveFlags);

            AddCurveToStrip(first);
            AddCurveToStrip(second);
            return second;
        }


        [APIDescription(@"AppendArc(arcCenter: Vec2D, endPoint: Vec2D, shorter: bool, curveFlags: CurveFlags = None) -> Arc2D
Adds an arc from the current strip end to endPoint with the given center. shorter=True picks the shorter arc, False the longer one (the two possible sweeps around the circle).")]
        public Arc2D AppendArc(Vec2D arcCenter, Vec2D endPoint, bool shorter, CurveFlags curveFlags = CurveFlags.None)
        {
            Arc2D arc = _curveFactory.CreateArc2D(PrevCurveEndPoint().Position, endPoint, arcCenter, shorter, curveFlags);
            AddCurveToStrip(arc);
            return arc;
        }

        [APIDescription(@"AppendArc(pointOnArc: Vec2D, endPoint: Vec2D, curveFlags: CurveFlags = None) -> Arc2D
Adds an arc from the current strip end through `pointOnArc` to `endPoint` (three-point arc).")]
        public Arc2D AppendArc(Vec2D pointOnArc, Vec2D endPoint, CurveFlags curveFlags = CurveFlags.None)
        {
            Arc2D arc = _curveFactory.CreateArc2D(PrevCurveEndPoint().Position, pointOnArc, endPoint, curveFlags);
            AddCurveToStrip(arc);
            return arc;
        }

        [APIDescription(@"AppendArcTangential(endPoint: Vec2D, curveFlags: CurveFlags = None) -> Curve2D
Adds an arc from the current strip end to endPoint that is tangent to the previous curve's end tangent. Falls back to a straight line if no tangent arc fits. Requires a previous curve with a defined normal at its end.")]
        public Curve2D AppendArcTangential(Vec2D endPoint, CurveFlags curveFlags = CurveFlags.None)
        {
            CurveVertex2D v = PrevCurveEndPoint();
            Vec2D normal = v.Normal.Normalized();

            Vec2D p = 0.5 * (v.Position + endPoint);
            Vec2D startToEnd = endPoint - v.Position;
            double s, t;
            Curve2D c;
            if (GeometricAlgorithms.IntersectionLineLine(v.Position, normal, p,
                new Vec2D(startToEnd.Y, -startToEnd.X), 1e-8, out s, out t))
            {
                Vec2D arcCenter = v.Position + s * normal;

                Vec2D startToCenter = arcCenter - v.Position;

                Vec2D offset = p - arcCenter;
                offset.Normalize();
                double radius = startToCenter.Length();
                offset = radius * offset;

                if (Vec2DOps.Dot(v.Tangent, offset) < 0)
                    offset = -offset;

                Vec2D pointOnArc = arcCenter + offset;

                c = _curveFactory.CreateArc2D(v.Position, pointOnArc, endPoint, curveFlags);
            }
            else
            {
                c = _curveFactory.CreateLine2D(v.Position, endPoint, curveFlags);
            }
            AddCurveToStrip(c);
            return c;
        }

        [APIDescription(@"AppendArcTangentialLeft(radius: float, angle: float = pi/2) -> Curve2D
Tangential arc of `radius` with a left turn of `angle` radians relative to strip travel (turtle on the curve). Example: strip ends traveling -X ? Left(pi/2) bulges toward -Y; strip ends traveling +X ? Left(pi/2) bulges toward +Y. Same turn sense as AppendLineAtAngleLeft. For explicit geometry use AppendArc(...).")]
        public Curve2D AppendArcTangentialLeft(double radius, double angle = Math.PI * 0.5)
        {
            angle = -angle;

            CurveVertex2D v = PrevCurveEndPoint();
            Vec2D normal = v.Normal.Normalized();

            if (angle < 0)
                normal = -normal;

            Vec2D arcCenter = v.Position + radius * normal;
            Vec2D arcEnd = arcCenter + radius * Vec2DOps.Rotated(normal, angle);
            return AppendArcTangential(arcEnd);
        }

        [APIDescription(@"AppendArcTangentialRight(radius: float, angle: float = pi/2) -> Curve2D
Tangential arc of `radius` with a right turn of `angle` radians relative to strip travel. Same turn sense as AppendLineAtAngleRight.")]
        public Curve2D AppendArcTangentialRight(double radius, double angle = Math.PI * 0.5)
        {
            return AppendArcTangentialLeft(radius, -angle);
        }

        [APIDescription(@"AppendArcTangentialLeftDeg(radius: float, angleDegree: float = 90)
Same as AppendArcTangentialLeft but angle is in degrees.")]
        public Curve2D AppendArcTangentialLeftDeg(double radius, double angleDegree = 90)
        {
            return AppendArcTangentialLeft(radius, angleDegree * DegToRad);
        }

        [APIDescription(@"AppendArcTangentialRightDeg(radius: float, angleDegree: float = 90)
Same as AppendArcTangentialRight but angle is in degrees.")]
        public Curve2D AppendArcTangentialRightDeg(double radius, double angleDegree = 90)
        {
            return AppendArcTangentialRight(radius, angleDegree * DegToRad);
        }

        /// <summary>
        /// Unit direction after turning left along strip travel: heading increases by <paramref name="angleRadians"/>.
        /// </summary>
        private static Vec2D TravelTurnDirection(Vec2D travelTangent, double angleRadians)
        {
            Vec2D t = travelTangent.Normalized();
            double heading = Math.Atan2(t.Y, t.X);
            double newHeading = heading + angleRadians;
            return new Vec2D(Math.Cos(newHeading), Math.Sin(newHeading));
        }

        private List<List<Curve2D>> GroupSourceCurvesByStrip(IReadOnlyList<Curve2D> sources)
        {
            if (sources == null || sources.Count == 0)
                throw new ArgumentException("Source curve list must not be empty.", nameof(sources));

            var list = new List<Curve2D>(sources.Count);
            for (int i = 0; i < sources.Count; i++)
            {
                Curve2D curve = sources[i];
                if (curve == null)
                    throw new ArgumentException("One or more source curves were not found in this sketch.", nameof(sources));
                list.Add(curve);
            }

            return SketchContourBuilder.ConnectOriginals(list, SketchContourBuilder.DefaultTolerance);
        }

        private List<Curve2D> RequireSingleContiguousStrip(IReadOnlyList<Curve2D> sources, double connectionTolerance)
        {
            var groups = GroupSourceCurvesByStrip(sources);
            if (groups.Count != 1)
                throw new ArgumentException("Offset requires all source curves to belong to a single connected strip.", nameof(sources));

            var ordered = groups[0];
            for (int i = 0; i < ordered.Count - 1; i++)
            {
                double gap = (ordered[i].EndPosition - ordered[i + 1].StartPosition).Length();
                if (gap > connectionTolerance)
                    throw new ArgumentException($"Selected curves are not connected (gap {gap} at index {i}).", nameof(sources));
            }

            return ordered;
        }

        [APIDescription(@"OffsetStrip(sourceCurves: List[Curve2D], offset: float, options: SketchStripOffsetOptions = default) -> OffsetSketchStrip2D
Offsets a contiguous subsequence of one connected strip by `offset` (positive = outward on closed CCW loops; for open Parallel mode, positive = left of travel). Returns a single compound dependent curve that tracks source geometry after constraint solves. Also adds sampled children split per source/side (`source@in_offset` / `source@out_offset`), outline end caps (`source@start_cap` / `source@end_cap`), and corner joins (`cap[a,b]` from the two named curves they connect).
  options.JoinType: Square / Round / Miter (default Round).
  options.OpenMode: Parallel (one-sided open offset, default) or Outline (both sides + end caps ? closed). Ignored for closed sources.
  options.EndCap: Round / Butt / Square (default Round) ? only used when OpenMode is Outline.
  options.TessellationTolerance / CornerArcTolerance: -1 uses the GeoAPI maxDeviation from part construction (same as Extrude maxDeviation=-1).")]
        public OffsetSketchStrip2D OffsetStrip(
            List<Curve2D> sourceCurves,
            double offset,
            SketchStripOffsetOptions options = default)
        {
            options = ResolveOffsetOptions(options);

            var strip = RequireSingleContiguousStrip(sourceCurves, options.ConnectionTolerance);
            var result = new OffsetSketchStrip2D(strip, offset, options);
            AddOffsetSampledCurves(result.CreateSampledCurves(), asHelperGeometry: false);
            return result;
        }

        [APIDescription(@"OffsetNetwork(sourceCurves: List[Curve2D], offset: float, options: SketchStripOffsetOptions = default) -> List[OffsetSampledCurve2D]
Offsets a connected graph of curves. Sources must not self-intersect; T-junctions onto the interior of another curve are not supported. Endpoint trees and simple polylines are fine. Returns sampled polylines split by source and side (`name@in_offset` / `name@out_offset`), end caps (`name@start_cap` / `name@end_cap`), and corner joins (`cap[a,b]`), one closed strip per offset loop.")]
        public List<OffsetSampledCurve2D> OffsetNetwork(
            List<Curve2D> sourceCurves,
            double offset,
            SketchStripOffsetOptions options = default)
        {
            options = ResolveOffsetOptions(options);
            if (options.OpenMode != SketchOffsetOpenMode.Outline)
            {
                options = new SketchStripOffsetOptions(
                    options.JoinType,
                    SketchOffsetOpenMode.Outline,
                    options.EndCap,
                    options.TessellationTolerance,
                    options.CornerArcTolerance,
                    options.ConnectionTolerance);
            }

            OffsetSketchNetwork2D network = new OffsetSketchNetwork2D(sourceCurves, offset, options);
            List<OffsetSampledCurve2D> pieces = network.CreateSampledCurves();
            AddOffsetSampledCurves(pieces, asHelperGeometry: false);
            return pieces;
        }

        void AddOffsetSampledCurves(List<OffsetSampledCurve2D> pieces, bool asHelperGeometry)
        {
            if (pieces == null || pieces.Count == 0)
                return;

            for (int i = 0; i < pieces.Count; i++)
            {
                if (asHelperGeometry)
                    pieces[i].Flags = pieces[i].Flags | CurveFlags.HelperGeometry;
                AddCurveToStrip(pieces[i]);
            }
        }

        private List<TransformedSketchCurve2D> AddTransformedStrips(
            IReadOnlyList<List<Curve2D>> sourceStrips,
            RigidTransform2D transform)
        {
            var added = new List<TransformedSketchCurve2D>();
            foreach (var strip in sourceStrips)
            {
                foreach (var source in strip)
                {
                    var wrapper = new TransformedSketchCurve2D(source, transform, source.Flags);
                    AddCurveToStrip(wrapper);
                    added.Add(wrapper);
                }
            }

            return added;
        }

        [APIDescription(@"RepeatGrid(sourceCurves: List[Curve2D], countX: int, countY: int, stepX: Vec2D, stepY: Vec2D, includeOriginal: bool = False) -> List[TransformedSketchCurve2D]
Creates a grid of dependent curve copies that track the source curves through rigid translation. Each returned TransformedSketchCurve2D stores the parent curve and transform; on ConstrainedSketcher, copies refresh after SolveConstraints(). includeOriginal=False (default) skips the (0,0) cell because sources are already in the sketch.")]
        public List<TransformedSketchCurve2D> RepeatGrid(
            List<Curve2D> sourceCurves,
            int countX,
            int countY,
            Vec2D stepX,
            Vec2D stepY,
            bool includeOriginal = false)
        {
            if (countX < 1)
                throw new ArgumentOutOfRangeException(nameof(countX), "countX must be at least 1.");
            if (countY < 1)
                throw new ArgumentOutOfRangeException(nameof(countY), "countY must be at least 1.");

            var sourceStrips = GroupSourceCurvesByStrip(sourceCurves);
            var added = new List<TransformedSketchCurve2D>();

            for (int iy = 0; iy < countY; iy++)
            {
                for (int ix = 0; ix < countX; ix++)
                {
                    if (!includeOriginal && ix == 0 && iy == 0)
                        continue;

                    Vec2D offset = ix * stepX + iy * stepY;
                    added.AddRange(AddTransformedStrips(sourceStrips, RigidTransform2D.Translate(offset)));
                }
            }

            return added;
        }

        [APIDescription(@"RepeatCircular(sourceCurves: List[Curve2D], center: Vec2D, count: int, totalAngle: float = 2*pi, includeOriginal: bool = False) -> List[TransformedSketchCurve2D]
Creates `count` rotational copies of the source curves about `center`. Each copy is a TransformedSketchCurve2D that tracks its parent after constraint solves. includeOriginal=False skips the zero-angle instance.")]
        public List<TransformedSketchCurve2D> RepeatCircular(
            List<Curve2D> sourceCurves,
            Vec2D center,
            int count,
            double totalAngle = Math.PI * 2.0,
            bool includeOriginal = false)
        {
            if (count < 1)
                throw new ArgumentOutOfRangeException(nameof(count), "count must be at least 1.");

            var sourceStrips = GroupSourceCurvesByStrip(sourceCurves);
            var added = new List<TransformedSketchCurve2D>();

            for (int i = 0; i < count; i++)
            {
                if (!includeOriginal && i == 0)
                    continue;

                double angle = i * totalAngle / count;
                added.AddRange(AddTransformedStrips(sourceStrips, RigidTransform2D.RotateAbout(center, angle)));
            }

            return added;
        }

    }
}