using System.Globalization;
using GeoCore;
using NURBS;

namespace Geo.Export
{
    public sealed class StepWriter
    {
        private readonly List<string> _entities = new();
        private readonly Dictionary<long, int> _vertexIds = new();
        private readonly Dictionary<long, SharedEdge> _sharedEdgeCurves = new();
        private readonly CultureInfo _ci = CultureInfo.InvariantCulture;
        private int _nextId = 1;
        private const double VertexTolerance = 1e-6;
        public double LinearUncertainty { get; set; } = 1e-3;

        public int NewId() => _nextId++;

        private string P(double v)
        {
            // STEP real values must lex as reals (0.0), not integers (0).
            if (double.IsNaN(v) || double.IsInfinity(v))
                return "0.0";
            return v.ToString("0.0################", _ci);
        }

        public int CreateCartesianPoint(Vec3D p)
        {
            int id = NewId();
            _entities.Add($"#{id}=CARTESIAN_POINT('',({P(p.X)},{P(p.Y)},{P(p.Z)}));");
            return id;
        }

        public int WriteVertex(Vec3D p)
        {
            long key = VertexKey(p);
            if (_vertexIds.TryGetValue(key, out int existing))
                return existing;

            int pid = CreateCartesianPoint(p);
            int vid = NewId();
            _entities.Add($"#{vid}=VERTEX_POINT('',#{pid});");
            _vertexIds[key] = vid;
            return vid;
        }

        private static long VertexKey(Vec3D p)
        {
            double q = 1.0 / VertexTolerance;
            long x = (long)Math.Round(p.X * q);
            long y = (long)Math.Round(p.Y * q);
            long z = (long)Math.Round(p.Z * q);
            return (x * 73856093L) ^ (y * 19349663L) ^ (z * 83492791L);
        }

        public int WriteLine3D(Vec3D p0, Vec3D p1)
        {
            int a = CreateCartesianPoint(p0);
            Vec3D d = p1 - p0;
            double len = d.Length();
            if (len < 1e-12)
                d = new Vec3D(1, 0, 0);
            else
                d = new Vec3D(d.X / len, d.Y / len, d.Z / len);
            int dir = WriteDirection(d);
            int vec = NewId();
            _entities.Add($"#{vec}=VECTOR('',#{dir},{P(len < 1e-12 ? 1.0 : len)});");
            int line = NewId();
            _entities.Add($"#{line}=LINE('',#{a},#{vec});");
            return line;
        }

        public int WriteNurbsCurve(BSplineCurve curve)
        {
            var points = NurbsEntityConverter.CurveControlPoints(curve, out var weights);
            bool rational = NurbsEntityConverter.IsRational(curve);
            var (knots, mults) = NurbsEntityConverter.CompressKnotVector(curve.Knots, curve.Degree);

            var pointIds = new List<int>();
            foreach (var pt in points)
                pointIds.Add(CreateCartesianPoint(pt));

            int bcurveId = NewId();
            _entities.Add(
                $"#{bcurveId}=B_SPLINE_CURVE_WITH_KNOTS('',{curve.Degree},({string.Join(",", pointIds.Select(i => "#" + i))}),.UNSPECIFIED.,.F.,.F.,({string.Join(",", mults)}),({string.Join(",", knots.Select(P))}),.UNSPECIFIED.);");
            return bcurveId;
        }

        public int WriteNurbsSurface(BSplineSurface surface)
        {
            // ISO 10303-42: control_points_list is LIST [u] OF LIST [v] OF cartesian_point.
            var (knotsU, multsU) = NurbsEntityConverter.CompressKnotVector(surface.KnotsU, surface.DegreeU);
            var (knotsV, multsV) = NurbsEntityConverter.CompressKnotVector(surface.KnotsV, surface.DegreeV);

            int nu = surface.NumControlPointsU;
            int nv = surface.NumControlPointsV;
            bool rational = NurbsEntityConverter.IsRational(surface);
            var rows = new List<string>(nu);
            var weightRows = new List<string>(nu);
            for (int u = 0; u < nu; u++)
            {
                var rowIds = new List<int>(nv);
                var rowW = new List<string>(nv);
                for (int v = 0; v < nv; v++)
                {
                    NurbsEntityConverter.DehomogenizeControlPoint(surface.ControlPoints[u][v], out var p, out var w);
                    rowIds.Add(CreateCartesianPoint(p));
                    rowW.Add(P(w));
                }
                rows.Add($"({string.Join(",", rowIds.Select(i => "#" + i))})");
                weightRows.Add($"({string.Join(",", rowW)})");
            }

            string cpRefs = string.Join(",", rows);
            // ISO 10303-42: u_multiplicities, v_multiplicities, u_knots, v_knots, knot_spec.
            string knotPart =
                $"({string.Join(",", multsU)}),({string.Join(",", multsV)}),({string.Join(",", knotsU.Select(P))}),({string.Join(",", knotsV.Select(P))}),.UNSPECIFIED.";
            int surfId = NewId();
            if (!rational)
            {
                _entities.Add(
                    $"#{surfId}=B_SPLINE_SURFACE_WITH_KNOTS('',{surface.DegreeU},{surface.DegreeV},({cpRefs}),.UNSPECIFIED.,.F.,.F.,.F.,{knotPart});");
                return surfId;
            }

            _entities.Add(
                $"#{surfId}=(BOUNDED_SURFACE() B_SPLINE_SURFACE({surface.DegreeU},{surface.DegreeV},({cpRefs}),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS({knotPart}) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({string.Join(",", weightRows)})) REPRESENTATION_ITEM('') SURFACE());");
            return surfId;
        }

        public int WriteEdgeCurve(int startVertexId, int endVertexId, int curveId)
        {
            int id = NewId();
            _entities.Add($"#{id}=EDGE_CURVE('',#{startVertexId},#{endVertexId},#{curveId},.T.);");
            return id;
        }

        private int? _param2dContextId;

        public int GetParametricRepresentationContext2D()
        {
            if (_param2dContextId == null)
            {
                int id = NewId();
                _entities.Add(
                    $"#{id}=( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D SPACE','') );");
                _param2dContextId = id;
            }
            return _param2dContextId.Value;
        }

        public int WriteUvCartesianPoint(double u, double v)
        {
            int id = NewId();
            _entities.Add($"#{id}=CARTESIAN_POINT('',({P(u)},{P(v)}));");
            return id;
        }

        /// <summary>
        /// Degree-1 2D spline on knot interval [0,1]. Geometric LINE pcurves are parameterized
        /// by |Δuv|, while 3D LINEs use world length — OCC then drops NURBS faces.
        /// </summary>
        public int WriteUvLine(Vec2D uv0, Vec2D uv1)
        {
            int p0 = WriteUvCartesianPoint(uv0.X, uv0.Y);
            int p1 = WriteUvCartesianPoint(uv1.X, uv1.Y);
            int id = NewId();
            _entities.Add(
                $"#{id}=B_SPLINE_CURVE_WITH_KNOTS('',1,(#{p0},#{p1}),.UNSPECIFIED.,.F.,.F.,(2,2),(0.0,1.0),.UNSPECIFIED.);");
            return id;
        }

        /// <summary>
        /// Degree-1 2D spline through all UV samples, same knot style as 3D strip polylines.
        /// </summary>
        public int WriteUvPolyline(IReadOnlyList<Vec2D> uv)
        {
            if (uv == null || uv.Count < 2)
                throw new ArgumentException("UV polyline needs at least two points.");
            if (uv.Count == 2)
                return WriteUvLine(uv[0], uv[uv.Count - 1]);

            var pointIds = new List<int>(uv.Count);
            for (int i = 0; i < uv.Count; i++)
                pointIds.Add(WriteUvCartesianPoint(uv[i].X, uv[i].Y));

            var knotsFull = BSplineCurve.UniformKnotVector(1, uv.Count);
            var (knots, mults) = NurbsEntityConverter.CompressKnotVector(knotsFull, 1);
            int id = NewId();
            _entities.Add(
                $"#{id}=B_SPLINE_CURVE_WITH_KNOTS('',1,({string.Join(",", pointIds.Select(i => "#" + i))}),.UNSPECIFIED.,.F.,.F.,({string.Join(",", mults)}),({string.Join(",", knots.Select(P))}),.UNSPECIFIED.);");
            return id;
        }

        /// <summary>Degree-1 3D spline on the same [0,1] interval as <see cref="WriteUvLine"/>.</summary>
        public int WriteUnitIntervalSegment3D(Vec3D p0, Vec3D p1)
        {
            int a = CreateCartesianPoint(p0);
            int b = CreateCartesianPoint(p1);
            int id = NewId();
            _entities.Add(
                $"#{id}=B_SPLINE_CURVE_WITH_KNOTS('',1,(#{a},#{b}),.UNSPECIFIED.,.F.,.F.,(2,2),(0.0,1.0),.UNSPECIFIED.);");
            return id;
        }

        public int WriteDefinitionalRepresentation(int curve2dId)
        {
            int ctx = GetParametricRepresentationContext2D();
            int id = NewId();
            _entities.Add($"#{id}=DEFINITIONAL_REPRESENTATION('',(#{curve2dId}),#{ctx});");
            return id;
        }

        public int WritePcurve(int surfaceId, int definitionalRepId)
        {
            int id = NewId();
            _entities.Add($"#{id}=PCURVE('',#{surfaceId},#{definitionalRepId});");
            return id;
        }

        public int WriteSurfaceCurve(int curve3dId, int pcurveId)
        {
            return WriteSurfaceCurve(curve3dId, new[] { pcurveId });
        }

        public int WriteSurfaceCurve(int curve3dId, IReadOnlyList<int> pcurveIds)
        {
            int id = NewId();
            string associated = pcurveIds == null || pcurveIds.Count == 0
                ? string.Empty
                : string.Join(",", pcurveIds.Select(i => "#" + i));
            _entities.Add($"#{id}=SURFACE_CURVE('',#{curve3dId},({associated}),.PCURVE_S1.);");
            return id;
        }

        public int WriteTrimEdgeCurve(int startVertexId, int endVertexId, int curve3dId, int surfaceId, Vec2D uv0, Vec2D uv1)
        {
            int uvLineId = WriteUvLine(uv0, uv1);
            int defRepId = WriteDefinitionalRepresentation(uvLineId);
            int pcurveId = WritePcurve(surfaceId, defRepId);
            int surfaceCurveId = WriteSurfaceCurve(curve3dId, pcurveId);
            return WriteEdgeCurve(startVertexId, endVertexId, surfaceCurveId);
        }

        public int GetOrCreateTrimEdgeCurve(
            int startVertexId,
            int endVertexId,
            int curve3dId,
            int surfaceId,
            Vec2D uv0,
            Vec2D uv1,
            out bool sameSense)
        {
            long key = EdgeKey(startVertexId, endVertexId);
            if (_sharedEdgeCurves.TryGetValue(key, out var shared))
            {
                sameSense = startVertexId == shared.StartVertexId && endVertexId == shared.EndVertexId;
                return shared.EdgeCurveId;
            }

            int edgeCurveId = WriteTrimEdgeCurve(startVertexId, endVertexId, curve3dId, surfaceId, uv0, uv1);
            _sharedEdgeCurves[key] = new SharedEdge(startVertexId, endVertexId, edgeCurveId);
            sameSense = true;
            return edgeCurveId;
        }

        private static long EdgeKey(int v0, int v1)
        {
            int a = Math.Min(v0, v1);
            int b = Math.Max(v0, v1);
            return ((long)a << 32) | (uint)b;
        }

        private readonly struct SharedEdge
        {
            public SharedEdge(int startVertexId, int endVertexId, int edgeCurveId)
            {
                StartVertexId = startVertexId;
                EndVertexId = endVertexId;
                EdgeCurveId = edgeCurveId;
            }

            public int StartVertexId { get; }
            public int EndVertexId { get; }
            public int EdgeCurveId { get; }
        }

        public int WriteOrientedEdge(int startVertexId, int endVertexId, int edgeCurveId, bool sameSense)
        {
            int id = NewId();
            _entities.Add($"#{id}=ORIENTED_EDGE('',*,*,#{edgeCurveId},{(sameSense ? ".T." : ".F.")});");
            return id;
        }

        public int WriteEdgeLoop(List<int> orientedEdgeIds)
        {
            int id = NewId();
            _entities.Add($"#{id}=EDGE_LOOP('',({string.Join(",", orientedEdgeIds.Select(i => "#" + i))}));");
            return id;
        }

        public int WriteFaceBound(int edgeLoopId, bool isOuter, bool orientation)
        {
            int id = NewId();
            string boundType = isOuter ? "FACE_OUTER_BOUND" : "FACE_BOUND";
            _entities.Add($"#{id}={boundType}('',#{edgeLoopId},{(orientation ? ".F." : ".T.")});");
            return id;
        }

        public int WriteAdvancedFace(int surfaceId, List<int> faceBoundIds)
        {
            int id = NewId();
            string bounds = faceBoundIds.Count > 0
                ? string.Join(",", faceBoundIds.Select(i => "#" + i))
                : string.Empty;
            _entities.Add($"#{id}=ADVANCED_FACE('',({bounds}),#{surfaceId},.F.);");
            return id;
        }

        public int WriteClosedShell(List<int> faceIds)
        {
            int id = NewId();
            _entities.Add($"#{id}=CLOSED_SHELL('',({string.Join(",", faceIds.Select(i => "#" + i))}));");
            return id;
        }

        public int WriteManifoldSolidBrep(int shellId)
        {
            int id = NewId();
            _entities.Add($"#{id}=MANIFOLD_SOLID_BREP('',#{shellId});");
            return id;
        }

        public int WriteOpenShell(List<int> faceIds)
        {
            int id = NewId();
            _entities.Add($"#{id}=OPEN_SHELL('',({string.Join(",", faceIds.Select(i => "#" + i))}));");
            return id;
        }

        public int WriteShellBasedSurfaceModel(int openShellId)
        {
            int id = NewId();
            _entities.Add($"#{id}=SHELL_BASED_SURFACE_MODEL('',(#{openShellId}));");
            return id;
        }

        /// <summary>
        /// AP214 product/shape root so OpenCASCADE and other kernels can transfer the solid.
        /// </summary>
        public void WriteAp214ProductStructure(int shapeItemId, string productName = "part", bool isSolid = true)
        {
            string safeName = string.IsNullOrWhiteSpace(productName) ? "part" : productName.Replace('\'', ' ');

            int appContext = NewId();
            _entities.Add($"#{appContext}=APPLICATION_CONTEXT('core data for automotive mechanical design processes');");

            int appProto = NewId();
            _entities.Add($"#{appProto}=APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{appContext});");

            int productContext = NewId();
            _entities.Add($"#{productContext}=PRODUCT_CONTEXT('',#{appContext},'mechanical');");

            int product = NewId();
            _entities.Add($"#{product}=PRODUCT('{safeName}','{safeName}','',(#{productContext}));");

            int pdf = NewId();
            _entities.Add($"#{pdf}=PRODUCT_DEFINITION_FORMATION('','',#{product});");

            int pdc = NewId();
            _entities.Add($"#{pdc}=PRODUCT_DEFINITION_CONTEXT('part definition',#{appContext},'design');");

            int pd = NewId();
            _entities.Add($"#{pd}=PRODUCT_DEFINITION('design','',#{pdf},#{pdc});");

            int pds = NewId();
            _entities.Add($"#{pds}=PRODUCT_DEFINITION_SHAPE('','',#{pd});");

            int axis = WriteAxis2Placement3D(new Vec3D(0, 0, 0));

            int lenUnit = NewId();
            _entities.Add($"#{lenUnit}=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );");

            int angleUnit = NewId();
            _entities.Add($"#{angleUnit}=( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );");

            int solidAngleUnit = NewId();
            _entities.Add($"#{solidAngleUnit}=( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );");

            int uncertainty = NewId();
            double unc = LinearUncertainty;
            if (double.IsNaN(unc) || double.IsInfinity(unc) || unc < 1e-9)
                unc = 1e-3;
            _entities.Add($"#{uncertainty}= UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({P(unc)}),#{lenUnit},'distance_accuracy_value','confusion accuracy');");

            int geomContext = NewId();
            _entities.Add(
                $"#{geomContext}=( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{lenUnit},#{angleUnit},#{solidAngleUnit})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') );");

            int shapeRep = NewId();
            string repType = isSolid
                ? "ADVANCED_BREP_SHAPE_REPRESENTATION"
                : "MANIFOLD_SURFACE_SHAPE_REPRESENTATION";
            _entities.Add($"#{shapeRep}={repType}('',(#{axis},#{shapeItemId}),#{geomContext});");

            int sdr = NewId();
            _entities.Add($"#{sdr}=SHAPE_DEFINITION_REPRESENTATION(#{pds},#{shapeRep});");
        }

        public int WriteDirection(Vec3D d)
        {
            var n = d.Length() > 1e-12 ? d.Normalized() : new Vec3D(0, 0, 1);
            int id = NewId();
            _entities.Add($"#{id}=DIRECTION('',({P(n.X)},{P(n.Y)},{P(n.Z)}));");
            return id;
        }

        public int WriteAxis2Placement3D(Vec3D origin)
        {
            return WriteAxis2Placement3D(origin, new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
        }

        public int WriteAxis2Placement3D(Vec3D origin, Vec3D axisZ, Vec3D axisX)
        {
            int p = CreateCartesianPoint(origin);
            int z = WriteDirection(axisZ);
            int x = WriteDirection(axisX);
            int id = NewId();
            _entities.Add($"#{id}=AXIS2_PLACEMENT_3D('',#{p},#{z},#{x});");
            return id;
        }

        public int WritePlane(Vec3D origin, Vec3D normal, Vec3D refDirection)
        {
            int axis = WriteAxis2Placement3D(origin, normal, refDirection);
            int id = NewId();
            _entities.Add($"#{id}=PLANE('',#{axis});");
            return id;
        }

        public int WriteCylindricalSurface(Vec3D origin, Vec3D axis, Vec3D refDirection, double radius)
        {
            int axisId = WriteAxis2Placement3D(origin, axis, refDirection);
            int id = NewId();
            _entities.Add($"#{id}=CYLINDRICAL_SURFACE('',#{axisId},{P(radius)});");
            return id;
        }

        public int WriteConicalSurface(Vec3D origin, Vec3D axis, Vec3D refDirection, double radius, double semiAngle)
        {
            int axisId = WriteAxis2Placement3D(origin, axis, refDirection);
            int id = NewId();
            _entities.Add($"#{id}=CONICAL_SURFACE('',#{axisId},{P(radius)},{P(semiAngle)});");
            return id;
        }

        public int WriteSphericalSurface(Vec3D center, Vec3D axis, Vec3D refDirection, double radius)
        {
            int axisId = WriteAxis2Placement3D(center, axis, refDirection);
            int id = NewId();
            _entities.Add($"#{id}=SPHERICAL_SURFACE('',#{axisId},{P(radius)});");
            return id;
        }

        public int WriteToroidalSurface(Vec3D center, Vec3D axis, Vec3D refDirection, double majorRadius, double minorRadius)
        {
            int axisId = WriteAxis2Placement3D(center, axis, refDirection);
            int id = NewId();
            _entities.Add($"#{id}=TOROIDAL_SURFACE('',#{axisId},{P(majorRadius)},{P(minorRadius)});");
            return id;
        }

        public int WriteCircle(Vec3D center, Vec3D axis, Vec3D refDirection, double radius)
        {
            int axisId = WriteAxis2Placement3D(center, axis, refDirection);
            int id = NewId();
            _entities.Add($"#{id}=CIRCLE('',#{axisId},{P(radius)});");
            return id;
        }

        public int WritePlaneFromBilinearPatch(BSplineSurface surface)
        {
            if (!ExportAnalyticSurface.PlaneFromBilinear(surface, out var origin, out var normal, out var refDir))
                throw new InvalidOperationException("WritePlaneFromBilinearPatch requires a non-degenerate 2x2 bilinear patch.");
            return WritePlane(origin, normal, refDir);
        }

        public void Save(string path)
        {
            string now = DateTime.UtcNow.ToString("yyyy-MM-ddTHH:mm:ss");
            using var w = new StreamWriter(path, false);
            w.WriteLine("ISO-10303-21;");
            w.WriteLine("HEADER;");
            w.WriteLine("FILE_DESCRIPTION(('NURBS B-Rep export'),'2;1');");
            w.WriteLine($"FILE_NAME('{Path.GetFileName(path)}','{now}',('CSG'),('CSG'),'','','');");
            w.WriteLine("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));");
            w.WriteLine("ENDSEC;");
            w.WriteLine("DATA;");
            foreach (var e in _entities)
                w.WriteLine(e);
            w.WriteLine("ENDSEC;");
            w.WriteLine("END-ISO-10303-21;");
        }
    }
}
