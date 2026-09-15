using CSG;
using Curves;
using Geo.NurbsConstruction;
using GeoCore;
using GeoMeta;
using GeoScriptViewer;
using GeoSolver.Sketcher;
using System.IO;

namespace Geo
{
    [APIDescription(@"
            GeoAPI: builder for parametric solids (CAD-style: sketches → extrude/revolve/loft/sweep → CSG booleans → fillet/copy → STL/OBJ/USDA).

Construction:
  GeoAPI(operatingSpace: Box3D, maxDeviation: float, operatingSpaceSlices: int = 1000000, name: str = """")
  - operatingSpace: bounding box for all geometry; coords are quantized to its integer lattice.
  - maxDeviation: required default chordal tessellation tolerance (world units) when methods receive maxDeviation=-1. Use ~1e-4 for metre-scale parts, ~0.01 for mm-scale.
  - operatingSpaceSlices: lattice divisions along the longest box axis (default 1M). Smallest unit ≈ extent / slices.
  - name: instance name (""Part<n>"" if empty); registered in GeoAPI.GetInstances().

Static:
  GeoAPI.Clear()                                                 # reset all instances + name counters; call at script start
  GeoAPI.GenerateName(prefix: str) -> str                        # returns ""prefix1"", ""prefix2"", ...
  GeoAPI.GetInstances() -> Dict[str, GeoAPI]

Names: every Add/Create/Extrude/Revolve/Loft/Boolean/Fillet/CopyMesh accepts an optional `name` (auto-generated when null).
Surface patches inside a mesh have hierarchical names (""<meshName>-<contourName>"" or ""<meshName>-<contourName>-<guideCurveName>""; caps use ""-ExtrudeTop"" / ""-ExtrudeBottom""); pass these strings to Fillet and feature lookups. Group edges use ""[{patchA},{patchB}]"".

Example Python Scripts:
Script 1 (extrude rectangle into cube via sketch on a default plane):
  from Geo import GeoAPI
  from GeoCore import Box3D, Vec2D, Vec3D
  from Curves import SketchFontSpec
  GeoAPI.Clear()
  part = GeoAPI(Box3D(Vec3D(-1), Vec3D(1)), 1e-4)
  sk = part.GetPlotterSketcher(""OriginXY"", ""mySketch"")
  sk.AddLine(Vec2D(0,0), Vec2D(1,0)); sk.AddLine(Vec2D(1,0), Vec2D(1,1))
  sk.AddLine(Vec2D(1,1), Vec2D(0,1)); sk.AddLine(Vec2D(0,1), Vec2D(0,0))
  cube = part.Extrude(sk, 1.0, name=""Cube"")

Text outlines come from TrueType fonts (no WinForms). Family names resolve against installed .ttf/.ttc files, falling back to Arial / Liberation Sans / DejaVu Sans:
  textSk = part.GetPlotterSketcher(""OriginXY"", ""textSketch"")
  textSk.AddText(""CSG"", Vec2D(0,0), SketchFontSpec(""Arial"", 0.2))
  textMesh = part.Extrude(textSk, 0.05, name=""TextBlock"")

Script 2 (boolean difference):
  cyl = part.CreateCylinder(CoordinateSystem(Vec3D(0.5,0.5,-0.1)), 0.2, 1.5)
  result = part.Boolean(cube, cyl, BooleanOp.Difference, ""CubeMinusCyl"")
  part.SaveBinaryStlFile(result, ""out.stl"")
")]
    public partial class GeoAPI
    {
        private static int GroupIndexer = 0;
        private static Dictionary<string, GeoAPI> Instances = new Dictionary<string, GeoAPI>();
        /// <summary>
        /// Protects <see cref="Instances"/>, global name counters, and registration in the GeoAPI ctor.
        /// Parallel tests (e.g. xUnit + VizRunner) otherwise race: <see cref="Clear"/> can run between
        /// <c>ContainsKey</c> and increment in name generation, producing <see cref="KeyNotFoundException"/>.
        /// </summary>
        private static readonly object StaticStateLock = new object();

        [APIDescription(@"GetInstances() -> Dict[str, GeoAPI]
Live dictionary of all GeoAPI instances by Name. Useful for cross-script lookup.")]
        public static Dictionary<string, GeoAPI> GetInstances() { return Instances; }
        [APIDescription(@"Clear()
Static. Removes all GeoAPI instances and resets all auto-name counters. Call once at the start of a script.")]
        public static void Clear()
        {
            lock (StaticStateLock)
            {
                Instances.Clear();
                EntityNaming.ResetGlobalNameCounters();
                // Leave GroupIndexer alone so parallel tests that call Clear() between
                // CreateCube A and CreateCube B cannot reuse the same group IDs.
            }
        }

        /// <summary>
        /// Generates a unique name for any object type within GeoAPI.
        /// </summary>
        /// <param name="prefix">The prefix for the name (e.g., "Part", "Sketch", "Plane", "Line")</param>
        /// <returns>A unique name like "Part1", "Sketch2", etc.</returns>
        [APIDescription(@"GenerateName(prefix: str) -> str
Static. Returns the next unique name with this prefix (""<prefix>1"", ""<prefix>2"", ...). Counters are per-prefix and reset by Clear(). Used internally when an optional `name` parameter is null.")]
        public static string GenerateName(string prefix) => EntityNaming.GenerateGlobalName(prefix);

        internal static string ResolveSketchName(string name) => name ?? GenerateName("Sketch");

        private List<AnchorMesh> meshes = new List<AnchorMesh>();
        private List<PlotterSketcherCoordSys> sketches = new List<PlotterSketcherCoordSys>();
        private List<Assembly> assemblies = new List<Assembly>();
        private GeoVisualOutputKind visualOutputKind = GeoVisualOutputKind.Mesh;
        private Assembly lastActiveAssembly;
        private List<Curve3D> curves3D = new List<Curve3D>();
        private List<Plane3D> planes3D = new List<Plane3D>();
        protected string _name; public string Name { get { return _name; } set { _name = value; } }

        private CoordinateConverter converter; // Operating space
        public CoordinateConverter Converter { get { return converter; } }
        public static CoordinateConverter debugConverter;

        private readonly double _maxDeviation;
        /// <summary>Default chordal tessellation tolerance for this part (world units).</summary>
        public double MaxDeviation => _maxDeviation;

        private double ResolveMaxDeviation(double maxDeviation) =>
            maxDeviation < 0 ? _maxDeviation : maxDeviation;



        [APIDescription(@"GeoAPI(operatingSpace: Box3D, maxDeviation: float, operatingSpaceSlices: int = 1000000, name: str = """")
Creates a part in the given operating space.
  operatingSpace: bounding box for all geometry; coords quantized to its integer lattice.
  maxDeviation: required default tessellation tolerance (world units) when methods pass maxDeviation=-1 (~1e-4 m, ~0.01 mm).
  operatingSpaceSlices: lattice divisions along longest box axis (default 1M). Smallest unit ≈ extent / slices.
  name: instance name (""Part<n>"" if empty); registered in GeoAPI.GetInstances().")]
        public GeoAPI(Box3D operatingSpace, double maxDeviation, int operatingSpaceSlices = 1_000_000, string name = "")
        {
            if (maxDeviation <= 0)
                throw new ArgumentOutOfRangeException(nameof(maxDeviation), "maxDeviation must be positive.");
            _maxDeviation = maxDeviation;
            // Use provided name or generate one
            _name = !string.IsNullOrEmpty(name) ? name : GenerateName("Part");
            converter = new CoordinateConverter(operatingSpace, operatingSpaceSlices);
            debugConverter = converter;
            lock (StaticStateLock)
            {
                if (Instances.ContainsKey(Name))
                    throw new NameCollisionException($"GeoAPI instance name already registered: '{Name}'.");
                Instances[Name] = this;
            }
        }

        public static int GetBaseGroupIndex()
        {
            lock (StaticStateLock)
                return GroupIndexer;
        }

        public static void IncrementBaseGroupIndex(int increment)
        {
            lock (StaticStateLock)
                GroupIndexer += increment;
        }



        [APIDescription(@"AddLine(start: Vec3D, end: Vec3D, lineName: str = None) -> Line3D
Adds a 3D line segment to the API's curve list. lineName auto-generated if null. Returned curve can be used as a sweep guide (ExtrudeAlongCurve).")]
        public Line3D AddLine(Vec3D start, Vec3D end, string lineName = null)
        {
            var line = new Line3D(start, end, lineName);
            curves3D.Add(line);
            return line;
        }
        [APIDescription(@"AddLine(pointNameStart: str, pointNameEnd: str, lineName: str = None) -> Line3D
Adds a 3D line between two named points (mesh anchors / DefaultPoints.Origin). Throws if either name does not resolve.")]
        public Line3D AddLine(string pointNameStart, string pointNameEnd, string lineName = null)
        {
            var startPoint = GetPointFromName(pointNameStart);
            var endPoint = GetPointFromName(pointNameEnd);

            var line = new Line3D(startPoint, endPoint, lineName);
            curves3D.Add(line);
            return line;
        }
        [APIDescription(@"AddPlane(planeName: str, originAnchorName: str)
Creates a Plane3D anchored on the top mesh's surface at the named anchor point. The plane normal = surface normal at anchor; tangentX/Y are orthonormalized from the surface frame. The plane is then resolvable by name (e.g. for GetPlotterSketcher(planeName)). Throws if the anchor does not exist or lies outside the surface border.")]
        public void AddPlane(string planeName, string originAnchorName)
        {
            var mesh = GetTopMesh();

            if (!mesh.TryGetPointOnSurface(originAnchorName, out var point, out var normal, out var uv, out var tangentX, out var tangentY))
                throw new Exception(originAnchorName + " does not exist or is outside of the surface border");

            normal.Normalize();

            tangentY = Vec3DOps.Cross(normal, tangentX);
            tangentY.Normalize();

            tangentX = Vec3DOps.Cross(tangentY, normal);
            tangentX.Normalize();

            planes3D.Add(new Plane3D(point, normal, tangentX, tangentY, planeName));
        }


        [APIDescription(@"GetPlotterSketcher(plane: Plane3D, name: str = None) -> ConstrainedSketcher
Creates a constrained 2D sketcher in the given plane's coordinate system. Use for parametric/constrained sketches (call .Add* and .Solve()). Sketch is registered for visualization. name auto-generated if null.")]
        public ConstrainedSketcher GetPlotterSketcher(Plane3D plane, string name = null)
        {
            var result = new ConstrainedSketcher(ResolveSketchName(name), plane.GetCoordinateSystem());
            RegisterSketch(result);
            return result;
        }

        // If you place a sketch onto a planar mesh surface, the sketch origin will be approximately
        // at the center of that surface patch. But it is recommended to use other reference points
        // when defining sketch entities.
        [APIDescription(@"GetPlotterSketcher(sketchPlaneName: str, name: str = None) -> PlotterSketcherCoordSys
Creates an unconstrained 2D sketcher on a named plane. Plane name resolves to: DefaultPlanes.OriginXY/OriginYZ/OriginZX, a plane added by AddPlane, or a planar surface patch on the top mesh. Throws if not resolvable.
Use only for very simple procedural sketches (plain polylines, no dimension/constraint annotations, no parametric intent). When recreating a user sketch image or dimensioned drawing, use GetConstraintSketcher instead.")]
        public PlotterSketcherCoordSys GetPlotterSketcher(string sketchPlaneName, string name = null)
        {
            if (!TryGetPlaneFromName(sketchPlaneName, out var plane) || plane == null)
                throw new Exception("Plane " + sketchPlaneName + " does not exist");
            var result = new PlotterSketcherCoordSys(ResolveSketchName(name), plane.GetCoordinateSystem());
            RegisterSketch(result);
            return result;
        }
        [APIDescription(@"GetConstraintSketcher(sketchPlaneName: str, name: str = None) -> ConstrainedSketcher
Like GetPlotterSketcher(string,...) but returns a constraint-capable sketcher (supports geometric constraints + .Solve()).
Default choice when turning a user-provided sketch image or dimensioned drawing into geometry — model lines/arcs/circles and apply Fix*/Set* constraints (standard CAD). Prefer this over GetPlotterSketcher for anything beyond trivial polylines without annotations.")]
        public ConstrainedSketcher GetConstraintSketcher(string sketchPlaneName, string name = null)
        {
            if (!TryGetPlaneFromName(sketchPlaneName, out var plane) || plane == null)
                throw new Exception("Plane " + sketchPlaneName + " does not exist");
            var result = new ConstrainedSketcher(ResolveSketchName(name), plane.GetCoordinateSystem(), new ConstrainableCurveFactory());
            RegisterSketch(result);
            return result;
        }

        [APIDescription(@"GetConstraintSketcher(coordinateSystem: CoordinateSystem, name: str = None) -> ConstrainedSketcher
Creates a constraint-capable sketcher in an explicit world CoordinateSystem (origin + axes). Use this for interactive sketches on a computed face frame.")]
        public ConstrainedSketcher GetConstraintSketcher(CoordinateSystem coordinateSystem, string name = null)
        {
            var result = new ConstrainedSketcher(ResolveSketchName(name), coordinateSystem, new ConstrainableCurveFactory());
            RegisterSketch(result);
            return result;
        }

        [APIDescription(@"GetAssembly(name: str = None) -> Assembly
Creates or returns a named 3D assembly mate solver for meshes on this GeoAPI instance.")]
        public Assembly GetAssembly(string name = null)
        {
            name = name ?? GenerateName("Assembly");
            for (int i = 0; i < assemblies.Count; i++)
            {
                if (assemblies[i].Name == name)
                    return assemblies[i];
            }

            var assembly = new Assembly(this, name);
            RegisterAssembly(assembly);
            return assembly;
        }

        [APIDescription(@"GetPlotterSketcher(sketchPlaneName: str, originPointName: str, name: str = None) -> PlotterSketcherCoordSys
Same as GetPlotterSketcher(planeName), but the sketch origin is the named point projected onto the plane. originPointName resolves like AddLine names (DefaultPoints.Origin or a mesh anchor). Throws if either name is unresolvable.")]
        public PlotterSketcherCoordSys GetPlotterSketcher(string sketchPlaneName, string originPointName, string name = null)
        {
            if (!TryGetPlaneFromName(sketchPlaneName, out var plane) || plane == null)
                throw new Exception("Plane " + sketchPlaneName + " does not exist");

            if(!TryGetPointFromName(originPointName, out var point))
                throw new Exception("Point " + originPointName + " does not exist");

            point = GeometricAlgorithms.ProjectPointOntoPlane(point, plane.Normal, plane.Origin);

            var cs = plane.GetCoordinateSystem();
            cs.Origin = point;

            var result = new PlotterSketcherCoordSys(ResolveSketchName(name), cs);
            RegisterSketch(result);
            return result;
        }

        /// <summary>
        /// Creates a sketch in the given world <see cref="CoordinateSystem"/> and registers it for visualization
        /// and <see cref="GeoAPI.Access.GetSketches"/>. Prefer this over constructing <see cref="PlotterSketcherCoordSys"/>
        /// directly so GeoScriptViewer can show the sketch.
        /// </summary>
        [APIDescription(@"GetPlotterSketcher(coordinateSystem: CoordinateSystem, name: str = None) -> PlotterSketcherCoordSys
Creates an unconstrained 2D sketcher in an explicit world CoordinateSystem (origin + axes). Use this when the sketch frame is computed (e.g. per-section frames in a loft). Sketch is registered for visualization. name auto-generated if null.")]
        public PlotterSketcherCoordSys GetPlotterSketcher(CoordinateSystem coordinateSystem, string name = null)
        {
            var result = new PlotterSketcherCoordSys(ResolveSketchName(name), coordinateSystem);
            RegisterSketch(result);
            return result;
        }

        [APIDescription(@"SaveStlFile(mesh: AnchorMesh, filePath: str)
Writes mesh as ASCII STL (positions + triangles only; no normals/UVs/groups).")]
        public void SaveStlFile(AnchorMesh mesh, string filePath)
        {
            mesh.EnsureCoplanarPostProcessed();
            mesh.Mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out var perTriangleGroup);
            filePath = ResolvePath(filePath);
            EnsureParentDirectory(filePath);
            StlWriter.WriteAscii(filePath, positions, triangles);
        }
        [APIDescription(@"SaveBinaryStlFile(mesh: AnchorMesh, filePath: str)
Writes mesh as binary STL (smaller files than ASCII STL). Vertex coordinates are world-space
positions in the same units as the GeoAPI operating box (e.g. meters when geometry is built in meters).")]
        public void SaveBinaryStlFile(AnchorMesh mesh, string filePath)
        {
            mesh.EnsureCoplanarPostProcessed();
            mesh.Mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out var perTriangleGroup);
            filePath = ResolvePath(filePath);
            EnsureParentDirectory(filePath);
            StlWriter.WriteBinary(filePath, positions, triangles);
        }

        static void EnsureParentDirectory(string filePath)
        {
            string dir = Path.GetDirectoryName(filePath);
            if (!string.IsNullOrEmpty(dir))
                Directory.CreateDirectory(dir);
        }

        [APIDescription(@"SaveOffFile(mesh: AnchorMesh, filePath: str)
Writes mesh as OFF (positions + triangles).")]
        public void SaveOffFile(AnchorMesh mesh, string filePath)
        {
            mesh.EnsureCoplanarPostProcessed();
            mesh.Mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out var perTriangleGroup);
            OffWriter.Write(filePath, positions, triangles);
        }

        [APIDescription(@"SaveStepFile(mesh: AnchorMesh, filePath: str)
Exports mesh as STEP B-Rep (NURBS surfaces + mesh-derived trim boundaries).")]
        public void SaveStepFile(AnchorMesh mesh, string filePath)
        {
            mesh.EnsureCoplanarPostProcessed();
            Geo.Export.StepExporter.Export(mesh, filePath, _maxDeviation);
        }

        [APIDescription(@"SaveIgesFile(mesh: AnchorMesh, filePath: str)
Exports mesh as IGES trimmed surfaces (analytic plane/cylinder/cone/sphere/torus when metadata is present, otherwise NURBS).")]
        public void SaveIgesFile(AnchorMesh mesh, string filePath)
        {
            mesh.EnsureCoplanarPostProcessed();
            Geo.Export.IgesExporter.Export(mesh, filePath);
        }

        [APIDescription(@"SaveWavefrontObjFile(mesh: AnchorMesh, filePath: str)
Writes mesh as Wavefront OBJ including normals, UVs, and per-group ""g <name>"" tags using the mesh's surface patch names.")]
        public void SaveWavefrontObjFile(AnchorMesh mesh, string filePath)
        {
            mesh.EnsureCoplanarPostProcessed();
            mesh.Mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out var perTriangleGroup);
            ObjWriter.Write(filePath, positions, normals, uvs, triangles, perTriangleGroup, mesh.groupIdToExtendedName);
        }

        [APIDescription(@"SaveUsdaFile(mesh: AnchorMesh, filePath: str)
Writes a triangulated USD ASCII (.usda) mesh with per-vertex normals, UVs, and GeomSubset patches named from surface groups.")]
        public void SaveUsdaFile(AnchorMesh mesh, string filePath)
        {
            mesh.EnsureCoplanarPostProcessed();
            mesh.Mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out var perTriangleGroup);
            filePath = ResolvePath(filePath);
            EnsureParentDirectory(filePath);
            UsdaWriter.Write(filePath, mesh.Name, positions, normals, uvs, triangles, perTriangleGroup, mesh.groupIdToExtendedName);
        }

        [APIDescription(@"LoadWavefrontObjFile(filePath: str, groupBorderAngleThresholdDegree: float = -1, name: str = None, scale: float = 1) -> AnchorMesh
Loads an OBJ file as an AnchorMesh and registers it.
  groupBorderAngleThresholdDegree: if > 0, surface patches are auto-detected by dihedral angle (degrees) between adjacent faces; if <= 0, the OBJ ""g <name>"" tags define groups.
  name auto-generated if null. scale multiplies vertex positions (e.g. 0.01 converts centimetres to metres). Throws on malformed input or non-positive scale.")]
        public AnchorMesh LoadWavefrontObjFile(string filePath, double groupBorderAngleThresholdDegree = -1, string name = null, double scale = 1.0)
        {
            if (scale <= 0)
                throw new ArgumentOutOfRangeException(nameof(scale), "Scale must be positive.");

            name = name ?? GenerateName("ObjMesh");
            List<PNTGeometry> geo = ObjReader.Load(filePath, out string materialLibraryPath);
            if (geo.Count == 0)
                throw new Exception("OBJ file contains no geometry.");

            int offset = GetBaseGroupIndex();

            List<Vec3D> allPoints = new List<Vec3D>();
            List<Vec3D> allNormals = new List<Vec3D>();
            List<Vec2D> allUv = new List<Vec2D>();
            List<Tri> allTriangles = new List<Tri>();
            List<int> groupPerTriangle = new List<int>();
            for (int i=0;i<geo.Count;++i)
            {
                int o = allPoints.Count;
                var g = geo[i];
                allPoints.AddRange(g.Position);
                allNormals.AddRange(g.Normal);
                allUv.AddRange(g.Texture);

                var tris = g.Triangles;
                for(int j=0;j<tris.Count;++j)
                {
                    var tri = tris[j];
                    tri.A += o;
                    tri.B += o;
                    tri.C += o;
                    allTriangles.Add(tri);
                    groupPerTriangle.Add(i + offset);
                }
            }

            if (scale != 1.0)
            {
                for (int i = 0; i < allPoints.Count; ++i)
                    allPoints[i] *= scale;
            }


            Dictionary<int, string> triangleGroupToName = new Dictionary<int, string>();
            int numGroups = -1;
            if (groupBorderAngleThresholdDegree > 0)
            {
                numGroups = AutoGroups.AutoDetectPatches(allPoints, allTriangles, groupBorderAngleThresholdDegree.ToRadians(), out groupPerTriangle, offset);
                IncrementBaseGroupIndex(numGroups);


                for (int i = offset; i < offset + numGroups; ++i)
                    triangleGroupToName.Add(i, EntityNaming.ImportAutoGroup(EntityNaming.ObjAutoGroupPrefix, i));
            }
            else
            {
                numGroups = geo.Count;
                IncrementBaseGroupIndex(numGroups);

                for (int i = offset; i < offset + numGroups; ++i)
                {
                    string groupName = geo[i - offset].Name;
                    triangleGroupToName.Add(i, groupName ?? EntityNaming.ImportAutoGroup(EntityNaming.ObjAutoGroupPrefix, i));
                }
            }

            bool hasVertexAttributes = allNormals.Count == allPoints.Count && allUv.Count == allPoints.Count;
            bool isWatertight = MeshAnalysis.IsWatertightMesh(allPoints, allTriangles);
            bool isVolume = isWatertight;

            MeshNormalUV mesh;
            if (hasVertexAttributes)
            {
                mesh = new MeshNormalUV(converter, allPoints, allNormals, allUv, allTriangles, groupPerTriangle, skipWatertightCheck: !isVolume);
            }
            else
            {
                double angleThreshold = groupBorderAngleThresholdDegree > 0
                    ? groupBorderAngleThresholdDegree.ToRadians()
                    : 60.0.ToRadians();

                var normals = AutoNormals.ComputeNormals(allPoints, allTriangles, angleThreshold);
                var autoUV = AutoUV.AutoUvPerPatch(allPoints, allTriangles, groupPerTriangle);

                List<MeshTriangle<TriangleVertexNormalUV>> triangleCornerData = new List<MeshTriangle<TriangleVertexNormalUV>>(allTriangles.Count);
                for (int i = 0; i < allTriangles.Count; ++i)
                {
                    var t = new MeshTriangle<TriangleVertexNormalUV>();
                    var uv = autoUV[i];
                    t.V0 = new TriangleVertexNormalUV { Normal = normals[3 * i + 0], UV = uv.UvA };
                    t.V1 = new TriangleVertexNormalUV { Normal = normals[3 * i + 1], UV = uv.UvB };
                    t.V2 = new TriangleVertexNormalUV { Normal = normals[3 * i + 2], UV = uv.UvC };
                    triangleCornerData.Add(t);
                }

                mesh = new MeshNormalUV(converter, allPoints, allTriangles, triangleCornerData, groupPerTriangle, skipWatertightCheck: !isVolume);
            }

            var result = new AnchorMesh(name, mesh, triangleGroupToName, new Dictionary<string, SurfaceMetaData>(), isVolume);
            ImportPatchMetadataBuilder.Attach(result);
            RegisterMesh(result);
            //nameOfMostRecentMesh = name;
            return result;
        }


        //public AnchorMesh LoadWavefrontObjFile(string name, string filePath, double groupBorderAngleThresholdDegree = -1)
        //{
        //    var geo = ObjReader.Load(filePath, out string materialLibraryPath);
        //    if (geo.Count != 1)
        //        throw new Exception("Obj files with multiple geometries not supported");

        //    var g = geo[0];
        //    List<Vec3D> allPoints = new List<Vec3D>();
        //    List<Vec3D> allNormals = new List<Vec3D>();
        //    List<Vec2D> allUv = new List<Vec2D>();


        //    int offset = GetBaseGroupIndex();
        //    var numGroups = AutoGroups.AutoDetectPatches(g.Position, g.Triangles, groupBorderAngleThresholdDegree.ToRadians(), out var groupPerTriangle, offset);
        //    IncrementBaseGroupIndex(numGroups);

        //    List<MeshTriangle<TriangleVertexNormalUV>> triangleCornerData = new List<MeshTriangle<TriangleVertexNormalUV>>(g.Triangles.Count);
        //    var normals = g.Normal;
        //    var uv = g.Texture;
        //    for (int i = 0; i < g.Triangles.Count; ++i)
        //    {
        //        var tri = g.Triangles[i];
        //        var t = new MeshTriangle<TriangleVertexNormalUV>();
        //        t.V0.UV = uv[tri.A];
        //        t.V0.Normal = normals[tri.A];
        //        t.V1.UV = uv[tri.B];
        //        t.V1.Normal = normals[tri.B];
        //        t.V2.UV = uv[tri.C];
        //        t.V2.Normal = normals[tri.C];
        //        triangleCornerData.Add(t);
        //    }

        //    MeshNormalUV mesh = new MeshNormalUV(converter, g.Position, g.Triangles, triangleCornerData, groupPerTriangle);

        //    Dictionary<int, string> triangleGroupToName = new Dictionary<int, string>();
        //    for (int i = offset; i < offset + numGroups; ++i)
        //        triangleGroupToName.Add(i, "ObjAutoGroup_" + i);

        //    var result = new AnchorMesh(mesh, /*edges,*/ triangleGroupToName);
        //    meshes.Add(name, result);
        //    nameOfMostRecentMesh = name;
        //    return result;
        //}

        [APIDescription(@"LoadOffFile(filePath: str, groupBorderAngleThresholdDegree: float, name: str = None, requireWatertight: bool = True) -> AnchorMesh
Loads OFF file. groupBorderAngleThresholdDegree: dihedral angle in degrees for auto patch detection (required, must be > 0). Patches are detected on both closed solids and open sheets. When requireWatertight=True (default), throws if the mesh is not watertight after duplicate point merge and marks it as a volume (IsVolume=True). Set requireWatertight=False to allow open trim/visualisation meshes (IsVolume=False when open). name auto-generated if null.")]
        public AnchorMesh LoadOffFile(string filePath, double groupBorderAngleThresholdDegree, string name = null, bool requireWatertight = true)
        {
            name = name ?? GenerateName("OffMesh");
            OffReader.Read(filePath, out var points, out var triangles);
            return MeshFromPointsAndTriangles(points, triangles, groupBorderAngleThresholdDegree, EntityNaming.OffAutoGroupPrefix, name, requireWatertight);
        }

        [APIDescription(@"LoadStlFile(filePath: str, groupBorderAngleThresholdDegree: float, name: str = None, requireWatertight: bool = True) -> AnchorMesh
Loads STL (ASCII or binary). groupBorderAngleThresholdDegree: dihedral angle in degrees for auto patch detection (required, must be > 0). Patch detection does not require a watertight mesh. When requireWatertight=True (default), throws if not watertight after duplicate point merge and sets IsVolume=True. Set requireWatertight=False for open trim surfaces or visualisation assemblies (IsVolume follows actual watertightness). name auto-generated if null.")]
        public AnchorMesh LoadStlFile(string filePath, double groupBorderAngleThresholdDegree, string name = null, bool requireWatertight = true)
        {
            name = name ?? GenerateName("StlMesh");
            StlReader.Read(filePath, out var points, out var triangles);
            return MeshFromPointsAndTriangles(points, triangles, groupBorderAngleThresholdDegree, EntityNaming.StlAutoGroupPrefix, name, requireWatertight);
        }

        private AnchorMesh MeshFromPointsAndTriangles(List<Vec3D> points, List<Tri> triangles, double groupBorderAngleThresholdDegree, string groupNamePrefix, string name, bool requireWatertight = true)
        {
            var map = DuplicatePointRemover.DuplicateMap(points);
            triangles = DuplicatePointRemover.MapTriangles(triangles, map);

            bool isWatertight = MeshAnalysis.IsWatertightMesh(points, triangles);
            if (requireWatertight && !isWatertight)
                throw new Exception("Mesh is not watertight after duplicate point merge");

            // Closed manifold → volume solid; open boundary → surface/sheet (trim, visualisation, …).
            bool isVolume = isWatertight;

            int offset = GetBaseGroupIndex();
            var numGroups = AutoGroups.AutoDetectPatches(points, triangles, groupBorderAngleThresholdDegree.ToRadians(), out var groupPerTriangle, offset);


            var normals = AutoNormals.ComputeNormals(points, triangles, groupBorderAngleThresholdDegree.ToRadians());

            var autoUV = AutoUV.AutoUvPerPatch(points, triangles, groupPerTriangle);

            IncrementBaseGroupIndex(numGroups);

            //List<Vec2D> uvZero = new List<Vec2D>(points.Count);
            //for (int i = 0; i < points.Count; ++i)
            //    uvZero.Add(new Vec2D(0));

            List<MeshTriangle<TriangleVertexNormalUV>> triangleCornerData = new List<MeshTriangle<TriangleVertexNormalUV>>(triangles.Count);
            for(int i=0;i<triangles.Count;++i)
            {
                var t = new MeshTriangle<TriangleVertexNormalUV>();
                var uv = autoUV[i];
                t.V0.UV = uv.UvA;
                t.V0.Normal = normals[3 * i + 0];
                t.V1.UV = uv.UvB;
                t.V1.Normal = normals[3 * i + 1];
                t.V2.UV = uv.UvC;
                t.V2.Normal = normals[3 * i + 2];
                triangleCornerData.Add(t);
            }

            MeshNormalUV mesh = new MeshNormalUV(converter, points, triangles, triangleCornerData, groupPerTriangle, skipWatertightCheck: !isVolume);

            Dictionary<int, string> triangleGroupToName = new Dictionary<int, string>();
            for (int i = offset; i < offset + numGroups; ++i)
                triangleGroupToName.Add(i, EntityNaming.ImportAutoGroup(groupNamePrefix, i));

            var result = new AnchorMesh(name, mesh, triangleGroupToName, new Dictionary<string, SurfaceMetaData>(), isVolume);
            ImportPatchMetadataBuilder.Attach(result);
            RegisterMesh(result);
            //nameOfMostRecentMesh = name;
            return result;
        }

        /// <summary>
        /// Sweeps a 2D profile sketch along a 3D guide curve (e.g. a <see cref="Curves.Helix3D"/> or <see cref="Curves.Spiral3D"/>).
        /// </summary>
        /// <remarks>
        /// <para><b>Profile sketch (<paramref name="sketch"/>) and world space</b></para>
        /// <list type="bullet">
        /// <item>
        /// <description>
        /// The sketch lives in the plane spanned by <see cref="PlotterSketcherCoordSys.CoordinateSystem"/> axes
        /// <b>X</b> and <b>Y</b> through that origin. Each tessellated contour point <c>(u, v)</c> is placed at
        /// <c>Origin + u·X + v·Y</c> in world space (see <see cref="CoordinateSystem.PointTo3D"/>).
        /// </description>
        /// </item>
        /// <item>
        /// <description>
        /// The sweep direction is <i>along the guide curve</i>: at each sample, the same 2D contour is mapped into the
        /// plane through the curve point with tangent as extrusion axis; see <see cref="Extruder"/> cross-section
        /// generation using <see cref="CurveVertex3D.GetCoordinateSystem"/> (profile plane uses curve frame X,Y; tangent is Z).
        /// </description>
        /// </item>
        /// </list>
        /// <para><b>Aligning the guide to the profile</b></para>
        /// <list type="bullet">
        /// <item>
        /// <description>
        /// <see cref="OrientCurveFramesToProfile"/> rotates/reorders the tessellated curve so the start frame matches
        /// <see cref="PlotterSketcherCoordSys.CoordinateSystem"/>. The profile CS and the curve’s first vertex frame must agree:
        /// put the sketch origin on the curve start (or the closest rotation will still be applied).
        /// </description>
        /// </item>
        /// </list>
        /// <para><b>Helix example (<see cref="Curves.Helix3D"/>)</b></para>
        /// <list type="bullet">
        /// <item>
        /// <description>
        /// <see cref="Curves.Helix3D.ZAxis"/> is <c>Cross(X, Y)</c> and is the axis along which the helix advances
        /// (<see cref="Curves.Helix3D.Evaluate"/> adds <c>zOffset * ZAxis</c> to the circular motion in the plane of X,Y).
        /// </description>
        /// </item>
        /// <item>
        /// <description>
        /// At parameter 0, position is <c>center + radius·X</c>. For a bolt thread aligned with world +Z, use a
        /// <see cref="CoordinateSystem"/> whose <b>Z</b> is the helix axis, <b>X</b> is the direction from axis to the
        /// helix start (zero angle), and <b>Y</b> completes a right-handed frame. Hex-head bolt recipes that use
        /// this frame live in GeoEx (<c>MetricHexBolt.CreateHexHeadBolt</c>).
        /// </description>
        /// </item>
        /// </list>
        /// <para>
        /// Mesh vertices are finally transformed from an internal contour space by
        /// <see cref="CoordinateSystem.Default"/> in the extruder call path (profile points are already expressed in world
        /// after cross-section placement).
        /// </para>
        /// </remarks>
        /// <param name="twistRatePerExtrudeDistance">Radians of twist per unit arc length along the tessellated guide.</param>
        [APIDescription(@"ExtrudeAlongCurve(sketch: PlotterSketcherCoordSys, curve: Curve3D, maxDeviation: float = -1, name: str = None, twistRatePerExtrudeDistance: float = 0) -> AnchorMesh
Sweeps a closed 2D profile sketch along a single 3D guide curve.
  sketch: profile in XY of its CoordinateSystem; sketch origin should sit on the curve start.
  curve: guide (Line3D / Arc3D / Circle3D / Helix3D / Spiral3D / ...); created via Add* or curve constructors.
  maxDeviation: chordal tessellation tolerance (world units); -1 uses the GeoAPI instance default.
  twistRatePerExtrudeDistance: radians of twist per unit arc length along the guide (0 = no twist).
  name auto-generated if null.
End caps generated for open guides; closed guides give no caps.")]
        public AnchorMesh ExtrudeAlongCurve(PlotterSketcherCoordSys sketch, Curve3D curve, double maxDeviation = -1, string name = null, double twistRatePerExtrudeDistance = 0)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? GenerateName("ExtrudeAlongCurve");
            List<List<List<Vec2D>>> contour = sketch.Tessellate(maxDeviation, out var contourN, out var names, out var metaData2D, maxDeviation);

            List<CurveVertex3D> curvePoints3D = curve.Tessellate(maxDeviation);
            
            // Orient the curve vertex frames to match the profile's coordinate system
            var s = new List<List<CurveVertex3D>> { curvePoints3D };
            var curveSegments = OrientCurveFramesToProfile(s, sketch.CoordinateSystem, out Vec3D sweepStartTangent);

            var output = new MeshOutput();
            int numGroups = Extruder.GenerateExtrudeAlongCurve(converter, CoordinateSystem.Default, contour, contourN,
                curveSegments, sketch.CoordinateSystem, sweepStartTangent,
                output.Triangles, output.Vertices, output.Normals,
                output.UVs, output.TriangleGroups, output.PrecisePositions, names, name, out var triangleGroupToName,
                twistRatePerExtrudeDistance: twistRatePerExtrudeDistance, maxDeviation: maxDeviation, baseGroupIndex: GetBaseGroupIndex());

            IncrementBaseGroupIndex(numGroups);

            MeshNormalUV mesh = new MeshNormalUV(converter, output.Vertices, output.Normals, output.UVs, output.Triangles, output.TriangleGroups, output.PrecisePositions);

            bool hasEndCaps = triangleGroupToName.Values.Any(n => n == EntityNaming.ExtrudeTop(name));
            Dictionary<string, SurfaceMetaData> surfaceMetaData;
            if (curve is Line3D lineGuide && Math.Abs(twistRatePerExtrudeDistance) < 1e-15)
            {
                surfaceMetaData = NurbsPatchMetadataBuilder.BuildSweepMetadata(
                    metaData2D, sketch.CoordinateSystem, lineGuide.Start, lineGuide.End, name, hasEndCaps, contour, names);
            }
            else
            {
                var flatFrames = curveSegments.SelectMany(s => s).ToList();
                surfaceMetaData = NurbsPatchMetadataBuilder.BuildSweepMetadata(
                    metaData2D, sketch.CoordinateSystem, flatFrames, contour, names, name, hasEndCaps, twistRatePerExtrudeDistance);
            }

            var result = new AnchorMesh(name, mesh, triangleGroupToName, surfaceMetaData);
            RegisterMesh(result);
            return result;
        }

        /// <summary>
        /// Like <see cref="ExtrudeAlongCurve"/> but with a connected strip of guide curves; same profile / frame rules apply.
        /// </summary>
        /// <param name="twistRatePerExtrudeDistance">Radians of twist per unit arc length along the flattened guide polyline.</param>
        [APIDescription(@"ExtrudeAlongCurveStrip(sketch: PlotterSketcherCoordSys, guideCurveStrip: List[Curve3D], maxDeviation: float = -1, twistRatePerExtrudeDistance: float = 0, name: str = None) -> AnchorMesh
Like ExtrudeAlongCurve but the guide is a connected strip of curves. The strip is auto-ordered/auto-reversed via endpoint matching (tolerance 1e-6); throws if the curves cannot form a single connected strip.
  maxDeviation: -1 uses the GeoAPI instance default.
Each per-segment surface patch is named ""<meshName>-<contourSegmentName>-<guideCurveName>"".")]
        public AnchorMesh ExtrudeAlongCurveStrip(PlotterSketcherCoordSys sketch, List<Curve3D> guideCurveStrip, double maxDeviation = -1, double twistRatePerExtrudeDistance = 0, string name = null)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? GenerateName("ExtrudeAlongCurve");
            if (guideCurveStrip == null || guideCurveStrip.Count == 0)
                throw new ArgumentException("guideCurveStrip must contain at least one curve");

            // Connect and order the guide curves to form a proper strip
            List<Curve3D> orderedCurves = ConnectAndOrderCurves(guideCurveStrip);

            List<List<List<Vec2D>>> contour = sketch.Tessellate(maxDeviation, out var contourN, out var names, out var metaData2D, maxDeviation);

            // Tessellate each guide curve and collect names
            List<List<CurveVertex3D>> s = new List<List<CurveVertex3D>>();
            List<string> guideCurveNames = new List<string>();
            
            for (int i = 0; i < orderedCurves.Count; i++)
            {
                var curve = orderedCurves[i];
                s.Add(curve.Tessellate(maxDeviation));
                guideCurveNames.Add(curve.Name ?? $"GuideCurve{i}");
            }

            // Orient the curve vertex frames to match the profile's coordinate system
            // This ensures the profile is placed correctly at each point along the guide
            var curveSegments = OrientCurveFramesToProfile(s, sketch.CoordinateSystem, out Vec3D sweepStartTangent);

            var output = new MeshOutput();
            int numGroups = Extruder.GenerateExtrudeAlongCurveStripSectioned(converter, CoordinateSystem.Default, contour, contourN,
                curveSegments, guideCurveNames, sketch.CoordinateSystem, sweepStartTangent,
                output.Triangles, output.Vertices, output.Normals,
                output.UVs, output.TriangleGroups, output.PrecisePositions, names, name, out var triangleGroupToName,
                twistRatePerExtrudeDistance: twistRatePerExtrudeDistance, maxDeviation: maxDeviation, baseGroupIndex: GetBaseGroupIndex());

            IncrementBaseGroupIndex(numGroups);

            MeshNormalUV mesh = new MeshNormalUV(converter, output.Vertices, output.Normals, output.UVs, output.Triangles, output.TriangleGroups, output.PrecisePositions);

            bool hasEndCaps = triangleGroupToName.Values.Any(n => n == EntityNaming.ExtrudeTop(name));
            var surfaceMetaData = NurbsPatchMetadataBuilder.BuildSweepStripMetadata(
                metaData2D, sketch.CoordinateSystem, curveSegments, guideCurveNames, contour, names, name, hasEndCaps,
                twistRatePerExtrudeDistance);

            var result = new AnchorMesh(name, mesh, triangleGroupToName, surfaceMetaData);
            RegisterMesh(result);
            return result;
        }

        /// <summary>
        /// Extrudes a profile sketch along a guide sketch (curve strip).
        /// The guide sketch's 2D curves are converted to 3D curves using its coordinate system.
        /// </summary>
        /// <param name="profileSketch">The 2D profile to extrude</param>
        /// <param name="guideSketch">The sketch containing the guide curve(s) - curves are converted to 3D using the sketch's coordinate system</param>
        /// <param name="maxDeviation">Maximum deviation for tessellation</param>
        /// <param name="name">Optional name for the resulting mesh (auto-generated if not provided)</param>
        /// <returns>The resulting AnchorMesh</returns>
        [APIDescription(@"ExtrudeAlongSketch(profileSketch: PlotterSketcherCoordSys, guideSketch: PlotterSketcherCoordSys, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Extrudes profileSketch along the curves of guideSketch (lifted to 3D using the guideSketch's coord system). Guide sketch must contain a single connected strip of non-helper curves (Line2D/Arc2D/Circle2D supported); helper geometry is excluded. Throws on empty/multiple guide strips.
  maxDeviation: -1 uses the GeoAPI instance default.")]
        public AnchorMesh ExtrudeAlongSketch(PlotterSketcherCoordSys profileSketch, PlotterSketcherCoordSys guideSketch, double maxDeviation = -1, string name = null)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            // Convert 2D curves from the guide sketch to 3D curves
            List<Curve3D> guideCurves = ConvertSketchCurvesTo3D(guideSketch);
            
            if (guideCurves.Count == 0)
                throw new ArgumentException("Guide sketch must contain at least one curve");
            
            return ExtrudeAlongCurveStrip(profileSketch, guideCurves, maxDeviation, name: name);
        }

        /// <summary>
        /// Converts all 2D curves from a sketch to 3D curves using the sketch's coordinate system.
        /// Excludes helper geometry and enforces a single curve strip.
        /// </summary>
        private List<Curve3D> ConvertSketchCurvesTo3D(PlotterSketcherCoordSys sketch)
        {
            List<Curve3D> result = new List<Curve3D>();
            CoordinateSystem cs = sketch.CoordinateSystem;
            
            // Get curves and filter out helper geometry, enforce single strip
            List<Curve2D> curves = GetSingleCurveStripExcludingHelpers(sketch.GetCurves());
            
            foreach (var curve2D in curves)
            {
                Curve3D curve3D = ConvertCurve2DTo3D(curve2D, cs);
                if (curve3D != null)
                {
                    result.Add(curve3D);
                }
            }
            
            return result;
        }

        /// <summary>
        /// Extracts a single curve strip from all curves, excluding helper geometry.
        /// Throws if no curves exist or if multiple non-empty curve strips are present.
        /// </summary>
        private List<Curve2D> GetSingleCurveStripExcludingHelpers(List<List<Curve2D>> allCurves)
        {
            List<List<Curve2D>> nonEmptyStrips = new List<List<Curve2D>>();
            
            foreach (var strip in allCurves)
            {
                List<Curve2D> filteredCurves = new List<Curve2D>();
                foreach (var curve in strip)
                {
                    if (!curve.IsHelperGeometry)
                        filteredCurves.Add(curve);
                }
                if (filteredCurves.Count > 0)
                    nonEmptyStrips.Add(filteredCurves);
            }

            if (nonEmptyStrips.Count == 0)
                throw new ArgumentException("Guide sketch contains no curves (excluding helper geometry)");
            
            if (nonEmptyStrips.Count > 1)
                throw new ArgumentException($"Guide sketch contains {nonEmptyStrips.Count} curve strips, but only one is allowed");
            
            return nonEmptyStrips[0];
        }

        /// <summary>
        /// Transforms 2D points to 3D using the coordinate system.
        /// </summary>
        private List<Vec3D> TransformPoints2DTo3D(List<Vec2D> points2D, CoordinateSystem cs)
        {
            List<Vec3D> result = new List<Vec3D>(points2D.Count);
            foreach (var p in points2D)
            {
                result.Add(cs.PointFromCoordSysToWorld(new Vec3D(p.X, p.Y, 0)));
            }
            return result;
        }

        /// <summary>
        /// Converts a single 2D curve to its 3D equivalent using the given coordinate system.
        /// Uses ToPoints() on the 2D curve, transforms points to 3D, then calls FromPoints() on the 3D curve.
        /// </summary>
        private Curve3D ConvertCurve2DTo3D(Curve2D curve2D, CoordinateSystem cs)
        {
            if (curve2D is Line2D line)
            {
                List<Vec3D> points3D = TransformPoints2DTo3D(line.ToPoints(), cs);
                return Line3D.FromPoints(points3D, line.Name);
            }
            else if (curve2D is Arc2D arc)
            {
                List<Vec3D> points3D = TransformPoints2DTo3D(arc.ToPoints(), cs);
                return Arc3D.FromPoints(points3D, arc.Name);
            }
            else if (curve2D is Circle2D circle)
            {
                List<Vec3D> points3D = TransformPoints2DTo3D(circle.ToPoints(), cs);
                return Circle3D.FromPoints(points3D, circle.Name);
            }
            else if (curve2D is OffsetSketchStrip2D offsetStrip)
            {
                var tess = offsetStrip.Tessellate(0.01);
                if (tess.Count < 2)
                    return null;
                var points3D = new List<Vec3D>(tess.Count);
                foreach (var v in tess)
                    points3D.Add(cs.PointFromCoordSysToWorld(new Vec3D(v.Position.X, v.Position.Y, 0)));
                return new CubicHermiteSpline3D(points3D, name: offsetStrip.Name);
            }
            
            // For unsupported curve types, return null (caller should handle)
            return null;
        }

        /// <summary>
        /// Reverses a Curve3D by getting its defining points, reversing them, and creating a new curve.
        /// </summary>
        private Curve3D ReverseCurve3D(Curve3D curve)
        {
            if (curve is Line3D line)
            {
                var points = line.ToPoints();
                points.Reverse();
                return Line3D.FromPoints(points, line.Name);
            }
            else if (curve is Arc3D arc)
            {
                var points = arc.ToPoints();
                points.Reverse();
                return Arc3D.FromPoints(points, arc.Name);
            }
            else if (curve is Circle3D circle)
            {
                // Circles have no direction, return as-is
                return circle;
            }
            
            throw new ArgumentException($"Cannot reverse curve of type {curve.GetType().Name}");
        }

        /// <summary>
        /// Connects and orders a list of curves into a proper connected strip.
        /// Uses SegmentConnector to find the correct order and orientation.
        /// </summary>
        /// <param name="curves">The curves to connect (will not be modified)</param>
        /// <param name="tolerance">Distance tolerance for considering endpoints as connected</param>
        /// <returns>A new list of curves in proper order, with reversed curves where needed</returns>
        private List<Curve3D> ConnectAndOrderCurves(List<Curve3D> curves, double tolerance = 1e-6)
        {
            if (curves == null || curves.Count == 0)
                return new List<Curve3D>();
            
            if (curves.Count == 1)
                return new List<Curve3D> { curves[0] };
            
            double toleranceSq = tolerance * tolerance;
            
            // Use SegmentConnector to find the correct order and orientation
            // It returns indices where negative values indicate the curve should be reversed
            var connectedIndices = SegmentConnector.Connect(
                curves,
                c => c.Start,  // getStart
                c => c.End,    // getEnd
                (a, b) => (a - b).LengthSquared() <= toleranceSq,  // areEqual
                out List<bool> closed);
            
            if (connectedIndices.Count == 0)
                throw new ArgumentException("Guide curves could not be connected");
            
            if (connectedIndices.Count > 1)
                throw new ArgumentException("Guide curves do not form a single connected strip");
            
            // Build the result list from the connected indices
            var indices = connectedIndices[0];
            List<Curve3D> result = new List<Curve3D>(indices.Count);
            
            foreach (int signedIndex in indices)
            {
                int index = Math.Abs(signedIndex);
                Curve3D curve = curves[index];
                
                if (signedIndex < 0)
                {
                    // Negative index means the curve should be reversed
                    result.Add(ReverseCurve3D(curve));
                }
                else
                {
                    result.Add(curve);
                }
            }
            
            return result;
        }

        private List<List<CurveVertex3D>> CyclicRotate(List<List<CurveVertex3D>> curveSegments, CoordinateSystem profileCS)
        {
            var lastSeg = curveSegments[curveSegments.Count - 1];
            bool isClosedLoop = (curveSegments[0][0].Origin - lastSeg[lastSeg.Count - 1].Origin).LengthSquared() < 1e-12;

            //Either search the start or end of the curve segment that is closest to profile CS and then cyclically rotate
            double minDistSquared = double.MaxValue;
            int id = -1;
            for (int i = 0; i < curveSegments.Count; i++)
            {
                var seg = curveSegments[i];
                var s = seg[0].Origin;
                var e = seg[1].Origin;

                var d = (s - profileCS.Origin).LengthSquared();
                if (d < minDistSquared)
                {
                    minDistSquared = d;
                    id = i;
                }
                d = (e - profileCS.Origin).LengthSquared();
                if (d < minDistSquared)
                {
                    minDistSquared = d;
                    id = i + 1;
                }
            }

            if (isClosedLoop)
            {
                List<List<CurveVertex3D>> res = new List<List<CurveVertex3D>>(curveSegments.Count);
                for (int i = 0; i < curveSegments.Count; i++)
                {
                    var seg = curveSegments[(i + id) % curveSegments.Count];
                    res.Add(seg);
                }
                return res;
            }


            return curveSegments;
        }


        private List<List<CoordinateSystem>> OrientCurveFramesToProfile(List<List<CurveVertex3D>> curveSegments, CoordinateSystem profileCS, out Vec3D sweepStartTangentWorld)
        {
            curveSegments = CyclicRotate(curveSegments, profileCS);
            sweepStartTangentWorld = curveSegments[0][0].Tangent;

            List<List<CoordinateSystem>> result = new List<List<CoordinateSystem>>(curveSegments.Count);

            // Compute relative orientation from first curve frame to profile CS (orientation only)
            var firstCurveFrame = curveSegments[0][0].GetCoordinateSystem();
            CoordinateSystem localOffsetTransform = firstCurveFrame.ToLocal(profileCS);

            // Accumulated correction for multi-segment curves (starts as identity)
            CoordinateSystem localCorrectionTransform = CoordinateSystem.Default;
          
            for (int i = 0; i < curveSegments.Count; i++)
            {
                var c = curveSegments[i];
                List<CoordinateSystem> r = new List<CoordinateSystem>(c.Count);
                result.Add(r);


                if (i > 0)
                {
                    // Get the original end frame of previous segment and first frame of current segment
                    var prevCurve = curveSegments[i - 1];
                    var curve = curveSegments[i];
                    var endFrame = prevCurve[prevCurve.Count - 1].GetCoordinateSystem();
                    var firstFrame = curve[0].GetCoordinateSystem();

                    var endFrameCorrected = endFrame.ToGlobal(localCorrectionTransform);
                    localCorrectionTransform = firstFrame.ToLocal(endFrameCorrected);

                    var debug = firstFrame.ToGlobal(localCorrectionTransform);
                }

                for (int j = 0; j < c.Count; j++)
                {
                    var cs = c[j];
                    var intermediate = cs.GetCoordinateSystem().ToGlobal(localCorrectionTransform);
                    var final = intermediate.ToGlobal(localOffsetTransform);
                    r.Add(final);
                }
            }

            return result;
        }

        [APIDescription(@"Extrude(sketch: PlotterSketcherCoordSys, height: float, maxDeviation: float = -1, name: str = None, twistRatePerExtrudeDistance: float = 0) -> AnchorMesh
Linear extrusion of the sketch along the sketch's local +Z (the sketch plane normal).
  height: extrusion distance along +Z (single direction; use ExtrudeTwoSides for double-sided).
  maxDeviation: chordal tessellation tolerance for curved sketch geometry (world units); -1 uses the GeoAPI instance default.
  twistRatePerExtrudeDistance: radians of twist per unit Z (0 = no twist).
Profile must be a closed contour (helper geometry excluded). Patches are named ""<name>-top"", ""<name>-bottom"", and ""<name>-<contourSegmentName>"" for the side walls.")]
        public AnchorMesh Extrude(PlotterSketcherCoordSys sketch, double height, double maxDeviation = -1, string name = null, double twistRatePerExtrudeDistance = 0)
        {
            return ExtrudeTwoSides(sketch, height, 0, maxDeviation, name, twistRatePerExtrudeDistance);
        }

        /// <summary>
        /// Extrudes a sketch in both directions along the sketch normal.
        /// </summary>
        /// <param name="sketch">The sketch to extrude</param>
        /// <param name="heightPositive">Extrusion distance in the positive normal direction</param>
        /// <param name="heightNegative">Extrusion distance in the negative normal direction</param>
        /// <param name="maxDeviation">Maximum deviation for tessellation</param>
        /// <param name="name">Optional name for the resulting mesh (auto-generated if not provided)</param>
        /// <param name="twistRatePerExtrudeDistance">Radians of twist per unit distance along local +Z (see <see cref="Extruder.GenerateExtrudedMesh"/>).</param>
        /// <returns>The extruded mesh</returns>
        [APIDescription(@"ExtrudeTwoSides(sketch: PlotterSketcherCoordSys, heightPositive: float, heightNegative: float, maxDeviation: float = -1, name: str = None, twistRatePerExtrudeDistance: float = 0) -> AnchorMesh
Like Extrude but extrudes in both directions along the sketch normal.
  heightPositive: distance along +Z; heightNegative: distance along -Z. Use 0 on one side to get a one-sided extrusion.
  maxDeviation: -1 uses the GeoAPI instance default.
Other parameters identical to Extrude.")]
        public AnchorMesh ExtrudeTwoSides(PlotterSketcherCoordSys sketch, double heightPositive, double heightNegative, double maxDeviation = -1, string name = null, double twistRatePerExtrudeDistance = 0)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? GenerateName("Extrude");
            List<List<List<Vec2D>>> contour = sketch.Tessellate(maxDeviation, out var contourN, out var names, out var metaData2D, maxDeviation, SketchTessellationFlags.ExcludeHelperGeometry);

            var output = new MeshOutput();
            var naming = new MeshNaming { ContourNames = names, OperationName = name };
            int numGroups = Extruder.GenerateExtrudedMesh(converter, sketch.CoordinateSystem, contour, contourN,
                heightPositive, heightNegative, output, naming,
                twistRatePerExtrudeDistance: twistRatePerExtrudeDistance, maxDeviation: maxDeviation, baseGroupIndex: GetBaseGroupIndex());
            var triangleGroupToName = naming.TriangleGroupToName;

            IncrementBaseGroupIndex(numGroups);

            MeshNormalUV mesh = new MeshNormalUV(converter, output.Vertices, output.Normals, output.UVs, output.Triangles, output.TriangleGroups, output.PrecisePositions);

            var profilePoints = NurbsPatchMetadataBuilder.CollectProfilePoints(contour);
            var surfaceMetaData = NurbsPatchMetadataBuilder.BuildExtrudeMetadata(
                metaData2D, sketch.CoordinateSystem, heightPositive, heightNegative, name,
                twistRatePerExtrudeDistance, profilePoints, contour, names);

            var result = new AnchorMesh(name, mesh, triangleGroupToName, surfaceMetaData);
            RegisterMesh(result);
            //nameOfMostRecentMesh = name;
            return result;
        }

        [APIDescription(@"ProjectSketchOntoMesh(sketch: PlotterSketcherCoordSys, target: AnchorMesh, maxDeviation: float = -1, name: str = None) -> ProjectedSketch
Tessellates the sketch (keeping per-curve names), lifts each vertex into world space, and casts a ray along the sketch-plane +Z onto `target`.
Each hit stores the triangle geometric normal as the curve-frame Up direction. Returns a ProjectedSketch with named PolylineCurve3D curves.
Throws if any sketch vertex misses the mesh (message includes the curve name).")]
        public ProjectedSketch ProjectSketchOntoMesh(
            PlotterSketcherCoordSys sketch,
            AnchorMesh target,
            double maxDeviation = -1,
            string name = null)
        {
            if (sketch == null)
                throw new ArgumentNullException(nameof(sketch));
            if (target == null)
                throw new ArgumentNullException(nameof(target));

            target.EnsureCoplanarPostProcessed();
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? GenerateName("ProjectedSketch");

            List<List<List<Vec2D>>> contours = sketch.Tessellate(
                maxDeviation, out var contourNormals, out var names, out _,
                maxDeviation, SketchTessellationFlags.ExcludeHelperGeometry);

            if (contours == null || contours.Count == 0)
                throw new ArgumentException("Sketch has no geometry to project.");

            Vec3D rayDir = sketch.CoordinateSystem.Z;
            var projectedVerts = new List<List<List<ProjectedVertex>>>(contours.Count);
            var curves = new List<PolylineCurve3D>();

            for (int stripIndex = 0; stripIndex < contours.Count; stripIndex++)
            {
                var strip = contours[stripIndex];
                var stripNames = names[stripIndex];
                var hitStrip = new List<List<ProjectedVertex>>(strip.Count);

                for (int segIndex = 0; segIndex < strip.Count; segIndex++)
                {
                    var seg = strip[segIndex];
                    string curveName = stripNames[segIndex];
                    var hitSeg = new List<ProjectedVertex>(seg.Count);

                    for (int i = 0; i < seg.Count; i++)
                    {
                        Vec2D uv = seg[i];
                        Vec3D origin = sketch.CoordinateSystem.PointFromCoordSysToWorld(new Vec3D(uv.X, uv.Y, 0));
                        if (!RayMeshExact.TryCast(target, converter, origin, rayDir, out RayMeshHit hit))
                        {
                            throw new InvalidOperationException(
                                $"ProjectSketchOntoMesh '{name}': ray from curve '{curveName}' (vertex {i}) missed mesh '{target.Name}'.");
                        }

                        hitSeg.Add(new ProjectedVertex
                        {
                            SketchUv = uv,
                            Hit = hit.Point,
                            SurfaceNormal = hit.GeometricNormal,
                            TriangleIndex = hit.TriangleIndex,
                            GroupId = hit.GroupId
                        });
                    }

                    // Snap shared endpoints within a strip for numerical consistency.
                    if (segIndex > 0 && hitStrip.Count > 0)
                    {
                        var prev = hitStrip[segIndex - 1];
                        if (prev.Count > 0 && hitSeg.Count > 0)
                            hitSeg[0] = prev[prev.Count - 1];
                    }

                    hitStrip.Add(hitSeg);

                    PolylineCurve3D poly = BuildProjectedPolyline(hitSeg, EntityNaming.ExtrudeSide(name, curveName));
                    curves.Add(poly);
                    curves3D.Add(poly);
                }

                // Closed strip: snap last endpoint of last segment to first of first.
                if (hitStrip.Count > 0 && hitStrip[0].Count > 0)
                {
                    var first = hitStrip[0][0];
                    var lastSeg = hitStrip[hitStrip.Count - 1];
                    if (lastSeg.Count > 0)
                    {
                        Vec2D firstUv = strip[0][0];
                        Vec2D lastUv = strip[strip.Count - 1][strip[strip.Count - 1].Count - 1];
                        if (Vec2DOps.DistanceSquared(firstUv, lastUv) < 1e-20)
                            lastSeg[lastSeg.Count - 1] = first;
                    }
                }

                projectedVerts.Add(hitStrip);
            }

            return new ProjectedSketch
            {
                Name = name,
                SourceSketch = sketch,
                TargetMesh = target,
                Curves = curves,
                Contours2D = contours,
                ContourNormals2D = contourNormals,
                ContourNames = names,
                Vertices = projectedVerts
            };
        }

        private static PolylineCurve3D BuildProjectedPolyline(List<ProjectedVertex> hits, string curveName)
        {
            var verts = new List<CurveVertex3D>(hits.Count);
            for (int i = 0; i < hits.Count; i++)
            {
                Vec3D origin = hits[i].Hit;
                Vec3D up = hits[i].SurfaceNormal.Normalized();
                Vec3D tangent;
                if (i + 1 < hits.Count)
                    tangent = (hits[i + 1].Hit - origin);
                else if (i > 0)
                    tangent = (origin - hits[i - 1].Hit);
                else
                    tangent = Vec3DOps.Cross(up, Math.Abs(up.Z) < 0.9 ? new Vec3D(0, 0, 1) : new Vec3D(1, 0, 0));

                if (tangent.LengthSquared() < 1e-30)
                    tangent = Vec3DOps.Cross(up, Math.Abs(up.Z) < 0.9 ? new Vec3D(0, 0, 1) : new Vec3D(1, 0, 0));
                tangent = tangent.Normalized();

                // Keep Up perpendicular to Tangent for a valid frame.
                Vec3D upAdj = (up - Vec3DOps.Dot(up, tangent) * tangent);
                if (upAdj.LengthSquared() < 1e-20)
                    upAdj = up;
                else
                    upAdj = upAdj.Normalized();

                double uniform = hits.Count <= 1 ? 0 : (double)i / (hits.Count - 1);
                verts.Add(new CurveVertex3D(origin, tangent, upAdj, uniform));
            }
            return new PolylineCurve3D(verts, curveName);
        }

        [APIDescription(@"ExtrudeProjectedSketch(projected: ProjectedSketch, height: float, name: str = None) -> AnchorMesh
Extrudes a closed ProjectedSketch along the per-vertex surface normals stored at projection time.
  height: distance along each hit normal (negative = opposite / into the solid for a pocket).
Caps are triangulated in the original sketch 2D domain and mapped onto the hit and offset rings.
Requires closed outer contour(s); holes are supported.")]
        public AnchorMesh ExtrudeProjectedSketch(ProjectedSketch projected, double height, string name = null)
        {
            if (projected == null)
                throw new ArgumentNullException(nameof(projected));

            name = name ?? GenerateName("ProjectedExtrude");
            var output = new MeshOutput();
            var naming = new MeshNaming { OperationName = name };
            int numGroups = NormalExtruder.Generate(
                converter, projected, height, output, naming, GetBaseGroupIndex());
            IncrementBaseGroupIndex(numGroups);

            MeshNormalUV mesh = new MeshNormalUV(
                converter, output.Vertices, output.Normals, output.UVs,
                output.Triangles, output.TriangleGroups, output.PrecisePositions);

            // Minimal Unknown metadata keyed by patch name (analytic params optional).
            var surfaceMetaData = new Dictionary<string, SurfaceMetaData>();
            foreach (var kv in naming.TriangleGroupToName)
            {
                if (!surfaceMetaData.ContainsKey(kv.Value))
                    surfaceMetaData[kv.Value] = new SurfaceMetaData(SurfaceType.Unknown);
            }

            var result = new AnchorMesh(name, mesh, naming.TriangleGroupToName, surfaceMetaData);
            RegisterMesh(result);
            return result;
        }

        /// <summary>
        /// Lofts ordered sketches into a mesh (see <see cref="LoftBuilder"/>). One connected curve strip per sketch.
        /// </summary>
        [APIDescription(@"Loft(sketches: IReadOnlyList[PlotterSketcherCoordSys], options: LoftOptions, name: str = None, maxDeviation: float = -1) -> AnchorMesh
Lofts an ordered list of profile sketches into a mesh. Each sketch contains a single connected curve strip (no holes). Profiles correspond by normalized arc-length parameter u in [0,1] after choosing a seam (u=0) on each closed profile.
  sketches: >= 2 ordered profiles (use System.Collections.Generic.List[PlotterSketcherCoordSys] in Python).
  maxDeviation: chordal tessellation tolerance for both u-direction (along profiles) and v-direction (across profiles for Hermite/CatmullRom); -1 uses the GeoAPI instance default.
  options: LoftOptions (class). Important fields:
    Style: LoftStyle.Ruled | SmoothCatmullRom | Hermite (default).
    ProfileSamplesU: minimum uniform u samples merged with profile anchors (default 32).
    VSubdivisionsPerSpan: minimum extra v samples per span (default 2).
    CapEnds: bool, end caps for closed profiles (default True).
    AllowOpenContour: bool, allow open profile strips.
    CapTriangulation: LoftCapTriangulationMode.Robust (default) | EarClipping (strict winding preflight only).
    AlignmentMode: AsAuthored (u=0 = sketch strip start) | OriginFootRoll (default: u=0 = closest point to sketch origin) | MinimumTwist (continuous alignment to previous closed profile).
    ProfileSeamPoints: optional list of LoftProfileSeamHint (one per profile); entries with HasPoint override AlignmentMode for that closed profile.
    CorrespondenceMode / CreasePolicy: defaults MergedArcLengthAnchors + FromAllProfiles. For NACA propeller blades use LoftOptions.PropellerBlade (tessellation-driven u, CreasePolicy.None). UniformUOnly is a fast preview mode that ignores tessellation anchors.
    ProfileSampling: see enum docs; defaults via LoftOptions.Default.
  Open profiles always keep authored start/end (seam u0=0). name auto-generated if null.")]
        public AnchorMesh Loft(IReadOnlyList<PlotterSketcherCoordSys> sketches, LoftOptions options, string name = null, double maxDeviation = -1)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? GenerateName("Loft");
            var output = new MeshOutput();
            int numGroups = LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation, options, output, name, out var triangleGroupToName, GetBaseGroupIndex());
            IncrementBaseGroupIndex(numGroups);

            var mesh = new MeshNormalUV(converter, output.Vertices, output.Normals, output.UVs, output.Triangles, output.TriangleGroups, output.PrecisePositions);
            var surfaceMetaData = NurbsPatchMetadataBuilder.BuildLoftMetadata(sketches, maxDeviation, options, name);

            var result = new AnchorMesh(name, mesh, triangleGroupToName, surfaceMetaData);
            RegisterMesh(result);
            return result;
        }

        [APIDescription(@"CreateCube(lowerLeftCornerPose: CoordinateSystem, extent: float, name: str = None) -> AnchorMesh
Axis-aligned cube of side `extent`, with its (0,0,0) corner placed at lowerLeftCornerPose.Origin and edges along the pose's X/Y/Z axes. name auto-generated if null.")]
        public AnchorMesh CreateCube(CoordinateSystem lowerLeftCornerPose, double extent, string name = null)
        {
            return CreateCuboid(lowerLeftCornerPose, new Vec3D(extent), name);
        }

        [APIDescription(@"CreateCuboid(aabbMin: Vec3D, aabbMax: Vec3D, name: str = None) -> AnchorMesh
World-axis-aligned box from aabbMin to aabbMax (componentwise min/max).")]
        public AnchorMesh CreateCuboid(Vec3D aabbMin, Vec3D aabbMax, string name = null)
        {
            Vec3D extents = new Vec3D(aabbMax.X - aabbMin.X, aabbMax.Y - aabbMin.Y, aabbMax.Z - aabbMin.Z);
            return CreateCuboid(new CoordinateSystem(aabbMin), extents, name);
        }

        [APIDescription(@"CreateCuboid(lowerLeftCornerPose: CoordinateSystem, extents: Vec3D, name: str = None) -> AnchorMesh
Box with side lengths (extents.X, extents.Y, extents.Z) along the pose's local X/Y/Z axes, with corner (0,0,0) at pose.Origin.")]
        public AnchorMesh CreateCuboid(CoordinateSystem lowerLeftCornerPose, Vec3D extents, string name = null)
        {
            name = name ?? GenerateName("Cuboid");
            PlotterSketcherCoordSys sketch = new PlotterSketcherCoordSys("GroundRectangle", lowerLeftCornerPose);
            sketch.AddLine(new Vec2D(0, 0), new Vec2D(extents.X, 0));
            sketch.AddLine(new Vec2D(extents.X, 0), new Vec2D(extents.X, extents.Y));
            sketch.AddLine(new Vec2D(extents.X, extents.Y), new Vec2D(0, extents.Y));
            sketch.AddLine(new Vec2D(0, extents.Y), new Vec2D(0, 0));

            return Extrude(sketch, extents.Z, 1, name); //Max deviation does not matter for a cuboid, it's zero anyway

            //var height = extents.Z;

            //// Each segment is a list of two points (start and end), and we need four segments for the rectangle
            //List<List<Vec2D>> contour = new List<List<Vec2D>>
            //{
            //    new List<Vec2D> { new Vec2D(0, 0), new Vec2D(extents.X, 0) },              // bottom
            //    new List<Vec2D> { new Vec2D(extents.X, 0), new Vec2D(extents.X, extents.Y) }, // right
            //    new List<Vec2D> { new Vec2D(extents.X, extents.Y), new Vec2D(0, extents.Y) }, // top
            //    new List<Vec2D> { new Vec2D(0, extents.Y), new Vec2D(0, 0) }               // left
            //};

            //// Each segment's normal (outward from the rectangle in 2D)
            //List<List<Vec2D>> contourN = new List<List<Vec2D>>
            //{
            //    new List<Vec2D> { new Vec2D(0, -1), new Vec2D(0, -1) },   // bottom
            //    new List<Vec2D> { new Vec2D(1, 0), new Vec2D(1, 0) },     // right
            //    new List<Vec2D> { new Vec2D(0, 1), new Vec2D(0, 1) },     // top
            //    new List<Vec2D> { new Vec2D(-1, 0), new Vec2D(-1, 0) }    // left
            //};

            //List<string> names = new List<string> { "bottom", "right", "top", "left" };

            //List<Tri> triangles = new List<Tri>();
            //List<Vec3D> vertices = new List<Vec3D>();
            //List<Vec3D> normals = new List<Vec3D>();
            //List<Vec2D> uv = new List<Vec2D>();
            //List<int> triangleGroups = new List<int>();
            //int numGroups = Extruder.GenerateExtrudedMesh(lowerLeftCorner, new List<List<List<Vec2D>>>() { contour }, new() { contourN },
            //    height, triangles, vertices, normals,
            //    uv, triangleGroups, new () { names }, name, out var triangleGroupToName, GetBaseGroupIndex()); //TODO: Pass group offset int ofunc, should also affect triangleGroups

            //IncrementBaseGroupIndex(numGroups);

            //MeshNormalUV mesh = new MeshNormalUV(converter, vertices, normals, uv, triangles, triangleGroups);

            //var result = new AnchorMesh(name, mesh, /*edges,*/ triangleGroupToName);
            //meshes.Add(result);
            ////nameOfMostRecentMesh = name;
            //return result;
        }

        [APIDescription(@"CreateCylinder(baseCircleCenterPose: CoordinateSystem, radius: float, heightAlongZ: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Cylinder built by extruding a circle of `radius` (sketched in pose's XY plane, centered at pose.Origin) by `heightAlongZ` along the pose's local +Z. maxDeviation: chordal tolerance for circle tessellation; -1 uses the GeoAPI instance default.")]
        public AnchorMesh CreateCylinder(CoordinateSystem baseCircleCenterPose, double radius, double heightAlongZ, double maxDeviation = -1, string name = null)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? GenerateName("Cylinder");
            PlotterSketcherCoordSys sketch = new PlotterSketcherCoordSys("GroundCircle", baseCircleCenterPose);
            sketch.AddCircle(new Vec2D(0, 0), radius);

            return Extrude(sketch, heightAlongZ, maxDeviation, name);
        }
        [APIDescription(@"CreateCylinderRevolve(centerPose: CoordinateSystem, radius: float, height: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Cylinder built by revolving a 3-line profile around the local X-axis (revolve axis). The cylinder is centered on pose.Origin (extends ±height/2 along the pose's local X). Use CreateCylinder for the more common ""extrude a circle"" version.
  maxDeviation: -1 uses the GeoAPI instance default.")]
        public AnchorMesh CreateCylinderRevolve(CoordinateSystem centerPose, double radius, double height, double maxDeviation = -1, string name = null)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? GenerateName("Cylinder");
            double halfH = height / 2.0;
            PlotterSketcherCoordSys sketch = new PlotterSketcherCoordSys("CylinderProfile", centerPose);

            // Open profile: 3 lines forming the cylinder cross-section in the XY plane.
            // Revolving 360° around the X-axis creates top cap, side wall, and bottom cap.
            //
            //  (halfH, 0) ---top cap--- (halfH, radius)
            //                                |
            //                             side wall
            //                                |
            // (-halfH, 0) --bottom cap-- (-halfH, radius)
            //
            sketch.AddLine(new Vec2D(halfH, 0), new Vec2D(halfH, radius));
            sketch.AddLine(new Vec2D(halfH, radius), new Vec2D(-halfH, radius));
            sketch.AddLine(new Vec2D(-halfH, radius), new Vec2D(-halfH, 0));

            return Revolve(sketch, 2 * Math.PI, maxDeviation, name);
        }

        [APIDescription(@"CreateSphere(centerPose: CoordinateSystem, radius: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Sphere centered at centerPose.Origin built by revolving a semicircle 360° around the pose's local X-axis. maxDeviation: chordal tolerance; -1 uses the GeoAPI instance default.")]
        public AnchorMesh CreateSphere(CoordinateSystem centerPose, double radius, double maxDeviation = -1, string name = null)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? GenerateName("Sphere");
            PlotterSketcherCoordSys sketch = new PlotterSketcherCoordSys("SphereArc", centerPose);
            
            // Create a semicircle arc from top (+radius, 0) through equator (0, radius) to bottom (-radius, 0)
            // When revolved 360° around the X-axis, this creates a sphere
            Vec2D start = new Vec2D(radius, 0);       // Top of sphere (on positive X-axis)
            Vec2D pointOnArc = new Vec2D(0, radius);  // Equator point (maximum radius from axis)
            Vec2D end = new Vec2D(-radius, 0);        // Bottom of sphere (on negative X-axis)
            
            sketch.AddArc(start, pointOnArc, end);

            return Revolve(sketch, 2 * Math.PI, maxDeviation, name);
        }


        //public AnchorMesh Extrude(string name, CoordinateSystem coordSys, List<Vec2D> contour, double angleThresholdRadian, double height)
        //{
        //    List<Tri> triangles = new List<Tri>();
        //    List<Vec3D> vertices = new List<Vec3D>();
        //    List<Vec3D> normals = new List<Vec3D>();
        //    List<Vec2D> uv = new List<Vec2D>();
        //    List<int> triangleGroups = new List<int>();
        //    int baseGroupIndex = GetBaseGroupIndex();
        //    int numGroups = Extruder.GenerateExtrudedMesh(coordSys, contour, angleThresholdRadian, height, triangles, vertices, normals, uv, triangleGroups, baseGroupIndex);

        //    IncrementBaseGroupIndex(numGroups);

        //    MeshNormalUV mesh = new MeshNormalUV(converter, vertices, normals, uv, triangles, triangleGroups);

        //    //var edges = GroupEdgeExtractor.ExtractGroupEdges(mesh.Triangles, mesh.GetGroupList(), mesh.Positions);

        //    Dictionary<string, int> nameToGroup = new Dictionary<string, int>();
        //    for (int i = 0; i < numGroups; ++i)
        //        nameToGroup.Add(name + "_" + i.ToString(), baseGroupIndex + i);

        //    var result = new AnchorMesh(name, mesh, /*edges,*/ nameToGroup);
        //    meshes.Add(result);
        //    //nameOfMostRecentMesh = name;
        //    return result;
        //}


        [APIDescription(@"Revolve(sketch: PlotterSketcher, angle: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Revolves a 2D profile around the sketch's local X-axis (so sketch Y is radius, sketch X is along axis).
  angle: revolve angle in radians (use 2*pi for a full revolution).
  maxDeviation: chordal tessellation tolerance; -1 uses the GeoAPI instance default.
Profile rules:
  - Closed profile (start point ≈ end point): full ring section (no caps required for a full revolution).
  - Open profile: first AND last points must lie on Y = 0 (the axis); intermediate points must NOT touch the axis (would create degenerate triangles). Throws on violation. Open profiles cannot include nested holes.
  - Nested holes: additional closed contours in the sketch (via MoveToPointAndStartNewCurveStrip), same layout as Extrude; supported for closed outers only.
Side patches are named ""<meshName>-<contourSegmentName>"". Helper geometry is excluded.")]
        public AnchorMesh Revolve(PlotterSketcher sketch, double angle, double maxDeviation = -1, string name = null)
        {
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? GenerateName("Revolve");
            // Allow open profiles for revolves - valid when start and end lie on the revolution axis
            List<List<List<Vec2D>>> profile = sketch.Tessellate(maxDeviation, out var contourN, out var names, out var metaData2D, maxDeviation, SketchTessellationFlags.AllowOpenContour | SketchTessellationFlags.ExcludeHelperGeometry);

            // Validate open profile geometry for revolves
            // For revolving around X-axis: Y coordinate represents radius from axis
            // Valid cases:
            // 1. Profile is closed (first point == last point)
            // 2. Profile is open with start and end on axis (Y=0), no intermediate points on axis
            // Nested holes require every contour to be closed.
            if (profile.Count > 1)
            {
                foreach (var profileStrip in profile)
                {
                    if (profileStrip.Count == 0) continue;
                    var firstCurve = profileStrip[0];
                    var lastCurve = profileStrip[profileStrip.Count - 1];
                    Vec2D startPoint = firstCurve[0];
                    Vec2D endPoint = lastCurve[lastCurve.Count - 1];
                    bool isClosed = Vec2DOps.DistanceSquared(startPoint, endPoint) < maxDeviation * maxDeviation;
                    if (!isClosed)
                        throw new Exception("Revolve: Nested holes require closed contours. Open profiles cannot include holes.");
                }
            }

            foreach (var profileStrip in profile)
            {
                if (profileStrip.Count == 0) continue;
                
                var firstCurve = profileStrip[0];
                var lastCurve = profileStrip[profileStrip.Count - 1];
                Vec2D startPoint = firstCurve[0];
                Vec2D endPoint = lastCurve[lastCurve.Count - 1];
                
                bool isClosed = Vec2DOps.DistanceSquared(startPoint, endPoint) < maxDeviation * maxDeviation;
                
                if (!isClosed)
                {
                    // For open profiles, start and end must lie on the axis (Y=0)
                    const double axisTolerance = 1e-10;
                    if (Math.Abs(startPoint.Y) > axisTolerance)
                        throw new Exception($"Revolve: Open profile start point must lie on revolution axis (Y=0). Start Y = {startPoint.Y}");
                    if (Math.Abs(endPoint.Y) > axisTolerance)
                        throw new Exception($"Revolve: Open profile end point must lie on revolution axis (Y=0). End Y = {endPoint.Y}");

                    firstCurve[0] = new Vec2D(startPoint.X, 0.0);
                    lastCurve[lastCurve.Count - 1] = new Vec2D(endPoint.X, 0.0);

                    // Check that no intermediate points lie on the axis
                    for (int curveIdx = 0; curveIdx < profileStrip.Count; curveIdx++)
                    {
                        var curve = profileStrip[curveIdx];
                        int startIdx = (curveIdx == 0) ? 1 : 0; // Skip first point of first curve
                        int endIdx = (curveIdx == profileStrip.Count - 1) ? curve.Count - 1 : curve.Count; // Skip last point of last curve
                        
                        for (int i = startIdx; i < endIdx; i++)
                        {
                            if (Math.Abs(curve[i].Y) < axisTolerance)
                                throw new Exception($"Revolve: Intermediate profile point lies on revolution axis (Y=0), which would create degenerate geometry.");
                        }
                    }
                }
            }

            var output = new MeshOutput();

            CoordinateSystem? revolveFrame = sketch is PlotterSketcherCoordSys coordSysSketch
                ? coordSysSketch.CoordinateSystem
                : null;

            int numGroups = Revolver.GenerateRevolvedMesh(converter, profile, contourN, angle,
                output, maxDeviation, names, name, out var triangleGroupToName, revolveFrame, GetBaseGroupIndex());

            IncrementBaseGroupIndex(numGroups);

            MeshNormalUV mesh = new MeshNormalUV(converter, output.Vertices, output.Normals, output.UVs,
                output.Triangles, output.TriangleGroups, output.PrecisePositions);

            var profilePoints = NurbsPatchMetadataBuilder.CollectProfilePoints(profile);
            var sketchFrame = sketch is PlotterSketcherCoordSys coordSketch
                ? coordSketch.CoordinateSystem
                : CoordinateSystem.Default;
            var surfaceMetaData = NurbsPatchMetadataBuilder.BuildRevolveMetadata(
                metaData2D, sketchFrame, angle, name, profilePoints, profile, names);

            var result = new AnchorMesh(name, mesh, triangleGroupToName, surfaceMetaData);
            RegisterMesh(result);
            //nameOfMostRecentMesh = name;
            return result;
        }

        //public AnchorMesh Revolve(string name, List<Vec2D> profile, double angle, double maxError)
        //{            
        //    List<Tri> triangles = new List<Tri>();
        //    List<Vec3D> vertices = new List<Vec3D>();
        //    List<Vec3D> normals = new List<Vec3D>();
        //    List<Vec2D> uv = new List<Vec2D>();
        //    List<int> triangleGroups = new List<int>();
        //    int baseGroupId = GetBaseGroupIndex();
        //    int numGroups = Revolver.GenerateRevolvedMesh(profile, angle, triangles, vertices, normals, uv, triangleGroups, maxError);

        //    IncrementBaseGroupIndex(numGroups);

        //    MeshNormalUV mesh = new MeshNormalUV(converter, vertices, normals, uv, triangles, triangleGroups);

        //    //var edges = GroupEdgeExtractor.ExtractGroupEdges(mesh.Triangles, mesh.GetGroupList(), mesh.Positions);

        //    Dictionary<string, int> nameToGroup = new Dictionary<string, int>();
        //    for (int i = 0; i < numGroups; ++i)
        //        nameToGroup.Add(name + "-" + i.ToString(), baseGroupId + i);

        //    var result = new AnchorMesh(name, mesh, /*edges,*/ nameToGroup);
        //    meshes.Add(result);
        //    //nameOfMostRecentMesh = name;
        //    return result;
        //}

        private readonly object _meshRegistryLock = new object();

        [APIDescription(@"BatchUnion(meshesToUnite: List[AnchorMesh]) -> AnchorMesh
Parallel pairwise (tournament-tree) union of N meshes. Boolean merges run on thread pool threads; mesh registration is synchronized. Returns null for empty input, returns the single input mesh for count==1.")]
        public AnchorMesh BatchUnion(List<AnchorMesh> meshesToUnite)
        {
            if (meshesToUnite.Count == 0)
                return null;
            if (meshesToUnite.Count == 1)
                return meshesToUnite[0];

            Queue<Task<AnchorMesh>> source = new();
            for (int i = 0; i < meshesToUnite.Count; ++i)
                source.Enqueue(Task.FromResult(meshesToUnite[i]));

            Queue<Task<AnchorMesh>> target = new();
            while (source.Count > 1)
            {
                while (source.Count > 0)
                {
                    var first = source.Dequeue();

                    if (source.Count > 0)
                    {
                        var second = source.Dequeue();
                        target.Enqueue(Task.Run(async () =>
                        {
                            var a = await first;
                            var b = await second;
                            try
                            {
                                return Boolean(a, b, BooleanOp.Union);
                            }
                            catch (Exception ex)
                            {
                                throw new InvalidOperationException(
                                    $"BatchUnion failed merging '{a?.Name}' with '{b?.Name}'.",
                                    ex);
                            }
                        }));
                    }
                    else
                    {
                        target.Enqueue(first);
                    }
                }
                Algorithms.Swap(ref source, ref target);
            }

            try
            {
                return source.Dequeue().GetAwaiter().GetResult();
            }
            catch (AggregateException ex)
            {
                throw ex.GetBaseException() ?? ex;
            }
        }

        [APIDescription(@"BatchBooleanChain(meshA: AnchorMesh, opChain: List[BooleanOpChainNode]) -> AnchorMesh
Applies a sequence of boolean ops starting from meshA. Each BooleanOpChainNode has fields: MeshB (AnchorMesh), Operation (BooleanOp.Union | Difference | Intersect — only these three supported, else NotSupportedException).
Consecutive Union or Intersect runs are batched in parallel (associative); Difference is strictly sequential and order-sensitive. Throws on null nodes or null MeshB.")]
        public AnchorMesh BatchBooleanChain(AnchorMesh meshA, List<BooleanOpChainNode> opChain)
        {
            if (meshA == null)
                throw new ArgumentNullException(nameof(meshA));
            if (opChain == null || opChain.Count == 0)
                return meshA;

            AnchorMesh BatchAssociative(AnchorMesh start, List<AnchorMesh> meshes, BooleanOp op)
            {
                if (meshes.Count == 0)
                    return start;
                if (meshes.Count == 1)
                    return Boolean(start, meshes[0], op);

                // Build a BatchUnion-style task tree, but with a fixed op.
                Queue<Task<AnchorMesh>> source = new();
                source.Enqueue(Task.FromResult(start));
                for (int i = 0; i < meshes.Count; i++)
                    source.Enqueue(Task.FromResult(meshes[i]));

                Queue<Task<AnchorMesh>> target = new();
                while (source.Count > 1)
                {
                    while (source.Count > 0)
                    {
                        var first = source.Dequeue();
                        if (source.Count > 0)
                        {
                            var second = source.Dequeue();
                            target.Enqueue(Task.Run(async () =>
                            {
                                var a = await first;
                                var b = await second;
                                return Boolean(a, b, op);
                            }));
                        }
                        else
                        {
                            target.Enqueue(first);
                        }
                    }
                    Algorithms.Swap(ref source, ref target);
                }
                return source.Dequeue().Result;
            }

            AnchorMesh current = meshA;
            int iNode = 0;
            while (iNode < opChain.Count)
            {
                var node = opChain[iNode] ?? throw new ArgumentException($"opChain[{iNode}] is null", nameof(opChain));
                if (node.MeshB == null)
                    throw new ArgumentException($"opChain[{iNode}].MeshB is null", nameof(opChain));

                if (node.Operation != BooleanOp.Union &&
                    node.Operation != BooleanOp.Difference &&
                    node.Operation != BooleanOp.Intersect)
                {
                    throw new NotSupportedException($"BatchBooleanChain only supports Union/Difference/Intersect, got {node.Operation} at index {iNode}.");
                }

                // For associative ops we can batch consecutive runs with a task tree.
                if (node.Operation == BooleanOp.Union || node.Operation == BooleanOp.Intersect)
                {
                    BooleanOp runOp = node.Operation;
                    var runMeshes = new List<AnchorMesh>();
                    while (iNode < opChain.Count)
                    {
                        var n = opChain[iNode] ?? throw new ArgumentException($"opChain[{iNode}] is null", nameof(opChain));
                        if (n.MeshB == null)
                            throw new ArgumentException($"opChain[{iNode}].MeshB is null", nameof(opChain));
                        if (n.Operation != runOp)
                            break;
                        runMeshes.Add(n.MeshB);
                        iNode++;
                    }

                    current = BatchAssociative(current, runMeshes, runOp);
                    continue;
                }

                // Difference remains strictly sequential and order-sensitive.
                current = Boolean(current, node.MeshB, node.Operation);
                iNode++;
            }

            return current;
        }

        [APIDescription(@"Boolean(meshA: AnchorMesh, meshB: AnchorMesh, operation: BooleanOp, name: str = None) -> AnchorMesh
CSG boolean between two meshes.
  operation: BooleanOp.Union | Difference (A minus B) | Intersect | Resolve | NoOpIntersectionContourOnly | AAsSurfaceBAsTrimSurfaceKeepInTriNormalDirection | AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection | AAsVolumeBAsTrimSurfaceKeepInTriNormalDirection | AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection | AAsSurfaceBAsTrimVolumeKeepInside | AAsSurfaceBAsTrimVolumeKeepOutside.
Group/patch names from both inputs are merged (throws on group-id conflict for the same name). The new mesh inherits surface metadata from both. Both meshes must come from the same GeoAPI instance (same operating space / converter).")]
        public AnchorMesh Boolean(AnchorMesh meshA, AnchorMesh meshB, BooleanOp operation, string name = null)
        {
            name = name ?? GenerateName("Boolean");
            Dictionary<string, int> nameToGroupCombined = new Dictionary<string, int>(meshA.extendedNameToGroupId);
            EntityNaming.MergePatchNameMaps(nameToGroupCombined, meshB.extendedNameToGroupId);

            Dictionary<string, SurfaceMetaData> metaDataCombined = SurfaceMetaData.CloneDictionary(meshA.surfaceMetaData);
            if (meshB.surfaceMetaData != null)
            {
                foreach (var v in meshB.surfaceMetaData)
                {
                    if (metaDataCombined.ContainsKey(v.Key))
                        throw new NameCollisionException("Group name conflict: " + v.Key);
                    if (v.Value != null)
                        metaDataCombined.Add(v.Key, v.Value.Clone());
                }
            }
            if (Resolver.LogBooleanOps && name != null)
                Console.WriteLine("BoolOp: " + name);
            MeshNormalUV combinedMesh = MeshNormalUV.BooleanOperation(meshA.Mesh, meshB.Mesh, operation, converter);

            //var edges = GroupEdgeExtractor.ExtractGroupEdges(combinedMesh.Triangles, combinedMesh.GetGroupList(), combinedMesh.Positions);

            AnchorMesh result;
            lock (_meshRegistryLock)
            {
                bool resultIsVolume = BooleanResultIsVolume(operation, meshA, meshB);
                result = new AnchorMesh(name, combinedMesh, nameToGroupCombined, metaDataCombined,
                    deferCoplanarPostProcess: true, preferLexClosedLoopStarts: true, isVolume: resultIsVolume);
                RegisterMesh(result);
            }
            //nameOfMostRecentMesh = name;
            return result;
        }

        private static bool BooleanResultIsVolume(BooleanOp operation, AnchorMesh meshA, AnchorMesh meshB)
        {
            switch (operation)
            {
                case BooleanOp.AAsSurfaceBAsTrimSurfaceKeepInTriNormalDirection:
                case BooleanOp.AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection:
                case BooleanOp.AAsSurfaceBAsTrimVolumeKeepInside:
                case BooleanOp.AAsSurfaceBAsTrimVolumeKeepOutside:
                    return false;
                case BooleanOp.AAsVolumeBAsTrimSurfaceKeepInTriNormalDirection:
                case BooleanOp.AAsVolumeBAsTrimSurfaceRemoveInTriNormalDirection:
                    return meshA.IsVolume;
                default:
                    // Classic CSG: result is a volume only when both inputs are volumes.
                    return meshA.IsVolume && meshB.IsVolume;
            }
        }

        [APIDescription(@"Fillet(mesh: AnchorMesh, edgeNamesToFillet: List[str], filletRadius: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Adds rolling-ball fillets along the named edges of `mesh`.
  edgeNamesToFillet: edge identifiers between two surface patches; format ""[{patchA},{patchB}]"" (optional ""_index"" suffix when multiple components share the same pair).
  filletRadius: fillet radius (world units).
  maxDeviation: chordal tolerance for the swept fillet surface; -1 uses the GeoAPI instance default.
Returned mesh has new patches for each fillet surface (names prefixed ""BlendEdge_"" / ""BlendCorner_""). When `name` is omitted, the result keeps the input mesh name (replacing it in the registry) so `pipe = part.Fillet(pipe, ...)` remains addressable as `pipe-ExtrudeTop`. Throws if any edge name does not exist on `mesh`.")]
        public AnchorMesh Fillet(AnchorMesh mesh, List<string> edgeNamesToFillet, double filletRadius, double maxDeviation = -1, string name = null)
        {
            mesh.EnsureCoplanarPostProcessed();
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? mesh.Name;
            int startingGroupId = GetBaseGroupIndex();
            int groupIdOffset = startingGroupId;
            
            EdgeBlending edgeBlending = new EdgeBlending();
            AnchorMesh result = edgeBlending.BlendEdges(
                mesh, 
                edgeNamesToFillet, 
                filletRadius, 
                converter, 
                maxDeviation,
                ref groupIdOffset);

            // Increment by the number of groups created
            int numGroupsCreated = groupIdOffset - startingGroupId;
            IncrementBaseGroupIndex(numGroupsCreated);

            result.Name = name;

            RegisterMesh(result);
            return result;
        }

        [APIDescription(@"Chamfer(mesh: AnchorMesh, edgeNamesToChamfer: List[str], chamferDistance: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Adds symmetric edge chamfers along the named edges of `mesh`.
  edgeNamesToChamfer: edge identifiers between two surface patches; format ""[{patchA},{patchB}]"" (optional ""_index"" suffix when multiple components share the same pair).
  chamferDistance: chamfer distance along each adjacent face (world units).
  maxDeviation: tessellation tolerance; -1 uses the GeoAPI instance default.
Returned mesh has new patches for each chamfer surface (names prefixed ""ChamferEdge_"" / ""ChamferCorner_""). When `name` is omitted, the result keeps the input mesh name (replacing it in the registry) so `block = part.Chamfer(block, ...)` remains addressable as `block-ExtrudeTop`. Throws if any edge name does not exist on `mesh`.")]
        public AnchorMesh Chamfer(AnchorMesh mesh, List<string> edgeNamesToChamfer, double chamferDistance, double maxDeviation = -1, string name = null)
        {
            mesh.EnsureCoplanarPostProcessed();
            maxDeviation = ResolveMaxDeviation(maxDeviation);
            name = name ?? mesh.Name;
            int startingGroupId = GetBaseGroupIndex();
            int groupIdOffset = startingGroupId;

            ChamferBlending chamferBlending = new ChamferBlending();
            AnchorMesh result = chamferBlending.ChamferEdges(
                mesh,
                edgeNamesToChamfer,
                chamferDistance,
                converter,
                maxDeviation,
                ref groupIdOffset);

            int numGroupsCreated = groupIdOffset - startingGroupId;
            IncrementBaseGroupIndex(numGroupsCreated);

            result.Name = name;
            RegisterMesh(result);
            return result;
        }
    }
}
