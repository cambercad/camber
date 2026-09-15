using System.Collections.Generic;
using System.Collections.ObjectModel;
using Curves;
using GeoCore;
using GeoMeta;
using GeoSolver;
using GeoSolver.Sketcher;

namespace Geo
{
    /// <summary>
    /// Which GeoAPI output drives the default viewer focus: latest mesh or latest assembly activity.
    /// </summary>
    public enum GeoVisualOutputKind
    {
        Mesh,
        Assembly
    }

    public static class DefaultPoints
    {
        public static readonly string Origin = EntityNaming.DefaultPoints.Origin;
    }

    public static class DefaultPlanes
    {
        public static readonly string OriginXY = EntityNaming.DefaultPlanes.OriginXY;
        public static readonly string OriginYZ = EntityNaming.DefaultPlanes.OriginYZ;
        public static readonly string OriginZX = EntityNaming.DefaultPlanes.OriginZX;
    }

    public partial class GeoAPI
    {
        [APIDescription(@"GetMeshFromName(meshName: str) -> AnchorMesh
Returns the registered mesh with this exact name, or null if not found.")]
        public AnchorMesh GetMeshFromName(string meshName)
        {
            AnchorMesh found = null;
            for (int i = 0; i < meshes.Count; i++)
            {
                if (meshes[i].Name != meshName)
                    continue;
                if (found != null)
                    throw new NameCollisionException($"Mesh name '{meshName}' is registered more than once on this GeoAPI.");
                meshes[i].EnsureCoplanarPostProcessed();
                found = meshes[i];
            }
            return found;
        }

        [APIDescription(@"GetTopMesh() -> AnchorMesh
Returns the most recently registered mesh on this API (or null if none). Default visualization uses this when VisualOutputKind is Mesh.")]
        public AnchorMesh GetTopMesh()
        {
            if (meshes.Count == 0)
                return null;
            var mesh = meshes[meshes.Count - 1];
            mesh.EnsureCoplanarPostProcessed();
            return mesh;
        }

        [APIDescription(@"GetTopAssembly() -> Assembly
Returns the assembly most recently touched by AddPart / mates / SolveConstraints (or null if none). Default visualization shows all of its parts when VisualOutputKind is Assembly.")]
        public Assembly GetTopAssembly() => lastActiveAssembly;

        [APIDescription(@"VisualOutputKind: GeoVisualOutputKind
Mesh when the latest registered output was a mesh; Assembly when the latest activity was on an assembly. Drives default GeoScriptViewer mesh selection.")]
        public GeoVisualOutputKind VisualOutputKind => visualOutputKind;

        internal void NotifyMeshRegistered() => visualOutputKind = GeoVisualOutputKind.Mesh;

        internal void NotifyAssemblyActivity(Assembly assembly)
        {
            lastActiveAssembly = assembly;
            visualOutputKind = GeoVisualOutputKind.Assembly;
        }

        /// <summary>
        /// All meshes added to this API (e.g. each <see cref="GeoAPI.Extrude"/>, <see cref="GeoAPI.Revolve"/>, <see cref="GeoAPI.Loft"/>).
        /// Read-only wrapper; does not copy geometry.
        /// </summary>
        [APIDescription(@"GetMeshes() -> IReadOnlyList[AnchorMesh]
Read-only list of all meshes registered on this API (no copy).")]
        public IReadOnlyList<AnchorMesh> GetMeshes() => new ReadOnlyCollection<AnchorMesh>(meshes);

        [APIDescription(@"GetCurves() -> List[Curve3D]
All 3D curves added via AddLine and similar (live list; do not mutate while iterating).")]
        public List<Curve3D> GetCurves()
        {
            return curves3D;
        }

        [APIDescription(@"GetPlanes() -> List[Plane3D]
All Plane3D objects registered via AddPlane (live list).")]
        public List<Plane3D> GetPlanes()
        {
            return planes3D;
        }

        [APIDescription(@"GetSketches() -> List[PlotterSketcherCoordSys]
All sketches registered via GetPlotterSketcher / GetSketcherFromDxf / GetSketcherFromSvg (live list).")]
        public List<PlotterSketcherCoordSys> GetSketches()
        {
            return sketches;
        }

        [APIDescription(@"GetAssemblies() -> List[Assembly]
All assemblies registered via GetAssembly (live list).")]
        public List<Assembly> GetAssemblies()
        {
            return assemblies;
        }

        [APIDescription(@"Get(name: str) -> object
Generic name lookup. Returns the first matching: 3D point, 2D constrained sketch point, 2D sketch point, 2D sketch curve, mesh edge (LineStrip3D), or Plane3D. Returns null if nothing matches. Sketch curves can be addressed as ""<curveName>"" or ""<sketchName>:<curveName>"".")]
        public object Get(string name)
        {
            TryGetFromName(name, out object result);
            return result;
        }

        [APIDescription(@"GetPointFromName(name: str) -> Vec3D
Resolves a 3D point. Accepts DefaultPoints.Origin, mesh anchors on any registered mesh (newest first), or ""{meshName}:{anchor}"". Throws if the name cannot be resolved.")]
        public Vec3D GetPointFromName(string name)
        {
            Vec3D result;
            if (TryGetPointFromName(name, out result))
                return result;

            throw new Exception($"Point {name} does not exist");
        }
        [APIDescription(@"TryGetPointFromName(name: str, out result: Vec3D) -> bool
Non-throwing variant of GetPointFromName. Accepts DefaultPoints.Origin, mesh anchors on any registered mesh (newest first), or ""{meshName}:{anchor}"" for a specific mesh. Anchor formats: ""[{patchA},{patchB}]@u"", ""[{patch}]@u,v"", or legacy ""{patchA}_{patchB}_{index}_{u}"".")]
        public bool TryGetPointFromName(string name, out Vec3D result)
        {
            if (name == DefaultPoints.Origin)
            {
                result = new Vec3D(0);
                return true;
            }

            result = default;
            string localAddress = name;
            if (TryGetMeshForQualifiedAddress(name, out AnchorMesh specificMesh, out localAddress))
            {
                if (TryGetPointOnMesh(specificMesh, localAddress, out result))
                    return true;
                return false;
            }

            var matchingMeshes = new List<string>();
            Vec3D found = default;
            foreach (AnchorMesh mesh in MeshesNewestFirst())
            {
                if (TryGetPointOnMesh(mesh, localAddress, out var point))
                {
                    matchingMeshes.Add(mesh.Name);
                    found = point;
                }
            }

            if (matchingMeshes.Count == 0)
                return false;

            EntityNaming.ThrowIfAmbiguous(localAddress, matchingMeshes, "Point");
            result = found;
            return true;
        }

        private bool TryGetPointOnMesh(AnchorMesh mesh, string localAddress, out Vec3D result)
        {
            if (mesh != null && mesh.TryGetPointOnEdge(localAddress, out result))
                return true;
            if (mesh != null && mesh.TryGetPointOnSurface(localAddress, out result))
                return true;
            result = default;
            return false;
        }

        [APIDescription(@"GetSketchConstraintPoint(sketch: ConstrainedSketcher, name: str) -> CVec2D
Resolves any named point for a constraint on `sketch`. Sketch-created handles stay live; every other resolvable 3D model point is projected onto the sketch plane as a constant.")]
        public CVec2D GetSketchConstraintPoint(ConstrainedSketcher sketch, string name)
        {
            if (TryGetSketchConstraintPoint(sketch, name, out CVec2D point))
                return point;
            throw new Exception("Point " + name + " cannot be resolved as a sketch constraint reference.");
        }

        [APIDescription(@"TryGetSketchConstraintPoint(sketch: ConstrainedSketcher, name: str, out point: CVec2D) -> bool
Non-throwing variant of GetSketchConstraintPoint.")]
        public bool TryGetSketchConstraintPoint(ConstrainedSketcher sketch, string name, out CVec2D point)
        {
            point = default;
            if (sketch == null)
                return false;
            if (sketch.TryGetConstraintPoint(name, out point))
                return true;
            return TryGetCPointOnSketchFromName(name, out point);
        }

        [APIDescription(@"TryGetCPointOnSketchFromName(name: str, out result: CVec2D) -> bool
Searches all ConstrainedSketcher instances for a constrained point on a curve with this name. Use ""<sketchName>:<curveName>@<u>"" to disambiguate; otherwise throws if multiple sketches match.")]
        public bool TryGetCPointOnSketchFromName(string name, out CVec2D result)
        {
            result = default;
            if (!EntityNaming.TryParseQualifiedSketchCurveAddress(name, out var qualified))
                return false;

            if (qualified.HasSketchQualifier)
            {
                for (int i = 0; i < sketches.Count; ++i)
                {
                    var sketch = sketches[i] as ConstrainedSketcher;
                    if (sketch != null && sketch.Name == qualified.SketchName)
                        return sketch.TryGetCPointOnEdge(qualified.LocalAddress, out result);
                }
                return false;
            }

            var matchingSketches = new List<string>();
            CVec2D found = default;
            for (int i = 0; i < sketches.Count; ++i)
            {
                var sketch = sketches[i] as ConstrainedSketcher;
                if (sketch != null && sketch.TryGetCPointOnEdge(qualified.LocalAddress, out var point))
                {
                    matchingSketches.Add(sketch.Name);
                    found = point;
                }
            }

            if (matchingSketches.Count == 0)
                return false;

            EntityNaming.ThrowIfAmbiguous(name, matchingSketches, "Constrained sketch point");
            result = found;
            return true;
        }
        [APIDescription(@"TryGetPointOnSketchFromName(name: str, out result: Vec2D) -> bool
Searches all sketches for a 2D point on a curve with this name. Use ""<sketchName>:<curveName>@<u>"" to disambiguate; otherwise throws if multiple sketches match.")]
        public bool TryGetPointOnSketchFromName(string name, out Vec2D result)
        {
            result = default;
            if (!EntityNaming.TryParseQualifiedSketchCurveAddress(name, out var qualified))
                return false;

            if (qualified.HasSketchQualifier)
            {
                for (int i = 0; i < sketches.Count; ++i)
                {
                    PlotterSketcherCoordSys sketch = sketches[i];
                    if (sketch.Name == qualified.SketchName)
                        return sketch.TryGetPointOnEdge(qualified.LocalAddress, out result);
                }
                return false;
            }

            var matchingSketches = new List<string>();
            Vec2D found = default;
            for (int i = 0; i < sketches.Count; ++i)
            {
                PlotterSketcherCoordSys sketch = sketches[i];
                if (sketch.TryGetPointOnEdge(qualified.LocalAddress, out var point))
                {
                    matchingSketches.Add(sketch.Name);
                    found = point;
                }
            }

            if (matchingSketches.Count == 0)
                return false;

            EntityNaming.ThrowIfAmbiguous(name, matchingSketches, "Sketch point");
            result = found;
            return true;
        }

        /// <summary>
        /// Try to get a curve from a sketch by name.
        /// Name format can be "curveName" or "sketchName:curveName"
        /// </summary>
        [APIDescription(@"TryGetCurveFromSketch(name: str, out result: Curve2D) -> bool
Looks up a 2D curve by name across all ConstrainedSketcher instances. Use ""<sketchName>:<curveName>"" to disambiguate; otherwise returns the first match across all sketches.")]
        public bool TryGetCurveFromSketch(string name, out Curves.Curve2D result)
        {
            result = null;

            var qualified = EntityNaming.ParseQualifiedCurveName(name);
            string sketchName = qualified.SketchName;
            string curveName = qualified.CurveName;

            if (sketchName != null)
            {
                for (int i = 0; i < sketches.Count; ++i)
                {
                    var sketch = sketches[i] as ConstrainedSketcher;
                    if (sketch != null && sketch.Name == sketchName)
                    {
                        try
                        {
                            result = sketch.GetCurve(curveName);
                            return result != null;
                        }
                        catch
                        {
                            return false;
                        }
                    }
                }
                return false;
            }

            var matchingSketches = new List<string>();
            Curves.Curve2D found = null;
            for (int i = 0; i < sketches.Count; ++i)
            {
                var sketch = sketches[i] as ConstrainedSketcher;
                if (sketch == null)
                    continue;
                try
                {
                    var curve = sketch.GetCurve(curveName);
                    if (curve != null)
                    {
                        matchingSketches.Add(sketch.Name);
                        found = curve;
                    }
                }
                catch
                {
                    // Curve not found in this sketch
                }
            }

            if (matchingSketches.Count == 0)
                return false;

            EntityNaming.ThrowIfAmbiguous(name, matchingSketches, "Sketch curve");
            result = found;
            return true;
        }

        [APIDescription(@"TryGetPlaneFromName(name: str, out result: Plane3D) -> bool
Resolves a plane. Recognized: DefaultPlanes.OriginXY / OriginYZ / OriginZX, planes added via AddPlane, or a planar surface patch on any mesh (""{meshName}:{patchName}"" or patch name alone, searched newest mesh first).")]
        public bool TryGetPlaneFromName(string name, out Plane3D result)
        {
            if (name == DefaultPlanes.OriginXY)
            {
                result = new Plane3D(new Vec3D(0), new Vec3D(0,0,1), new Vec3D(1,0,0), new Vec3D(0,1,0));
                return true;
            }
            if (name == DefaultPlanes.OriginYZ)
            {
                result = new Plane3D(new Vec3D(0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
                return true;
            }
            if (name == DefaultPlanes.OriginZX)
            {
                result = new Plane3D(new Vec3D(0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
                return true;
            }

            for(int i=0;i<planes3D.Count;++i)
            {
                if (planes3D[i].Name==name)
                {
                    result = planes3D[i];
                    return true;
                }
            }

            result = null;

            string localAddress = name;
            if (TryGetMeshForQualifiedAddress(name, out AnchorMesh specificMesh, out localAddress))
            {
                if (TryGetPlaneOnMesh(specificMesh, localAddress, out result))
                    return true;
                return false;
            }

            var matchingMeshes = new List<string>();
            Plane3D found = null;
            foreach (AnchorMesh mesh in MeshesNewestFirst())
            {
                if (TryGetPlaneOnMesh(mesh, localAddress, out var plane))
                {
                    matchingMeshes.Add(mesh.Name);
                    found = plane;
                }
            }

            if (matchingMeshes.Count == 0)
            {
                result = null;
                return false;
            }

            EntityNaming.ThrowIfAmbiguous(localAddress, matchingMeshes, "Planar patch");
            result = found;
            return true;
        }

        private bool TryGetPlaneOnMesh(AnchorMesh mesh, string localAddress, out Plane3D result)
        {
            result = null;
            if (mesh == null)
                return false;
            if (!mesh.TryGetSurface(localAddress, out UVSurface s))
                return false;
            if (!s.IsPlanar())
                return false;

            var point = s.ApproximatePlanarSurfaceCenter(out var normal, out var tangentX, out var tangentY);
            normal.Normalize();
            tangentY = Vec3DOps.Cross(normal, tangentX);
            tangentY.Normalize();
            tangentX = Vec3DOps.Cross(tangentY, normal);
            tangentX.Normalize();
            result = new Plane3D(point, normal, tangentX, tangentY);
            return true;
        }

        [APIDescription(@"GetEdgeFromName(name: str) -> LineStrip3D
Resolves a named edge (boundary between two surface patches) on the top mesh. Throws if not found.")]
        public LineStrip3D GetEdgeFromName(string name)
        {
            LineStrip3D result;
            if (TryGetEdgeFromName(name, out result))
                return result;

            throw new Exception($"Edge {name} does not exist");
        }
        [APIDescription(@"TryGetEdgeFromName(name: str, out result: LineStrip3D) -> bool
Non-throwing variant of GetEdgeFromName. Resolves ""[{patchA},{patchB}]"" (optional ""_index"") on any mesh, or ""{meshName}:{edgeAddress}"" for a specific mesh.")]
        public bool TryGetEdgeFromName(string name, out LineStrip3D result)
        {
            result = default;
            string localAddress = name;
            if (TryGetMeshForQualifiedAddress(name, out AnchorMesh specificMesh, out localAddress))
            {
                if (specificMesh != null && specificMesh.TryGetEdge(localAddress, out result))
                    return true;
                return false;
            }

            var matchingMeshes = new List<string>();
            LineStrip3D found = default;
            foreach (AnchorMesh mesh in MeshesNewestFirst())
            {
                if (mesh.TryGetEdge(localAddress, out var edge))
                {
                    matchingMeshes.Add(mesh.Name);
                    found = edge;
                }
            }

            if (matchingMeshes.Count == 0)
                return false;

            EntityNaming.ThrowIfAmbiguous(localAddress, matchingMeshes, "Edge");
            result = found;
            return true;
        }
        [APIDescription(@"TryGetFromName(name: str, out value: object) -> bool
Generic name lookup, same priority order as Get(name). On success `value` is one of: Vec3D, CVec2D, Vec2D, Curve2D, LineStrip3D, Plane3D.")]
        public bool TryGetFromName(string name, out object value)
        {
            value = null;
            Vec3D point3D;
            if (TryGetPointFromName(name, out point3D))
            {
                value = point3D;
                return true;
            }
            CVec2D cPoint2D;
            if (TryGetCPointOnSketchFromName(name, out cPoint2D))
            {
                value = cPoint2D;
                return true;
            }
            Vec2D point2D;
            if (TryGetPointOnSketchFromName(name, out point2D))
            {
                value = point2D;
                return true;
            }
            // Try to get sketch curves
            Curves.Curve2D curve2D;
            if (TryGetCurveFromSketch(name, out curve2D))
            {
                value = curve2D;
                return true;
            }
            LineStrip3D edge;
            if (TryGetEdgeFromName(name, out edge))
            {
                value = edge;
                return true;
            }
            Plane3D plane;
            if (TryGetPlaneFromName(name, out plane))
            {
                value = plane;
                return true;
            }
            return false;
        }

        /// <summary>Resolves <c>{meshName}:{localAddress}</c> when the prefix is a registered mesh name.</summary>
        private bool TryGetMeshForQualifiedAddress(string name, out AnchorMesh mesh, out string localAddress)
        {
            mesh = null;
            localAddress = name;
            if (!EntityNaming.TryParseQualifiedMeshAddress(name, IsKnownMeshName, out var qualified))
                return false;
            mesh = GetMeshFromName(qualified.MeshName);
            localAddress = qualified.LocalAddress;
            return mesh != null;
        }

        private bool IsKnownMeshName(string name) => GetMeshFromName(name) != null;

        private IEnumerable<AnchorMesh> MeshesNewestFirst()
        {
            for (int i = meshes.Count - 1; i >= 0; i--)
            {
                meshes[i].EnsureCoplanarPostProcessed();
                yield return meshes[i];
            }
        }

        /// <summary>Registers a mesh, replacing any previously registered mesh with the same name.</summary>
        private void RegisterMesh(AnchorMesh mesh)
        {
            UnregisterMeshesNamed(mesh.Name, except: mesh);
            mesh.MeshNameEvictor = name => UnregisterMeshesNamed(name, except: mesh);
            meshes.Add(mesh);
            NotifyMeshRegistered();
        }

        private void UnregisterMeshesNamed(string name, AnchorMesh except)
        {
            for (int i = meshes.Count - 1; i >= 0; i--)
            {
                if (meshes[i].Name == name && !ReferenceEquals(meshes[i], except))
                    meshes.RemoveAt(i);
            }
        }

        /// <summary>Registers a sketch; throws if another sketch with the same name exists.</summary>
        internal void RegisterSketch(PlotterSketcherCoordSys sketch)
        {
            for (int i = 0; i < sketches.Count; i++)
            {
                if (sketches[i].Name == sketch.Name)
                    throw new NameCollisionException($"Sketch name already registered: '{sketch.Name}'.");
            }
            sketch.SetDefaultMaxDeviation(_maxDeviation);
            sketch.SetWorldPointQuery(ResolveWorldPointOrNaN);
            sketches.Add(sketch);
        }

        Vec3D ResolveWorldPointOrNaN(string name)
        {
            if (TryGetPointFromName(name, out Vec3D point))
                return point;
            return new Vec3D(double.NaN, double.NaN, double.NaN);
        }

        /// <summary>
        /// Removes a previously registered sketch (e.g. interactive session discard). No-op if not found.
        /// </summary>
        public bool UnregisterSketch(PlotterSketcherCoordSys sketch)
        {
            if (sketch == null) return false;
            return sketches.Remove(sketch);
        }

        internal bool IsRegisteredMesh(AnchorMesh mesh) => mesh != null && meshes.Contains(mesh);

        private void RegisterAssembly(Assembly assembly)
        {
            for (int i = 0; i < assemblies.Count; i++)
            {
                if (assemblies[i].Name == assembly.Name)
                    throw new NameCollisionException($"Assembly name already registered: '{assembly.Name}'.");
            }
            assemblies.Add(assembly);
        }
    }
}
