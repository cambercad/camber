using CSG;
using GeoCore;
using GeoMeta;
using GeoSolver.Kinematics;
using System.Linq.Expressions;

namespace Geo
{
    public class AnchorMesh : IUpdateTransform
    {
        public string Name;
        public MeshNormalUV Mesh;
        public List<GroupEdge> GroupEdges; //These are the anchors (plus their end points and mid points)

        /// <summary>Return the unique current name of a selectable edge.</summary>
        public string GetEdgeReference(int index)
        {
            EnsureCoplanarPostProcessed();
            if (index < 0 || index >= GroupEdges.Count)
                throw new ArgumentOutOfRangeException(nameof(index));
            string reference = GroupEdges[index].Name;
            string prefix = (string.IsNullOrEmpty(Name) ? "mesh" : Name) + ":";
            return reference.StartsWith(prefix, StringComparison.Ordinal)
                ? reference : prefix + reference;
        }

        public Dictionary<string, int> extendedNameToGroupId;
        public Dictionary<int, string> groupIdToExtendedName;

        public Dictionary<string, SurfaceMetaData> surfaceMetaData;
        internal Dictionary<int, FaceLineage> FaceLineages = new();
        internal HashSet<string> AmbiguousFaceReferences = new(StringComparer.Ordinal);
        // Dilation distributes over union. Inward shelling uses the current
        // mesh directly and does not need intersection construction history.
        internal (AnchorMesh First, AnchorMesh Second)? UnionOperands;

        private void InitializeFaceLineage(Dictionary<int, FaceLineage> inherited = null)
        {
            FaceLineages = inherited == null ? new() : new(inherited);
            foreach (var (id, label) in groupIdToExtendedName)
                if (!FaceLineages.ContainsKey(id)) FaceLineages[id] = new FaceLineage(new[] { label });
        }


        /// <summary>
        /// True for closed solid meshes (Extrude/Loft/CSG volumes, watertight imports).
        /// False for open sheets / trim surfaces / visualisation assemblies — patch detection,
        /// display, and surface-trim booleans still work; volume invariants are not enforced.
        /// </summary>
        public bool IsVolume = true;

        /// <summary>
        /// When true, closed group-edge strips use world lex-min (x,y,z) as uniform 0
        /// instead of cylinder/plane RefDir (Boolean/CSG results).
        /// </summary>
        public bool PreferLexClosedLoopStarts;

        private bool _coplanarPostProcessPending;

        private bool _rigidBodyActive;
        private Vec3D[] _rigidRestLocal;
        private Rat3Hybrid[] _rigidRestExact;
        private MeshTriangle<TriangleVertexNormalUV>[] _rigidRestCorners;
        private Quaternion _rigidDisplayOrientation;
        private bool _rigidDisplayNormalsCurrent;
        private Dictionary<string, SurfaceMetaData> _rigidRestMeta;
        private CoordinateConverter _rigidConverter;
        private Dictionary<GroupEdge, LineStrip3D> _rigidRestEdges;
        private bool _rigidConverterSet;

        /// <summary>Set by <see cref="GeoAPI"/> at registration to replace any other registered mesh with the same name.</summary>
        internal Action<string> MeshNameEvictor;

        /// <summary>
        /// Runs coplanar fusion, retriangulation, and mesh clean once (idempotent).
        /// Boolean results defer this until export or explicit finalize so intermediate CSG steps avoid redundant work.
        /// </summary>
        public void EnsureCoplanarPostProcessed()
        {
            if (!_coplanarPostProcessPending)
                return;

            _coplanarPostProcessPending = false;
            FuseCoplanarPlanes();
            GroupEdges = GroupEdgeExtractor.ExtractGroupEdges(
                Mesh.Triangles, Mesh.GetTriangleGroups(), Mesh.Positions, null, groupIdToExtendedName, surfaceMetaData,
                PreferLexClosedLoopStarts);
            AttachEdgeMetadata();
            ValidateMeshInvariants();
        }

        public AnchorMesh(string name, MeshNormalUV mesh, Dictionary<string, int> extendedNameToGroupId, Dictionary<string, SurfaceMetaData> surfaceMetaData, bool isVolume = true)
            : this(name, mesh, extendedNameToGroupId, surfaceMetaData, deferCoplanarPostProcess: false, preferLexClosedLoopStarts: false, isVolume: isVolume)
        {
        }

        internal AnchorMesh(string name, MeshNormalUV mesh, Dictionary<string, int> extendedNameToGroupId, Dictionary<string, SurfaceMetaData> surfaceMetaData, bool deferCoplanarPostProcess, bool preferLexClosedLoopStarts = false, bool isVolume = true,
            Dictionary<int, FaceLineage> faceLineages = null, HashSet<string> ambiguousReferences = null)
        {
            Name = name;
            Mesh = mesh;
            this.extendedNameToGroupId = extendedNameToGroupId;

            groupIdToExtendedName = EntityNaming.BuildGroupIdToName(extendedNameToGroupId);

            this.surfaceMetaData = surfaceMetaData;
            PreferLexClosedLoopStarts = preferLexClosedLoopStarts;
            IsVolume = isVolume;
            InitializeFaceLineage(faceLineages);
            if (ambiguousReferences != null) AmbiguousFaceReferences.UnionWith(ambiguousReferences);
            FinishConstruction(deferCoplanarPostProcess);
        }

        public AnchorMesh(string name, MeshNormalUV mesh, Dictionary<int, string> groupIdToName, Dictionary<string, SurfaceMetaData> surfaceMetaData, bool isVolume = true)
            : this(name, mesh, groupIdToName, surfaceMetaData, deferCoplanarPostProcess: false, skipCoplanarFusion: false, isVolume: isVolume)
        {
        }

        internal AnchorMesh(
            string name,
            MeshNormalUV mesh,
            Dictionary<int, string> groupIdToName,
            Dictionary<string, SurfaceMetaData> surfaceMetaData,
            bool deferCoplanarPostProcess,
            bool skipCoplanarFusion = false,
            bool isVolume = true,
            bool preserveTriangulation = false,
            Dictionary<int, FaceLineage> faceLineages = null, HashSet<string> ambiguousReferences = null)
        {
            Name = name;
            Mesh = mesh;
            this.groupIdToExtendedName = groupIdToName;

            extendedNameToGroupId = EntityNaming.BuildNameToGroupId(groupIdToExtendedName);

            this.surfaceMetaData = new Dictionary<string, SurfaceMetaData>();
            foreach (var n in extendedNameToGroupId.Keys)
            {
                if (surfaceMetaData.TryGetValue(n, out var v))
                    this.surfaceMetaData.Add(n, v);
            }

            IsVolume = isVolume;
            InitializeFaceLineage(faceLineages);
            if (ambiguousReferences != null) AmbiguousFaceReferences.UnionWith(ambiguousReferences);
            FinishConstruction(deferCoplanarPostProcess, skipCoplanarFusion, preserveTriangulation);
        }

        private void FinishConstruction(
            bool deferCoplanarPostProcess,
            bool skipCoplanarFusion = false,
            bool preserveTriangulation = false)
        {
            if (preserveTriangulation)
            {
                ValidateLineageReferences(Mesh.GetTriangleGroups());
                EnsureUniqueCurrentPatchNames(Mesh.GetTriangleGroups());
                GroupEdges = GroupEdgeExtractor.ExtractGroupEdges(
                    Mesh.Triangles, Mesh.GetTriangleGroups(), Mesh.Positions, null,
                    groupIdToExtendedName, surfaceMetaData, PreferLexClosedLoopStarts);
                AttachEdgeMetadata();
                ValidateMeshInvariants();
                return;
            }

            if (deferCoplanarPostProcess)
            {
                _coplanarPostProcessPending = true;
                GroupEdges = GroupEdgeExtractor.ExtractGroupEdges(
                    Mesh.Triangles, Mesh.GetTriangleGroups(), Mesh.Positions, null, groupIdToExtendedName, surfaceMetaData,
                    PreferLexClosedLoopStarts);
                AttachEdgeMetadata();
                return;
            }

            FuseCoplanarPlanes(skipCoplanarFusion);
            GroupEdges = GroupEdgeExtractor.ExtractGroupEdges(
                Mesh.Triangles, Mesh.GetTriangleGroups(), Mesh.Positions, null, groupIdToExtendedName, surfaceMetaData,
                PreferLexClosedLoopStarts);
            AttachEdgeMetadata();
            ValidateMeshInvariants();
        }

        private void AttachEdgeMetadata()
        {
            Geo.NurbsConstruction.EdgeMetaDataBuilder.Attach(this);
        }

#if DEBUG
        private void ValidateMeshInvariants()
        {
            // Open sheets / trim surfaces / visualisation assemblies are not solids.
            if (!IsVolume || Mesh.Triangles.Count == 0)
                return;

            if (!MeshAnalysis.IsWatertightMesh(Mesh.PrecisionPositions, Mesh.Triangles, true))
                throw new Exception("Mesh is not watertight");
            if (!MeshAnalysis.AreTrianglesConsistentlyOriented(Mesh.PrecisionPositions, Mesh.Triangles, true))
                throw new Exception("Mesh triangles are not consistently oriented");
            if (MeshAnalysis.ComputeSignedMeshVolume(Mesh.Positions, Mesh.Triangles) < 0)
                throw new Exception("Mesh has negative signed volume (solid orientation inverted relative to outward normals)");
        }
#else
        private void ValidateMeshInvariants() { }
#endif

        private HashSet<int> GetPlanarSurfaceIds()
        {
            HashSet<int> result = new HashSet<int>();
            Dictionary<int, List<int>> triangleIdsPerGroup = CoplanarGroupFusion.ExtractTriangleIdsPerGroup(Mesh.GetTriangleGroups());
            // Patches whose metadata is not Planar (curved, twisted caps, Unknown) are not
            // fusion candidates; this mirrors the original post-planarity metadata check.
            // This scan stays sequential: it can run underneath an existing parallel
            // Boolean or tessellation, and a nested Parallel.For would stall the pool.
            foreach (var group in triangleIdsPerGroup)
            {
                if (HasExplicitNonPlanarSurfaceType(group.Key))
                    continue;
                List<Tri> tris = GetTriangles(group.Value);
                UVSurface tmp = new UVSurface(Mesh.Positions, null, null, tris, Mesh.PrecisionPositions);
                if (!tmp.IsSurfacePlanar())
                    continue;
                // Groups with UV seams or sharp edges must not be retriangulated.
                if (GroupHasVertexDataDiscontinuities(group.Key, group.Value))
                    continue;
                result.Add(group.Key);
            }
            return result;
        }

        private bool HasExplicitNonPlanarSurfaceType(int groupId)
        {
            return surfaceMetaData != null
                && groupIdToExtendedName.TryGetValue(groupId, out string name)
                && surfaceMetaData.TryGetValue(name, out var metadata)
                && metadata.SurfaceType != SurfaceType.Planar;
        }

        /// <summary>
        /// Checks if a group has any vertex data discontinuities (UV seams, sharp edges, etc.).
        /// Groups with discontinuities should not be retriangulated or Delaunay-optimized
        /// as this would lose per-corner vertex data.
        /// </summary>
        private bool GroupHasVertexDataDiscontinuities(int groupId, List<int> triangleIndices)
        {
            // Build: vertexIdx → list of vertex data at that vertex (within this group)
            var vertexData = new Dictionary<int, List<TriangleVertexNormalUV>>();

            foreach (int triIndex in triangleIndices)
            {
                var tri = Mesh.Triangles[triIndex];
                var triEx = Mesh.TrianglesEx[triIndex];

                // Only check triangles in this group
                if (triEx.GroupId != groupId)
                    continue;

                // Record vertex data for each vertex
                RecordVertexData(vertexData, tri.A, triEx.V0);
                RecordVertexData(vertexData, tri.B, triEx.V1);
                RecordVertexData(vertexData, tri.C, triEx.V2);
            }

            // Check if any vertex has inconsistent data (UV seam or normal discontinuity)
            foreach (var kvp in vertexData)
            {
                var dataList = kvp.Value;
                if (dataList.Count <= 1)
                    continue;

                TriangleVertexNormalUV first = dataList[0];
                for (int i = 1; i < dataList.Count; i++)
                {
                    if (!AreVertexDataEqual(first, dataList[i]))
                    {
                        return true; // Discontinuity detected (UV seam or sharp edge)
                    }
                }
            }

            return false;
        }

        private void RecordVertexData(Dictionary<int, List<TriangleVertexNormalUV>> vertexData, int vertexIdx, TriangleVertexNormalUV data)
        {
            if (!vertexData.ContainsKey(vertexIdx))
                vertexData[vertexIdx] = new List<TriangleVertexNormalUV>();
            vertexData[vertexIdx].Add(data);
        }

        private List<Tri> GetTriangles(List<int> triIndices)
        {
            List<Tri> result = new List<Tri>(triIndices.Count);
            for (int i = 0; i < triIndices.Count; ++i)
            {
                var id = triIndices[i];
                result.Add(Mesh.Triangles[id]);
            }
            return result;
        }

        /// <param name="skipCoplanarFusion">
        /// When true, skip coplanar group fusion and planar retriangulation (e.g. fillet output where input
        /// was already fused). Still runs collinear edge collapse and Delaunay on planar groups,
        /// then splits disconnected same-group islands into named faces.
        /// </param>
        private void FuseCoplanarPlanes(bool skipCoplanarFusion = false)
        {
            HashSet<int> idsOfPlanarGroups = GetPlanarSurfaceIds();

            if (idsOfPlanarGroups.Count > 0)
            {
                HashSet<int> retriangulatedGroupIds = new HashSet<int>();
                HashSet<int> affineUvGroups = new();
                if (!skipCoplanarFusion)
                {
                    var oldGroupsForLineage = Mesh.GetTriangleGroups();
                    var representativeTriangles = CoplanarGroupFusion.FuseCoplanarGroups(Mesh.Triangles, Mesh.GetTriangleGroups(), Mesh.PrecisionPositions,
                        groupIdToExtendedName, idsOfPlanarGroups, out var newGroupPerTriangle, out var newGroupToId);
                    var mergedLineage = MergeLineageByGroup(oldGroupsForLineage, newGroupPerTriangle);
                    FaceLineages = mergedLineage;

                    groupIdToExtendedName = newGroupToId;
                    RefreshFusedLineageNames();
                    Mesh.SetTriangleGroups(newGroupPerTriangle);

                    CoplanarGroupFusionUVUpdate.UpdateUVsForFusedGroups(
                        Mesh.Positions,
                        Mesh.Triangles,
                        Mesh.TrianglesEx,
                        newGroupPerTriangle,
                        representativeTriangles);
                    affineUvGroups.UnionWith(representativeTriangles.Keys);

                    retriangulatedGroupIds = CoplanarGroupRetriangulation.RetriangulateCoplanar(
                        Mesh.PrecisionPositions, Mesh.Triangles, Mesh.TrianglesEx, idsOfPlanarGroups);
                }

                CleanMesh(Mesh.PrecisionPositions, Mesh.Triangles, Mesh.TrianglesEx, affineUvGroups);

                int triCountBeforeDelaunay = Mesh.Triangles.Count;
                DelaunayFlipper.MakeDelaunay(Mesh.PrecisionPositions, Mesh.Triangles, Mesh.TrianglesEx, idsOfPlanarGroups, retriangulatedGroupIds);
#if DEBUG
                if (triCountBeforeDelaunay != Mesh.Triangles.Count)
                    throw new Exception("Delaunay optimization changed triangle count");
#endif
            }

            SplitDisconnectedPatchGroups();
        }

        private Dictionary<int, FaceLineage> MergeLineageByGroup(List<int> oldGroups, List<int> newGroups)
        {
            var oldIdsByNewGroup = new Dictionary<int, List<int>>();
            for (int i = 0; i < newGroups.Count; i++)
            {
                int newId = newGroups[i];
                int oldId = oldGroups[i];
                if (!oldIdsByNewGroup.TryGetValue(newId, out List<int> oldIds))
                {
                    oldIds = new List<int>(2);
                    oldIdsByNewGroup[newId] = oldIds;
                }
                bool seen = false;
                for (int k = 0; k < oldIds.Count; k++)
                {
                    if (oldIds[k] == oldId)
                    {
                        seen = true;
                        break;
                    }
                }
                if (!seen)
                    oldIds.Add(oldId);
            }

            var merged = new Dictionary<int, FaceLineage>(oldIdsByNewGroup.Count);
            foreach (var pair in oldIdsByNewGroup)
            {
                List<int> oldIds = pair.Value;
                if (oldIds.Count == 1)
                {
                    merged[pair.Key] = FaceLineages[oldIds[0]];
                    continue;
                }

                var faces = new FaceLineage[oldIds.Count];
                for (int i = 0; i < oldIds.Count; i++)
                    faces[i] = FaceLineages[oldIds[i]];
                merged[pair.Key] = FaceLineage.Merge(faces);
            }
            return merged;
        }

        /// <summary>
        /// Promote disconnected triangle islands that still share one group id into separately
        /// named faces (<c>origin</c>, <c>origin_1</c>, …). Must run after coplanar fusion /
        /// clean / Delaunay and before group-edge extraction.
        /// </summary>
        private void SplitDisconnectedPatchGroups()
        {
            var groupIdPerTriangle = Mesh.GetTriangleGroups();
            var originalLineageGroups = groupIdPerTriangle.ToArray();
            bool changed = DisconnectedGroupSplit.SplitDisconnectedGroups(
                Mesh.Triangles,
                Mesh.Positions,
                groupIdPerTriangle,
                groupIdToExtendedName,
                surfaceMetaData,
                GeoAPI.ReserveGroupIds);

            if (changed)
            {
                var before = new Dictionary<int, FaceLineage>(FaceLineages);
                var splitGroups = Enumerable.Range(0, groupIdPerTriangle.Count)
                    .GroupBy(i => originalLineageGroups[i])
                    .Where(group => group.Select(i => groupIdPerTriangle[i]).Distinct().Count() > 1)
                    .Select(group => group.Key).ToHashSet();
                var assigned = new HashSet<int>();
                for (int i = 0; i < groupIdPerTriangle.Count; i++)
                {
                    if (!assigned.Add(groupIdPerTriangle[i])) continue;
                    var old = before[originalLineageGroups[i]];
                    FaceLineages[groupIdPerTriangle[i]] = splitGroups.Contains(originalLineageGroups[i])
                        ? new FaceLineage(old.Roots, old.Separators, true, old.Supported) : old;
                }
                Mesh.SetTriangleGroups(groupIdPerTriangle);
            }
            ValidateLineageReferences(groupIdPerTriangle);
            EnsureUniqueCurrentPatchNames(groupIdPerTriangle);


        }

        /// <summary>
        /// Cleans the mesh by collapsing collinear vertices while preserving UV seams.
        /// </summary>
        /// <remarks>
        /// Key insight: UV data is stored per triangle corner, not per vertex.
        /// When a vertex index changes during collapse, the corner UV stays correct.
        /// We simply backup trianglesEx before collapse and restore it after (filtering deleted triangles).
        /// </remarks>
        /// <param name="positions">Vertex positions (modified in-place by edge collapser)</param>
        /// <param name="triangles">Index triplets (cleared and rebuilt)</param>
        /// <param name="trianglesEx">Per-triangle UV/normal/group data (cleared and rebuilt)</param>
        private void CleanMesh(
            List<Rat3Hybrid> positions,
            List<Tri> triangles,
            List<MeshTriangle<TriangleVertexNormalUV>> trianglesEx,
            HashSet<int> affineUvGroups)
        {
            // ═══════════════════════════════════════════════════════════════════════════════
            // STEP 1: Identify vertices that CANNOT be collapsed
            // These are vertices where UV or normals differ between adjacent triangles
            // (e.g., UV seam = same position but different texture coords on each side)
            // ═══════════════════════════════════════════════════════════════════════════════
            bool[] vertexMustBePreserved = DetectPreservedVertices(positions, triangles, trianglesEx);

            var originalTriangles = new List<Tri>(triangles);
            var originalTrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>(trianglesEx);
            var vertexDataByGroupAndVertex = BuildVertexDataByGroupAndVertex(triangles, trianglesEx);

            // ═══════════════════════════════════════════════════════════════════════════════
            // STEP 3: Convert to EdgeCollapser's format (strips UV/normal, keeps groupId)
            // The collapser only cares about topology (vertex indices) and group boundaries
            // ═══════════════════════════════════════════════════════════════════════════════
            var trianglesWithGroups = new List<Remeshing.TriWithGroupId>(triangles.Count);
            for (int i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                var triEx = trianglesEx[i];
                trianglesWithGroups.Add(new Remeshing.TriWithGroupId
                {
                    A = tri.A,
                    B = tri.B,
                    C = tri.C,
                    GroupId = triEx.GroupId
                });
            }

            // ═══════════════════════════════════════════════════════════════════════════════
            // STEP 4: Run edge collapse algorithm
            // - minDist=0: only collapse vertices at EXACT same position (duplicates)
            // - allowVertexRelocation=false: don't move vertices, just merge
            // - removeCollinearEdges=true: collapse edges where middle vertex is collinear
            // - vertexMustBePreserved: prevents collapsing UV seams / sharp edges
            // ═══════════════════════════════════════════════════════════════════════════════
            // Planar fusion has already rebuilt these groups from one affine UV
            // frame. They require no per-collapse parameter validation.
            Func<Dictionary<int, Remeshing.FullTriangle>, bool> validateUv = null;
            for (int i = 0; i < trianglesEx.Count; i++)
            {
                if (!affineUvGroups.Contains(trianglesEx[i].GroupId))
                {
                    validateUv = new AffineUvCollapseGuard(positions, originalTriangles,
                        originalTrianglesEx, affineUvGroups).Validate;
                    break;
                }
            }
            Remeshing.EdgeCollapser.CleanMesh(
                positions,
                trianglesWithGroups,
                minDist: BigRationalHybrid.Zero,
                allowVertexRelocation: false,
                removeCollinearEdges: true,
                validateCollapseCallback: validateUv,
                vertexMustBePreserved: vertexMustBePreserved);

            // ═══════════════════════════════════════════════════════════════════════════════
            // STEP 5: Rebuild output lists from collapsed mesh
            // - Skip deleted triangles (marked with A < 0)
            // - Use ORIGINAL trianglesEx for UV/normal - corner data is still correct!
            // ═══════════════════════════════════════════════════════════════════════════════
            triangles.Clear();
            trianglesEx.Clear();

            for (int i = 0; i < trianglesWithGroups.Count; i++)
            {
                var triWithGroup = trianglesWithGroups[i];

                // A < 0 means triangle was deleted (degenerate after collapse)
                if (triWithGroup.A >= 0)
                {
                    // Use new vertex indices from collapsed mesh
                    triangles.Add(new Tri(triWithGroup.A, triWithGroup.B, triWithGroup.C));

                    var oldTri = originalTriangles[i];
                    var oldTriEx = originalTrianglesEx[i];
                    trianglesEx.Add(new MeshTriangle<TriangleVertexNormalUV>
                    {
                        GroupId = triWithGroup.GroupId,
                        V0 = GetCollapsedVertexData(triWithGroup.GroupId, triWithGroup.A, oldTri, oldTriEx, vertexDataByGroupAndVertex),
                        V1 = GetCollapsedVertexData(triWithGroup.GroupId, triWithGroup.B, oldTri, oldTriEx, vertexDataByGroupAndVertex),
                        V2 = GetCollapsedVertexData(triWithGroup.GroupId, triWithGroup.C, oldTri, oldTriEx, vertexDataByGroupAndVertex)
                    });
                }
            }
        }

        private Dictionary<(int GroupId, int VertexId), TriangleVertexNormalUV> BuildVertexDataByGroupAndVertex(
            List<Tri> triangles,
            List<MeshTriangle<TriangleVertexNormalUV>> trianglesEx)
        {
            var result = new Dictionary<(int GroupId, int VertexId), TriangleVertexNormalUV>();
            for (int i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                var triEx = trianglesEx[i];
                result.TryAdd((triEx.GroupId, tri.A), triEx.V0);
                result.TryAdd((triEx.GroupId, tri.B), triEx.V1);
                result.TryAdd((triEx.GroupId, tri.C), triEx.V2);
            }
            return result;
        }

        private TriangleVertexNormalUV GetCollapsedVertexData(
            int groupId,
            int vertexId,
            Tri originalTri,
            MeshTriangle<TriangleVertexNormalUV> originalTriEx,
            Dictionary<(int GroupId, int VertexId), TriangleVertexNormalUV> vertexDataByGroupAndVertex)
        {
            if (originalTri.A == vertexId)
                return originalTriEx.V0;
            if (originalTri.B == vertexId)
                return originalTriEx.V1;
            if (originalTri.C == vertexId)
                return originalTriEx.V2;

            if (vertexDataByGroupAndVertex.TryGetValue((groupId, vertexId), out var data))
                return data;

            throw new Exception("No vertex data found for collapsed mesh vertex");
        }

        /// <summary>
        /// Finds vertices that must NOT be collapsed because they carry discontinuous attributes.
        /// Detects: same vertex index with different UV/normals across triangles within same group.
        /// Example: UV seam where same vertex has UV (0,0) in one triangle and UV (1,0) in another.
        /// </summary>
        /// <returns>Boolean array indexed by vertex index. true = preserve, false = can collapse</returns>
        private bool[] DetectPreservedVertices(
            List<Rat3Hybrid> positions,
            List<Tri> triangles,
            List<MeshTriangle<TriangleVertexNormalUV>> trianglesEx)
        {
            bool[] mustPreserve = new bool[positions.Count];

            // ─────────────────────────────────────────────────────────────────────────────
            // PHASE 1: Build adjacency map
            // Structure: vertexIdx → groupId → list of triangle indices
            // This groups all triangles touching each vertex, subdivided by material group
            // ─────────────────────────────────────────────────────────────────────────────
            var trianglesPerVertexPerGroup = new Dictionary<int, Dictionary<int, List<int>>>();

            for (int i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                var triEx = trianglesEx[i];
                int groupId = triEx.GroupId;

                // Register this triangle for each of its 3 vertices
                foreach (int vertexIdx in new[] { tri.A, tri.B, tri.C })
                {
                    // Ensure nested dictionaries exist
                    if (!trianglesPerVertexPerGroup.ContainsKey(vertexIdx))
                        trianglesPerVertexPerGroup[vertexIdx] = new Dictionary<int, List<int>>();

                    if (!trianglesPerVertexPerGroup[vertexIdx].ContainsKey(groupId))
                        trianglesPerVertexPerGroup[vertexIdx][groupId] = new List<int>();

                    trianglesPerVertexPerGroup[vertexIdx][groupId].Add(i);  // store triangle index
                }
            }

            // ─────────────────────────────────────────────────────────────────────────────
            // PHASE 2: Check each vertex for attribute discontinuities
            // Within each group, compare UV/normal across all triangles sharing this vertex
            // If ANY differ → mark vertex as preserved (can't collapse without breaking seam)
            // ─────────────────────────────────────────────────────────────────────────────
            foreach (var vertexEntry in trianglesPerVertexPerGroup)
            {
                int vertexIdx = vertexEntry.Key;
                var groupsDict = vertexEntry.Value;  // groupId → triangles

                // Check each material group independently
                // (different groups having different UVs is expected and OK)
                foreach (var groupEntry in groupsDict)
                {
                    int groupId = groupEntry.Key;
                    var trianglesInGroup = groupEntry.Value;

                    // Single triangle at vertex in this group = nothing to compare
                    if (trianglesInGroup.Count <= 1)
                        continue;

                    // Use first triangle's data as reference
                    int firstTriIdx = trianglesInGroup[0];
                    var firstTri = triangles[firstTriIdx];
                    var firstTriEx = trianglesEx[firstTriIdx];
                    TriangleVertexNormalUV firstVertexData = GetVertexData(firstTri, firstTriEx, vertexIdx);

                    // Compare against all other triangles in SAME group
                    for (int i = 1; i < trianglesInGroup.Count; i++)
                    {
                        int triIdx = trianglesInGroup[i];
                        var tri = triangles[triIdx];
                        var triEx = trianglesEx[triIdx];
                        TriangleVertexNormalUV vertexData = GetVertexData(tri, triEx, vertexIdx);

                        // Mismatch detected → this vertex is a seam/sharp edge
                        if (!AreVertexDataEqual(firstVertexData, vertexData))
                        {
                            mustPreserve[vertexIdx] = true;
                            break;  // no need to check more triangles
                        }
                    }

                    if (mustPreserve[vertexIdx])
                        break;  // no need to check other groups
                }
            }

            return mustPreserve;
        }

        /// <summary>
        /// Compares UV and Normal with epsilon tolerance (eps ≈ 1e-5).
        /// Uses squared distance to avoid sqrt overhead.
        /// </summary>
        private bool AreVertexDataEqual(TriangleVertexNormalUV a, TriangleVertexNormalUV b)
        {
            const double epsSquared = 1e-10;  // (1e-5)² - anything within 0.00001 units is "equal"

            // UV check: |a.UV - b.UV|² ≤ eps²
            if (Vec2DOps.DistanceSquared(a.UV, b.UV) > epsSquared)
                return false;

            // Normal check: |a.Normal - b.Normal|² ≤ eps²
            if (Vec3DOps.DistanceSquared(a.Normal, b.Normal) > epsSquared)
                return false;

            return true;
        }

        /// <summary>
        /// Maps vertex index → per-vertex attribute data within a triangle.
        /// Triangles store V0/V1/V2, this finds which one matches the given vertexIdx.
        /// </summary>
        private TriangleVertexNormalUV GetVertexData(Tri tri, MeshTriangle<TriangleVertexNormalUV> triEx, int vertexIdx)
        {
            // tri.A/B/C are vertex indices, triEx.V0/V1/V2 are corresponding attributes
            if (tri.A == vertexIdx) return triEx.V0;  // first vertex
            if (tri.B == vertexIdx) return triEx.V1;  // second vertex
            if (tri.C == vertexIdx) return triEx.V2;  // third vertex
            throw new Exception("Vertex not found in triangle");  // should never happen
        }

        public string GetNameOfGroup(int groupId)
        {
            return groupIdToExtendedName[groupId];
        }

        /// <summary>
        /// Renames the mesh and updates surface-patch keys that use the previous name as a prefix
        /// (e.g. ExtrudeAlongCurve1-CircleYZ-CLine0 → pipe-CircleYZ-CLine0).
        /// </summary>
        public void Rename(string newName)
        {
            if (string.IsNullOrEmpty(newName) || newName == Name)
                return;

            MeshNameEvictor?.Invoke(newName);

            string oldName = Name;
            Name = newName;
            RenameExtendedNamesPrefix(oldName, newName);
        }

        private void RenameExtendedNamesPrefix(string oldPrefix, string newPrefix)
        {
            string RenameAtom(string value) => EntityNaming.RewriteMeshNameInEntity(value, oldPrefix, newPrefix);
            FaceLineages = FaceLineages.ToDictionary(pair => pair.Key, pair => pair.Value.Remap(RenameAtom));
            AmbiguousFaceReferences = AmbiguousFaceReferences.Select(RenameAtom).ToHashSet(StringComparer.Ordinal);

            extendedNameToGroupId = EntityNaming.RewriteDictionaryKeys(
                extendedNameToGroupId,
                key => EntityNaming.RewriteMeshNameInEntity(key, oldPrefix, newPrefix));

            groupIdToExtendedName = EntityNaming.RewriteDictionaryValues(
                groupIdToExtendedName,
                value => EntityNaming.RewriteMeshNameInEntity(value, oldPrefix, newPrefix));

            surfaceMetaData = EntityNaming.RewriteDictionaryKeys(
                surfaceMetaData,
                key => EntityNaming.RewriteMeshNameInEntity(key, oldPrefix, newPrefix));

            if (GroupEdges == null)
                return;

            for (int i = 0; i < GroupEdges.Count; i++)
                GroupEdges[i].Name = EntityNaming.RewriteMeshNameInEntity(GroupEdges[i].Name, oldPrefix, newPrefix);
        }

        internal void RewriteEntityPrefixForCopy(string oldPrefix, string newPrefix)
        {
            if (string.IsNullOrEmpty(oldPrefix) || oldPrefix == newPrefix)
                return;
            RenameExtendedNamesPrefix(oldPrefix, newPrefix);
        }

        //public List<Named<Vec3D>> GetDefaultAnchorPoints()
        //{

        //}

        //Returns the indices where the first triangle appears with a groupId different from the previous triangle
        //public List<int> GetTriangleGroupChangeIndices()
        //{
        //    List<int> changeIndices = new List<int>();

        //    if (Mesh.TrianglesEx.Count == 0)
        //        return changeIndices;

        //    // First triangle is always a change point (start of first group)
        //    changeIndices.Add(0);

        //    // Find all indices where GroupId changes
        //    for (int i = 1; i < Mesh.TrianglesEx.Count; i++)
        //    {
        //        int currentGroupId = Mesh.TrianglesEx[i].GroupId;
        //        int previousGroupId = Mesh.TrianglesEx[i - 1].GroupId;

        //        if (currentGroupId != previousGroupId)
        //        {
        //            changeIndices.Add(i);
        //        }
        //    }

        //    return changeIndices;
        //}

        //public List<Tri> GetTriangleRange(int start, int count)
        //{
        //    if (start < 0 || start + count > Mesh.Triangles.Count)
        //        throw new ArgumentOutOfRangeException("Invalid start or count for triangle range.");
        //    return Mesh.Triangles.GetRange(start, count);
        //}

        //private void SortTrianglesByGroupId()
        //{
        //    if (Mesh.TrianglesEx.Count != Mesh.Triangles.Count)
        //        throw new InvalidOperationException("Triangle arrays have mismatched lengths");

        //    // Create array of indices to sort
        //    int[] indices = new int[Mesh.TrianglesEx.Count];
        //    for (int i = 0; i < indices.Length; i++)
        //        indices[i] = i;

        //    // Sort indices based on GroupId using Array.Sort with custom comparison
        //    Array.Sort(indices, (i1, i2) => Mesh.TrianglesEx[i1].GroupId.CompareTo(Mesh.TrianglesEx[i2].GroupId));

        //    // Create new sorted arrays
        //    var sortedTriangles = new List<Tri>(Mesh.Triangles.Count);
        //    var sortedTrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>(Mesh.TrianglesEx.Count);
        //    //var sortedTriangleMarker = new List<int>(Mesh.TriangleMarker.Count);

        //    // Reorder all triangle-related arrays based on sorted indices
        //    for (int i = 0; i < indices.Length; i++)
        //    {
        //        int originalIndex = indices[i];
        //        sortedTriangles.Add(Mesh.Triangles[originalIndex]);
        //        sortedTrianglesEx.Add(Mesh.TrianglesEx[originalIndex]);
        //        //sortedTriangleMarker.Add(Mesh.TriangleMarker[originalIndex]);
        //    }

        //    // Replace the original arrays with sorted ones
        //    Mesh.Triangles = sortedTriangles;
        //    Mesh.TrianglesEx = sortedTrianglesEx;
        //    //Mesh.TriangleMarker = sortedTriangleMarker;
        //}

        // A surface patch anchor
        public UVSurface GetSurfacePatch(int groupId)
        {
            Mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out var perTriangleGroup);

            if (triangles.Count != Mesh.TrianglesEx.Count)
                throw new Exception();

            List<Tri> result = new List<Tri>();
            for (int i = 0; i < Mesh.TrianglesEx.Count; i++)
            {
                var triEx = Mesh.TrianglesEx[i];
                if (triEx.GroupId == groupId)
                {
                    result.Add(triangles[i]);
                }
            }
            return new UVSurface(positions, normals, uvs, result);
        }

        // Edge (line strip) anchor
        /// <summary>
        /// Infers one outward normal per strip point from the two surface groups adjacent to an
        /// anchor edge. Each side is averaged independently, then the two sides are combined with
        /// equal weight so tessellation density does not skew the edge normal.
        /// </summary>
        public List<Vec3D> InferNormalsForEdgeStrip(GroupEdge edge, LineStrip3D strip)
        {
            var sums = new Dictionary<(double X, double Y, double Z, int GroupId), Vec3D>();

            void Add(int vertexIndex, int groupId, Vec3D normal)
            {
                Vec3D p = Mesh.Positions[vertexIndex];
                var key = (p.X, p.Y, p.Z, groupId);
                sums.TryGetValue(key, out Vec3D sum);
                sums[key] = sum + normal;
            }

            for (int i = 0; i < Mesh.Triangles.Count; i++)
            {
                var triEx = Mesh.TrianglesEx[i];
                int groupId = triEx.GroupId;
                if (groupId != edge.GroupIdA && groupId != edge.GroupIdB)
                    continue;

                var tri = Mesh.Triangles[i];
                Add(tri.A, groupId, triEx.V0.Normal);
                Add(tri.B, groupId, triEx.V1.Normal);
                Add(tri.C, groupId, triEx.V2.Normal);
            }

            var result = new List<Vec3D>(strip.Points.Count);
            foreach (Vec3D point in strip.Points)
            {
                Vec3D combined = default;
                if (sums.TryGetValue((point.X, point.Y, point.Z, edge.GroupIdA), out Vec3D a) &&
                    a.LengthSquared() > 1e-20)
                    combined += a.Normalized();
                if (sums.TryGetValue((point.X, point.Y, point.Z, edge.GroupIdB), out Vec3D b) &&
                    b.LengthSquared() > 1e-20)
                    combined += b.Normalized();
                result.Add(combined.LengthSquared() > 1e-20 ? combined.Normalized() : default);
            }
            return result;
        }

        /// <summary>Arc-length interpolates and renormalizes per-strip normals.</summary>
        public static Vec3D EvaluateStripNormal(LineStrip3D strip, IList<Vec3D> normals, double uniform)
        {
            if (normals.Count != strip.Points.Count || normals.Count == 0)
                return default;

            strip.EvaluateUniform(uniform, out double index);
            if (index <= 0)
                return normals[0];
            if (index >= normals.Count - 1)
                return normals[^1];

            int i = (int)Math.Floor(index);
            double w = index - i;
            Vec3D normal = normals[i] * (1.0 - w) + normals[i + 1] * w;
            return normal.LengthSquared() > 1e-20 ? normal.Normalized() : default;
        }

        public LineStrip3D GetEdge(int groupIdA, int groupIdB, int edgeIndex)
            => GetEdgeCore(groupIdA, groupIdB, edgeIndex, false);

        private LineStrip3D GetEdgeCore(int groupIdA, int groupIdB, int edgeIndex, bool localGeometry)
        {
            // Find all GroupEdges between these two groups
            var matchingEdges = new List<GroupEdge>();
            for(int i = 0; i < GroupEdges.Count; i++)
            {
                var ge = GroupEdges[i];
                if ((ge.GroupIdA == groupIdA && ge.GroupIdB == groupIdB) ||
                    (ge.GroupIdA == groupIdB && ge.GroupIdB == groupIdA))
                {
                    matchingEdges.Add(ge);
                }
            }

            // With the new extraction logic, each GroupEdge represents one connected edge component
            // The edges are already sorted spatially during extraction
            if (edgeIndex < matchingEdges.Count)
            {
                var ge = matchingEdges[edgeIndex];
                // Each GroupEdge should now have exactly one LineStrip3D
                if (ge.LineStrips3D.Count > 0)
                    return EdgeGeometry(ge, localGeometry);
            }

            return null;
        }

        // Point anchor
        public Vec3D GetPointOnEdge(int groupIdA, int groupIdB, int edgeIndex, double uniformParam, out string name)
        {
            name = EntityNaming.FormatLegacyEdgePointName(
                groupIdToExtendedName[groupIdA],
                groupIdToExtendedName[groupIdB],
                edgeIndex,
                uniformParam);

            var edge = GetEdge(groupIdA, groupIdB, edgeIndex);
            return edge.EvaluateUniform(uniformParam);
        }


        //public Vec3D GetPointOnEdge(string name)
        //{
        //    string[] parts = name.Split('_');
        //    int groupIdA = extendedNameToGroupId[parts[0]];
        //    int groupIdB = extendedNameToGroupId[parts[1]];
        //    int edgeIndex = int.Parse(parts[2]);
        //    double uniformParam = double.Parse(parts[3]);

        //    string verify;
        //    var result = GetPointOnEdge(groupIdA, groupIdB, edgeIndex, uniformParam, out verify);

        //    if (verify != name)
        //        throw new Exception("AnchorMesh.GetPointOnEdge: internal error in name generation.");

        //    return result;
        //}

        public bool TryGetPointOnSurface(string name, out Vec3D point)
        {
            return TryGetPointOnSurface(name, out point, out _, out _, out _, out _);
        }

        public bool TryGetPointOnSurface(string name, out Vec3D point, out Vec3D normal,
            out Vec2D uv, out Vec3D tangentX, out Vec3D tangentY)
            => TryGetPointOnSurfaceCore(name, out point, out normal, out uv, out tangentX, out tangentY, false);

        internal bool TryGetLocalPointOnSurface(string name, out Vec3D point)
            => TryGetPointOnSurfaceCore(name, out point, out _, out _, out _, out _, true);

        private bool TryGetPointOnSurfaceCore(string name, out Vec3D point, out Vec3D normal,
            out Vec2D uv, out Vec3D tangentX, out Vec3D tangentY, bool localGeometry)
        {
            point = default;
            if (!EntityNaming.TryParseSurfacePointAddress(name, out var address))
            {
                normal = default;
                uv = default;
                tangentX = default;
                tangentY = default;
                return false;
            }

            ValidateEntityReference(name);
            if (!extendedNameToGroupId.TryGetValue(address.PatchName, out int groupId))
            {
                normal = default;
                uv = default;
                tangentX = default;
                tangentY = default;
                return false;
            }

            UVSurface surface;
            if (localGeometry)
                TryGetLocalSurface(address.PatchName, out surface);
            else
                surface = GetSurfacePatch(groupId);
            return surface.Evaluate(address.UniformX, address.UniformY, out point, out normal, out uv, out tangentX, out tangentY);
        }


        public bool TryGetPointOnEdge(string name, out Vec3D point)
            => TryGetPointOnEdgeCore(name, out point, false);

        internal bool TryGetLocalPointOnEdge(string name, out Vec3D point)
            => TryGetPointOnEdgeCore(name, out point, true);

        private bool TryGetPointOnEdgeCore(string name, out Vec3D point, bool localGeometry)
        {
            point = default;
            if (TryResolveEdgePointAddress(name, out var address))
                return TryEvaluateEdgePoint(address, out point, localGeometry);
            return false;
        }

        private bool TryResolveEdgePointAddress(string name, out EntityNaming.EdgePointAddress address)
        {
            if (EntityNaming.TryParseEdgePointAddress(name, out address))
                return true;
            return EntityNaming.TryParseLegacyEdgePointAddress(name, extendedNameToGroupId.Keys, out address);
        }

        private bool TryEvaluateEdgePoint(EntityNaming.EdgePointAddress address, out Vec3D point, bool localGeometry = false)
        {
            point = default;
            if (!extendedNameToGroupId.TryGetValue(address.PatchA, out int groupIdA) ||
                !extendedNameToGroupId.TryGetValue(address.PatchB, out int groupIdB))
                return false;

            LineStrip3D edge = GetEdgeCore(groupIdA, groupIdB, address.EdgeIndex, localGeometry);
            if (edge == null)
                return false;
            point = edge.EvaluateUniform(address.Uniform);
            return true;
        }

        public bool TryGetEdge(string name, out LineStrip3D result)
            => TryGetEdgeCore(name, out result, false);

        internal bool TryGetLocalEdge(string name, out LineStrip3D result)
            => TryGetEdgeCore(name, out result, true);

        private LineStrip3D EdgeGeometry(GroupEdge edge, bool localGeometry)
            => localGeometry && _rigidBodyActive ? _rigidRestEdges[edge] : edge.LineStrips3D[0];

        private bool TryGetEdgeCore(string name, out LineStrip3D result, bool localGeometry)
        {
            ValidateEntityReference(name);
            name = name.Trim();
            result = default;
            if (GroupEdges == null)
                return false;

            for (int i = 0; i < GroupEdges.Count; i++)
            {
                var ge = GroupEdges[i];
                if (!EntityNaming.MatchesGroupEdgeName(ge.Name, name))
                    continue;
                if (ge.LineStrips3D.Count > 0)
                {
                    result = EdgeGeometry(ge, localGeometry);
                    return true;
                }
            }

            return false;
        }
        /// <summary>Extracts a patch with per-corner UV/normal seams and exact source positions.</summary>
        public bool TryGetSurface(string name, out UVSurface result) => TryGetSurfaceCore(name, out result, false, true);

        // Geometric algorithms operate on parent-mesh vertex identities. Blend
        // extension and fitted export boundaries require this welded topology.
        internal bool TryGetTopologySurface(string name, out UVSurface result) => TryGetSurfaceCore(name, out result, false, false);

        // Assembly datums refer to the captured body-local geometry, including
        // when another occurrence has already updated this shared mesh's pose.
        internal bool TryGetLocalSurface(string name, out UVSurface result) => TryGetSurfaceCore(name, out result, true, false);

        private bool TryGetSurfaceCore(string name, out UVSurface result, bool localGeometry, bool preserveCornerAttributes)
        {
            result = null;
            if (!extendedNameToGroupId.ContainsKey(name))
            {
                if (!TryResolveCurrentFaceReference(name, out name))
                    return false;
            }

            int groupId = extendedNameToGroupId[name];

            if (preserveCornerAttributes)
            {
                var precise = localGeometry && _rigidRestExact != null ? _rigidRestExact.ToList() : Mesh.PrecisionPositions;
                var positions = localGeometry && _rigidConverterSet ? _rigidConverter.Convert(precise) : Mesh.Positions;
                var patch = new MeshNormalUV
                {
                    Positions = positions,
                    PrecisionPositions = precise,
                    Triangles = new List<Tri>(),
                    TrianglesEx = new()
                };
                for (int i = 0; i < Mesh.Triangles.Count; i++)
                {
                    if (Mesh.TrianglesEx[i].GroupId != groupId)
                        continue;
                    patch.Triangles.Add(Mesh.Triangles[i]);
                    patch.TrianglesEx.Add(Mesh.TrianglesEx[i]);
                }

                // A geometric vertex can carry distinct UVs or normals in adjacent
                // corners. Decomposition preserves those seams and their exact source
                // coordinates instead of allowing the last corner to overwrite them.
                List<Rat3Hybrid> patchPrecise = null;
                List<Vec3D> patchPositions, normals;
                List<Vec2D> uv;
                List<Tri> triangles;
                if (precise == null)
                    patch.Decompose(out patchPositions, out normals, out uv, out triangles, out _);
                else
                    patch.Decompose(out patchPositions, out normals, out uv, out triangles, out _, out patchPrecise);
                result = new UVSurface(patchPositions, normals, uv, triangles, patchPrecise);
            }
            else
            {
                int vertexCount = Mesh.Positions.Count;

                List<Tri> triangles = new List<Tri>();
                List<Vec3D> normals = EmptyList<Vec3D>(vertexCount);
                List<Vec2D> uv = EmptyList<Vec2D>(vertexCount);
                for (int i=0;i<Mesh.Triangles.Count;++i)
                {
                    var triEx = Mesh.TrianglesEx[i];
                    if(triEx.GroupId == groupId)
                    {
                        var tri = Mesh.Triangles[i];
                        triangles.Add(tri);

                        normals[tri.A] = triEx.V0.Normal;
                        normals[tri.B] = triEx.V1.Normal;
                        normals[tri.C] = triEx.V2.Normal;

                        uv[tri.A] = triEx.V0.UV;
                        uv[tri.B] = triEx.V1.UV;
                        uv[tri.C] = triEx.V2.UV;
                    }
                }

                // Using the full size vertex lists is a bit wasteful but it does not modify vertex indexing so it is worth it
                var precise = localGeometry && _rigidRestExact != null ? _rigidRestExact.ToList() : Mesh.PrecisionPositions;
                var positions = localGeometry && _rigidConverterSet ? _rigidConverter.Convert(precise) : Mesh.Positions;
                result = new UVSurface(positions, normals, uv, triangles, precise);
            }
            // Local face datums need authoritative geometry, not transformed
            // analytic display metadata or lazy NURBS materialization.
            if (!localGeometry && surfaceMetaData != null && surfaceMetaData.TryGetValue(name, out var meta))
            {
                result.NurbsSurface = meta.NurbsSurface;
                result.NurbsParamRange = meta.ParamRange;
            }

            return true;

            throw new NotImplementedException();

            //result = default;
            //Regex r = new Regex(@"\[(?<surfA>[^,]+),(?<surfB>[^]]+)\](_(?<index>\d+))?");
            //Match match = r.Match(name);
            //if (!match.Success)
            //    return false;

            //int groupIdA = int.Parse(match.Groups["surfA"].Value);
            //int groupIdB = int.Parse(match.Groups["surfB"].Value);
            //int edgeIndex = 0;
            //var g = match.Groups["index"];
            //if (g.Success)
            //    edgeIndex = int.Parse(g.Value);

            //result = GetEdge(groupIdA, groupIdB, edgeIndex);

            //return true;
        }

        private List<T> EmptyList<T>(int vertexCount)
        {
           List<T> result = new List<T>(vertexCount);
            for (int i = 0; i < vertexCount; i++)
                result.Add(default(T));
            return result;
        }

        public void ApplyTranslation(Vec3D vec3D, CoordinateConverter c)
        {
            Mesh.ApplyTranslation(vec3D, c);
            var t = Mat4DOps.Identity();
            t.M14 = vec3D.X;
            t.M24 = vec3D.Y;
            t.M34 = vec3D.Z;
            TransformSurfaceMeta(in t);
            for (int i = 0; i < GroupEdges.Count; ++i)
                GroupEdges[i].UpdatePositions(Mesh.Positions);
        }

        internal void CaptureRigidRestPose(CoordinateConverter converter)
        {
            // Reusing a solid must retain its original local geometry, even
            // when an earlier occurrence has already updated the shared mesh.
            if (_rigidBodyActive)
                return;
            EnsureCoplanarPostProcessed();

            int count = Mesh.Positions.Count;
            _rigidRestLocal = new Vec3D[count];
            _rigidRestExact = Mesh.PrecisionPositions.ToArray();
            _rigidRestCorners = Mesh.TrianglesEx.ToArray();
            _rigidDisplayNormalsCurrent = false;
            _rigidConverter = converter;
            _rigidConverterSet = true;

            // Body-local frame matches the mesh as built; assembly pose is applied via Update().
            for (int i = 0; i < count; i++)
                _rigidRestLocal[i] = Mesh.Positions[i];

            _rigidRestMeta = SurfaceMetaData.CloneDictionary(surfaceMetaData);
            // Edge addresses retain their original phase when display poses change.
            // Capture exact body-local strips once rather than undoing world transforms.
            _rigidRestEdges = GroupEdges.ToDictionary(edge => edge, edge =>
            {
                // Some display edges only carry floating coordinates. Reconstruct their
                // exact strip from mesh indices, retaining the established phase and winding.
                var local = new GroupEdge(edge.Name, edge.GroupIdA, edge.GroupIdB,
                    edge.EdgeSegments, null, null, edge.PreferLexClosedLoopStart)
                {
                    ClosedLoopZeroFrame = edge.ClosedLoopZeroFrame,
                    PreferredClosedLoopStartVertex = edge.PreferredClosedLoopStartVertex,
                    PreferredClosedLoopNextVertex = edge.PreferredClosedLoopNextVertex
                };
                local.UpdateExactPositions(Mesh.Positions, Mesh.PrecisionPositions);
                return new LineStrip3D(local.LineStripsExact[0].Select(point => converter.Convert(point)).ToList());
            });
            _rigidBodyActive = true;
        }

        public MeshNormalUV SnapshotRigidPose(Transform transform, int? groupId = null)
        {
            if (!_rigidBodyActive || _rigidRestExact == null)
                throw new InvalidOperationException($"Mesh '{Name}' is not an assembly body.");
            return Mesh.SnapshotRigidPose(_rigidRestExact, _rigidConverter, transform, groupId, _rigidRestCorners);
        }

        /// <summary>Copy a body definition with its original display and exact coordinates.</summary>
        internal AnchorMesh SnapshotRigidDefinition(string name)
        {
            var identity = new Transform(new Vec3D(0), TransformMath.IdentityOrientation);
            var mesh = SnapshotRigidPose(identity);
            // Inspection snapshots deliberately display the precision topology.
            // A copied definition must retain the authored continuous display geometry too.
            mesh.Positions = new List<Vec3D>(_rigidRestLocal);
            return new AnchorMesh(name, mesh, new Dictionary<int, string>(groupIdToExtendedName),
                SurfaceMetaData.CloneDictionary(_rigidRestMeta), deferCoplanarPostProcess: true,
                isVolume: IsVolume, preserveTriangulation: true,
                faceLineages: FaceLineages, ambiguousReferences: AmbiguousFaceReferences);
        }

        // Use rest geometry even if a prior display updated the shared body.
        internal AnchorMesh SnapshotForInspection(CoordinateConverter converter, Transform pose, string name)
        {
            CoordinateConverter geometryConverter = _rigidBodyActive && _rigidConverterSet
                ? _rigidConverter : converter;
            var mesh = _rigidBodyActive ? SnapshotRigidPose(pose)
                : Mesh.SnapshotRigidPose(Mesh.PrecisionPositions, geometryConverter, pose);
            mesh = mesh.Rebase(geometryConverter, converter);
            var metadata = SurfaceMetaData.CloneDictionary(_rigidBodyActive ? _rigidRestMeta : surfaceMetaData);
            var matrix = TransformMath.ToMat4D(in pose);
            foreach (var item in metadata.Values) item.Transform(in matrix);
            return new AnchorMesh(name, mesh, new Dictionary<int, string>(groupIdToExtendedName),
                metadata, deferCoplanarPostProcess: true, isVolume: IsVolume, preserveTriangulation: true,
                faceLineages: FaceLineages, ambiguousReferences: AmbiguousFaceReferences);
        }

        public void Update(Transform transform)
        {
            if (!_rigidBodyActive || _rigidRestLocal == null || !_rigidConverterSet)
                throw new InvalidOperationException($"Mesh '{Name}' is not registered as a rigid assembly body.");

            Mat4D worldFromLocal = TransformMath.ToMat4D(in transform);
            var preciseTransform = new PreciseRigidTransform(_rigidConverter, in transform);

            for (int i = 0; i < _rigidRestLocal.Length; i++)
            {
                Vec3D world = worldFromLocal.TransformPoint(_rigidRestLocal[i]);
                Mesh.Positions[i] = world;
                Mesh.PrecisionPositions[i] = preciseTransform.Apply(_rigidRestExact[i]);
            }

            // Solving mates often changes only translation. Recompute normals from
            // the rest pose only when the shared display mesh changes orientation.
            Quaternion orientation = TransformMath.NormalizeDefault(transform.Orientation);
            if (!_rigidDisplayNormalsCurrent || orientation.X != _rigidDisplayOrientation.X
                || orientation.Y != _rigidDisplayOrientation.Y || orientation.Z != _rigidDisplayOrientation.Z
                || orientation.W != _rigidDisplayOrientation.W)
            {
                for (int i = 0; i < _rigidRestCorners.Length; i++)
                {
                    var corner = _rigidRestCorners[i];
                    corner.V0.Normal = worldFromLocal.TransformDirection(corner.V0.Normal);
                    corner.V1.Normal = worldFromLocal.TransformDirection(corner.V1.Normal);
                    corner.V2.Normal = worldFromLocal.TransformDirection(corner.V2.Normal);
                    Mesh.TrianglesEx[i] = corner;
                }
                _rigidDisplayOrientation = orientation;
                _rigidDisplayNormalsCurrent = true;
            }

            RestoreSurfaceMetaFromRest(in worldFromLocal);

            for (int i = 0; i < GroupEdges.Count; ++i)
                GroupEdges[i].UpdatePositions(Mesh.Positions);
        }

        private void TransformSurfaceMeta(in Mat4D t)
        {
            if (surfaceMetaData == null)
                return;
            foreach (var kv in surfaceMetaData)
                kv.Value.Transform(in t);
        }

        private void RestoreSurfaceMetaFromRest(in Mat4D t)
        {
            if (_rigidRestMeta == null)
                return;
            surfaceMetaData = SurfaceMetaData.CloneDictionary(_rigidRestMeta);
            foreach (var kv in surfaceMetaData)
                kv.Value.Transform(in t);
        }

        public bool TryGetCylinderFromPatch(string patchName, out CylinderSurfaceParams cylinder)
            => TryGetCylinderFromPatchCore(patchName, localGeometry: false, out cylinder);

        // Assembly datums are body-local. Rendering updates surfaceMetaData to
        // the current world pose, so datum resolution must use the captured rest
        // metadata instead of feeding world coordinates back into the solver.
        internal bool TryGetLocalCylinderFromPatch(string patchName, out CylinderSurfaceParams cylinder)
            => TryGetCylinderFromPatchCore(patchName, localGeometry: true, out cylinder);

        private bool TryGetCylinderFromPatchCore(
            string patchName,
            bool localGeometry,
            out CylinderSurfaceParams cylinder)
        {
            cylinder = null;
            string localName = ResolveLocalPatchName(patchName);
            if (localName == null)
                return false;
            var metadata = localGeometry && _rigidBodyActive ? _rigidRestMeta : surfaceMetaData;
            if (metadata == null || !metadata.TryGetValue(localName, out SurfaceMetaData meta))
                return false;
            if (meta.SurfaceType != SurfaceType.Cylindrical || meta.CylinderParams == null)
                return false;
            cylinder = meta.CylinderParams;
            return true;
        }

        public bool TryGetPlaneFromPatch(string patchName, out PlaneSurfaceParams plane)
            => TryGetPlaneFromPatchCore(patchName, false, out plane);

        internal bool TryGetLocalPlaneFromPatch(string patchName, out PlaneSurfaceParams plane)
            => TryGetPlaneFromPatchCore(patchName, true, out plane);

        private bool TryGetPlaneFromPatchCore(string patchName, bool localGeometry, out PlaneSurfaceParams plane)
        {
            plane = null;
            string localName = ResolveLocalPatchName(patchName);
            if (localName == null)
                return false;
            var metadata = localGeometry && _rigidBodyActive ? _rigidRestMeta : surfaceMetaData;
            if (metadata.TryGetValue(localName, out SurfaceMetaData meta) &&
                meta.SurfaceType == SurfaceType.Planar &&
                meta.PlaneParams != null)
            {
                plane = meta.PlaneParams;
                return true;
            }

            if (!TryGetSurfaceCore(localName, out UVSurface surface, localGeometry, false) || !surface.IsPlanar())
                return false;

            Vec3D center = surface.ApproximatePlanarSurfaceCenter(out Vec3D normal, out _, out _);
            normal.Normalize();
            plane = new PlaneSurfaceParams
            {
                Origin = center,
                Normal = normal,
                RefDir = Vec3DOps.Cross(normal, new Vec3D(0, 1, 0))
            };
            if (plane.RefDir.LengthSquared() < 1e-12)
                plane.RefDir = Vec3DOps.Cross(normal, new Vec3D(1, 0, 0));
            plane.RefDir.Normalize();
            return true;
        }

        internal bool TryGetLocalPlaneReferenceDirection(string reference, out Vec3D direction)
        {
            direction = default;
            string name = ResolveLocalPatchName(reference);
            var metadata = _rigidBodyActive ? _rigidRestMeta : surfaceMetaData;
            if (name == null || metadata == null || !metadata.TryGetValue(name, out var face) ||
                face?.PlaneParams == null)
                return false;
            direction = face.PlaneParams.RefDir;
            return true;
        }

        internal void ValidateEntityReference(string reference)
        {
            if (string.IsNullOrWhiteSpace(reference)) return;
            reference = reference.Trim();
            // Renderer and public name queries expose current patch identities
            // as ``mesh:patch``. A Boolean result may itself have the same mesh
            // name as the first operand, so do not mistake an exact current
            // patch key for a mesh qualification and strip its leading scope.
            if (extendedNameToGroupId.ContainsKey(reference)) return;
            if (TryResolveCurrentFaceReference(reference, out _)) return;
            if (reference.StartsWith(Name + ":", StringComparison.Ordinal))
            {
                reference = reference.Substring(Name.Length + 1);
                // The renderer/API form is mesh:current-patch. After removing
                // the mesh scope, recognize that exact current key before
                // treating the same text as an ambiguous split ancestor.
                if (extendedNameToGroupId.ContainsKey(reference) ||
                    TryResolveCurrentFaceReference(reference, out _)) return;
            }
            if (TryResolveCurrentFaceReference(reference, out _)) return;
            if (EntityNaming.TryParseSurfacePointAddress(reference, out var surface))
                Check(surface.PatchName);
            else if (EntityNaming.TryParseGroupEdgeAddress(reference, out var edge))
            {
                Check(edge.PatchA);
                Check(edge.PatchB);
            }
            else Check(reference);

            void Check(string patch)
            {
                if (!AmbiguousFaceReferences.Contains(patch))
                    return;
                var current = CurrentFacesMatching(patch);
                if (current.Count > 1)
                    throw new GeoMeta.NameCollisionException(
                        $"Face reference '{patch}' is ambiguous among current faces: {string.Join(", ", current)}.");
                throw new GeoMeta.NameCollisionException($"Face reference '{patch}' has ambiguous or obsolete split ancestry; use a current provenance-qualified face name.");
            }
        }

        private void RefreshFusedLineageNames()
        {
            foreach (var group in FaceLineages.GroupBy(pair => pair.Value.Reference))
            {
                if (group.Count() != 1 || AmbiguousFaceReferences.Contains(group.Key)) continue;
                var pair = group.Single();
                var lineage = pair.Value;
                if (!lineage.Supported || (lineage.Roots.Count < 2 && !lineage.Split)) continue;
                string previous = groupIdToExtendedName[pair.Key];
                if (previous == lineage.Reference) continue;
                // Fusion publishes one combined identity. Each original name stays
                // an alias while it still identifies this single face.
                if (groupIdToExtendedName.Any(item => item.Key != pair.Key && item.Value == lineage.Reference)) continue;
                groupIdToExtendedName[pair.Key] = lineage.Reference;
                if (surfaceMetaData.TryGetValue(previous, out var metadata))
                    surfaceMetaData[lineage.Reference] = metadata?.Clone();
                if (!lineage.Roots.Contains(previous))
                    AmbiguousFaceReferences.Add(previous);
            }
        }

        private void ValidateLineageReferences(List<int> groups)
        {
            var active = groups.Distinct().Where(FaceLineages.ContainsKey).ToArray();
            foreach (var byReference in active.GroupBy(id => FaceLineages[id].Reference))
            {
                bool ambiguous = byReference.Count() > 1 || byReference.Any(id => !FaceLineages[id].Supported);
                foreach (int id in byReference)
                {
                    var lineage = FaceLineages[id];
                    if (!lineage.Split) continue;
                    foreach (var root in lineage.Roots) AmbiguousFaceReferences.Add(root);
                    if (ambiguous)
                    {
                        AmbiguousFaceReferences.Add(lineage.Reference);
                    }
                }
            }
        }

        private void EnsureUniqueCurrentPatchNames(List<int> groups)
        {
            // An obsolete ancestor may still be the stored label of one split
            // component. Give that component its own address before publishing
            // patches/edges; provenance aliases remain available only when unique.
            var reserved = new HashSet<string>(groupIdToExtendedName.Values, StringComparer.Ordinal);
            reserved.UnionWith(AmbiguousFaceReferences);
            var orderedIds = new List<int>(new HashSet<int>(groups));
            orderedIds.Sort();
            foreach (int id in orderedIds)
            {
                if (!FaceLineages.TryGetValue(id, out var lineage) || !lineage.Split ||
                    !groupIdToExtendedName.TryGetValue(id, out string previous) ||
                    !AmbiguousFaceReferences.Contains(previous))
                    continue;
                string basis = lineage.Roots.Count == 1 && lineage.Separators.Count == 0
                    ? lineage.Roots[0] : lineage.Reference;
                string current = basis;
                int suffix = 1;
                while (reserved.Contains(current))
                    current = EntityNaming.FormatPatchComponentName(basis, suffix++);
                reserved.Add(current);
                groupIdToExtendedName[id] = current;
                if (surfaceMetaData.TryGetValue(previous, out var metadata))
                {
                    surfaceMetaData[current] = metadata?.Clone();
                    surfaceMetaData.Remove(previous);
                }
            }
            extendedNameToGroupId = EntityNaming.BuildNameToGroupId(groupIdToExtendedName);
        }

        internal string ResolveLocalPatchNamePublic(string reference) => ResolveLocalPatchName(reference);

        /// <summary>Current, selectable name for a surface patch group.</summary>
        public string GetCurrentPatchName(int groupId)
        {
            EnsureCoplanarPostProcessed();
            if (groupIdToExtendedName == null || !groupIdToExtendedName.TryGetValue(groupId, out var name))
                throw new ArgumentOutOfRangeException(nameof(groupId));
            return name;
        }

        private string ResolveLocalPatchName(string reference)
        {
            if (string.IsNullOrWhiteSpace(reference))
                return null;

            reference = reference.Trim();
            // Exact current names take precedence over the convenient
            // ``mesh:local-name`` form. This matters when the mesh itself is
            // named after an operand and a descendant patch carries that same
            // prefix as part of its authoritative lineage.
            if (extendedNameToGroupId.ContainsKey(reference))
                return reference;
            if (TryResolveCurrentFaceReference(reference, out string current))
                return current;
            ValidateEntityReference(reference);

            string qualified = $"{Name}-{reference}";
            if (extendedNameToGroupId.ContainsKey(qualified))
                return qualified;

            if (reference.StartsWith(Name + ":", StringComparison.Ordinal))
            {
                string suffix = reference.Substring(Name.Length + 1);
                string resolved = ResolveLocalPatchName(suffix);
                if (resolved != null)
                    return resolved;
            }

            string unique = TryUniquePatchSuffixMatch(reference);
            if (unique != null)
                return unique;

            int dash = reference.IndexOf('-');
            if (dash >= 0 && dash < reference.Length - 1)
            {
                string curvePart = reference.Substring(dash + 1);
                unique = TryUniquePatchSuffixMatch(curvePart);
                if (unique != null)
                    return unique;
            }

            return null;
        }

        internal bool TryResolveCurrentFaceReference(string reference, out string patchName)
        {
            patchName = null;
            var matches = CurrentFacesMatching(reference);
            if (matches.Count != 1)
                return false;
            patchName = matches[0];
            return true;
        }

        private List<string> CurrentFacesMatching(string reference)
        {
            var names = new List<string>();
            if (FaceLineages == null || string.IsNullOrWhiteSpace(reference))
                return names;
            foreach (var pair in FaceLineages)
            {
                if (!string.Equals(pair.Value.Reference, reference, StringComparison.Ordinal) &&
                    !pair.Value.Roots.Contains(reference))
                    continue;
                if (groupIdToExtendedName.TryGetValue(pair.Key, out string name))
                    names.Add(name);
            }
            names.Sort(StringComparer.Ordinal);
            return names;
        }

        private string TryUniquePatchSuffixMatch(string curveName)
        {
            string suffix = "-" + curveName;
            string found = null;
            foreach (string key in extendedNameToGroupId.Keys)
            {
                if (!key.EndsWith(suffix, StringComparison.Ordinal))
                    continue;
                if (found != null)
                    return null;
                found = key;
            }
            return found;
        }
    }
}

