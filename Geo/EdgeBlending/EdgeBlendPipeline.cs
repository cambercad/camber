#pragma warning disable CS8600 // Converting null literal or possible null value to non-nullable type
#pragma warning disable CS8625 // Cannot convert null literal to non-nullable reference type

using GeoCore;

namespace Geo
{
    /// <summary>
    /// Shared orchestration for fillet and chamfer edge operations.
    /// </summary>
    public class EdgeBlendPipeline
    {
        public EdgeGraph edgeGraph;
        public List<GraphEdge> graphEdgesToBlend;
        public List<BlendEdge> blendEdges;
        public Dictionary<int, UVSurface> originalSurfaces;
        private AnchorMesh blendTopology;
        private readonly HashSet<long> tangentSegments = new();

        public AnchorMesh Run(
            AnchorMesh mesh,
            List<string> edgeNamesToBlend,
            IEdgeBlendProfile profile,
            CoordinateConverter cc,
            double maxDiscretizationDeviation,
            ref int groupIdOffset,
            string resultNameSuffix = "_blended")
            => Run(mesh, edgeNamesToBlend, profile, cc, maxDiscretizationDeviation,
                ref groupIdOffset, resultNameSuffix, null);

        internal AnchorMesh Run(AnchorMesh mesh, List<string> edgeNamesToBlend,
            IEdgeBlendProfile profile, CoordinateConverter cc, double maxDiscretizationDeviation,
            ref int groupIdOffset, string resultNameSuffix, Func<int, int> allocateGroupIds)
        {
            edgeGraph = new EdgeGraph(
                mesh.Mesh.Triangles,
                mesh.Mesh.GetTriangleGroups(),
                mesh.Mesh.Positions,
                mesh.Mesh.PrecisionPositions,
                mesh.groupIdToExtendedName);

            // Resolve user references once; categorization and every later
            // stage must use the same graph edge, including reversed face pairs.
            edgeNamesToBlend = edgeNamesToBlend.Select(query =>
            {
                if (!edgeGraph.TryGetEdge(query, out var edge))
                    throw new ArgumentException($"Edge '{query}' not found in edge graph", nameof(edgeNamesToBlend));
                return edge.Name;
            }).Distinct(StringComparer.Ordinal).ToList();

            edgeNamesToBlend = BuildSmoothTopology(mesh, edgeNamesToBlend, profile.OffsetDistance);
            CategorizeEdges(
                edgeGraph,
                mesh.Mesh.Triangles,
                blendTopology.Mesh.GetTriangleGroups(),
                mesh.Mesh.Positions,
                edgeNamesToBlend);

            List<List<string>> blendGroups = PartitionEdgesIntoBlendGroups(edgeGraph, edgeNamesToBlend, tangentSegments);
            ValidateBlendGroupConsistency(edgeGraph, blendGroups);

            AnchorMesh resultMesh = mesh;
            foreach (var group in blendGroups)
            {
                resultMesh = BlendEdgesGroup(
                    resultMesh,
                    group,
                    profile,
                    cc,
                    maxDiscretizationDeviation,
                    ref groupIdOffset,
                    resultNameSuffix, allocateGroupIds);
            }

            return resultMesh;
        }

        // Smooth patch seams are naming boundaries, not fillet terminations.
        // This view changes only the topology used to construct the blend; the
        // Boolean still receives the original solid and retains its face groups.
        private List<string> BuildSmoothTopology(AnchorMesh mesh, List<string> selected, double offsetDistance)
        {
            tangentSegments.Clear();
            var sourceEdges = selected.Select(n => { edgeGraph.TryGetEdge(n, out var e); return e; }).ToList();
            var groups = mesh.groupIdToExtendedName.Keys.OrderBy(i => i).ToList();
            var indices = groups.Select((id, i) => (id, i)).ToDictionary(p => p.id, p => p.i);
            var sets = new UnionFind(groups.Count);
            var surfaces = new Dictionary<int, UVSurface>();
            UVSurface Surface(int id)
            {
                if (!surfaces.TryGetValue(id, out var surface))
                {
                    mesh.TryGetTopologySurface(mesh.groupIdToExtendedName[id], out surface);
                    surfaces.Add(id, surface);
                }
                return surface;
            }
            var smoothEdges = new HashSet<GraphEdge>();
            foreach (var edge in edgeGraph.Edges)
            {
                if (!mesh.surfaceMetaData.TryGetValue(mesh.groupIdToExtendedName[edge.GroupIdA], out var metaA) ||
                    !mesh.surfaceMetaData.TryGetValue(mesh.groupIdToExtendedName[edge.GroupIdB], out var metaB)) continue;
                var a = Surface(edge.GroupIdA);
                var b = Surface(edge.GroupIdB);
                if (!AreAnalyticallyTangent(metaA, metaB) &&
                    !AreLinearExtrusionSeamsTangent(metaA, a, metaB, b, edge)) continue;
                // Analytic geometry establishes continuity. Display normals only
                // check matching orientation, not near-bitwise vector equality.
                bool smooth = edge.EdgeSegments.SelectMany(e => new[] { e.X, e.Y }).Distinct().All(i =>
                    a.Normals[i].LengthSquared() > 0 && b.Normals[i].LengthSquared() > 0 &&
                    Vec3DOps.Dot(a.Normals[i], b.Normals[i]) > 0);
                if (!smooth) continue;
                foreach (var segment in edge.EdgeSegments)
                    tangentSegments.Add(Algorithms.Key(segment.X, segment.Y));
                // A coalesced offset surface must remain regular. Preserve the
                // original face decomposition at a cylindrical offset singularity.
                if ((metaA.CylinderParams != null && metaA.CylinderParams.Radius <= offsetDistance) ||
                    (metaB.CylinderParams != null && metaB.CylinderParams.Radius <= offsetDistance)) continue;

                sets.Union(indices[edge.GroupIdA], indices[edge.GroupIdB]);
                smoothEdges.Add(edge);
            }
            blendTopology = mesh;
            if (smoothEdges.Count == 0) return selected;
            var mapping = new Dictionary<int, int>();
            foreach (var component in sets.GetComponents())
            {
                var members = component.Select(i => groups[i]).ToHashSet();
                // A path of tangent seams can run around a face and meet at a
                // real crease (for example a teardrop's sharp tip). Never erase
                // that crease through transitive face grouping.
                bool hasSharpSeam = edgeGraph.Edges.Any(edge => !smoothEdges.Contains(edge) &&
                    members.Contains(edge.GroupIdA) && members.Contains(edge.GroupIdB));
                int representative = members.Min();
                foreach (int id in members) mapping[id] = hasSharpSeam ? id : representative;
            }
            if (mapping.All(pair => pair.Key == pair.Value)) return selected;
            var corners = new List<MeshTriangle<TriangleVertexNormalUV>>(mesh.Mesh.TrianglesEx);
            for (int i = 0; i < corners.Count; i++)
            {
                var corner = corners[i];
                corner.GroupId = mapping[corner.GroupId];
                corners[i] = corner;
            }
            var view = new MeshNormalUV {
                Positions = mesh.Mesh.Positions, PrecisionPositions = mesh.Mesh.PrecisionPositions,
                Triangles = mesh.Mesh.Triangles, TrianglesEx = corners,
            };
            var topologyMetadata = SurfaceMetaData.CloneDictionary(mesh.surfaceMetaData);
            foreach (var group in mapping.GroupBy(p => p.Value).Where(g => g.Count() > 1))
                topologyMetadata[mesh.groupIdToExtendedName[group.Key]] = new SurfaceMetaData(SurfaceType.Unknown);
            blendTopology = new AnchorMesh(mesh.Name, view, mesh.groupIdToExtendedName,
                topologyMetadata, deferCoplanarPostProcess: true, skipCoplanarFusion: true,
                isVolume: mesh.IsVolume,
                faceLineages: mapping.GroupBy(pair => pair.Value).ToDictionary(group => group.Key,
                    group => FaceLineage.Merge(group.Select(pair => mesh.FaceLineages[pair.Key]))),
                ambiguousReferences: mesh.AmbiguousFaceReferences);
            edgeGraph = new EdgeGraph(view.Triangles, view.GetTriangleGroups(), view.Positions,
                view.PrecisionPositions, mesh.groupIdToExtendedName);
            var bySegment = new Dictionary<long, GraphEdge>();
            foreach (var edge in edgeGraph.Edges)
                foreach (var segment in edge.EdgeSegments)
                    bySegment[Algorithms.Key(segment.X, segment.Y)] = edge;
            return sourceEdges.Select(edge => {
                var segment = edge.EdgeSegments[0];
                if (!bySegment.TryGetValue(Algorithms.Key(segment.X, segment.Y), out var merged))
                    throw new ArgumentException($"Edge '{edge.Name}' is a smooth face seam and has no corner to blend.");
                return merged.Name;
            }).Distinct(StringComparer.Ordinal).ToList();
        }

        private static bool AreLinearExtrusionSeamsTangent(
            SurfaceMetaData a, UVSurface surfaceA, SurfaceMetaData b, UVSurface surfaceB, GraphEdge edge)
        {
            // An extrusion's profile-end generator has a constant analytic
            // tangent plane along its whole length. This certificate is not
            // applicable to arbitrary NURBS patches or smooth display normals.
            static bool IsLinearExtrusion(INurbsSurface surface) => surface switch
            {
                NURBS.BSplineLinearExtrudeSurface => true,
                NurbsConstruction.MeshUvMappedSurface mapped => IsLinearExtrusion(mapped.Inner),
                TransformedNurbsSurface transformed => IsLinearExtrusion(transformed.Inner),
                _ => false,
            };
            if (!a.HasNurbs || !b.HasNurbs ||
                !IsLinearExtrusion(a.NurbsSurface) || !IsLinearExtrusion(b.NurbsSurface)) return false;
            var vertices = edge.EdgeSegments.SelectMany(s => new[] { s.X, s.Y }).Distinct().ToArray();
            double uA = surfaceA.Uv[vertices[0]].X, uB = surfaceB.Uv[vertices[0]].X;
            if ((uA != 0 && uA != 1) || (uB != 0 && uB != 1) ||
                vertices.Any(i => surfaceA.Uv[i].X != uA || surfaceB.Uv[i].X != uB)) return false;
            var normalA = a.NurbsSurface.EvaluateNormal(uA, .5);
            var normalB = b.NurbsSurface.EvaluateNormal(uB, .5);
            if (normalA.LengthSquared() == 0 || normalB.LengthSquared() == 0) return false;
            // Same analytic parameter tolerance as the primitive cases below;
            // mesh incidence and Boolean classification remain exact.
            return Vec3DOps.Cross(normalA.Normalized(), normalB.Normalized()).LengthSquared() <= 1e-24;
        }

        private static bool AreAnalyticallyTangent(SurfaceMetaData a, SurfaceMetaData b)
        {
            // These analytic cases check continuity along the complete shared
            // seam. Display normals alone cannot distinguish a tangent seam from
            // a sharp corner on a smooth-shaded imported mesh. The tolerance here
            // is for floating-point analytic parameters, not mesh incidence.
            const double parameterTolerance = 1e-12;
            bool Parallel(Vec3D x, Vec3D y) => x.LengthSquared() > 0 && y.LengthSquared() > 0 &&
                Vec3DOps.Cross(x.Normalized(), y.Normalized()).LengthSquared() <= parameterTolerance * parameterTolerance;
            if (a.PlaneParams != null && b.PlaneParams != null)
                return Parallel(a.PlaneParams.Normal, b.PlaneParams.Normal);
            var plane = a.PlaneParams ?? b.PlaneParams;
            var cylinder = a.CylinderParams ?? b.CylinderParams;
            if (plane != null && cylinder != null)
            {
                var normal = plane.Normal.Normalized();
                return Math.Abs(Vec3DOps.Dot(normal, cylinder.Axis.Normalized())) <= parameterTolerance &&
                    Math.Abs(Math.Abs(Vec3DOps.Dot(cylinder.Origin - plane.Origin, normal)) - cylinder.Radius)
                        <= parameterTolerance * Math.Max(1, cylinder.Radius);
            }
            if (a.CylinderParams != null && b.CylinderParams != null)
            {
                var first = a.CylinderParams;
                var second = b.CylinderParams;
                double tolerance = parameterTolerance * Math.Max(1, Math.Max(first.Radius, second.Radius));
                if (!Parallel(first.Axis, second.Axis)) return false;
                // Distinct parallel cylinders can share a tangent generator,
                // including opposite-curvature run-outs. Their real shared seam
                // and consistent normals are checked by the caller.
                double distance = Vec3DOps.Cross(second.Origin - first.Origin, first.Axis.Normalized()).Length();
                return (Math.Abs(first.Radius - second.Radius) <= tolerance && distance <= tolerance) ||
                    Math.Abs(distance - (first.Radius + second.Radius)) <= tolerance ||
                    Math.Abs(distance - Math.Abs(first.Radius - second.Radius)) <= tolerance;
            }
            return false;
        }

        private static void CategorizeEdges(
            EdgeGraph edgeGraph,
            List<Tri> triangles,
            List<int> groupIdPerTriangle,
            List<Vec3D> positions,
            IReadOnlyList<string> edgeNamesToBlend)
        {
            var edgesToCategorize = new HashSet<string>(edgeNamesToBlend, StringComparer.Ordinal);
            Dictionary<Int2, List<int>> edgeToTriangles = new Dictionary<Int2, List<int>>();

            for (int triIdx = 0; triIdx < triangles.Count; triIdx++)
            {
                Tri tri = triangles[triIdx];
                AddEdgeTriangle(edgeToTriangles, tri.A, tri.B, triIdx);
                AddEdgeTriangle(edgeToTriangles, tri.B, tri.C, triIdx);
                AddEdgeTriangle(edgeToTriangles, tri.C, tri.A, triIdx);
            }

            foreach (GraphEdge graphEdge in edgeGraph.Edges)
            {
                if (!edgesToCategorize.Contains(graphEdge.Name))
                    continue;

                int groupA = graphEdge.GroupIdA;
                int groupB = graphEdge.GroupIdB;
                List<double> signedDistances = new List<double>();

                foreach (var edgeSeg in graphEdge.EdgeSegments)
                {
                    Int2 key = new Int2(Math.Min(edgeSeg.X, edgeSeg.Y), Math.Max(edgeSeg.X, edgeSeg.Y));
                    if (!edgeToTriangles.TryGetValue(key, out var tris) || tris.Count < 2)
                        continue;

                    int triA = -1, triB = -1;
                    foreach (int triIdx in tris)
                    {
                        int group = groupIdPerTriangle[triIdx];
                        if (group == groupA && triA == -1) triA = triIdx;
                        if (group == groupB && triB == -1) triB = triIdx;
                    }

                    if (triA == -1 || triB == -1)
                        continue;

                    Tri tA = triangles[triA];
                    Tri tB = triangles[triB];
                    int v0 = edgeSeg.X;
                    int v1 = edgeSeg.Y;
                    int v3 = tB.GetRemaining(v0, v1);

                    Vec3D p0 = positions[tA.A];
                    Vec3D p1 = positions[tA.B];
                    Vec3D p2 = positions[tA.C];
                    Vec3D p3 = positions[v3];

                    Plane tri1Plane = new Plane(p0, p1, p2);
                    signedDistances.Add(tri1Plane.SignedDistance(p3));
                }

                if (signedDistances.Count == 0)
                    throw new Exception($"Edge {graphEdge.Name}: No valid triangle pairs found");

                graphEdge.BlendType = ClassifyEdgeBlendType(graphEdge.Name, signedDistances);
            }
        }

        private static EdgeBlendType ClassifyEdgeBlendType(string edgeName, List<double> signedDistances)
        {
            const double flatTolerance = 1e-8;
            int concaveVotes = 0;
            int convexVotes = 0;

            foreach (double dist in signedDistances)
            {
                if (Math.Abs(dist) <= flatTolerance)
                    continue;
                if (dist < 0)
                    concaveVotes++;
                else
                    convexVotes++;
            }

            if (concaveVotes == 0 && convexVotes == 0)
                throw new Exception($"Edge {edgeName}: Cannot determine convexity (all samples coplanar)");

            bool isConcave = concaveVotes >= convexVotes;
            return isConcave ? EdgeBlendType.Convex : EdgeBlendType.Concave;
        }

        private static void AddEdgeTriangle(Dictionary<Int2, List<int>> edgeToTriangles, int v1, int v2, int triIdx)
        {
            Int2 key = new Int2(Math.Min(v1, v2), Math.Max(v1, v2));
            if (!edgeToTriangles.ContainsKey(key))
                edgeToTriangles[key] = new List<int>();
            edgeToTriangles[key].Add(triIdx);
        }

        private static void ValidateBlendGroupConsistency(EdgeGraph edgeGraph, List<List<string>> blendGroups)
        {
            Dictionary<string, GraphEdge> edgesByName = new Dictionary<string, GraphEdge>();
            foreach (GraphEdge edge in edgeGraph.Edges)
                edgesByName[edge.Name] = edge;

            for (int groupIdx = 0; groupIdx < blendGroups.Count; groupIdx++)
            {
                List<string> group = blendGroups[groupIdx];
                if (group.Count == 0)
                    continue;

                if (!edgesByName.TryGetValue(group[0], out GraphEdge firstEdge))
                    throw new Exception($"Edge {group[0]} not found in edge graph");

                EdgeBlendType groupType = firstEdge.BlendType;
                for (int i = 1; i < group.Count; i++)
                {
                    if (!edgesByName.TryGetValue(group[i], out GraphEdge edge))
                        throw new Exception($"Edge {group[i]} not found in edge graph");

                    if (edge.BlendType != groupType)
                    {
                        throw new Exception(
                            $"Blend group {groupIdx} contains edges with inconsistent blend types: " +
                            $"Edge '{group[0]}' is {groupType}, but edge '{group[i]}' is {edge.BlendType}. " +
                            $"All edges in a blend group must be either all convex or all concave.");
                    }
                }
            }
        }

        private static List<List<string>> PartitionEdgesIntoBlendGroups(
            EdgeGraph edgeGraph,
            List<string> edgeNamesToBlend, HashSet<long> tangentSegments)
        {
            Dictionary<string, int> edgeNameToBlendIndex = new Dictionary<string, int>();
            for (int i = 0; i < edgeNamesToBlend.Count; i++)
                edgeNameToBlendIndex[edgeNamesToBlend[i]] = i;

            Dictionary<EdgeGraphNode, int> perNodeEdgeCounts = new Dictionary<EdgeGraphNode, int>();
            foreach (var node in edgeGraph.Nodes)
                perNodeEdgeCounts.Add(node, 0);

            foreach (var blendEdgeName in edgeNamesToBlend)
            {
                if (!edgeGraph.TryGetEdge(blendEdgeName, out var edge))
                    throw new Exception($"Edge '{blendEdgeName}' not found in edge graph");

                perNodeEdgeCounts[edge.StartNode]++;
                perNodeEdgeCounts[edge.EndNode]++;
            }

            HashSet<EdgeGraphNode> connectorNodes = new HashSet<EdgeGraphNode>();
            foreach (var v in perNodeEdgeCounts)
            {
                if (v.Value > 0 && v.Key.ConnectedEdges.All(edge =>
                    edgeNameToBlendIndex.ContainsKey(edge.Name) || edge.EdgeSegments.All(segment =>
                        tangentSegments.Contains(Algorithms.Key(segment.X, segment.Y)))))
                    connectorNodes.Add(v.Key);
            }

            UnionFind uf = new UnionFind(edgeNamesToBlend.Count);
            foreach (var node in connectorNodes)
            {
                List<int> blendIndicesAtNode = new List<int>();
                foreach (var edge in node.ConnectedEdges)
                {
                    if (edgeNameToBlendIndex.TryGetValue(edge.Name, out int blendIndex))
                        blendIndicesAtNode.Add(blendIndex);
                }

                if (blendIndicesAtNode.Count > 1)
                {
                    for (int i = 1; i < blendIndicesAtNode.Count; i++)
                        uf.Union(blendIndicesAtNode[0], blendIndicesAtNode[i]);
                }
            }

            List<List<string>> blendGroups = new List<List<string>>();
            foreach (var component in uf.GetComponents())
            {
                List<string> group = new List<string>();
                foreach (int index in component)
                    group.Add(edgeNamesToBlend[index]);
                blendGroups.Add(group);
            }

            return blendGroups;
        }

        private AnchorMesh BlendEdgesGroup(
            AnchorMesh mesh,
            List<string> edgeNamesToBlend,
            IEdgeBlendProfile profile,
            CoordinateConverter cc,
            double maxDiscretizationDeviation,
            ref int groupIdOffset,
            string resultNameSuffix, Func<int, int> allocateGroupIds)
        {
            if (edgeGraph == null)
            {
                edgeGraph = new EdgeGraph(
                    mesh.Mesh.Triangles,
                    mesh.Mesh.GetTriangleGroups(),
                    mesh.Mesh.Positions,
                    mesh.Mesh.PrecisionPositions,
                    mesh.groupIdToExtendedName);
            }

            graphEdgesToBlend = new List<GraphEdge>();
            foreach (string edgeName in edgeNamesToBlend)
            {
                if (!edgeGraph.TryGetEdge(edgeName, out GraphEdge edge) || edge == null)
                    throw new ArgumentException($"Edge '{edgeName}' not found in mesh");
                graphEdgesToBlend.Add(edge);
            }

            // A closed cylinder/end disk has a point spine when both radii
            // agree; its consumed disk is replaced by a spherical cap.
            if (profile is FilletProfile && graphEdgesToBlend.Count == 1 &&
                CollapsedCylinderFillet.TryCreate(mesh, graphEdgesToBlend[0], profile.OffsetDistance,
                    cc, maxDiscretizationDeviation, ref groupIdOffset, out var sphereResult, out var sphereMetadata, allocateGroupIds))
            {
                int patchGroup = groupIdOffset - 1;
                string patchName = profile.EdgePatchName(graphEdgesToBlend[0].Name);
                var names = new Dictionary<int, string>(mesh.groupIdToExtendedName) { [patchGroup] = patchName };
                var metadata = SurfaceMetaData.CloneDictionary(mesh.surfaceMetaData);
                metadata[patchName] = sphereMetadata;
                return new AnchorMesh(mesh.Name + resultNameSuffix, sphereResult, names, metadata,
                    deferCoplanarPostProcess: false, skipCoplanarFusion: true, isVolume: mesh.IsVolume,
                    faceLineages: mesh.FaceLineages, ambiguousReferences: mesh.AmbiguousFaceReferences);
            }

            blendEdges = CreateBlendEdges(graphEdgesToBlend, profile.OffsetDistance);
            originalSurfaces = GetOriginalSurfaces(blendTopology, blendEdges);

            Dictionary<int, UVSurface> openCornerTrimSurfaces = new Dictionary<int, UVSurface>();
            Dictionary<int, SurfaceMetaData> openCornerTrimSurfacesMetaData = new Dictionary<int, SurfaceMetaData>();
            for (int i = 0; i < blendEdges.Count; i++)
            {
                var blendEdge = blendEdges[i];
                var e = graphEdgesToBlend[i];
                if (IsOpenEnd(e.StartNode))
                {
                    var trim = FindTrimSurface(e, e.StartNode, e.BlendType == EdgeBlendType.Convex ? blendTopology : mesh, out var surfaceName);
                    trim = EndpointTrim(e, e.StartNode, trim);
                    if (trim != null) openCornerTrimSurfaces[blendEdge.StartCornerId] = trim;
                    if (trim != null && mesh.surfaceMetaData.TryGetValue(surfaceName, out var metaData))
                        openCornerTrimSurfacesMetaData[blendEdge.StartCornerId] = metaData;
                }
                if (IsOpenEnd(e.EndNode))
                {
                    var trim = FindTrimSurface(e, e.EndNode, e.BlendType == EdgeBlendType.Convex ? blendTopology : mesh, out var surfaceName);
                    trim = EndpointTrim(e, e.EndNode, trim);
                    if (trim != null) openCornerTrimSurfaces[blendEdge.EndCornerId] = trim;
                    if (trim != null && mesh.surfaceMetaData.TryGetValue(surfaceName, out var metaData))
                        openCornerTrimSurfacesMetaData[blendEdge.EndCornerId] = metaData;
                }
            }

            var collapsedContacts = new Dictionary<BlendEdge, List<Rat3Hybrid>>();
            var collapsedSurfaces = new HashSet<int>();
            if (profile is FilletProfile)
                foreach (var edge in blendEdges)
                {
                    bool startSelected = blendEdges.Any(other => other != edge &&
                        (other.StartCornerId == edge.StartCornerId || other.EndCornerId == edge.StartCornerId));
                    bool endSelected = blendEdges.Any(other => other != edge &&
                        (other.StartCornerId == edge.EndCornerId || other.EndCornerId == edge.EndCornerId));
                    if (startSelected && endSelected && CollapsedCylinderFillet.TryGetSectorContact(mesh,
                        edge.SourceEdge, profile.OffsetDistance, cc, out int cylinderId, out var contact))
                    {
                        collapsedContacts.Add(edge, contact);
                        collapsedSurfaces.Add(cylinderId);
                    }
                }

            MeshNormalUV result = ApplyEdgeBlend(
                blendEdges,
                originalSurfaces,
                profile,
                cc,
                maxDiscretizationDeviation,
                mesh.Mesh,
                profile.OffsetDistance,
                ref groupIdOffset,
                openCornerTrimSurfaces,
                openCornerTrimSurfacesMetaData, collapsedContacts, collapsedSurfaces,
                FindClosedSupportCaps(blendEdges), out var patches, allocateGroupIds);
            int startingGroupId = groupIdOffset - patches.Count;

            Dictionary<int, string> completeGroupMapping = new Dictionary<int, string>(mesh.groupIdToExtendedName);
            var meta = SurfaceMetaData.CloneDictionary(mesh.surfaceMetaData);
            var usedPatchNames = new HashSet<string>(completeGroupMapping.Values, StringComparer.Ordinal);
            int cornerIndex = 0;
            // Preserve emitted order: a strip can be followed by planar end
            // closures before the next strip. Those closures inherit their trim face.
            for (int i = 0; i < patches.Count; ++i)
            {
                var patch = patches[i];
                string name = patch.Name;
                if (name == null)
                {
                    do { name = profile.CornerPatchName(cornerIndex++); }
                    while (!usedPatchNames.Add(name));
                }
                completeGroupMapping[startingGroupId+i] = name;
                meta[name] = patch.Metadata?.Clone() ?? new SurfaceMetaData(SurfaceType.Unknown);
            }

            var probeMesh = new AnchorMesh(mesh.Name + "_probe", result, completeGroupMapping, meta, deferCoplanarPostProcess: true, isVolume: mesh.IsVolume,
                faceLineages: mesh.FaceLineages, ambiguousReferences: mesh.AmbiguousFaceReferences);
            profile.AttachPatchNurbs(meta, blendEdges, probeMesh);

            // End closures can extend an existing planar face. Run the shared exact
            // coplanar fusion so their triangulation does not become a selectable seam.
            return new AnchorMesh(mesh.Name + resultNameSuffix, result, completeGroupMapping, meta, deferCoplanarPostProcess: false, isVolume: mesh.IsVolume,
                    faceLineages: mesh.FaceLineages, ambiguousReferences: mesh.AmbiguousFaceReferences);
        }

        private static Vec3D ExactDirection(Rat3Hybrid vector)
        {
            var scale = vector.X.Sign() < 0 ? -vector.X : vector.X;
            for (int axis = 1; axis < 3; ++axis)
            {
                var magnitude = vector[axis].Sign() < 0 ? -vector[axis] : vector[axis];
                if (magnitude > scale) scale = magnitude;
            }
            if (scale.Sign() == 0)
                throw new InvalidOperationException("A planar closure requires a nonzero direction.");
            vector /= scale;
            return new Vec3D(vector.X.ToDouble(), vector.Y.ToDouble(), vector.Z.ToDouble()).Normalized();
        }

        private static SurfaceMetaData PlanarClosureMetadata(UVSurface patch)
        {
            var triangle = patch.Triangles.First(t => Rat3Hybrid.Cross(
                patch.PointsPrecise[t.B] - patch.PointsPrecise[t.A],
                patch.PointsPrecise[t.C] - patch.PointsPrecise[t.A]) != new Rat3Hybrid(0, 0, 0));
            var direction = patch.PointsPrecise[triangle.B] - patch.PointsPrecise[triangle.A];
            var normal = Rat3Hybrid.Cross(direction, patch.PointsPrecise[triangle.C] - patch.PointsPrecise[triangle.A]);
            return new SurfaceMetaData(SurfaceType.Planar)
            {
                PlaneParams = new PlaneSurfaceParams
                {
                    Origin = patch.Points[triangle.A],
                    Normal = ExactDirection(normal),
                    RefDir = ExactDirection(direction)
                }
            };
        }

        // Convex strips entering a reentrant support need a local end closure.
        // Use the original support: an earlier selected strip may already have
        // removed the graph vertex from the evolving result.
        private static UVSurface EndpointTrim(GraphEdge edge, EdgeGraphNode node, UVSurface support)
        {
            if (support == null || edge.BlendType == EdgeBlendType.Concave) return support;
            var incident = support.Triangles.Where(t => support.PointsPrecise[t.A] == node.PositionExact ||
                support.PointsPrecise[t.B] == node.PositionExact || support.PointsPrecise[t.C] == node.PositionExact).ToList();
            var nondegenerate = incident.Where(t => Rat3Hybrid.Cross(
                support.PointsPrecise[t.B] - support.PointsPrecise[t.A],
                support.PointsPrecise[t.C] - support.PointsPrecise[t.A]) != new Rat3Hybrid(0, 0, 0)).ToList();
            if (nondegenerate.Count == 0) return null;
            var seed = nondegenerate[0];
            var normal = Rat3Hybrid.Cross(support.PointsPrecise[seed.B] - support.PointsPrecise[seed.A],
                support.PointsPrecise[seed.C] - support.PointsPrecise[seed.A]);
            var planeD = Rat3Hybrid.Dot(normal, support.PointsPrecise[seed.A]);
            bool OnPlane(Tri triangle) => new[] { triangle.A, triangle.B, triangle.C }.All(i =>
                Rat3Hybrid.Dot(normal, support.PointsPrecise[i]) == planeD);
            // Every incident facet must agree; a curved fan is not a local plane.
            if (incident.Any(t => !OnPlane(t))) return null;
            var adjacent = edge.LineStripExact[0] == node.PositionExact ? edge.LineStripExact[1] : edge.LineStripExact[^2];
            if (Rat3Hybrid.Dot(normal, adjacent) <= planeD) return null;

            // A locally planar tessellation row is not a planar support face.
            // Keep the connected curved face so its endpoint closure follows the
            // actual shell across subsequent rows instead of extrapolating one facet.
            var candidates = support.Triangles;
            var adjacency = Adjacency.BuildAdjacencyInformation(candidates);
            int seedIndex = candidates.FindIndex(t => t.A == seed.A && t.B == seed.B && t.C == seed.C);
            var pending = new Queue<int>();
            var visited = new HashSet<int> { seedIndex };
            pending.Enqueue(seedIndex);
            while (pending.Count > 0)
            {
                var neighbours = adjacency[pending.Dequeue()];
                foreach (int neighbour in new[] { neighbours.NeighbourAB, neighbours.NeighbourBC, neighbours.NeighbourCA })
                    if (neighbour >= 0 && visited.Add(neighbour)) pending.Enqueue(neighbour);
            }
            // Remote islands do not extend this endpoint's support.
            var local = Enumerable.Range(0, candidates.Count).Where(visited.Contains).Select(i => candidates[i]).ToList();
            return new UVSurface(support.Points, support.Normals.Select(n => -n).ToList(), support.Uv,
                local.Select(t => new Tri(t.A, t.C, t.B)).ToList(), support.PointsPrecise);
        }

        private static List<BlendEdge> CreateBlendEdges(List<GraphEdge> graphEdges, double offsetDistance)
        {
            List<BlendEdge> result = new List<BlendEdge>();
            foreach (var graphEdge in graphEdges)
            {
                var blendEdge = new BlendEdge(graphEdge, graphEdge.GroupIdA, graphEdge.GroupIdB, offsetDistance);
                blendEdge.BlendType = graphEdge.BlendType;
                blendEdge.StartCornerId = graphEdge.StartNodeId;
                blendEdge.EndCornerId = graphEdge.EndNodeId;
                result.Add(blendEdge);
            }
            return result;
        }

        private static Dictionary<int, UVSurface> GetOriginalSurfaces(AnchorMesh mesh, List<BlendEdge> blendEdges)
        {
            Dictionary<int, UVSurface> surfaces = new Dictionary<int, UVSurface>();
            HashSet<int> groupIds = new HashSet<int>();
            foreach (var edge in blendEdges)
            {
                groupIds.Add(edge.SurfaceIndexA);
                groupIds.Add(edge.SurfaceIndexB);
            }

            foreach (int groupId in groupIds)
            {
                string surfaceName = mesh.groupIdToExtendedName[groupId];
                if (mesh.TryGetTopologySurface(surfaceName, out UVSurface surface) && surface != null)
                    surfaces[groupId] = surface;
            }

            return surfaces;
        }

        private Dictionary<BlendEdge, List<(UVSurface Surface, SurfaceMetaData Metadata)>> FindClosedSupportCaps(
            List<BlendEdge> edges)
        {
            var result = new Dictionary<BlendEdge, List<(UVSurface, SurfaceMetaData)>>();
            foreach (var edge in edges)
            {
                if (edge.BlendType != EdgeBlendType.Concave || edge.SourceEdge.LineStripExact[0] != edge.SourceEdge.LineStripExact[^1])
                    continue;
                var supportIds = new HashSet<int> { edge.SurfaceIndexA, edge.SurfaceIndexB };
                var neighbors = new HashSet<int>();
                foreach (var seam in edgeGraph.Edges)
                {
                    if (supportIds.Contains(seam.GroupIdA) && !supportIds.Contains(seam.GroupIdB))
                        neighbors.Add(seam.GroupIdB);
                    if (supportIds.Contains(seam.GroupIdB) && !supportIds.Contains(seam.GroupIdA))
                        neighbors.Add(seam.GroupIdA);
                }
                var caps = new List<(UVSurface, SurfaceMetaData)>();
                foreach (int id in neighbors)
                {
                    string name = blendTopology.groupIdToExtendedName[id];
                    if (!blendTopology.TryGetTopologySurface(name, out var surface) || !surface.IsSurfacePlanar())
                        continue;
                    var triangle = surface.Triangles.First(t => Rat3Hybrid.Cross(
                        surface.PointsPrecise[t.B] - surface.PointsPrecise[t.A],
                        surface.PointsPrecise[t.C] - surface.PointsPrecise[t.A]) != new Rat3Hybrid(0, 0, 0));
                    var origin = surface.PointsPrecise[triangle.A];
                    var normal = Rat3Hybrid.Cross(surface.PointsPrecise[triangle.B] - origin, surface.PointsPrecise[triangle.C] - origin);
                    // A terminating cap bounds this entire original closed edge.
                    // A nearby plane crossing it is not a legal truncation face.
                    if (edge.SourceEdge.LineStripExact.Any(point => Rat3Hybrid.Dot(point - origin, normal).Sign() > 0))
                        continue;
                    blendTopology.surfaceMetaData.TryGetValue(name, out var metadata);
                    caps.Add((surface, metadata));
                }
                if (caps.Count > 0)
                    result.Add(edge, caps);
            }
            return result;
        }

        private static UVSurface ExtendTrimSurface(UVSurface source, double distance,
            CoordinateConverter converter, out UVSurface extensionOnly)
        {
            if (!source.IsSurfacePlanar())
                return source.GetExtendedSurface(distance, converter, out extensionOnly);

            // A planar supporting face extends as a plane. A collar extruded
            // from a concave trimmed boundary can fold over itself.
            Rat3Hybrid normal = new Rat3Hybrid(0,0,0), origin = source.PointsPrecise[0];
            foreach (var triangle in source.Triangles)
            {
                origin = source.PointsPrecise[triangle.A];
                normal = Rat3Hybrid.Cross(source.PointsPrecise[triangle.B] - origin,
                    source.PointsPrecise[triangle.C] - origin);
                if (normal != new Rat3Hybrid(0,0,0)) break;
            }
            int dropped = 0;
            for (int axis=1;axis<3;++axis)
                if (Math.Abs(normal[axis].ToDouble())>Math.Abs(normal[dropped].ToDouble())) dropped=axis;
            var scale=normal[dropped].Sign()>0?normal[dropped]:-normal[dropped];
            normal/=scale;
            normal.Simplify();
            int x=(dropped+1)%3, y=(dropped+2)%3;
            var margin = new BigRationalHybrid(converter.ConvertDirection(new Vec3D(distance,0,0)).X);
            var usedPoints=source.Triangles.SelectMany(t=>new[]{t.A,t.B,t.C}).Distinct().Select(i=>source.PointsPrecise[i]).ToList();
            var minX=usedPoints.Select(point=>point[x]).Aggregate((a,b)=>a<b?a:b)-margin;
            var maxX=usedPoints.Select(point=>point[x]).Aggregate((a,b)=>a>b?a:b)+margin;
            var minY=usedPoints.Select(point=>point[y]).Aggregate((a,b)=>a<b?a:b)-margin;
            var maxY=usedPoints.Select(point=>point[y]).Aggregate((a,b)=>a>b?a:b)+margin;
            var planeD=Rat3Hybrid.Dot(normal,origin);
            Rat3Hybrid Point(BigRationalHybrid a, BigRationalHybrid b)
            {
                var coordinates=new BigRationalHybrid[3];
                coordinates[x]=a;coordinates[y]=b;
                coordinates[dropped]=(planeD-normal[x]*a-normal[y]*b)/normal[dropped];
                var point=new Rat3Hybrid(coordinates[0],coordinates[1],coordinates[2]);
                point.Simplify();return point;
            }
            var points=new List<Rat3Hybrid>{Point(minX,minY),Point(maxX,minY),Point(maxX,maxY),Point(minX,maxY)};
            // Retain the source-face sampling in the supporting plane so exact
            // intersection work is distributed over its existing local triangles.
            var sourceMap=new Dictionary<int,int>();
            foreach(int index in source.Triangles.SelectMany(t=>new[]{t.A,t.B,t.C}).Distinct())
            {
                var point=source.PointsPrecise[index];
                int mapped=points.FindIndex(existing=>existing==point);
                if(mapped<0){mapped=points.Count;points.Add(point);}
                sourceMap.Add(index,mapped);
            }
            var constraints=source.Triangles.SelectMany(t=>new[]{
                new Int2(sourceMap[t.A],sourceMap[t.B]),new Int2(sourceMap[t.B],sourceMap[t.C]),
                new Int2(sourceMap[t.C],sourceMap[t.A])}).ToList();
            var projected=points.Select(point=>new Rat2Hybrid(point[x],point[y])).ToList();
            var triangles=Triangulator.TriangulatePolygon(projected,new List<int>{0,1,2,3},
                constraints,Enumerable.Range(4,points.Count-4).ToList(),false);
            if(normal[dropped].Sign()<0)
                triangles=triangles.Select(t=>new Tri(t.A,t.C,t.B)).ToList();
            var direction=new Vec3D(normal.X.ToDouble(),normal.Y.ToDouble(),normal.Z.ToDouble()).Normalized();
            var uv=points.Select(point=>new Vec2D(((point[x]-minX)/(maxX-minX)).ToDouble(),
                ((point[y]-minY)/(maxY-minY)).ToDouble())).ToList();
            extensionOnly=new UVSurface(converter.Convert(points),Enumerable.Repeat(direction,points.Count).ToList(),
                uv,triangles,points);
            return extensionOnly;
        }

        private static MeshNormalUV ApplyEdgeBlend(
            List<BlendEdge> blendEdges,
            Dictionary<int, UVSurface> originalSurfaces,
            IEdgeBlendProfile profile,
            CoordinateConverter cc,
            double maxDiscretizationDeviation,
            MeshNormalUV fullMesh,
            double edgeOffsetDistance,
            ref int groupIdOffset,
            Dictionary<int, UVSurface> openCornerTrimSurfaces,
            Dictionary<int, SurfaceMetaData> openCornerTrimSurfacesMetaData,
            Dictionary<BlendEdge, List<Rat3Hybrid>> collapsedContacts,
            HashSet<int> collapsedSurfaces,
            Dictionary<BlendEdge,List<(UVSurface Surface,SurfaceMetaData Metadata)>> closedSupportCaps,
            out List<(string Name, SurfaceMetaData Metadata)> patches,
            Func<int, int> allocateGroupIds)
        {
            patches = new List<(string Name, SurfaceMetaData Metadata)>();
            EdgeBlendType blendType = blendEdges.Count > 0 ? blendEdges[0].BlendType : EdgeBlendType.Convex;

            Dictionary<int, UVSurface> allElargedOffsetSurfaces = new Dictionary<int, UVSurface>(originalSurfaces.Count);
            Dictionary<int, UVSurface> allElargedSurfaces = new Dictionary<int, UVSurface>(originalSurfaces.Count);

            double enlarge = 2 * edgeOffsetDistance;
            double offset = blendType == EdgeBlendType.Convex ? -edgeOffsetDistance : edgeOffsetDistance;

            foreach (var v in originalSurfaces)
            {
                var extendedSurface = ExtendTrimSurface(v.Value, enlarge, cc, out _);
                var extendedAndOffsetSurface = extendedSurface.GetOffsetSurface(offset, cc);
                allElargedSurfaces.Add(v.Key, extendedSurface);
                allElargedOffsetSurfaces.Add(v.Key, extendedAndOffsetSurface);
            }

            Dictionary<int, UVSurface> extendedOpenCornerTrimSurfaces = new Dictionary<int, UVSurface>();
            Dictionary<int, UVSurface> extensionOnlyOpenCornerTrimSurfaces = new Dictionary<int, UVSurface>();
            foreach (var v in openCornerTrimSurfaces)
            {
                var extendedSurface = ExtendTrimSurface(v.Value, enlarge, cc, out var extensionOnly);
                extendedOpenCornerTrimSurfaces.Add(v.Key, extendedSurface);
                extensionOnlyOpenCornerTrimSurfaces.Add(v.Key, extensionOnly);
            }

            List<UVSurface> allSurfaces = new List<UVSurface>();
            foreach (var blendEdge in blendEdges)
            {
                if (collapsedContacts.TryGetValue(blendEdge, out var contact))
                {
                    // A zero-length strip contributes its cylinder contact arc
                    // to the adjoining spherical corner, not an empty offset mesh.
                    blendEdge.TrimArcs.Add(contact);
                    continue;
                }
                if (!originalSurfaces.TryGetValue(blendEdge.SurfaceIndexA, out UVSurface originalSurfaceA) || originalSurfaceA == null)
                    continue;
                if (!originalSurfaces.TryGetValue(blendEdge.SurfaceIndexB, out UVSurface originalSurfaceB) || originalSurfaceB == null)
                    continue;
                if (!allElargedSurfaces.TryGetValue(blendEdge.SurfaceIndexA, out UVSurface extendedSurfaceA) || extendedSurfaceA == null)
                    continue;
                if (!allElargedSurfaces.TryGetValue(blendEdge.SurfaceIndexB, out UVSurface extendedSurfaceB) || extendedSurfaceB == null)
                    continue;
                if (!allElargedOffsetSurfaces.TryGetValue(blendEdge.SurfaceIndexA, out UVSurface extendedAndOffsetSurfaceA) || extendedAndOffsetSurfaceA == null)
                    continue;
                if (!allElargedOffsetSurfaces.TryGetValue(blendEdge.SurfaceIndexB, out UVSurface extendedAndOffsetSurfaceB) || extendedAndOffsetSurfaceB == null)
                    continue;

                if (!blendEdge.ComputeSpineAndBoundaries(
                        originalSurfaceA, originalSurfaceB,
                        extendedSurfaceA, extendedSurfaceB,
                        extendedAndOffsetSurfaceA, extendedAndOffsetSurfaceB, cc))
                    continue;

                profile.BuildStripSurface(blendEdge, cc, maxDiscretizationDeviation);


                foreach (var v in allElargedOffsetSurfaces)
                {
                    var surfId = v.Key;
                    if (collapsedSurfaces.Contains(surfId) || surfId == blendEdge.SurfaceIndexA || surfId == blendEdge.SurfaceIndexB)
                        continue;
                    blendEdge.TrimByOffsetSurface(blendEdge.PerpendicularCornerTrimSurface(v.Value));
                }

                if (blendEdge.BlendType == EdgeBlendType.Convex)
                {
                    blendEdge.TrimBySurface(extendedOpenCornerTrimSurfaces, extensionOnlyOpenCornerTrimSurfaces);
                    blendEdge.TrimByVolume(fullMesh, cc);
                }
                else
                    blendEdge.TrimBySurface(extendedOpenCornerTrimSurfaces, extensionOnlyOpenCornerTrimSurfaces);

                var capClosures = new List<(UVSurface Surface, SurfaceMetaData Metadata)>();
                if (closedSupportCaps.TryGetValue(blendEdge, out var caps))
                    foreach (var cap in caps)
                    {
                        var extended = ExtendTrimSurface(cap.Surface, enlarge, cc, out var extension);
                        if (blendEdge.TrimBySupportingCap(extended, extension, out var closures))
                            capClosures.AddRange(closures.Select(surface => (surface, cap.Metadata)));
                    }
                allSurfaces.Add(blendEdge.BlendSurface);
                patches.Add((profile.EdgePatchName(blendEdge.SourceEdge.Name), null));
                foreach (var closure in capClosures)
                {
                    allSurfaces.Add(closure.Surface);
                    patches.Add((null, closure.Metadata));
                }
                if (blendEdge.EdgeStartGapFillSurface != null)
                {
                    allSurfaces.Add(blendEdge.EdgeStartGapFillSurface);
                    openCornerTrimSurfacesMetaData.TryGetValue(blendEdge.StartCornerId, out var metadata);
                    patches.Add((null, blendEdge.BlendType == EdgeBlendType.Convex
                        ? blendEdge.EdgeStartGapFillSurface.IsSurfacePlanar() ? PlanarClosureMetadata(blendEdge.EdgeStartGapFillSurface) : null : metadata));
                }
                if (blendEdge.EdgeEndGapFillSurface != null)
                {
                    allSurfaces.Add(blendEdge.EdgeEndGapFillSurface);
                    openCornerTrimSurfacesMetaData.TryGetValue(blendEdge.EndCornerId, out var metadata);
                    patches.Add((null, blendEdge.BlendType == EdgeBlendType.Convex
                        ? blendEdge.EdgeEndGapFillSurface.IsSurfacePlanar() ? PlanarClosureMetadata(blendEdge.EdgeEndGapFillSurface) : null : metadata));
                }
            }

            List<List<Rat3Hybrid>> edgeTerminationArcs = new List<List<Rat3Hybrid>>();
            foreach (var e in blendEdges)
            {
                e.RefreshTrimArcs();
                edgeTerminationArcs.AddRange(e.TrimArcs);
            }

            var res = SegmentConnector.Connect(
                edgeTerminationArcs,
                l => l[0],
                l => l[l.Count - 1],
                (a, b) => a == b,
                out var closed);

            if (blendEdges.Count > 1)
            {
                int cornerCount = res.Count;
                for (int i = 0; i < cornerCount; ++i)
                {
                    if (!closed[i])
                    {
                        throw new InvalidOperationException("Spherical corner boundary arcs are disconnected.");
                    }

                    List<Rat3Hybrid> cornerOutline = Resolve(res[i], edgeTerminationArcs, out var cornerIndices);
                    List<Vec3D> cornerOutlineV3 = cc.Convert(cornerOutline);

                    var corner = profile.BuildCornerPatch(
                        cornerOutlineV3, cornerOutline, cornerIndices, cc, blendType, maxDiscretizationDeviation);
                    BlendCorner.OrientToNeighbours(corner, allSurfaces);
                    // Tessellation near a chordal support can extend outside
                    // the source shell. Clip corners just as the strips are clipped.
                    if (blendType == EdgeBlendType.Convex)
                    {
                        var cornerMesh = BlendEdge.ToMesh(corner, cc);
                        var clippedCorner = MeshNormalUV.BooleanOperation(cornerMesh, fullMesh,
                            CSG.BooleanOp.AAsSurfaceBAsTrimVolumeKeepInside, cc);
                        var clippedPatch = new AnchorMesh("corner", clippedCorner,
                            new Dictionary<int, string> { { 0, "corner" } }, new Dictionary<string, SurfaceMetaData>(),
                            deferCoplanarPostProcess: true, skipCoplanarFusion: true, isVolume: false);
                        clippedPatch.TryGetTopologySurface("corner", out corner);
                    }
                    allSurfaces.Add(corner);
                    patches.Add((null, null));
                }
            }

            if (allocateGroupIds != null)
                groupIdOffset = allocateGroupIds(allSurfaces.Count);
            MeshNormalUV surface = BlendEdge.ToMesh(allSurfaces, cc, ref groupIdOffset);
            return BlendEdge.ApplyBlendSurfaceToVolume(surface, fullMesh, cc, blendType);
        }

        private static List<Rat3Hybrid> Resolve(List<int> encodedSourceIndices, List<List<Rat3Hybrid>> source, out List<int> cornerIndices)
        {
            List<Rat3Hybrid> result = new List<Rat3Hybrid>();
            cornerIndices = new List<int>(encodedSourceIndices.Count);
            for (int i = 0; i < encodedSourceIndices.Count; ++i)
            {
                int rawId = encodedSourceIndices[i];
                int id = Math.Abs(rawId);
                var strip = source[id];
                var copy = new List<Rat3Hybrid>(strip);
                if (rawId < 0)
                    copy.Reverse();

                for (int j = 1; j < copy.Count; ++j)
                {
                    result.Add(copy[j]);
                    if (j == copy.Count - 1)
                        cornerIndices.Add(result.Count - 1);
                }
            }
            return result;
        }

        private bool IsOpenEnd(EdgeGraphNode node)
        {
            if (node == null)
                throw new ArgumentNullException(nameof(node));

            var connectedEdges = node.ConnectedEdges;
            if (connectedEdges.Count == 0)
                return true;

            foreach (var edge in connectedEdges)
            {
                bool isBeingBlended = false;
                foreach (var blendEdge in graphEdgesToBlend)
                {
                    if (edge.Id == blendEdge.Id)
                    {
                        isBeingBlended = true;
                        break;
                    }
                }

                if (!isBeingBlended)
                    return true;
            }

            return false;
        }

        private UVSurface FindTrimSurface(GraphEdge graphEdge, EdgeGraphNode node, AnchorMesh mesh, out string surfaceName)
        {
            if (graphEdge == null)
                throw new ArgumentNullException(nameof(graphEdge));
            if (node == null)
                throw new ArgumentNullException(nameof(node));

            int groupA = graphEdge.GroupIdA;
            int groupB = graphEdge.GroupIdB;

            HashSet<int> adjacentToA = new HashSet<int>();
            foreach (var edge in edgeGraph.Edges)
            {
                if (edge.GroupIdA == groupA)
                    adjacentToA.Add(edge.GroupIdB);
                else if (edge.GroupIdB == groupA)
                    adjacentToA.Add(edge.GroupIdA);
            }

            HashSet<int> adjacentToB = new HashSet<int>();
            foreach (var edge in edgeGraph.Edges)
            {
                if (edge.GroupIdA == groupB)
                    adjacentToB.Add(edge.GroupIdB);
                else if (edge.GroupIdB == groupB)
                    adjacentToB.Add(edge.GroupIdA);
            }

            List<int> trimSurfaceIds = new List<int>();
            foreach (int surfaceId in adjacentToA)
            {
                if (adjacentToB.Contains(surfaceId) && surfaceId != groupA && surfaceId != groupB)
                    trimSurfaceIds.Add(surfaceId);
            }

            // Common faces can meet the two supports at another endpoint.
            // Only a face incident to this graph node can terminate this strip.
            trimSurfaceIds.RemoveAll(id => !node.ConnectedEdges.Any(edge =>
                edge.GroupIdA == id || edge.GroupIdB == id));

            if (trimSurfaceIds.Count == 0)
            {
                // Convex strips ordinarily terminate by clipping against the
                // volume; a separate supporting closure is only needed for a
                // reentrant face. Rounded multi-face junctions may have none.
                if (graphEdge.BlendType == EdgeBlendType.Convex)
                {
                    surfaceName = null;
                    return null;
                }
                throw new Exception($"No trim surface found for open end at node {node.Id} on edge {graphEdge.Name}. " +
                    $"Graph edge connects surfaces {groupA} and {groupB}.");
            }
            if (trimSurfaceIds.Count == 1)
            {
                int trimSurfaceId = trimSurfaceIds[0];
                if (!mesh.groupIdToExtendedName.TryGetValue(trimSurfaceId, out surfaceName))
                    throw new Exception($"Trim surface with ID {trimSurfaceId} not found in group name mapping.");
                if (!mesh.TryGetTopologySurface(surfaceName, out UVSurface trimSurface) || trimSurface == null)
                    throw new Exception($"Trim surface '{surfaceName}' (ID {trimSurfaceId}) not found in mesh.");
                return trimSurface;
            }
            if (trimSurfaceIds.Count == 2)
            {
                foreach (int trimSurfaceId in trimSurfaceIds)
                {
                    foreach (var edge in node.ConnectedEdges)
                    {
                        if (edge.GroupIdA == trimSurfaceId || edge.GroupIdB == trimSurfaceId)
                        {
                            if (!mesh.groupIdToExtendedName.TryGetValue(trimSurfaceId, out surfaceName))
                                throw new Exception($"Trim surface with ID {trimSurfaceId} not found in group name mapping.");
                            if (!mesh.TryGetTopologySurface(surfaceName, out UVSurface trimSurface) || trimSurface == null)
                                throw new Exception($"Trim surface '{surfaceName}' (ID {trimSurfaceId}) not found in mesh.");
                            return trimSurface;
                        }
                    }
                }

                throw new Exception($"Two trim surfaces found ({trimSurfaceIds[0]}, {trimSurfaceIds[1]}) for open end at node {node.Id} " +
                    $"on edge {graphEdge.Name}, but neither is connected to the node.");
            }

            throw new Exception($"Found {trimSurfaceIds.Count} trim surfaces for open end at node {node.Id} on edge {graphEdge.Name}. " +
                $"Expected 1 or 2. Trim surface IDs: [{string.Join(", ", trimSurfaceIds)}]");
        }
    }
}
