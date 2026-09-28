#pragma warning disable CS8600 // Converting null literal or possible null value to non-nullable type
#pragma warning disable CS8625 // Cannot convert null literal to non-nullable reference type

using GeoCore;
using Geo.Shelling;

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

            // Near-collapsed closed cylinder rims become hemispherical caps.
            // Independent rims may be processed in sequence; mixed or adjacent
            // selections fall through to the ordinary blend pipeline unchanged.
            if (profile is FilletProfile && TryBlendCollapsedCylinderGroup(mesh, graphEdgesToBlend,
                profile, cc, maxDiscretizationDeviation, ref groupIdOffset, resultNameSuffix,
                allocateGroupIds, out var collapsedResult))
                return collapsedResult;

            blendEdges = CreateBlendEdges(graphEdgesToBlend, profile.OffsetDistance);
            originalSurfaces = GetOriginalSurfaces(blendTopology, blendEdges);
            var resolvedCorners = FindResolvableCorners(mesh, graphEdgesToBlend, profile.OffsetDistance, cc);

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
                FindClosedSupportCaps(blendEdges), resolvedCorners, out var patches, allocateGroupIds);
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

        private static bool TryBlendCollapsedCylinderGroup(AnchorMesh mesh, IReadOnlyList<GraphEdge> edges,
            IEdgeBlendProfile profile, CoordinateConverter cc, double maxDeviation, ref int groupIdOffset,
            string resultNameSuffix, Func<int, int> allocateGroupIds, out AnchorMesh result)
        {
            result = null;
            int startingGroupId = groupIdOffset;
            var usedVertices = new HashSet<int>();
            foreach (var edge in edges)
            {
                var edgeVertices = edge.EdgeSegments.SelectMany(segment => new[] { segment.X, segment.Y }).ToHashSet();
                if (usedVertices.Overlaps(edgeVertices)) return false;
                usedVertices.UnionWith(edgeVertices);
            }

            var current = mesh;
            foreach (var requestedEdge in edges)
            {
                var graph = new EdgeGraph(current.Mesh.Triangles, current.Mesh.GetTriangleGroups(),
                    current.Mesh.Positions, current.Mesh.PrecisionPositions, current.groupIdToExtendedName);
                if (!graph.TryGetEdge(requestedEdge.Name, out var edge) || edge == null ||
                    !SameSupportPair(mesh, requestedEdge, current, edge))
                {
                    groupIdOffset = startingGroupId;
                    return false;
                }

                // The rebuilt graph has not passed through CategorizeEdges; carry
                // over the already-computed category only when its named supports
                // are unchanged. This group handles disjoint rims, so earlier
                // replacements cannot alter this edge's local convexity.
                edge.BlendType = requestedEdge.BlendType;
                if (!CollapsedCylinderFillet.TryCreate(current, edge, profile.OffsetDistance, cc, maxDeviation,
                    ref groupIdOffset, out var sphereResult, out var sphereMetadata, allocateGroupIds))
                {
                    groupIdOffset = startingGroupId;
                    return false;
                }

                int patchGroup = groupIdOffset - 1;
                string patchName = profile.EdgePatchName(edge.Name);
                var names = new Dictionary<int, string>(current.groupIdToExtendedName) { [patchGroup] = patchName };
                var metadata = SurfaceMetaData.CloneDictionary(current.surfaceMetaData);
                metadata[patchName] = sphereMetadata;
                current = new AnchorMesh(current.Name + resultNameSuffix, sphereResult, names, metadata,
                    deferCoplanarPostProcess: false, skipCoplanarFusion: true, isVolume: current.IsVolume,
                    faceLineages: current.FaceLineages, ambiguousReferences: current.AmbiguousFaceReferences);
            }

            result = current;
            return true;
        }

        private static bool SameSupportPair(AnchorMesh originalMesh, GraphEdge originalEdge,
            AnchorMesh currentMesh, GraphEdge currentEdge)
        {
            var originalA = originalMesh.groupIdToExtendedName[originalEdge.GroupIdA];
            var originalB = originalMesh.groupIdToExtendedName[originalEdge.GroupIdB];
            var currentA = currentMesh.groupIdToExtendedName[currentEdge.GroupIdA];
            var currentB = currentMesh.groupIdToExtendedName[currentEdge.GroupIdB];
            return (originalA == currentA && originalB == currentB) ||
                (originalA == currentB && originalB == currentA);
        }

        // Three supports define one corner. Pairwise fillet construction does not
        // imply a common endpoint: resolve one shared junction through the same
        // support adapters used by offset construction, then reuse it on each strip.
        private static Dictionary<int, (Vec3D Center, HashSet<int> Groups)> FindResolvableCorners(
            AnchorMesh mesh, List<GraphEdge> edges, double radius, CoordinateConverter cc)
        {
            var answer = new Dictionary<int, (Vec3D, HashSet<int>)>();
            var selected = edges.Select(edge => edge.Name).ToHashSet(StringComparer.Ordinal);
            var registry = ShellSurfaceRegistry.AnalyticV1;
            foreach (var node in edges.SelectMany(edge => new[] { edge.StartNode, edge.EndNode }).Distinct())
            {
                int selectedDegree = node.ConnectedEdges.Count(edge => selected.Contains(edge.Name));
                if (selectedDegree < 3) continue;
                // This adapter's positive distance means inward, matching the
                // profile offset used when the blend spine is constructed.
                double offset = node.ConnectedEdges.First(edge => selected.Contains(edge.Name)).BlendType == EdgeBlendType.Convex
                    ? radius : -radius;
                var groups = node.ConnectedEdges.SelectMany(edge => new[] { edge.GroupIdA, edge.GroupIdB }).ToHashSet();
                if (groups.Count < 3) continue;
                var supports = new List<IShellSurfaceSupport>();
                foreach (int id in groups)
                {
                    string patch = mesh.groupIdToExtendedName[id];
                    if (!mesh.TryGetTopologySurface(patch, out var surface)) { supports.Clear(); break; }
                    var meta = mesh.surfaceMetaData[patch];
                    int index = mesh.Mesh.TrianglesEx.FindIndex(triangle => triangle.GroupId == id);
                    if (index < 0) { supports.Clear(); break; }
                    var triangle = mesh.Mesh.Triangles[index];
                    var normal = Vec3DOps.Cross(mesh.Mesh.Positions[triangle.B] - mesh.Mesh.Positions[triangle.A],
                        mesh.Mesh.Positions[triangle.C] - mesh.Mesh.Positions[triangle.A]).Normalized();
                    var adapter = registry.Resolve(meta, surface, offset);
                    if (adapter == null) { supports.Clear(); break; }
                    supports.Add(adapter.CreateOffsetSupport(meta, surface, normal, offset, cc));
                }
                if (supports.Count >= 3 && registry.TryResolveVertex(-1, node.Position, supports, 1e-8, out var center))
                    answer[node.Id] = (center, groups);
                else if (selectedDegree > 3)
                    // Even when a high-valence offset has no exact common miter,
                    // retain its incident-support set. The strips must be closed
                    // by their shared corner boundary, not trimmed against one
                    // another's nearly parallel infinite extensions.
                    answer[node.Id] = (node.Position, groups);
            }
            return answer;
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
            Dictionary<int, (Vec3D Center, HashSet<int> Groups)> resolvedCorners,
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

            // Intersections below are exact in the kernel's rational triangle
            // representation, not on the underlying continuous curved surface.
            // Project the common offset point back to each source support once;
            // independently recomputing pairwise contacts is what creates seams.
            var exactCorners = new Dictionary<int,
                (Rat3Hybrid Center, Dictionary<int, Rat3Hybrid> Contacts, bool IsExact)>();
            foreach (var (cornerId, corner) in resolvedCorners)
            {
                int[] groups = corner.Groups.OrderBy(id => id).ToArray();
                Rat3Hybrid point = default;
                bool tripleFound = groups.Length >= 3 && TryMultiSupportIntersection(groups,
                    allElargedOffsetSurfaces, corner.Center, cc, out point);
                var contacts = new Dictionary<int, Rat3Hybrid>(groups.Length);
                bool sharedContact = tripleFound;
                if (sharedContact)
                {
                    foreach (int group in groups)
                    {
                        if (!TryProjectOffsetContact(allElargedOffsetSurfaces[group],
                            allElargedSurfaces[group], point, out var contact))
                        {
                            sharedContact = false;
                            break;
                        }
                        contacts.Add(group, contact);
                    }
                }

                if (sharedContact)
                {
                    exactCorners.Add(cornerId, (point, contacts, true));
                    continue;
                }

                // Mesh offsets quantize each vertex independently, so five
                // nominally concurrent planar supports may miss one another by
                // a few lattice units. Give every incident strip one canonical
                // exact contact per support instead of cutting each strip with
                // its own approximate corner plane. Projection stays rational;
                // the explicit deviation check bounds the geometric correction.
                if (profile is not FilletProfile) continue;
                var roundedCenter = cc.Convert(corner.Center);
                point = new Rat3Hybrid(roundedCenter.X, roundedCenter.Y, roundedCenter.Z);
                contacts.Clear();
                sharedContact = true;
                foreach (int group in groups)
                {
                    Rat3Hybrid contact = default;
                    if (!allElargedOffsetSurfaces.TryGetValue(group, out var offsetSurface) ||
                        !allElargedSurfaces.TryGetValue(group, out var sourceSurface) ||
                        !TryProjectPlanarOffsetContact(offsetSurface, sourceSurface, point, cc,
                            maxDiscretizationDeviation, out contact))
                    {
                        sharedContact = false;
                        break;
                    }
                    contacts.Add(group, contact);
                }
                if (sharedContact)
                    exactCorners.Add(cornerId, (point, contacts, false));
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

                var anchoredEnds = new List<bool>(2);
                foreach (int cornerId in new[] { blendEdge.StartCornerId, blendEdge.EndCornerId }.Distinct())
                    if (exactCorners.TryGetValue(cornerId, out var corner))
                    {
                        Rat3Hybrid anchorCenter = corner.Center;
                        if (!corner.IsExact)
                        {
                            if (!blendEdge.TryProjectCornerCenter(corner.Center,
                                maxDiscretizationDeviation, out anchorCenter, out double centerError))
                                throw new InvalidOperationException(
                                    $"Fillet corner {cornerId} misses edge '{blendEdge.SourceEdge.Name}' spine by " +
                                    $"{centerError:G6}, beyond the requested deviation {maxDiscretizationDeviation:G6}.");
                            double radiusError = Math.Max(
                                Math.Abs((cc.Convert(corner.Contacts[blendEdge.SurfaceIndexA]) - cc.Convert(anchorCenter)).Length() - blendEdge.BlendRadius),
                                Math.Abs((cc.Convert(corner.Contacts[blendEdge.SurfaceIndexB]) - cc.Convert(anchorCenter)).Length() - blendEdge.BlendRadius));
                            if (radiusError > maxDiscretizationDeviation)
                                throw new InvalidOperationException(
                                    $"Fillet corner {cornerId} cannot share exact strip endpoints within the requested deviation " +
                                    $"(radius {radiusError:G6}, limit {maxDiscretizationDeviation:G6}).");
                        }
                        anchoredEnds.Add(blendEdge.AnchorCorner(cornerId, anchorCenter,
                            corner.Contacts[blendEdge.SurfaceIndexA], corner.Contacts[blendEdge.SurfaceIndexB]));
                    }

                profile.BuildStripSurface(blendEdge, cc, maxDiscretizationDeviation);
                foreach (bool first in anchoredEnds) blendEdge.RegisterCornerArc(first);


                foreach (var v in allElargedOffsetSurfaces)
                {
                    var surfId = v.Key;
                    if (collapsedSurfaces.Contains(surfId) || surfId == blendEdge.SurfaceIndexA || surfId == blendEdge.SurfaceIndexB)
                        continue;
                    if ((resolvedCorners.TryGetValue(blendEdge.StartCornerId, out var startCorner) && startCorner.Groups.Contains(surfId)) ||
                        (resolvedCorners.TryGetValue(blendEdge.EndCornerId, out var endCorner) && endCorner.Groups.Contains(surfId)))
                        continue;
                    try { blendEdge.TrimByOffsetSurface(blendEdge.PerpendicularCornerTrimSurface(v.Value)); }
                    catch (Exception ex)
                    {
                        throw new InvalidOperationException($"Fillet edge '{blendEdge.SourceEdge.Name}' failed trimming by support {surfId}; resolved corners {resolvedCorners.Count}.", ex);
                    }
                }

                foreach (int cornerId in new[] { blendEdge.StartCornerId, blendEdge.EndCornerId }.Distinct())
                    if (resolvedCorners.TryGetValue(cornerId, out var corner) && !exactCorners.ContainsKey(cornerId))
                    {
                        try
                        {
                            blendEdge.TrimByOffsetSurface(CornerCutPlane(blendEdge, corner.Center, cornerId,
                                cc));
                        }
                        catch (Exception ex)
                        {
                            throw new InvalidOperationException($"Fillet corner {cornerId} on '{blendEdge.SourceEdge.Name}' could not be trimmed at {corner.Center}; spine {blendEdge.CenterCurveVec3[0]} to {blendEdge.CenterCurveVec3[^1]}.", ex);
                        }
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
            var arcOwners = new List<string>();
            var arcOwnerEdges = new List<BlendEdge>();
            foreach (var e in blendEdges)
            {
                e.RefreshTrimArcs();
                edgeTerminationArcs.AddRange(e.TrimArcs);
                arcOwners.AddRange(Enumerable.Repeat(e.SourceEdge.Name, e.TrimArcs.Count));
                for (int i = 0; i < e.TrimArcs.Count; ++i)
                    arcOwnerEdges.Add(e);
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
                        string owners = string.Join(", ", res[i].Select(index => arcOwners[Math.Abs(index)]).Distinct());
                        throw new InvalidOperationException(
                            $"Spherical corner boundary arcs did not form a closed loop (source edges: {owners}).");
                    }

                    List<Rat3Hybrid> cornerOutline = Resolve(res[i], edgeTerminationArcs, out var cornerIndices);
                    List<Vec3D> cornerOutlineV3 = cc.Convert(cornerOutline);

                    Vec3D? exactSphereCenter = null;
                    int firstCandidate = -1, secondCandidate = -1;
                    bool firstArc = true;
                    foreach (int arcIndex in res[i])
                    {
                        var owner = arcOwnerEdges[Math.Abs(arcIndex)];
                        if (firstArc)
                        {
                            firstCandidate = owner.StartCornerId;
                            secondCandidate = owner.EndCornerId;
                            firstArc = false;
                            continue;
                        }
                        if (firstCandidate != owner.StartCornerId && firstCandidate != owner.EndCornerId)
                            firstCandidate = -1;
                        if (secondCandidate != owner.StartCornerId && secondCandidate != owner.EndCornerId)
                            secondCandidate = -1;
                    }
                    int? ownerCornerId = firstCandidate >= 0 && secondCandidate < 0 ? firstCandidate :
                        secondCandidate >= 0 && firstCandidate < 0 ? secondCandidate :
                        firstCandidate == secondCandidate && firstCandidate >= 0 ? firstCandidate : null;
                    bool resolvedCorner = false;
                    if (ownerCornerId.HasValue &&
                        resolvedCorners.TryGetValue(ownerCornerId.Value, out var resolved))
                    {
                        resolvedCorner = true;
                        exactSphereCenter = exactCorners.TryGetValue(ownerCornerId.Value, out var exact)
                            ? cc.Convert(exact.Center) : resolved.Center;
                    }

                    var corner = profile.BuildCornerPatch(
                        cornerOutlineV3, cornerOutline, cornerIndices, cc, blendType, maxDiscretizationDeviation,
                        exactSphereCenter);
                    // Tessellation near a chordal support can extend outside
                    // the source shell. Clip corners just as the strips are clipped.
                    // Exact common-support rolling-ball corners are already bounded
                    // by their common support intersection. Clipping their
                    // open tessellated caps through the volume can be partial
                    // by construction; the caller still checks the final
                    // rounded solid against the sharp offset envelope.
                    if (blendType == EdgeBlendType.Convex && !resolvedCorner)
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

            BlendCorner.OrientBlendPatches(allSurfaces);
            ValidateBlendPatchBoundaries(allSurfaces, patches, fullMesh, cc);

            if (allocateGroupIds != null)
                groupIdOffset = allocateGroupIds(allSurfaces.Count);
            MeshNormalUV surface = BlendEdge.ToMesh(allSurfaces, cc, ref groupIdOffset);
            return BlendEdge.ApplyBlendSurfaceToVolume(surface, fullMesh, cc, blendType);
        }

        private static bool TryTripleSupportIntersection(UVSurface first, UVSurface second, UVSurface third,
            Vec3D expected, CoordinateConverter converter, out Rat3Hybrid point)
        {
            point = default;
            var (segments, _) = CSG.Intersector.IntersectSurfaces(first, second, converter);
            double best = double.PositiveInfinity;
            bool found = false;
            foreach (var segment in segments)
                foreach (var triangle in third.Triangles)
                {
                    var a = third.PointsPrecise[triangle.A];
                    var b = third.PointsPrecise[triangle.B];
                    var c = third.PointsPrecise[triangle.C];
                    if (CSG.TriangleSegmentIntersector.SegmentIntersectsTriangle(
                        segment.PointStart, segment.PointEnd, a, b, c,
                        out var candidate, out _, out _, out _) != CSG.SegmentTriangleIntersectionType.Intersect)
                        continue;
                    double distance = (converter.Convert(candidate) - expected).LengthSquared();
                    if (distance >= best) continue;
                    best = distance;
                    point = candidate;
                    found = true;
                }
            return found;
        }

        private static bool TryMultiSupportIntersection(IReadOnlyList<int> groups,
            IReadOnlyDictionary<int, UVSurface> supports, Vec3D expected,
            CoordinateConverter converter, out Rat3Hybrid point)
        {
            point = default;
            double best = double.PositiveInfinity;
            bool found = false;
            for (int i = 0; i < groups.Count - 2; i++)
                for (int j = i + 1; j < groups.Count - 1; j++)
                    for (int k = j + 1; k < groups.Count; k++)
                    {
                        if (!supports.TryGetValue(groups[i], out var first) ||
                            !supports.TryGetValue(groups[j], out var second) ||
                            !supports.TryGetValue(groups[k], out var third) ||
                            !TryTripleSupportIntersection(first, second, third, expected,
                                converter, out var candidate)) continue;
                        double distance = (converter.Convert(candidate) - expected).LengthSquared();
                        if (distance >= best) continue;
                        best = distance;
                        point = candidate;
                        found = true;
                    }
            return found;
        }

        private static bool TryProjectOffsetContact(UVSurface offset, UVSurface source,
            Rat3Hybrid center, out Rat3Hybrid contact)
        {
            contact = default;
            for (int i = 0; i < offset.Triangles.Count; i++)
            {
                var triangle = offset.Triangles[i];
                var a = offset.PointsPrecise[triangle.A];
                var b = offset.PointsPrecise[triangle.B];
                var c = offset.PointsPrecise[triangle.C];
                var normal = Rat3Hybrid.Cross(b - a, c - a);
                if (normal.IsZero() || Rat3Hybrid.Dot(center - a, normal).Sign() != 0) continue;
                var barycentric = CSG.Intersector.ComputeBarycentricCoordinates(center, a, b, c);
                if (barycentric.X.Sign() < 0 || barycentric.Y.Sign() < 0 ||
                    barycentric.Z.Sign() < 0) continue;
                contact = source.PointsPrecise[triangle.A] * barycentric.X +
                    source.PointsPrecise[triangle.B] * barycentric.Y +
                    source.PointsPrecise[triangle.C] * barycentric.Z;
                contact.Simplify();
                return true;
            }
            return false;
        }

        private static bool TryProjectPlanarOffsetContact(UVSurface offset, UVSurface source,
            Rat3Hybrid center, CoordinateConverter converter, double maxDeviation,
            out Rat3Hybrid contact)
        {
            contact = default;
            if (offset.PointsPrecise.Count != source.PointsPrecise.Count ||
                source.Triangles.Count == 0 || !source.IsSurfacePlanar()) return false;

            var vertices = new HashSet<int>();
            foreach (var triangle in source.Triangles)
            {
                vertices.Add(triangle.A);
                vertices.Add(triangle.B);
                vertices.Add(triangle.C);
            }
            if (vertices.Count == 0) return false;

            var translation = new Rat3Hybrid(0, 0, 0);
            foreach (int vertex in vertices)
                translation += offset.PointsPrecise[vertex] - source.PointsPrecise[vertex];
            translation /= new BigRationalHybrid(vertices.Count);

            // A mesh offset can differ from this canonical translation only by
            // its coordinate discretization. Reject larger/nonuniform offsets;
            // they need their own surface-specific corner construction.
            foreach (int vertex in vertices)
            {
                var residual = offset.PointsPrecise[vertex] - source.PointsPrecise[vertex] - translation;
                double error = (converter.Convert(residual) - converter.Convert(new Rat3Hybrid(0, 0, 0))).Length();
                if (error > maxDeviation) return false;
            }

            var first = source.Triangles[0];
            var a = source.PointsPrecise[first.A];
            var normal = Rat3Hybrid.Cross(source.PointsPrecise[first.B] - a,
                source.PointsPrecise[first.C] - a);
            var normalLengthSquared = Rat3Hybrid.Dot(normal, normal);
            if (normalLengthSquared.Sign() == 0) return false;
            var offsetOrigin = a + translation;
            var projected = center - normal *
                (Rat3Hybrid.Dot(center - offsetOrigin, normal) / normalLengthSquared);
            double planeError = (converter.Convert(center) - converter.Convert(projected)).Length();
            if (planeError > maxDeviation) return false;

            var candidate = projected - translation;
            foreach (var triangle in source.Triangles)
            {
                var p0 = source.PointsPrecise[triangle.A];
                var p1 = source.PointsPrecise[triangle.B];
                var p2 = source.PointsPrecise[triangle.C];
                var barycentric = CSG.Intersector.ComputeBarycentricCoordinates(candidate, p0, p1, p2);
                if (barycentric.X.Sign() < 0 || barycentric.Y.Sign() < 0 || barycentric.Z.Sign() < 0)
                    continue;
                contact = p0 * barycentric.X + p1 * barycentric.Y + p2 * barycentric.Z;
                contact.Simplify();
                return true;
            }
            return false;
        }

        /// <summary>
        /// Rejects non-manifold patch edges, T-junctions, and interior open edges.
        /// A blend boundary may remain open only where the target volume boundary
        /// covers it. Do not repair these failures by snapping: that can hide a
        /// gap while leaving non-conforming topology.
        /// </summary>
        internal static void ValidateBlendPatchBoundaries(IReadOnlyList<UVSurface> surfaces,
            IReadOnlyList<(string Name, SurfaceMetaData Metadata)> patchNames,
            MeshNormalUV volume, CoordinateConverter converter)
        {
            static BigRationalHybrid Min2(BigRationalHybrid a, BigRationalHybrid b) => a < b ? a : b;
            static BigRationalHybrid Max2(BigRationalHybrid a, BigRationalHybrid b) => a > b ? a : b;
            static BigRationalHybrid Min3(BigRationalHybrid a, BigRationalHybrid b, BigRationalHybrid c) =>
                Min2(Min2(a, b), c);
            static BigRationalHybrid Max3(BigRationalHybrid a, BigRationalHybrid b, BigRationalHybrid c) =>
                Max2(Max2(a, b), c);
            static (Rat3Hybrid Min, Rat3Hybrid Max) SegmentBounds(Rat3Hybrid a, Rat3Hybrid b) =>
                (new Rat3Hybrid(Min2(a.X, b.X), Min2(a.Y, b.Y), Min2(a.Z, b.Z)),
                 new Rat3Hybrid(Max2(a.X, b.X), Max2(a.Y, b.Y), Max2(a.Z, b.Z)));
            static bool BoundsOverlap((Rat3Hybrid Min, Rat3Hybrid Max) a,
                (Rat3Hybrid Min, Rat3Hybrid Max) b) =>
                a.Max.X >= b.Min.X && a.Min.X <= b.Max.X &&
                a.Max.Y >= b.Min.Y && a.Min.Y <= b.Max.Y &&
                a.Max.Z >= b.Min.Z && a.Min.Z <= b.Max.Z;

            static int Compare(Rat3Hybrid a, Rat3Hybrid b)
            {
                int result = a.X.CompareTo(b.X);
                if (result != 0) return result;
                result = a.Y.CompareTo(b.Y);
                return result != 0 ? result : a.Z.CompareTo(b.Z);
            }

            var edgeUses = new Dictionary<(Rat3Hybrid A, Rat3Hybrid B), List<int>>();
            for (int surfaceId = 0; surfaceId < surfaces.Count; surfaceId++)
            {
                var surface = surfaces[surfaceId];
                foreach (var (edge, adjacent) in AdjacencyEx.BuildEdgeToTrianglesMap(surface.Triangles))
                {
                    if (adjacent.Count != 1) continue;
                    var (a, b) = edge;
                    var triangle = surface.Triangles[adjacent[0]];
                    if (!((triangle.A == a && triangle.B == b) ||
                          (triangle.B == a && triangle.C == b) ||
                          (triangle.C == a && triangle.A == b)))
                        (a, b) = (b, a);
                    var first = surface.PointsPrecise[a];
                    var second = surface.PointsPrecise[b];
                    var key = Compare(first, second) < 0 ? (first, second) : (second, first);
                    if (!edgeUses.TryGetValue(key, out var owners)) edgeUses.Add(key, owners = new List<int>(2));
                    owners.Add(surfaceId);
                }
            }

            static bool LiesStrictlyOnSegment(Rat3Hybrid point, Rat3Hybrid start, Rat3Hybrid end)
            {
                var direction = end - start;
                var offset = point - start;
                if (Rat3Hybrid.Cross(direction, offset) != new Rat3Hybrid(0, 0, 0)) return false;
                var along = Rat3Hybrid.Dot(offset, direction);
                return along.Sign() > 0 && along.CompareTo(Rat3Hybrid.Dot(direction, direction)) < 0;
            }

            var points = volume.PrecisionPositions;
            var boundaryEdges = new List<(Rat3Hybrid A, Rat3Hybrid B, int Surface)>();
            foreach (var (edge, owners) in edgeUses)
            {
                if (owners.Count > 2)
                    throw new InvalidOperationException("Edge-blend patches have a non-manifold shared edge.");
                if (owners.Count == 2) continue;
                boundaryEdges.Add((edge.A, edge.B, owners[0]));
            }
            var boundaryBounds = boundaryEdges.Select(edge => SegmentBounds(edge.A, edge.B)).ToArray();

            for (int i = 0; i < boundaryEdges.Count; i++)
                for (int j = i + 1; j < boundaryEdges.Count; j++)
                {
                    if (!BoundsOverlap(boundaryBounds[i], boundaryBounds[j])) continue;
                    var a = boundaryEdges[i];
                    var b = boundaryEdges[j];
                    if (LiesStrictlyOnSegment(a.A, b.A, b.B) || LiesStrictlyOnSegment(a.B, b.A, b.B) ||
                        LiesStrictlyOnSegment(b.A, a.A, a.B) || LiesStrictlyOnSegment(b.B, a.A, a.B))
                        throw new InvalidOperationException(
                            $"Edge-blend patches contain a non-conforming T-junction between patches {a.Surface} and {b.Surface}.");
                }

            var boundaryValence = new Dictionary<Rat3Hybrid, int>();
            foreach (var edge in boundaryEdges)
            {
                boundaryValence.TryGetValue(edge.A, out int atA);
                boundaryValence[edge.A] = atA + 1;
                boundaryValence.TryGetValue(edge.B, out int atB);
                boundaryValence[edge.B] = atB + 1;
            }
            foreach (var (point, valence) in boundaryValence)
                if (valence != 2)
                    throw new InvalidOperationException(
                        $"Edge-blend trim boundary is not a closed contour: exact boundary vertex " +
                        $"{converter.Convert(point)} has valence {valence}; boundary loops must join exactly.");

            var triangleBounds = volume.Triangles.Select(triangle =>
            {
                var a = points[triangle.A];
                var b = points[triangle.B];
                var c = points[triangle.C];
                return (Min: new Rat3Hybrid(Min3(a.X, b.X, c.X), Min3(a.Y, b.Y, c.Y),
                                           Min3(a.Z, b.Z, c.Z)),
                        Max: new Rat3Hybrid(Max3(a.X, b.X, c.X), Max3(a.Y, b.Y, c.Y),
                                           Max3(a.Z, b.Z, c.Z)));
            }).ToArray();
            static bool CoveredByBoundaryTriangles((Rat3Hybrid A, Rat3Hybrid B, int Surface) edge,
                (Rat3Hybrid Min, Rat3Hybrid Max) edgeBounds, MeshNormalUV volume,
                List<Rat3Hybrid> points, (Rat3Hybrid Min, Rat3Hybrid Max)[] triangleBounds)
            {
                var intervals = new List<(BigRationalHybrid Start, BigRationalHybrid End)>();
                for (int triangleIndex = 0; triangleIndex < volume.Triangles.Count; triangleIndex++)
                {
                    if (!BoundsOverlap(edgeBounds, triangleBounds[triangleIndex])) continue;
                    var triangle = volume.Triangles[triangleIndex];
                    var a = points[triangle.A];
                    var b = points[triangle.B];
                    var c = points[triangle.C];
                    var normal = Rat3Hybrid.Cross(b - a, c - a);
                    if (normal == new Rat3Hybrid(0, 0, 0) ||
                        Rat3Hybrid.Dot(edge.A - a, normal).Sign() != 0 ||
                        Rat3Hybrid.Dot(edge.B - a, normal).Sign() != 0)
                        continue;

                    var low = BigRationalHybrid.Zero;
                    var high = BigRationalHybrid.One;
                    bool Clip(BigRationalHybrid startSide, BigRationalHybrid endSide)
                    {
                        if (startSide.Sign() < 0 && endSide.Sign() < 0) return false;
                        if (startSide.Sign() >= 0 && endSide.Sign() >= 0) return true;
                        var crossing = startSide / (startSide - endSide);
                        if (startSide.Sign() < 0)
                        {
                            if (crossing > low) low = crossing;
                        }
                        else if (crossing < high) high = crossing;
                        return low <= high;
                    }

                    if (!Clip(Rat3Hybrid.Dot(Rat3Hybrid.Cross(b - a, edge.A - a), normal),
                              Rat3Hybrid.Dot(Rat3Hybrid.Cross(b - a, edge.B - a), normal)) ||
                        !Clip(Rat3Hybrid.Dot(Rat3Hybrid.Cross(c - b, edge.A - b), normal),
                              Rat3Hybrid.Dot(Rat3Hybrid.Cross(c - b, edge.B - b), normal)) ||
                        !Clip(Rat3Hybrid.Dot(Rat3Hybrid.Cross(a - c, edge.A - c), normal),
                              Rat3Hybrid.Dot(Rat3Hybrid.Cross(a - c, edge.B - c), normal)))
                        continue;
                    intervals.Add((low, high));
                }

                intervals.Sort((left, right) => left.Start.CompareTo(right.Start));
                var coveredUntil = BigRationalHybrid.Zero;
                foreach (var interval in intervals)
                {
                    if (interval.Start > coveredUntil) return false;
                    if (interval.End > coveredUntil) coveredUntil = interval.End;
                    if (coveredUntil >= BigRationalHybrid.One) return true;
                }
                return false;
            }

            for (int edgeIndex = 0; edgeIndex < boundaryEdges.Count; edgeIndex++)
            {
                var edge = boundaryEdges[edgeIndex];
                if (!CoveredByBoundaryTriangles(edge, boundaryBounds[edgeIndex], volume, points, triangleBounds))
                {
                    string patchName = patchNames[edge.Surface].Name ?? $"closure/corner {edge.Surface}";
                    throw new InvalidOperationException(
                        $"Edge-blend patch '{patchName}' has an interior open boundary from {converter.Convert(edge.A)} " +
                        $"to {converter.Convert(edge.B)}; edge/corner patches must join exactly.");
                }
            }
        }

        private static UVSurface CornerCutPlane(BlendEdge edge, Vec3D center, int cornerId,
            CoordinateConverter cc)
        {
            var node = cornerId == edge.StartCornerId ? edge.SourceEdge.StartNode : edge.SourceEdge.EndNode;
            var spine = edge.CenterCurveVec3;
            bool first = (spine[0] - node.Position).LengthSquared() < (spine[^1] - node.Position).LengthSquared();
            Rat3Hybrid outward = first
                ? edge.CenterCurve[0] - edge.CenterCurve[1]
                : edge.CenterCurve[^1] - edge.CenterCurve[^2];
            // The trim keeps the side opposite the normal for convex blends,
            // and the normal side for concave blends. Point the normal toward
            // the corner only for convex strips so each cut retains the span.
            if (edge.BlendType == EdgeBlendType.Concave) outward = -outward;
            if (outward == new Rat3Hybrid(0, 0, 0))
                throw new InvalidOperationException($"Fillet edge '{edge.SourceEdge.Name}' has a degenerate corner direction.");
            var centerPoint = cc.Convert(center);
            var origin = new Rat3Hybrid(centerPoint.X, centerPoint.Y, centerPoint.Z);
            if (edge.RawSurface == null || edge.RawSurface.Triangles.Count == 0)
                throw new InvalidOperationException($"Fillet edge '{edge.SourceEdge.Name}' has no corner-trim target surface.");
            return BlendCorner.PlaneCoveringSurface(edge.RawSurface, origin, outward, cc);
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
