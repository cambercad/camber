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

        public AnchorMesh Run(
            AnchorMesh mesh,
            List<string> edgeNamesToBlend,
            IEdgeBlendProfile profile,
            CoordinateConverter cc,
            double maxDiscretizationDeviation,
            ref int groupIdOffset,
            string resultNameSuffix = "_blended")
        {
            edgeGraph = new EdgeGraph(
                mesh.Mesh.Triangles,
                mesh.Mesh.GetTriangleGroups(),
                mesh.Mesh.Positions,
                mesh.Mesh.PrecisionPositions,
                mesh.groupIdToExtendedName);

            CategorizeEdges(
                edgeGraph,
                mesh.Mesh.Triangles,
                mesh.Mesh.GetTriangleGroups(),
                mesh.Mesh.Positions,
                edgeNamesToBlend);

            List<List<string>> blendGroups = PartitionEdgesIntoBlendGroups(edgeGraph, edgeNamesToBlend);
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
                    resultNameSuffix);
            }

            return resultMesh;
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
            List<string> edgeNamesToBlend)
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
                if (v.Key.ConnectedEdges.Count == v.Value && v.Value > 0)
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
            string resultNameSuffix)
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

            blendEdges = CreateBlendEdges(graphEdgesToBlend, profile.OffsetDistance);
            originalSurfaces = GetOriginalSurfaces(mesh, blendEdges);

            Dictionary<int, UVSurface> openCornerTrimSurfaces = new Dictionary<int, UVSurface>();
            Dictionary<int, SurfaceMetaData> openCornerTrimSurfacesMetaData = new Dictionary<int, SurfaceMetaData>();
            if (blendEdges.First().BlendType == EdgeBlendType.Concave)
            {
                for (int i = 0; i < blendEdges.Count; i++)
                {
                    var blendEdge = blendEdges[i];
                    var e = graphEdgesToBlend[i];
                    if (IsOpenEnd(e.StartNode))
                    {
                        openCornerTrimSurfaces[blendEdge.StartCornerId] = FindTrimSurface(e, e.StartNode, mesh, out var surfaceName);
                        if (mesh.surfaceMetaData.TryGetValue(surfaceName, out var metaData))
                            openCornerTrimSurfacesMetaData[blendEdge.StartCornerId] = metaData;
                    }
                    if (IsOpenEnd(e.EndNode))
                    {
                        openCornerTrimSurfaces[blendEdge.EndCornerId] = FindTrimSurface(e, e.EndNode, mesh, out var surfaceName);
                        if (mesh.surfaceMetaData.TryGetValue(surfaceName, out var metaData))
                            openCornerTrimSurfacesMetaData[blendEdge.EndCornerId] = metaData;
                    }
                }
            }

            int startingGroupId = groupIdOffset;
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
                openCornerTrimSurfacesMetaData);

            Dictionary<int, string> completeGroupMapping = new Dictionary<int, string>(mesh.groupIdToExtendedName);
            int numBlendSurfaces = groupIdOffset - startingGroupId;

            int blendSurfaceIndex = 0;
            for (int i = 0; i < blendEdges.Count; i++)
            {
                int groupId = startingGroupId + blendSurfaceIndex;
                completeGroupMapping[groupId] = profile.EdgePatchName(blendEdges[i].SourceEdge.Name);
                blendSurfaceIndex++;
            }

            for (int i = blendSurfaceIndex; i < numBlendSurfaces; i++)
            {
                int groupId = startingGroupId + i;
                completeGroupMapping[groupId] = profile.CornerPatchName(i - blendSurfaceIndex);
            }

            var meta = SurfaceMetaData.CloneDictionary(mesh.surfaceMetaData);
            foreach (var kv in completeGroupMapping)
            {
                if (!meta.ContainsKey(kv.Value))
                    meta[kv.Value] = new SurfaceMetaData(SurfaceType.Unknown);
            }

            var probeMesh = new AnchorMesh(mesh.Name + "_probe", result, completeGroupMapping, meta, deferCoplanarPostProcess: true, isVolume: mesh.IsVolume);
            profile.AttachPatchNurbs(meta, blendEdges, probeMesh);

            return new AnchorMesh(mesh.Name + resultNameSuffix, result, completeGroupMapping, meta, deferCoplanarPostProcess: false, skipCoplanarFusion: true, isVolume: mesh.IsVolume);
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
                if (mesh.TryGetSurface(surfaceName, out UVSurface surface) && surface != null)
                    surfaces[groupId] = surface;
            }

            return surfaces;
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
            Dictionary<int, SurfaceMetaData> openCornerTrimSurfacesMetaData)
        {
            EdgeBlendType blendType = blendEdges.Count > 0 ? blendEdges[0].BlendType : EdgeBlendType.Convex;

            Dictionary<int, UVSurface> allElargedOffsetSurfaces = new Dictionary<int, UVSurface>(originalSurfaces.Count);
            Dictionary<int, UVSurface> allElargedSurfaces = new Dictionary<int, UVSurface>(originalSurfaces.Count);

            double enlarge = 2 * edgeOffsetDistance;
            double offset = blendType == EdgeBlendType.Convex ? -edgeOffsetDistance : edgeOffsetDistance;

            foreach (var v in originalSurfaces)
            {
                var extendedSurface = v.Value.GetExtendedSurface(enlarge, cc, out _);
                var extendedAndOffsetSurface = extendedSurface.GetOffsetSurface(offset, cc);
                allElargedSurfaces.Add(v.Key, extendedSurface);
                allElargedOffsetSurfaces.Add(v.Key, extendedAndOffsetSurface);
            }

            Dictionary<int, UVSurface> extendedOpenCornerTrimSurfaces = new Dictionary<int, UVSurface>();
            Dictionary<int, UVSurface> extensionOnlyOpenCornerTrimSurfaces = new Dictionary<int, UVSurface>();
            foreach (var v in openCornerTrimSurfaces)
            {
                var extendedSurface = v.Value.GetExtendedSurface(enlarge, cc, out var extensionOnly);
                extendedOpenCornerTrimSurfaces.Add(v.Key, extendedSurface);
                extensionOnlyOpenCornerTrimSurfaces.Add(v.Key, extensionOnly);
            }

            List<UVSurface> allSurfaces = new List<UVSurface>();
            foreach (var blendEdge in blendEdges)
            {
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
                    if (surfId == blendEdge.SurfaceIndexA || surfId == blendEdge.SurfaceIndexB)
                        continue;
                    blendEdge.TrimByOffsetSurface(v.Value);
                }

                if (blendEdge.BlendType == EdgeBlendType.Convex)
                    blendEdge.TrimByVolume(fullMesh, cc);
                else
                    blendEdge.TrimBySurface(extendedOpenCornerTrimSurfaces, extensionOnlyOpenCornerTrimSurfaces);

                allSurfaces.Add(blendEdge.BlendSurface);
                if (blendEdge.EdgeStartGapFillSurface != null)
                    allSurfaces.Add(blendEdge.EdgeStartGapFillSurface);
                if (blendEdge.EdgeEndGapFillSurface != null)
                    allSurfaces.Add(blendEdge.EdgeEndGapFillSurface);
            }

            List<List<Rat3Hybrid>> edgeTerminationArcs = new List<List<Rat3Hybrid>>();
            foreach (var e in blendEdges)
                edgeTerminationArcs.AddRange(e.TrimArcs);

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
                        throw new Exception();

                    List<Rat3Hybrid> cornerOutline = Resolve(res[i], edgeTerminationArcs, out var cornerIndices);
                    List<Vec3D> cornerOutlineV3 = cc.Convert(cornerOutline);

                    var corner = profile.BuildCornerPatch(
                        cornerOutlineV3, cornerOutline, cornerIndices, cc, blendType, maxDiscretizationDeviation);
                    allSurfaces.Add(corner);
                }
            }

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

            if (trimSurfaceIds.Count == 0)
            {
                throw new Exception($"No trim surface found for open end at node {node.Id} on edge {graphEdge.Name}. " +
                    $"Graph edge connects surfaces {groupA} and {groupB}.");
            }
            if (trimSurfaceIds.Count == 1)
            {
                int trimSurfaceId = trimSurfaceIds[0];
                if (!mesh.groupIdToExtendedName.TryGetValue(trimSurfaceId, out surfaceName))
                    throw new Exception($"Trim surface with ID {trimSurfaceId} not found in group name mapping.");
                if (!mesh.TryGetSurface(surfaceName, out UVSurface trimSurface) || trimSurface == null)
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
                            if (!mesh.TryGetSurface(surfaceName, out UVSurface trimSurface) || trimSurface == null)
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
