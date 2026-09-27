using Geo.Shelling;
using GeoCore;
using GeoMeta;

namespace Geo;

public partial class GeoAPI
{
    /// <summary>
    /// Turn an oriented surface into a solid. Positive thickness follows the surface
    /// normals; negative thickness goes against them. With <paramref name="bothSides"/>,
    /// the same distance is applied on both sides of the source surface.
    /// </summary>
    [APIDescription(@"Thicken(surface: AnchorMesh, thickness: float, bothSides: bool = false, name: str = None) -> AnchorMesh
Creates a watertight solid from an oriented surface. Positive thickness follows its normals; negative thickness goes against them. With bothSides, the absolute distance is applied to each side.")]
    public AnchorMesh Thicken(AnchorMesh surface, double thickness, bool bothSides = false,
        string name = null)
    {
        if (surface == null) throw new ArgumentNullException(nameof(surface));
        if (surface.IsVolume)
            throw new ArgumentException("Thicken requires a surface, not a closed solid.", nameof(surface));
        if (!double.IsFinite(thickness) || thickness == 0)
            throw new ArgumentOutOfRangeException(nameof(thickness), "Thickness must be finite and nonzero.");
        if (surface.Mesh.Triangles.Count == 0)
            throw new ArgumentException("Thicken requires a nonempty surface.", nameof(surface));

        var boundary = ValidateSurfaceTopology(surface.Mesh);
        double distance = Math.Abs(thickness);
        double lowNormalOffset = bothSides ? -distance : Math.Min(0, thickness);
        double highNormalOffset = bothSides ? distance : Math.Max(0, thickness);
        var builder = new ShellTopologyBuilder(converter, ShellSurfaceRegistry.AnalyticV1);
        var low = builder.ResolveOffsetSkin(surface, -lowNormalOffset);
        var high = builder.ResolveOffsetSkin(surface, -highNormalOffset);

        int patchCount = surface.groupIdToExtendedName.Count;
        int firstGroup = ReserveGroupIds(checked(patchCount * 2 + boundary.Loops.Count));
        var negativeGroups = new Dictionary<int, int>();
        var positiveGroups = new Dictionary<int, int>();
        var groupNames = new Dictionary<int, string>();
        var metadata = new Dictionary<string, SurfaceMetaData>();
        int nextGroup = firstGroup;
        var sourceGroups = surface.groupIdToExtendedName.Keys.ToList();
        sourceGroups.Sort();
        foreach (int sourceGroup in sourceGroups)
        {
            string sourceName = surface.groupIdToExtendedName[sourceGroup];
            int negative = nextGroup++;
            int positive = nextGroup++;
            negativeGroups.Add(sourceGroup, negative);
            positiveGroups.Add(sourceGroup, positive);
            string negativeName = EntityNaming.ThickenNegative(sourceName);
            string positiveName = EntityNaming.ThickenPositive(sourceName);
            groupNames.Add(negative, negativeName);
            groupNames.Add(positive, positiveName);
            metadata.Add(negativeName, MetadataAt(low, sourceGroup, lowNormalOffset));
            metadata.Add(positiveName, MetadataAt(high, sourceGroup, highNormalOffset));

            SurfaceMetaData MetadataAt(ShellTopologyBuilder.OffsetSkin skin, int group, double normalOffset)
            {
                if (normalOffset == 0 && surface.surfaceMetaData != null &&
                    surface.surfaceMetaData.TryGetValue(sourceName, out var sourceMetadata) && sourceMetadata != null)
                    return sourceMetadata.Clone();
                return skin.Supports[group].Metadata.Clone();
            }
        }

        var points = new List<Vec3D>(low.Points.Count + high.Points.Count);
        points.AddRange(low.Points);
        points.AddRange(high.Points);
        var precise = new List<Rat3Hybrid>(low.PrecisePoints.Count + high.PrecisePoints.Count);
        precise.AddRange(low.PrecisePoints);
        precise.AddRange(high.PrecisePoints);
        int highBase = low.Points.Count;
        var triangles = new List<Tri>(surface.Mesh.Triangles.Count * 2 + boundary.Edges.Count * 2);
        var corners = new List<MeshTriangle<TriangleVertexNormalUV>>(triangles.Capacity);

        for (int i = 0; i < surface.Mesh.Triangles.Count; i++)
        {
            Tri sourceTriangle = surface.Mesh.Triangles[i];
            var sourceCorners = surface.Mesh.TrianglesEx[i];
            triangles.Add(new Tri(sourceTriangle.A, sourceTriangle.C, sourceTriangle.B));
            corners.Add(Reverse(sourceCorners, negativeGroups[sourceCorners.GroupId]));
            triangles.Add(new Tri(sourceTriangle.A + highBase, sourceTriangle.B + highBase,
                sourceTriangle.C + highBase));
            sourceCorners.GroupId = positiveGroups[sourceCorners.GroupId];
            corners.Add(sourceCorners);
        }

        for (int loopIndex = 0; loopIndex < boundary.Loops.Count; loopIndex++)
        {
            int rimGroup = nextGroup++;
            string rimName = EntityNaming.ThickenRim(loopIndex + 1);
            groupNames.Add(rimGroup, rimName);
            metadata.Add(rimName, new SurfaceMetaData(SurfaceType.Unknown));
            foreach (var edge in boundary.Loops[loopIndex])
            {
                int lowA = edge.Start;
                int lowB = edge.End;
                int highA = lowA + highBase;
                int highB = lowB + highBase;
                AddWallTriangle(lowA, lowB, highB, rimGroup, points, triangles, corners);
                AddWallTriangle(lowA, highB, highA, rimGroup, points, triangles, corners);
            }
        }

        var mesh = new MeshNormalUV {
            Positions = points,
            PrecisionPositions = precise,
            Triangles = triangles,
            TrianglesEx = corners
        };
        if (!MeshAnalysis.IsWatertightMesh(precise, triangles) ||
            !MeshAnalysis.AreTrianglesConsistentlyOriented(precise, triangles))
            throw new InvalidOperationException("Thicken produced a non-watertight result; the offset changes topology.");
        if (MeshAnalysis.ComputeSignedMeshVolume(precise, triangles).Sign() <= 0)
            throw new InvalidOperationException("Thicken collapsed or inverted the surface.");

        name ??= GenerateName("Thicken");
        var result = new AnchorMesh(name, mesh, groupNames, metadata,
            deferCoplanarPostProcess: true, isVolume: true);
        RegisterMesh(result);
        return result;
    }

    private readonly record struct DirectedBoundaryEdge(int Start, int End);
    private sealed class SurfaceBoundary
    {
        internal List<DirectedBoundaryEdge> Edges { get; } = new();
        internal List<List<DirectedBoundaryEdge>> Loops { get; } = new();
    }

    private static SurfaceBoundary ValidateSurfaceTopology(MeshNormalUV mesh)
    {
        if (mesh.PrecisionPositions.Count != mesh.Positions.Count)
            throw new ArgumentException("Surface has incomplete exact coordinates.");
        var canonical = DuplicatePointRemover.DuplicateMap(mesh.PrecisionPositions);
        var incidence = new Dictionary<(int, int), List<DirectedBoundaryEdge>>();
        foreach (Tri triangle in mesh.Triangles)
        {
            int a = canonical[triangle.A], b = canonical[triangle.B], c = canonical[triangle.C];
            if (a == b || b == c || c == a)
                throw new ArgumentException("Surface contains an exactly collapsed triangle.");
            AddEdge(a, b); AddEdge(b, c); AddEdge(c, a);
        }

        var result = new SurfaceBoundary();
        foreach (var pair in incidence)
        {
            if (pair.Value.Count > 2)
                throw new ArgumentException("Surface is non-manifold: an edge belongs to more than two triangles.");
            if (pair.Value.Count == 2)
            {
                var first = pair.Value[0]; var second = pair.Value[1];
                if (first.Start != second.End || first.End != second.Start)
                    throw new ArgumentException("Surface triangles are inconsistently oriented.");
            }
            else result.Edges.Add(pair.Value[0]);
        }

        var outgoing = new Dictionary<int, DirectedBoundaryEdge>();
        var incoming = new Dictionary<int, int>();
        foreach (var edge in result.Edges)
        {
            if (!outgoing.TryAdd(edge.Start, edge) || !incoming.TryAdd(edge.End, edge.Start))
                throw new ArgumentException("Surface boundary branches or contains a T-junction.");
        }
        foreach (int vertex in outgoing.Keys)
            if (!incoming.ContainsKey(vertex))
                throw new ArgumentException("Surface boundary is open.");

        var unused = new HashSet<int>(outgoing.Keys);
        while (unused.Count > 0)
        {
            int start = unused.First();
            int current = start;
            var loop = new List<DirectedBoundaryEdge>();
            do
            {
                if (!unused.Remove(current) || !outgoing.TryGetValue(current, out var edge))
                    throw new ArgumentException("Surface boundary does not form closed loops.");
                loop.Add(edge);
                current = edge.End;
            } while (current != start);
            result.Loops.Add(loop);
        }
        return result;

        void AddEdge(int start, int end)
        {
            var key = start < end ? (start, end) : (end, start);
            if (!incidence.TryGetValue(key, out var entries))
                incidence.Add(key, entries = new List<DirectedBoundaryEdge>(2));
            entries.Add(new DirectedBoundaryEdge(start, end));
        }
    }

    private static MeshTriangle<TriangleVertexNormalUV> Reverse(
        MeshTriangle<TriangleVertexNormalUV> source, int group)
    {
        var a = source.V0; var b = source.V1; var c = source.V2;
        a.Normal = -a.Normal; b.Normal = -b.Normal; c.Normal = -c.Normal;
        return new MeshTriangle<TriangleVertexNormalUV> { V0 = a, V1 = c, V2 = b, GroupId = group };
    }

    private static void AddWallTriangle(int a, int b, int c, int group, List<Vec3D> points,
        List<Tri> triangles, List<MeshTriangle<TriangleVertexNormalUV>> corners)
    {
        Vec3D normal = Vec3DOps.Cross(points[b] - points[a], points[c] - points[a]);
        if (normal.LengthSquared() == 0)
            throw new InvalidOperationException("Thicken collapsed a boundary wall.");
        normal = normal.Normalized();
        var vertex = new TriangleVertexNormalUV { Normal = normal };
        triangles.Add(new Tri(a, b, c));
        corners.Add(new MeshTriangle<TriangleVertexNormalUV> {
            V0 = vertex, V1 = vertex, V2 = vertex, GroupId = group
        });
    }
}
