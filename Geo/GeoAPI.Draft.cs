using Curves;
using GeoCore;
using GeoMeta;

namespace Geo;

public partial class GeoAPI
{
    [APIDescription(@"DraftPrismaticFaces(solid: AnchorMesh, sideFaceNames: List[str], neutralPlane: Plane3D, pullDirection: Vec3D, angle: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Drafts selected planar side faces of a straight, convex polygonal prism. The neutral plane must coincide with one cap and its normal must align with the pull direction. Positive angles move the far ends of selected faces inward; the neutral cap stays fixed. Other geometry and collapsed profiles are rejected.")]
    public AnchorMesh DraftPrismaticFaces(AnchorMesh solid, List<string> sideFaceNames,
        Plane3D neutralPlane, Vec3D pullDirection, double angle, double maxDeviation = -1, string name = null)
    {
        ArgumentNullException.ThrowIfNull(solid);
        ArgumentNullException.ThrowIfNull(sideFaceNames);
        ArgumentNullException.ThrowIfNull(neutralPlane);
        if (!solid.IsVolume) throw new ArgumentException("Draft requires a closed solid.", nameof(solid));
        if (sideFaceNames.Count == 0) throw new ArgumentException("Select at least one side face.", nameof(sideFaceNames));
        if (!double.IsFinite(angle) || Math.Abs(angle) >= Math.PI / 2)
            throw new ArgumentOutOfRangeException(nameof(angle));
        if (!double.IsFinite(pullDirection.X) || !double.IsFinite(pullDirection.Y) || !double.IsFinite(pullDirection.Z)
            || pullDirection.LengthSquared() < 1e-24)
            throw new ArgumentException("Pull direction must be finite and nonzero.", nameof(pullDirection));
        var axis = pullDirection.Normalized();
        var planeNormal = neutralPlane.Normal.Normalized();
        if (Math.Abs(Vec3DOps.Dot(axis, planeNormal)) < 1 - 1e-8)
            throw new NotSupportedException("The neutral plane must be perpendicular to the pull direction.");

        solid.EnsureCoplanarPostProcessed();
        var selected = new HashSet<int>();
        foreach (var requested in sideFaceNames)
        {
            var local = solid.ResolveLocalPatchNamePublic(requested);
            if (local == null || !solid.extendedNameToGroupId.TryGetValue(local, out var group))
                throw new ArgumentException($"Unknown face '{requested}'.", nameof(sideFaceNames));
            selected.Add(group);
        }

        var positions = solid.Mesh.Positions;
        var triangles = solid.Mesh.Triangles;
        var groups = solid.Mesh.GetTriangleGroups();
        if (positions.Count < 6 || triangles.Count == 0)
            throw new NotSupportedException("Draft requires a polygonal prism.");
        var tolerance = Math.Max(converter.SmallestUnit() * 4, 1e-8);
        var heights = positions.Select(p => Vec3DOps.Dot(p - neutralPlane.Origin, axis)).ToArray();
        var far = heights.Max();
        if (heights.Min() < -tolerance || far <= tolerance)
            throw new NotSupportedException("The neutral plane must coincide with the start cap and pull toward the other cap.");
        if (heights.Any(z => Math.Abs(z) > tolerance && Math.Abs(z - far) > tolerance))
            throw new NotSupportedException("Draft currently supports straight two-cap prisms only.");
        bool IsNear(int v) => Math.Abs(heights[v]) <= tolerance;
        bool IsFar(int v) => Math.Abs(heights[v] - far) <= tolerance;

        var capEdges = new Dictionary<(int, int), int>();
        var sideEdges = new Dictionary<(int, int), int>();
        var capGroups = new HashSet<int>();
        var nearCapGroups = new HashSet<int>();
        var farCapGroups = new HashSet<int>();
        var sideGroups = new HashSet<int>();
        static (int, int) Edge(int a, int b) => a < b ? (a, b) : (b, a);
        for (int i = 0; i < triangles.Count; i++)
        {
            var t = triangles[i];
            var ids = new[] { t.A, t.B, t.C };
            int nearCount = ids.Count(IsNear);
            if (nearCount == 3)
            {
                capGroups.Add(groups[i]);
                nearCapGroups.Add(groups[i]);
                for (int j = 0; j < 3; j++)
                {
                    var edge = Edge(ids[j], ids[(j + 1) % 3]);
                    capEdges[edge] = capEdges.GetValueOrDefault(edge) + 1;
                }
            }
            else if (nearCount == 0 && ids.All(IsFar))
            {
                capGroups.Add(groups[i]);
                farCapGroups.Add(groups[i]);
            }
            else
            {
                if (ids.Any(v => !IsNear(v) && !IsFar(v)))
                    throw new NotSupportedException("Draft requires two planar caps.");
                sideGroups.Add(groups[i]);
                if (nearCount == 2)
                {
                    var edge = Edge(ids.First(IsNear), ids.Last(IsNear));
                    if (sideEdges.TryGetValue(edge, out int previous) && previous != groups[i])
                        throw new NotSupportedException("A neutral edge belongs to multiple side faces.");
                    sideEdges[edge] = groups[i];
                }
            }
        }
        if (capGroups.Overlaps(selected) || selected.Any(g => !sideGroups.Contains(g)))
            throw new ArgumentException("Only planar prism side faces can be drafted.", nameof(sideFaceNames));
        if (nearCapGroups.Count != 1 || farCapGroups.Count != 1 ||
            nearCapGroups.Overlaps(farCapGroups) || capGroups.Overlaps(sideGroups))
            throw new NotSupportedException("Draft requires one named patch on each cap and separate side patches.");
        var boundary = capEdges.Where(kv => kv.Value == 1).Select(kv => kv.Key).ToArray();
        if (boundary.Length < 3 || capEdges.Any(kv => kv.Value > 2))
            throw new NotSupportedException("The neutral cap must contain one simple polygon.");
        var adjacency = new Dictionary<int, List<int>>();
        foreach (var (a, b) in boundary)
        {
            if (!adjacency.TryGetValue(a, out var aroundA)) adjacency[a] = aroundA = new();
            if (!adjacency.TryGetValue(b, out var aroundB)) adjacency[b] = aroundB = new();
            aroundA.Add(b); aroundB.Add(a);
        }
        if (adjacency.Any(kv => kv.Value.Count != 2))
            throw new NotSupportedException("The neutral cap boundary is not a simple loop.");
        var vertexLoop = new List<int> { boundary[0].Item1 };
        int previousVertex = -1, current = vertexLoop[0];
        do
        {
            var neighbours = adjacency[current];
            int next = neighbours[0] == previousVertex ? neighbours[1] : neighbours[0];
            previousVertex = current;
            current = next;
            if (current != vertexLoop[0]) vertexLoop.Add(current);
        } while (current != vertexLoop[0] && vertexLoop.Count <= boundary.Length);
        if (vertexLoop.Count != boundary.Length)
            throw new NotSupportedException("The neutral cap has multiple contours.");
        var frame = new CoordinateSystem(neutralPlane.Origin, neutralPlane.X, neutralPlane.Y, axis);
        var outline = vertexLoop.Select(v => frame.PointFromWorldToCoordSys(positions[v]))
            .Select(v => new Vec2D(v.X, v.Y)).ToArray();
        var patches = new int[outline.Length];
        for (int i = 0; i < patches.Length; i++)
        {
            if (!sideEdges.TryGetValue(Edge(vertexLoop[i], vertexLoop[(i + 1) % patches.Length]), out patches[i]))
                throw new NotSupportedException("Could not match each neutral edge to one side face.");
        }
        if (patches.Distinct().Count() != patches.Length || !sideGroups.SetEquals(patches))
            throw new NotSupportedException("Each prism side must be one planar face corresponding to one neutral edge.");
        for (int i = 0; i < patches.Length; i++)
        {
            var a = outline[i]; var b = outline[(i + 1) % patches.Length];
            var dx = b.X - a.X; var dy = b.Y - a.Y;
            var length = Math.Sqrt(dx * dx + dy * dy);
            foreach (var vertex in triangles.Select((t, ti) => (t, ti))
                .Where(pair => groups[pair.ti] == patches[i])
                .SelectMany(pair => new[] { pair.t.A, pair.t.B, pair.t.C }).Distinct())
            {
                var local = frame.PointFromWorldToCoordSys(positions[vertex]);
                if (Math.Abs(dx * (local.Y - a.Y) - dy * (local.X - a.X)) > tolerance * length)
                    throw new NotSupportedException("Side faces must be planar and parallel to the pull direction.");
            }
        }
        // Every far boundary vertex must project to a neutral boundary vertex.
        foreach (int v in Enumerable.Range(0, positions.Count).Where(IsFar))
        {
            var local = frame.PointFromWorldToCoordSys(positions[v]);
            // Interior cap tessellation vertices are allowed; only side vertices must match.
            if (!triangles.Where((t, i) => sideGroups.Contains(groups[i]))
                .Any(t => t.A == v || t.B == v || t.C == v)) continue;
            if (!outline.Any(p => Math.Abs(p.X - local.X) <= tolerance && Math.Abs(p.Y - local.Y) <= tolerance))
                throw new NotSupportedException("Side faces are not parallel to the pull direction.");
        }

        double area2 = 0;
        for (int i = 0; i < outline.Length; i++)
        {
            var a = outline[i]; var b = outline[(i + 1) % outline.Length];
            area2 += a.X * b.Y - b.X * a.Y;
        }
        if (Math.Abs(area2) <= tolerance * tolerance)
            throw new NotSupportedException("The neutral profile has zero area.");
        double winding = Math.Sign(area2);
        var inward = new Vec2D[outline.Length];
        for (int i = 0; i < outline.Length; i++)
        {
            var a = outline[i]; var b = outline[(i + 1) % outline.Length];
            var dx = b.X - a.X; var dy = b.Y - a.Y;
            var length = Math.Sqrt(dx * dx + dy * dy);
            if (length <= tolerance) throw new NotSupportedException("The neutral profile has a collapsed edge.");
            inward[i] = new Vec2D(-winding * dy / length, winding * dx / length);
            var c = outline[(i + 2) % outline.Length];
            if (winding * (dx * (c.Y - b.Y) - dy * (c.X - b.X)) <= tolerance * tolerance)
                throw new NotSupportedException("Draft currently requires a strictly convex polygon.");
        }
        var offsets = patches.Select(g => selected.Contains(g) ? far * Math.Tan(angle) : 0).ToArray();
        var farOutline = new Vec2D[outline.Length];
        for (int i = 0; i < outline.Length; i++)
        {
            int before = (i + outline.Length - 1) % outline.Length;
            var first = inward[before]; var second = inward[i];
            double det = first.X * second.Y - first.Y * second.X;
            if (Math.Abs(det) <= 1e-10)
                throw new NotSupportedException("Adjacent side planes are parallel.");
            double firstLine = first.X * outline[i].X + first.Y * outline[i].Y + offsets[before];
            double secondLine = second.X * outline[i].X + second.Y * outline[i].Y + offsets[i];
            farOutline[i] = new Vec2D(
                (firstLine * second.Y - first.Y * secondLine) / det,
                (first.X * secondLine - firstLine * second.X) / det);
        }
        for (int i = 0; i < farOutline.Length; i++)
        {
            var a = farOutline[i]; var b = farOutline[(i + 1) % farOutline.Length];
            var c = farOutline[(i + 2) % farOutline.Length];
            if (winding * ((b.X - a.X) * (c.Y - b.Y) - (b.Y - a.Y) * (c.X - b.X)) <= tolerance * tolerance)
                throw new InvalidOperationException("Draft angle collapses or reverses the far profile.");
        }

        name ??= GenerateName("Draft");
        var nearSketch = new PlotterSketcherCoordSys(name + "_neutral", frame);
        var farFrame = frame;
        farFrame.Origin += axis * far;
        var farSketch = new PlotterSketcherCoordSys(name + "_far", farFrame);
        nearSketch.SetCurves(new List<List<Curve2D>> { MakeLines(outline, name + "_neutral") });
        farSketch.SetCurves(new List<List<Curve2D>> { MakeLines(farOutline, name + "_far") });
        var result = Loft(new[] { nearSketch, farSketch }, new LoftOptions
        {
            Style = LoftStyle.Ruled,
            CapEnds = true,
            CorrespondenceMode = LoftCorrespondenceMode.MatchingVertices,
            FirstCurves = new[] { name + "_neutral_1", name + "_far_1" }
        }, name, maxDeviation);
        if (!result.IsVolume)
            throw new InvalidOperationException("Draft did not produce a closed volume.");
        var sourceNames = new Dictionary<string, int>
        {
            [$"{name}-StartCap"] = nearCapGroups.Single(),
            [$"{name}-EndCap"] = farCapGroups.Single()
        };
        for (int i = 0; i < patches.Length; i++)
            sourceNames[$"{name}-Side-{name}_neutral_{i + 1}"] = patches[i];
        if (result.extendedNameToGroupId.Count != sourceNames.Count ||
            sourceNames.Keys.Any(key => !result.extendedNameToGroupId.ContainsKey(key)))
            throw new InvalidOperationException("Draft could not preserve the source face correspondence.");
        var groupNames = new Dictionary<int, string>();
        var metadata = new Dictionary<string, SurfaceMetaData>();
        var lineages = new Dictionary<int, FaceLineage>();
        foreach (var (generatedName, sourceGroup) in sourceNames)
        {
            int group = result.extendedNameToGroupId[generatedName];
            var sourceName = solid.groupIdToExtendedName[sourceGroup];
            groupNames[group] = sourceName;
            if (result.surfaceMetaData.TryGetValue(generatedName, out var data))
                metadata[sourceName] = data.Clone();
            if (solid.FaceLineages.TryGetValue(sourceGroup, out var lineage))
                lineages[group] = lineage;
        }
        var named = new AnchorMesh(name, result.Mesh, groupNames, metadata,
            deferCoplanarPostProcess: false, faceLineages: lineages);
        RegisterMesh(named);
        return named;
    }

    private static List<Curve2D> MakeLines(IReadOnlyList<Vec2D> points, string prefix)
    {
        var result = new List<Curve2D>(points.Count);
        for (int i = 0; i < points.Count; i++)
            result.Add(new Line2D(points[i], points[(i + 1) % points.Count])
            { Name = $"{prefix}_{i + 1}" });
        return result;
    }
}
