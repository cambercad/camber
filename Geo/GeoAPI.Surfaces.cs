using Geo.Surfaces;
using Curves;
using CSG;
using GeoCore;
using GeoMeta;

namespace Geo;

public partial class GeoAPI
{
    [APIDescription(@"Sew(surfaces: IReadOnlyList[AnchorMesh], makeSolid: bool = false, name: str = None) -> AnchorMesh
Joins exactly coincident rational surface boundaries. Surface orientations are reconciled automatically. makeSolid requires a completely watertight result.")]
    public AnchorMesh Sew(IReadOnlyList<AnchorMesh> surfaces, bool makeSolid = false, string name = null)
    {
        name ??= GenerateName(makeSolid ? "SewnSolid" : "SewnSurface");
        var result = SurfaceSewing.Sew(this, surfaces, makeSolid, name);
        RegisterMesh(result);
        return result;
    }

    [APIDescription(@"ExtrudeSurface(sketch: PlotterSketcherCoordSys, height: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Extrudes a closed sketch profile and returns the uncapped side patches as an open surface.")]
    public AnchorMesh ExtrudeSurface(PlotterSketcherCoordSys sketch, double height,
        double maxDeviation = -1, string name = null)
    {
        name ??= GenerateName("ExtrudeSurface");
        var generated = Extrude(sketch, height, maxDeviation, name);
        return RegisterSideSurface(generated, name,
            EntityNaming.ExtrudeBottom(name), EntityNaming.ExtrudeTop(name));
    }

    [APIDescription(@"RevolveSurface(sketch: PlotterSketcher, angle: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh
Revolves a closed sketch profile and returns the uncapped side patches as an open surface.")]
    public AnchorMesh RevolveSurface(PlotterSketcher sketch, double angle,
        double maxDeviation = -1, string name = null)
    {
        name ??= GenerateName("RevolveSurface");
        var generated = Revolve(sketch, angle, maxDeviation, name);
        return RegisterSideSurface(generated, name,
            EntityNaming.RevolveStartCap(name), EntityNaming.RevolveEndCap(name));
    }

    [APIDescription(@"SweepSurface(profile: PlotterSketcherCoordSys, guide: Curve3D, maxDeviation: float = -1, name: str = None, twistRatePerDistance: float = 0, referenceDirection: Vec3D = None) -> AnchorMesh
Sweeps a closed sketch profile along one 3D guide and returns the uncapped side surface.")]
    public AnchorMesh SweepSurface(PlotterSketcherCoordSys profile, Curve3D guide,
        double maxDeviation = -1, string name = null, double twistRatePerDistance = 0,
        Vec3D? referenceDirection = null)
    {
        name ??= GenerateName("SweepSurface");
        var generated = ExtrudeAlongCurve(profile, guide, maxDeviation, name,
            twistRatePerDistance, referenceDirection);
        return RegisterSideSurface(generated, name,
            EntityNaming.ExtrudeBottom(name), EntityNaming.ExtrudeTop(name));
    }

    [APIDescription(@"SweepSurface(profile: PlotterSketcherCoordSys, guideCurveStrip: List[Curve3D], maxDeviation: float = -1, name: str = None, twistRatePerDistance: float = 0, referenceDirection: Vec3D = None) -> AnchorMesh
Sweeps a closed sketch profile along a connected curve strip and returns the uncapped side surface. Sharp guide joins use a shared bisector section.")]
    public AnchorMesh SweepSurface(PlotterSketcherCoordSys profile, List<Curve3D> guideCurveStrip,
        double maxDeviation = -1, string name = null, double twistRatePerDistance = 0,
        Vec3D? referenceDirection = null)
    {
        name ??= GenerateName("SweepSurface");
        var generated = ExtrudeAlongCurveStrip(profile, guideCurveStrip, maxDeviation,
            twistRatePerDistance, name, referenceDirection);
        return RegisterSideSurface(generated, name,
            EntityNaming.ExtrudeBottom(name), EntityNaming.ExtrudeTop(name));
    }

    [APIDescription(@"TrimSurface(surface: AnchorMesh, cutter: AnchorMesh, keepNormalSide: bool = true, name: str = None) -> AnchorMesh
Trims one open surface by another through the exact CSG intersection pipeline. The cutter must span every intersection needed to separate the retained side.")]
    public AnchorMesh TrimSurface(AnchorMesh surface, AnchorMesh cutter,
        bool keepNormalSide = true, string name = null)
    {
        ArgumentNullException.ThrowIfNull(surface);
        ArgumentNullException.ThrowIfNull(cutter);
        if (surface.IsVolume)
            throw new ArgumentException("The body to trim must be an open surface.", nameof(surface));
        if (cutter.IsVolume)
            throw new ArgumentException("The trimming body must be an open surface.", nameof(cutter));

        var operation = keepNormalSide
            ? BooleanOp.AAsSurfaceBAsTrimSurfaceKeepInTriNormalDirection
            : BooleanOp.AAsSurfaceBAsTrimSurfaceRemoveInTriNormalDirection;
        return Boolean(surface, cutter, operation, name ?? GenerateName("TrimSurface"));
    }

    [APIDescription(@"SplitSurface(surface: AnchorMesh, cutter: AnchorMesh, name: str = None) -> IReadOnlyList[AnchorMesh]
Returns both exact sides produced by trimming an open surface with another open surface.")]
    public IReadOnlyList<AnchorMesh> SplitSurface(AnchorMesh surface, AnchorMesh cutter,
        string name = null)
    {
        string basis = name ?? GenerateName("SplitSurface");
        return new[] {
            TrimSurface(surface, cutter, true, basis + "_Normal"),
            TrimSurface(surface, cutter, false, basis + "_Opposite")
        };
    }

    [APIDescription(@"CapPlanarBoundaries(surface: AnchorMesh, makeSolid: bool = true, name: str = None) -> AnchorMesh
Fills every closed, exactly planar boundary loop with an exact constrained triangulation. Non-planar or branched boundaries are rejected.")]
    public AnchorMesh CapPlanarBoundaries(AnchorMesh surface, bool makeSolid = true, string name = null)
    {
        ArgumentNullException.ThrowIfNull(surface);
        if (surface.IsVolume)
            throw new ArgumentException("Capping requires an open surface.", nameof(surface));
        if (surface.Mesh.PrecisionPositions.Count != surface.Mesh.Positions.Count)
            throw new ArgumentException("Surface has incomplete exact coordinates.", nameof(surface));

        SurfaceBoundary boundary = ValidateSurfaceTopology(surface.Mesh);
        if (boundary.Loops.Count == 0)
            throw new ArgumentException("Surface has no boundary loops to cap.", nameof(surface));

        var canonical = DuplicatePointRemover.DuplicateMap(surface.Mesh.PrecisionPositions);
        var exactByCanonical = new Dictionary<int, Rat3Hybrid>();
        foreach (var pair in canonical)
            if (!exactByCanonical.ContainsKey(pair.Value))
                exactByCanonical.Add(pair.Value, surface.Mesh.PrecisionPositions[pair.Key]);

        var planes = new List<PlanarBoundaryGroup>();
        foreach (var loop in boundary.Loops)
        {
            Rat3Hybrid normal = FindLoopNormal(loop, exactByCanonical);
            Rat3Hybrid origin = exactByCanonical[loop[0].Start];
            foreach (var edge in loop)
                if (Rat3Hybrid.Dot(exactByCanonical[edge.Start] - origin, normal).Sign() != 0)
                    throw new NotSupportedException("A boundary loop is not exactly planar.");

            PlanarBoundaryGroup group = null;
            foreach (var candidate in planes)
            {
                if (!Rat3Hybrid.Cross(candidate.Normal, normal).IsZero()) continue;
                if (Rat3Hybrid.Dot(origin - candidate.Origin, candidate.Normal).Sign() != 0) continue;
                group = candidate;
                break;
            }
            if (group == null)
            {
                group = new PlanarBoundaryGroup(origin, normal);
                planes.Add(group);
            }
            group.Loops.Add(loop);
        }

        int firstCapGroup = ReserveGroupIds(planes.Count);
        var names = new Dictionary<int, string>(surface.groupIdToExtendedName);
        var metadata = surface.surfaceMetaData == null
            ? new Dictionary<string, SurfaceMetaData>()
            : SurfaceMetaData.CloneDictionary(surface.surfaceMetaData);
        var triangles = new List<Tri>(surface.Mesh.Triangles);
        var corners = new List<MeshTriangle<TriangleVertexNormalUV>>(surface.Mesh.TrianglesEx);
        var capBoundary = new Dictionary<(int, int), DirectedBoundaryEdge>();
        foreach (var edge in boundary.Edges)
            capBoundary.Add(Normalize(edge.Start, edge.End), edge);

        for (int planeIndex = 0; planeIndex < planes.Count; planeIndex++)
        {
            PlanarBoundaryGroup plane = planes[planeIndex];
            var contours = new List<List<Rat2Hybrid>>(plane.Loops.Count);
            var vertexIndices = new List<int>();
            foreach (var loop in plane.Loops)
            {
                var contour = new List<Rat2Hybrid>(loop.Count);
                foreach (var edge in loop)
                {
                    contour.Add(Project(exactByCanonical[edge.Start], plane.DropAxis));
                    vertexIndices.Add(edge.Start);
                }
                contours.Add(contour);
            }

            List<Tri> capTriangles = Triangulator.TriangulatePolygon(contours);
            if (capTriangles.Count == 0)
                throw new InvalidOperationException("Planar boundary triangulation produced no triangles.");
            var expectedBoundary = new HashSet<(int, int)>();
            foreach (var loop in plane.Loops)
                foreach (var edge in loop)
                    expectedBoundary.Add(Normalize(edge.Start, edge.End));
            bool flip = CapMustFlip(capTriangles, vertexIndices, capBoundary, expectedBoundary);
            int capGroup = firstCapGroup + planeIndex;
            string capName = EntityNaming.SurfaceCap(surface.Name, planeIndex + 1);
            names.Add(capGroup, capName);
            Vec3D normalWorld = new(plane.Normal.X.ToDouble(), plane.Normal.Y.ToDouble(), plane.Normal.Z.ToDouble());
            normalWorld = normalWorld.Normalized();
            Vec3D originWorld = converter.Convert(plane.Origin);
            Vec3D refDir = Math.Abs(normalWorld.Z) < .9
                ? Vec3DOps.Cross(new Vec3D(0, 0, 1), normalWorld).Normalized()
                : Vec3DOps.Cross(new Vec3D(0, 1, 0), normalWorld).Normalized();
            metadata[capName] = new SurfaceMetaData(SurfaceType.Planar) {
                PlaneParams = new PlaneSurfaceParams { Origin = originWorld, Normal = normalWorld, RefDir = refDir }
            };

            foreach (Tri cap in capTriangles)
            {
                int a = vertexIndices[cap.A], b = vertexIndices[cap.B], c = vertexIndices[cap.C];
                if (flip) (b, c) = (c, b);
                Tri triangle = new(a, b, c);
                Vec3D triangleNormal = Vec3DOps.Cross(
                    surface.Mesh.Positions[b] - surface.Mesh.Positions[a],
                    surface.Mesh.Positions[c] - surface.Mesh.Positions[a]).Normalized();
                triangles.Add(triangle);
                corners.Add(new MeshTriangle<TriangleVertexNormalUV> {
                    V0 = CapVertex(triangleNormal, ProjectUv(exactByCanonical[a], plane.DropAxis)),
                    V1 = CapVertex(triangleNormal, ProjectUv(exactByCanonical[b], plane.DropAxis)),
                    V2 = CapVertex(triangleNormal, ProjectUv(exactByCanonical[c], plane.DropAxis)),
                    GroupId = capGroup
                });
            }
        }

        var resultMesh = new MeshNormalUV {
            Positions = new List<Vec3D>(surface.Mesh.Positions),
            PrecisionPositions = new List<Rat3Hybrid>(surface.Mesh.PrecisionPositions),
            Triangles = triangles,
            TrianglesEx = corners
        };
        bool isWatertight = MeshAnalysis.IsWatertightMesh(resultMesh.PrecisionPositions, resultMesh.Triangles);
        if (makeSolid && !isWatertight)
            throw new InvalidOperationException("Capping did not close every exact boundary edge.");
        if (makeSolid && !MeshAnalysis.AreTrianglesConsistentlyOriented(resultMesh.PrecisionPositions, resultMesh.Triangles))
            throw new InvalidOperationException("Capping produced inconsistent face orientations.");
        if (makeSolid)
        {
            SurfaceSewing.OrientClosedComponentsOutward(resultMesh.PrecisionPositions,
                resultMesh.Triangles, resultMesh.TrianglesEx);
            if (MeshAnalysis.ComputeSignedMeshVolume(resultMesh.PrecisionPositions, resultMesh.Triangles).Sign() == 0)
                throw new InvalidOperationException("Capped surface encloses zero volume.");
        }

        name ??= GenerateName("CapSurface");
        var result = new AnchorMesh(name, resultMesh, names, metadata,
            deferCoplanarPostProcess: true, isVolume: makeSolid);
        RegisterMesh(result);
        return result;

        static (int, int) Normalize(int a, int b) => a < b ? (a, b) : (b, a);
    }

    private sealed class PlanarBoundaryGroup(Rat3Hybrid origin, Rat3Hybrid normal)
    {
        internal Rat3Hybrid Origin { get; } = origin;
        internal Rat3Hybrid Normal { get; } = normal;
        internal List<List<DirectedBoundaryEdge>> Loops { get; } = new();
        internal int DropAxis
        {
            get
            {
                double x = Math.Abs(Normal.X.ToDouble());
                double y = Math.Abs(Normal.Y.ToDouble());
                double z = Math.Abs(Normal.Z.ToDouble());
                return x >= y && x >= z ? 0 : y >= z ? 1 : 2;
            }
        }
    }

    private static Rat3Hybrid FindLoopNormal(List<DirectedBoundaryEdge> loop,
        IReadOnlyDictionary<int, Rat3Hybrid> exactPoints)
    {
        Rat3Hybrid origin = exactPoints[loop[0].Start];
        for (int i = 1; i < loop.Count - 1; i++)
        {
            Rat3Hybrid first = exactPoints[loop[i].Start] - origin;
            Rat3Hybrid second = exactPoints[loop[i + 1].Start] - origin;
            Rat3Hybrid normal = Rat3Hybrid.Cross(first, second);
            if (!normal.IsZero()) return normal;
        }
        throw new ArgumentException("A boundary loop is degenerate.");
    }

    private static Rat2Hybrid Project(in Rat3Hybrid point, int dropAxis) => dropAxis switch
    {
        0 => new Rat2Hybrid(point.Y, point.Z),
        1 => new Rat2Hybrid(point.X, point.Z),
        _ => new Rat2Hybrid(point.X, point.Y)
    };

    private static Vec2D ProjectUv(in Rat3Hybrid point, int dropAxis)
    {
        Rat2Hybrid projected = Project(point, dropAxis);
        return new Vec2D(projected.X.ToDouble(), projected.Y.ToDouble());
    }

    private static bool CapMustFlip(List<Tri> caps, List<int> vertexIndices,
        IReadOnlyDictionary<(int, int), DirectedBoundaryEdge> boundary,
        IReadOnlySet<(int, int)> expectedBoundary)
    {
        var capEdges = new Dictionary<(int, int), List<DirectedBoundaryEdge>>();
        foreach (Tri triangle in caps)
        {
            Add(vertexIndices[triangle.A], vertexIndices[triangle.B]);
            Add(vertexIndices[triangle.B], vertexIndices[triangle.C]);
            Add(vertexIndices[triangle.C], vertexIndices[triangle.A]);
        }

        bool? flip = null;
        int matchedBoundaryEdges = 0;
        foreach (var pair in capEdges)
        {
            if (expectedBoundary.Contains(pair.Key))
            {
                if (!boundary.TryGetValue(pair.Key, out DirectedBoundaryEdge source))
                    throw new InvalidOperationException("Cap triangulation introduced a non-boundary seam.");
                if (pair.Value.Count != 1)
                    throw new InvalidOperationException("Cap triangulation did not preserve a boundary edge.");
                DirectedBoundaryEdge cap = pair.Value[0];
                bool edgeMustFlip = cap.Start == source.Start && cap.End == source.End;
                if (flip.HasValue && flip.Value != edgeMustFlip)
                    throw new InvalidOperationException("Cap loops have inconsistent winding relative to their source boundary.");
                flip = edgeMustFlip;
                matchedBoundaryEdges++;
            }
            if (pair.Value.Count != 2 || pair.Value[0].Start != pair.Value[1].End ||
                pair.Value[0].End != pair.Value[1].Start)
            {
                if (expectedBoundary.Contains(pair.Key)) continue;
                throw new InvalidOperationException("Cap triangulation produced an open or inconsistent interior edge.");
            }
        }
        if (matchedBoundaryEdges != expectedBoundary.Count || !flip.HasValue)
            throw new InvalidOperationException("Cap triangulation did not preserve every source boundary edge.");
        return flip.Value;

        void Add(int start, int end)
        {
            var key = start < end ? (start, end) : (end, start);
            if (!capEdges.TryGetValue(key, out var uses)) capEdges.Add(key, uses = new());
            uses.Add(new DirectedBoundaryEdge(start, end));
        }
    }

    private static TriangleVertexNormalUV CapVertex(Vec3D normal, Vec2D uv) =>
        new() { Normal = normal, UV = uv };

    private AnchorMesh RegisterSideSurface(AnchorMesh generated, string name, params string[] caps)
    {
        var excluded = new HashSet<int>();
        foreach (string cap in caps)
            if (generated.extendedNameToGroupId.TryGetValue(cap, out int group)) excluded.Add(group);
        var triangles = new List<Tri>();
        var corners = new List<MeshTriangle<TriangleVertexNormalUV>>();
        var usedGroups = new HashSet<int>();
        for (int i = 0; i < generated.Mesh.Triangles.Count; i++)
        {
            var corner = generated.Mesh.TrianglesEx[i];
            if (excluded.Contains(corner.GroupId)) continue;
            triangles.Add(generated.Mesh.Triangles[i]);
            corners.Add(corner);
            usedGroups.Add(corner.GroupId);
        }
        if (triangles.Count == 0)
            throw new InvalidOperationException("Surface construction produced no side patches.");
        var names = new Dictionary<int, string>();
        var metadata = new Dictionary<string, SurfaceMetaData>();
        foreach (int group in usedGroups)
        {
            string patch = generated.groupIdToExtendedName[group];
            names.Add(group, patch);
            if (generated.surfaceMetaData.TryGetValue(patch, out var data) && data != null)
                metadata.Add(patch, data.Clone());
            else metadata.Add(patch, new SurfaceMetaData(SurfaceType.Unknown));
        }
        var mesh = new MeshNormalUV {
            Positions = new List<Vec3D>(generated.Mesh.Positions),
            PrecisionPositions = new List<Rat3Hybrid>(generated.Mesh.PrecisionPositions),
            Triangles = triangles, TrianglesEx = corners
        };
        var result = new AnchorMesh(name, mesh, names, metadata,
            deferCoplanarPostProcess: true, isVolume: false);
        RegisterMesh(result); // Replaces the same-named temporary generator result.
        return result;
    }
}
