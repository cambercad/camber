using GeoCore;
using GeoMeta;
using GeoSolver;
using GeoSolver.Kinematics;

namespace Geo
{
    [APIDescription(@"Assembly: 3D mate solver for registered meshes. AddPart places solids; AddSubAssembly places child definitions as rigid occurrences. A definition may be placed repeatedly; use each occurrence's GetParts() references for unambiguous parent mates. Create datums on parts; apply mates; then SolveConstraints().")]
    public partial class Assembly
    {
        private readonly GeoAPI _api;
        internal CoordinateConverter Converter => _api.Converter;
        private readonly KinematicSolver _solver = new KinematicSolver();
        private readonly List<AssemblyPart> _parts = new List<AssemblyPart>();
        private readonly List<AssemblyOccurrence> _occurrences = new List<AssemblyOccurrence>();
        private readonly List<AssemblyMateRecord> _mateRecords = new List<AssemblyMateRecord>();
        private readonly Dictionary<AnchorMesh, (GeoAPI Source, AnchorMesh Definition)> _foreignDefinitions = new();
        private bool _solveAfterEveryConstraint = true;
        internal bool SuppressOccurrenceMeshUpdates { get; private set; }

        internal Assembly(GeoAPI api, string name)
        {
            _api = api ?? throw new ArgumentNullException(nameof(api));
            Name = name ?? throw new ArgumentNullException(nameof(name));
        }

        public string Name { get; }

        internal Assembly Parent { get; private set; }

        [APIDescription(@"GetParts() -> IReadOnlyList[AssemblyPart]
All parts added via AddPart on this assembly (live list, not nested sub-assembly parts).")]
        public IReadOnlyList<AssemblyPart> GetParts() => _parts;

        [APIDescription(@"GetSubAssemblies() -> IReadOnlyList[AssemblyOccurrence]
Child assemblies added via AddSubAssembly on this assembly (live list).")]
        public IReadOnlyList<AssemblyOccurrence> GetSubAssemblies() => _occurrences;

        [APIDescription(@"GetMateRecords() -> IReadOnlyList[AssemblyMateRecord]
Mate metadata recorded by Fix*/Set* calls (for viewers and diagnostics).")]
        public IReadOnlyList<AssemblyMateRecord> GetMateRecords() => _mateRecords;

        [APIDescription(@"SolveAfterEveryConstraint (bool, default True)
When True, every Fix*/Set* call triggers SolveConstraints(). Set False to batch constraints.")]
        public bool SolveAfterEveryConstraint
        {
            get => _solveAfterEveryConstraint;
            set => _solveAfterEveryConstraint = value;
        }

        [APIDescription(@"AddPart(mesh: AnchorMesh, position: Vec3D, orientation: Quaternion = identity) -> AssemblyPart
Registers a rigid body at the given initial pose. Mesh must belong to this assembly's GeoAPI instance.")]
        public AssemblyPart AddPart(AnchorMesh mesh, Vec3D position, Quaternion orientation = default)
            => AddPart(_api, mesh, position, orientation);

        [APIDescription(@"AddPart(source: GeoAPI, mesh: AnchorMesh, position: Vec3D, orientation: Quaternion = identity) -> AssemblyPart
Registers a solid owned by another GeoAPI instance. Its exact geometry is re-expressed in this assembly's lattice without coordinate rounding; repeated placements share the converted definition.")]
        public AssemblyPart AddPart(GeoAPI source, AnchorMesh mesh, Vec3D position, Quaternion orientation = default)
        {
            if (source == null)
                throw new ArgumentNullException(nameof(source));
            if (mesh == null)
                throw new ArgumentNullException(nameof(mesh));

            if (!source.IsRegisteredMesh(mesh))
                throw new ArgumentException($"Mesh '{mesh.Name}' is not registered on the source GeoAPI instance.", nameof(mesh));

            AnchorMesh definition = mesh;
            if (!ReferenceEquals(source, _api))
                definition = GetForeignDefinition(source, mesh);

            orientation = TransformMath.NormalizeDefault(orientation);
            var initialPose = new Transform(position, orientation);
            definition.CaptureRigidRestPose(_api.Converter);

            _solver.IncludeCharacteristicLength(MeshCharacteristicLength(definition));
            RigidTransform<AnchorMesh> rigidBody = _solver.AddRigidBody(definition, initialPose);
            var part = new AssemblyPart(this, definition, rigidBody);
            _parts.Add(part);
            TouchActivity();
            return part;
        }

        private AnchorMesh GetForeignDefinition(GeoAPI source, AnchorMesh mesh)
        {
            if (_foreignDefinitions.TryGetValue(mesh, out var cached))
            {
                if (!ReferenceEquals(cached.Source, source))
                    throw new ArgumentException("The same mesh was registered by multiple source GeoAPI instances; specify a distinct mesh for each source lattice.", nameof(mesh));
                return cached.Definition;
            }

            mesh.EnsureCoplanarPostProcessed();
            var sourceConverter = source.Converter;
            var targetConverter = _api.Converter;
            var exactPositions = new List<Rat3Hybrid>(mesh.Mesh.PrecisionPositions.Count);
            for (int i = 0; i < mesh.Mesh.PrecisionPositions.Count; i++)
                exactPositions.Add(targetConverter.ConvertExact(mesh.Mesh.PrecisionPositions[i], sourceConverter));

            var definitionMesh = new MeshNormalUV
            {
                PrecisionPositions = exactPositions,
                Positions = targetConverter.Convert(exactPositions),
                Triangles = new List<Tri>(mesh.Mesh.Triangles),
                TrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>(mesh.Mesh.TrianglesEx),
            };
            var definition = new AnchorMesh(mesh.Name, definitionMesh,
                new Dictionary<int, string>(mesh.groupIdToExtendedName),
                SurfaceMetaData.CloneDictionary(mesh.surfaceMetaData),
                deferCoplanarPostProcess: false,
                skipCoplanarFusion: true,
                isVolume: mesh.IsVolume,
                preserveTriangulation: true,
                faceLineages: mesh.FaceLineages,
                ambiguousReferences: mesh.AmbiguousFaceReferences);
            _foreignDefinitions.Add(mesh, (source, definition));
            return definition;
        }

        [APIDescription(@"AddSubAssembly(child: Assembly, position: Vec3D, orientation: Quaternion = identity, flexible: bool = false) -> AssemblyOccurrence
Places a child definition. Rigid occurrences share the definition's internal pose. A flexible occurrence gets an independent copy of its internal mate state and part geometry, so its joints can move without affecting other occurrences. Use the returned occurrence's GetParts() references for unambiguous parent mates.")]
        public AssemblyOccurrence AddSubAssembly(Assembly child, Vec3D position, Quaternion orientation = default, bool flexible = false)
            => AddSubAssembly(child, position, orientation, flexible, cloneFlexibleDefinition: flexible);

        internal AssemblyOccurrence AddSubAssembly(Assembly child, Vec3D position, Quaternion orientation,
            bool flexible, bool cloneFlexibleDefinition)
        {
            if (child == null)
                throw new ArgumentNullException(nameof(child));
            if (child == this)
                throw new ArgumentException("An assembly cannot contain itself.", nameof(child));
            if (!flexible && child.Parent != null && child.Parent != this)
                throw new ArgumentException(
                    $"Assembly '{child.Name}' is already nested in '{child.Parent.Name}'.",
                    nameof(child));

            for (Assembly ancestor = this; ancestor != null; ancestor = ancestor.Parent)
            {
                if (ancestor == child)
                    throw new ArgumentException(
                        $"Nesting '{child.Name}' in '{Name}' would create a cycle.",
                        nameof(child));
            }

            if (child.ContainsAssembly(this))
                throw new ArgumentException(
                    $"Nesting '{child.Name}' in '{Name}' would create a cycle.",
                    nameof(child));

            orientation = TransformMath.NormalizeDefault(orientation);
            Assembly definition = child;
            if (flexible && cloneFlexibleDefinition)
            {
                string instanceName;
                do { instanceName = GeoAPI.GenerateName(child.Name + "_flexible"); }
                while (_api.GetAssemblies().Any(existing => existing.Name == instanceName));
                definition = child.CloneHierarchy(instanceName, independentGeometry: true);
            }
            if (_occurrences.Any(existing => ReferenceEquals(existing.Child, definition)))
                definition.SuppressOccurrenceMeshUpdates = true;
            var initialPose = new Transform(position, orientation);
            IncludeSubtreeCharacteristicLength(definition);
            _solver.IncludeCharacteristicLength(position.Length());

            var rig = new AssemblyOccurrenceRig(definition);
            RigidTransform<AssemblyOccurrenceRig> rigidBody = _solver.AddRigidBody(rig, initialPose);
            var occurrence = new AssemblyOccurrence(this, definition, rigidBody, flexible: flexible)
            { DisplayName = child.Name };
            _occurrences.Add(occurrence);
            // Parent is the definition-tree parent used for internal local
            // transforms. Repeated placements in this same assembly share the
            // definition and are distinguished by AssemblyPart occurrence refs.
            definition.Parent ??= this;
            TouchActivity();
            return occurrence;
        }

        [APIDescription(@"FixPart(part: AssemblyPart) -> None
Locks the part at its current pose (6 DOF). A nested part locks the rigid sub-assembly that contains it.")]
        public void FixPart(AssemblyPart part)
        {
            EnsurePartInTree(part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.FixPart,
                $"Fix {part.Mesh.Name}",
                part).WithFixedTarget(SolverTransformOf(part).Evaluate(),
                    FindOccurrenceContaining(part)),
                part.Mesh.Name + ":");
            AddSolverConstraints(new FixedTransformConstraint3d(SolverTransformOf(part)));
        }

        [APIDescription(@"FixPart(part: AssemblyPart, position: Vec3D, orientation: Quaternion) -> None
Locks the part at the given world pose.")]
        public void FixPart(AssemblyPart part, Vec3D position, Quaternion orientation)
        {
            EnsurePartInTree(part);
            orientation = TransformMath.NormalizeDefault(orientation);
            Transform desired = new Transform(position, orientation);
            AssemblyOccurrence occurrence = FindOccurrenceContaining(part);
            if (occurrence != null)
            {
                Transform relative = RelativePoseInOccurrence(part, occurrence);
                desired = TransformMath.Compose(desired, TransformMath.Inverse(relative));
            }
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.FixPart,
                $"Fix {part.Mesh.Name}",
                part).WithFixedTarget(desired,
                    occurrence),
                part.Mesh.Name + ":");
            AddSolverConstraints(new FixedTransformConstraint3d(SolverTransformOf(part), desired));
        }

        [APIDescription(@"FixSubAssembly(occurrence: AssemblyOccurrence) -> None
Locks a nested sub-assembly at its current pose (6 DOF).")]
        public void FixSubAssembly(AssemblyOccurrence occurrence)
        {
            AssemblyOccurrence root = occurrence?.InstanceRoot;
            EnsureOwnedOccurrence(root);
            AssemblyPart leaf = FirstLeafPart(occurrence.Child);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.FixPart,
                $"Fix {occurrence.Child.Name}",
                leaf.ForOccurrence(root)).WithFixedTarget(root.EvaluatePose(), root),
                occurrence.Child.Name + ":");
            AddSolverConstraints(new FixedTransformConstraint3d(root.Transform));
        }

        [APIDescription(@"SetCoincident(a: AssemblyPointDatum, b: AssemblyPointDatum) -> None
Makes two point datums coincident in world space.")]
        public void SetCoincident(AssemblyPointDatum a, AssemblyPointDatum b)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.CoincidentPoints,
                MateLabel("Coincident", a.Part, b.Part),
                a.Part, b.Part, a.LocalPoint, b.LocalPoint),
                a.Entity,
                b.Entity);
            AddSolverConstraints(new PointOnPoint3d(WorldPoint(a.Part, a.LocalPoint), WorldPoint(b.Part, b.LocalPoint)));
        }

        [APIDescription(@"SetCoincident(a: AssemblyAxisDatum, b: AssemblyAxisDatum) -> None
Makes two axis datum centers coincident in world space.")]
        public void SetCoincident(AssemblyAxisDatum a, AssemblyAxisDatum b)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.CoincidentAxes,
                MateLabel("Coincident", a.Part, b.Part),
                a.Part, b.Part, a.LocalPoint, b.LocalPoint, a.LocalDirection, b.LocalDirection),
                a.Entity,
                b.Entity);
            AddSolverConstraints(new PointOnPoint3d(WorldPoint(a.Part, a.LocalPoint), WorldPoint(b.Part, b.LocalPoint)));
        }

        [APIDescription(@"SetParallel(a: AssemblyAxisDatum, b: AssemblyAxisDatum) -> None
Makes two axis directions parallel in world space.")]
        public void SetParallel(AssemblyAxisDatum a, AssemblyAxisDatum b)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.ParallelAxes,
                MateLabel("Parallel", a.Part, b.Part),
                a.Part, b.Part, a.LocalPoint, b.LocalPoint, a.LocalDirection, b.LocalDirection),
                a.Entity,
                b.Entity);
            AddSolverConstraints(new ParallelDirections3d(
                WorldDirection(a.Part, a.LocalDirection),
                WorldDirection(b.Part, b.LocalDirection)));
        }

        [APIDescription(@"SetPerpendicular(a: AssemblyAxisDatum, b: AssemblyAxisDatum) -> None
Makes two axis directions perpendicular in world space.")]
        public void SetPerpendicular(AssemblyAxisDatum a, AssemblyAxisDatum b)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.PerpendicularAxes,
                MateLabel("Perp", a.Part, b.Part),
                a.Part, b.Part, a.LocalPoint, b.LocalPoint, a.LocalDirection, b.LocalDirection),
                a.Entity,
                b.Entity);
            AddSolverConstraints(new PerpendicularDirections3d(
                WorldDirection(a.Part, a.LocalDirection),
                WorldDirection(b.Part, b.LocalDirection)));
        }

        [APIDescription(@"SetConcentric(a: AssemblyAxisDatum, b: AssemblyAxisDatum) -> None
Coaxial mate: axis lines are collinear. Translation and rotation along the shared axis remain free (typical pin-in-hole / cylinder-on-cylinder).")]
        public void SetConcentric(AssemblyAxisDatum a, AssemblyAxisDatum b)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.Concentric,
                MateLabel("Concentric", a.Part, b.Part),
                a.Part, b.Part, a.LocalPoint, b.LocalPoint, a.LocalDirection, b.LocalDirection),
                a.Entity,
                b.Entity);
            IncludeLocalLength(a.LocalPoint);
            IncludeLocalLength(b.LocalPoint);
            Vec3D perpendicularB1 = Vec3DOps.GetOrthoNormal(b.LocalDirection);
            Vec3D perpendicularB2 = Vec3DOps.Cross(
                b.LocalDirection.Normalized(),
                perpendicularB1).Normalized();
            AddSolverConstraints(new CoincidentAxes3d(
                WorldPoint(a.Part, a.LocalPoint),
                WorldDirection(a.Part, a.LocalDirection),
                WorldPoint(b.Part, b.LocalPoint),
                WorldDirection(b.Part, perpendicularB1),
                WorldDirection(b.Part, perpendicularB2)));
        }

        [APIDescription(@"SetDistance(a: AssemblyPointDatum, b: AssemblyPointDatum, distance: float) -> None
Keeps two point datums at the given world-space distance.")]
        public void SetDistance(AssemblyPointDatum a, AssemblyPointDatum b, double distance)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.DistancePoints,
                $"{MateLabel("Distance", a.Part, b.Part)} = {distance:G4}",
                a.Part, b.Part, a.LocalPoint, b.LocalPoint, scalar: distance),
                a.Entity,
                b.Entity);
            AddSolverConstraints(new DistanceBetweenPoints3d(
                WorldPoint(a.Part, a.LocalPoint),
                WorldPoint(b.Part, b.LocalPoint),
                distance));
        }

        [APIDescription(@"SetAngle(a: AssemblyAxisDatum, b: AssemblyAxisDatum, angleRadians: float) -> None
Sets the angle between two axis directions in world space.")]
        public void SetAngle(AssemblyAxisDatum a, AssemblyAxisDatum b, double angleRadians)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            double deg = angleRadians * 180.0 / Math.PI;
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.AngleAxes,
                $"{MateLabel("Angle", a.Part, b.Part)} = {deg:G2}°",
                a.Part, b.Part, a.LocalPoint, b.LocalPoint, a.LocalDirection, b.LocalDirection, angleRadians),
                a.Entity,
                b.Entity);
            AddSolverConstraints(new AngleBetweenVectors3d(
                WorldDirection(a.Part, a.LocalDirection),
                WorldDirection(b.Part, b.LocalDirection),
                angleRadians));
        }

        [APIDescription(@"SetCoincident(a: AssemblyPlaneDatum, b: AssemblyPlaneDatum) -> None
Face-on-face mate: planes coplanar (parallel normals and coincident origins).")]
        public void SetCoincident(AssemblyPlaneDatum a, AssemblyPlaneDatum b)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.CoincidentPlanes,
                MateLabel("Coincident", a.Part, b.Part),
                a.Part, b.Part, a.LocalOrigin, b.LocalOrigin, a.LocalNormal, b.LocalNormal),
                a.Entity,
                b.Entity);
            CPlane3D planeB = WorldPlane(b);
            AddSolverConstraints(
                new ParallelDirections3d(
                    WorldDirection(a.Part, a.LocalNormal),
                    WorldDirection(b.Part, b.LocalNormal)),
                new PointOnPlane3d(WorldPoint(a.Part, a.LocalOrigin), planeB));
        }

        [APIDescription(@"SetCoincidentOriented(a: AssemblyPlaneDatum, b: AssemblyPlaneDatum, oppositeNormals: bool) -> None
Face-on-face mate with directed normals. True requires n_a = −n_b; False requires n_a = n_b.
Named planar faces use their actual outward surface normals; mating two opposing bearing faces uses True.")]
        public void SetCoincidentOriented(
            AssemblyPlaneDatum a,
            AssemblyPlaneDatum b,
            bool oppositeNormals)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.CoincidentPlanes,
                MateLabel("Coincident oriented", a.Part, b.Part),
                a.Part, b.Part, a.LocalOrigin, b.LocalOrigin, a.LocalNormal, b.LocalNormal,
                scalar: oppositeNormals ? -1.0 : 1.0),
                a.Entity,
                b.Entity);
            IncludeLocalLength(a.LocalOrigin);
            IncludeLocalLength(b.LocalOrigin);
            CPlane3D planeB = WorldPlane(b);
            AddSolverConstraints(
                new DirectedParallelDirections3d(
                    WorldDirection(a.Part, a.LocalNormal),
                    WorldDirection(b.Part, b.LocalNormal),
                    oppositeNormals),
                new PointOnPlane3d(WorldPoint(a.Part, a.LocalOrigin), planeB));
        }

        [APIDescription(@"SetParallel(a: AssemblyPlaneDatum, b: AssemblyPlaneDatum) -> None
Makes two face normals parallel in world space.")]
        public void SetParallel(AssemblyPlaneDatum a, AssemblyPlaneDatum b)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.ParallelPlanes,
                MateLabel("Parallel", a.Part, b.Part),
                a.Part, b.Part, a.LocalOrigin, b.LocalOrigin, a.LocalNormal, b.LocalNormal),
                a.Entity,
                b.Entity);
            AddSolverConstraints(new ParallelDirections3d(
                WorldDirection(a.Part, a.LocalNormal),
                WorldDirection(b.Part, b.LocalNormal)));
        }

        [APIDescription(@"SetPerpendicular(a: AssemblyPlaneDatum, b: AssemblyPlaneDatum) -> None
Makes two face normals perpendicular in world space.")]
        public void SetPerpendicular(AssemblyPlaneDatum a, AssemblyPlaneDatum b)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.PerpendicularPlanes,
                MateLabel("Perp", a.Part, b.Part),
                a.Part, b.Part, a.LocalOrigin, b.LocalOrigin, a.LocalNormal, b.LocalNormal),
                a.Entity,
                b.Entity);
            AddSolverConstraints(new PerpendicularDirections3d(
                WorldDirection(a.Part, a.LocalNormal),
                WorldDirection(b.Part, b.LocalNormal)));
        }

        [APIDescription(@"SetDistance(a: AssemblyPlaneDatum, b: AssemblyPlaneDatum, distance: float) -> None
Offsets plane A from plane B along B's normal by the given signed distance (parallel normals implied).")]
        public void SetDistance(AssemblyPlaneDatum a, AssemblyPlaneDatum b, double distance)
        {
            EnsureOwnedDatum(a.Part, b.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.DistancePlanes,
                $"{MateLabel("Offset", a.Part, b.Part)} = {distance:G4}",
                a.Part, b.Part, a.LocalOrigin, b.LocalOrigin, a.LocalNormal, b.LocalNormal, distance),
                a.Entity,
                b.Entity);
            CPlane3D planeB = WorldPlane(b);
            AddSolverConstraints(
                new ParallelDirections3d(
                    WorldDirection(a.Part, a.LocalNormal),
                    WorldDirection(b.Part, b.LocalNormal)),
                new PlaneOffset3d(WorldPoint(a.Part, a.LocalOrigin), planeB, distance));
        }

        [APIDescription(@"SetPointOnPlane(point: AssemblyPointDatum, plane: AssemblyPlaneDatum) -> None
Constrains a point datum to lie on a plane datum.")]
        public void SetPointOnPlane(AssemblyPointDatum point, AssemblyPlaneDatum plane)
        {
            EnsureOwnedDatum(point.Part, plane.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.PointOnPlane,
                MateLabel("On plane", point.Part, plane.Part),
                point.Part, plane.Part, point.LocalPoint, plane.LocalOrigin, dirB: plane.LocalNormal),
                point.Entity,
                plane.Entity);
            AddSolverConstraints(new PointOnPlane3d(
                WorldPoint(point.Part, point.LocalPoint),
                WorldPlane(plane)));
        }

        [APIDescription(@"SetContact(point: AssemblyPointDatum, plane: AssemblyPlaneDatum) -> None
Unilateral contact: keeps the point in the positive half-space of the plane (n · (p − a) ≥ 0). Open contacts leave motion free; penetrating contacts close to g ≈ 0.")]
        public void SetContact(AssemblyPointDatum point, AssemblyPlaneDatum plane)
        {
            EnsureOwnedDatum(point.Part, plane.Part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.Contact,
                MateLabel("Contact", point.Part, plane.Part),
                point.Part, plane.Part, point.LocalPoint, plane.LocalOrigin, dirB: plane.LocalNormal),
                point.Entity,
                plane.Entity);
            AddSolverConstraints(new ContactHalfSpace3d(
                WorldPoint(point.Part, point.LocalPoint),
                WorldPlane(plane)));
        }

        [APIDescription(@"SolveConstraints(preferMinimalMovement: bool = False) -> None
Runs the 6-DOF rigid-body solver. preferMinimalMovement biases toward the current poses (sketch-style); leave it False when parts start stacked and must travel.")]
        public void SolveConstraints(bool preferMinimalMovement = false)
        {
            SolveConstraintsDetailed(preferMinimalMovement);
        }

        public SolveResult SolveConstraintsDetailed(bool preferMinimalMovement = false)
        {
            SolveResult result = _solver.SolveConstraints(preferMinimalMovement);
            TouchActivity();
            return result;
        }

        private static double MeshCharacteristicLength(AnchorMesh mesh)
        {
            IList<Vec3D> positions = mesh.Mesh.Positions;
            if (positions == null || positions.Count == 0)
                return 1.0;

            Vec3D low = positions[0];
            Vec3D high = low;
            for (int i = 1; i < positions.Count; i++)
            {
                Vec3D p = positions[i];
                low.X = Math.Min(low.X, p.X);
                low.Y = Math.Min(low.Y, p.Y);
                low.Z = Math.Min(low.Z, p.Z);
                high.X = Math.Max(high.X, p.X);
                high.Y = Math.Max(high.Y, p.Y);
                high.Z = Math.Max(high.Z, p.Z);
            }
            double length = (high - low).Length();
            return length > 1e-12 ? length : 1.0;
        }

        private void RecordMate(AssemblyMateRecord record, params string[] entities)
        {
            if (entities != null && entities.Length > 0)
                record.WithEntities(entities);
            _mateRecords.Add(record);
        }

        private void AddSolverConstraints(params IBaseEquation[] constraints)
        {
            _mateEquations.Add((_mateRecords[^1], (IBaseEquation[])constraints.Clone()));
            for (int i = 0; i < constraints.Length; i++)
                _solver.AddConstraint(constraints[i]);

            if (_solveAfterEveryConstraint)
                SolveConstraints();
            else
                TouchActivity();
        }

        private void IncludeLocalLength(Vec3D local)
        {
            double length = Math.Sqrt(local.X * local.X + local.Y * local.Y + local.Z * local.Z);
            _solver.IncludeCharacteristicLength(length);
        }

        private static string MateLabel(string kind, AssemblyPart a, AssemblyPart b) =>
            $"{kind}: {a.Mesh.Name} <> {b.Mesh.Name}";

        private void TouchActivity() => _api.NotifyAssemblyActivity(this);

        internal void EnsureOwnedPart(AssemblyPart part)
        {
            if (part == null)
                throw new ArgumentNullException(nameof(part));
            if (part.OccurrenceContext != null)
            {
                if (part.Assembly != this || !part.OccurrenceContext.Child.ContainsAssembly(this) ||
                    !_parts.Contains(part.DefinitionPart))
                    throw new ArgumentException("AssemblyPart does not refer to a part in this occurrence definition.");
                return;
            }
            if (part.Assembly != this)
                throw new ArgumentException("AssemblyPart belongs to a different Assembly.");
            if (!_parts.Contains(part.DefinitionPart))
                throw new ArgumentException("AssemblyPart was not created by this Assembly.");
        }

        private void EnsurePartInTree(AssemblyPart part)
        {
            if (part == null)
                throw new ArgumentNullException(nameof(part));
            if (part.Assembly == this && part.OccurrenceContext == null)
            {
                if (!_parts.Contains(part.DefinitionPart))
                    throw new ArgumentException("AssemblyPart was not created by this Assembly.");
                return;
            }
            if (part.OccurrenceContext != null)
            {
                if (!IsPartReferenceInTree(part))
                    throw new ArgumentException(
                        $"AssemblyPart '{part.Mesh.Name}' does not belong to this assembly occurrence.");
                return;
            }
            if (!ContainsAssembly(part.Assembly))
                throw new ArgumentException(
                    $"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}' or its sub-assemblies.");
            if (FindDirectOccurrenceContaining(part.Assembly, out int matches) != null && matches > 1)
                throw new ArgumentException(
                    $"AssemblyPart '{part.Mesh.Name}' is in multiple occurrences; get it from the intended AssemblyOccurrence.");
        }

        private void EnsureOwnedDatum(AssemblyPart a, AssemblyPart b)
        {
            EnsurePartInTree(a);
            EnsurePartInTree(b);
        }

        private void EnsureOwnedOccurrence(AssemblyOccurrence occurrence)
        {
            if (occurrence == null)
                throw new ArgumentNullException(nameof(occurrence));
            if (occurrence.Parent != this || !_occurrences.Contains(occurrence))
                throw new ArgumentException("AssemblyOccurrence was not created by this Assembly.");
        }

        internal bool ContainsAssembly(Assembly other)
        {
            if (other == null)
                return false;
            if (other == this)
                return true;
            for (int i = 0; i < _occurrences.Count; i++)
            {
                if (_occurrences[i].Child.ContainsAssembly(other))
                    return true;
            }
            return false;
        }

        internal AssemblyOccurrence FindDirectOccurrenceContaining(Assembly nested)
            => FindDirectOccurrenceContaining(nested, out _);

        private AssemblyOccurrence FindDirectOccurrenceContaining(Assembly nested, out int matches)
        {
            AssemblyOccurrence found = null;
            matches = 0;
            for (int i = 0; i < _occurrences.Count; i++)
            {
                if (_occurrences[i].Child.ContainsAssembly(nested))
                {
                    found = _occurrences[i];
                    matches++;
                }
            }
            return found;
        }

        private AssemblyOccurrence FindOccurrenceContaining(AssemblyPart part)
        {
            if (part.OccurrenceContext != null)
            {
                if (part.OccurrenceContext.Parent == this && _occurrences.Contains(part.OccurrenceContext))
                    return part.OccurrenceContext;
                for (int i = 0; i < part.DefinitionOccurrencePath.Count; i++)
                    if (part.DefinitionOccurrencePath[i].Parent == this)
                        return part.DefinitionOccurrencePath[i];
                return null;
            }
            AssemblyOccurrence occurrence = FindDirectOccurrenceContaining(part.Assembly, out int matches);
            if (matches > 1)
                throw new ArgumentException(
                    $"AssemblyPart '{part.Mesh.Name}' is in multiple occurrences; get it from the intended AssemblyOccurrence.");
            return occurrence;
        }

        private bool IsPartReferenceInTree(AssemblyPart part)
        {
            AssemblyOccurrence context = part.OccurrenceContext;
            if (context.Parent == this && _occurrences.Contains(context))
                return context.Child.ContainsAssembly(part.Assembly);
            for (int i = 0; i < part.DefinitionOccurrencePath.Count; i++)
            {
                AssemblyOccurrence step = part.DefinitionOccurrencePath[i];
                if (step.Parent == this)
                    return step.Child.ContainsAssembly(part.Assembly);
            }
            return part.Assembly == this && context.Child == this;
        }

        internal static Transform PoseInAssembly(AssemblyPart part, Assembly frame)
        {
            if (part.Assembly == frame)
                return part.EvaluateDefinitionPose();

            Transform pose = part.EvaluateDefinitionPose();
            Assembly current = part.Assembly;
            while (current != frame)
            {
                if (current.Parent == null)
                    throw new ArgumentException(
                        $"AssemblyPart '{part.Mesh.Name}' is not in assembly '{frame.Name}'.");
                AssemblyOccurrence step = current.Parent.FindDirectOccurrenceContaining(current);
                if (step == null)
                    throw new InvalidOperationException("Broken assembly occurrence chain.");
                pose = TransformMath.Compose(step.EvaluatePose(), pose);
                current = current.Parent;
            }
            return pose;
        }

        internal static Transform PoseInOccurrence(AssemblyPart part, Assembly frame,
            IReadOnlyList<AssemblyOccurrence> definitionPath)
        {
            Transform pose = part.EvaluateDefinitionPose();
            Assembly current = part.Assembly;
            for (int i = definitionPath.Count - 1; i >= 0; i--)
            {
                AssemblyOccurrence step = definitionPath[i];
                if (step.Child != current)
                    throw new ArgumentException("Occurrence path does not lead to the referenced AssemblyPart.");
                pose = TransformMath.Compose(step.Transform.Evaluate(), pose);
                current = step.Parent;
            }
            if (current != frame)
                throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not inside '{frame.Name}'.");
            return pose;
        }

        internal static Transform PoseOfOccurrence(Assembly frame,
            IReadOnlyList<AssemblyOccurrence> definitionPath)
        {
            if (definitionPath.Count == 0)
                return new Transform(new Vec3D(0), TransformMath.IdentityOrientation);
            Transform pose = definitionPath[^1].Transform.Evaluate();
            for (int i = definitionPath.Count - 2; i >= 0; i--)
                pose = TransformMath.Compose(definitionPath[i].Transform.Evaluate(), pose);
            if (definitionPath[0].Parent != frame)
                throw new ArgumentException($"Occurrence path does not begin inside '{frame.Name}'.");
            return pose;
        }

        private static Transform RelativePoseInOccurrence(AssemblyPart part, AssemblyOccurrence occurrence)
        {
            if (part.OccurrenceContext == null)
                return PoseInAssembly(part, occurrence.Child);
            if (ReferenceEquals(part.OccurrenceContext, occurrence))
                return PoseInOccurrence(part, occurrence.Child, part.DefinitionOccurrencePath);
            for (int i = 0; i < part.DefinitionOccurrencePath.Count; i++)
            {
                if (!ReferenceEquals(part.DefinitionOccurrencePath[i], occurrence))
                    continue;
                var suffix = new AssemblyOccurrence[part.DefinitionOccurrencePath.Count - i - 1];
                for (int j = 0; j < suffix.Length; j++)
                    suffix[j] = part.DefinitionOccurrencePath[i + j + 1];
                return PoseInOccurrence(part, occurrence.Child, suffix);
            }
            throw new ArgumentException("AssemblyPart reference does not pass through this occurrence.");
        }

        internal CTransform SolverTransformOf(AssemblyPart part)
        {
            AssemblyOccurrence occ = FindOccurrenceContaining(part);
            if (occ == null && part.Assembly == this)
                return part.Transform;
            if (occ == null)
                throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}'.");
            return occ.Transform;
        }

        internal CVec3D WorldPoint(AssemblyPart part, Vec3D localPoint)
        {
            EnsurePartInTree(part);
            AssemblyOccurrence occ = FindOccurrenceContaining(part);
            if (occ == null && part.Assembly == this)
                return part.WorldPoint(localPoint);

            if (occ == null)
                throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}'.");
            Transform relative = RelativePoseInOccurrence(part, occ);
            Vec3D inOccurrence = TransformMath.TransformPoint(in relative, localPoint);
            return occ.Transform.PointLocalToGlobal(CVec3D.Constant(inOccurrence));
        }

        internal CVec3D WorldDirection(AssemblyPart part, Vec3D localDirection)
        {
            EnsurePartInTree(part);
            AssemblyOccurrence occ = FindOccurrenceContaining(part);
            if (occ == null && part.Assembly == this)
                return part.WorldDirection(localDirection);

            if (occ == null)
                throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}'.");
            Transform relative = RelativePoseInOccurrence(part, occ);
            Vec3D inOccurrence = TransformMath.TransformDirection(in relative, localDirection);
            return occ.Transform.DirectionLocalToGlobal(CVec3D.Constant(inOccurrence));
        }

        internal CPlane3D WorldPlane(AssemblyPlaneDatum plane)
        {
            return new CPlane3D(
                WorldPoint(plane.Part, plane.LocalOrigin),
                WorldDirection(plane.Part, plane.LocalNormal));
        }

        [APIDescription(@"WorldPoseOf(part: AssemblyPart) -> Transform
Pose of a direct or nested part in this assembly's frame.")]
        public Transform WorldPoseOf(AssemblyPart part)
        {
            EnsurePartInTree(part);
            AssemblyOccurrence occ = FindOccurrenceContaining(part);
            if (occ == null && part.Assembly == this)
                return part.EvaluateDefinitionPose();
            if (occ == null)
                throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}'.");
            return TransformMath.Compose(occ.EvaluatePose(), RelativePoseInOccurrence(part, occ));
        }

        [APIDescription(@"GetLeaves() -> IReadOnlyList[AssemblyLeaf]
Recursive leaf parts with occurrence paths and current world poses.")]
        public IReadOnlyList<AssemblyLeaf> GetLeaves()
        {
            var parts = new List<AssemblyPart>();
            var poses = new List<Transform>();
            var paths = new List<string>();
            CollectLeafWorldPoses(parts, poses, paths);
            return parts.Select((part, index) => new AssemblyLeaf(paths[index], part, poses[index])).ToList();
        }
        public void CollectLeafWorldPoses(List<AssemblyPart> parts, List<Transform> worldPoses, List<string> paths = null)
        {
            CollectLeafWorldPoses(
                parts,
                worldPoses,
                new Transform(default, TransformMath.IdentityOrientation), paths, Name, null,
                Array.Empty<AssemblyOccurrence>());
        }

        private void CollectLeafWorldPoses(List<AssemblyPart> parts, List<Transform> worldPoses,
            Transform parentWorld, List<string> paths, string path, AssemblyOccurrence context,
            IReadOnlyList<AssemblyOccurrence> definitionPath)
        {
            for (int i = 0; i < _parts.Count; i++)
            {
                AssemblyPart part = _parts[i];
                parts.Add(context == null ? part : part.ForOccurrence(context, definitionPath));
                paths?.Add($"{path}/{part.Mesh.Name}[{i+1}]");
                worldPoses.Add(TransformMath.Compose(parentWorld, part.EvaluateDefinitionPose()));
            }
            for (int i = 0; i < _occurrences.Count; i++)
            {
                AssemblyOccurrence occ = _occurrences[i];
                Transform occWorld = TransformMath.Compose(parentWorld, occ.EvaluatePose());
                AssemblyOccurrence nextContext = context ?? occ;
                IReadOnlyList<AssemblyOccurrence> nextPath = definitionPath;
                if (context != null)
                {
                    var extended = new AssemblyOccurrence[definitionPath.Count + 1];
                    for (int j = 0; j < definitionPath.Count; j++)
                        extended[j] = definitionPath[j];
                    extended[^1] = occ;
                    nextPath = extended;
                }
                occ.Child.CollectLeafWorldPoses(parts, worldPoses, occWorld, paths,
                    $"{path}/{occ.Name}[{i+1}]", nextContext, nextPath);
            }
        }

        internal void ApplyComposedPose(Transform parentWorld)
        {
            // A shared definition has no single display pose. Assembly leaves,
            // sections and interference queries use occurrence-specific snapshots.
            if (SuppressOccurrenceMeshUpdates)
                return;
            for (int i = 0; i < _parts.Count; i++)
            {
                AssemblyPart part = _parts[i];
                part.Mesh.Update(TransformMath.Compose(parentWorld, part.EvaluatePose()));
            }
            for (int i = 0; i < _occurrences.Count; i++)
            {
                AssemblyOccurrence occ = _occurrences[i];
                occ.Child.ApplyComposedPose(TransformMath.Compose(parentWorld, occ.EvaluatePose()));
            }
        }

        private void IncludeSubtreeCharacteristicLength(Assembly child)
        {
            for (int i = 0; i < child._parts.Count; i++)
                _solver.IncludeCharacteristicLength(MeshCharacteristicLength(child._parts[i].Mesh));
            for (int i = 0; i < child._occurrences.Count; i++)
                IncludeSubtreeCharacteristicLength(child._occurrences[i].Child);
        }

        private static AssemblyPart FirstLeafPart(Assembly assembly)
        {
            if (assembly._parts.Count > 0)
                return assembly._parts[0];
            for (int i = 0; i < assembly._occurrences.Count; i++)
            {
                AssemblyPart leaf = FirstLeafPart(assembly._occurrences[i].Child);
                if (leaf != null)
                    return leaf;
            }
            return null;
        }
    }
}
