using GeoCore;
using GeoMeta;
using GeoSolver;
using GeoSolver.Kinematics;

namespace Geo
{
    [APIDescription(@"Assembly: 3D mate solver for registered meshes. AddPart places solids; AddSubAssembly places a child assembly (which may itself contain parts and sub-assemblies) as a rigid occurrence. Create datums on parts (including nested ones); apply mates; then SolveConstraints().")]
    public class Assembly
    {
        private readonly GeoAPI _api;
        private readonly KinematicSolver _solver = new KinematicSolver();
        private readonly List<AssemblyPart> _parts = new List<AssemblyPart>();
        private readonly List<AssemblyOccurrence> _occurrences = new List<AssemblyOccurrence>();
        private readonly List<AssemblyMateRecord> _mateRecords = new List<AssemblyMateRecord>();
        private bool _solveAfterEveryConstraint = true;

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
Registers a rigid body at the given initial pose. Mesh must belong to this GeoAPI instance.")]
        public AssemblyPart AddPart(AnchorMesh mesh, Vec3D position, Quaternion orientation = default)
        {
            if (mesh == null)
                throw new ArgumentNullException(nameof(mesh));

            if (!_api.IsRegisteredMesh(mesh))
                throw new ArgumentException($"Mesh '{mesh.Name}' is not registered on this GeoAPI instance.", nameof(mesh));

            orientation = TransformMath.NormalizeDefault(orientation);
            var initialPose = new Transform(position, orientation);
            mesh.CaptureRigidRestPose(_api.Converter);

            _solver.IncludeCharacteristicLength(MeshCharacteristicLength(mesh));
            RigidTransform<AnchorMesh> rigidBody = _solver.AddRigidBody(mesh, initialPose);
            var part = new AssemblyPart(this, mesh, rigidBody);
            _parts.Add(part);
            TouchActivity();
            return part;
        }

        [APIDescription(@"AddSubAssembly(child: Assembly, position: Vec3D, orientation: Quaternion = identity) -> AssemblyOccurrence
Places a child assembly as a rigid occurrence at the given pose. The child may contain parts and further sub-assemblies. Child internals keep their last solved relative poses; this assembly owns 6 DOF for the whole subtree. Mates on this assembly may use datums on nested parts.")]
        public AssemblyOccurrence AddSubAssembly(Assembly child, Vec3D position, Quaternion orientation = default)
        {
            if (child == null)
                throw new ArgumentNullException(nameof(child));
            if (!ReferenceEquals(child._api, _api))
                throw new ArgumentException("Sub-assembly must belong to the same GeoAPI instance.", nameof(child));
            if (child == this)
                throw new ArgumentException("An assembly cannot contain itself.", nameof(child));
            if (child.Parent != null)
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
            var initialPose = new Transform(position, orientation);
            IncludeSubtreeCharacteristicLength(child);
            _solver.IncludeCharacteristicLength(position.Length());

            var rig = new AssemblyOccurrenceRig(child);
            RigidTransform<AssemblyOccurrenceRig> rigidBody = _solver.AddRigidBody(rig, initialPose);
            var occurrence = new AssemblyOccurrence(this, child, rigidBody);
            _occurrences.Add(occurrence);
            child.Parent = this;
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
                part),
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
            if (part.Assembly != this)
            {
                AssemblyOccurrence occ = FindDirectOccurrenceContaining(part.Assembly);
                if (occ == null)
                    throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}'.");
                Transform relative = PoseInAssembly(part, occ.Child);
                desired = TransformMath.Compose(desired, TransformMath.Inverse(relative));
            }
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.FixPart,
                $"Fix {part.Mesh.Name}",
                part),
                part.Mesh.Name + ":");
            AddSolverConstraints(new FixedTransformConstraint3d(SolverTransformOf(part), desired));
        }

        [APIDescription(@"FixSubAssembly(occurrence: AssemblyOccurrence) -> None
Locks a nested sub-assembly at its current pose (6 DOF).")]
        public void FixSubAssembly(AssemblyOccurrence occurrence)
        {
            EnsureOwnedOccurrence(occurrence);
            AssemblyPart leaf = FirstLeafPart(occurrence.Child);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.FixPart,
                $"Fix {occurrence.Child.Name}",
                leaf),
                occurrence.Child.Name + ":");
            AddSolverConstraints(new FixedTransformConstraint3d(occurrence.Transform));
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
Named extrude caps (ExtrudeTop / ExtrudeBottom) both store the sketch +Z, not the solid outward normal, so a flange-to-flange mate of those names uses False to get anti-parallel outward faces.")]
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
            if (part.Assembly != this)
                throw new ArgumentException("AssemblyPart belongs to a different Assembly.");
            if (!_parts.Contains(part))
                throw new ArgumentException("AssemblyPart was not created by this Assembly.");
        }

        private void EnsurePartInTree(AssemblyPart part)
        {
            if (part == null)
                throw new ArgumentNullException(nameof(part));
            if (part.Assembly == this)
            {
                if (!_parts.Contains(part))
                    throw new ArgumentException("AssemblyPart was not created by this Assembly.");
                return;
            }
            if (!ContainsAssembly(part.Assembly))
                throw new ArgumentException(
                    $"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}' or its sub-assemblies.");
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
        {
            for (int i = 0; i < _occurrences.Count; i++)
            {
                if (_occurrences[i].Child.ContainsAssembly(nested))
                    return _occurrences[i];
            }
            return null;
        }

        internal static Transform PoseInAssembly(AssemblyPart part, Assembly frame)
        {
            if (part.Assembly == frame)
                return part.EvaluatePose();

            Transform pose = part.EvaluatePose();
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

        internal CTransform SolverTransformOf(AssemblyPart part)
        {
            if (part.Assembly == this)
                return part.Transform;
            AssemblyOccurrence occ = FindDirectOccurrenceContaining(part.Assembly);
            if (occ == null)
                throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}'.");
            return occ.Transform;
        }

        internal CVec3D WorldPoint(AssemblyPart part, Vec3D localPoint)
        {
            EnsurePartInTree(part);
            if (part.Assembly == this)
                return part.WorldPoint(localPoint);

            AssemblyOccurrence occ = FindDirectOccurrenceContaining(part.Assembly);
            if (occ == null)
                throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}'.");
            Transform relative = PoseInAssembly(part, occ.Child);
            Vec3D inOccurrence = TransformMath.TransformPoint(in relative, localPoint);
            return occ.Transform.PointLocalToGlobal(CVec3D.Constant(inOccurrence));
        }

        internal CVec3D WorldDirection(AssemblyPart part, Vec3D localDirection)
        {
            EnsurePartInTree(part);
            if (part.Assembly == this)
                return part.WorldDirection(localDirection);

            AssemblyOccurrence occ = FindDirectOccurrenceContaining(part.Assembly);
            if (occ == null)
                throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}'.");
            Transform relative = PoseInAssembly(part, occ.Child);
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
            if (part.Assembly == this)
                return part.EvaluatePose();
            AssemblyOccurrence occ = FindDirectOccurrenceContaining(part.Assembly);
            if (occ == null)
                throw new ArgumentException($"AssemblyPart '{part.Mesh.Name}' is not in assembly '{Name}'.");
            return TransformMath.Compose(occ.EvaluatePose(), PoseInAssembly(part, occ.Child));
        }

        public void CollectLeafWorldPoses(List<AssemblyPart> parts, List<Transform> worldPoses)
        {
            CollectLeafWorldPoses(
                parts,
                worldPoses,
                new Transform(default, TransformMath.IdentityOrientation));
        }

        private void CollectLeafWorldPoses(List<AssemblyPart> parts, List<Transform> worldPoses, Transform parentWorld)
        {
            for (int i = 0; i < _parts.Count; i++)
            {
                AssemblyPart part = _parts[i];
                parts.Add(part);
                worldPoses.Add(TransformMath.Compose(parentWorld, part.EvaluatePose()));
            }
            for (int i = 0; i < _occurrences.Count; i++)
            {
                AssemblyOccurrence occ = _occurrences[i];
                Transform occWorld = TransformMath.Compose(parentWorld, occ.EvaluatePose());
                occ.Child.CollectLeafWorldPoses(parts, worldPoses, occWorld);
            }
        }

        internal void ApplyComposedPose(Transform parentWorld)
        {
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
