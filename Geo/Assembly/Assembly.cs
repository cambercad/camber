using GeoCore;
using GeoMeta;
using GeoSolver;
using GeoSolver.Kinematics;

namespace Geo
{
    [APIDescription(@"Assembly: 3D mate solver for registered meshes. AddPart returns AssemblyPart handles; create datums on parts; apply mates (SetCoincident, SetParallel, SetPerpendicular, SetConcentric, SetDistance, SetAngle, SetContact, …); then SolveConstraints().")]
    public class Assembly
    {
        private readonly GeoAPI _api;
        private readonly KinematicSolver _solver = new KinematicSolver();
        private readonly List<AssemblyPart> _parts = new List<AssemblyPart>();
        private readonly List<AssemblyMateRecord> _mateRecords = new List<AssemblyMateRecord>();
        private bool _solveAfterEveryConstraint = true;

        internal Assembly(GeoAPI api, string name)
        {
            _api = api ?? throw new ArgumentNullException(nameof(api));
            Name = name ?? throw new ArgumentNullException(nameof(name));
        }

        public string Name { get; }

        [APIDescription(@"GetParts() -> IReadOnlyList[AssemblyPart]
All parts added via AddPart on this assembly (live list).")]
        public IReadOnlyList<AssemblyPart> GetParts() => _parts;

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

        [APIDescription(@"FixPart(part: AssemblyPart) -> None
Locks the part at its current pose (6 DOF).")]
        public void FixPart(AssemblyPart part)
        {
            EnsureOwnedPart(part);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.FixPart,
                $"Fix {part.Mesh.Name}",
                part),
                part.Mesh.Name + ":");
            AddSolverConstraints(new FixedTransformConstraint3d(part.Transform));
        }

        [APIDescription(@"FixPart(part: AssemblyPart, position: Vec3D, orientation: Quaternion) -> None
Locks the part at the given world pose.")]
        public void FixPart(AssemblyPart part, Vec3D position, Quaternion orientation)
        {
            EnsureOwnedPart(part);
            orientation = TransformMath.NormalizeDefault(orientation);
            RecordMate(new AssemblyMateRecord(
                AssemblyMateKind.FixPart,
                $"Fix {part.Mesh.Name}",
                part),
                part.Mesh.Name + ":");
            AddSolverConstraints(new FixedTransformConstraint3d(part.Transform, new Transform(position, orientation)));
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
            AddSolverConstraints(new PointOnPoint3d(a.Part.WorldPoint(a.LocalPoint), b.Part.WorldPoint(b.LocalPoint)));
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
            AddSolverConstraints(new PointOnPoint3d(a.Part.WorldPoint(a.LocalPoint), b.Part.WorldPoint(b.LocalPoint)));
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
                a.Part.WorldDirection(a.LocalDirection),
                b.Part.WorldDirection(b.LocalDirection)));
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
                a.Part.WorldDirection(a.LocalDirection),
                b.Part.WorldDirection(b.LocalDirection)));
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
                a.Part.WorldPoint(a.LocalPoint),
                a.Part.WorldDirection(a.LocalDirection),
                b.Part.WorldPoint(b.LocalPoint),
                b.Part.WorldDirection(perpendicularB1),
                b.Part.WorldDirection(perpendicularB2)));
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
                a.Part.WorldPoint(a.LocalPoint),
                b.Part.WorldPoint(b.LocalPoint),
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
                a.Part.WorldDirection(a.LocalDirection),
                b.Part.WorldDirection(b.LocalDirection),
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
            CPlane3D planeB = b.Part.WorldPlane(b);
            AddSolverConstraints(
                new ParallelDirections3d(
                    a.Part.WorldDirection(a.LocalNormal),
                    b.Part.WorldDirection(b.LocalNormal)),
                new PointOnPlane3d(a.Part.WorldPoint(a.LocalOrigin), planeB));
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
            CPlane3D planeB = b.Part.WorldPlane(b);
            AddSolverConstraints(
                new DirectedParallelDirections3d(
                    a.Part.WorldDirection(a.LocalNormal),
                    b.Part.WorldDirection(b.LocalNormal),
                    oppositeNormals),
                new PointOnPlane3d(a.Part.WorldPoint(a.LocalOrigin), planeB));
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
                a.Part.WorldDirection(a.LocalNormal),
                b.Part.WorldDirection(b.LocalNormal)));
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
                a.Part.WorldDirection(a.LocalNormal),
                b.Part.WorldDirection(b.LocalNormal)));
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
            CPlane3D planeB = b.Part.WorldPlane(b);
            AddSolverConstraints(
                new ParallelDirections3d(
                    a.Part.WorldDirection(a.LocalNormal),
                    b.Part.WorldDirection(b.LocalNormal)),
                new PlaneOffset3d(a.Part.WorldPoint(a.LocalOrigin), planeB, distance));
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
                point.Part.WorldPoint(point.LocalPoint),
                plane.Part.WorldPlane(plane)));
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
                point.Part.WorldPoint(point.LocalPoint),
                plane.Part.WorldPlane(plane)));
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

        private void EnsureOwnedDatum(AssemblyPart a, AssemblyPart b)
        {
            EnsureOwnedPart(a);
            EnsureOwnedPart(b);
        }
    }
}
