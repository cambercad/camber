using GeoCore;
using GeoMeta;
using GeoSolver;
using GeoSolver.Kinematics;

namespace Geo
{
    [APIDescription(@"AssemblyPart: rigid-body handle returned by Assembly.AddPart. Create datums here; pass to Assembly mate methods.")]
    public sealed class AssemblyPart
    {
        internal AssemblyPart(Assembly assembly, AnchorMesh mesh, RigidTransform<AnchorMesh> rigidBody)
        {
            Assembly = assembly;
            Mesh = mesh;
            RigidBody = rigidBody;
        }

        internal Assembly Assembly { get; }
        public AnchorMesh Mesh { get; }
        internal RigidTransform<AnchorMesh> RigidBody { get; }
        internal CTransform Transform => RigidBody.Transform;

        [APIDescription(@"EvaluatePose() -> Transform
Current solved world pose (position + orientation) of this part.")]
        public Transform EvaluatePose() => Transform.Evaluate();

        [APIDescription(@"AddAxisDatum(reference: str) -> AssemblyAxisDatum
Infers axis point and direction from a patch or edge on this part's mesh (e.g. cylindrical hole side, ExtrudeTop/Bottom, edge tangent).")]
        public AssemblyAxisDatum AddAxisDatum(string reference)
        {
            Assembly.EnsureOwnedPart(this);
            AxisDatumLocal axis = AssemblyDatumResolver.ResolveAxis(Mesh, reference);
            Vec3D localDirection = axis.Direction;
            localDirection.Normalize();
            return new AssemblyAxisDatum(this, axis.Point, localDirection, AssemblyDatumResolver.EntityName(Mesh, reference));
        }

        [APIDescription(@"AddAxisDatumAt(localPoint: Vec3D, localDirection: Vec3D) -> AssemblyAxisDatum
Explicit axis datum in part-local coordinates.")]
        public AssemblyAxisDatum AddAxisDatumAt(Vec3D localPoint, Vec3D localDirection)
        {
            Assembly.EnsureOwnedPart(this);
            localDirection.Normalize();
            return new AssemblyAxisDatum(this, localPoint, localDirection);
        }

        [APIDescription(@"AddPointDatum(reference: str) -> AssemblyPointDatum
Infers a point from a patch, surface UV, or edge on this part's mesh.")]
        public AssemblyPointDatum AddPointDatum(string reference)
        {
            Assembly.EnsureOwnedPart(this);
            Vec3D localPoint = AssemblyDatumResolver.ResolvePoint(Mesh, reference);
            return new AssemblyPointDatum(this, localPoint, AssemblyDatumResolver.EntityName(Mesh, reference));
        }

        [APIDescription(@"AddPointDatumAt(localPoint: Vec3D) -> AssemblyPointDatum
Explicit point datum in part-local coordinates.")]
        public AssemblyPointDatum AddPointDatumAt(Vec3D localPoint)
        {
            Assembly.EnsureOwnedPart(this);
            return new AssemblyPointDatum(this, localPoint);
        }

        [APIDescription(@"AddPlaneDatum(reference: str) -> AssemblyPlaneDatum
Infers a planar face from a patch on this part's mesh (e.g. ExtrudeTop, ExtrudeBottom).")]
        public AssemblyPlaneDatum AddPlaneDatum(string reference)
        {
            Assembly.EnsureOwnedPart(this);
            PlaneDatumLocal plane = AssemblyDatumResolver.ResolvePlane(Mesh, reference, Assembly.Converter);
            Vec3D normal = plane.Normal;
            normal.Normalize();
            return new AssemblyPlaneDatum(this, plane.Origin, normal, AssemblyDatumResolver.EntityName(Mesh, reference));
        }

        /// <summary>Return an oriented planar face frame in body-local coordinates.
        /// Preserves its authored in-plane reference direction where available;
        /// otherwise uses a deterministic orthonormal basis. Display poses do not affect it.</summary>
        public CoordinateSystem GetPlaneFrame(string reference)
        {
            Assembly.EnsureOwnedPart(this);
            return AssemblyDatumResolver.ResolvePlaneFrame(Mesh, reference, Assembly.Converter);
        }

        [APIDescription(@"AddPlaneDatumAt(localOrigin: Vec3D, localNormal: Vec3D) -> AssemblyPlaneDatum
Explicit plane datum in part-local coordinates.")]
        public AssemblyPlaneDatum AddPlaneDatumAt(Vec3D localOrigin, Vec3D localNormal)
        {
            Assembly.EnsureOwnedPart(this);
            localNormal.Normalize();
            return new AssemblyPlaneDatum(this, localOrigin, localNormal);
        }

        internal CVec3D WorldPoint(Vec3D localPoint)
        {
            return Transform.PointLocalToGlobal(CVec3D.Constant(localPoint));
        }

        internal CVec3D WorldDirection(Vec3D localDirection)
        {
            return Transform.DirectionLocalToGlobal(CVec3D.Constant(localDirection));
        }

        internal CPlane3D WorldPlane(AssemblyPlaneDatum plane)
        {
            return new CPlane3D(WorldPoint(plane.LocalOrigin), WorldDirection(plane.LocalNormal));
        }
    }
}
