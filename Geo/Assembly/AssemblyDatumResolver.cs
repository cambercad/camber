using GeoCore;
using GeoSolver;
using GeoSolver.Kinematics;

namespace Geo
{
    internal readonly struct AxisDatumLocal
    {
        public AxisDatumLocal(Vec3D point, Vec3D direction)
        {
            Point = point;
            Direction = direction;
        }

        public Vec3D Point { get; }
        public Vec3D Direction { get; }
    }

    internal readonly struct PlaneDatumLocal
    {
        public PlaneDatumLocal(Vec3D origin, Vec3D normal)
        {
            Origin = origin;
            Normal = normal;
        }

        public Vec3D Origin { get; }
        public Vec3D Normal { get; }
    }

    internal static class AssemblyDatumResolver
    {
        public static string EntityName(AnchorMesh mesh, string reference)
        {
            if (mesh == null || string.IsNullOrWhiteSpace(reference))
                return "";

            string patchName = mesh.ResolveLocalPatchNamePublic(reference);
            if (patchName != null)
                return mesh.Name + ":" + patchName;

            string edgeName = reference.Trim();
            int slash = edgeName.IndexOf(':');
            if (slash >= 0 && slash < edgeName.Length - 1)
                edgeName = edgeName.Substring(slash + 1);
            return mesh.Name + ":" + edgeName;
        }

        public static AxisDatumLocal ResolveAxis(AnchorMesh mesh, string reference)
        {
            string patchName = mesh.ResolveLocalPatchNamePublic(reference);
            if (patchName == null)
                throw new ArgumentException($"Cannot resolve axis datum '{reference}' on mesh '{mesh.Name}'.");

        if (mesh.TryGetCylinderFromPatch(patchName, out CylinderSurfaceParams cylinder))
        {
            Vec3D axis = cylinder.Axis;
            axis.Normalize();
            Vec3D axisPoint = cylinder.Origin + axis * (cylinder.Height * 0.5);
            return new AxisDatumLocal(axisPoint, axis);
        }

            if (mesh.TryGetPlaneFromPatch(patchName, out PlaneSurfaceParams plane))
            {
                Vec3D normal = plane.Normal;
                normal.Normalize();
                return new AxisDatumLocal(plane.Origin, normal);
            }

            if (mesh.TryGetPointOnEdge(reference, out Vec3D edgePoint))
            {
                if (!TryGetEdgeTangent(mesh, reference, out Vec3D tangent))
                    throw new ArgumentException($"Edge '{reference}' does not define a tangent direction.");
                tangent.Normalize();
                return new AxisDatumLocal(edgePoint, tangent);
            }

            throw new ArgumentException($"Reference '{reference}' on mesh '{mesh.Name}' does not define an axis.");
        }

        public static Vec3D ResolvePoint(AnchorMesh mesh, string reference)
        {
            string patchName = mesh.ResolveLocalPatchNamePublic(reference);
            if (patchName != null)
            {
                if (mesh.TryGetCylinderFromPatch(patchName, out CylinderSurfaceParams cylinder))
                {
                    Vec3D axis = cylinder.Axis;
                    axis.Normalize();
                    return cylinder.Origin + axis * (cylinder.Height * 0.5);
                }

                if (mesh.TryGetPlaneFromPatch(patchName, out PlaneSurfaceParams plane))
                    return plane.Origin;

                if (mesh.TryGetPointOnSurface(reference, out Vec3D surfacePoint))
                    return surfacePoint;
            }

            if (mesh.TryGetPointOnSurface(reference, out Vec3D uvPoint))
                return uvPoint;

            if (mesh.TryGetPointOnEdge(reference, out Vec3D edgePoint))
                return edgePoint;

            throw new ArgumentException($"Cannot resolve point datum '{reference}' on mesh '{mesh.Name}'.");
        }

        public static PlaneDatumLocal ResolvePlane(AnchorMesh mesh, string reference)
        {
            string patchName = mesh.ResolveLocalPatchNamePublic(reference);
            if (patchName == null)
                throw new ArgumentException($"Cannot resolve plane datum '{reference}' on mesh '{mesh.Name}'.");

            if (mesh.TryGetPlaneFromPatch(patchName, out PlaneSurfaceParams plane))
            {
                Vec3D normal = plane.Normal;
                normal.Normalize();
                return new PlaneDatumLocal(plane.Origin, normal);
            }

            throw new ArgumentException($"Reference '{reference}' on mesh '{mesh.Name}' does not define a plane.");
        }

        private static bool TryGetEdgeTangent(AnchorMesh mesh, string reference, out Vec3D tangent)
        {
            tangent = default;
            if (!mesh.TryGetEdge(reference, out LineStrip3D edge) || edge == null)
                return false;

            Vec3D start = edge.EvaluateUniform(0);
            Vec3D end = edge.EvaluateUniform(1);
            tangent = end - start;
            return tangent.LengthSquared() > 1e-18;
        }
    }
}
