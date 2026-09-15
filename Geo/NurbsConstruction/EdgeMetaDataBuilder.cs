using GeoCore;

namespace Geo.NurbsConstruction
{
    public static class EdgeMetaDataBuilder
    {
        private const double AlignTol = 1e-4;
        private const double RadiusTol = 1e-4;

        public static void Attach(AnchorMesh mesh)
        {
            if (mesh == null || mesh.GroupEdges == null || mesh.surfaceMetaData == null)
                return;

            foreach (var edge in mesh.GroupEdges)
            {
                if (edge.MetaData != null && edge.MetaData.HasCurve)
                    continue;

                string nameA = mesh.groupIdToExtendedName != null &&
                               mesh.groupIdToExtendedName.TryGetValue(edge.GroupIdA, out var na) ? na : null;
                string nameB = mesh.groupIdToExtendedName != null &&
                               mesh.groupIdToExtendedName.TryGetValue(edge.GroupIdB, out var nb) ? nb : null;
                mesh.surfaceMetaData.TryGetValue(nameA ?? "", out var metaA);
                mesh.surfaceMetaData.TryGetValue(nameB ?? "", out var metaB);

                Vec3D mid = EdgeMidpoint(edge);
                if (TryInfer(metaA, metaB, mid, edge, out var inferred) ||
                    TryInfer(metaB, metaA, mid, edge, out inferred))
                    edge.MetaData = inferred;
            }
        }

        private static Vec3D EdgeMidpoint(GroupEdge edge)
        {
            if (edge.LineStrips3D != null && edge.LineStrips3D.Count > 0)
            {
                var pts = edge.LineStrips3D[0].Points;
                if (pts != null && pts.Count > 0)
                    return pts[pts.Count / 2];
            }
            return default;
        }

        private static bool TryInfer(
            SurfaceMetaData a, SurfaceMetaData b, Vec3D mid, GroupEdge edge, out EdgeMetaData meta)
        {
            meta = null;
            if (a == null || b == null)
                return false;

            if (a.PlaneParams != null && b.CylinderParams != null)
                return TryPlaneCylinder(a.PlaneParams, b.CylinderParams, mid, out meta);
            if (a.PlaneParams != null && b.ConeParams != null)
                return TryPlaneCone(a.PlaneParams, b.ConeParams, mid, out meta);
            if (a.PlaneParams != null && b.SphereParams != null)
                return TryPlaneSphere(a.PlaneParams, b.SphereParams, mid, out meta);
            if (a.PlaneParams != null && b.TorusParams != null)
                return TryPlaneTorus(a.PlaneParams, b.TorusParams, mid, out meta);
            if (a.PlaneParams != null && b.PlaneParams != null)
                return TryPlanePlane(a.PlaneParams, b.PlaneParams, edge, out meta);
            return false;
        }

        private static bool TryPlaneCylinder(
            PlaneSurfaceParams plane, CylinderSurfaceParams cyl, Vec3D mid, out EdgeMetaData meta)
        {
            meta = null;
            var n = plane.Normal.Normalized();
            var axis = cyl.Axis.Normalized();
            if (Math.Abs(Vec3DOps.Dot(n, axis)) < 1.0 - AlignTol)
                return false;

            var center = cyl.Origin + axis * Vec3DOps.Dot(mid - cyl.Origin, axis);
            if (Math.Abs(Vec3DOps.Dot(center - plane.Origin, n)) > RadiusTol)
                center = plane.Origin + n * Vec3DOps.Dot(cyl.Origin - plane.Origin, n);

            if (Math.Abs((mid - center).Length() - cyl.Radius) > Math.Max(RadiusTol, cyl.Radius * 1e-3))
                return false;

            var refDir = Orthonormalize(n, cyl.RefDir);
            meta = EdgeMetaData.Circle(center, n, refDir, cyl.Radius);
            return true;
        }

        private static bool TryPlaneCone(
            PlaneSurfaceParams plane, ConeSurfaceParams cone, Vec3D mid, out EdgeMetaData meta)
        {
            meta = null;
            var n = plane.Normal.Normalized();
            var axis = cone.Axis.Normalized();
            if (Math.Abs(Vec3DOps.Dot(n, axis)) < 1.0 - AlignTol)
                return false;

            var center = cone.Origin + axis * Vec3DOps.Dot(mid - cone.Origin, axis);
            double along = Vec3DOps.Dot(center - cone.Origin, axis);
            double radius = cone.Radius + along * Math.Tan(cone.SemiAngle);
            if (radius < 1e-12)
                return false;
            if (Math.Abs((mid - center).Length() - radius) > Math.Max(RadiusTol, radius * 1e-3))
                return false;

            meta = EdgeMetaData.Circle(center, n, Orthonormalize(n, cone.RefDir), radius);
            return true;
        }

        private static bool TryPlaneSphere(
            PlaneSurfaceParams plane, SphereSurfaceParams sphere, Vec3D mid, out EdgeMetaData meta)
        {
            meta = null;
            var n = plane.Normal.Normalized();
            double dist = Vec3DOps.Dot(sphere.Center - plane.Origin, n);
            double r2 = sphere.Radius * sphere.Radius - dist * dist;
            if (r2 < 1e-12)
                return false;
            double radius = Math.Sqrt(r2);
            var center = sphere.Center - n * dist;
            if (Math.Abs((mid - center).Length() - radius) > Math.Max(RadiusTol, radius * 1e-3))
                return false;

            meta = EdgeMetaData.Circle(center, n, Orthonormalize(n, sphere.RefDir), radius);
            return true;
        }

        private static bool TryPlaneTorus(
            PlaneSurfaceParams plane, TorusSurfaceParams torus, Vec3D mid, out EdgeMetaData meta)
        {
            meta = null;
            var n = plane.Normal.Normalized();
            var axis = torus.Axis.Normalized();
            if (Math.Abs(Vec3DOps.Dot(n, axis)) < 1.0 - AlignTol)
                return false;

            var center = torus.Center + axis * Vec3DOps.Dot(plane.Origin - torus.Center, axis);
            double radial = (mid - center).Length();
            double r1 = torus.MajorRadius + torus.MinorRadius;
            double r2 = Math.Abs(torus.MajorRadius - torus.MinorRadius);
            double radius;
            if (Math.Abs(radial - r1) < Math.Abs(radial - r2))
                radius = r1;
            else
                radius = r2;
            if (Math.Abs(radial - radius) > Math.Max(RadiusTol, radius * 1e-3))
                return false;

            meta = EdgeMetaData.Circle(center, n, Orthonormalize(n, torus.RefDir), radius);
            return true;
        }

        private static bool TryPlanePlane(
            PlaneSurfaceParams a, PlaneSurfaceParams b, GroupEdge edge, out EdgeMetaData meta)
        {
            meta = null;
            var n1 = a.Normal.Normalized();
            var n2 = b.Normal.Normalized();
            var dir = Vec3DOps.Cross(n1, n2);
            if (dir.LengthSquared() < 1e-12)
                return false;
            dir = dir.Normalized();

            Vec3D p0 = EdgeEndpoint(edge, true);
            Vec3D p1 = EdgeEndpoint(edge, false);
            if ((p1 - p0).Length() < 1e-12)
                return false;

            meta = EdgeMetaData.Line(p0, p1);
            return Math.Abs(Vec3DOps.Dot((p1 - p0).Normalized(), dir)) > 1.0 - AlignTol;
        }

        private static Vec3D EdgeEndpoint(GroupEdge edge, bool start)
        {
            if (edge.LineStrips3D == null || edge.LineStrips3D.Count == 0)
                return default;
            var pts = edge.LineStrips3D[0].Points;
            if (pts == null || pts.Count == 0)
                return default;
            return start ? pts[0] : pts[pts.Count - 1];
        }

        private static Vec3D Orthonormalize(Vec3D axis, Vec3D refDir)
        {
            var r = refDir - axis * Vec3DOps.Dot(refDir, axis);
            if (r.LengthSquared() < 1e-16)
            {
                var hint = Math.Abs(axis.X) < 0.9 ? new Vec3D(1, 0, 0) : new Vec3D(0, 1, 0);
                r = Vec3DOps.Cross(axis, hint);
            }
            return r.Normalized();
        }
    }
}
