using Geo.BRep;
using Geo.NurbsConstruction;
using GeoCore;
using NURBS;

namespace Geo.Export
{
    /// <summary>
    /// Reads analytic STEP/IGES surface placement from patch metadata or NURBS payload.
    /// Topology and trim come from the mesh; surfaces are not fitted at export time.
    /// </summary>
    internal static class ExportAnalyticSurface
    {
        public static bool TryPlane(BRepFace face, out Vec3D origin, out Vec3D normal, out Vec3D refDir)
        {
            origin = default;
            normal = default;
            refDir = default;

            if (face.PlaneParams != null)
            {
                origin = face.PlaneParams.Origin;
                normal = face.PlaneParams.Normal;
                refDir = face.PlaneParams.RefDir;
                return true;
            }

            if (face.SurfaceType != SurfaceType.Planar && !face.UsesMeshFallback)
                return false;
            if (face.TessellationSurface == null)
                return false;

            return PlaneFromBilinear(face.TessellationSurface, out origin, out normal, out refDir);
        }

        public static bool TryCone(
            BRepFace face,
            out Vec3D origin,
            out Vec3D axis,
            out Vec3D refDir,
            out double radius,
            out double semiAngle)
        {
            origin = default;
            axis = default;
            refDir = default;
            radius = 0;
            semiAngle = 0;
            if (face.ConeParams == null)
                return false;
            origin = face.ConeParams.Origin;
            axis = face.ConeParams.Axis;
            refDir = face.ConeParams.RefDir;
            radius = face.ConeParams.Radius;
            semiAngle = face.ConeParams.SemiAngle;
            return true;
        }

        public static bool TrySphere(
            BRepFace face,
            out Vec3D center,
            out Vec3D axis,
            out Vec3D refDir,
            out double radius)
        {
            center = default;
            axis = default;
            refDir = default;
            radius = 0;
            if (face.SphereParams == null)
                return false;
            center = face.SphereParams.Center;
            axis = face.SphereParams.Axis;
            refDir = face.SphereParams.RefDir;
            radius = face.SphereParams.Radius;
            return true;
        }

        public static bool TryTorus(
            BRepFace face,
            out Vec3D center,
            out Vec3D axis,
            out Vec3D refDir,
            out double majorRadius,
            out double minorRadius)
        {
            center = default;
            axis = default;
            refDir = default;
            majorRadius = 0;
            minorRadius = 0;
            if (face.TorusParams == null)
                return false;
            center = face.TorusParams.Center;
            axis = face.TorusParams.Axis;
            refDir = face.TorusParams.RefDir;
            majorRadius = face.TorusParams.MajorRadius;
            minorRadius = face.TorusParams.MinorRadius;
            return true;
        }

        public static bool HasElementarySurface(BRepFace face)
        {
            return TryPlane(face, out _, out _, out _) ||
                   TryCylinder(face, out _, out _, out _, out _, out _) ||
                   TryCone(face, out _, out _, out _, out _, out _) ||
                   TrySphere(face, out _, out _, out _, out _) ||
                   TryTorus(face, out _, out _, out _, out _, out _);
        }

        public static bool TryCylinder(
            BRepFace face,
            out Vec3D origin,
            out Vec3D axis,
            out Vec3D refDir,
            out double radius,
            out double height)
        {
            origin = default;
            axis = default;
            refDir = default;
            radius = 0;
            height = 0;

            if (face.CylinderParams == null)
                return false;

            origin = face.CylinderParams.Origin;
            axis = face.CylinderParams.Axis;
            refDir = face.CylinderParams.RefDir;
            radius = face.CylinderParams.Radius;
            height = face.CylinderParams.Height;
            return true;
        }

        /// <summary>Mesh UV uses normalized arc length and height; STEP CYLINDRICAL_SURFACE uses radians and metric height.</summary>
        public static Vec2D CylinderMeshUvToStep(Vec2D meshUv, double height) =>
            new Vec2D(meshUv.X * 2.0 * Math.PI, meshUv.Y * height);

        /// <summary>
        /// IGES type 192 is the same map as STEP, but OpenCASCADE's IGES reader treats U as degrees.
        /// Writing radians (0–2π) becomes a ~6° sliver of the wall.
        /// </summary>
        public static Vec2D CylinderMeshUvToIges(Vec2D meshUv, double height) =>
            new Vec2D(meshUv.X * 360.0, meshUv.Y * height);

        /// <summary>PLANE / IGES 190 parametric coordinates: (refDir, normal×refDir) from origin.</summary>
        public static Vec2D PlaneWorldToParam(Vec3D point, Vec3D origin, Vec3D normal, Vec3D refDir)
        {
            var n = normal.LengthSquared() > 1e-24 ? normal.Normalized() : new Vec3D(0, 0, 1);
            var r = refDir - n * Vec3DOps.Dot(refDir, n);
            if (r.LengthSquared() < 1e-16)
            {
                var hint = Math.Abs(n.X) < 0.9 ? new Vec3D(1, 0, 0) : new Vec3D(0, 1, 0);
                r = Vec3DOps.Cross(n, hint);
            }
            r = r.Normalized();
            var y = Vec3DOps.Cross(n, r);
            var d = point - origin;
            return new Vec2D(Vec3DOps.Dot(d, r), Vec3DOps.Dot(d, y));
        }

        public static BRepTrimLoop LoopWithPlaneUv(
            BRepTrimLoop loop, Vec3D origin, Vec3D normal, Vec3D refDir)
        {
            var uv = new List<Vec2D>(loop.WorldPoints.Count);
            for (int i = 0; i < loop.WorldPoints.Count; i++)
                uv.Add(PlaneWorldToParam(loop.WorldPoints[i], origin, normal, refDir));
            return new BRepTrimLoop
            {
                WorldPoints = loop.WorldPoints,
                UvPoints = uv,
                IsOuter = loop.IsOuter
            };
        }

        public static Vec2D MeshUvToNative(BRepFace face, Vec2D meshUv)
        {
            if (face.Surface is MeshUvMappedSurface mapped)
                return mapped.MapMeshUv(meshUv.X, meshUv.Y);
            if (face.Surface is CapUvMappedSurface cap)
                return cap.MapMeshUv(meshUv.X, meshUv.Y);
            return meshUv;
        }

        public static BRepTrimLoop LoopOnNurbs(BRepFace face, BRepTrimLoop loop)
        {
            if (face.TessellationSurface == null || loop.UvPoints == null || loop.UvPoints.Count == 0)
                return loop;

            var uv = new List<Vec2D>(loop.UvPoints.Count);
            var world = new List<Vec3D>(loop.UvPoints.Count);
            for (int i = 0; i < loop.UvPoints.Count; i++)
            {
                Vec2D native = MeshUvToNative(face, loop.UvPoints[i]);
                uv.Add(native);
                world.Add(face.TessellationSurface.Evaluate(native.X, native.Y));
            }

            return new BRepTrimLoop
            {
                WorldPoints = world,
                UvPoints = uv,
                IsOuter = loop.IsOuter
            };
        }

        /// <summary>
        /// Map mesh UV onto the NURBS domain but keep tessellation world points.
        /// Re-evaluating the surface (LoopOnNurbs) invented 3D trims that missed the mesh.
        /// </summary>
        public static BRepTrimLoop LoopOnNurbsKeepWorld(BRepFace face, BRepTrimLoop loop)
        {
            if (loop.UvPoints == null || loop.UvPoints.Count == 0)
                return loop;

            var uv = new List<Vec2D>(loop.UvPoints.Count);
            for (int i = 0; i < loop.UvPoints.Count; i++)
                uv.Add(MeshUvToNative(face, loop.UvPoints[i]));

            return new BRepTrimLoop
            {
                WorldPoints = loop.WorldPoints,
                UvPoints = uv,
                IsOuter = loop.IsOuter
            };
        }

        /// <summary>Keep circumferential mesh-u continuous across the periodic seam.</summary>
        public static List<Vec2D> UnwrapCylinderMeshUv(IReadOnlyList<Vec2D> meshUv)
        {
            var result = new List<Vec2D>(meshUv.Count);
            double offset = 0;
            for (int i = 0; i < meshUv.Count; i++)
            {
                if (i > 0)
                {
                    double du = meshUv[i].X - meshUv[i - 1].X;
                    if (du < -0.5)
                        offset += 1.0;
                    else if (du > 0.5)
                        offset -= 1.0;
                }

                result.Add(new Vec2D(meshUv[i].X + offset, meshUv[i].Y));
            }

            return result;
        }

        public static bool PlaneFromBilinear(BSplineSurface surface, out Vec3D origin, out Vec3D normal, out Vec3D refDir)
        {
            origin = default;
            normal = default;
            refDir = default;
            if (surface.DegreeU != 1 || surface.DegreeV != 1 ||
                surface.NumControlPointsU != 2 || surface.NumControlPointsV != 2)
                return false;

            var cp = NurbsEntityConverter.FlattenSurfaceControlPoints(surface, out _);
            Vec3D p00 = cp[0];
            Vec3D p10 = cp[1];
            Vec3D p01 = cp[2];
            Vec3D xDir = p10 - p00;
            Vec3D yDir = p01 - p00;
            Vec3D n = Vec3DOps.Cross(xDir, yDir);
            if (n.Length() < 1e-12)
                return false;
            origin = p00;
            normal = n.Normalized();
            refDir = xDir.Normalized();
            return true;
        }
    }
}
