using GeoCore;
using NURBS;

namespace Geo.NurbsConstruction
{
    /// <summary>
    /// Attaches analytic metadata to imported triangle patches when the
    /// geometry is reliably planar, cylindrical, or spherical. Fitting a
    /// general B-spline is left to export as a last resort.
    /// </summary>
    public static class ImportPatchMetadataBuilder
    {
        public static void Attach(AnchorMesh mesh)
        {
            if (mesh == null || mesh.extendedNameToGroupId == null)
                return;
            if (mesh.surfaceMetaData == null)
                mesh.surfaceMetaData = new Dictionary<string, SurfaceMetaData>();

            foreach (var kv in mesh.extendedNameToGroupId)
            {
                if (mesh.surfaceMetaData.TryGetValue(kv.Key, out var existing) && existing.HasNurbs)
                    continue;
                if (!mesh.TryGetSurface(kv.Key, out var uvSurf) || uvSurf.Triangles.Count == 0)
                    continue;

                if (uvSurf.IsPlanar(1e-6))
                {
                    mesh.surfaceMetaData[kv.Key] = BuildPlanar(uvSurf);
                    continue;
                }

                if (TryCylinder(uvSurf, out var cylinder))
                    mesh.surfaceMetaData[kv.Key] = cylinder;
                else if (TrySphere(uvSurf, out var sphere))
                    mesh.surfaceMetaData[kv.Key] = sphere;
            }
        }

        private static SurfaceMetaData BuildPlanar(UVSurface patch)
        {
            var origin = patch.ApproximatePlanarSurfaceCenter(out var normal, out var tangentX, out _);
            var plane = new BSplinePlane(origin, normal, tangentX, Vec3DOps.Cross(normal, tangentX));
            return new SurfaceMetaData(SurfaceType.Planar, plane, ParametricRange.UnitSquare)
            {
                PlaneParams = new PlaneSurfaceParams
                {
                    Origin = origin,
                    Normal = normal,
                    RefDir = tangentX
                }
            };
        }

        private static bool TrySphere(UVSurface patch, out SurfaceMetaData meta)
        {
            meta = null;
            var pts = new List<Vec3D>();
            var seen = new HashSet<int>();
            for (int i = 0; i < patch.Triangles.Count; i++)
            {
                var tri = patch.Triangles[i];
                Add(seen, pts, patch.Points, tri.A);
                Add(seen, pts, patch.Points, tri.B);
                Add(seen, pts, patch.Points, tri.C);
            }
            if (!SphereFitter.FitSphere(pts, out var center, out double radius) || radius < 1e-9)
                return false;

            double maxErr = 0;
            for (int i = 0; i < pts.Count; i++)
            {
                double e = Math.Abs((pts[i] - center).Length() - radius);
                if (e > maxErr)
                    maxErr = e;
            }
            if (maxErr > Math.Max(1e-4, radius * 1e-3))
                return false;

            for (int i = 0; i < patch.Triangles.Count; i++)
            {
                var tri = patch.Triangles[i];
                var c = (patch.Points[tri.A] + patch.Points[tri.B] + patch.Points[tri.C]) * (1.0 / 3.0);
                if (Math.Abs((c - center).Length() - radius) > Math.Max(1e-4, radius * 1e-3))
                    return false;
            }

            var axis = new Vec3D(0, 0, 1);
            var refDir = new Vec3D(1, 0, 0);
            meta = new SurfaceMetaData(SurfaceType.Spherical, ParametricRange.UnitSquare)
            {
                SphereParams = new SphereSurfaceParams
                {
                    Center = center,
                    Axis = axis,
                    RefDir = refDir,
                    Radius = radius
                }
            };
            return true;
        }

        private static bool TryCylinder(UVSurface patch, out SurfaceMetaData meta)
        {
            meta = null;
            if (patch.Triangles.Count < 8)
                return false;

            var seen = new HashSet<int>();
            for (int i = 0; i < patch.Triangles.Count; i++)
            {
                var tri = patch.Triangles[i];
                seen.Add(tri.A);
                seen.Add(tri.B);
                seen.Add(tri.C);
            }
            if (seen.Count < 8)
                return false;

            var fallbackNormals = patch.Normals == null || patch.Normals.Count != patch.Points.Count
                ? UVSurface.ComputeAngleWeightedNormals(patch.Points, patch.Triangles)
                : patch.Normals;

            if (!CylinderAxisFit.TryFit(patch.Points, fallbackNormals, seen,
                    out var axis, out var pointOnAxis, out double radius, out double minH, out double maxH))
                return false;

            meta = new SurfaceMetaData(SurfaceType.Cylindrical, ParametricRange.UnitSquare)
            {
                CylinderParams = new CylinderSurfaceParams
                {
                    Origin = pointOnAxis + axis * minH,
                    Axis = axis,
                    RefDir = Vec3DOps.GetOrthoNormal(axis),
                    Radius = radius,
                    Height = maxH - minH
                }
            };
            return true;
        }

        private static void Add(HashSet<int> seen, List<Vec3D> pts, List<Vec3D> positions, int index)
        {
            if (seen.Add(index))
                pts.Add(positions[index]);
        }
    }
}
