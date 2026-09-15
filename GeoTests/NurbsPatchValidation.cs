using Geo;
using Geo.NurbsConstruction;
using GeoCore;
using NURBS;

namespace GeoTests;

/// <summary>
/// Test-only helpers for validating triangle patches against attached NURBS surfaces.
/// </summary>
internal static class NurbsPatchValidation
{
    public sealed class TessellatedNurbsBvh
    {
        public List<Vec3D> Positions { get; }
        public List<Tri> Triangles { get; }
        public BVHNode[] Tree { get; }

        public TessellatedNurbsBvh(List<Vec3D> positions, List<Tri> triangles, BVHNode[] tree)
        {
            Positions = positions;
            Triangles = triangles;
            Tree = tree;
        }
    }

    public static TessellatedNurbsBvh TessellateForValidation(BSplineSurface surface, double maxDeviation = 0.01)
    {
        int n = Math.Clamp((int)Math.Ceiling(2.0 / maxDeviation), 4, 64);
        var geo = UniformGridTessellate(surface, n);
        var tree = Tree.BuildTreeFast(i =>
        {
            var tri = geo.Triangles[i];
            var a = geo.Points[tri.A];
            var b = geo.Points[tri.B];
            var c = geo.Points[tri.C];
            return new Box3F(
                new Vec3F((float)a.X, (float)a.Y, (float)a.Z),
                new Vec3F((float)b.X, (float)b.Y, (float)b.Z),
                new Vec3F((float)c.X, (float)c.Y, (float)c.Z));
        }, geo.Triangles.Count);

        return new TessellatedNurbsBvh(geo.Points, geo.Triangles, tree);
    }

    private static TriangulatedGeometry UniformGridTessellate(BSplineSurface surface, int segments)
    {
        var points = new List<Vec3D>((segments + 1) * (segments + 1));
        var uv = new List<Vec2D>((segments + 1) * (segments + 1));
        for (int i = 0; i <= segments; i++)
        {
            double u = (double)i / segments;
            for (int j = 0; j <= segments; j++)
            {
                double v = (double)j / segments;
                points.Add(surface.Evaluate(u, v));
                uv.Add(new Vec2D(u, v));
            }
        }

        var triangles = new List<Tri>();
        int stride = segments + 1;
        for (int i = 0; i < segments; i++)
        {
            for (int j = 0; j < segments; j++)
            {
                int i0 = i * stride + j;
                int i1 = i0 + 1;
                int i2 = i0 + stride;
                int i3 = i2 + 1;
                triangles.Add(new Tri(i0, i1, i2));
                triangles.Add(new Tri(i1, i3, i2));
            }
        }

        var normals = new List<Vec3D>(points.Count);
        for (int i = 0; i < points.Count; i++)
            normals.Add(surface.EvaluateNormal(uv[i].X, uv[i].Y));

        return new TriangulatedGeometry(triangles, points, normals, uv);
    }

    public static double MaxDistanceToSurface(IEnumerable<Vec3D> patchVertices, TessellatedNurbsBvh bvh)
    {
        var controller = new ClosestDistanceToTrimeshTraversalController(
            bvh.Triangles, bvh.Positions, bvh.Tree, new Vec3D(0, 0, 0));

        double maxDist = 0;
        foreach (var p in patchVertices)
        {
            controller.Update(p);
            Tree.Traverse(bvh.Tree, controller);
            if (!controller.Success)
                throw new InvalidOperationException("BVH closest-point query failed.");
            double d = Math.Sqrt(controller.ClosestDistanceSquared);
            if (d > maxDist)
                maxDist = d;
        }
        return maxDist;
    }

    public static void AssertUvConsistent(UVSurface patch, INurbsSurface nurbs, ParametricRange? paramRange, double tolerance)
    {
        if (tolerance < 0)
            return;
        var seen = new HashSet<(int, Vec2D)>();

        for (int ti = 0; ti < patch.Triangles.Count; ti++)
        {
            var tri = patch.Triangles[ti];
            CheckCorner(tri.A, patch.Uv[tri.A], patch.Points[tri.A]);
            CheckCorner(tri.B, patch.Uv[tri.B], patch.Points[tri.B]);
            CheckCorner(tri.C, patch.Uv[tri.C], patch.Points[tri.C]);
        }

        void CheckCorner(int idx, Vec2D meshUv, Vec3D pos)
        {
            var key = (idx, meshUv);
            if (!seen.Add(key))
                return;

            Vec3D onSurface = nurbs.Evaluate(meshUv.X, meshUv.Y);
            double dist = (onSurface - pos).Length();
            Assert.True(dist <= tolerance,
                $"UV evaluate mismatch at vertex {idx}: distance {dist} > {tolerance} (meshUv={meshUv})");
        }
    }

    public static void AssertPatchOnNurbsSurface(
        AnchorMesh mesh, string patchName, double distanceTolerance, double uvTolerance, double tessellationDeviation = 0.01)
    {
        Assert.True(mesh.TryGetSurface(patchName, out var patch), $"Patch '{patchName}' not found.");
        Assert.True(mesh.surfaceMetaData.TryGetValue(patchName, out var meta), $"No metadata for '{patchName}'.");
        Assert.NotNull(meta.NurbsSurface);

        var nurbs = MeshUvMappedSurface.ResolveTessellationTarget(meta.NurbsSurface);

        var bvh = TessellateForValidation(nurbs, tessellationDeviation);

        var vertices = new HashSet<int>();
        foreach (var tri in patch.Triangles)
        {
            vertices.Add(tri.A);
            vertices.Add(tri.B);
            vertices.Add(tri.C);
        }

        double maxDist = MaxDistanceToSurface(vertices.Select(i => patch.Points[i]), bvh);
        Assert.True(maxDist <= distanceTolerance,
            $"Patch '{patchName}' max distance to NURBS {maxDist} > {distanceTolerance}");

        if (uvTolerance >= 0)
            AssertUvConsistent(patch, meta.NurbsSurface, meta.ParamRange, uvTolerance);
    }

    public static IEnumerable<string> AllNamedPatches(AnchorMesh mesh)
    {
        return mesh.extendedNameToGroupId.Keys;
    }
}
