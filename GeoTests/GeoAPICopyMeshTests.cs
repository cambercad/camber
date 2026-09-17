using CSG;
using Geo;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public class GeoAPICopyMeshTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    private static double AbsMeshVolume(AnchorMesh anchor)
    {
        return Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(anchor.Mesh.Positions, anchor.Mesh.Triangles));
    }

    [Fact]
    public void CopyMesh_DecoratesSurfaceNames_PrefixAndSuffix()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(5)), 1e-4);
        var fromCs = new CoordinateSystem(new Vec3D(0, 0, 0));
        var box = api.CreateCuboid(fromCs, new Vec3D(1, 1, 1), "box");

        var toCs = new CoordinateSystem(new Vec3D(0.1, 0, 0));
        var copyP = api.CopyMesh(box, fromCs, toCs, "P_", SurfacePatchNameAffix.Prefix, "boxP");
        foreach (var name in copyP.groupIdToExtendedName.Values)
            Assert.StartsWith("P_", name);

        var copyS = api.CopyMesh(box, fromCs, toCs, "_S", SurfacePatchNameAffix.Suffix, "boxS");
        foreach (var name in copyS.groupIdToExtendedName.Values)
            Assert.EndsWith("_S", name);
    }

    [Fact]
    public void CopyMesh_TranslationOnly_PreservesLatticeOffsetPerVertex()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-1), new Vec3D(10)), 1e-4);
        double cell = api.Converter.SmallestUnit();
        int kx = 4, ky = -2, kz = 7;
        var fromCs = new CoordinateSystem(new Vec3D(0, 0, 0));
        var box = api.CreateCuboid(fromCs, new Vec3D(1, 1, 1), "box");
        var toCs = new CoordinateSystem(new Vec3D(kx * cell, ky * cell, kz * cell));
        Mat4D tf = CoordinateSystem.GetTransform(fromCs, toCs);
        Int3 delta = api.Converter.ConvertDirection(new Vec3D(tf.M14, tf.M24, tf.M34));

        var copy = api.CopyMesh(box, fromCs, toCs, "n", SurfacePatchNameAffix.Prefix, "boxCopy");

        Assert.Equal(box.Mesh.PrecisionPositions.Count, copy.Mesh.PrecisionPositions.Count);
        static string LatticeKey(in Rat3Hybrid p) =>
            $"{p.X.ToDouble()},{p.Y.ToDouble()},{p.Z.ToDouble()}";
        var expected = new HashSet<string>();
        var shift = new Rat3Hybrid(delta.X, delta.Y, delta.Z);
        for (int i = 0; i < box.Mesh.PrecisionPositions.Count; i++)
            expected.Add(LatticeKey(box.Mesh.PrecisionPositions[i] + shift));
        var actual = new HashSet<string>();
        for (int i = 0; i < copy.Mesh.PrecisionPositions.Count; i++)
            actual.Add(LatticeKey(copy.Mesh.PrecisionPositions[i]));
        Assert.Equal(expected.Count, actual.Count);
        Assert.True(expected.SetEquals(actual));
    }

    [Fact]
    public void CopyMesh_PreservesAbsVolume_Translation()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(5)), 1e-4);
        var fromCs = new CoordinateSystem(new Vec3D(0, 0, 0));
        var box = api.CreateCuboid(fromCs, new Vec3D(1, 1, 1), "box");
        double vBox = AbsMeshVolume(box);

        double cell = api.Converter.SmallestUnit();
        var toTranslate = new CoordinateSystem(new Vec3D(3 * cell, 5 * cell, -2 * cell));
        var copyT = api.CopyMesh(box, fromCs, toTranslate, "t", SurfacePatchNameAffix.Prefix, "boxT");
        Assert.True(Math.Abs(vBox - AbsMeshVolume(copyT)) < 2e-5, $"volume translation copy: {vBox} vs {AbsMeshVolume(copyT)}");
    }

    [Fact]
    public void CopyMesh_PreservesAbsVolume_Rotation_Cylinder()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(3)), 1e-4);
        var fromCs = new CoordinateSystem(new Vec3D(0, 0, 0));
        var cyl = api.CreateCylinder(fromCs, 0.5, 1.0, 0.0001, "cyl");
        double vCyl = AbsMeshVolume(cyl);
        var toRot = new CoordinateSystem(
            new Vec3D(0, 0, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(-1, 0, 0),
            new Vec3D(0, 0, 1));
        var copyR = api.CopyMesh(cyl, fromCs, toRot, "r", SurfacePatchNameAffix.Prefix, "cylR");
        Assert.True(Math.Abs(vCyl - AbsMeshVolume(copyR)) < 2e-4, $"volume rotation copy: {vCyl} vs {AbsMeshVolume(copyR)}");
    }

    [Fact]
    public void RationalApproximation_ApproximateDouble_IsClose()
    {
        var r = RationalApproximation.ApproximateDouble(0.5, 10_000);
        Assert.True(Math.Abs(r.ToDouble() - 0.5) < 1e-12);
    }

    [Fact]
    public void CopyMesh_BooleanUnion_DistinctAffix_DoesNotThrowNameConflict()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(6)), 1e-4);
        var fromCs = new CoordinateSystem(new Vec3D(0, 0, 0));
        var box = api.CreateCuboid(fromCs, new Vec3D(1, 1, 1), "box");
        double cell = api.Converter.SmallestUnit();
        var toCs = new CoordinateSystem(new Vec3D(2 * cell, 0, 0));
        var copy = api.CopyMesh(box, fromCs, toCs, "C_", SurfacePatchNameAffix.Prefix, "boxCopy");

        var merged = api.Boolean(box, copy, BooleanOp.Union, "merged");
        Assert.NotNull(merged);
        Assert.NotEmpty(merged.Mesh.Triangles);
    }

    [Fact]
    public void CopyMeshAsInstance_RequiresNewName_AndRewritesPrefixes()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(5)), 1e-4);
        var box = api.CreateCuboid(new CoordinateSystem(new Vec3D(0, 0, 0)), new Vec3D(1, 1, 1), "box");

        Assert.Throws<ArgumentException>(() => api.CopyMeshAsInstance(box, ""));
        Assert.Throws<ArgumentException>(() => api.CopyMeshAsInstance(box, "box"));

        var copy = api.CopyMeshAsInstance(box, "box2");
        Assert.Equal("box2", copy.Name);
        var copyNames = copy.groupIdToExtendedName.Values.ToList();
        foreach (string sourceName in box.groupIdToExtendedName.Values)
        {
            string want = EntityNaming.RewriteMeshNameInEntity(sourceName, "box", "box2");
            Assert.Contains(want, copyNames);
        }
        foreach (string name in copyNames)
            Assert.False(name == "box" || (name.StartsWith("box-", StringComparison.Ordinal) && !name.StartsWith("box2-", StringComparison.Ordinal)), name);
        if (copy.GroupEdges != null)
        {
            foreach (var edge in copy.GroupEdges)
            {
                string n = edge.Name;
                Assert.False(n.Contains("[box,") || n.Contains(",box]") || n.Contains("[box-") || n.Contains(",box-"), n);
            }
        }
    }

    [Fact]
    public void CopyMeshAsInstance_PreservesExactTriangulationAndCornerData()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(3)), 1e-4);
        AnchorMesh source = api.CreateCylinder(
            new CoordinateSystem(new Vec3D(0, 0, 0)), 0.5, 1.0, 0.01, "pipe");

        AnchorMesh copy = api.CopyMeshAsInstance(source, "pipe2");

        Assert.Equal(source.Mesh.Positions, copy.Mesh.Positions);
        Assert.Equal(source.Mesh.PrecisionPositions, copy.Mesh.PrecisionPositions);
        Assert.Equal(source.Mesh.Triangles, copy.Mesh.Triangles);
        Assert.Equal(source.Mesh.TrianglesEx.Count, copy.Mesh.TrianglesEx.Count);
        for (int i = 0; i < source.Mesh.TrianglesEx.Count; i++)
        {
            MeshTriangle<TriangleVertexNormalUV> a = source.Mesh.TrianglesEx[i];
            MeshTriangle<TriangleVertexNormalUV> b = copy.Mesh.TrianglesEx[i];
            Assert.Equal(a.V0.Normal, b.V0.Normal);
            Assert.Equal(a.V1.Normal, b.V1.Normal);
            Assert.Equal(a.V2.Normal, b.V2.Normal);
            Assert.Equal(a.V0.UV, b.V0.UV);
            Assert.Equal(a.V1.UV, b.V1.UV);
            Assert.Equal(a.V2.UV, b.V2.UV);
            Assert.NotEqual(a.GroupId, b.GroupId);
        }
    }
}
