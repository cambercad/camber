using System.Globalization;
using Geo;
using GeoCore;

namespace GeoTests;

public class ObjExportTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void SolidAndAssemblyExportKeepUvAndNormalSeams()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(40)), .01);
        var block = api.CreateCuboid(new Vec3D(0), new Vec3D(1, 2, 3), "block");
        var assembly = api.GetAssembly("two_blocks");
        assembly.AddPart(block, new Vec3D(0));
        assembly.AddPart(block, new Vec3D(5, 0, 0));
        string solidPath = Path.Combine(Path.GetTempPath(), "camber_obj_solid_" + Guid.NewGuid() + ".obj");
        string assemblyPath = Path.Combine(Path.GetTempPath(), "camber_obj_assembly_" + Guid.NewGuid() + ".obj");
        try
        {
            api.SaveWavefrontObjFile(block, solidPath);
            ObjWriter.WriteAssembly(assemblyPath, assembly.GetLeaves());
            CheckAttributes(File.ReadAllLines(solidPath), 1);
            CheckAttributes(File.ReadAllLines(assemblyPath), 2);
            var imported = api.LoadWavefrontObjFile(solidPath, name: "roundtrip");
            Assert.True(imported.IsVolume);
            imported.Mesh.Decompose(out var positions, out var normals, out var uvs,
                out var triangles, out _);
            Assert.Equal(positions.Count, normals.Count);
            Assert.Equal(positions.Count, uvs.Count);
            Assert.Equal(block.Mesh.Triangles.Count, triangles.Count);
        }
        finally
        {
            File.Delete(solidPath);
            File.Delete(assemblyPath);
        }
    }

    [Fact]
    public void ImportRetainsUvOnlyCornersAndNegativePolygonIndices()
    {
        string path = Path.Combine(Path.GetTempPath(), "camber_obj_uv_" + Guid.NewGuid() + ".obj");
        try
        {
            File.WriteAllText(path, "o plate\ng top\nv 0 0 0\nv 2 0 0\nv 2 2 0\nv 0 2 0\n" +
                "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nf -4/-4 -3/-3 -2/-2 -1/-1\n");
            var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(5)), .01);
            var mesh = api.LoadWavefrontObjFile(path, name: "uv_plate");
            mesh.Mesh.Decompose(out _, out _, out var uvs, out var tris, out _);
            Assert.Equal(2, tris.Count);
            Assert.Contains(uvs, uv => uv.X == 1 && uv.Y == 1);
            Assert.Contains("plate/top", mesh.groupIdToExtendedName.Values);
        }
        finally { File.Delete(path); }
    }

    [Fact]
    public void ImportRetainsExplicitNormalsWithoutUv()
    {
        string path = Path.Combine(Path.GetTempPath(), "camber_obj_normals_" + Guid.NewGuid() + ".obj");
        try
        {
            File.WriteAllText(path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1\n");
            var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(5)), .01);
            var mesh = api.LoadWavefrontObjFile(path, name: "normal_plate");
            mesh.Mesh.Decompose(out _, out var normals, out var uvs, out var tris, out _);
            Assert.Single(tris);
            Assert.All(normals, n => Assert.True(n.Z > .999));
            Assert.Equal(normals.Count, uvs.Count);
        }
        finally { File.Delete(path); }
    }

    private static void CheckAttributes(string[] lines, int instances)
    {
        var positions = new List<string>();
        var normals = new List<string>();
        var uvs = new List<string>();
        int uvCount = 0, faceCount = 0, objects = 0;
        foreach (string line in lines)
        {
            if (line.StartsWith("v ")) positions.Add(line[2..]);
            else if (line.StartsWith("vt ")) { uvCount++; uvs.Add(line[3..]); }
            else if (line.StartsWith("vn ")) normals.Add(line[3..]);
            else if (line.StartsWith("o ")) objects++;
            else if (line.StartsWith("f "))
            {
                faceCount++;
                var face = line[2..].Split(' ', StringSplitOptions.RemoveEmptyEntries);
                var indicesForFace = new int[face.Length];
                int cornerNumber = 0;
                foreach (string corner in line[2..].Split(' ', StringSplitOptions.RemoveEmptyEntries))
                {
                    var indices = corner.Split('/');
                    Assert.Equal(3, indices.Length);
                    Assert.Equal(indices[0], indices[1]);
                    Assert.Equal(indices[0], indices[2]);
                    int index = int.Parse(indices[0], CultureInfo.InvariantCulture);
                    Assert.InRange(index, 1, positions.Count);
                    indicesForFace[cornerNumber++] = index - 1;
                }
                Vec3D a = Point(indicesForFace[0]), b = Point(indicesForFace[1]), c = Point(indicesForFace[2]);
                Vec3D geometric = Vec3DOps.Cross(b - a, c - a);
                Vec3D authored = Normal(indicesForFace[0]);
                Assert.True(Vec3DOps.Dot(geometric, authored) > 0, "OBJ normal must follow face winding.");
            }
        }
        Assert.Equal(instances == 1 ? 0 : instances, objects);
        Assert.True(faceCount > 0);
        Assert.Equal(positions.Count, uvCount);
        Assert.Equal(positions.Count, normals.Count);
        Assert.True(new HashSet<string>(uvs).Count > 1, "UV coordinates must not all be identical.");
        var normalAtPosition = new Dictionary<string, string>();
        var uvAtPosition = new Dictionary<string, string>();
        bool sharpSeam = false, uvSeam = false;
        for (int i = 0; i < positions.Count; i++)
        {
            if (normalAtPosition.TryGetValue(positions[i], out string prior) && prior != normals[i])
                sharpSeam = true;
            else
                normalAtPosition[positions[i]] = normals[i];
            if (uvAtPosition.TryGetValue(positions[i], out string priorUv) && priorUv != uvs[i])
                uvSeam = true;
            else
                uvAtPosition[positions[i]] = uvs[i];
        }
        Assert.True(sharpSeam, "A sharp cube corner needs distinct normals at the same position.");
        Assert.True(uvSeam, "A cube corner needs independent UV coordinates for adjacent faces.");

        Vec3D Point(int index) => Parse3(positions[index]);
        Vec3D Normal(int index) => Parse3(normals[index]);
        static Vec3D Parse3(string line)
        {
            var fields = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            return new Vec3D(double.Parse(fields[0], CultureInfo.InvariantCulture),
                double.Parse(fields[1], CultureInfo.InvariantCulture),
                double.Parse(fields[2], CultureInfo.InvariantCulture));
        }
    }
}
