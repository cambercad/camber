using System.Globalization;
using System.Text.RegularExpressions;
using Geo;
using GeoCore;

namespace GeoTests;

public class UsdaExportTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Fact]
    public void SaveUsdaFile_Cube_WritesNormalsUvSubsetsAndVolume()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        var cube = api.CreateCube(CoordinateSystem.Default, 1.0, "cube");
        cube.Mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out var groups);
        Assert.Equal(positions.Count, normals.Count);
        Assert.Equal(positions.Count, uvs.Count);

        string path = Path.Combine(Path.GetTempPath(), $"csg_cube_{Guid.NewGuid():N}.usda");
        try
        {
            api.SaveUsdaFile(cube, path);
            string text = File.ReadAllText(path);
            Assert.StartsWith("#usda 1.0", text);
            Assert.Contains("upAxis = \"Z\"", text);
            Assert.Contains("subdivisionScheme = \"none\"", text);
            Assert.Contains("normal3f[] normals", text);
            Assert.Contains("texCoord2f[] primvars:st", text);
            Assert.Contains("interpolation = \"vertex\"", text);
            Assert.Contains("def GeomSubset", text);

            var filePoints = ParseVec3Array(text, "point3f[] points");
            var fileNormals = ParseVec3Array(text, "normal3f[] normals");
            var fileUv = ParseVec2Array(text, "texCoord2f[] primvars:st");
            var indices = ParseIntArray(text, "int[] faceVertexIndices");
            Assert.Equal(positions.Count, filePoints.Count);
            Assert.Equal(positions.Count, fileNormals.Count);
            Assert.Equal(positions.Count, fileUv.Count);
            Assert.Equal(triangles.Count * 3, indices.Count);

            var fileTris = new List<Tri>(triangles.Count);
            for (int i = 0; i < indices.Count; i += 3)
                fileTris.Add(new Tri(indices[i], indices[i + 1], indices[i + 2]));
            double vol = Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(filePoints, fileTris));
            Assert.InRange(vol, 0.99, 1.01);

            int subsets = Regex.Matches(text, @"def GeomSubset ").Count;
            Assert.Equal(6, subsets);
            Assert.DoesNotContain("def GeomSubset \"cube-", text);
        }
        finally
        {
            if (File.Exists(path))
                File.Delete(path);
        }
    }

    [Fact]
    public void UsdaWriter_SanitizesIdentifiers()
    {
        Assert.Equal("cube_ExtrudeTop", UsdaWriter.MakeIdentifier("cube-ExtrudeTop", "Mesh"));
        Assert.Equal("Mesh", UsdaWriter.MakeIdentifier("", "Mesh"));
        Assert.Equal("_12mm", UsdaWriter.MakeIdentifier("12mm", "Mesh"));
    }

    static List<Vec3D> ParseVec3Array(string text, string decl)
    {
        string body = SliceArrayBody(text, decl);
        var pts = new List<Vec3D>();
        foreach (Match m in Regex.Matches(body, @"\(([^,]+),\s*([^,]+),\s*([^)]+)\)"))
        {
            pts.Add(new Vec3D(
                double.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture),
                double.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture),
                double.Parse(m.Groups[3].Value, CultureInfo.InvariantCulture)));
        }
        return pts;
    }

    static List<Vec2D> ParseVec2Array(string text, string decl)
    {
        string body = SliceArrayBody(text, decl);
        var pts = new List<Vec2D>();
        foreach (Match m in Regex.Matches(body, @"\(([^,]+),\s*([^)]+)\)"))
        {
            pts.Add(new Vec2D(
                double.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture),
                double.Parse(m.Groups[2].Value, CultureInfo.InvariantCulture)));
        }
        return pts;
    }

    static List<int> ParseIntArray(string text, string decl)
    {
        string body = SliceArrayBody(text, decl);
        var values = new List<int>();
        foreach (Match m in Regex.Matches(body, @"-?\d+"))
            values.Add(int.Parse(m.Value, CultureInfo.InvariantCulture));
        return values;
    }

    static string SliceArrayBody(string text, string decl)
    {
        int start = text.IndexOf(decl, StringComparison.Ordinal);
        Assert.True(start >= 0, "missing " + decl);
        int eq = text.IndexOf('=', start);
        int open = text.IndexOf('[', eq);
        int close = text.IndexOf(']', open);
        Assert.True(eq >= 0 && open >= 0 && close > open);
        return text.Substring(open + 1, close - open - 1);
    }
}
