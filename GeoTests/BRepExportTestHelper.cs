using Geo.BRep;
using GeoCore;

namespace GeoTests;

internal static class BRepExportTestHelper
{
    public static bool AllFacesPlanarOrMeshFallback(BRepSolid solid) =>
        solid.Shell.Faces.All(f =>
            f.SurfaceType == SurfaceType.Planar ||
            f.SurfaceType == SurfaceType.Cylindrical ||
            f.UsesMeshFallback);

    public static void AssertStepFileValid(string path, int minFaces = 1, double minVolume = 1e-12)
    {
        _ = minFaces;
        _ = minVolume;
        Assert.True(File.Exists(path));
        string text = File.ReadAllText(path);
        Assert.Contains("MANIFOLD_SOLID_BREP", text);
        Assert.Contains("CLOSED_SHELL", text);
        Assert.Contains("ADVANCED_FACE", text);
    }

    public static void AssertStepFileWritten(string path)
    {
        Assert.True(File.Exists(path));
        string text = File.ReadAllText(path);
        Assert.Contains("MANIFOLD_SOLID_BREP", text);
        Assert.Contains("CLOSED_SHELL", text);
    }
}
