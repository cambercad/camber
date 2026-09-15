using GeoCore;

namespace GeoTests;

internal static class MeshTestHelpers
{
    public static void AssertWatertight(IList<Vec3D> positions, IList<Tri> triangles)
    {
        Assert.True(
            MeshAnalysis.IsWatertightMesh(positions, triangles, out var problemEdges),
            $"Mesh is not watertight. {problemEdges.Count / 2} problematic edge(s).");
    }

    public static void AssertConsistentOrientation(IList<Vec3D> positions, IList<Tri> triangles)
    {
        Assert.True(
            MeshAnalysis.AreTrianglesConsistentlyOriented(triangles.ToList()),
            "Triangles are not consistently oriented.");
    }

    public static void AssertValidMesh(IList<Vec3D> positions, IList<Tri> triangles)
    {
        Assert.NotEmpty(positions);
        Assert.NotEmpty(triangles);
        AssertWatertight(positions, triangles);
        AssertConsistentOrientation(positions, triangles);
    }

    public static void AssertBoundingBox(
        IList<Vec3D> vertices, Vec3D expectedMin, Vec3D expectedMax, double tolerance)
    {
        var min = new Vec3D(double.MaxValue, double.MaxValue, double.MaxValue);
        var max = new Vec3D(double.MinValue, double.MinValue, double.MinValue);
        foreach (var v in vertices)
        {
            if (v.X < min.X) min.X = v.X;
            if (v.Y < min.Y) min.Y = v.Y;
            if (v.Z < min.Z) min.Z = v.Z;
            if (v.X > max.X) max.X = v.X;
            if (v.Y > max.Y) max.Y = v.Y;
            if (v.Z > max.Z) max.Z = v.Z;
        }

        Assert.InRange(min.X, expectedMin.X - tolerance, expectedMin.X + tolerance);
        Assert.InRange(min.Y, expectedMin.Y - tolerance, expectedMin.Y + tolerance);
        Assert.InRange(min.Z, expectedMin.Z - tolerance, expectedMin.Z + tolerance);
        Assert.InRange(max.X, expectedMax.X - tolerance, expectedMax.X + tolerance);
        Assert.InRange(max.Y, expectedMax.Y - tolerance, expectedMax.Y + tolerance);
        Assert.InRange(max.Z, expectedMax.Z - tolerance, expectedMax.Z + tolerance);
    }

    public static CoordinateConverter MakeConverter(double extent = 10.0)
    {
        return new CoordinateConverter(
            new Box3D(new Vec3D(-extent), new Vec3D(extent)));
    }

    public static List<Vec2D> MakeSquareContourCCW(double size = 1.0)
    {
        double h = size / 2.0;
        return new List<Vec2D>
        {
            new Vec2D(-h, -h),
            new Vec2D( h, -h),
            new Vec2D( h,  h),
            new Vec2D(-h,  h),
            new Vec2D(-h, -h)
        };
    }

    public static List<Vec2D> MakeCircleProfile(int segments, double radius, double centerX = 0, double centerY = 0)
    {
        var pts = new List<Vec2D>();
        for (int i = 0; i <= segments; i++)
        {
            double t = 2.0 * Math.PI * i / segments;
            pts.Add(new Vec2D(
                centerX + radius * Math.Cos(t),
                centerY + radius * Math.Sin(t)));
        }
        return pts;
    }

    public static List<Vec2D> MakeSemicircleProfile(int segments, double radius)
    {
        var pts = new List<Vec2D>();
        for (int i = 0; i <= segments; i++)
        {
            double t = Math.PI * i / segments;
            pts.Add(new Vec2D(
                radius * Math.Cos(t),
                radius * Math.Sin(t)));
        }
        return pts;
    }
}
