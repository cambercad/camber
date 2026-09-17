using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class RoundedSlopeSupportTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);


    [Theory]
    [InlineData(false, 1, 4, false, 0)]
    [InlineData(true, 1, 2, false, 0)]
    [InlineData(true, 2, 2, false, 0)]
    [InlineData(true, 3, 2, false, 0)]
    [InlineData(true, 4, 2, false, 0)]
    [InlineData(true, 1, 3, false, 0)]
    [InlineData(true, 2, 3, false, 0)]
    [InlineData(true, 3, 3, false, 0)]
    [InlineData(true, 1, 4, false, 0)]
    [InlineData(true, 2, 4, false, 0)]
    [InlineData(true, 1, 2, true, 0)]
    [InlineData(true, 2, 2, true, 0)]
    [InlineData(true, 3, 2, true, 0)]
    [InlineData(true, 4, 2, true, 0)]
    [InlineData(true, 4, 2, false, .4)]
    [InlineData(true, 4, 2, true, .4)]
    public void RoofTrimmedPostsPreserveManifoldShell(bool selfOnly, int nx, int ny, bool ridge, double angle)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-160, -160, -20), new Vec3D(160, 160, 160)), .02);
        AnchorMesh Extrude(string name, double width, params Vec2D[] points)
        {
            var frame = new CoordinateSystem(new Vec3D(-width / 2, 0, 0),
                new Vec3D(-Math.Sin(angle), Math.Cos(angle), 0), new Vec3D(0, 0, 1),
                new Vec3D(Math.Cos(angle), Math.Sin(angle), 0));
            var sketch = new PlotterSketcherCoordSys(name, frame, points[0]);
            for (int i = 1; i <= points.Length; i++)
                sketch.AppendLine(points[i % points.Length].X, points[i % points.Length].Y);
            return api.Extrude(sketch, width, name: name);
        }
        double end = ny * 4 - .1, width = nx * 8 - .2;
        double shoulder = 8 - end;
        double height = 3 * 3.2, gradient = (height - 3.2) / (end - shoulder);
        var blank = ridge
            ? Extrude("outer", width, new(-end, 0), new(end, 0), new(end, 3.2), new(0, height), new(-end, 3.2))
            : Extrude("outer", width, new(-end, 0), new(end, 0), new(end, 3.2), new(shoulder, height), new(-end, height));
        blank.EnsureCoplanarPostProcessed();
        blank = api.Fillet(blank, blank.GroupEdges.Select(edge => edge.Name).ToList(), .08, .01, "rounded");
        AssertCornerPatchesDoNotFold(blank);
        if (selfOnly)
        {
            var self = api.Boolean(blank, blank, BooleanOp.Intersect, "self");
            self.EnsureCoplanarPostProcessed();
            Assert.True(MeshAnalysis.IsWatertightMesh(self.Mesh.PrecisionPositions, self.Mesh.Triangles));
            return;
        }
        double drop = 1.2 * Math.Sqrt(1 + gradient * gradient);
        double front = 15.9 - 1.2, rear = -15.9 + 1.2;
        var cavity = Extrude("cavity", 7.8 - 2 * 1.2, new(rear, -.1), new(front, -.1),
            new(front, height - gradient * (front + 7.9) - drop),
            new(-7.9 + (1.2 - drop) / gradient, height - 1.2), new(rear, height - 1.2));
        var shell = api.Boolean(blank, cavity, BooleanOp.Difference, "shell");
        var post = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, -8, .15)), 1.6, height - .15, name: "post");
        var posts = api.PatternLinear(post, 3, new Vec3D(0, 8, 0), "posts");
        var supported = api.Boolean(shell, api.BatchUnion(posts.ToList()), BooleanOp.Union, "supported");
        var trimmed = api.Boolean(supported, blank, BooleanOp.Intersect, "trimmed");
        trimmed.EnsureCoplanarPostProcessed();
        Assert.True(MeshAnalysis.IsWatertightMesh(trimmed.Mesh.PrecisionPositions, trimmed.Mesh.Triangles));
    }
    private static void AssertCornerPatchesDoNotFold(AnchorMesh mesh)
    {
        var corners = mesh.groupIdToExtendedName.Where(pair => pair.Value.StartsWith("BlendCorner_", StringComparison.Ordinal)).ToList();
        Assert.NotEmpty(corners);
        foreach (var corner in corners)
        {
            var normals = mesh.Mesh.Triangles.Select((triangle, index) => (triangle, index))
                .Where(item => mesh.Mesh.TrianglesEx[item.index].GroupId == corner.Key)
                .Select(item => Rat3Hybrid.Cross(
                    mesh.Mesh.PrecisionPositions[item.triangle.B] - mesh.Mesh.PrecisionPositions[item.triangle.A],
                    mesh.Mesh.PrecisionPositions[item.triangle.C] - mesh.Mesh.PrecisionPositions[item.triangle.A])).ToList();
            var projectionNormal = new Rat3Hybrid(0, 0, 0);
            foreach (var normal in normals)
            {
                projectionNormal += normal;
                projectionNormal.Simplify();
            }
            // These convex corner disks fit in one hemisphere. Every projected
            // triangle must retain the same winding; a folded strip reverses it.
            Assert.All(normals, normal => Assert.True(Rat3Hybrid.Dot(normal, projectionNormal).Sign() > 0));
        }
    }

}
