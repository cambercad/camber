using Curves;
using Geo;
using Geo.NurbsConstruction;
using GeoCore;

namespace GeoTests;

public class PolygonLoftSupportTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    private static readonly (double Width, double Height)[] Sections = [(2, 4), (5, 3), (3, 7)];

    private static Vec3D ProfilePoint(int station, double u)
    {
        var (width, height) = Sections[station];
        double distance = u * 2 * (width + height);
        Vec2D point;
        if (distance <= width) point = new Vec2D(distance, 0);
        else if ((distance -= width) <= height) point = new Vec2D(width, distance);
        else if ((distance -= height) <= width) point = new Vec2D(width - distance, height);
        else point = new Vec2D(0, height - (distance - width));
        return new Vec3D(point.X, point.Y, station * 10);
    }

    [Theory]
    [InlineData(LoftStyle.Ruled, 0)]
    [InlineData(LoftStyle.Ruled, .137)]
    [InlineData(LoftStyle.SmoothCatmullRom, 0)]
    [InlineData(LoftStyle.SmoothCatmullRom, .137)]
    [InlineData(LoftStyle.Hermite, 0)]
    [InlineData(LoftStyle.Hermite, .137)]
    public void PolygonSupportAgreesWithMeshCornersAndBetweenConstructionSamples(LoftStyle style, double seam)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(30)), .005);
        var sketches = new List<PlotterSketcherCoordSys>();
        for (int station = 0; station < Sections.Length; station++)
        {
            var frame = new CoordinateSystem(new Vec3D(0, 0, station * 10),
                new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
            var sketch = new PlotterSketcherCoordSys("profile_" + station, frame, new Vec2D(0, 0));
            sketch.AddRectangleFromCorners(new Vec2D(0, 0), new Vec2D(Sections[station].Width, Sections[station].Height));
            sketches.Add(sketch);
        }
        var options = new LoftOptions
        {
            Style = style, AlignmentMode = LoftAlignmentMode.AsAuthored,
            ProfileSamplesU = 7, CreasePolicy = LoftCreasePolicy.None
        };
        var resolvedSeams = new double[3];
        if (seam != 0)
            options.ProfileSeamPoints = Enumerable.Range(0, 3).Select(station =>
            {
                var point = ProfilePoint(station, seam);
                var hint = new Vec2D(point.X, point.Y);
                var (width, height) = Sections[station];
                List<Vec2D> perimeter = [new(0, 0), new(width, 0), new(width, height), new(0, height)];
                // Seam hints currently use an approximate closest-point search.
                // Test support against that resolved construction seam, without
                // changing its geometry or assuming exact hint placement.
                resolvedSeams[station] = LoftBuilder.ClosestPointNormalizedU(perimeter,
                    perimeter.Select(_ => new Vec2D(0, 1)).ToList(), true, hint);
                return LoftProfileSeamHint.FromPoint(hint);
            }).ToList();
        var body = api.Loft(sketches, options, "polygon_loft", .005);
        var metadata = body.surfaceMetaData["polygon_loft-Side"];
        Assert.False(metadata.IsNurbsMaterialized);
        var support = metadata.NurbsSurface;
        // Export must resolve the same construction, not an unrelated fallback fit.
        var exported = MeshUvMappedSurface.ResolveTessellationTarget(support);
        int group = body.extendedNameToGroupId["polygon_loft-Side"];
        double latticeBound = 2 * Math.Sqrt(3) * api.Converter.SmallestUnit();
        for (int i = 0; i < body.Mesh.Triangles.Count; i++)
        {
            var corners = body.Mesh.TrianglesEx[i];
            if (corners.GroupId != group) continue;
            var triangle = body.Mesh.Triangles[i];
            var indices = new[] { triangle.A, triangle.B, triangle.C };
            var uv = new[] { corners.V0.UV, corners.V1.UV, corners.V2.UV };
            for (int j = 0; j < 3; j++)
                Assert.InRange((support.Evaluate(uv[j].X, uv[j].Y) - body.Mesh.Positions[indices[j]]).Length(), 0, latticeBound);
        }

        options.ProfileSamplesU = 19;
        var refined = api.Loft(sketches, options, "refined_polygon_loft", .002);
        var refinedSupport = refined.surfaceMetaData["refined_polygon_loft-Side"].NurbsSurface;

        // These coordinates are deliberately between the emitted U columns and V
        // rows. The witness follows the authored rectangular perimeter and an
        // independent existing curve primitive through the three section points.
        foreach (double u in new[] { .0713, .2137, .3971, .6231, .8813 })
        foreach (double v in new[] { .071, .193, .417, .613, .887 })
        {
            var points = Enumerable.Range(0, 3).Select(station => ProfilePoint(station, (u + resolvedSeams[station]) % 1)).ToArray();
            int span = v < .5 ? 0 : 1;
            double fraction = 2 * v - span;
            var expected = style == LoftStyle.Ruled
                ? points[span] * (1 - fraction) + points[span + 1] * fraction
                : new CubicHermiteSpline3D(points).Evaluate(v).Origin;
            Assert.InRange((support.Evaluate(u, v) - expected).Length(), 0, 2e-11);
            Assert.InRange((exported.Evaluate(u, v) - expected).Length(), 0, 2e-11);
            Assert.InRange((refinedSupport.Evaluate(u, v) - expected).Length(), 0, 2e-11);
        }
    }
}
