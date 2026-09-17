using Curves;
using Geo;
using Geo.NurbsConstruction;
using GeoCore;

namespace GeoTests;

public class LoftTriangleDeviationTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(LoftStyle.Ruled)]
    [InlineData(LoftStyle.Hermite)]
    [InlineData(LoftStyle.SmoothCatmullRom)]
    public void StraightColumnsStillRequireRefinementAcrossTwistedTriangleCells(LoftStyle style)
    {
        const double deviation = .05;
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .005);
        var sections = new List<PlotterSketcherCoordSys>();
        for (int station = 0; station < 2; station++)
        {
            double angle = station * Math.PI / 3;
            var frame = new CoordinateSystem(new Vec3D(0, 0, station * 10),
                new Vec3D(Math.Cos(angle), Math.Sin(angle), 0),
                new Vec3D(-Math.Sin(angle), Math.Cos(angle), 0), new Vec3D(0, 0, 1));
            var section = new PlotterSketcherCoordSys("station_" + station, frame, new Vec2D(-5, -1));
            section.AddRectangleFromCorners(new Vec2D(-5, -1), new Vec2D(5, 1));
            sections.Add(section);
        }
        var options = new LoftOptions
        {
            Style = style, ProfileSamplesU = 4, VSubdivisionsPerSpan = 0,
            AlignmentMode = LoftAlignmentMode.AsAuthored,
            CreasePolicy = LoftCreasePolicy.None,
            CorrespondenceMode = LoftCorrespondenceMode.UniformUOnly
        };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(api.Converter, sections, deviation,
            options, output, "twisted", out var names, 0);
        var metadata = NurbsPatchMetadataBuilder.BuildLoftMetadata(sections, deviation,
            options, "twisted", output.LoftSideSupportFactory);
        var mesh = new MeshNormalUV(api.Converter, output.Vertices, output.Normals, output.UVs,
            output.Triangles, output.TriangleGroups, output.PrecisePositions);
        var body = new AnchorMesh("twisted", mesh, names, metadata, false, preserveTriangulation: true);
        AssertTriangleInteriors(body, "twisted-Side", deviation, api.Converter.SmallestUnit());
        Assert.Contains(output.UVs, uv => uv.Y > 0 && uv.Y < 1);
    }

    [Fact]
    public void AdaptiveSurfaceTessellationPreservesImpellerInteriorAccuracy()
    {
        var (_, _, blade, _) = ImpellerLoftSupportTests.Fixture(false);
        var support = Assert.IsType<NURBS.BSplineSurface>(blade.surfaceMetaData["blade_loft-Side"].NurbsSurface);
        var originalControls = support.ControlPoints.Select(row => row.ToArray()).ToArray();
        var result = NURBS.AdaptiveSurfaceSplitter.TriangulateAdaptive(support, maxDeviation: .12);
        NurbsTriangleDeviationTests.AssertInteriorAccuracy(support, result, .12);
        for (int row = 0; row < originalControls.Length; row++)
            Assert.Equal(originalControls[row], support.ControlPoints[row]);
    }

    internal static void AssertTriangleInteriors(AnchorMesh body, string face, double deviation, double latticeUnit)
    {
        var support = body.surfaceMetaData[face].NurbsSurface;
        int group = body.extendedNameToGroupId[face];
        double bound = deviation + 2 * Math.Sqrt(3) * latticeUnit;
        for (int index = 0; index < body.Mesh.Triangles.Count; index++)
        {
            var data = body.Mesh.TrianglesEx[index];
            if (data.GroupId != group) continue;
            var triangle = body.Mesh.Triangles[index];
            // A triangular grid covers corners, edge interiors, and interior
            // points rather than merely the emitted construction vertices.
            for (int i = 0; i <= 6; i++)
            for (int j = 0; j <= 6 - i; j++)
            {
                double a = i / 6.0;
                double b = j / 6.0;
                double c = 1 - a - b;
                var uv = data.V0.UV * a + data.V1.UV * b + data.V2.UV * c;
                var point = body.Mesh.Positions[triangle.A] * a +
                    body.Mesh.Positions[triangle.B] * b + body.Mesh.Positions[triangle.C] * c;
                double error = (support.Evaluate(uv.X, uv.Y) - point).Length();
                Assert.True(error <= bound,
                    $"Triangle {index}, barycentric ({a}, {b}, {c}): {error:G17} exceeds {bound:G17}.");
            }
        }
    }
}
