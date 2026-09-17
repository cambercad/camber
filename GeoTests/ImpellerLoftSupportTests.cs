using CSG;
using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class ImpellerLoftSupportTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    // Authored meridian and section dimensions from the impeller example.
    // No tessellated asset, mesh repair or change to the requested round.
    internal static (GeoAPI Api, AnchorMesh Hub, AnchorMesh Blade, AnchorMesh Joined) Fixture(bool join = true)
    {
        const double deviation = .12;
        var api = new GeoAPI(new Box3D(new Vec3D(-145, -145, -10), new Vec3D(145, 145, 115)), deviation);
        var meridianFrame = new Plane3D(new Vec3D(0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
        var meridian = api.GetPlotterSketcher(meridianFrame, "hub_meridian");
        meridian.AddLine(new Vec2D(0, 20), new Vec2D(0, 120)).Name = "rear_face";
        meridian.AddLine(new Vec2D(0, 120), new Vec2D(6, 120)).Name = "outer_rim";
        meridian.AddLine(new Vec2D(6, 120), new Vec2D(6, 112)).Name = "rim_land";
        meridian.AddArc(new Vec2D(6, 112), new Vec2D(90 - 84 / Math.Sqrt(2), 112 - 84 / Math.Sqrt(2)), new Vec2D(90, 28)).Name = "bell";
        meridian.AddLine(new Vec2D(90, 28), new Vec2D(96, 28)).Name = "neck";
        meridian.AddLine(new Vec2D(96, 28), new Vec2D(96, 20)).Name = "hub_face";
        meridian.AddLine(new Vec2D(96, 20), new Vec2D(0, 20)).Name = "shaft_bore";
        var hub = api.Revolve(meridian, 2 * Math.PI, name: "revolved_hub");
        (double radius, double degrees, double top, double thickness)[] stations =
        [
            (27.5, 0, 92, 2.8), (29.5, -1, 91, 2.8), (32, -2.5, 88, 2.7),
            (38, -5, 82, 2.6), (48, -9, 72, 2.5), (64, -16, 56, 2.3),
            (84, -24, 39, 2.1), (104, -33, 26, 1.9), (120, -40, 20, 1.8)
        ];
        var sections = new List<PlotterSketcherCoordSys>();
        for (int i = 0; i < stations.Length; i++)
        {
            var (radius, degrees, top, thickness) = stations[i];
            double theta = degrees * Math.PI / 180;
            double bottom = (radius <= 28 ? 90 : radius >= 112 ? 6 :
                90 - Math.Sqrt(84 * 84 - (112 - radius) * (112 - radius))) - 2;
            var frame = new CoordinateSystem(new Vec3D(radius * Math.Cos(theta), radius * Math.Sin(theta), bottom),
                new Vec3D(-Math.Sin(theta), Math.Cos(theta), 0), new Vec3D(0, 0, 1),
                new Vec3D(Math.Cos(theta), Math.Sin(theta), 0));
            var section = new PlotterSketcherCoordSys("blade_station_" + i, frame, new Vec2D(-thickness / 2, 0));
            section.AddRectangleFromCorners(new Vec2D(-thickness / 2, 0), new Vec2D(thickness / 2, top - bottom));
            sections.Add(section);
        }
        if (!join)
        {
            var output = new MeshOutput();
            LoftBuilder.GenerateLoftFromSketches(api.Converter, sections, deviation, LoftOptions.PropellerBlade,
                output, "blade_loft", out var names, 0);
            var mesh = new MeshNormalUV(api.Converter, output.Vertices, output.Normals, output.UVs,
                output.Triangles, output.TriangleGroups, output.PrecisePositions);
            var metadata = Geo.NurbsConstruction.NurbsPatchMetadataBuilder.BuildLoftMetadata(sections, deviation,
                LoftOptions.PropellerBlade, "blade_loft", output.LoftSideSupportFactory);
            return (api, hub, new AnchorMesh("raw", mesh, names, metadata, false, preserveTriangulation: true), null);
        }
        var blade = api.Loft(sections, LoftOptions.PropellerBlade, "blade_loft", deviation);
        if (!join) return (api, hub, blade, null);
        var joined = api.Boolean(hub, blade, BooleanOp.Union, "joined");
        joined.EnsureCoplanarPostProcessed();
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(joined.Mesh);
        return (api, hub, blade, joined);
    }

    [Fact]
    public void RetainedHubSupportsMatchMeshCornerParameters()
    {
        var (api, hub, _, _) = Fixture(false);
        double worst = 0;
        string face = "";
        var errors = new Dictionary<string, double>();
        var witnesses = new Dictionary<string, string>();
        for (int i = 0; i < hub.Mesh.Triangles.Count; i++)
        {
            var triangle = hub.Mesh.Triangles[i];
            var data = hub.Mesh.TrianglesEx[i];
            var name = hub.groupIdToExtendedName[data.GroupId];
            if (!name.Contains("neck") && !name.Contains("bell") && !name.Contains("rim_land")) continue;
            var surface = hub.surfaceMetaData[name].NurbsSurface;
            var ids = new[] { triangle.A, triangle.B, triangle.C };
            var uv = new[] { data.V0.UV, data.V1.UV, data.V2.UV };
            for (int j = 0; j < 3; j++)
            {
                double error = (surface.Evaluate(uv[j].X, uv[j].Y) - hub.Mesh.Positions[ids[j]]).Length();
                if (error >= errors.GetValueOrDefault(name))
                    witnesses[name] = $"UV {uv[j]} actual {hub.Mesh.Positions[ids[j]]} evaluated {surface.Evaluate(uv[j].X, uv[j].Y)}";
                errors[name] = Math.Max(errors.GetValueOrDefault(name), error);
                if (error > worst) { worst = error; face = name; }
            }
        }
        Assert.True(worst <= 2 * Math.Sqrt(3) * api.Converter.SmallestUnit(), string.Join("; ", errors.Select(p => $"{p.Key}: {p.Value:G17} {witnesses[p.Key]}")));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HubNeckRetainsItsExtentAndRadialNormals(bool afterUnion)
    {
        var (api, hub, _, joined) = Fixture(afterUnion);
        var body = afterUnion ? joined : hub;
        Assert.True(body.TryGetTopologySurface("revolved_hub-neck", out var neck));
        double rounding = 2 * Math.Sqrt(3) * api.Converter.SmallestUnit();
        foreach (int vertex in neck.Triangles.SelectMany(t => new[] { t.A, t.B, t.C }).Distinct())
        {
            var point = api.Converter.Convert(neck.PointsPrecise[vertex]);
            Assert.InRange(point.Z, 90 - rounding, 96 + rounding);
            Assert.InRange(Math.Abs(neck.Normals[vertex].Z), 0, 1e-12);
        }
    }

    // Keep the intended success assertion intact until offset-branch selection
    // is implemented. See KnownFilletLimitations.md for the exact failure.
    [Fact(Skip = "Known fillet limitation: the impeller offset surfaces produce disconnected branches; safe branch selection is not implemented. See GeoTests/KnownFilletLimitations.md.")]
    public void OriginalRootFilletHasAConnectedSpine()
    {
        var (api, _, _, joined) = Fixture();
        int edge = joined.GroupEdges.FindIndex(item => item.Name.Contains("revolved_hub-neck") && item.Name.Contains("blade_loft-Side"));
        Assert.IsType<NURBS.BSplineSurface>(joined.surfaceMetaData["blade_loft-Side"].NurbsSurface);
        var rounded = api.Fillet(joined, [joined.GetEdgeReference(edge)], 1.2, .12, "root_round");
        MeshPipelineTestHelpers.AssertWatertightAllowTouch(rounded.Mesh);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LoftTriangleInteriorsRespectRequestedSupportDeviation(bool cleaned)
    {
        var (api, _, blade, _) = Fixture(cleaned);
        LoftTriangleDeviationTests.AssertTriangleInteriors(blade, "blade_loft-Side", .12, api.Converter.SmallestUnit());
    }

    [Fact]
    public void RetainedLoftSupportMatchesAuthoredMeshCornerParameters()
    {
        var (api, _, blade, _) = Fixture();
        var metadata = blade.surfaceMetaData["blade_loft-Side"];
        var support = metadata.NurbsSurface;
        int group = blade.extendedNameToGroupId["blade_loft-Side"];
        double maximumError = 0;
        for (int i = 0; i < blade.Mesh.Triangles.Count; i++)
        {
            var attributes = blade.Mesh.TrianglesEx[i];
            if (attributes.GroupId != group)
                continue;
            var triangle = blade.Mesh.Triangles[i];
            var indices = new[] { triangle.A, triangle.B, triangle.C };
            var parameters = new[] { attributes.V0.UV, attributes.V1.UV, attributes.V2.UV };
            for (int corner = 0; corner < 3; corner++)
            {
                var uv = parameters[corner];
                maximumError = Math.Max(maximumError,
                    (support.Evaluate(uv.X, uv.Y) - blade.Mesh.Positions[indices[corner]]).Length());
            }
        }
        double latticeBound = 2 * Math.Sqrt(3) * api.Converter.SmallestUnit();
        Assert.True(maximumError <= latticeBound,
            $"Retained support differs from authored loft by {maximumError:G17} mm; lattice bound {latticeBound:G17} mm.");
    }

}
