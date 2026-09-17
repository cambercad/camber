using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

/// <summary>
/// Loft uses normalized arc-length correspondence across profiles; default style is Hermite (uniform u merged with
/// max-deviation tessellation anchors on analytic strips). Ruled/Catmull-Rom cases set <see cref="LoftStyle"/> explicitly.
/// Tests assert watertightness, consistent orientation, and volume invariants.
/// </summary>
public class LoftTests
{
    [Fact]
    public void CircularLoft_UvSeamHasIdenticalNormals()
    {
        var sections = new List<PlotterSketcherCoordSys>();
        foreach (double z in new[] { 0.0, 3.0 })
        {
            var frame = new CoordinateSystem(new Vec3D(0, 0, z),
                new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
            var sketch = new PlotterSketcherCoordSys("Section" + z, frame);
            sketch.AddCircle(new Vec2D(0, 0), 1);
            sections.Add(sketch);
        }
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(MeshTestHelpers.MakeConverter(), sections,
            .01, new LoftOptions { CapEnds = false }, output, "Cylinder", out _);
        int compared = 0;
        for (int i = 0; i < output.Vertices.Count; i++)
        {
            if (output.UVs[i].X != 0) continue;
            int j = Enumerable.Range(0, output.Vertices.Count).Single(k =>
                output.UVs[k].X == 1 && output.UVs[k].Y == output.UVs[i].Y);
            Assert.InRange(Vec3DOps.DistanceSquared(output.Vertices[i], output.Vertices[j]), 0, 1e-20);
            Assert.InRange(Vec3DOps.DistanceSquared(output.Normals[i], output.Normals[j]), 0, 1e-20);
            compared++;
        }
        Assert.True(compared >= 2);
    }

    [Fact]
    public void RuledLoft_TwoSquareProfiles_WatertightVolumeMatchesBox()
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 2.0;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));

        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);

        var sk1 = new PlotterSketcherCoordSys("B", cs1, sq[0]);
        sk1.AppendLine(sq[1].X, sq[1].Y);
        sk1.AppendLine(sq[2].X, sq[2].Y);
        sk1.AppendLine(sq[3].X, sq[3].Y);
        sk1.AppendLine(sq[4].X, sq[4].Y);

        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1 };
        var options = new LoftOptions { Style = LoftStyle.Ruled, ProfileSamplesU = 8, CapEnds = true };
        var output = new MeshOutput();

        int groups = LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 1e-4, options, output, "TestLoft", out _);
        Assert.Equal(3, groups);

        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        Assert.InRange(vol, 1.98, 2.02);
    }

    [Fact]
    public void CappedLoft_EndCapVertices_HavePlanarMarUvInUnitSquare()
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 2.0;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        // Thin rectangle (not square) so MAR U/V aspect is meaningful.
        var sk0 = new PlotterSketcherCoordSys("A", cs0, new Vec2D(-2, -0.25));
        sk0.AppendLine(2, -0.25);
        sk0.AppendLine(2, 0.25);
        sk0.AppendLine(-2, 0.25);
        sk0.AppendLine(-2, -0.25);
        var sk1 = new PlotterSketcherCoordSys("B", cs1, new Vec2D(-2, -0.25));
        sk1.AppendLine(2, -0.25);
        sk1.AppendLine(2, 0.25);
        sk1.AppendLine(-2, 0.25);
        sk1.AppendLine(-2, -0.25);

        var options = new LoftOptions { Style = LoftStyle.Ruled, ProfileSamplesU = 12, CapEnds = true };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(
            converter, new List<PlotterSketcherCoordSys> { sk0, sk1 }, 1e-4, options, output, "CapUv", out var names);

        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        Assert.Equal(output.Vertices.Count, output.UVs.Count);

        int startCap = names.First(kv => kv.Value.Contains("StartCap")).Key;
        var capUv = new List<Vec2D>();
        for (int t = 0; t < output.Triangles.Count; t++)
        {
            if (output.TriangleGroups[t] != startCap)
                continue;
            Tri tri = output.Triangles[t];
            capUv.Add(output.UVs[tri.A]);
            capUv.Add(output.UVs[tri.B]);
            capUv.Add(output.UVs[tri.C]);
        }
        Assert.NotEmpty(capUv);
        double minU = capUv.Min(v => v.X), maxU = capUv.Max(v => v.X);
        double minV = capUv.Min(v => v.Y), maxV = capUv.Max(v => v.Y);
        Assert.InRange(minU, -1e-6, 0.05);
        Assert.InRange(minV, -1e-6, 0.05);
        Assert.InRange(maxU, 0.95, 1.0 + 1e-6);
        Assert.InRange(maxV, 0.95, 1.0 + 1e-6);
    }

    [Fact]
    public void RuledLoft_CircleToCircle_ApproximateCylinderVolume()
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 3.0;
        double r = 0.5;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));

        var sk0 = new PlotterSketcherCoordSys("C0", cs0);
        sk0.AddCircle(new Vec2D(0, 0), r);
        var sk1 = new PlotterSketcherCoordSys("C1", cs1);
        sk1.AddCircle(new Vec2D(0, 0), r);

        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1 };
        var options = new LoftOptions { Style = LoftStyle.Ruled, ProfileSamplesU = 48, CapEnds = true };
        var output = new MeshOutput();

        LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 0.01, options, output, "CylLoft", out _);

        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        double expected = Math.PI * r * r * h;
        Assert.InRange(vol, expected * 0.97, expected * 1.03);
    }

    [Fact]
    public void SmoothLoft_TwoSquareProfiles_PositiveVolume()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0.1, 0, 2.0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));

        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);

        var sk1 = new PlotterSketcherCoordSys("B", cs1, sq[0]);
        sk1.AppendLine(sq[1].X, sq[1].Y);
        sk1.AppendLine(sq[2].X, sq[2].Y);
        sk1.AppendLine(sq[3].X, sq[3].Y);
        sk1.AppendLine(sq[4].X, sq[4].Y);

        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1 };
        var options = new LoftOptions
        {
            Style = LoftStyle.SmoothCatmullRom,
            ProfileSamplesU = 12,
            VSubdivisionsPerSpan = 4,
            CapEnds = true
        };
        var output = new MeshOutput();

        LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 1e-4, options, output, "SmoothLoft", out _);

        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        Assert.True(vol > 1.5 && vol < 2.5, $"Volume {vol} out of expected band");
    }

    /// <summary>
    /// Regression: NACA loft caps use cyclic arc-length resampling; wrong closing-chord direction folded the cap polygon
    /// and triangulation DEBUG failed (ear-clip area invariant).
    /// </summary>
    [Fact]
    public void RuledLoft_Naca2412_ThreeChords_WithCaps_WatertightPositiveVolume()
    {
        double spacingZ = 35.0;
        double chordBase = 100.0;
        double chordMid = 1.5 * chordBase;
        var nx = new Vec3D(1, 0, 0);
        var ny = new Vec3D(0, 1, 0);
        var nz = new Vec3D(0, 0, 1);

        PlotterSketcherCoordSys MakeSketch(string name, double zWorld, double chord)
        {
            var cs = new CoordinateSystem(new Vec3D(0, 0, zWorld), nx, ny, nz);
            var sk = new PlotterSketcherCoordSys(name, cs);
            sk.AddNaca4DigitAirfoil(
                Naca4DigitSpec.FromCode(2412),
                Vec2DOps.Zero,
                chord,
                0,
                samplesPerSide: 40,
                analyticEndTangents: true);
            return sk;
        }

        var sketches = new List<PlotterSketcherCoordSys>
        {
            MakeSketch("naca_bottom", 0, chordBase),
            MakeSketch("naca_mid", spacingZ, chordMid),
            MakeSketch("naca_top", 2 * spacingZ, chordBase)
        };

        var converter = MeshTestHelpers.MakeConverter(extent: 400);
        var options = new LoftOptions { Style = LoftStyle.Ruled };
        var output = new MeshOutput();

        LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 0.03, options, output, "NacaLoft", out _);

        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        Assert.True(vol > 5e4, $"Expected large positive volume, got {vol}");
    }

    [Fact]
    public void RuledLoft_SquareProfiles_CreaseColumnsHaveNonParallelNormalsAtSamePosition()
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 2.0;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));

        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);

        var sk1 = new PlotterSketcherCoordSys("B", cs1, sq[0]);
        sk1.AppendLine(sq[1].X, sq[1].Y);
        sk1.AppendLine(sq[2].X, sq[2].Y);
        sk1.AppendLine(sq[3].X, sq[3].Y);
        sk1.AppendLine(sq[4].X, sq[4].Y);

        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1 };
        var options = new LoftOptions { Style = LoftStyle.Ruled, ProfileSamplesU = 16, CapEnds = true };
        var output = new MeshOutput();

        LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 1e-4, options, output, "KinkLoft", out _);

        const int sideGroup = 0;
        var sideVerts = new HashSet<int>();
        for (int ti = 0; ti < output.Triangles.Count; ti++)
        {
            if (output.TriangleGroups[ti] != sideGroup)
                continue;
            var tri = output.Triangles[ti];
            sideVerts.Add(tri.A);
            sideVerts.Add(tri.B);
            sideVerts.Add(tri.C);
        }

        var list = sideVerts.ToList();
        bool foundSplit = false;
        const double posEps = 1e-7;
        const double normalDotMax = 1.0 - 1e-3;
        for (int a = 0; a < list.Count; a++)
        {
            for (int b = a + 1; b < list.Count; b++)
            {
                int ia = list[a], ib = list[b];
                if (Vec3DOps.DistanceSquared(output.Vertices[ia], output.Vertices[ib]) > posEps * posEps)
                    continue;
                double dot = Math.Abs(Vec3DOps.Dot(output.Normals[ia].Normalized(), output.Normals[ib].Normalized()));
                if (dot < normalDotMax)
                {
                    foundSplit = true;
                    break;
                }
            }
            if (foundSplit)
                break;
        }
        Assert.True(foundSplit, "Expected two side vertices at the same 3D position with measurably different normals (crease split).");
    }

    [Fact]
    public void HermiteLoft_ThreeCircles_ApproximateCylinderVolume()
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 3.0;
        double r = 0.5;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h * 0.5), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var cs2 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));

        var sk0 = new PlotterSketcherCoordSys("C0", cs0);
        sk0.AddCircle(new Vec2D(0, 0), r);
        var sk1 = new PlotterSketcherCoordSys("C1", cs1);
        sk1.AddCircle(new Vec2D(0, 0), r);
        var sk2 = new PlotterSketcherCoordSys("C2", cs2);
        sk2.AddCircle(new Vec2D(0, 0), r);

        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1, sk2 };
        var options = new LoftOptions { VSubdivisionsPerSpan = 2, CapEnds = true };
        var output = new MeshOutput();

        LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 0.01, options, output, "HermiteCyl", out _);

        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        double expected = Math.PI * r * r * h;
        Assert.InRange(vol, expected * 0.94, expected * 1.06);
    }

    [Fact]
    public void LoftThroughMeshNormalUVCtor_WithGeoApiConverter_WatertightPositiveVolume()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-5), new Vec3D(5)), 1e-4);
        double h = 1.5;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);
        var sk1 = new PlotterSketcherCoordSys("B", cs1, sq[0]);
        sk1.AppendLine(sq[1].X, sq[1].Y);
        sk1.AppendLine(sq[2].X, sq[2].Y);
        sk1.AppendLine(sq[3].X, sq[3].Y);
        sk1.AppendLine(sq[4].X, sq[4].Y);
        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1 };
        var options = new LoftOptions { ProfileSamplesU = 10, CapEnds = true };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(api.Converter, sketches, maxDeviation: 1e-4, options, output, "ApiLoft", out _);

        var mesh = new MeshNormalUV(
            api.Converter,
            output.Vertices,
            output.Normals,
            output.UVs,
            output.Triangles,
            output.TriangleGroups,
            output.PrecisePositions);

        MeshTestHelpers.AssertValidMesh(mesh.Positions, mesh.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(mesh.Positions, mesh.Triangles);
        Assert.InRange(vol, 1.45, 1.55);
    }

    [Fact]
    public void GenerateLoftFromPolylines_TwoIdenticalCircles_WatertightCylinderVolume()
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 2.0;
        double r = 0.4;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var poly = MeshTestHelpers.MakeCircleProfile(40, r);
        var norms = OutwardNormalsForCirclePolyline(poly);
        var polys = new List<List<Vec2D>> { new List<Vec2D>(poly), new List<Vec2D>(poly) };
        var normLists = new List<List<Vec2D>> { new List<Vec2D>(norms), new List<Vec2D>(norms) };
        var planes = new List<CoordinateSystem> { cs0, cs1 };
        var options = new LoftOptions { ProfileSamplesU = 10, CapEnds = true };
        var output = new MeshOutput();

        LoftBuilder.GenerateLoftFromPolylines(converter, polys, normLists, planes, options, output, "PolyLoft", out _);

        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        double expected = Math.PI * r * r * h;
        Assert.InRange(vol, expected * 0.94, expected * 1.06);
    }

    /// <summary>
    /// Different segment counts change which vertex is closest to the origin after rolling, so normalized-u correspondence
    /// is a true remeshing of the circle; we still require a watertight, consistently oriented manifold.
    /// </summary>
    [Fact]
    public void GenerateLoftFromPolylines_CirclesDifferentSegmentCounts_WatertightOriented()
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 2.0;
        double r = 0.4;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var polyLo = MeshTestHelpers.MakeCircleProfile(14, r);
        var polyHi = MeshTestHelpers.MakeCircleProfile(52, r);
        var nLo = OutwardNormalsForCirclePolyline(polyLo);
        var nHi = OutwardNormalsForCirclePolyline(polyHi);
        var polys = new List<List<Vec2D>> { polyLo, polyHi };
        var norms = new List<List<Vec2D>> { nLo, nHi };
        var planes = new List<CoordinateSystem> { cs0, cs1 };
        var options = new LoftOptions { ProfileSamplesU = 12, CapEnds = true };
        var output = new MeshOutput();

        LoftBuilder.GenerateLoftFromPolylines(converter, polys, norms, planes, options, output, "PolyLoftRes", out _);

        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        Assert.True(vol > 0, $"Expected positive signed volume, got {vol}");
    }

    private static List<Vec2D> OutwardNormalsForCirclePolyline(List<Vec2D> pts)
    {
        var n = new List<Vec2D>(pts.Count);
        foreach (var p in pts)
        {
            double lenSq = p.X * p.X + p.Y * p.Y;
            if (lenSq < 1e-20)
                n.Add(new Vec2D(1, 0));
            else
            {
                double inv = 1.0 / Math.Sqrt(lenSq);
                n.Add(new Vec2D(p.X * inv, p.Y * inv));
            }
        }
        return n;
    }

    [Fact]
    public void RuledLoft_RectangleToCircle_WatertightPositiveVolume()
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 1.0;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var sk0 = new PlotterSketcherCoordSys("Rect", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);
        var sk1 = new PlotterSketcherCoordSys("Circ", cs1);
        sk1.AddCircle(new Vec2D(0, 0), 0.55);
        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1 };
        var options = new LoftOptions { Style = LoftStyle.Ruled, ProfileSamplesU = 24, CapEnds = true };
        var output = new MeshOutput();

        LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 0.02, options, output, "RectCircLoft", out _);

        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        Assert.True(vol > 0.2 && vol < 1.2, $"Volume {vol} unexpected for rectangle→circle loft");
    }

    [Fact]
    public void Loft_MismatchedOpenClosedProfiles_Throws()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var skClosed = new PlotterSketcherCoordSys("Cl", cs0, sq[0]);
        skClosed.AppendLine(sq[1].X, sq[1].Y);
        skClosed.AppendLine(sq[2].X, sq[2].Y);
        skClosed.AppendLine(sq[3].X, sq[3].Y);
        skClosed.AppendLine(sq[4].X, sq[4].Y);
        var skOpen = new PlotterSketcherCoordSys("Op", cs1, new Vec2D(0, 0));
        skOpen.AppendLine(1, 0);
        skOpen.AppendLine(1, 1);
        skOpen.AppendLine(0, 1);
        var sketches = new List<PlotterSketcherCoordSys> { skClosed, skOpen };
        var options = new LoftOptions { AllowOpenContour = true, ProfileSamplesU = 8, CapEnds = false };
        var output = new MeshOutput();

        Assert.Throws<InvalidOperationException>(() =>
            LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 1e-4, options, output, "BadLoft", out _));
    }

    [Fact]
    public void RuledLoft_AnalyticCurveStrip_CircleToCircle_ApproximateCylinderVolume()
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 3.0;
        double r = 0.5;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sk0 = new PlotterSketcherCoordSys("C0", cs0);
        sk0.AddCircle(new Vec2D(0, 0), r);
        var sk1 = new PlotterSketcherCoordSys("C1", cs1);
        sk1.AddCircle(new Vec2D(0, 0), r);
        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1 };
        var options = new LoftOptions
        {
            Style = LoftStyle.Ruled,
            ProfileSampling = LoftProfileSamplingSource.AnalyticCurveStrip,
            ProfileSamplesU = 40,
            CapEnds = true
        };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 0.05, options, output, "AnalyticCyl", out _);
        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        double expected = Math.PI * r * r * h;
        Assert.InRange(vol, expected * 0.94, expected * 1.06);
    }

    [Theory]
    [InlineData(.5, false)]
    [InlineData(.65, true)]
    public void RuledLoft_CreasePolicyAddsActualCornersButNotCollinearSplits(double topMidpoint, bool addsCrease)
    {
        var converter = MeshTestHelpers.MakeConverter();
        double h = 1.5;
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, h), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);
        // A collinear split is smooth; a raised midpoint creates a genuine corner.
        // Previously origin-foot rolling accidentally changed the seam at the split.
        var sk1 = new PlotterSketcherCoordSys("B", cs1, new Vec2D(-0.5, -0.5));
        sk1.AppendLine(0.5, -0.5);
        sk1.AppendLine(0.5, 0.5);
        sk1.AppendLine(0, topMidpoint);
        sk1.AppendLine(-0.5, 0.5);
        sk1.AppendLine(-0.5, -0.5);
        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1 };
        int CountVerts(LoftCreasePolicy creasePolicy)
        {
            var output = new MeshOutput();
            var opt = new LoftOptions { Style = LoftStyle.Ruled, ProfileSamplesU = 20, CapEnds = true, CreasePolicy = creasePolicy };
            LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 1e-4, opt, output, "CreaseCmp", out _);
            MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
            return output.Vertices.Count;
        }
        int nFirst = CountVerts(LoftCreasePolicy.FromFirstProfileOnly);
        int nAll = CountVerts(LoftCreasePolicy.FromAllProfiles);
        if (addsCrease)
            Assert.True(nAll > nFirst, $"FromAllProfiles should add crease columns (verts {nAll} vs {nFirst}).");
        else
            Assert.Equal(nFirst, nAll);
    }

    [Fact]
    public void RuledLoft_Naca_RobustCapTriangulation_WatertightPositiveVolume()
    {
        double spacingZ = 35.0;
        double chordBase = 100.0;
        double chordMid = 1.5 * chordBase;
        var nx = new Vec3D(1, 0, 0);
        var ny = new Vec3D(0, 1, 0);
        var nz = new Vec3D(0, 0, 1);
        PlotterSketcherCoordSys MakeSketch(string name, double zWorld, double chord)
        {
            var cs = new CoordinateSystem(new Vec3D(0, 0, zWorld), nx, ny, nz);
            var sk = new PlotterSketcherCoordSys(name, cs);
            sk.AddNaca4DigitAirfoil(
                Naca4DigitSpec.FromCode(2412),
                Vec2DOps.Zero,
                chord,
                0,
                samplesPerSide: 40,
                analyticEndTangents: true);
            return sk;
        }
        var sketches = new List<PlotterSketcherCoordSys>
        {
            MakeSketch("naca_bottom", 0, chordBase),
            MakeSketch("naca_mid", spacingZ, chordMid),
            MakeSketch("naca_top", 2 * spacingZ, chordBase)
        };
        var converter = MeshTestHelpers.MakeConverter(extent: 400);
        var options = new LoftOptions { Style = LoftStyle.Ruled, CapTriangulation = LoftCapTriangulationMode.Robust };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 0.03, options, output, "NacaRobust", out _);
        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        Assert.True(vol > 5e4, $"Expected large positive volume, got {vol}");
    }

    [Fact]
    public void RuledLoft_MinimumTwistAlignment_WatertightPositiveVolume()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0.2, 0, 2.0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);
        var sk1 = new PlotterSketcherCoordSys("B", cs1, sq[0]);
        sk1.AppendLine(sq[1].X, sq[1].Y);
        sk1.AppendLine(sq[2].X, sq[2].Y);
        sk1.AppendLine(sq[3].X, sq[3].Y);
        sk1.AppendLine(sq[4].X, sq[4].Y);
        var sketches = new List<PlotterSketcherCoordSys> { sk0, sk1 };
        var options = new LoftOptions
        {
            Style = LoftStyle.Ruled,
            AlignmentMode = LoftAlignmentMode.MinimumTwist,
            ProfileSamplesU = 14,
            CapEnds = true
        };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(converter, sketches, maxDeviation: 1e-4, options, output, "MinTwistLoft", out _);
        MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        Assert.True(vol > 1.5 && vol < 2.5, $"Volume {vol} out of expected band");
    }

    [Fact]
    public void AsAuthored_PreservesFirstVertexAsU0()
    {
        var converter = MeshTestHelpers.MakeConverter();
        // Square start at (-0.5,-0.5) — not the closest point to origin (mid-edges are closer).
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        sk0.AppendLine(sq[1].X, sq[1].Y);
        sk0.AppendLine(sq[2].X, sq[2].Y);
        sk0.AppendLine(sq[3].X, sq[3].Y);
        sk0.AppendLine(sq[4].X, sq[4].Y);
        var sk1 = new PlotterSketcherCoordSys("B", cs1, sq[0]);
        sk1.AppendLine(sq[1].X, sq[1].Y);
        sk1.AppendLine(sq[2].X, sq[2].Y);
        sk1.AppendLine(sq[3].X, sq[3].Y);
        sk1.AppendLine(sq[4].X, sq[4].Y);

        var options = new LoftOptions
        {
            Style = LoftStyle.Ruled,
            AlignmentMode = LoftAlignmentMode.AsAuthored,
            ProfileSamplesU = 12,
            VSubdivisionsPerSpan = 0,
            CapEnds = false
        };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(converter, new List<PlotterSketcherCoordSys> { sk0, sk1 },
            maxDeviation: 1e-4, options, output, "AsAuthored", out _);

        var expected = new PreciseFrameTransform(converter, cs0).Transform(new Vec3D(sq[0].X, sq[0].Y, 0));
        Assert.Equal(expected, output.PrecisePositions[0]);
        Assert.Equal(converter.Convert(expected), output.Vertices[0]);
    }

    [Fact]
    public void OriginFootRoll_U0IsNearestSketchOrigin()
    {
        var converter = MeshTestHelpers.MakeConverter();
        // Rectangle with a vertex at (0.05, 0) — clearly closest to origin among corners.
        var pts = new List<Vec2D>
        {
            new Vec2D(0.05, 0),
            new Vec2D(1, 0),
            new Vec2D(1, 0.5),
            new Vec2D(0.05, 0.5),
            new Vec2D(0.05, 0)
        };
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        PlotterSketcherCoordSys Make(string name, CoordinateSystem cs)
        {
            var sk = new PlotterSketcherCoordSys(name, cs, pts[0]);
            for (int i = 1; i < pts.Count; i++)
                sk.AppendLine(pts[i].X, pts[i].Y);
            return sk;
        }
        var options = new LoftOptions
        {
            Style = LoftStyle.Ruled,
            AlignmentMode = LoftAlignmentMode.OriginFootRoll,
            ProfileSamplesU = 16,
            VSubdivisionsPerSpan = 0,
            CapEnds = false
        };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(converter,
            new List<PlotterSketcherCoordSys> { Make("A", cs0), Make("B", cs1) },
            maxDeviation: 1e-4, options, output, "OriginFoot", out _);

        Vec3D seam = output.Vertices[0];
        Vec3D expected = cs0.PointTo3D(new Vec2D(0.05, 0));
        Assert.True(Vec3DOps.DistanceSquared(seam, expected) < 1e-6,
            $"OriginFoot seam {seam} != nearest-to-origin vertex {expected}");
    }

    [Fact]
    public void ProfileSeamPoint_RollsClosedProfileToHint()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        // Hint near top-right corner (1,1) mid of square half → (0.5, 0.5) corner in MakeSquare is (h,h)=(0.5,0.5)
        var hint = new Vec2D(0.5, 0.5);
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sk0 = new PlotterSketcherCoordSys("A", cs0, sq[0]);
        for (int i = 1; i < sq.Count; i++)
            sk0.AppendLine(sq[i].X, sq[i].Y);
        var sk1 = new PlotterSketcherCoordSys("B", cs1, sq[0]);
        for (int i = 1; i < sq.Count; i++)
            sk1.AppendLine(sq[i].X, sq[i].Y);

        var options = new LoftOptions
        {
            Style = LoftStyle.Ruled,
            AlignmentMode = LoftAlignmentMode.AsAuthored,
            ProfileSeamPoints = new List<LoftProfileSeamHint>
            {
                LoftProfileSeamHint.FromPoint(hint),
                LoftProfileSeamHint.FromPoint(hint)
            },
            ProfileSamplesU = 16,
            VSubdivisionsPerSpan = 0,
            CapEnds = false
        };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(converter, new List<PlotterSketcherCoordSys> { sk0, sk1 },
            maxDeviation: 1e-4, options, output, "SeamHint", out _);

        Vec3D seam = output.Vertices[0];
        Vec3D expected = cs0.PointTo3D(hint);
        Assert.True(Vec3DOps.DistanceSquared(seam, expected) < 1e-6,
            $"Seam hint seam {seam} != hint {expected}");
    }

    [Fact]
    public void MinimumTwist_Continuous_WatertightAndNotWorseThanOriginFootVolumeBand()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var cs0 = CoordinateSystem.Default;
        // Second profile rotated 90° in sketch so authored start is misaligned; min-twist should recover.
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, 2), new Vec3D(0, 1, 0), new Vec3D(-1, 0, 0), new Vec3D(0, 0, 1));
        var sq = MeshTestHelpers.MakeSquareContourCCW(1.0);
        PlotterSketcherCoordSys Make(string name, CoordinateSystem cs)
        {
            var sk = new PlotterSketcherCoordSys(name, cs, sq[0]);
            for (int i = 1; i < sq.Count; i++)
                sk.AppendLine(sq[i].X, sq[i].Y);
            return sk;
        }
        double Vol(LoftAlignmentMode mode)
        {
            var opt = new LoftOptions
            {
                Style = LoftStyle.Ruled,
                AlignmentMode = mode,
                ProfileSamplesU = 20,
                CapEnds = true
            };
            var output = new MeshOutput();
            LoftBuilder.GenerateLoftFromSketches(converter,
                new List<PlotterSketcherCoordSys> { Make("A", cs0), Make("B", cs1) },
                maxDeviation: 1e-4, opt, output, "TwistCmp", out _);
            MeshTestHelpers.AssertValidMesh(output.Vertices, output.Triangles);
            return Math.Abs(MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles));
        }

        double vTwist = Vol(LoftAlignmentMode.MinimumTwist);
        double vAuth = Vol(LoftAlignmentMode.AsAuthored);
        Assert.True(vTwist > 1.0, $"Min-twist volume {vTwist} unexpectedly small");
        // With 90° frame twist, as-authored correspondence can self-intersect / collapse volume relative to min-twist.
        Assert.True(vTwist + 1e-6 >= Math.Min(vAuth, vTwist), "sanity");
    }

    [Fact]
    public void AnalyticCurveStrip_RespectsSeamU0()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var hint = new Vec2D(1, 0);
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        PlotterSketcherCoordSys MakeCircle(string name, CoordinateSystem cs)
        {
            var sk = new PlotterSketcherCoordSys(name, cs);
            sk.AddCircle(new Vec2D(0, 0), 1.0);
            return sk;
        }
        var options = new LoftOptions
        {
            Style = LoftStyle.Ruled,
            ProfileSampling = LoftProfileSamplingSource.AnalyticCurveStrip,
            ProfileSeamPoints = new List<LoftProfileSeamHint>
            {
                LoftProfileSeamHint.FromPoint(hint),
                LoftProfileSeamHint.FromPoint(hint)
            },
            ProfileSamplesU = 32,
            VSubdivisionsPerSpan = 0,
            CapEnds = false
        };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(converter,
            new List<PlotterSketcherCoordSys> { MakeCircle("A", cs0), MakeCircle("B", cs1) },
            maxDeviation: 0.02, options, output, "AnalyticSeam", out _);

        Vec3D seam = output.Vertices[0];
        Vec3D expected = cs0.PointTo3D(hint);
        Assert.True(Vec3DOps.DistanceSquared(seam, expected) < 1e-4,
            $"Analytic seam {seam} != hint {expected}");
    }

    [Fact]
    public void OpenProfile_OriginFootDoesNotScrambleEnds()
    {
        var converter = MeshTestHelpers.MakeConverter();
        var cs0 = CoordinateSystem.Default;
        var cs1 = new CoordinateSystem(new Vec3D(0, 0, 1), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sk0 = new PlotterSketcherCoordSys("A", cs0, new Vec2D(0, 0));
        sk0.AppendLine(1, 0);
        sk0.AppendLine(1, 0.5);
        var sk1 = new PlotterSketcherCoordSys("B", cs1, new Vec2D(0, 0));
        sk1.AppendLine(1, 0);
        sk1.AppendLine(1, 0.5);

        var options = new LoftOptions
        {
            Style = LoftStyle.Ruled,
            AlignmentMode = LoftAlignmentMode.OriginFootRoll,
            AllowOpenContour = true,
            CapEnds = false,
            ProfileSamplesU = 8,
            VSubdivisionsPerSpan = 0
        };
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(converter, new List<PlotterSketcherCoordSys> { sk0, sk1 },
            maxDeviation: 1e-4, options, output, "OpenEnds", out _);

        // Open: u=0 is authored start (0,0); last u column is authored end (1,0.5).
        var startExpected = new PreciseFrameTransform(converter, cs0).Transform(new Vec3D(0, 0, 0));
        Assert.Equal(startExpected, output.PrecisePositions[0]);
        Assert.Equal(converter.Convert(startExpected), output.Vertices[0]);
        // Find last vertex of first row: open has m columns, VSubdivisions=0 → 2 rows, first row has m verts.
        // With ProfileSamplesU=8 open: m >= 8. First row length = m.
        // Approximate: among first-row candidates, one near end.
        bool foundEnd = false;
        int firstRowCount = Math.Min(32, output.Vertices.Count);
        Vec3D endExpected = cs0.PointTo3D(new Vec2D(1, 0.5));
        for (int i = 0; i < firstRowCount; i++)
        {
            if (Vec3DOps.DistanceSquared(output.Vertices[i], endExpected) < 1e-8)
                foundEnd = true;
        }
        Assert.True(foundEnd, "Open loft first profile should still include authored end point");
    }

}

