using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

/// <summary>
/// <c>GeoScriptViewer/TestScriptNaca.py</c> calls <see cref="GeoAPI.Loft(IReadOnlyList{PlotterSketcherCoordSys}, LoftOptions, string, double)"/> with <see cref="LoftOptions.Default"/>,
/// which builds an <see cref="AnchorMesh"/> and runs coplanar / planarity logic on <see cref="GeoCore.UVSurface"/>
/// (exact lattice normals). That path is not exercised by <see cref="LoftBuilder.GenerateLoftFromSketches"/> alone.
/// </summary>
public class NacaLoftDegenerateRegressionTests
{
    private static List<PlotterSketcherCoordSys> BuildTestScriptNacaSketches()
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

        return new List<PlotterSketcherCoordSys>
        {
            MakeSketch("naca_loft_bottom", 0.0, chordBase),
            MakeSketch("naca_loft_middle", spacingZ, chordMid),
            MakeSketch("naca_loft_top", 2.0 * spacingZ, chordBase)
        };
    }

    /// <summary>
    /// Collects diagnostics when triangles fall below the loft degeneracy threshold (same formula as mesh emission).
    /// </summary>
    private static (int badCount, double minSq, string? firstReport) AnalyzeDegenerateTriangles(
        IReadOnlyList<Vec3D> vertices,
        IReadOnlyList<Tri> triangles,
        double minSquaredCrossNorm)
    {
        double minObserved = double.MaxValue;
        int bad = 0;
        string? first = null;
        for (int ti = 0; ti < triangles.Count; ti++)
        {
            var t = triangles[ti];
            var a = vertices[t.A];
            var b = vertices[t.B];
            var c = vertices[t.C];
            double sq = MeshConstructionHelpers.SquaredTriangleCrossNorm(a, b, c);
            if (sq < minObserved)
                minObserved = sq;
            if (MeshConstructionHelpers.IsDegenerateTriangleMesh(t, a, b, c, minSquaredCrossNorm))
            {
                bad++;
                first ??=
                    $"tri#{ti} ({t.A},{t.B},{t.C}) sqCross={sq:R} minAllowed={minSquaredCrossNorm:R} " +
                    $"A=({a.X:R},{a.Y:R},{a.Z:R}) B=({b.X:R},{b.Y:R},{b.Z:R}) C=({c.X:R},{c.Y:R},{c.Z:R})";
            }
        }

        if (minObserved == double.MaxValue)
            minObserved = 0;
        return (bad, minObserved, first);
    }

    [Fact]
    public void NacaLoft_TestScriptReplica_MatchesPythonScript_AndHasNoDegenerateTriangles()
    {
        var sketches = BuildTestScriptNacaSketches();
        var converter = new CoordinateConverter(
            new Box3D(new Vec3D(-20, -30, -10), new Vec3D(200, 30, 100)));
        var options = LoftOptions.Default;
        var output = new MeshOutput();

        LoftBuilder.GenerateLoftFromSketches(
            converter, sketches, maxDeviation: 0.03, options, output, "naca_three_section_loft", out _);

        MeshTestHelpers.AssertWatertight(output.Vertices, output.Triangles);
        MeshTestHelpers.AssertConsistentOrientation(output.Vertices, output.Triangles);

        double L = MeshConstructionHelpers.MeshAxisAlignedMaxExtent(output.Vertices);
        double minSqCross = MeshConstructionHelpers.LoftMinSquaredCrossNormFromMaxExtent(L);
        var (badCount, minObserved, firstBad) = AnalyzeDegenerateTriangles(
            output.Vertices, output.Triangles, minSqCross);

        Assert.True(
            badCount == 0,
            $"Degenerate triangles: count={badCount}, min ‖cross‖² observed={minObserved:R}, threshold={minSqCross:R}, L={L:R}. First: {firstBad}");

        Assert.True(minObserved >= minSqCross * (1.0 - 1e-9),
            $"Minimum ‖cross‖² {minObserved:R} below emission threshold {minSqCross:R} (extent L={L:R}).");

        Assert.True(minObserved > 0,
            "Expected strictly positive ‖(b-a)×(c-a)‖² for every triangle (no numerically zero-area faces).");

        double vol = MeshAnalysis.ComputeSignedMeshVolume(output.Vertices, output.Triangles);
        Assert.True(vol > 5e4, $"Expected large positive volume, got {vol}");
    }

    [Fact]
    public void NacaLoft_TestScriptReplica_NoDuplicateVertexIndicesInTriangles()
    {
        var sketches = BuildTestScriptNacaSketches();
        var converter = new CoordinateConverter(
            new Box3D(new Vec3D(-20, -30, -10), new Vec3D(200, 30, 100)));
        var output = new MeshOutput();
        LoftBuilder.GenerateLoftFromSketches(
            converter, sketches, maxDeviation: 0.03, LoftOptions.Default, output, "naca_three_section_loft", out _);

        for (int ti = 0; ti < output.Triangles.Count; ti++)
        {
            var t = output.Triangles[ti];
            Assert.False(MeshConstructionHelpers.IsDegenerateTriangle(t),
                $"Triangle {ti} repeats a vertex index: ({t.A},{t.B},{t.C})");
        }
    }
}
