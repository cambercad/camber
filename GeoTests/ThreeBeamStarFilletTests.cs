using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

public sealed class ThreeBeamStarFilletTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void FilletsTheSixUpperArmsOfThreeRolledBeams()
    {
        const double length = 40;
        const double width = 8;
        const double height = 8;
        const double roll = Math.PI / 4;
        const double tolerance = .001;
        var api = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), tolerance);

        AnchorMesh star = null;
        for (int i = 0; i < 3; i++)
        {
            double angle = i * 2 * Math.PI / 3;
            CoordinateSystem frame = BeamFrame(angle, roll, length, width, height);
            var beam = api.CreateCuboid(frame, new Vec3D(length, width, height), $"beam_{i + 1}");
            var ridge = GetUpperLongitudinalEdges(beam, frame.X, tolerance);
            Assert.Single(ridge);
            double originalVolume = Volume(beam);
            beam = api.Fillet(beam, ridge, .8, tolerance, $"rounded_beam_{i + 1}");
            Assert.True(MeshAnalysis.IsWatertightMesh(beam.Mesh.PrecisionPositions, beam.Mesh.Triangles));
            Assert.True(Volume(beam) < originalVolume);
            star = star == null ? beam : api.Boolean(star, beam, BooleanOp.Union, $"star_{i + 1}");
        }

        Assert.NotNull(star);
        star.EnsureCoplanarPostProcessed();
        Assert.True(MeshAnalysis.IsWatertightMesh(star.Mesh.PrecisionPositions, star.Mesh.Triangles));
        Assert.True(Volume(star) > 0);
    }

    [Fact]
    public void ThreeBeamAllEdgeFilletReproducesOpenBlendBoundary()
    {
        const double length = 40;
        const double section = 8;
        const double tolerance = .001;
        var api = new GeoAPI(new Box3D(new Vec3D(-30), new Vec3D(30)), tolerance);
        AnchorMesh star = null;
        for (int i = 0; i < 3; i++)
        {
            var frame = BeamFrame(i * 2 * Math.PI / 3, Math.PI / 4,
                length, section, section);
            var beam = api.CreateCuboid(frame, new Vec3D(length, section, section), $"beam_{i + 1}");
            star = star == null ? beam : api.Boolean(star, beam, BooleanOp.Union, $"star_{i + 1}");
        }

        // The three source ridges are split by the union; the six extra edges
        // are the face-to-face junctions where the bars cross.
        string[] expectedPairs =
        {
            "[beam_3-Line1_2,beam_3-ExtrudeTop_2]",
            "[beam_2-Line1_1,beam_2-ExtrudeTop_1]",
            "[beam_1-Line1_1,beam_1-ExtrudeTop_1]",
            "[beam_3-Line1_1,beam_3-ExtrudeTop_1]",
            "[beam_2-Line1_2,beam_2-ExtrudeTop_2]",
            "[beam_1-Line1_2,beam_1-ExtrudeTop_2]",
            "[beam_1-Line1_2,beam_3-Line1_2]",
            "[beam_3-ExtrudeTop_2,beam_2-ExtrudeTop_1]",
            "[beam_1-Line1_1,beam_2-Line1_1]",
            "[beam_1-ExtrudeTop_1,beam_3-ExtrudeTop_1]",
            "[beam_2-Line1_2,beam_3-Line1_1]",
            "[beam_1-ExtrudeTop_2,beam_2-ExtrudeTop_2]"
        };
        var edgeReferences = new List<string>();
        for (int i = 0; i < star.GroupEdges.Count; i++)
        {
            string reference = star.GetEdgeReference(i);
            for (int j = 0; j < expectedPairs.Length; j++)
                if (reference.Contains(expectedPairs[j], StringComparison.Ordinal))
                {
                    edgeReferences.Add(reference);
                    break;
                }
        }
        Assert.Equal(expectedPairs.Length, edgeReferences.Count);

        var error = Assert.Throws<InvalidOperationException>(() =>
            api.Fillet(star, edgeReferences, .1, tolerance, "rounded_star_all_edges"));
        Assert.Contains("interior open boundary", error.Message, StringComparison.Ordinal);
    }

    private static CoordinateSystem BeamFrame(double angle, double roll, double length, double width, double height)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        double cr = Math.Cos(roll), sr = Math.Sin(roll);
        var x = new Vec3D(c, 0, s);
        // Apply the 45° roll about local X, then orient that X-axis about world Y.
        var y = new Vec3D(-s * sr, cr, c * sr);
        var z = new Vec3D(-s * cr, -sr, c * cr);
        var corner = x * (-length / 2) + y * (-width / 2) + z * (-height / 2);
        return new CoordinateSystem(corner, x, y, z);
    }

    private static List<string> GetUpperLongitudinalEdges(AnchorMesh beam, Vec3D axis, double tolerance)
    {
        beam.EnsureCoplanarPostProcessed();
        double maxY = double.NegativeInfinity;
        foreach (Vec3D point in beam.Mesh.Positions)
            maxY = Math.Max(maxY, point.Y);
        var result = new List<string>();
        for (int edgeIndex = 0; edgeIndex < beam.GroupEdges.Count; edgeIndex++)
        {
            var edge = beam.GroupEdges[edgeIndex];
            foreach (LineStrip3D strip in edge.LineStrips3D)
            {
                if (strip.Points.Count < 2 || strip.TotalLength < 1)
                    continue;
                bool allAtHeight = true;
                for (int i = 0; i < strip.Points.Count; i++)
                    if (Math.Abs(strip.Points[i].Y - maxY) > tolerance * 2)
                    {
                        allAtHeight = false;
                        break;
                    }
                if (!allAtHeight)
                    continue;
                Vec3D direction = strip.Points[strip.Points.Count - 1] - strip.Points[0];
                double directionLength = Math.Sqrt(direction.LengthSquared());
                if (directionLength > 0 && Math.Abs(Vec3DOps.Dot(direction / directionLength, axis)) > .999)
                    result.Add(beam.GetEdgeReference(edgeIndex));
            }
        }
        return result;
    }

    private static double Volume(AnchorMesh mesh)
    {
        double volume = 0;
        foreach (Tri triangle in mesh.Mesh.Triangles)
            volume += Vec3DOps.Dot(mesh.Mesh.Positions[triangle.A],
                Vec3DOps.Cross(mesh.Mesh.Positions[triangle.B], mesh.Mesh.Positions[triangle.C]));
        return Math.Abs(volume / 6);
    }
}
