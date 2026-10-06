using Geo;
using GeoCore;
using CSG;

namespace GeoTests;

public class BooleanExactCornerInterpolationTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Fact]
    public void ExactBoundaryWeightsStayOnTheSourceEdge()
    {
        var points = new List<Rat3Hybrid> { new(0, 0, 0), new(8, 0, 0), new(0, 8, 0) };
        var weights = InterpolationHelpers.GetBarycentricWeights(new Rat3Hybrid(3, 5, 0), new Tri(0, 1, 2), points);
        Assert.Equal(0, weights.X);
        Assert.Equal(3.0 / 8, weights.Y);
        Assert.Equal(5.0 / 8, weights.Z);
    }

    [Fact]
    public void ExactAttributeInterpolationPreservesConstantSeamsAndSharedEdges()
    {
        TriangleVertexNormalUV Vertex(double u, double v) => new() { UV = new(u, v), Normal = new(0, 0, 1) };
        var positions = new List<Rat3Hybrid> { new(0, 0, 0), new(6, 0, 0), new(0, 6, 0) };
        var weights = InterpolationHelpers.GetExactBarycentricWeights(new Rat3Hybrid(4, 1, 0), new Tri(0, 1, 2), positions);
        // Separately rounded weights 1/6 + 4/6 + 1/6 sum to 1 - one ULP.
        var constant = Vertex(1, .1).InterpolateExact(Vertex(1, .7), Vertex(1, .2), weights);
        Assert.Equal(1, constant.UV.X);

        positions = [new(0, 0, 0), new(7, 0, 0), new(0, 7, 0), new(0, -7, 0)];
        var point = new Rat3Hybrid(2, 0, 0);
        var forward = InterpolationHelpers.GetExactBarycentricWeights(point, new Tri(0, 1, 2), positions);
        var reversed = InterpolationHelpers.GetExactBarycentricWeights(point, new Tri(1, 0, 3), positions);
        var first = Vertex(.1, .9).InterpolateExact(Vertex(.3, .2), Vertex(8, 9), forward);
        var second = Vertex(.3, .2).InterpolateExact(Vertex(.1, .9), Vertex(-8, -9), reversed);
        Assert.Equal(first.UV, second.UV);
    }

    [Theory]
    [InlineData(BooleanOp.Union)]
    [InlineData(BooleanOp.Subtract)]
    [InlineData(BooleanOp.Intersect)]
    public void BooleanCornerReuseMatchesUncachedExactInterpolation(BooleanOp operation)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-20), new Vec3D(20)), .1);
        var a = api.CreateCuboid(new CoordinateSystem(new Vec3D(0)), new Vec3D(4), "a").Mesh;
        var b = api.CreateCuboid(new CoordinateSystem(new Vec3D(2, 1, -1)), new Vec3D(4), "b").Mesh;
        // Distinct attributes on every source triangle and mesh exercise both
        // coincident result positions and intentional UV/normal seams.
        SetAttributes(a, 0);
        SetAttributes(b, 100);
        Resolver.Resolve(operation, a.PrecisionPositions, a.Triangles, b.PrecisionPositions, b.Triangles,
            out var points, out var triangles, out var sources);
        var expected = new MeshNormalUV
        {
            PrecisionPositions = points, Positions = api.Converter.Convert(points), Triangles = triangles,
            TrianglesEx = new List<MeshTriangle<TriangleVertexNormalUV>>(triangles.Count)
        };
        for (int i = 0; i < triangles.Count; i++)
        {
            var source = sources[i].MeshOrigin == MeshOrigin.MeshA ? a : b;
            int sourceIndex = sources[i].SourceTriangleIndex;
            var sourceData = source.TrianglesEx[sourceIndex];
            var tri = triangles[i];
            TriangleVertexNormalUV Corner(int vertex) => sourceData.InterpolateExact(
                InterpolationHelpers.GetExactBarycentricWeights(points[vertex], source.Triangles[sourceIndex], source.PrecisionPositions));
            var data = new MeshTriangle<TriangleVertexNormalUV>
            {
                V0 = Corner(tri.A), V1 = Corner(tri.B), V2 = Corner(tri.C), GroupId = sourceData.GroupId
            };
            var geometricNormal = Vec3DOps.Cross(expected.Positions[tri.B] - expected.Positions[tri.A],
                expected.Positions[tri.C] - expected.Positions[tri.A]);
            if (Vec3DOps.Dot(geometricNormal, data.V0.Normal + data.V1.Normal + data.V2.Normal) < 0)
            {
                data.V0.Normal = -data.V0.Normal;
                data.V1.Normal = -data.V1.Normal;
                data.V2.Normal = -data.V2.Normal;
            }
            expected.TrianglesEx.Add(data);
        }
        var actual = MeshNormalUV.BooleanOperation(a, b, operation, api.Converter);
        Assert.NotEmpty(actual.Triangles);
        Assert.Equal(CanonicalCorners(expected), CanonicalCorners(actual));

        static void SetAttributes(MeshNormalUV mesh, double offset)
        {
            for (int i = 0; i < mesh.TrianglesEx.Count; i++)
            {
                var data = mesh.TrianglesEx[i];
                data.V0.UV = new Vec2D(offset + i + .1, .9);
                data.V1.UV = new Vec2D(offset + i + .3, .2);
                data.V2.UV = new Vec2D(offset + i + .7, .4);
                data.V0.Normal += new Vec3D(.1, .2, .3);
                data.V1.Normal += new Vec3D(.2, .3, .1);
                data.V2.Normal += new Vec3D(.3, .1, .2);
                mesh.TrianglesEx[i] = data;
            }
        }

        static string[] CanonicalCorners(MeshNormalUV mesh)
        {
            string Bits(double value) => BitConverter.DoubleToInt64Bits(value).ToString("X16");
            string Corner(int vertex, TriangleVertexNormalUV data)
            {
                var p = mesh.PrecisionPositions[vertex];
                return $"{p.X}|{p.Y}|{p.Z}|{Bits(data.UV.X)}|{Bits(data.UV.Y)}|" +
                    $"{Bits(data.Normal.X)}|{Bits(data.Normal.Y)}|{Bits(data.Normal.Z)}";
            }
            var rows = new string[mesh.Triangles.Count];
            for (int i = 0; i < rows.Length; i++)
            {
                var tri = mesh.Triangles[i];
                var data = mesh.TrianglesEx[i];
                string a = Corner(tri.A, data.V0), b = Corner(tri.B, data.V1), c = Corner(tri.C, data.V2);
                string[] rotations = { a + ";" + b + ";" + c, b + ";" + c + ";" + a, c + ";" + a + ";" + b };
                Array.Sort(rotations, StringComparer.Ordinal);
                rows[i] = data.GroupId + ":" + rotations[0];
            }
            Array.Sort(rows, StringComparer.Ordinal);
            return rows;
        }
    }

    [Fact]
    public void TrimmedImpellerCornersStayWithinTheirAuthoredSupportDomain()
    {
        var (_, _, _, joined) = ImpellerLoftSupportTests.Fixture();
        int side = joined.extendedNameToGroupId["blade_loft-Side"];
        foreach (var triangle in joined.Mesh.TrianglesEx.Where(t => t.GroupId == side))
        {
            foreach (var uv in new[] { triangle.V0.UV, triangle.V1.UV, triangle.V2.UV })
            {
                Assert.InRange(uv.X, 0, 1);
                Assert.InRange(uv.Y, 0, 1);
            }
        }
    }
}
