using Geo;
using GeoCore;

namespace GeoTests;

public class LoftTopologyScaleTests
{
    [Fact]
    public void CollapsedQuadEndpointRetainsTheOtherValidTriangle()
    {
        Rat3Hybrid[] precise = [new(0,0,0),new(1,0,0),new(0,0,1),new(0,0,1)];
        var triangles = new List<Tri>(); var groups = new List<int>();
        LoftBuilder.EmitSideQuads(2,2,0,false,precise,triangles,groups,7);
        Assert.Single(triangles);
        Assert.Equal(new Tri(0,1,2),triangles[0]);
        Assert.Equal(7,Assert.Single(groups));
    }
    [Fact]
    public void CapRetainsDistinctRationalVerticesThatAreNearlyCoincidentInSketchSpace()
    {
        Vec2D[] loop = [new(0,0),new(.5,0),new(.5+1e-15,0),new(1,0),new(1,1),new(0,1)];
        var rim = new Rat3Hybrid[loop.Length];
        for (int i = 0; i < rim.Length; i++)
        {
            var x = new BigRational(loop[i].X); var y = new BigRational(loop[i].Y);
            rim[i] = new(new BigRationalHybrid(x.Numerator,x.Denominator),
                new BigRationalHybrid(y.Numerator,y.Denominator),BigRationalHybrid.Zero);
        }
        var output = new MeshOutput();
        var converter = new CoordinateConverter();
        LoftBuilder.EmitPlanarCapFromSketch2D(loop,rim,new(0,0,1),true,0,LoftCapTriangulationMode.Robust,
            converter,output.Vertices,output.Normals,output.UVs,output.PrecisePositions,output.Triangles,output.TriangleGroups);
        for (int i = 0; i < rim.Length; i++)
        {
            int next = (i + 1) % rim.Length;
            Assert.Single(output.Triangles, t => (t.A==i && t.B==next) || (t.B==i && t.C==next) || (t.C==i && t.A==next));
        }
        Assert.All(output.Triangles,t=>Assert.False(MeshConstructionHelpers.IsCollapsedTriangle(t,output.PrecisePositions)));
    }
    [Theory]
    [InlineData(.01, .01, 10000)]
    [InlineData(1, .0000001, .00001)]
    public void RepresentableThinFacesMustNotBeDiscarded(double width, double depth, double height)
    {
        // Exercise emission directly: extreme aspect ratios should not require
        // analytic support fitting just to verify the mesh's shared topology.
        var converter = new CoordinateConverter(new Box3D(new Vec3D(-1, -1, -1),
            new Vec3D(Math.Max(2, width + 1), Math.Max(2, depth + 1), height + 1)), 2_000_000_000);
        Vec2D[] loop = [new(0,0), new(width,0), new(width,depth), new(0,depth)];
        var output = new MeshOutput();
        var rows = new Vec3D[2][];
        var rims = new Rat3Hybrid[2][];
        for (int row = 0; row < 2; row++)
        {
            rows[row] = new Vec3D[4]; rims[row] = new Rat3Hybrid[4];
            for (int i = 0; i < 5; i++)
            {
                int k = i % 4;
                var precise = MeshConstructionHelpers.ToPrecise(converter, new(loop[k].X,loop[k].Y,height*row));
                var point = converter.Convert(precise);
                output.Vertices.Add(point); output.PrecisePositions.Add(precise);
                output.Normals.Add(default); output.UVs.Add(default);
                if (i < 4) { rows[row][i] = point; rims[row][i] = precise; }
            }
        }
        LoftBuilder.EmitSideQuads(4,2,0,true,output.PrecisePositions,output.Triangles,output.TriangleGroups,0);
        for (int row = 0; row < 2; row++)
            LoftBuilder.EmitPlanarCapFromSketch2D(loop,rims[row],new(0,0,row==0 ? -1 : 1),
                row==1,row+1,LoftCapTriangulationMode.Robust,converter,output.Vertices,output.Normals,
                output.UVs,output.PrecisePositions,output.Triangles,output.TriangleGroups);
        Assert.True(MeshAnalysis.IsWatertightMesh(output.PrecisePositions, output.Triangles));
        foreach (var tri in output.Triangles)
            Assert.False(MeshConstructionHelpers.IsCollapsedTriangle(tri,output.PrecisePositions));
        // These valid triangles would have been deleted by the old area test.
        double threshold = MeshConstructionHelpers.LoftMinSquaredCrossNormFromMaxExtent(
            MeshConstructionHelpers.MeshAxisAlignedMaxExtent(output.Vertices));
        Assert.Contains(output.Triangles, tri => MeshConstructionHelpers.IsDegenerateTriangleMesh(tri,
            output.Vertices[tri.A],output.Vertices[tri.B],output.Vertices[tri.C],threshold));
    }
}
