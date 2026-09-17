using System.Numerics;
using GeoCore;

namespace GeoTests;

public class PreciseWatertightTests
{
    private static List<Tri> Tetrahedron(int offset = 0) =>
        [new(offset, offset + 2, offset + 1), new(offset, offset + 1, offset + 3),
         new(offset + 1, offset + 2, offset + 3), new(offset + 2, offset, offset + 3)];

    [Fact]
    public void DistinctRationalEdgesRemainDistinctBelowDisplayPrecision()
    {
        var epsilon = new BigRationalHybrid(BigInteger.One, BigInteger.One << 80);
        var one = new BigRationalHybrid(1);
        var shifted = one + epsilon;
        List<Rat3Hybrid> positions = [new(1, 0, 0), new(1, 1, 0), new(0, 0, 0), new(1, 0, 1),
            new(shifted, BigRationalHybrid.Zero, BigRationalHybrid.Zero), new(shifted, BigRationalHybrid.One, BigRationalHybrid.Zero), new(2, 0, 0), new(shifted, BigRationalHybrid.Zero, new BigRationalHybrid(-1))];
        var triangles = Tetrahedron();
        triangles.AddRange(Tetrahedron(4));
        Assert.True(MeshAnalysis.IsWatertightMesh(positions, triangles));
        var doubles = positions.Select(p => new Vec3D(p.X.ToDouble(), p.Y.ToDouble(), p.Z.ToDouble())).ToList();
        Assert.False(MeshAnalysis.IsWatertightMesh(doubles, triangles));
        triangles.RemoveAt(0);
        Assert.False(MeshAnalysis.IsWatertightMesh(positions, triangles));
    }

    [Fact]
    public void EqualRationalCoordinatesWeldAndRealNonmanifoldEdgesStillFail()
    {
        List<Rat3Hybrid> original = [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, 0, 1)];
        var points = new List<Rat3Hybrid>();
        var triangles = new List<Tri>();
        foreach (var triangle in Tetrahedron())
        {
            int start = points.Count;
            points.Add(original[triangle.A]);
            points.Add(original[triangle.B]);
            points.Add(original[triangle.C]);
            triangles.Add(new Tri(start, start + 1, start + 2));
        }
        var saved = points.ToArray();
        Assert.True(MeshAnalysis.IsWatertightMesh(points, triangles));
        Assert.Equal(saved, points);
        triangles.Add(triangles[0]);
        Assert.False(MeshAnalysis.IsWatertightMesh(points, triangles));
        Assert.False(MeshAnalysis.IsWatertightMesh(points, triangles, allowTouch: true));
    }
}
