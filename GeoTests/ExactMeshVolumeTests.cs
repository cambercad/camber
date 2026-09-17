using GeoCore;

namespace GeoTests;

public class ExactMeshVolumeTests
{
    private static BigRationalHybrid Rational(int numerator, int denominator) => new(numerator, denominator);

    [Theory]
    [InlineData(1)]
    [InlineData(512)]
    public void RationalFacetSubdivisionPreservesExactSignedVolume(int divisions)
    {
        var zero = BigRationalHybrid.Zero;
        var translation = new Rat3Hybrid(Rational(123, 37), Rational(-71, 29), Rational(19, 31));
        var x = new Rat3Hybrid(Rational(3, 7), zero, zero);
        var y = new Rat3Hybrid(zero, Rational(5, 11), zero);
        var z = new Rat3Hybrid(zero, zero, Rational(13, 17));
        List<Rat3Hybrid> points = [translation, translation + z];
        for (int i = 0; i <= divisions; i++)
            points.Add(translation + x * Rational(divisions - i, divisions) + y * Rational(i, divisions));
        var triangles = new List<Tri>();
        for (int i = 0; i < divisions; i++)
        {
            triangles.Add(new Tri(2 + i, 3 + i, 1));
            triangles.Add(new Tri(0, 3 + i, 2 + i));
        }
        triangles.Add(new Tri(0, 2, 1));
        triangles.Add(new Tri(0, 1, points.Count - 1));
        var before = points.Select(p => new Rat3Hybrid(in p)).ToArray();
        var expected = Rational(3 * 5 * 13, 6 * 7 * 11 * 17);
        var volume = MeshAnalysis.ComputeSignedMeshVolume(points, triangles);
        Assert.True((volume - expected).Sign() == 0);
        for (int i = 0; i < points.Count; i++)
        {
            var difference = points[i] - before[i];
            Assert.Equal(0, difference.X.Sign());
            Assert.Equal(0, difference.Y.Sign());
            Assert.Equal(0, difference.Z.Sign());
        }
        var reversed = triangles.Select(t => new Tri(t.A, t.C, t.B)).ToList();
        Assert.True((MeshAnalysis.ComputeSignedMeshVolume(points, reversed) + expected).Sign() == 0);
    }
}
