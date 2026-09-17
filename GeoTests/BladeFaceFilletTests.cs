using Geo;
using GeoCore;
using Curves;

namespace GeoTests;

public class BladeFaceFilletTests
{
    [Fact]
    public void PlanarOffsetPreservesExactPlanarityAndSourceVertices()
    {
        var cc = new CoordinateConverter(new Box3D(new Vec3D(-10), new Vec3D(10)));
        var x = new BigRationalHybrid(10000, 3);
        var y = new BigRationalHybrid(10000, 7);
        var origin = new Rat3Hybrid(500000,500000,500000);
        var points = new List<Rat3Hybrid> { origin, origin+new Rat3Hybrid(x,BigRationalHybrid.Zero,-x),
            origin+new Rat3Hybrid(BigRationalHybrid.Zero,y,-y) };
        var normal = new Vec3D(1,1,1).Normalized();
        var surface = new UVSurface(cc.Convert(points), Enumerable.Repeat(normal,3).ToList(),
            new List<Vec2D>{new(0,0),new(1,0),new(0,1)}, new List<Tri>{new(0,1,2)}, points);
        var offset = surface.GetOffsetSurface(.6,cc);
        Assert.True(offset.IsSurfacePlanar());
        var translation = offset.PointsPrecise[0] - points[0];
        for (int i=0; i<points.Count; ++i)
            Assert.True(translation == offset.PointsPrecise[i]-points[i]);
        Assert.Equal(origin, surface.PointsPrecise[0]);
        Assert.InRange(Vec3DOps.Dot(offset.Points[0]-surface.Points[0],normal),
            .6-2*cc.SmallestUnit(), .6+2*cc.SmallestUnit());
    }
    [Fact]
    public void CuboidFaceRoundAfterEqualRadiusPerimeterRound()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(0), new Vec3D(3)), 1e-4);
        var blank = api.CreateCuboid(new CoordinateSystem(new Vec3D(0,0,0)), new Vec3D(1,1,3), "a");
        var rounded = api.Fillet(blank, new List<string> { "[a-Line3,a-Line4]" }, .5, .0001, "rounded");
        var faces = api.Fillet(rounded, new List<string> { "[a-Line3,a-ExtrudeTop]" }, .5, .0001, "faces");
        MeshTestHelpers.AssertValidMesh(faces.Mesh.Positions, faces.Mesh.Triangles);
    }
    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(.7, false)]
    [InlineData(.7, true)]
    public void FaceRoundsAfterPerimeterRoundsRemainWatertight(double angle, bool atOrigin)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-420,-300,-50), new Vec3D(1420,300,1120)), .2);
        var frame = new CoordinateSystem(atOrigin ? new Vec3D(0,0,100) : new Vec3D(1033,207,878),
            new Vec3D(Math.Cos(angle),Math.Sin(angle),0), new Vec3D(0,0,1),
            new Vec3D(Math.Sin(angle),-Math.Cos(angle),0));
        var points = new Vec2D[] { new(-6,5),new(4,7),new(8,2),new(12,-15),new(14,-42),new(9,-69),new(2,-87),new(-4,-91),new(-7,-88),new(-1,-73),new(4,-47),new(4,-22),new(-6,-2) };
        var sketch = new PlotterSketcherCoordSys("profile", frame, points[0]);
        for (int i=1;i<=points.Length;i++) sketch.AppendLine(points[i%points.Length].X, points[i%points.Length].Y);
        var blank = api.Extrude(sketch,14,name:"blade");
        var edges = Enumerable.Range(1,points.Length).Select(i=>$"[blade-Line{(i==1?points.Length:i-1)},blade-Line{i}]").ToList();
        var rounded = api.Fillet(blank,edges,1,.02,"rounded");
        var cylinders = rounded.surfaceMetaData.Values.Where(m => m.CylinderParams != null).ToList();
        Assert.Equal(points.Length, cylinders.Count);
        Assert.All(cylinders, m => Assert.Equal(1, m.CylinderParams.Radius));
        var endClosures = rounded.surfaceMetaData.Where(m => m.Key.StartsWith("BlendCorner_")).ToList();
        Assert.Equal(6, endClosures.Count);
        Assert.All(endClosures, m => Assert.NotNull(m.Value.PlaneParams));
        var faces = api.Fillet(rounded, new List<string> { "[blade-Line4,blade-ExtrudeTop]", "[blade-Line4,blade-ExtrudeBottom]" }, .6, .02, "face_rounded");
        MeshTestHelpers.AssertValidMesh(faces.Mesh.Positions,faces.Mesh.Triangles);
        var rimBlends = faces.surfaceMetaData.Where(m => m.Key.StartsWith("BlendEdge_") && m.Key.Contains("Extrude")).ToList();
        Assert.NotEmpty(rimBlends);
        Assert.All(rimBlends, m => Assert.Null(m.Value.CylinderParams));
        double Volume(AnchorMesh solid) => Math.Abs(solid.Mesh.Triangles.Sum(t =>
            Vec3DOps.Dot(solid.Mesh.Positions[t.A], Vec3DOps.Cross(solid.Mesh.Positions[t.B], solid.Mesh.Positions[t.C])))) / 6;
        Assert.InRange(Volume(rounded) - Volume(faces), 1, 500);
    }
}
