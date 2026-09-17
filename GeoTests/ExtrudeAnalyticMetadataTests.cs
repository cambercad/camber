using Curves;
using Geo.NurbsConstruction;
using GeoCore;
using GeoMeta;

namespace GeoTests;

public class ExtrudeAnalyticMetadataTests
{
    [Theory]
    [InlineData(0.0)]
    [InlineData(0.1)]
    public void StraightLineAndArc_DeclareGeometryOnlyWithoutTwist(double twist)
    {
        var frame = new CoordinateSystem(new Vec3D(7, 11, 13),
            new Vec3D(0, 1, 0), new Vec3D(0, 0, 1), new Vec3D(1, 0, 0));
        var line = new Line2D(new Vec2D(-2, 3), new Vec2D(4, 3));
        var arc = new Arc2D(new Vec2D(4, 0), 3, 0, Math.PI / 2);
        var source = new Dictionary<string, CurveMetaData>
        {
            ["line"] = new(CurveType.Line2D, line),
            ["arc"] = new(CurveType.Arc2D, arc)
        };
        var metadata = NurbsPatchMetadataBuilder.BuildExtrudeMetadata(source, frame,
            5, 2, "stock", twist, new[] { new Vec2D(-2, -3), new Vec2D(7, 3) },
            new List<List<List<Vec2D>>>(), new List<List<string>>());
        var lineMeta = metadata[EntityNaming.ExtrudeSide("stock", "line")];
        var arcMeta = metadata[EntityNaming.ExtrudeSide("stock", "arc")];
        Assert.False(lineMeta.IsNurbsMaterialized);
        Assert.False(arcMeta.IsNurbsMaterialized);
        if (twist != 0)
        {
            Assert.Equal(SurfaceType.Unknown, lineMeta.SurfaceType);
            Assert.Equal(SurfaceType.Unknown, arcMeta.SurfaceType);
            Assert.Null(lineMeta.PlaneParams);
            Assert.Null(arcMeta.CylinderParams);
            return;
        }
        var plane = Assert.IsType<PlaneSurfaceParams>(lineMeta.PlaneParams);
        var cylinder = Assert.IsType<CylinderSurfaceParams>(arcMeta.CylinderParams);
        Assert.Equal(SurfaceType.Planar, lineMeta.SurfaceType);
        Assert.Equal(SurfaceType.Cylindrical, arcMeta.SurfaceType);
        Assert.Equal(3, cylinder.Radius);
        Assert.Equal(7, cylinder.Height);
        Assert.InRange((cylinder.Origin - (frame.PointTo3D(arc.Center) - 2*frame.Z)).Length(), 0, 1e-14);
        Assert.InRange((cylinder.Axis - frame.Z).Length(), 0, 1e-14);
        for (int i = 0; i <= 8; i++)
            for (int j = 0; j <= 2; j++)
            {
                double u = i / 8.0, v = j / 2.0;
                var p = lineMeta.NurbsSurface.Evaluate(u, v);
                Assert.InRange(Math.Abs(Vec3DOps.Dot(p - plane.Origin, plane.Normal)), 0, 1e-13);
                var q = arcMeta.NurbsSurface.Evaluate(u, v) - cylinder.Origin;
                var radial = q - cylinder.Axis * Vec3DOps.Dot(q, cylinder.Axis);
                Assert.InRange(Math.Abs(radial.Length() - cylinder.Radius), 0, 1e-13);
            }
    }
}
