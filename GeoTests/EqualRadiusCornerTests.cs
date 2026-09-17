using Geo;
using GeoCore;

namespace GeoTests;

public class EqualRadiusCornerTests
{
    static double Volume(AnchorMesh solid)
    {
        var origin = solid.Mesh.Positions[solid.Mesh.Triangles[0].A];
        return Math.Abs(solid.Mesh.Triangles.Sum(t => Vec3DOps.Dot(solid.Mesh.Positions[t.A] - origin,
            Vec3DOps.Cross(solid.Mesh.Positions[t.B] - origin, solid.Mesh.Positions[t.C] - origin)))) / 6;
    }

    static EdgeGraph Graph(AnchorMesh solid) => new EdgeGraph(solid.Mesh.Triangles,
        solid.Mesh.GetTriangleGroups(), solid.Mesh.Positions, solid.Mesh.PrecisionPositions,
        solid.groupIdToExtendedName);

    [Theory]
    [InlineData(0, false, false)]
    [InlineData(0, true, false)]
    [InlineData(.7, false, true)]
    [InlineData(.7, true, true)]
    public void ThreeTangentRimSegmentsAtEqualRadiusProduceSphericalCorner(double angle, bool bottom, bool translated)
    {
        var origin = translated ? new Vec3D(33, 17, 8) : new Vec3D(0);
        var api = new GeoAPI(new Box3D(origin - new Vec3D(2), origin + new Vec3D(5)), 1e-4);
        var frame = new CoordinateSystem(origin, new Vec3D(Math.Cos(angle), 0, -Math.Sin(angle)), new Vec3D(0, 1, 0), new Vec3D(Math.Sin(angle), 0, Math.Cos(angle)));
        var blank = api.CreateCuboid(frame, new Vec3D(1, 1, 3), "a");
        var rounded = api.Fillet(blank, new List<string> { "[a-Line3,a-Line4]" }, .5, .0001, "rounded");
        var top = rounded.groupIdToExtendedName.Single(p => p.Value == "a-Extrude" + (bottom ? "Bottom" : "Top")).Key;
        bool Wanted(int id)
        {
            var name = rounded.groupIdToExtendedName[id];
            return name == "a-Line3" || name == "a-Line4" ||
                (rounded.surfaceMetaData.TryGetValue(name, out var metadata) && metadata.CylinderParams != null);
        }
        var edges = Graph(rounded).Edges.Where(e => (e.GroupIdA == top && Wanted(e.GroupIdB)) ||
            (e.GroupIdB == top && Wanted(e.GroupIdA))).Select(e => e.Name).ToList();
        Assert.Equal(3, edges.Count);
        var faces = api.Fillet(rounded, edges, .5, .0001, "faces");
        MeshTestHelpers.AssertValidMesh(faces.Mesh.Positions, faces.Mesh.Triangles);
        // Integrate the retained footprint through the spherical cap, including
        // its two adjoining quarter-cylinder strips, independently of the mesh.
        double r = .5, h = 3;
        double expected = (1 - r * r + Math.PI * r * r / 4) * (h - r) + (1 - r) * (1 - r) * r +
            Math.PI * (1 - r) * r * r / 2 + Math.PI * r * r * r / 6;
        Assert.InRange(Volume(faces), expected - .002, expected + .002);
    }
}
