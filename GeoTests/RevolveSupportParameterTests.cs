using Curves;
using Geo;
using GeoCore;

namespace GeoTests;

public class RevolveSupportParameterTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear(resetNameCounters: false);

    [Theory]
    [InlineData(1.0, false)]
    [InlineData(1.0, true)]
    [InlineData(.35, false)]
    [InlineData(.35, true)]
    public void FaceParametersFollowAuthoredSegmentsAndRotation(double turns, bool reverse)
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-10), new Vec3D(10)), .01);
        var frame = new Plane3D(new Vec3D(0), new Vec3D(1, 0, 0), new Vec3D(0, 1, 0), new Vec3D(0, 0, 1));
        var sketch = api.GetPlotterSketcher(frame, "meridian");
        Vec2D[] points = [new(0, 2), new(3, 2), new(3, 4), new(0, 4)];
        if (reverse) Array.Reverse(points);
        for (int i = 0; i < points.Length; i++)
            sketch.AddLine(points[i], points[(i + 1) % points.Length]).Name = "side" + i;
        var body = api.Revolve(sketch, turns * 2 * Math.PI, name: "turned");
        for (int i = 0; i < body.Mesh.Triangles.Count; i++)
        {
            var triangle = body.Mesh.Triangles[i];
            var data = body.Mesh.TrianglesEx[i];
            string name = body.groupIdToExtendedName[data.GroupId];
            if (!name.Contains("-side")) continue;
            var support = body.surfaceMetaData[name].NurbsSurface;
            var vertices = new[] { triangle.A, triangle.B, triangle.C };
            var uv = new[] { data.V0.UV, data.V1.UV, data.V2.UV };
            for (int corner = 0; corner < 3; corner++)
                Assert.InRange((support.Evaluate(uv[corner].X, uv[corner].Y) -
                    body.Mesh.Positions[vertices[corner]]).Length(), 0, 1e-5);
        }
    }
}
