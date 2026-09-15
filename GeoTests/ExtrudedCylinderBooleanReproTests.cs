using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

/// <summary>
/// Regression: two <see cref="GeoAPI.CreateCylinder"/> solids in orthogonal right-handed frames;
/// union should stay consistently oriented and yield positive signed volume when watertight.
/// </summary>
public sealed class ExtrudedCylinderBooleanReproTests : IDisposable
{
    public void Dispose() => GeoAPI.Clear();

    [Fact]
    public void Boolean_OrthogonalExtrudedCylinders_Union_IsConsistentlyOrientedAndPositiveVolume()
    {
        var api = new GeoAPI(new Box3D(new Vec3D(-2), new Vec3D(2)), 1e-4);

        const double maxDev = 0.008;
        const double len = 2.5;
        const double radius = 0.25;

        // Default cylinder: extrusion along world +Z, symmetric about origin in Z.
        var cylZ = api.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, -len / 2)), radius, len, maxDev, "cylZ");

        // Second cylinder: extrusion along world +X (local +Z mapped to +X).
        // Local +Z → world +X; X×Y must equal Z (right-handed).
        var csX = new CoordinateSystem(
            new Vec3D(-len / 2, 0, 0),
            new Vec3D(0, 1, 0),
            new Vec3D(0, 0, 1),
            new Vec3D(1, 0, 0));
        var cylX = api.CreateCylinder(csX, radius, len, maxDev, "cylX");

        Assert.True(
            MeshAnalysis.AreTrianglesConsistentlyOriented(cylZ.Mesh.PrecisionPositions, cylZ.Mesh.Triangles),
            "precondition: single Z cylinder should be consistently oriented");
        Assert.True(
            MeshAnalysis.AreTrianglesConsistentlyOriented(cylX.Mesh.PrecisionPositions, cylX.Mesh.Triangles),
            "precondition: single X cylinder should be consistently oriented");

        var unionMesh = MeshNormalUV.BooleanOperation(cylZ.Mesh, cylX.Mesh, BooleanOp.Union, api.Converter);

        Assert.True(
            MeshAnalysis.AreTrianglesConsistentlyOriented(unionMesh.PrecisionPositions, unionMesh.Triangles),
            "union of orthogonal extruded cylinders (RH frames) should be consistently oriented");

        double unionVol = MeshAnalysis.ComputeSignedMeshVolume(unionMesh.Positions, unionMesh.Triangles);
        Assert.True(unionVol > 0, $"union signed volume should be positive (outward winding), got {unionVol:F6}");
    }
}
