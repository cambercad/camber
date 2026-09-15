using CSG;
using Geo;
using GeoCore;

namespace GeoTests;

/// <summary>
/// Exact symmetric chamfer volumes for 90° planar face pairs (distance measured along each face).
/// </summary>
internal static class ChamferAnalytic
{
    /// <summary>Material removed by chamfering one straight edge of length <paramref name="edgeLength"/>.</summary>
    public static double RemovedVolumeSingleEdge(double edgeLength, double chamferDistance) =>
        edgeLength * chamferDistance * chamferDistance / 2.0;

    /// <summary>Remaining volume after chamfering one edge of a box.</summary>
    public static double BoxVolumeAfterSingleEdgeChamfer(double boxVolume, double edgeLength, double chamferDistance) =>
        boxVolume - RemovedVolumeSingleEdge(edgeLength, chamferDistance);

    /// <summary>
    /// Cube of side <paramref name="side"/> after chamfering three mutually perpendicular edges at one corner.
    /// Exact for symmetric face-distance chamfer on a cube: removed = (3/2)Ld² − (2/3)d³.
    /// </summary>
    public static double CubeVolumeAfterThreeEdgeCornerChamfer(double side, double chamferDistance)
    {
        double d = chamferDistance;
        double removed = 1.5 * side * d * d - (2.0 / 3.0) * d * d * d;
        return side * side * side - removed;
    }

    /// <summary>
    /// Rectangular corner with perpendicular edge lengths meeting at the chamfered vertex.
    /// Exact when two short edges have equal length <paramref name="shortSide"/> and the third has
    /// <paramref name="longSide"/> (cuboid 1×1×L corner). Removed = (d²/2)(2·short + long) − (2/3)d³.
    /// </summary>
    public static double CuboidCornerVolumeAfterThreeEdgeChamfer(
        double shortSide, double longSide, double chamferDistance, double boxVolume)
    {
        double d = chamferDistance;
        double sumEdgeLengths = 2.0 * shortSide + longSide;
        double removed = 0.5 * d * d * sumEdgeLengths - (2.0 / 3.0) * d * d * d;
        return boxVolume - removed;
    }
}
