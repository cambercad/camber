using CSG;
using Curves;
using GeoCore;
using GeoMeta;

namespace Geo;

public partial class GeoAPI
{
    /// <summary>Cut a flat-bottom drilled hole. The mouth frame's +Z axis points into the solid.</summary>
    /// <param name="depth">Distance from the mouth to the flat bottom, or null to drill through the whole solid.</param>
    [APIDescription("DrillHole(solid: AnchorMesh, mouth: CoordinateSystem, diameter: float, depth: float = None, maxDeviation: float = -1, name: str = None) -> AnchorMesh\nCuts a flat-bottom hole along mouth.Z into the solid. Omit depth to cut through the solid.")]
    public AnchorMesh DrillHole(AnchorMesh solid, CoordinateSystem mouth, double diameter,
        double? depth = null, double maxDeviation = -1, string name = null)
    {
        ValidateHoleInput(solid, mouth, diameter, depth, maxDeviation);
        name ??= GenerateName("Hole");
        double end = ResolveHoleEnd(solid, mouth, depth);
        double lead = HoleLead();
        return CutHoleProfile(solid, mouth,
            [new Vec2D(-lead, 0), new Vec2D(-lead, diameter / 2),
             new Vec2D(end, diameter / 2), new Vec2D(end, 0)], maxDeviation, name);
    }

    /// <summary>Cut a drilled hole with a flat, larger-diameter counterbore at its mouth.</summary>
    [APIDescription("CounterboreHole(solid: AnchorMesh, mouth: CoordinateSystem, diameter: float, depth: float, counterboreDiameter: float, counterboreDepth: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh\nCuts a cylindrical hole and larger flat-bottom recess. Omit depth to drill through; mouth.Z points into the solid.")]
    public AnchorMesh CounterboreHole(AnchorMesh solid, CoordinateSystem mouth, double diameter,
        double? depth, double counterboreDiameter, double counterboreDepth,
        double maxDeviation = -1, string name = null)
    {
        ValidateHoleInput(solid, mouth, diameter, depth, maxDeviation);
        PositiveFinite(counterboreDiameter, nameof(counterboreDiameter));
        PositiveFinite(counterboreDepth, nameof(counterboreDepth));
        if (counterboreDiameter <= diameter)
            throw new ArgumentOutOfRangeException(nameof(counterboreDiameter), "Counterbore diameter must exceed the bore diameter.");
        double end = ResolveHoleEnd(solid, mouth, depth);
        if (counterboreDepth >= end)
            throw new ArgumentOutOfRangeException(nameof(counterboreDepth), "Counterbore depth must be less than the drilled depth.");
        name ??= GenerateName("CounterboreHole");
        double lead = HoleLead();
        return CutHoleProfile(solid, mouth,
            [new Vec2D(-lead, 0), new Vec2D(-lead, counterboreDiameter / 2),
             new Vec2D(counterboreDepth, counterboreDiameter / 2),
             new Vec2D(counterboreDepth, diameter / 2),
             new Vec2D(end, diameter / 2), new Vec2D(end, 0)], maxDeviation, name);
    }

    /// <summary>Cut a drilled hole with an included-angle conical countersink at its mouth.</summary>
    [APIDescription("CountersinkHole(solid: AnchorMesh, mouth: CoordinateSystem, diameter: float, depth: float, countersinkDiameter: float, includedAngle: float, maxDeviation: float = -1, name: str = None) -> AnchorMesh\nCuts a cylindrical hole and conical countersink. includedAngle is in radians. Omit depth to drill through; mouth.Z points into the solid.")]
    public AnchorMesh CountersinkHole(AnchorMesh solid, CoordinateSystem mouth, double diameter,
        double? depth, double countersinkDiameter, double includedAngle,
        double maxDeviation = -1, string name = null)
    {
        ValidateHoleInput(solid, mouth, diameter, depth, maxDeviation);
        PositiveFinite(countersinkDiameter, nameof(countersinkDiameter));
        if (countersinkDiameter <= diameter)
            throw new ArgumentOutOfRangeException(nameof(countersinkDiameter), "Countersink diameter must exceed the bore diameter.");
        if (!double.IsFinite(includedAngle) || includedAngle <= 0 || includedAngle >= Math.PI)
            throw new ArgumentOutOfRangeException(nameof(includedAngle), "Included angle must be between zero and pi radians.");
        double slope = Math.Tan(includedAngle / 2);
        double sinkDepth = (countersinkDiameter - diameter) / (2 * slope);
        double end = ResolveHoleEnd(solid, mouth, depth);
        if (!double.IsFinite(sinkDepth) || sinkDepth >= end)
            throw new ArgumentOutOfRangeException(nameof(countersinkDiameter), "Countersink must end before the drilled bottom.");
        name ??= GenerateName("CountersinkHole");
        double lead = HoleLead();
        return CutHoleProfile(solid, mouth,
            [new Vec2D(-lead, 0), new Vec2D(-lead, countersinkDiameter / 2 + lead * slope),
             new Vec2D(sinkDepth, diameter / 2),
             new Vec2D(end, diameter / 2), new Vec2D(end, 0)], maxDeviation, name);
    }

    private AnchorMesh CutHoleProfile(AnchorMesh solid, CoordinateSystem mouth,
        IReadOnlyList<Vec2D> profile, double maxDeviation, string name)
    {
        // Revolve uses local X as its axis and local Y as its radial direction.
        var meridian = new CoordinateSystem(mouth.Origin, mouth.Z, mouth.X, mouth.Y);
        var sketch = new PlotterSketcherCoordSys(name + "_meridian", meridian);
        for (int i = 0; i < profile.Count; ++i)
            sketch.AddLine(profile[i], profile[(i + 1) % profile.Count]);
        var cutter = Revolve(sketch, 2 * Math.PI, maxDeviation, name + "_cutter");
        return Boolean(solid, cutter, BooleanOp.Subtract, name);
    }

    private static void PositiveFinite(double value, string parameter)
    {
        if (!double.IsFinite(value) || value <= 0)
            throw new ArgumentOutOfRangeException(parameter, "Dimension must be finite and positive.");
    }

    private static void ValidateHoleInput(AnchorMesh solid, CoordinateSystem mouth,
        double diameter, double? depth, double maxDeviation)
    {
        ArgumentNullException.ThrowIfNull(solid);
        if (!solid.IsVolume) throw new ArgumentException("Hole requires a closed solid.", nameof(solid));
        PositiveFinite(diameter, nameof(diameter));
        if (depth.HasValue) PositiveFinite(depth.Value, nameof(depth));
        if (maxDeviation != -1) PositiveFinite(maxDeviation, nameof(maxDeviation));
        static bool Finite(Vec3D v) => double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);
        if (!Finite(mouth.Origin) || !Finite(mouth.X) || !Finite(mouth.Y) || !Finite(mouth.Z) ||
            Math.Abs(mouth.X.LengthSquared() - 1) > CoordinateSystem.OrthonormalUnitSquaredTolerance ||
            Math.Abs(mouth.Y.LengthSquared() - 1) > CoordinateSystem.OrthonormalUnitSquaredTolerance ||
            Math.Abs(mouth.Z.LengthSquared() - 1) > CoordinateSystem.OrthonormalUnitSquaredTolerance ||
            Math.Abs(Vec3DOps.Dot(mouth.X, mouth.Y)) > CoordinateSystem.OrthonormalDotTolerance ||
            Math.Abs(Vec3DOps.Dot(mouth.Y, mouth.Z)) > CoordinateSystem.OrthonormalDotTolerance ||
            Math.Abs(Vec3DOps.Dot(mouth.Z, mouth.X)) > CoordinateSystem.OrthonormalDotTolerance ||
            Vec3DOps.Dot(Vec3DOps.Cross(mouth.X, mouth.Y), mouth.Z) < 0)
            throw new ArgumentException("Mouth must be a finite, right-handed orthonormal frame.", nameof(mouth));
    }

    private double HoleLead() => Math.Max(4 * converter.SmallestUnit(), 1e-8);

    private double ResolveHoleEnd(AnchorMesh solid, CoordinateSystem mouth, double? depth)
    {
        if (depth.HasValue) return depth.Value;
        if (solid.Mesh.Positions.Count == 0)
            throw new ArgumentException("Solid has no vertices.", nameof(solid));
        double farthest = solid.Mesh.Positions.Max(p => Vec3DOps.Dot(p - mouth.Origin, mouth.Z));
        if (!double.IsFinite(farthest) || farthest <= 0)
            throw new ArgumentException("The solid does not extend into mouth +Z.", nameof(mouth));
        return farthest + HoleLead();
    }
}
