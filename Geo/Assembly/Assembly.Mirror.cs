using GeoCore;

namespace Geo;

public partial class Assembly
{
    internal AnchorMesh MirrorRestBody(AssemblyPart source)
        => _api.MirrorRigidRestBody(source.Mesh);

    /// <summary>
    /// Create an opposite-handed body at its current reflected placement.
    /// Recorded rigid mates then make it follow the seed, as an assembly pattern does.
    /// </summary>
    public AssemblyPart Mirror(AssemblyPart seed, CoordinateSystem plane, string name = null)
    {
        if (seed == null) throw new ArgumentNullException(nameof(seed));
        if (!ReferenceEquals(seed.Assembly, this))
            throw new ArgumentException("Mirror seed must be a direct part of this assembly.", nameof(seed));
        var target = ReflectionTransforms.Placement(seed.EvaluatePose(), plane);
        var mesh = _api.MirrorRigidRestBody(seed.Mesh, name);
        bool solveAfter = SolveAfterEveryConstraint;
        SolveAfterEveryConstraint = false;
        AssemblyPart result;
        try
        {
            result = AddPart(mesh, target.Position, target.Orientation);
            BindPatternPlacement(seed, result);
        }
        finally { SolveAfterEveryConstraint = solveAfter; }
        if (solveAfter) SolveConstraints();
        return result;
    }

    /// <summary>Mirror a complete hierarchy, retaining its internal mate definitions.</summary>
    public AssemblyOccurrence Mirror(AssemblyOccurrence seed, CoordinateSystem plane, string name = null)
    {
        if (seed == null) throw new ArgumentNullException(nameof(seed));
        if (!ReferenceEquals(seed.Parent, this))
            throw new ArgumentException("Mirror seed must be a direct subassembly of this assembly.", nameof(seed));
        var representative = seed.Child.RepresentativePart();
        if (representative == null)
            throw new ArgumentException("Cannot mirror an empty subassembly.", nameof(seed));
        var target = ReflectionTransforms.Placement(seed.EvaluatePose(), plane);
        name = string.IsNullOrEmpty(name) ? GeoAPI.GenerateName(seed.Name + "_mirror") : name;
        var child = seed.Child.CloneHierarchy(name, mirrored: true);
        bool solveAfter = SolveAfterEveryConstraint;
        SolveAfterEveryConstraint = false;
        AssemblyOccurrence result;
        try
        {
            result = AddSubAssembly(child, target.Position, target.Orientation);
            BindPatternPlacement(representative, child.RepresentativePart());
        }
        finally { SolveAfterEveryConstraint = solveAfter; }
        if (solveAfter) SolveConstraints();
        return result;
    }
}
