using GeoCore;

namespace Geo;

public partial class Assembly
{
    /// <summary>Create a linear pattern including the seed, rigidly related to it.</summary>
    public IReadOnlyList<AssemblyPart> PatternLinear(AssemblyPart seed,int count,Vec3D step)
        => PatternParts(seed,PatternTransforms.Linear(count,step));

    /// <summary>Create a circular pattern including the seed around an assembly-frame axis.</summary>
    public IReadOnlyList<AssemblyPart> PatternCircular(AssemblyPart seed,int count,
        CoordinateSystem axis,double angle=2*Math.PI)
        => PatternParts(seed,PatternTransforms.Circular(count,axis,angle));

    private IReadOnlyList<AssemblyPart> PatternParts(AssemblyPart seed,IReadOnlyList<Transform> placements)
    {
        if(seed == null) throw new ArgumentNullException(nameof(seed));
        if(!ReferenceEquals(seed.Assembly,this))
            throw new ArgumentException("Pattern seed must be a direct part of this assembly.",nameof(seed));
        var result=new List<AssemblyPart>{seed};
        var original=seed.EvaluatePose();
        bool solveAfter=SolveAfterEveryConstraint;
        SolveAfterEveryConstraint=false;
        try
        {
            for(int i=1;i<placements.Count;i++)
            {
                var target=TransformMath.Compose(placements[i],original);
                var instance=AddPart(seed.Mesh,target.Position,target.Orientation);
                BindPatternPlacement(seed,instance);
                result.Add(instance);
            }
        }
        finally { SolveAfterEveryConstraint=solveAfter; }
        if(solveAfter && result.Count>1) SolveConstraints();
        return result;
    }
    /// <summary>Lock two representative bodies at their current relative pose.</summary>
    internal void BindPatternPlacement(AssemblyPart seed,AssemblyPart instance)
    {
        var relative=TransformMath.Compose(TransformMath.Inverse(WorldPoseOf(seed)),WorldPoseOf(instance));
        var origin=relative.Position;
        var z=TransformMath.TransformDirection(relative,new Vec3D(0,0,1));
        var x=TransformMath.TransformDirection(relative,new Vec3D(1,0,0));
        SetConcentric(seed.AddAxisDatumAt(origin,z),
            instance.AddAxisDatumAt(new Vec3D(0),new Vec3D(0,0,1)));
        SetCoincidentOriented(seed.AddPlaneDatumAt(origin,z),
            instance.AddPlaneDatumAt(new Vec3D(0),new Vec3D(0,0,1)),false);
        SetCoincidentOriented(seed.AddPlaneDatumAt(origin,x),
            instance.AddPlaneDatumAt(new Vec3D(0),new Vec3D(1,0,0)),false);
    }

    internal AssemblyPart RepresentativePart() => FirstLeafPart(this);

    public IReadOnlyList<AssemblyOccurrence> PatternLinear(AssemblyOccurrence seed,int count,Vec3D step)
        => PatternOccurrences(seed,PatternTransforms.Linear(count,step));

    public IReadOnlyList<AssemblyOccurrence> PatternCircular(AssemblyOccurrence seed,int count,
        CoordinateSystem axis,double angle=2*Math.PI)
        => PatternOccurrences(seed,PatternTransforms.Circular(count,axis,angle));

    private IReadOnlyList<AssemblyOccurrence> PatternOccurrences(AssemblyOccurrence seed,
        IReadOnlyList<Transform> placements)
    {
        if(seed == null) throw new ArgumentNullException(nameof(seed));
        if(!ReferenceEquals(seed.Parent,this))
            throw new ArgumentException("Pattern seed must be a direct subassembly of this assembly.",nameof(seed));
        var sourcePart=seed.Child.RepresentativePart();
        if(sourcePart == null)
            throw new ArgumentException("Cannot pattern an empty subassembly.",nameof(seed));
        var result=new List<AssemblyOccurrence>{seed};
        var original=seed.EvaluatePose();
        bool solveAfter=SolveAfterEveryConstraint;
        SolveAfterEveryConstraint=false;
        try
        {
            for(int i=1;i<placements.Count;i++)
            {
                var child=seed.Child.CloneHierarchy(GeoAPI.GenerateName(seed.Name+"_pattern"));
                var target=TransformMath.Compose(placements[i],original);
                var instance=AddSubAssembly(child,target.Position,target.Orientation);
                BindPatternPlacement(sourcePart,child.RepresentativePart());
                result.Add(instance);
            }
        }
        finally { SolveAfterEveryConstraint=solveAfter; }
        if(solveAfter && result.Count>1) SolveConstraints();
        return result;
    }

}
