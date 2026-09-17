using GeoCore;
namespace Geo;
public partial class Assembly
{
    /// <summary>Non-destructive capped snapshot in the current pose. Retains plane-local Z ≤ 0.</summary>
    public SectionView Section(CoordinateSystem plane)
    {
        var parts=new List<AssemblyPart>(); var poses=new List<Transform>(); var paths=new List<string>();
        CollectLeafWorldPoses(parts,poses,paths);
        return new SectionView(parts.Select((p,i)=>(p.Mesh,poses[i],paths[i])),Converter,plane);
    }
}
