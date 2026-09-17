using CSG;
using GeoCore;

namespace Geo;

/// <summary>A positive-volume intersection between two placed leaf occurrences.</summary>
public sealed record AssemblyInterference(string First, string Second, double Volume, AnchorMesh Geometry);

public partial class Assembly
{
    /// <summary>
    /// Intersections in the assembly's current pose, sorted by descending volume.
    /// The threshold is in cubic model units. Does not solve, move or register parts.
    /// Surface-only parts are rejected; touching boundaries are not interference.
    /// </summary>
    public IReadOnlyList<AssemblyInterference> Interferences(double minVolume = 0)
    {
        if (!double.IsFinite(minVolume) || minVolume < 0)
            throw new ArgumentOutOfRangeException(nameof(minVolume), "Minimum volume must be finite and nonnegative.");
        var parts = new List<AssemblyPart>();
        var poses = new List<Transform>();
        var paths = new List<string>();
        CollectLeafWorldPoses(parts, poses, paths);
        var meshes = new List<MeshNormalUV>(parts.Count);
        var bounds = new List<(Rat3Hybrid Min, Rat3Hybrid Max)>(parts.Count);
        for (int i = 0; i < parts.Count; i++)
        {
            if (!parts[i].Mesh.IsVolume)
                throw new InvalidOperationException($"Interference requires solids: '{paths[i]}' is a surface.");
            var mesh = parts[i].Mesh.SnapshotRigidPose(poses[i], i);
            meshes.Add(mesh);
            // Boolean meshes may retain unused tool vertices. Bounds describe
            // the actual surface, otherwise unrelated parts reach exact CSG.
            var referenced = new HashSet<int>();
            foreach (var triangle in mesh.Triangles)
            {
                referenced.Add(triangle.A);
                referenced.Add(triangle.B);
                referenced.Add(triangle.C);
            }
            var min = mesh.Triangles.Count == 0 ? new Rat3Hybrid(0,0,0)
                : mesh.PrecisionPositions[mesh.Triangles[0].A];
            var max = min;
            foreach (int index in referenced)
            {
                var p = mesh.PrecisionPositions[index];
                if (p.X.CompareTo(min.X)<0) min.X=p.X;
                if (p.Y.CompareTo(min.Y)<0) min.Y=p.Y;
                if (p.Z.CompareTo(min.Z)<0) min.Z=p.Z;
                if (p.X.CompareTo(max.X)>0) max.X=p.X;
                if (p.Y.CompareTo(max.Y)>0) max.Y=p.Y;
                if (p.Z.CompareTo(max.Z)>0) max.Z=p.Z;
            }
            bounds.Add((min,max));
        }
        // A common rigid pose cannot change an intersection or its volume.
        // Keep such pairs in their original exact coordinates: arbitrary world
        // rotations otherwise inflate rational denominators throughout CSG.
        var localMeshes = new MeshNormalUV[parts.Count];
        var identity = new Transform(new Vec3D(0), TransformMath.IdentityOrientation);
        MeshNormalUV LocalMesh(int index) => localMeshes[index] ??=
            parts[index].Mesh.SnapshotRigidPose(identity, index);
        var result = new List<AssemblyInterference>();
        double unit = _api.Converter.SmallestUnit();
        for (int i = 0; i < meshes.Count; i++)
        for (int j = i+1; j < meshes.Count; j++)
        {
            var a=bounds[i]; var b=bounds[j];
            if (a.Max.X.CompareTo(b.Min.X)<=0 || b.Max.X.CompareTo(a.Min.X)<=0 ||
                a.Max.Y.CompareTo(b.Min.Y)<=0 || b.Max.Y.CompareTo(a.Min.Y)<=0 ||
                a.Max.Z.CompareTo(b.Min.Z)<=0 || b.Max.Z.CompareTo(a.Min.Z)<=0)
                continue;
            bool commonPose = poses[i].Equals(poses[j]);
            MeshNormalUV overlap;
            try { overlap = MeshNormalUV.BooleanOperation(
                commonPose ? LocalMesh(i) : meshes[i],
                commonPose ? LocalMesh(j) : meshes[j], BooleanOp.Intersect, _api.Converter); }
            catch (Exception error)
            {
                throw new InvalidOperationException($"Interference check failed for '{paths[i]}' and '{paths[j]}'.", error);
            }
            // Accumulate exactly, then convert once for reporting in model units.
            var exactVolume = MeshAnalysis.ComputeSignedMeshVolume(overlap.PrecisionPositions, overlap.Triangles);
            if (exactVolume.Sign() == 0) continue;
            double volume = Math.Abs(exactVolume.ToDouble()) * unit * unit * unit;
            if (volume <= minVolume) continue;
            if (commonPose)
                overlap = overlap.SnapshotRigidPose(overlap.PrecisionPositions, _api.Converter, poses[i]);
            var names = new Dictionary<int,string> { [i]="first", [j]="second" };
            var geometry = new AnchorMesh($"Interference_{i+1}_{j+1}", overlap, names,
                new Dictionary<string,SurfaceMetaData>(), deferCoplanarPostProcess:true);
            result.Add(new(paths[i],paths[j],volume,geometry));
        }
        return result.OrderByDescending(x=>x.Volume).ThenBy(x=>x.First,StringComparer.Ordinal)
            .ThenBy(x=>x.Second,StringComparer.Ordinal).ToList();
    }
}
