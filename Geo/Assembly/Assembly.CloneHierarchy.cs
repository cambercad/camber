using GeoCore;
using GeoMeta;
using GeoSolver.Kinematics;

namespace Geo
{
    public partial class Assembly
    {
        /// <summary>Copies the hierarchy and internal mates, without solving or flattening bodies.</summary>
        internal Assembly CloneHierarchy(string name, bool mirrored = false)
        {
            name ??= GeoAPI.GenerateName("AssemblyCopy");
            if (_api.GetAssemblies().Any(existing => existing.Name == name))
                throw new ArgumentException($"Assembly '{name}' already exists.", nameof(name));
            var parts = new Dictionary<AssemblyPart, AssemblyPart>();
            var occurrences = new Dictionary<AssemblyOccurrence, AssemblyOccurrence>();
            var nodes = new List<(Assembly Source, Assembly Copy)>();
            var meshes = new Dictionary<AnchorMesh, AnchorMesh>();
            Vec3D MapVector(Vec3D value) => mirrored ? ReflectionTransforms.LocalPoint(value) : value;
            Transform MapPose(Transform value) => mirrored ? ReflectionTransforms.LocalPose(value) : value;

            Assembly CopyTree(Assembly source, string prefix)
            {
                Assembly copy = _api.GetAssembly(prefix);
                copy.SolveAfterEveryConstraint = false;
                nodes.Add((source, copy));
                foreach (AssemblyPart part in source._parts)
                {
                    AnchorMesh mesh = part.Mesh;
                    if (mirrored && !meshes.TryGetValue(part.Mesh, out mesh))
                        meshes.Add(part.Mesh, mesh = MirrorRestBody(part));
                    Transform pose = MapPose(part.EvaluatePose());
                    parts.Add(part, copy.AddPart(mesh, pose.Position, pose.Orientation));
                }
                foreach (AssemblyOccurrence occurrence in source._occurrences)
                {
                    string childName;
                    do { childName = GeoAPI.GenerateName(prefix + "Child"); }
                    while (_api.GetAssemblies().Any(existing => existing.Name == childName));
                    Assembly child = CopyTree(occurrence.Child, childName);
                    Transform pose = MapPose(occurrence.EvaluatePose());
                    occurrences.Add(occurrence, copy.AddSubAssembly(child, pose.Position, pose.Orientation));
                }
                return copy;
            }

            Assembly result = CopyTree(this, name);
            // Replay after all descendants exist: parent mates can refer to nested leaves.
            foreach (var (source, copy) in nodes)
            {
                foreach (AssemblyMateRecord mate in source._mateRecords)
                {
                    AssemblyPart a = mate.PartA == null ? null : parts[mate.PartA];
                    AssemblyPart b = mate.PartB == null ? null : parts[mate.PartB];
                    string Entity(int index, AssemblyPart oldPart, AssemblyPart newPart)
                    {
                        if (index >= mate.OperandEntities.Length) return "";
                        string entity = mate.OperandEntities[index] ?? "";
                        string prefix = oldPart.Mesh.Name + ":";
                        if (entity.StartsWith(prefix, StringComparison.Ordinal))
                            return newPart.Mesh.Name + ":" + EntityNaming.RewriteMeshNameInEntity(
                                entity.Substring(prefix.Length), oldPart.Mesh.Name, newPart.Mesh.Name);
                        return EntityNaming.RewriteMeshNameInEntity(entity, oldPart.Mesh.Name, newPart.Mesh.Name);
                    }
                    AssemblyPointDatum PointA() => new(a, MapVector(mate.LocalA), Entity(0, mate.PartA, a));
                    AssemblyPointDatum PointB() => new(b, MapVector(mate.LocalB), Entity(1, mate.PartB, b));
                    AssemblyAxisDatum AxisA() => new(a, MapVector(mate.LocalA), MapVector(mate.DirA), Entity(0, mate.PartA, a));
                    AssemblyAxisDatum AxisB() => new(b, MapVector(mate.LocalB), MapVector(mate.DirB), Entity(1, mate.PartB, b));
                    AssemblyPlaneDatum PlaneA() => new(a, MapVector(mate.LocalA), MapVector(mate.DirA), Entity(0, mate.PartA, a));
                    AssemblyPlaneDatum PlaneB() => new(b, MapVector(mate.LocalB), MapVector(mate.DirB), Entity(1, mate.PartB, b));
                    switch (mate.Kind)
                    {
                        case AssemblyMateKind.FixPart:
                            if (!mate.FixedPose.HasValue)
                                throw new InvalidOperationException("Fixed mate has no recorded body target.");
                            Transform target = MapPose(mate.FixedPose.Value);
                            AssemblyOccurrence occurrence = mate.FixedOccurrence == null ? null : occurrences[mate.FixedOccurrence];
                            copy.RecordMate(new AssemblyMateRecord(AssemblyMateKind.FixPart, mate.Label, a)
                                .WithFixedTarget(target, occurrence),
                                occurrence == null ? a.Mesh.Name + ":" : occurrence.Child.Name + ":");
                            copy.AddSolverConstraints(new FixedTransformConstraint3d(
                                occurrence == null ? a.Transform : occurrence.Transform, target));
                            break;
                        case AssemblyMateKind.CoincidentPoints: copy.SetCoincident(PointA(), PointB()); break;
                        case AssemblyMateKind.CoincidentAxes: copy.SetCoincident(AxisA(), AxisB()); break;
                        case AssemblyMateKind.CoincidentPlanes:
                            if (double.IsNaN(mate.Scalar)) copy.SetCoincident(PlaneA(), PlaneB());
                            else copy.SetCoincidentOriented(PlaneA(), PlaneB(), mate.Scalar < 0);
                            break;
                        case AssemblyMateKind.ParallelAxes: copy.SetParallel(AxisA(), AxisB()); break;
                        case AssemblyMateKind.ParallelPlanes: copy.SetParallel(PlaneA(), PlaneB()); break;
                        case AssemblyMateKind.PerpendicularAxes: copy.SetPerpendicular(AxisA(), AxisB()); break;
                        case AssemblyMateKind.PerpendicularPlanes: copy.SetPerpendicular(PlaneA(), PlaneB()); break;
                        case AssemblyMateKind.Concentric: copy.SetConcentric(AxisA(), AxisB()); break;
                        case AssemblyMateKind.DistancePoints: copy.SetDistance(PointA(), PointB(), mate.Scalar); break;
                        case AssemblyMateKind.DistancePlanes: copy.SetDistance(PlaneA(), PlaneB(), mate.Scalar); break;
                        case AssemblyMateKind.AngleAxes: copy.SetAngle(AxisA(), AxisB(), mate.Scalar); break;
                        case AssemblyMateKind.PointOnPlane: copy.SetPointOnPlane(PointA(), PlaneB()); break;
                        case AssemblyMateKind.Contact: copy.SetContact(PointA(), PlaneB()); break;
                        default: throw new NotSupportedException($"Cannot replay assembly mate {mate.Kind}.");
                    }
                }
                copy.SolveAfterEveryConstraint = source.SolveAfterEveryConstraint;
            }
            return result;
        }
    }
}
