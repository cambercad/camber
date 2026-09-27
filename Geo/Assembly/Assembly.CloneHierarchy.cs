using GeoCore;
using GeoMeta;
using GeoSolver.Kinematics;

namespace Geo
{
    public partial class Assembly
    {
        private sealed class AssemblyCloneNode
        {
            public Assembly Source;
            public Assembly Copy;
            public readonly Dictionary<AssemblyPart, AssemblyPart> Parts = new();
            public readonly Dictionary<AssemblyOccurrence, AssemblyOccurrence> Occurrences = new();
            public readonly Dictionary<AssemblyOccurrence, AssemblyCloneNode> Children = new();
        }

        /// <summary>Copies the hierarchy and internal mates, without solving or flattening bodies.</summary>
        internal Assembly CloneHierarchy(string name, bool mirrored = false, bool independentGeometry = false)
        {
            name ??= GeoAPI.GenerateName("AssemblyCopy");
            if (_api.GetAssemblies().Any(existing => existing.Name == name))
                throw new ArgumentException($"Assembly '{name}' already exists.", nameof(name));
            var nodes = new List<AssemblyCloneNode>();
            var meshes = new Dictionary<AssemblyPart, AnchorMesh>();
            Vec3D MapVector(Vec3D value) => mirrored ? ReflectionTransforms.LocalPoint(value) : value;
            Transform MapPose(Transform value) => mirrored ? ReflectionTransforms.LocalPose(value) : value;

            AssemblyCloneNode CopyTree(Assembly source, string prefix)
            {
                Assembly copy = _api.GetAssembly(prefix);
                copy.SolveAfterEveryConstraint = false;
                var node = new AssemblyCloneNode { Source = source, Copy = copy };
                nodes.Add(node);
                foreach (AssemblyPart part in source._parts)
                {
                    AnchorMesh mesh = part.Mesh;
                    if (mirrored)
                    {
                        if (!meshes.TryGetValue(part, out mesh))
                            meshes.Add(part, mesh = MirrorRestBody(part));
                    }
                    else if (independentGeometry)
                    {
                        AnchorMesh rest = part.Mesh.SnapshotRigidDefinition(part.Mesh.Name);
                        string meshName = GeoAPI.GenerateName(part.Mesh.Name + "_instance");
                        mesh = _api.CopyMeshAsInstance(rest, meshName);
                    }
                    Transform pose = MapPose(part.EvaluatePose());
                    node.Parts.Add(part, copy.AddPart(mesh, pose.Position, pose.Orientation));
                }
                foreach (AssemblyOccurrence occurrence in source._occurrences)
                {
                    string childName;
                    do { childName = GeoAPI.GenerateName(prefix + "Child"); }
                    while (_api.GetAssemblies().Any(existing => existing.Name == childName));
                    AssemblyCloneNode childNode = CopyTree(occurrence.Child, childName);
                    Transform pose = MapPose(occurrence.EvaluatePose());
                    AssemblyOccurrence copiedOccurrence = copy.AddSubAssembly(childNode.Copy, pose.Position, pose.Orientation,
                        flexible: occurrence.IsFlexible, cloneFlexibleDefinition: false);
                    copiedOccurrence.DisplayName = occurrence.Name;
                    node.Occurrences.Add(occurrence, copiedOccurrence);
                    node.Children.Add(occurrence, childNode);
                }
                return node;
            }

            AssemblyCloneNode root = CopyTree(this, name);
            Assembly result = root.Copy;

            AssemblyPart MapPart(AssemblyCloneNode node, AssemblyPart sourcePart)
            {
                if (sourcePart.OccurrenceContext == null && sourcePart.Assembly == node.Source)
                    return node.Parts[sourcePart.DefinitionPart];

                IReadOnlyList<AssemblyOccurrence> path = sourcePart.DefinitionOccurrencePath;
                AssemblyOccurrence first;
                int offset;
                if (sourcePart.OccurrenceContext != null)
                {
                    first = sourcePart.OccurrenceContext;
                    offset = 0;
                }
                else
                {
                    var matches = new List<IReadOnlyList<AssemblyOccurrence>>();
                    void FindPaths(Assembly currentAssembly, List<AssemblyOccurrence> currentPath)
                    {
                        if (ReferenceEquals(currentAssembly, sourcePart.Assembly))
                        {
                            matches.Add(currentPath.ToArray());
                            return;
                        }
                        foreach (AssemblyOccurrence occurrence in currentAssembly._occurrences)
                        {
                            currentPath.Add(occurrence);
                            FindPaths(occurrence.Child, currentPath);
                            currentPath.RemoveAt(currentPath.Count - 1);
                        }
                    }
                    FindPaths(node.Source, new List<AssemblyOccurrence>());
                    if (matches.Count != 1 || matches[0].Count == 0)
                        throw new InvalidOperationException(
                            "An unscoped nested mate reference is missing or ambiguous in the cloned hierarchy.");
                    path = matches[0];
                    first = path[0];
                    offset = 1;
                }

                if (!node.Children.TryGetValue(first, out AssemblyCloneNode current))
                    throw new InvalidOperationException("Mate references an occurrence outside the cloned assembly node.");
                for (int i = offset; i < path.Count; i++)
                {
                    AssemblyOccurrence step = path[i];
                    if (!current.Children.TryGetValue(step, out current))
                        throw new InvalidOperationException("Mate references an occurrence path outside the cloned hierarchy.");
                }
                return current.Parts[sourcePart.DefinitionPart];
            }

            // Replay after all descendants exist: parent mates can refer to nested leaves.
            foreach (AssemblyCloneNode node in nodes)
            {
                Assembly source = node.Source;
                Assembly copy = node.Copy;
                foreach (AssemblyMateRecord mate in source._mateRecords)
                {
                    AssemblyPart a = mate.PartA == null ? null : MapPart(node, mate.PartA);
                    AssemblyPart b = mate.PartB == null ? null : MapPart(node, mate.PartB);
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
                            AssemblyOccurrence occurrence = mate.FixedOccurrence == null ? null : node.Occurrences[mate.FixedOccurrence];
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
