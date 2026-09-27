using GeoCore;
using GeoMeta;
using GeoSolver.Kinematics;

namespace Geo
{
    [APIDescription(@"AssemblyOccurrence: one rigid placement of a child Assembly definition. GetParts() returns references scoped to this occurrence so mates can distinguish repeated placements.")]
    public sealed class AssemblyOccurrence
    {
        internal AssemblyOccurrence(Assembly parent, Assembly child, RigidTransform<AssemblyOccurrenceRig> rigidBody,
            AssemblyOccurrence definitionOccurrence = null, AssemblyOccurrence rootContext = null,
            IReadOnlyList<AssemblyOccurrence> definitionPath = null, bool flexible = false)
        {
            Parent = parent;
            Child = child;
            RigidBody = rigidBody;
            DefinitionOccurrence = definitionOccurrence;
            RootContext = rootContext;
            DefinitionPath = definitionPath ?? Array.Empty<AssemblyOccurrence>();
            _flexible = flexible;
            DisplayName = child.Name;
        }

        private readonly bool _flexible;

        public Assembly Parent { get; }
        public Assembly Child { get; }
        [APIDescription(@"IsFlexible (bool): whether this occurrence has independent internal mates and part geometry.")]
        public bool IsFlexible => DefinitionOccurrence?.IsFlexible ?? _flexible;
        internal RigidTransform<AssemblyOccurrenceRig> RigidBody { get; }
        internal AssemblyOccurrence DefinitionOccurrence { get; }
        internal AssemblyOccurrence RootContext { get; }
        internal IReadOnlyList<AssemblyOccurrence> DefinitionPath { get; }
        internal string DisplayName { get; set; }
        internal AssemblyOccurrence InstanceRoot => RootContext ?? this;
        internal CTransform Transform => RigidBody.Transform;

        [APIDescription(@"Name (str): name of the nested child assembly.")]
        public string Name => DisplayName;

        [APIDescription(@"EvaluatePose() -> Transform
Current solved pose of this occurrence in the parent assembly.")]
        public Transform EvaluatePose()
        {
            if (DefinitionOccurrence == null)
                return Transform.Evaluate();
            Transform withinDefinition = Assembly.PoseOfOccurrence(InstanceRoot.Child, DefinitionPath);
            return TransformMath.Compose(InstanceRoot.EvaluatePose(), withinDefinition);
        }

        [APIDescription(@"GetParts() -> IReadOnlyList[AssemblyPart]
Direct parts of this occurrence. References are occurrence-scoped so repeated instances can be mated independently.")]
        public IReadOnlyList<AssemblyPart> GetParts()
        {
            var parts = Child.GetParts();
            var result = new AssemblyPart[parts.Count];
            for (int i = 0; i < parts.Count; i++)
                result[i] = parts[i].ForOccurrence(InstanceRoot, DefinitionPath);
            return result;
        }

        [APIDescription(@"GetSubAssemblies() -> IReadOnlyList[AssemblyOccurrence]
Direct sub-assemblies of the nested child.")]
        public IReadOnlyList<AssemblyOccurrence> GetSubAssemblies()
        {
            var occurrences = Child.GetSubAssemblies();
            var result = new AssemblyOccurrence[occurrences.Count];
            for (int i = 0; i < occurrences.Count; i++)
            {
                AssemblyOccurrence definition = occurrences[i];
                var path = new AssemblyOccurrence[DefinitionPath.Count + 1];
                for (int j = 0; j < DefinitionPath.Count; j++)
                    path[j] = DefinitionPath[j];
                path[^1] = definition;
                result[i] = new AssemblyOccurrence(InstanceRoot.Parent, definition.Child,
                    definition.RigidBody, definition, InstanceRoot, path, definition.IsFlexible)
                { DisplayName = definition.Name };
            }
            return result;
        }
    }

    internal sealed class AssemblyOccurrenceRig : IUpdateTransform
    {
        private readonly Assembly _child;

        public AssemblyOccurrenceRig(Assembly child)
        {
            _child = child;
        }

        public void Update(Transform transform)
        {
            _child.ApplyComposedPose(transform);
        }
    }
}
