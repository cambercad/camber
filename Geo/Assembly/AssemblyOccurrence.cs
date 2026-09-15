using GeoCore;
using GeoMeta;
using GeoSolver.Kinematics;

namespace Geo
{
    [APIDescription(@"AssemblyOccurrence: rigid instance of a child Assembly inside a parent (returned by AddSubAssembly). Child mates stay in the child; the parent has 6 DOF for the whole subtree.")]
    public sealed class AssemblyOccurrence
    {
        internal AssemblyOccurrence(Assembly parent, Assembly child, RigidTransform<AssemblyOccurrenceRig> rigidBody)
        {
            Parent = parent;
            Child = child;
            RigidBody = rigidBody;
        }

        public Assembly Parent { get; }
        public Assembly Child { get; }
        internal RigidTransform<AssemblyOccurrenceRig> RigidBody { get; }
        internal CTransform Transform => RigidBody.Transform;

        [APIDescription(@"Name (str): name of the nested child assembly.")]
        public string Name => Child.Name;

        [APIDescription(@"EvaluatePose() -> Transform
Current solved pose of this occurrence in the parent assembly.")]
        public Transform EvaluatePose() => Transform.Evaluate();

        [APIDescription(@"GetParts() -> IReadOnlyList[AssemblyPart]
Direct parts of the nested child assembly.")]
        public IReadOnlyList<AssemblyPart> GetParts() => Child.GetParts();

        [APIDescription(@"GetSubAssemblies() -> IReadOnlyList[AssemblyOccurrence]
Direct sub-assemblies of the nested child.")]
        public IReadOnlyList<AssemblyOccurrence> GetSubAssemblies() => Child.GetSubAssemblies();
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
