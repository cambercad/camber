using GeoCore;

namespace GeoSolver.Kinematics
{
    public class KinematicSolver
    {
        protected List<IBaseEquation> constraints = new List<IBaseEquation>();
        public List<IUpdate> degreesOfFreedom = new List<IUpdate>();
        private double characteristicLength = 1.0;

        public void AddConstraint(IBaseEquation constraint)
        {
            constraints.Add(constraint);
        }

        public void IncludeCharacteristicLength(double length)
        {
            if (double.IsFinite(length) && length > characteristicLength)
                characteristicLength = length;
        }

        public RigidTransform<T> AddRigidBody<T>(T body, Transform initialPose) where T : IUpdateTransform
        {
            var cTransform = CTransform.FromPose(initialPose.Position, initialPose.Orientation);
            var rigid = new RigidTransform<T>(body, cTransform);
            degreesOfFreedom.Add(rigid);
            return rigid;
        }

        public SolveResult SolveConstraints(bool preferMinimalMovement = false)
        {
            HashSet<Param> draggedParams = null;
            if (preferMinimalMovement)
            {
                draggedParams = new HashSet<Param>();
                for (int i = 0; i < constraints.Count; i++)
                {
                    foreach (Param parameter in constraints[i])
                        draggedParams.Add(parameter);
                }
            }

            // Gibbs/Cayley cannot take a 180° step from identity: n1−s n2 is then
            // parallel to the vectors and the rotation Jacobian is zero. Between
            // charts, Bake recenters ω=0; a 90° Rest bump about an axis ⊥ n2
            // turns that ridge into a regular 90° Gibbs problem (not a pose seed).
            SolveResult result = new SolveResult(false, 0, 0, 0, "No kinematic solve attempted.");
            const int chartCount = 8;
            const int newtonPerChart = 400;
            double bestSse = double.PositiveInfinity;
            for (int chart = 0; chart < chartCount; chart++)
            {
                result = ConstraintSolver.SolveDetailed(
                    constraints,
                    draggedParams,
                    scaling: characteristicLength,
                    allowSoftOverconstrained: true,
                    applySketchReductions: true,
                    maxNewtonIterations: newtonPerChart);

                for (int i = 0; i < degreesOfFreedom.Count; i++)
                {
                    if (degreesOfFreedom[i] is IHasCTransform posed)
                        posed.Transform.Bake();
                }

                if (result.Converged)
                    break;

                if (result.SumOfSquaredErrors < bestSse * 0.9)
                {
                    bestSse = result.SumOfSquaredErrors;
                    continue;
                }

                if (!NudgeFreeBodiesOffDirectedParallelRidge())
                    break;
                bestSse = double.PositiveInfinity;
            }

            UpdateAllObjects();
            return result;
        }

        private bool NudgeFreeBodiesOffDirectedParallelRidge()
        {
            bool nudged = false;
            for (int i = 0; i < constraints.Count; i++)
            {
                DirectedParallelDirections3d directed = constraints[i] as DirectedParallelDirections3d;
                if (directed == null)
                    continue;

                Vec3D n1 = directed.Direction1.Evaluate();
                Vec3D n2 = directed.Direction2.Evaluate();
                if (n1.LengthSquared() < 1e-18 || n2.LengthSquared() < 1e-18)
                    continue;
                n1.Normalize();
                n2.Normalize();
                double alignment = n1.X * n2.X + n1.Y * n2.Y + n1.Z * n2.Z;
                bool wrongHemisphere = directed.Opposite ? alignment > 0.25 : alignment < -0.25;
                if (!wrongHemisphere)
                    continue;

                CTransform body = FindFreeBodyOwning(directed.Direction2);
                if (body == null)
                    body = FindFreeBodyOwning(directed.Direction1);
                if (body == null)
                    continue;

                Vec3D axis = Vec3DOps.Cross(n1, n2);
                if (axis.LengthSquared() < 1e-12)
                    axis = Vec3DOps.GetOrthoNormal(n2);
                axis.Normalize();
                Quaternion bump = AxisAngleQuaternion(axis, 0.5 * Math.PI);
                Transform pose = body.Evaluate();
                body.SetPose(new Transform(pose.Position, CTransform.Compose(bump, pose.Orientation)));
                nudged = true;
            }
            return nudged;
        }

        private CTransform FindFreeBodyOwning(CVec3D direction)
        {
            HashSet<Param> dirParams = new HashSet<Param>();
            foreach (Param p in direction)
                dirParams.Add(p);

            for (int i = 0; i < degreesOfFreedom.Count; i++)
            {
                IHasCTransform posed = degreesOfFreedom[i] as IHasCTransform;
                if (posed == null)
                    continue;
                bool owns = false;
                bool hasFreeRotation = false;
                foreach (Param p in posed.Transform.RotationVector)
                {
                    if (dirParams.Contains(p))
                        owns = true;
                    if (!p.Frozen)
                        hasFreeRotation = true;
                }
                if (!owns)
                {
                    foreach (Param p in posed.Transform.Rest)
                    {
                        if (dirParams.Contains(p))
                            owns = true;
                    }
                }
                if (owns && hasFreeRotation)
                    return posed.Transform;
            }
            return null;
        }

        private static Quaternion AxisAngleQuaternion(Vec3D axis, double radians)
        {
            double half = 0.5 * radians;
            double s = Math.Sin(half);
            return CTransform.NormalizeQuaternion(new Quaternion(
                axis.X * s, axis.Y * s, axis.Z * s, Math.Cos(half)));
        }

        private void UpdateAllObjects()
        {
            for (int i = 0; i < degreesOfFreedom.Count; i++)
                degreesOfFreedom[i].Update();
        }
    }

    public class RigidTransform<T> : IUpdate, IHasCTransform where T : IUpdateTransform
    {
        public T Value;
        public CTransform Transform { get; }

        public RigidTransform(T value, CTransform transform)
        {
            Value = value;
            Transform = transform;
        }

        public void Update()
        {
            Value.Update(Transform.Evaluate());
        }
    }

    public interface IHasCTransform
    {
        CTransform Transform { get; }
    }

    public interface IUpdateTransform
    {
        public void Update(Transform transform);
    }
}
