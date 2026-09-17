using GeoCore;

namespace GeoSolver.Kinematics
{
    public class KinematicSolver
    {
        protected List<IBaseEquation> constraints = new List<IBaseEquation>();
        public List<IUpdate> degreesOfFreedom = new List<IUpdate>();
        private double characteristicLength = 1.0;
        public double CharacteristicLength => characteristicLength;

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

            // Bake recenters the Gibbs/Cayley chart between Newton solves.
            // At a collinear stationary point, a deterministic quarter-turn
            // restart supplies a nonzero angular Jacobian. Fixed bodies remain
            // fixed, and the complete constraint system is solved again.
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

                if (!RestartCollinearDirections())
                    break;
                bestSse = double.PositiveInfinity;
            }

            UpdateAllObjects();
            return result;
        }

        private bool RestartCollinearDirections()
        {
            bool nudged = false;
            for (int i = 0; i < constraints.Count; i++)
            {
                CVec3D direction1, direction2;
                double desiredDot;
                if (constraints[i] is DirectedParallelDirections3d directed)
                {
                    direction1 = directed.Direction1;
                    direction2 = directed.Direction2;
                    desiredDot = directed.Opposite ? -1 : 1;
                }
                else if (constraints[i] is AngleBetweenVectors3d angular)
                {
                    direction1 = angular.Vector1;
                    direction2 = angular.Vector2;
                    desiredDot = Math.Cos(angular.Angle.Evaluate());
                }
                else continue;

                Vec3D n1 = direction1.Evaluate();
                Vec3D n2 = direction2.Evaluate();
                if (n1.LengthSquared() < 1e-18 || n2.LengthSquared() < 1e-18)
                    continue;
                n1.Normalize();
                n2.Normalize();
                double alignment = Vec3DOps.Dot(n1,n2);
                if (constraints[i] is DirectedParallelDirections3d)
                {
                    if (alignment * desiredDot >= -0.25) continue;
                }
                else if (Math.Abs(alignment-desiredDot) <= NewtonSolver.LENGTH_EPS ||
                         Vec3DOps.Cross(n1,n2).LengthSquared() > 1e-12)
                    continue;

                CTransform body = FindFreeBodyOwning(direction2) ?? FindFreeBodyOwning(direction1);
                if (body == null)
                    continue;

                Vec3D axis = Vec3DOps.Cross(n1, n2);
                if (constraints[i] is AngleBetweenVectors3d && TryGetBearingAxis(body,n2,out Vec3D bearingAxis))
                    axis = bearingAxis;
                else if (axis.LengthSquared() < 1e-12)
                    axis = Vec3DOps.GetOrthoNormal(n2);
                axis.Normalize();
                Quaternion bump = AxisAngleQuaternion(axis, 0.5 * Math.PI);
                Transform pose = body.Evaluate();
                body.SetPose(new Transform(pose.Position, CTransform.Compose(bump, pose.Orientation)));
                nudged = true;
            }
            return nudged;
        }

        // A coaxial mate identifies the admissible angular restart direction.
        // Reuse that geometry instead of tilting a revolute body off its bearing.
        private bool TryGetBearingAxis(CTransform body, Vec3D direction, out Vec3D axis)
        {
            foreach (var constraint in constraints)
            {
                if (constraint is not CoincidentAxes3d bearing) continue;
                if (ReferenceEquals(FindFreeBodyOwning(bearing.Direction1),body))
                    axis = bearing.Direction1.Evaluate();
                else if (ReferenceEquals(FindFreeBodyOwning(bearing.Perpendicular2A),body))
                    axis = Vec3DOps.Cross(bearing.Perpendicular2A.Evaluate(),bearing.Perpendicular2B.Evaluate());
                else continue;
                if (Vec3DOps.Cross(axis,direction).LengthSquared() > 1e-12)
                    return true;
            }
            axis = default;
            return false;
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
