using GeoCore;
using System.Collections;

namespace GeoSolver.Kinematics
{
    public class FixedTransformConstraint3d : IBaseEquation
    {
        public CTransform Transform;
        public Vec3D FixedPosition;

        public FixedTransformConstraint3d(CTransform transform)
        {
            Transform = transform;
            transform.Bake();
            FixedPosition = transform.Position.Evaluate();
            FreezeFreePoseParameters(transform);
        }

        public FixedTransformConstraint3d(CTransform transform, Transform fixedValue)
        {
            Transform = transform;
            transform.SetPose(fixedValue);
            FixedPosition = fixedValue.Position;
            FreezeFreePoseParameters(transform);
        }

        private static void FreezeFreePoseParameters(CTransform transform)
        {
            foreach (Param p in transform.Position)
                p.Frozen = true;
            foreach (Param p in transform.RotationVector)
                p.Frozen = true;
        }

        public void GenerateEquations(List<Expr> equations, double scaling)
        {
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param v in Transform) yield return v;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }

    public class CTransform : IEnumerableParams
    {
        public CVec3D Position;
        public CVec3D RotationVector;
        public CQuaternion Rest;

        public CVec3D Orientation { get { return RotationVector; } }

        public Quaternion RestOrientation
        {
            get { return Rest.Evaluate(); }
            set { Rest.SetValue(NormalizeQuaternion(value)); }
        }

        public CTransform(CVec3D position, CVec3D rotationVector, Quaternion restOrientation)
        {
            Position = position;
            RotationVector = rotationVector;
            Rest = CQuaternion.Frozen(NormalizeQuaternion(restOrientation));
            MarkRotationParameters();
        }

        public static CTransform FromPose(Vec3D position, Quaternion orientation)
        {
            CVec3D omega = new CVec3D(0, 0, 0);
            var transform = new CTransform(
                new CVec3D(position.X, position.Y, position.Z),
                omega,
                orientation);
            return transform;
        }

        public void SetPose(Transform pose)
        {
            Position.Ex.SetValue(pose.Position.X);
            Position.Ey.SetValue(pose.Position.Y);
            Position.Ez.SetValue(pose.Position.Z);
            RotationVector.Ex.SetValue(0);
            RotationVector.Ey.SetValue(0);
            RotationVector.Ez.SetValue(0);
            Rest.SetValue(NormalizeQuaternion(pose.Orientation));
        }

        public void Bake()
        {
            Transform pose = Evaluate();
            SetPose(pose);
        }

        public CVec3D PointLocalToGlobal(CVec3D localPoint)
        {
            CVec3D rotated = DirectionLocalToGlobal(localPoint);
            return new CVec3D(
                Position.Ex + rotated.Ex,
                Position.Ey + rotated.Ey,
                Position.Ez + rotated.Ez);
        }

        public CVec3D DirectionLocalToGlobal(CVec3D localDirection)
        {
            CVec3D inRest = RotateByQuaternion(Rest, localDirection);
            return RotateByGibbsVector(RotationVector, inRest);
        }

        public Transform Evaluate()
        {
            Quaternion delta = GibbsVectorToQuaternion(RotationVector.Evaluate());
            return new Transform(Position.Evaluate(), NormalizeQuaternion(Compose(delta, Rest.Evaluate())));
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in Position) yield return p;
            foreach (Param p in RotationVector) yield return p;
            foreach (Param p in Rest) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        private void MarkRotationParameters()
        {
            foreach (Param p in RotationVector)
                p.ScaleKind = ParamScaleKind.Orientation;
        }

        internal static CVec3D RotateByGibbsVector(CVec3D omega, CVec3D v)
        {
            Expr t2 = omega.Ex * omega.Ex + omega.Ey * omega.Ey + omega.Ez * omega.Ez;
            Expr invN = Expr.Constant(1.0) / Expr.Sqrt(t2 + Expr.Constant(1.0));
            Expr qx = omega.Ex * invN;
            Expr qy = omega.Ey * invN;
            Expr qz = omega.Ez * invN;
            Expr qw = invN;

            Expr lx = v.Ex;
            Expr ly = v.Ey;
            Expr lz = v.Ez;
            Expr tx = 2 * (qy * lz - qz * ly);
            Expr ty = 2 * (qz * lx - qx * lz);
            Expr tz = 2 * (qx * ly - qy * lx);

            return new CVec3D(
                lx + qw * tx + (qy * tz - qz * ty),
                ly + qw * ty + (qz * tx - qx * tz),
                lz + qw * tz + (qx * ty - qy * tx));
        }

        internal static Quaternion GibbsVectorToQuaternion(Vec3D omega)
        {
            double n = Math.Sqrt(omega.X * omega.X + omega.Y * omega.Y + omega.Z * omega.Z + 1.0);
            if (n < 1e-18)
                return TransformMath.IdentityOrientation;
            n = 1.0 / n;
            return new Quaternion(omega.X * n, omega.Y * n, omega.Z * n, n);
        }

        internal static CVec3D RotateByQuaternion(CQuaternion q, CVec3D vector)
        {
            Expr lx = vector.Ex;
            Expr ly = vector.Ey;
            Expr lz = vector.Ez;
            Expr ex = q.Ex;
            Expr ey = q.Ey;
            Expr ez = q.Ez;
            Expr ew = q.Ew;

            Expr tx = 2 * (ey * lz - ez * ly);
            Expr ty = 2 * (ez * lx - ex * lz);
            Expr tz = 2 * (ex * ly - ey * lx);

            return new CVec3D(
                lx + ew * tx + (ey * tz - ez * ty),
                ly + ew * ty + (ez * tx - ex * tz),
                lz + ew * tz + (ex * ty - ey * tx));
        }

        internal static Quaternion Compose(Quaternion a, Quaternion b)
        {
            return new Quaternion(
                a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
                a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
                a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
                a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);
        }

        internal static Quaternion NormalizeQuaternion(Quaternion q)
        {
            q = TransformMath.NormalizeDefault(q);
            double n = Math.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
            if (n < 1e-18)
                return TransformMath.IdentityOrientation;
            n = 1.0 / n;
            return new Quaternion(q.X * n, q.Y * n, q.Z * n, q.W * n);
        }
    }

    public class CQuaternion : IEnumerableParams
    {
        private Expr _x; public Expr Ex { get { return _x; } }
        private Expr _y; public Expr Ey { get { return _y; } }
        private Expr _z; public Expr Ez { get { return _z; } }
        private Expr _w; public Expr Ew { get { return _w; } }

        public CQuaternion(Expr x, Expr y, Expr z, Expr w)
        {
            _x = x;
            _y = y;
            _z = z;
            _w = w;
        }

        public static CQuaternion Frozen(Quaternion q)
        {
            q = CTransform.NormalizeQuaternion(q);
            var rest = new CQuaternion(
                Expr.Parameter(q.X),
                Expr.Parameter(q.Y),
                Expr.Parameter(q.Z),
                Expr.Parameter(q.W));
            foreach (Param p in rest)
            {
                p.Frozen = true;
                p.ScaleKind = ParamScaleKind.Orientation;
            }
            return rest;
        }

        public void SetValue(Quaternion q)
        {
            q = CTransform.NormalizeQuaternion(q);
            _x.SetValue(q.X);
            _y.SetValue(q.Y);
            _z.SetValue(q.Z);
            _w.SetValue(q.W);
        }

        public Quaternion Evaluate()
        {
            return new Quaternion(_x.Evaluate(), _y.Evaluate(), _z.Evaluate(), _w.Evaluate());
        }

        public IEnumerator<Param> GetEnumerator()
        {
            foreach (Param p in _x) yield return p;
            foreach (Param p in _y) yield return p;
            foreach (Param p in _z) yield return p;
            foreach (Param p in _w) yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }
}
