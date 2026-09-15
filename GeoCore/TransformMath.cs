namespace GeoCore
{
    public static class TransformMath
    {
        public static readonly Quaternion IdentityOrientation = new Quaternion(0, 0, 0, 1);

        public static Quaternion NormalizeDefault(Quaternion orientation)
        {
            if (orientation.X == 0 && orientation.Y == 0 && orientation.Z == 0 && orientation.W == 0)
                return IdentityOrientation;
            return orientation;
        }

        public static Mat4D ToMat4D(in Transform transform)
        {
            Quaternion q = NormalizeDefault(transform.Orientation);
            double x = q.X, y = q.Y, z = q.Z, w = q.W;

            double xx = x * x, yy = y * y, zz = z * z;
            double xy = x * y, xz = x * z, yz = y * z;
            double wx = w * x, wy = w * y, wz = w * z;

            return new Mat4D(
                1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), transform.Position.X,
                2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), transform.Position.Y,
                2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), transform.Position.Z,
                0, 0, 0, 1);
        }

        public static bool TryInverseRigid(in Mat4D m, out Mat4D inverse)
        {
            return Mat4DOps.Inverse(in m, out inverse);
        }

        public static Mat4D InverseRigid(in Transform transform)
        {
            Mat4D m = ToMat4D(in transform);
            if (!Mat4DOps.Inverse(in m, out Mat4D inverse))
                throw new InvalidOperationException("Rigid transform is not invertible.");
            return inverse;
        }

        public static Vec3D TransformPoint(in Transform transform, Vec3D localPoint)
        {
            return ToMat4D(in transform).TransformPoint(localPoint);
        }

        public static Vec3D TransformDirection(in Transform transform, Vec3D localDirection)
        {
            return ToMat4D(in transform).TransformDirection(localDirection);
        }

        public static Vec3D InverseTransformPoint(in Transform transform, Vec3D worldPoint)
        {
            return InverseRigid(in transform).TransformPoint(worldPoint);
        }

        public static Vec3D InverseTransformDirection(in Transform transform, Vec3D worldDirection)
        {
            return InverseRigid(in transform).TransformDirection(worldDirection);
        }

        public static Vec3D RotateVector(in Quaternion orientation, Vec3D vector)
        {
            Quaternion q = NormalizeDefault(orientation);
            double x = q.X, y = q.Y, z = q.Z, w = q.W;

            double tx = 2 * (y * vector.Z - z * vector.Y);
            double ty = 2 * (z * vector.X - x * vector.Z);
            double tz = 2 * (x * vector.Y - y * vector.X);

            return new Vec3D(
                vector.X + w * tx + (y * tz - z * ty),
                vector.Y + w * ty + (z * tx - x * tz),
                vector.Z + w * tz + (x * ty - y * tx));
        }

        /// <summary>
        /// Parent ∘ local: apply <paramref name="local"/> first, then <paramref name="parent"/>.
        /// </summary>
        public static Transform Compose(in Transform parent, in Transform local)
        {
            Quaternion parentQ = NormalizeDefault(parent.Orientation);
            Quaternion localQ = NormalizeDefault(local.Orientation);
            Vec3D position = parent.Position + RotateVector(in parentQ, local.Position);
            Quaternion orientation = ComposeOrientations(parentQ, localQ);
            return new Transform(position, orientation);
        }

        public static Transform Inverse(in Transform transform)
        {
            Quaternion q = NormalizeDefault(transform.Orientation);
            Quaternion qInv = new Quaternion(-q.X, -q.Y, -q.Z, q.W);
            Vec3D negPos = new Vec3D(-transform.Position.X, -transform.Position.Y, -transform.Position.Z);
            return new Transform(RotateVector(in qInv, negPos), qInv);
        }

        public static Quaternion ComposeOrientations(Quaternion a, Quaternion b)
        {
            a = NormalizeDefault(a);
            b = NormalizeDefault(b);
            return new Quaternion(
                a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
                a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
                a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
                a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);
        }
    }
}
