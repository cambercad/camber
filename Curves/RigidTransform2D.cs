using GeoCore;

namespace Curves
{
    /// <summary>
    /// Immutable rigid 2D transform: optional rotation about a center, then translation.
    /// </summary>
    public readonly struct RigidTransform2D
    {
        public Vec2D Translation { get; }
        public Vec2D RotationCenter { get; }
        public double RotationAngle { get; }

        public RigidTransform2D(Vec2D translation, Vec2D rotationCenter, double rotationAngle)
        {
            Translation = translation;
            RotationCenter = rotationCenter;
            RotationAngle = rotationAngle;
        }

        public Vec2D Map(Vec2D p)
        {
            Vec2D rotated = p;
            if (RotationAngle != 0.0)
            {
                Vec2D offset = p - RotationCenter;
                rotated = RotationCenter + Vec2DOps.Rotated(offset, RotationAngle);
            }

            return rotated + Translation;
        }

        public static RigidTransform2D Translate(Vec2D delta)
            => new RigidTransform2D(delta, Vec2DOps.Zero, 0.0);

        public static RigidTransform2D RotateAbout(Vec2D center, double angle)
            => new RigidTransform2D(Vec2DOps.Zero, center, angle);

        public static RigidTransform2D Identity
            => new RigidTransform2D(Vec2DOps.Zero, Vec2DOps.Zero, 0.0);
    }
}
