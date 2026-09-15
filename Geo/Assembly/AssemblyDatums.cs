using GeoCore;
using GeoMeta;
using GeoSolver.Kinematics;

namespace Geo
{
    [APIDescription(@"AssemblyPointDatum: point datum on an AssemblyPart (returned from AddPointDatum). Use in SetCoincident mates.")]
    public sealed class AssemblyPointDatum
    {
        internal AssemblyPointDatum(AssemblyPart part, Vec3D localPoint, string entity = "")
        {
            Part = part;
            LocalPoint = localPoint;
            Entity = entity ?? "";
        }

        internal AssemblyPart Part { get; }
        public Vec3D LocalPoint { get; }

        [APIDescription(@"Entity (str): camber pick name of the named geometry this datum was created from, or empty for AddPointDatumAt.")]
        public string Entity { get; }
    }

    [APIDescription(@"AssemblyAxisDatum: axis datum on an AssemblyPart (returned from AddAxisDatum). Exposes LocalPoint and LocalDirection; use in SetCoincident / SetParallel mates.")]
    public sealed class AssemblyAxisDatum
    {
        internal AssemblyAxisDatum(AssemblyPart part, Vec3D localPoint, Vec3D localDirection, string entity = "")
        {
            Part = part;
            LocalPoint = localPoint;
            LocalDirection = localDirection;
            Entity = entity ?? "";
        }

        internal AssemblyPart Part { get; }
        public Vec3D LocalPoint { get; }
        public Vec3D LocalDirection { get; }

        [APIDescription(@"Entity (str): camber pick name of the named geometry this datum was created from, or empty for AddAxisDatumAt.")]
        public string Entity { get; }
    }

    [APIDescription(@"AssemblyPlaneDatum: planar face datum on an AssemblyPart (returned from AddPlaneDatum). Exposes LocalOrigin and LocalNormal; use in face mates (SetCoincident / SetParallel / SetPerpendicular / SetDistance).")]
    public sealed class AssemblyPlaneDatum
    {
        internal AssemblyPlaneDatum(AssemblyPart part, Vec3D localOrigin, Vec3D localNormal, string entity = "")
        {
            Part = part;
            LocalOrigin = localOrigin;
            LocalNormal = localNormal;
            Entity = entity ?? "";
        }

        internal AssemblyPart Part { get; }
        public Vec3D LocalOrigin { get; }
        public Vec3D LocalNormal { get; }

        [APIDescription(@"Entity (str): camber pick name of the named geometry this datum was created from, or empty for AddPlaneDatumAt.")]
        public string Entity { get; }
    }
}
