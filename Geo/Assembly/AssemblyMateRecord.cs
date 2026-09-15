using GeoCore;
using System.Collections.Generic;

namespace Geo
{
    public enum AssemblyMateKind
    {
        FixPart,
        CoincidentPoints,
        CoincidentAxes,
        CoincidentPlanes,
        ParallelAxes,
        ParallelPlanes,
        PerpendicularAxes,
        PerpendicularPlanes,
        Concentric,
        DistancePoints,
        DistancePlanes,
        AngleAxes,
        PointOnPlane,
        Contact,
    }

    /// <summary>User-facing metadata for one assembly mate (used by viewers to draw mate glyphs).</summary>
    public sealed class AssemblyMateRecord
    {
        private readonly List<string> _entities = new List<string>();

        internal AssemblyMateRecord(
            AssemblyMateKind kind,
            string label,
            AssemblyPart partA,
            AssemblyPart partB = null,
            Vec3D localA = default,
            Vec3D localB = default,
            Vec3D dirA = default,
            Vec3D dirB = default,
            double scalar = double.NaN)
        {
            Kind = kind;
            Label = label;
            PartA = partA;
            PartB = partB;
            LocalA = localA;
            LocalB = localB;
            DirA = dirA;
            DirB = dirB;
            Scalar = scalar;
        }

        public AssemblyMateKind Kind { get; }
        public string Label { get; }
        public AssemblyPart PartA { get; }
        public AssemblyPart PartB { get; }
        public Vec3D LocalA { get; }
        public Vec3D LocalB { get; }
        public Vec3D DirA { get; }
        public Vec3D DirB { get; }
        public double Scalar { get; }

        /// <summary>Camber entity strings this mate refers to, in operand order (A, B, …).</summary>
        public IReadOnlyList<string> Entities { get { return _entities; } }

        internal AssemblyMateRecord WithEntities(params string[] entities)
        {
            _entities.Clear();
            if (entities == null)
                return this;
            for (int i = 0; i < entities.Length; i++)
            {
                string name = entities[i];
                if (string.IsNullOrEmpty(name))
                    continue;
                _entities.Add(name);
            }
            return this;
        }
    }
}
