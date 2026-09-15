using System.Collections.Generic;

namespace GeoMeta
{
    /// <summary>A named pickable: a point or a polyline curve.</summary>
    public sealed class NamedPickTarget
    {
        public string Name { get; set; }
        public string Kind { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public List<NamedPickPoint> Polyline { get; set; }
        public bool Closed { get; set; }
    }

    public sealed class NamedPickPoint
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
    }
}
