using GeoCore;
using GeoMeta;

namespace Curves
{
    [APIDescription(@"SketchOffsetEndCap: end-cap style when OpenMode is Outline on an open strip.
  Round — semicircle caps (uses CornerArcTolerance).
  Butt — flat caps flush with the path ends.
  Square — square caps extending by the offset distance.
Ignored for Parallel mode and for closed source strips.")]
    public enum SketchOffsetEndCap
    {
        Round,
        Butt,
        Square,
    }
}
