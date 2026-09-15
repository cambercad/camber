using GeoCore;
using GeoMeta;

namespace Curves
{
    [APIDescription(@"SketchOffsetJoinType: corner style for OffsetStrip / SketchStripOffsetOptions.
  Square — extended square corners.
  Round — rounded corners (uses CornerArcTolerance).
  Miter — sharp mitered corners.")]
    public enum SketchOffsetJoinType
    {
        Square,
        Round,
        Miter,
    }
}
