using GeoCore;
using GeoMeta;

namespace Curves
{
    [APIDescription(@"SketchOffsetOpenMode: how OffsetStrip treats open (non-closed) source strips.
  Parallel — one-sided offset along the left of travel for positive `offset` (open result).
  Outline — both sides plus end caps, producing a closed outline around the path.
Ignored for closed source strips (always a closed polygon offset).")]
    public enum SketchOffsetOpenMode
    {
        Parallel,
        Outline,
    }
}
