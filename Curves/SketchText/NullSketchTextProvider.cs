using GeoCore;

namespace Curves
{
    public sealed class NullSketchTextProvider : SketchTextProvider
    {
        public static readonly NullSketchTextProvider Instance = new NullSketchTextProvider();

        private NullSketchTextProvider()
        {
        }

        public override SketchFontMetrics GetMetrics(SketchFontSpec font)
        {
            throw NotConfigured();
        }

        public override double GetAdvanceWidth(SketchFontSpec font, char c)
        {
            throw NotConfigured();
        }

        public override IReadOnlyList<IReadOnlyList<Curve2D>> GetGlyphContours(SketchFontSpec font, char c, Vec2D origin)
        {
            throw NotConfigured();
        }

        private static InvalidOperationException NotConfigured()
        {
            return new InvalidOperationException("SketchTextProvider not configured. The default is TrueTypeSketchTextProvider; host apps may also set GdiSketchTextProvider on Windows.");
        }
    }
}
