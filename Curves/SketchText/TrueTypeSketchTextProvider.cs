using GeoCore;

namespace Curves
{
    public sealed class TrueTypeSketchTextProvider : SketchTextProvider
    {
        public static readonly TrueTypeSketchTextProvider Instance = new TrueTypeSketchTextProvider();

        private readonly TrueTypeFontCatalog _catalog;

        public TrueTypeSketchTextProvider()
        {
            _catalog = TrueTypeFontCatalog.Shared;
        }

        public TrueTypeSketchTextProvider(IReadOnlyList<string> extraSearchPaths)
        {
            _catalog = TrueTypeFontCatalog.FromSearchPaths(extraSearchPaths);
        }

        private TrueTypeSketchTextProvider(TrueTypeFontCatalog catalog)
        {
            _catalog = catalog ?? TrueTypeFontCatalog.Shared;
        }

        public static TrueTypeSketchTextProvider FromFontFile(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
                throw new ArgumentException("Font path is empty.", nameof(path));
            return new TrueTypeSketchTextProvider(TrueTypeFontCatalog.FromFontFile(path));
        }

        public override SketchFontMetrics GetMetrics(SketchFontSpec font)
        {
            var face = RequireFont(font);
            double scale = Scale(font, face);
            return new SketchFontMetrics(face.Ascent * scale, face.Descent * scale, face.LineHeight * scale);
        }

        public override double GetAdvanceWidth(SketchFontSpec font, char c)
        {
            if (c == '\r' || c == '\n')
                return 0;

            var face = RequireFont(font);
            return face.GetAdvanceWidth(c) * Scale(font, face);
        }

        public override IReadOnlyList<IReadOnlyList<Curve2D>> GetGlyphContours(SketchFontSpec font, char c, Vec2D origin)
        {
            if (char.IsWhiteSpace(c))
                return Array.Empty<IReadOnlyList<Curve2D>>();

            var face = RequireFont(font);
            return face.GetContours(c, origin, SafeEm(font));
        }

        public string ResolvedFamily(SketchFontSpec font)
        {
            var face = _catalog.ResolveRef(font);
            if (face == null || face.Names == null)
                return string.Empty;
            return face.Names.BestFamily;
        }

        private TrueTypeFontFile RequireFont(SketchFontSpec font)
        {
            TrueTypeFontFile loaded;
            if (_catalog.TryResolve(font, out loaded))
                return loaded;

            string family = font != null && !string.IsNullOrWhiteSpace(font.Family) ? font.Family : "Calibri";
            throw new InvalidOperationException(
                "No TrueType (glyf) font found for family '" + family + "'. Install a .ttf font such as Arial, Liberation Sans, or DejaVu Sans.");
        }

        private static double Scale(SketchFontSpec font, TrueTypeFontFile face)
        {
            int em = face.UnitsPerEm > 0 ? face.UnitsPerEm : 1000;
            return SafeEm(font) / em;
        }

        private static double SafeEm(SketchFontSpec font)
        {
            return font != null && font.EmSize > 0 ? font.EmSize : 100.0;
        }
    }
}
