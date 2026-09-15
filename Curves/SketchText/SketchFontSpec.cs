namespace Curves
{
    [Flags]
    public enum SketchFontStyleFlags
    {
        Regular = 0,
        Bold = 1,
        Italic = 2,
        Underline = 4,
        Strikeout = 8,
    }

    public class SketchFontSpec
    {
        public string Family { get; set; }
        public SketchFontStyleFlags Style { get; set; }
        public double EmSize { get; set; }

        public SketchFontSpec()
            : this("Calibri", 100.0, SketchFontStyleFlags.Regular)
        {
        }

        public SketchFontSpec(string family, double emSize)
            : this(family, emSize, SketchFontStyleFlags.Regular)
        {
        }

        public SketchFontSpec(string family, double emSize, SketchFontStyleFlags style)
        {
            Family = string.IsNullOrWhiteSpace(family) ? "Calibri" : family;
            EmSize = emSize;
            Style = style;
        }

        public SketchFontSpec WithEmSize(double emSize)
        {
            return new SketchFontSpec(Family, emSize, Style);
        }
    }
}
