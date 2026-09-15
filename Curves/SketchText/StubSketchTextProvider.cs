using GeoCore;

namespace Curves
{
    public sealed class StubSketchTextProvider : SketchTextProvider
    {
        public override SketchFontMetrics GetMetrics(SketchFontSpec font)
        {
            double em = SafeEm(font);
            return new SketchFontMetrics(0.8 * em, 0.2 * em, em);
        }

        public override double GetAdvanceWidth(SketchFontSpec font, char c)
        {
            double em = SafeEm(font);
            return char.IsWhiteSpace(c) ? 0.5 * em : em;
        }

        public override IReadOnlyList<IReadOnlyList<Curve2D>> GetGlyphContours(SketchFontSpec font, char c, Vec2D origin)
        {
            if (char.IsWhiteSpace(c))
                return Array.Empty<IReadOnlyList<Curve2D>>();

            double em = SafeEm(font);
            double bottom = origin.Y - 0.2 * em;
            double top = origin.Y + 0.8 * em;
            double left = origin.X;
            double right = origin.X + em;

            var contours = new List<IReadOnlyList<Curve2D>> { Rectangle(left, bottom, right, top, clockwise: false) };

            if (c == 'O' || c == 'o' || c == '0')
            {
                double inset = 0.25 * em;
                contours.Add(Rectangle(left + inset, bottom + inset, right - inset, top - inset, clockwise: true));
            }

            return contours;
        }

        private static double SafeEm(SketchFontSpec font)
        {
            return font != null && font.EmSize > 0 ? font.EmSize : 1.0;
        }

        private static List<Curve2D> Rectangle(double left, double bottom, double right, double top, bool clockwise)
        {
            Vec2D bottomLeft = new Vec2D(left, bottom);
            Vec2D bottomRight = new Vec2D(right, bottom);
            Vec2D topRight = new Vec2D(right, top);
            Vec2D topLeft = new Vec2D(left, top);

            if (clockwise)
            {
                return new List<Curve2D>
                {
                    new Line2D(bottomLeft, topLeft),
                    new Line2D(topLeft, topRight),
                    new Line2D(topRight, bottomRight),
                    new Line2D(bottomRight, bottomLeft),
                };
            }

            return new List<Curve2D>
            {
                new Line2D(bottomLeft, bottomRight),
                new Line2D(bottomRight, topRight),
                new Line2D(topRight, topLeft),
                new Line2D(topLeft, bottomLeft),
            };
        }
    }
}
