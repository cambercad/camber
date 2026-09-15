using Curves.Base;
using GeoCore;

namespace Curves
{
    public readonly struct SketchFontMetrics
    {
        public readonly double Ascent;
        public readonly double Descent;
        public readonly double LineHeight;

        public SketchFontMetrics(double ascent, double descent, double lineHeight)
        {
            Ascent = ascent;
            Descent = descent;
            LineHeight = lineHeight;
        }
    }

    public abstract class SketchTextProvider
    {
        public static SketchTextProvider Instance { get; set; } = TrueTypeSketchTextProvider.Instance;

        public abstract SketchFontMetrics GetMetrics(SketchFontSpec font);
        public abstract double GetAdvanceWidth(SketchFontSpec font, char c);
        public abstract IReadOnlyList<IReadOnlyList<Curve2D>> GetGlyphContours(SketchFontSpec font, char c, Vec2D origin);

        public List<List<Curve2D>> GenerateAt(string text, SketchFontSpec font, Vec2D origin)
        {
            font ??= new SketchFontSpec();
            text ??= string.Empty;

            var metrics = GetMetrics(font);
            double lineHeight = ResolveLineHeight(metrics, 1.2);
            double x = origin.X;
            double y = origin.Y;
            var result = new List<List<Curve2D>>();

            foreach (char c in text.Replace("\r", ""))
            {
                if (c == '\n')
                {
                    x = origin.X;
                    y -= lineHeight;
                    continue;
                }

                char glyph = c == '\t' ? ' ' : c;
                AppendContours(result, GetGlyphContours(font, glyph, new Vec2D(x, y)));
                x += GetAdvanceWidth(font, glyph);
            }

            return result;
        }

        public SketchTextLayoutResult Layout(string text, SketchTextLayoutOptions options)
        {
            if (options == null)
                throw new ArgumentNullException(nameof(options));

            text ??= string.Empty;
            var font = options.Font ?? new SketchFontSpec();
            Vec2D min = new Vec2D(Math.Min(options.Min.X, options.Max.X), Math.Min(options.Min.Y, options.Max.Y));
            Vec2D max = new Vec2D(Math.Max(options.Min.X, options.Max.X), Math.Max(options.Min.Y, options.Max.Y));
            double targetWidth = max.X - min.X;
            double targetHeight = max.Y - min.Y;

            var lines = BuildLines(text, font, options.WordWrap ? targetWidth : double.PositiveInfinity, options.MaxLines);
            var metrics = GetMetrics(font);
            double lineHeight = ResolveLineHeight(metrics, options.LineSpacing);
            double widestLine = lines.Count == 0 ? 0 : lines.Max(line => MeasureLine(font, line));
            double totalHeight = TextBlockHeight(metrics, lineHeight, lines.Count);
            double scale = ComputeAutoFitScale(options.AutoFit, targetWidth, targetHeight, widestLine, totalHeight);

            if (Math.Abs(scale - 1.0) > 1e-12)
            {
                font = font.WithEmSize(font.EmSize * scale);
                metrics = GetMetrics(font);
                lineHeight = ResolveLineHeight(metrics, options.LineSpacing);
                widestLine = lines.Count == 0 ? 0 : lines.Max(line => MeasureLine(font, line));
                totalHeight = TextBlockHeight(metrics, lineHeight, lines.Count);
            }

            double firstBaseline = FirstBaseline(min, max, metrics, lineHeight, lines.Count, totalHeight, options.VerticalAlignment);
            var contours = new List<List<Curve2D>>();

            for (int i = 0; i < lines.Count; i++)
            {
                string line = lines[i];
                double lineWidth = MeasureLine(font, line);
                double x = LineStartX(min.X, max.X, lineWidth, options.HorizontalAlignment);
                double y = firstBaseline - i * lineHeight;

                foreach (char raw in line)
                {
                    char glyph = raw == '\t' ? ' ' : raw;
                    AppendContours(contours, GetGlyphContours(font, glyph, new Vec2D(x, y)));
                    x += GetAdvanceWidth(font, glyph);
                }
            }

            BoundsOf(contours, out var boundsMin, out var boundsMax);
            return new SketchTextLayoutResult(contours, boundsMin, boundsMax, scale);
        }

        protected static List<List<Curve2D>> TransformContours(
            IReadOnlyList<IReadOnlyList<Curve2D>> contours,
            Func<Vec2D, Vec2D> transform)
            => SketchCurveTransform.TransformContours(contours, transform, DefaultCurveFactory.Instance);

        private static void AppendContours(List<List<Curve2D>> target, IReadOnlyList<IReadOnlyList<Curve2D>> source)
        {
            if (source == null)
                return;

            foreach (var contour in source)
            {
                if (contour.Count == 0)
                    continue;

                target.Add(contour.Select(CloneCurve).ToList());
            }
        }

        private static Curve2D CloneCurve(Curve2D curve)
        {
            var copy = curve.GetCopy();
            copy.Flags = curve.Flags;
            copy.Name = curve.Name;
            return copy;
        }

        private List<string> BuildLines(string text, SketchFontSpec font, double maxWidth, int? maxLines)
        {
            var result = new List<string>();
            string normalized = text.Replace("\r", "");
            foreach (string paragraph in normalized.Split('\n'))
            {
                if (double.IsPositiveInfinity(maxWidth) || maxWidth <= 0)
                {
                    result.Add(paragraph);
                }
                else
                {
                    AddWrappedParagraph(result, paragraph, font, maxWidth);
                }

                if (maxLines.HasValue && result.Count >= maxLines.Value)
                    break;
            }

            if (maxLines.HasValue && result.Count > maxLines.Value)
                result.RemoveRange(maxLines.Value, result.Count - maxLines.Value);

            return result;
        }

        private void AddWrappedParagraph(List<string> result, string paragraph, SketchFontSpec font, double maxWidth)
        {
            if (paragraph.Length == 0)
            {
                result.Add(string.Empty);
                return;
            }

            string[] words = paragraph.Split(' ');
            string current = string.Empty;
            foreach (string word in words)
            {
                string candidate = current.Length == 0 ? word : current + " " + word;
                if (current.Length > 0 && MeasureLine(font, candidate) > maxWidth)
                {
                    result.Add(current);
                    current = word;
                }
                else
                {
                    current = candidate;
                }
            }

            result.Add(current);
        }

        private double MeasureLine(SketchFontSpec font, string line)
        {
            double width = 0;
            foreach (char raw in line)
            {
                char c = raw == '\t' ? ' ' : raw;
                width += GetAdvanceWidth(font, c);
            }
            return width;
        }

        private static double ResolveLineHeight(SketchFontMetrics metrics, double lineSpacing)
        {
            double spacing = lineSpacing > 0 ? lineSpacing : 1.2;
            double measured = metrics.LineHeight > 0 ? metrics.LineHeight : metrics.Ascent + metrics.Descent;
            return measured * spacing;
        }

        private static double TextBlockHeight(SketchFontMetrics metrics, double lineHeight, int lineCount)
        {
            if (lineCount <= 0)
                return 0;
            return metrics.Ascent + metrics.Descent + (lineCount - 1) * lineHeight;
        }

        private static double ComputeAutoFitScale(bool autoFit, double targetWidth, double targetHeight, double width, double height)
        {
            if (!autoFit)
                return 1.0;

            double scale = double.PositiveInfinity;
            if (targetWidth > 0 && width > 0)
                scale = Math.Min(scale, targetWidth / width);
            if (targetHeight > 0 && height > 0)
                scale = Math.Min(scale, targetHeight / height);

            return double.IsPositiveInfinity(scale) || scale <= 0 ? 1.0 : scale;
        }

        private static double FirstBaseline(
            Vec2D min,
            Vec2D max,
            SketchFontMetrics metrics,
            double lineHeight,
            int lineCount,
            double totalHeight,
            SketchTextVerticalAlignment alignment)
        {
            return alignment switch
            {
                SketchTextVerticalAlignment.Top => max.Y - metrics.Ascent,
                SketchTextVerticalAlignment.Middle => (min.Y + max.Y + totalHeight) * 0.5 - metrics.Ascent,
                SketchTextVerticalAlignment.Bottom => min.Y + (lineCount - 1) * lineHeight + metrics.Descent,
                SketchTextVerticalAlignment.Baseline => min.Y,
                _ => max.Y - metrics.Ascent,
            };
        }

        private static double LineStartX(double minX, double maxX, double lineWidth, SketchTextHorizontalAlignment alignment)
        {
            return alignment switch
            {
                SketchTextHorizontalAlignment.Center => (minX + maxX - lineWidth) * 0.5,
                SketchTextHorizontalAlignment.Right => maxX - lineWidth,
                _ => minX,
            };
        }

        private static void BoundsOf(List<List<Curve2D>> contours, out Vec2D min, out Vec2D max)
        {
            bool any = false;
            double minX = double.PositiveInfinity;
            double minY = double.PositiveInfinity;
            double maxX = double.NegativeInfinity;
            double maxY = double.NegativeInfinity;

            foreach (var contour in contours)
            {
                foreach (var curve in contour)
                {
                    foreach (var vertex in curve.Tessellate(16))
                    {
                        any = true;
                        minX = Math.Min(minX, vertex.Position.X);
                        minY = Math.Min(minY, vertex.Position.Y);
                        maxX = Math.Max(maxX, vertex.Position.X);
                        maxY = Math.Max(maxY, vertex.Position.Y);
                    }
                }
            }

            min = any ? new Vec2D(minX, minY) : default;
            max = any ? new Vec2D(maxX, maxY) : default;
        }
    }
}
