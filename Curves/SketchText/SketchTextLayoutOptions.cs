using GeoCore;

namespace Curves
{
    public class SketchTextLayoutOptions
    {
        public Vec2D Min { get; set; }
        public Vec2D Max { get; set; }
        public SketchFontSpec Font { get; set; }
        public SketchTextHorizontalAlignment HorizontalAlignment { get; set; }
        public SketchTextVerticalAlignment VerticalAlignment { get; set; }
        public bool AutoFit { get; set; }
        public bool WordWrap { get; set; }
        public double LineSpacing { get; set; }
        public int? MaxLines { get; set; }

        public SketchTextLayoutOptions()
        {
            Font = new SketchFontSpec();
            HorizontalAlignment = SketchTextHorizontalAlignment.Left;
            VerticalAlignment = SketchTextVerticalAlignment.Top;
            AutoFit = false;
            WordWrap = true;
            LineSpacing = 1.2;
        }

        public SketchTextLayoutOptions(Vec2D min, Vec2D max, SketchFontSpec font)
            : this()
        {
            Min = min;
            Max = max;
            Font = font ?? new SketchFontSpec();
        }
    }

    public class SketchTextLayoutResult
    {
        public List<List<Curve2D>> Contours { get; }
        public Vec2D Min { get; }
        public Vec2D Max { get; }
        public double Scale { get; }

        public SketchTextLayoutResult(List<List<Curve2D>> contours, Vec2D min, Vec2D max, double scale)
        {
            Contours = contours;
            Min = min;
            Max = max;
            Scale = scale;
        }
    }
}
