namespace Curves
{
    public readonly struct SketchStripTessellateOptions
    {
        public bool IncludeHelperGeometry { get; init; }
        public bool AllowOpenContour { get; init; }
        public bool SkipValidation { get; init; }
        public double CurveMatchingTolerance { get; init; }

        public static SketchStripTessellateOptions FromSketchFlags(
            SketchTessellationFlags flags,
            double curveMatchingTolerance = 1e-8)
            => new SketchStripTessellateOptions
            {
                IncludeHelperGeometry = (flags & SketchTessellationFlags.ExcludeHelperGeometry) == 0,
                AllowOpenContour = (flags & SketchTessellationFlags.AllowOpenContour) != 0,
                SkipValidation = (flags & SketchTessellationFlags.SkipValidation) != 0,
                CurveMatchingTolerance = curveMatchingTolerance,
            };

        public static SketchStripTessellateOptions DefaultOpen(double curveMatchingTolerance = 1e-8)
            => new SketchStripTessellateOptions
            {
                IncludeHelperGeometry = true,
                AllowOpenContour = true,
                SkipValidation = false,
                CurveMatchingTolerance = curveMatchingTolerance,
            };
    }
}
