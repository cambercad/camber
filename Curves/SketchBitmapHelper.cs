using GeoCore;

namespace Curves
{
    /// <summary>
    /// Display-only sketch overlay: a bitmap drawn as a textured quad on the sketch plane.
    /// Does not participate in tessellation, constraints, or solid generation.
    /// </summary>
    public sealed class SketchBitmapHelper
    {
        public string ImagePath { get; }
        public Vec2D Center { get; }
        /// <summary>Sketch-space width. Null means derive from <see cref="Height"/> and image aspect.</summary>
        public double? Width { get; }
        /// <summary>Sketch-space height. Null means derive from <see cref="Width"/> and image aspect.</summary>
        public double? Height { get; }
        /// <summary>Multiplies texture alpha (0–1).</summary>
        public float Opacity { get; }

        public SketchBitmapHelper(string imagePath, Vec2D center, double? width, double? height, float opacity = 1f)
        {
            if (string.IsNullOrWhiteSpace(imagePath))
                throw new ArgumentException("Image path is required.", nameof(imagePath));
            if (!width.HasValue && !height.HasValue)
                throw new ArgumentException("Specify width, height, or both.", nameof(width));
            if (width.HasValue && width.Value <= 0)
                throw new ArgumentOutOfRangeException(nameof(width), "Width must be positive.");
            if (height.HasValue && height.Value <= 0)
                throw new ArgumentOutOfRangeException(nameof(height), "Height must be positive.");
            if (opacity <= 0 || opacity > 1.0001f)
                throw new ArgumentOutOfRangeException(nameof(opacity), "Opacity must be in (0, 1].");

            ImagePath = imagePath;
            Center = center;
            Width = width;
            Height = height;
            Opacity = Math.Clamp(opacity, 0.001f, 1f);
        }

        /// <summary>
        /// Resolves sketch-space width/height from optional sizes and pixel aspect (width/height of the image).
        /// </summary>
        public void ResolveSize(int pixelWidth, int pixelHeight, out double width, out double height)
        {
            if (pixelWidth <= 0 || pixelHeight <= 0)
                throw new ArgumentException("Image pixel size must be positive.");

            double aspect = pixelWidth / (double)pixelHeight;
            if (Width.HasValue && Height.HasValue)
            {
                width = Width.Value;
                height = Height.Value;
            }
            else if (Width.HasValue)
            {
                width = Width.Value;
                height = width / aspect;
            }
            else
            {
                height = Height!.Value;
                width = height * aspect;
            }
        }
    }
}
