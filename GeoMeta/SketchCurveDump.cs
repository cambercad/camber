using System.Globalization;

namespace GeoMeta
{
    /// <summary>Tab-separated sketch dump consumed by the Python viewer.</summary>
    public static class SketchCurveDump
    {
        public static string Line(string name, double x0, double y0, double x1, double y1)
        {
            return string.Format(
                CultureInfo.InvariantCulture,
                "L\t{0}\t{1:R}\t{2:R}\t{3:R}\t{4:R}",
                name ?? "", x0, y0, x1, y1);
        }

        public static string Circle(string name, double cx, double cy, double radius)
        {
            return string.Format(
                CultureInfo.InvariantCulture,
                "C\t{0}\t{1:R}\t{2:R}\t{3:R}",
                name ?? "", cx, cy, radius);
        }

        public static string Arc(string name, double x0, double y0, double xm, double ym, double x1, double y1)
        {
            return string.Format(
                CultureInfo.InvariantCulture,
                "A\t{0}\t{1:R}\t{2:R}\t{3:R}\t{4:R}\t{5:R}\t{6:R}",
                name ?? "", x0, y0, xm, ym, x1, y1);
        }
    }
}
