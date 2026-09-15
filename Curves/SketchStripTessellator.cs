using GeoCore;

namespace Curves
{
    public static class SketchStripTessellator
    {
        public const double DefaultClosedToleranceSq = 1e-16;

        public static void TessellateStrip(
            IReadOnlyList<Curve2D> strip,
            double maxDeviation,
            SketchStripTessellateOptions options,
            out List<List<Vec2D>> segmentPoints,
            out List<List<Vec2D>> segmentNormals)
        {
            TessellateStrip(strip, maxDeviation, options, out segmentPoints, out segmentNormals, null, -1);
        }

        public static void TessellateStrip(
            IReadOnlyList<Curve2D> strip,
            double maxDeviation,
            SketchStripTessellateOptions options,
            out List<List<Vec2D>> segmentPoints,
            out List<List<Vec2D>> segmentNormals,
            IList<string> curveNames,
            int stripIndex)
        {
            segmentPoints = new List<List<Vec2D>>();
            segmentNormals = new List<List<Vec2D>>();

            if (strip == null || strip.Count == 0)
                return;

            int nameIndex = 0;
            for (int i = 0; i < strip.Count; i++)
            {
                var curve = strip[i];
                if (!options.IncludeHelperGeometry && curve.IsHelperGeometry)
                    continue;

                var t = curve.Tessellate(maxDeviation);
                var c = new List<Vec2D>(t.Count);
                var n = new List<Vec2D>(t.Count);
                for (int j = 0; j < t.Count; j++)
                {
                    c.Add(t[j].Position);
                    n.Add(t[j].Normal);
                }

                segmentPoints.Add(c);
                segmentNormals.Add(n);
                curveNames?.Add(curve.Name);
                nameIndex++;
            }

            if (segmentPoints.Count == 0 || options.SkipValidation)
                return;

            ValidateStripTessellation(segmentPoints, options, curveNames, stripIndex);
        }

        public static void ValidateStripTessellation(
            List<List<Vec2D>> segmentPoints,
            SketchStripTessellateOptions options,
            IList<string> curveNames,
            int stripIndex)
        {
            int checkCount = options.AllowOpenContour ? segmentPoints.Count - 1 : segmentPoints.Count;
            double tol = options.CurveMatchingTolerance;

            for (int i = 0; i < checkCount; i++)
            {
                var end = segmentPoints[i][^1];
                var start = segmentPoints[(i + 1) % segmentPoints.Count][0];
                double l2 = Vec2DOps.DistanceSquared(start, end);
                if (l2 > tol * tol)
                {
                    double distance = Math.Sqrt(l2);
                    if (stripIndex >= 0 && curveNames != null && curveNames.Count == segmentPoints.Count)
                    {
                        Console.WriteLine($"Sketch strip tessellation gap at strip {stripIndex}, curve {i} -> {(i + 1) % segmentPoints.Count}: {distance:E3} (tolerance: {tol:E3})");
                        Console.WriteLine($"  Curve names: {curveNames[i]} -> {curveNames[(i + 1) % segmentPoints.Count]}");
                    }

                    throw new Exception($"Tessellation gap of {distance:E3} exceeds tolerance {tol:E3}");
                }

                segmentPoints[(i + 1) % segmentPoints.Count][0] = end;
            }
        }

        public static void FlattenStripWithCreases(
            List<List<Vec2D>> points,
            List<List<Vec2D>> norms,
            out List<Vec2D> poly,
            out List<Vec2D> pn,
            out List<int> creaseStartIndicesInPoly)
        {
            poly = new List<Vec2D>();
            pn = new List<Vec2D>();
            creaseStartIndicesInPoly = new List<int>();
            for (int si = 0; si < points.Count; si++)
            {
                var seg = points[si];
                var ns = norms[si];
                if (si > 0)
                    creaseStartIndicesInPoly.Add(poly.Count);
                for (int j = 0; j < seg.Count - 1; j++)
                {
                    poly.Add(seg[j]);
                    pn.Add(ns[j]);
                }
            }

            if (points.Count > 0)
            {
                var lastSeg = points[^1];
                var lastN = norms[^1];
                poly.Add(lastSeg[^1]);
                pn.Add(lastN[^1]);
            }

            if (points.Count > 1 && poly.Count >= 2 &&
                Vec2DOps.DistanceSquared(poly[0], poly[^1]) <= DefaultClosedToleranceSq &&
                !creaseStartIndicesInPoly.Contains(0))
                creaseStartIndicesInPoly.Add(0);
        }

        public static bool IsClosedPolyline(IReadOnlyList<Vec2D> polyline, double toleranceSq = DefaultClosedToleranceSq)
        {
            if (polyline == null || polyline.Count < 3)
                return false;
            return Vec2DOps.DistanceSquared(polyline[0], polyline[^1]) <= toleranceSq;
        }
    }
}
