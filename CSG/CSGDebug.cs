using GeoCore;

namespace CSG
{
    public static class CSGDebug
    {
        /// <summary>
        /// Exports the border polygon (as a closed loop) and constraints as .obj file lines.
        /// Border polygon is exported as a closed polyline (with 'l' statement).
        /// Constraints are exported as lines ('l' statement).
        /// Vertices are exported at the top as 'v x y 0'.
        /// </summary>
        public static void WriteLineObj(string fileName, List<Rat2Hybrid> bigRationalPoints, List<int> borderPolygon, List<Int2> constraints)
        {
            using (var writer = new System.IO.StreamWriter(fileName))
            {
                // Write vertices
                for (int i = 0; i < bigRationalPoints.Count; ++i)
                {
                    var p = bigRationalPoints[i];
                    // In .obj, vertices are 1-based
                    double x = p.X.ToDouble();
                    double y = p.Y.ToDouble();
                    writer.WriteLine($"v {x} {y} 0");
                }

                // Write border polygon as a closed loop
                if (borderPolygon != null && borderPolygon.Count > 0)
                {
                    for (int i = 0; i < borderPolygon.Count; ++i)
                    {
                        var s = borderPolygon[i] + 1;
                        var e = borderPolygon[(i + 1) % borderPolygon.Count] + 1;
                        writer.WriteLine($"l {s} {e}");
                    }
                }

                // Write constraints as lines
                if (constraints != null)
                {
                    foreach (var c in constraints)
                    {
                        // .obj is 1-based, Int2.X and Int2.Y are 0-based
                        int v1 = c.X + 1;
                        int v2 = c.Y + 1;
                        writer.WriteLine($"l {v1} {v2}");
                    }
                }
            }
        }
        public static void WriteLineObjRational(string fileName, List<Rat2Hybrid> bigRationalPoints, List<int> borderPolygon, List<Int2> constraints)
        {
            using (var writer = new System.IO.StreamWriter(fileName))
            {
                // Write vertices
                for (int i = 0; i < bigRationalPoints.Count; ++i)
                {
                    var p = bigRationalPoints[i];
                    // In .obj, vertices are 1-based                    
                    writer.WriteLine($"v {p.X.Numerator()}_{p.X.Denominator()} {p.Y.Numerator()}_{p.Y.Denominator()} 0");
                }

                // Write border polygon as a closed loop
                if (borderPolygon != null && borderPolygon.Count > 0)
                {
                    for (int i = 0; i < borderPolygon.Count; ++i)
                    {
                        var s = borderPolygon[i] + 1;
                        var e = borderPolygon[(i + 1) % borderPolygon.Count] + 1;
                        writer.WriteLine($"l {s} {e}");
                    }
                }

                // Write constraints as lines
                if (constraints != null)
                {
                    foreach (var c in constraints)
                    {
                        // .obj is 1-based, Int2.X and Int2.Y are 0-based
                        int v1 = c.X + 1;
                        int v2 = c.Y + 1;
                        writer.WriteLine($"l {v1} {v2}");
                    }
                }
            }
        }

        public static void AnalyzeConstraints(List<Rat2Hybrid> bigRationalPoints, List<Int2> constraints)
        {
            Rat2HybridArithmetic a = new Rat2HybridArithmetic();
            for (int i = 0; i < constraints.Count; ++i)
            {
                var c1 = constraints[i];
                for (int j = i + 1; j < constraints.Count; ++j)
                {
                    var c2 = constraints[j];

                    if (TriangulationHelper<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>.SegmentsIntersect(a, bigRationalPoints, c1.X, c1.Y, c2))
                    {

                    }
                }
            }
        }
    }
}
