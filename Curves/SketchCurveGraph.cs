using GeoCore;

namespace Curves
{
    /// <summary>
    /// Walks a curve collection as an endpoint graph so T-junctions and forks
    /// become separate paths (split at vertices of degree ≠ 2).
    /// </summary>
    public static class SketchCurveGraph
    {
        public class Path
        {
            public List<Curve2D> Curves { get; set; }
            public List<bool> Reversed { get; set; }
            public bool IsClosed { get; set; }

            public Path()
            {
                Curves = new List<Curve2D>();
                Reversed = new List<bool>();
            }
        }

        public static List<Path> SplitIntoPaths(IReadOnlyList<Curve2D> curves, double tolerance)
        {
            if (curves == null || curves.Count == 0)
                throw new ArgumentException("Curve list must not be empty.", nameof(curves));

            double tol = tolerance > 0 ? tolerance : 1e-6;
            double tolSq = tol * tol;
            var vertices = new List<Vec2D>();
            var startAt = new int[curves.Count];
            var endAt = new int[curves.Count];

            for (int i = 0; i < curves.Count; i++)
            {
                startAt[i] = IndexOfVertex(vertices, curves[i].StartPosition, tolSq);
                endAt[i] = IndexOfVertex(vertices, curves[i].EndPosition, tolSq);
            }

            var incident = new List<int>[vertices.Count];
            for (int v = 0; v < vertices.Count; v++)
                incident[v] = new List<int>();
            for (int i = 0; i < curves.Count; i++)
            {
                incident[startAt[i]].Add(i);
                if (endAt[i] != startAt[i])
                    incident[endAt[i]].Add(i);
            }

            var unused = new bool[curves.Count];
            for (int i = 0; i < curves.Count; i++)
                unused[i] = true;

            var paths = new List<Path>();
            foreach (int seed in PreferredSeeds(incident, unused, startAt, endAt))
            {
                if (!unused[seed])
                    continue;
                paths.Add(WalkPath(seed, curves, unused, startAt, endAt, incident));
            }

            return paths;
        }

        static int IndexOfVertex(List<Vec2D> vertices, Vec2D point, double tolSq)
        {
            for (int i = 0; i < vertices.Count; i++)
            {
                if (Vec2DOps.DistanceSquared(vertices[i], point) <= tolSq)
                    return i;
            }
            vertices.Add(point);
            return vertices.Count - 1;
        }

        static IEnumerable<int> PreferredSeeds(
            List<int>[] incident, bool[] unused, int[] startAt, int[] endAt)
        {
            for (int i = 0; i < unused.Length; i++)
            {
                if (!unused[i])
                    continue;
                if (Degree(incident, startAt[i]) != 2 || Degree(incident, endAt[i]) != 2)
                    yield return i;
            }
            for (int i = 0; i < unused.Length; i++)
            {
                if (unused[i])
                    yield return i;
            }
        }

        static int Degree(List<int>[] incident, int vertex)
        {
            return incident[vertex].Count;
        }

        static Path WalkPath(
            int seed,
            IReadOnlyList<Curve2D> curves,
            bool[] unused,
            int[] startAt,
            int[] endAt,
            List<int>[] incident)
        {
            var path = new Path();
            unused[seed] = false;

            int startVertex = startAt[seed];
            int endVertex = endAt[seed];
            bool seedReversed = false;
            if (Degree(incident, startAt[seed]) == 2 && Degree(incident, endAt[seed]) != 2)
            {
                startVertex = endAt[seed];
                endVertex = startAt[seed];
                seedReversed = true;
            }

            path.Curves.Add(curves[seed]);
            path.Reversed.Add(seedReversed);

            int current = endVertex;
            int origin = startVertex;
            while (true)
            {
                if (path.Curves.Count > 1 && current == origin)
                {
                    path.IsClosed = true;
                    break;
                }
                if (Degree(incident, current) != 2)
                    break;
                int next = UnusedAt(incident[current], unused);
                if (next < 0)
                {
                    if (current == origin)
                        path.IsClosed = true;
                    break;
                }
                bool reversed = startAt[next] != current;
                unused[next] = false;
                path.Curves.Add(curves[next]);
                path.Reversed.Add(reversed);
                current = OtherEnd(next, current, startAt, endAt);
            }

            if (!path.IsClosed && Degree(incident, origin) == 2)
            {
                current = origin;
                while (Degree(incident, current) == 2)
                {
                    int next = UnusedAt(incident[current], unused);
                    if (next < 0)
                        break;
                    bool reversed = startAt[next] == current;
                    unused[next] = false;
                    path.Curves.Insert(0, curves[next]);
                    path.Reversed.Insert(0, reversed);
                    current = OtherEnd(next, current, startAt, endAt);
                }
            }

            if (path.Curves.Count == 1 && startAt[seed] == endAt[seed])
                path.IsClosed = true;
            return path;
        }

        static int OtherEnd(int curve, int vertex, int[] startAt, int[] endAt)
        {
            return startAt[curve] == vertex ? endAt[curve] : startAt[curve];
        }

        static int UnusedAt(List<int> incident, bool[] unused)
        {
            for (int i = 0; i < incident.Count; i++)
            {
                int curve = incident[i];
                if (unused[curve])
                    return curve;
            }
            return -1;
        }
    }
}
