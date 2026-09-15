using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GeoCore
{
    public class DebugWrite
    {
        /// <summary>
        /// Writes 2D points to a simple text file format
        /// </summary>
        public static void Write(List<Rat2Hybrid> points, string path)
        {
            using (StreamWriter writer = new StreamWriter(path))
            {
                writer.WriteLine("POINTS");
                writer.WriteLine(points.Count);
                foreach (var point in points)
                {
                    writer.WriteLine($"{point.X.ToDouble().ToString(CultureInfo.InvariantCulture)} {point.Y.ToDouble().ToString(CultureInfo.InvariantCulture)}");
                }
            }
        }

        /// <summary>
        /// Writes triangles to a simple text file format
        /// </summary>
        public static void Write(List<Tri> triangles, string path)
        {
            using (StreamWriter writer = new StreamWriter(path))
            {
                writer.WriteLine("TRIANGLES");
                writer.WriteLine(triangles.Count);
                foreach (var tri in triangles)
                {
                    writer.WriteLine($"{tri.A} {tri.B} {tri.C}");
                }
            }
        }

        /// <summary>
        /// Writes segments (edges) to a simple text file format
        /// </summary>
        public static void Write(List<Int2> segments, string path)
        {
            using (StreamWriter writer = new StreamWriter(path))
            {
                writer.WriteLine("SEGMENTS");
                writer.WriteLine(segments.Count);
                foreach (var seg in segments)
                {
                    writer.WriteLine($"{seg.X} {seg.Y}");
                }
            }
        }

        /// <summary>
        /// Reads 2D points from a text file
        /// </summary>
        public static List<Rat2Hybrid> ReadPoints(string path)
        {
            var points = new List<Rat2Hybrid>();
            using (StreamReader reader = new StreamReader(path))
            {
                string header = reader.ReadLine();
                if (header != "POINTS")
                    throw new InvalidDataException("Invalid file format: expected POINTS header");

                int count = int.Parse(reader.ReadLine());
                for (int i = 0; i < count; i++)
                {
                    string line = reader.ReadLine();
                    string[] parts = line.Split(' ');
                    double x = double.Parse(parts[0], CultureInfo.InvariantCulture);
                    double y = double.Parse(parts[1], CultureInfo.InvariantCulture);
                    points.Add(new Rat2Hybrid(new BigRationalHybrid((long)(x * 1000000)), new BigRationalHybrid((long)(y * 1000000))));
                }
            }
            return points;
        }

        /// <summary>
        /// Reads triangles from a text file
        /// </summary>
        public static List<Tri> ReadTriangles(string path)
        {
            var triangles = new List<Tri>();
            using (StreamReader reader = new StreamReader(path))
            {
                string header = reader.ReadLine();
                if (header != "TRIANGLES")
                    throw new InvalidDataException("Invalid file format: expected TRIANGLES header");

                int count = int.Parse(reader.ReadLine());
                for (int i = 0; i < count; i++)
                {
                    string line = reader.ReadLine();
                    string[] parts = line.Split(' ');
                    int a = int.Parse(parts[0]);
                    int b = int.Parse(parts[1]);
                    int c = int.Parse(parts[2]);
                    triangles.Add(new Tri(a, b, c));
                }
            }
            return triangles;
        }

        /// <summary>
        /// Reads segments from a text file
        /// </summary>
        public static List<Int2> ReadSegments(string path)
        {
            var segments = new List<Int2>();
            using (StreamReader reader = new StreamReader(path))
            {
                string header = reader.ReadLine();
                if (header != "SEGMENTS")
                    throw new InvalidDataException("Invalid file format: expected SEGMENTS header");

                int count = int.Parse(reader.ReadLine());
                for (int i = 0; i < count; i++)
                {
                    string line = reader.ReadLine();
                    string[] parts = line.Split(' ');
                    int x = int.Parse(parts[0]);
                    int y = int.Parse(parts[1]);
                    segments.Add(new Int2(x, y));
                }
            }
            return segments;
        }

       

        /// <summary>
        /// Reads a complete 2D debug data set from files
        /// </summary>
        public static (List<Rat2Hybrid> points, List<Tri> triangles, List<Int2> segments) ReadDebugData(string basePath)
        {
            var points = ReadPoints(basePath + ".pts");
            var triangles = ReadTriangles(basePath + ".tri");
            var segments = ReadSegments(basePath + ".edge");
            return (points, triangles, segments);
        }
    }
}
