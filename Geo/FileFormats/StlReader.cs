using GeoCore;
using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;

namespace Geo
{
    public class StlReader
    {
        private static readonly Regex _whiteSpace = new Regex(@"\s+");

        /// <summary>
        /// Reads an STL file (supports both ASCII and binary formats)
        /// </summary>
        /// <param name="fileName">Path to the STL file</param>
        /// <param name="points">Output list of vertices</param>
        /// <param name="triangles">Output list of triangles</param>
        public static void Read(string fileName, out List<Vec3D> points, out List<Tri> triangles)
        {
            if (IsBinaryStl(fileName))
            {
                ReadBinary(fileName, out points, out triangles);
            }
            else
            {
                ReadAscii(fileName, out points, out triangles);
            }
        }

        /// <summary>
        /// Determines if an STL file is binary or ASCII format
        /// </summary>
        private static bool IsBinaryStl(string fileName)
        {
            using (var fs = new FileStream(fileName, FileMode.Open, FileAccess.Read))
            {
                if (fs.Length < 80) return false;

                // Read first 80 bytes (header)
                byte[] header = new byte[80];
                fs.Read(header, 0, 80);

                // Check if header starts with "solid " (ASCII STL marker)
                string headerText = Encoding.ASCII.GetString(header).TrimStart();
                if (headerText.StartsWith("solid "))
                {
                    // Could still be binary - check if file size matches expected binary size
                    if (fs.Length >= 84) // At least header + triangle count
                    {
                        fs.Seek(80, SeekOrigin.Begin);
                        byte[] countBytes = new byte[4];
                        fs.Read(countBytes, 0, 4);
                        uint triangleCount = BitConverter.ToUInt32(countBytes, 0);

                        // Binary STL size = 80 (header) + 4 (count) + triangleCount * 50
                        long expectedSize = 84 + triangleCount * 50;
                        if (fs.Length == expectedSize)
                        {
                            return true; // Binary format
                        }
                    }
                    return false; // ASCII format
                }
                return true; // Binary format (doesn't start with "solid ")
            }
        }

        /// <summary>
        /// Reads ASCII STL format
        /// </summary>
        private static void ReadAscii(string fileName, out List<Vec3D> points, out List<Tri> triangles)
        {
            points = new List<Vec3D>();
            triangles = new List<Tri>();
            var pointDict = new Dictionary<Vec3D, int>();

            using (var sr = new StreamReader(fileName))
            {
                string line;
                while ((line = sr.ReadLine()) != null)
                {
                    line = line.Trim();
                    if (line.StartsWith("facet"))
                    {
                        // Read the three vertices of this facet
                        Vec3D[] vertices = new Vec3D[3];
                        
                        // Skip "outer loop" line
                        sr.ReadLine();
                        
                        for (int i = 0; i < 3; i++)
                        {
                            string vertexLine = sr.ReadLine()?.Trim();
                            if (vertexLine != null && vertexLine.StartsWith("vertex"))
                            {
                                string[] parts = _whiteSpace.Split(vertexLine);
                                if (parts.Length >= 4)
                                {
                                    double x = Convert.ToDouble(parts[1], CultureInfo.InvariantCulture);
                                    double y = Convert.ToDouble(parts[2], CultureInfo.InvariantCulture);
                                    double z = Convert.ToDouble(parts[3], CultureInfo.InvariantCulture);
                                    vertices[i] = new Vec3D(x, y, z);
                                }
                            }
                        }
                        
                        // Skip "endloop" and "endfacet" lines
                        sr.ReadLine();
                        sr.ReadLine();

                        // Add vertices to points list and create triangle
                        int[] indices = new int[3];
                        for (int i = 0; i < 3; i++)
                        {
                            if (pointDict.TryGetValue(vertices[i], out int existingIndex))
                            {
                                indices[i] = existingIndex;
                            }
                            else
                            {
                                indices[i] = points.Count;
                                pointDict[vertices[i]] = points.Count;
                                points.Add(vertices[i]);
                            }
                        }
                        
                        triangles.Add(new Tri(indices[0], indices[1], indices[2]));
                    }
                }
            }
        }

        /// <summary>
        /// Reads binary STL format
        /// </summary>
        private static void ReadBinary(string fileName, out List<Vec3D> points, out List<Tri> triangles)
        {
            points = new List<Vec3D>();
            triangles = new List<Tri>();
            var pointDict = new Dictionary<Vec3D, int>();

            using (var fs = new FileStream(fileName, FileMode.Open, FileAccess.Read))
            using (var br = new BinaryReader(fs))
            {
                // Skip 80-byte header
                br.ReadBytes(80);
                
                // Read number of triangles
                uint triangleCount = br.ReadUInt32();
                
                for (uint i = 0; i < triangleCount; i++)
                {
                    // Skip normal vector (3 floats)
                    br.ReadSingle();
                    br.ReadSingle();
                    br.ReadSingle();
                    
                    // Read three vertices
                    Vec3D[] vertices = new Vec3D[3];
                    for (int j = 0; j < 3; j++)
                    {
                        float x = br.ReadSingle();
                        float y = br.ReadSingle();
                        float z = br.ReadSingle();
                        vertices[j] = new Vec3D(x, y, z);
                    }
                    
                    // Skip attribute byte count
                    br.ReadUInt16();
                    
                    // Add vertices to points list and create triangle
                    int[] indices = new int[3];
                    for (int j = 0; j < 3; j++)
                    {
                        if (pointDict.TryGetValue(vertices[j], out int existingIndex))
                        {
                            indices[j] = existingIndex;
                        }
                        else
                        {
                            indices[j] = points.Count;
                            pointDict[vertices[j]] = points.Count;
                            points.Add(vertices[j]);
                        }
                    }
                    
                    triangles.Add(new Tri(indices[0], indices[1], indices[2]));
                }
            }
        }
    }

    public class StlWriter
    {
        /// <summary>
        /// Writes an STL file in ASCII format
        /// </summary>
        /// <param name="fileName">Output file path</param>
        /// <param name="points">List of vertices</param>
        /// <param name="triangles">List of triangles</param>
        /// <param name="solidName">Name for the solid (optional)</param>
        /// <param name="format">Number format for coordinates</param>
        public static void WriteAscii(string fileName, IList<Vec3D> points, IList<Tri> triangles, string solidName = "object", string format = "0.######")
        {
            using (var sw = new StreamWriter(fileName))
            {
                sw.WriteLine($"solid {solidName}");
                
                for (int i = 0; i < triangles.Count; i++)
                {
                    Tri triangle = triangles[i];
                    Vec3D p1 = points[triangle.A];
                    Vec3D p2 = points[triangle.B];
                    Vec3D p3 = points[triangle.C];
                    
                    // Calculate normal vector
                    Vec3D v1 = p2 - p1;
                    Vec3D v2 = p3 - p1;
                    Vec3D normal = v1.Cross(v2).Normalized();
                    
                    sw.WriteLine($"  facet normal {normal.X.ToString(format, CultureInfo.InvariantCulture)} {normal.Y.ToString(format, CultureInfo.InvariantCulture)} {normal.Z.ToString(format, CultureInfo.InvariantCulture)}");
                    sw.WriteLine("    outer loop");
                    sw.WriteLine($"      vertex {p1.X.ToString(format, CultureInfo.InvariantCulture)} {p1.Y.ToString(format, CultureInfo.InvariantCulture)} {p1.Z.ToString(format, CultureInfo.InvariantCulture)}");
                    sw.WriteLine($"      vertex {p2.X.ToString(format, CultureInfo.InvariantCulture)} {p2.Y.ToString(format, CultureInfo.InvariantCulture)} {p2.Z.ToString(format, CultureInfo.InvariantCulture)}");
                    sw.WriteLine($"      vertex {p3.X.ToString(format, CultureInfo.InvariantCulture)} {p3.Y.ToString(format, CultureInfo.InvariantCulture)} {p3.Z.ToString(format, CultureInfo.InvariantCulture)}");
                    sw.WriteLine("    endloop");
                    sw.WriteLine("  endfacet");
                }
                
                sw.WriteLine($"endsolid {solidName}");
            }
        }

        /// <summary>
        /// Writes an STL file in binary format
        /// </summary>
        /// <param name="fileName">Output file path</param>
        /// <param name="points">List of vertices</param>
        /// <param name="triangles">List of triangles</param>
        /// <param name="header">80-byte header string (optional)</param>
        public static void WriteBinary(string fileName, IList<Vec3D> points, IList<Tri> triangles, string header = "Binary STL file")
        {
            using (var fs = new FileStream(fileName, FileMode.Create, FileAccess.Write))
            using (var bw = new BinaryWriter(fs))
            {
                // Write 80-byte header
                byte[] headerBytes = new byte[80];
                byte[] headerStringBytes = Encoding.ASCII.GetBytes(header);
                Array.Copy(headerStringBytes, headerBytes, Math.Min(headerStringBytes.Length, 80));
                bw.Write(headerBytes);
                
                // Write triangle count
                bw.Write((uint)triangles.Count);
                
                for (int i = 0; i < triangles.Count; i++)
                {
                    Tri triangle = triangles[i];
                    Vec3D p1 = points[triangle.A];
                    Vec3D p2 = points[triangle.B];
                    Vec3D p3 = points[triangle.C];
                    
                    // Calculate normal vector
                    Vec3D v1 = p2 - p1;
                    Vec3D v2 = p3 - p1;
                    Vec3D normal = v1.Cross(v2).Normalized();
                    
                    // Write normal vector
                    bw.Write((float)normal.X);
                    bw.Write((float)normal.Y);
                    bw.Write((float)normal.Z);
                    
                    // Write vertices
                    bw.Write((float)p1.X);
                    bw.Write((float)p1.Y);
                    bw.Write((float)p1.Z);
                    
                    bw.Write((float)p2.X);
                    bw.Write((float)p2.Y);
                    bw.Write((float)p2.Z);
                    
                    bw.Write((float)p3.X);
                    bw.Write((float)p3.Y);
                    bw.Write((float)p3.Z);
                    
                    // Write attribute byte count (usually 0)
                    bw.Write((ushort)0);
                }
            }
        }

        /// <summary>
        /// Writes an STL file (defaults to ASCII format)
        /// </summary>
        /// <param name="fileName">Output file path</param>
        /// <param name="points">List of vertices</param>
        /// <param name="triangles">List of triangles</param>
        /// <param name="format">Number format for coordinates (ASCII only)</param>
        public static void Write(string fileName, IList<Vec3D> points, IList<Tri> triangles, string format = "0.######")
        {
            WriteAscii(fileName, points, triangles, "object", format);
        }

        /// <summary>
        /// Convenience method for writing with arrays (similar to OffWriter)
        /// </summary>
        /// <param name="fileName">Output file path</param>
        /// <param name="points">Point coordinates as flat array (x1,y1,z1,x2,y2,z2,...)</param>
        /// <param name="triangles">Triangle indices as flat array (i1,i2,i3,i4,i5,i6,...)</param>
        /// <param name="binary">Whether to write in binary format</param>
        //public static void Write(string fileName, double[] points, uint[] triangles, bool binary = false)
        //{
        //    List<Vec3D> pointsList = new List<Vec3D>();
        //    for (int i = 0; i < points.Length / 3; ++i)
        //        pointsList.Add(new Vec3D(points[3 * i + 0], points[3 * i + 1], points[3 * i + 2]));
            
        //    List<Tri> trianglesList = new List<Tri>();
        //    for (int i = 0; i < triangles.Length / 3; ++i)
        //        trianglesList.Add(new Tri((int)triangles[3 * i + 0], (int)triangles[3 * i + 1], (int)triangles[3 * i + 2]));
            
        //    if (binary)
        //        WriteBinary(fileName, pointsList, trianglesList);
        //    else
        //        WriteAscii(fileName, pointsList, trianglesList);
        //}
    }
}
