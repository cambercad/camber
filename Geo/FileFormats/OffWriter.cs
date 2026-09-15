
using GeoCore;

namespace GeoScriptViewer
{
    public class OffWriter
    {
        public static void Write(string fileName, IList<Vec3D> points, IList<Tri> triangles, string format = "0.######")
        {
            StreamWriter sw = new StreamWriter(fileName);

            sw.WriteLine("OFF");
            sw.WriteLine(points.Count.ToString() + " " + triangles.Count.ToString() + " 0");

            for (int i = 0; i < points.Count; ++i)
            {
                Vec3D p = points[i];
                sw.WriteLine(p.X.ToString(format) + " " + p.Y.ToString(format) + " " + p.Z.ToString(format));
            }

            for (int i = 0; i < triangles.Count; ++i)
            {
                Tri t = triangles[i];
                sw.WriteLine("3 " + t.A.ToString(format) + " " + t.B.ToString(format) + " " + t.C.ToString(format));
            }

            sw.Close();
        }

        internal static void Write(string fileName, double[] points, uint[] triangles)
        {
            List<Vec3D> points2 = new List<Vec3D>();
            for (int i = 0; i < points.Length / 3; ++i)
                points2.Add(new Vec3D(points[3 * i + 0], points[3 * i + 1], points[3 * i + 2]));
            List<Tri> triangles2 = new List<Tri>();
            for (int i = 0; i < triangles.Length / 3; ++i)
                triangles2.Add(new Tri((int)triangles[3 * i + 0], (int)triangles[3 * i + 1], (int)triangles[3 * i + 2]));
            Write(fileName, points2, triangles2);
        }
    }
}
