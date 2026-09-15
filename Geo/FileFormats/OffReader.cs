using GeoCore;
using System.Text.RegularExpressions;

namespace Geo
{
    public class OffReader
    {
        private static Regex _whiteSpace = new Regex(@"\s+");


        public static void Read(string fileName, out List<Vec3D> points, out List<Tri> triangles)
        {
            StreamReader sr = new StreamReader(fileName);
            sr.ReadLine();
            string[] parts = _whiteSpace.Split(sr.ReadLine());

            int numPoints = Convert.ToInt32(parts[0]);
            int numTris = Convert.ToInt32(parts[1]);

            points = new List<Vec3D>(numPoints);
            for (int i = 0; i < numPoints; ++i)
            {
                parts = _whiteSpace.Split(sr.ReadLine());
                double x = Convert.ToDouble(parts[0]);
                double y = Convert.ToDouble(parts[1]);
                double z = Convert.ToDouble(parts[2]);
                points.Add(new Vec3D(x, y, z));
            }

            triangles = new List<Tri>(numTris);
            for (int i = 0; i < numTris; ++i)
            {
                parts = _whiteSpace.Split(sr.ReadLine());
                int a = Convert.ToInt32(parts[1]);
                int b = Convert.ToInt32(parts[2]);
                int c = Convert.ToInt32(parts[3]);
                triangles.Add(new Tri(a, b, c));
            }
            sr.Close();
        }
    }
}
