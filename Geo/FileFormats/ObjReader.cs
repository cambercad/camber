using GeoCore;

using System.Globalization;

namespace Geo
{
    public struct IndexTuple
    {
        public int A;
        public int B;
        public int C;
        public int SmoothingGroup;

        public override int GetHashCode()
        {
            return HashCode.Combine(A,B,C,SmoothingGroup);
        }

        public override bool Equals(object obj)
        {
            IndexTuple other = (IndexTuple)obj;
            return A == other.A && B == other.B && C == other.C && SmoothingGroup == other.SmoothingGroup;
        }
    }


    public class PNTGeometry
    {
        public string Name = null;
        public string MaterialName = null;
        public List<Tri> Triangles;
        public List<Vec3D> Position;
        public List<Vec3D> Normal;
        public List<Vec2D> Texture;

        public PNTGeometry(List<Tri> triangles, List<Vec3D> position, List<Vec3D> normal, List<Vec2D> texture)
        {
            Triangles = triangles;
            Position = position;
            Normal = normal;
            Texture = texture;
        }

        public PNTGeometry(string name, List<Tri> triangles, List<Vec3D> position, List<Vec3D> normal, List<Vec2D> texture)
        {
            Name = name;
            Triangles = triangles;
            Position = position;
            Normal = normal;
            Texture = texture;
        }

        public void ApplyScaling(float scaling)
        {
            for (int i = 0; i < Position.Count; ++i)
                Position[i] *= scaling;
        }
    }

    public static class ObjReader
    {
        public static Dictionary<string, ObjMaterial> LoadMaterials(string path)
        {
            path = path.Trim();
            if (path.ToLower().EndsWith(".obj"))
            {
                path = path.Substring(0, path.Length - 4);
                path += ".mtl";
            }
            Dictionary<string, ObjMaterial> materials = new Dictionary<string, ObjMaterial>();

            var dir = Path.GetDirectoryName(path);

            List<double> values = new List<double>();

            ObjMaterial currentMaterial = null;
            StreamReader sr = new StreamReader(path);
            while (!sr.EndOfStream)
            {
                string line = sr.ReadLine().Trim();

                if (line.StartsWith("newmtl "))
                {
                    currentMaterial = new ObjMaterial();
                    currentMaterial.Name = line.Substring(7).Trim();
                    materials.Add(currentMaterial.Name, currentMaterial);
                }
                else if (line.StartsWith("Ns "))
                {
                    currentMaterial.Ns = Convert.ToSingle(line.Substring(3));
                }
                else if (line.StartsWith("Ni "))
                {
                    currentMaterial.Ni = Convert.ToSingle(line.Substring(3));
                }
                else if (line.StartsWith("d "))
                {
                    currentMaterial.d = Convert.ToSingle(line.Substring(2));
                }
                else if (line.StartsWith("Tr "))
                {
                    currentMaterial.Tr = Convert.ToSingle(line.Substring(3));
                }
                else if (line.StartsWith("Tf "))
                {
                    ReadFloatNumbers(line, 3, values);
                    currentMaterial.Tf = new Vec3D(values[0], values[1], values[2]);
                }
                else if (line.StartsWith("illum "))
                {
                    currentMaterial.illum = Convert.ToInt32(line.Substring(6));
                }
                else if (line.StartsWith("Ka "))
                {
                    ReadFloatNumbers(line, 3, values);
                    currentMaterial.Ka = new Vec3D(values[0], values[1], values[2]);
                }
                else if (line.StartsWith("Kd "))
                {
                    ReadFloatNumbers(line, 3, values);
                    currentMaterial.Kd = new Vec3D(values[0], values[1], values[2]);
                }
                else if (line.StartsWith("Ks "))
                {
                    ReadFloatNumbers(line, 3, values);
                    currentMaterial.Ks = new Vec3D(values[0], values[1], values[2]);
                }
                else if (line.StartsWith("Ke "))
                {
                    ReadFloatNumbers(line, 3, values);
                    currentMaterial.Ke = new Vec3D(values[0], values[1], values[2]);
                }
                else if (line.StartsWith("map_Ka "))
                {
                    currentMaterial.map_Ka = Path.GetFullPath(Path.Combine(dir, line.Substring(7).Trim()));
                }
                else if (line.StartsWith("map_Kd "))
                {
                    currentMaterial.map_Kd = Path.GetFullPath(Path.Combine(dir, line.Substring(7).Trim()));
                }
                else if (line.StartsWith("map_bump "))
                {
                    currentMaterial.map_bump = Path.GetFullPath(Path.Combine(dir, line.Substring(9).Trim()));
                }
                else if (line.StartsWith("bump "))
                {
                    currentMaterial.bump = Path.GetFullPath(Path.Combine(dir, line.Substring(5).Trim()));
                }
            }
            sr.Close();

            return materials;
        }


        private static void ReadFloatNumbers(string line, int startCharIndex, List<double> buffer)
        {
            buffer.Clear();
            line = line.Trim();
            line = line + " ";
            int l = line.Length;
            int start = startCharIndex;
            for (int i = start; i < l; ++i)
            {
                char c = line[i];
                if (c == ' ' || c == '\t')
                {
                    int count = i - start;
                    if (count > 0)
                    {
                        string n = line.Substring(start, count);
                        if (!double.TryParse(n, NumberStyles.Float, CultureInfo.InvariantCulture, out double f))
                            throw new FormatException("Invalid OBJ number: " + n);
                        buffer.Add(f);
                    }

                    start = i + 1;
                }
            }
        }

        private static int FaceIndex(string text, int count)
        {
            if (!int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out int value) || value == 0)
                throw new FormatException("Invalid OBJ index: " + text);
            int index = value > 0 ? value - 1 : count + value;
            if (index < 0 || index >= count)
                throw new FormatException("OBJ index out of range: " + text);
            return index;
        }

        public static List<PNTGeometry> Load(string path)
        {
            string materialLibraryPath;
            return Load(path, out materialLibraryPath);
        }

        public static List<PNTGeometry> Load(string path, out string materialLibraryPath)
        {
            List<Vec3D> vBuffer = new List<Vec3D>();
            List<Vec3D> vnBuffer = new List<Vec3D>();
            List<Vec2D> vtBuffer = new List<Vec2D>();

            Dictionary<IndexTuple, int> map = new Dictionary<IndexTuple, int>();
            List<Vec3D> pos = null;
            List<Vec3D> nor = null;
            List<Vec2D> tex = null;
            List<Tri> triangles = null;
            int smoothingGroup = -1;

            materialLibraryPath = null;

            StreamReader sr = new StreamReader(path);
            List<PNTGeometry> geometry = new List<PNTGeometry>();
            PNTGeometry current = null;
            List<double> values = new List<double>();
            string objectName = null;
            while (!sr.EndOfStream)
            {
                string line = sr.ReadLine().Trim();
                if (line.StartsWith("mtllib "))
                {
                    materialLibraryPath = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(path), line.Substring(7).Trim()));
                }
                else if (line.StartsWith("v "))
                {
                    ReadFloatNumbers(line, 2, values);
                    vBuffer.Add(new Vec3D(values[0], values[1], values[2]));
                }
                else if (line.StartsWith("vn "))
                {
                    ReadFloatNumbers(line, 3, values);
                    Vec3D n = new Vec3D(values[0], values[1], values[2]);
                    n.Normalize();
                    vnBuffer.Add(n);
                }
                else if (line.StartsWith("vt "))
                {
                    ReadFloatNumbers(line, 3, values);
                    vtBuffer.Add(new Vec2D(values[0], values[1]));
                }
                else if (line.StartsWith("g "))
                {
                    pos = new List<Vec3D>();
                    nor = new List<Vec3D>();
                    tex = new List<Vec2D>();
                    triangles = new List<Tri>();
                    string groupName = line.Substring(2).Trim();
                    PNTGeometry tg = new PNTGeometry(objectName == null ? groupName : objectName + "/" + groupName,
                        triangles, pos, nor, tex);

                    current = tg;

                    geometry.Add(tg);
                    map.Clear();
                }
                else if (line.StartsWith("o "))
                {
                    objectName = line.Substring(2).Trim();
                    current = null;
                    triangles = null;
                    map.Clear();
                }
                else if (line.StartsWith("s "))
                {
                    string s = line.Substring(2);
                    if (s == "off" || s == "0")
                        smoothingGroup = -1;
                    else if (s == "on")
                        smoothingGroup = 1;
                    else
                        smoothingGroup = int.Parse(s, NumberStyles.Integer, CultureInfo.InvariantCulture);
                }
                else if (line.StartsWith("f "))
                {
                    if (triangles == null)
                    {
                        pos = new List<Vec3D>();
                        nor = new List<Vec3D>();
                        tex = new List<Vec2D>();
                        triangles = new List<Tri>();
                        map.Clear();
                       
                    }

                    if(current == null)
                    {
                        PNTGeometry tg = new PNTGeometry(objectName, triangles, pos, nor, tex);

                        current = tg;

                        geometry.Add(tg);
                    }

                    string[] corners = line.Substring(2).Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (corners.Length < 3) throw new FormatException("OBJ face needs at least three vertices.");
                    int[] face = new int[corners.Length];
                    for (int i = 0; i < corners.Length; i++)
                    {
                        string[] fields = corners[i].Split('/');
                        if (fields.Length < 1 || fields.Length > 3)
                            throw new FormatException("Invalid OBJ face corner: " + corners[i]);
                        int p = FaceIndex(fields[0], vBuffer.Count);
                        int t = fields.Length > 1 && fields[1].Length > 0 ? FaceIndex(fields[1], vtBuffer.Count) : -1;
                        int n = fields.Length > 2 && fields[2].Length > 0 ? FaceIndex(fields[2], vnBuffer.Count) : -1;
                        var key = new IndexTuple { A = p, B = n, C = t, SmoothingGroup = smoothingGroup };
                        if (!map.TryGetValue(key, out int vertex))
                        {
                            vertex = pos.Count;
                            map.Add(key, vertex);
                            pos.Add(vBuffer[p]);
                            if (n >= 0) nor.Add(vnBuffer[n]);
                            if (t >= 0) tex.Add(vtBuffer[t]);
                        }
                        face[i] = vertex;
                    }
                    for (int i = 1; i + 1 < face.Length; i++)
                        triangles.Add(new Tri(face[0], face[i], face[i + 1]));
                }
                else if (line.StartsWith("usemtl "))
                {
                    if (triangles == null)
                    {
                        pos = new List<Vec3D>();
                        nor = new List<Vec3D>();
                        tex = new List<Vec2D>();
                        triangles = new List<Tri>();
                        map.Clear();

                    }
                    if (current == null)
                    {
                        PNTGeometry tg = new PNTGeometry(triangles, pos, nor, tex);

                        current = tg;

                        geometry.Add(tg);
                    }

                    if (current.MaterialName != null)
                    {
                        pos = new List<Vec3D>();
                        nor = new List<Vec3D>();
                        tex = new List<Vec2D>();
                        triangles = new List<Tri>();
                        PNTGeometry tg = new PNTGeometry(current.Name, triangles, pos, nor, tex);

                        current = tg;

                        geometry.Add(tg);
                        map.Clear();
                    }

                    current.MaterialName = line.Substring(7).Trim();
                }
            }
            sr.Close();

            return geometry;
        }
    }
}
