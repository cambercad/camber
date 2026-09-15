using GeoCore;

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

            List<float> values = new List<float>();

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


        private static int AddIfNotPresent(this Dictionary<IndexTuple, int> map, int pIndex,
            List<Vec3D> pos, List<Vec3D> posInputBuffer, int smoothingGroup = 0)
        {
            IndexTuple triple;
            triple.A = pIndex;
            triple.B = 0;
            triple.C = 0;
            triple.SmoothingGroup = smoothingGroup;
            int id;
            if (map.TryGetValue(triple, out id))
                return id;

            map.Add(triple, pos.Count);
            pos.Add(posInputBuffer[pIndex]);
            return pos.Count - 1;
        }
        private static int AddIfNotPresent(this Dictionary<IndexTuple, int> map, int pIndex, List<Vec3D> pos, List<Vec3D> posInputBuffer,
            int nIndex, List<Vec3D> nor, List<Vec3D> norInputBuffer, int smoothingGroup = 0)
        {
            IndexTuple triple;
            triple.A = pIndex;
            triple.B = nIndex;
            triple.C = 0;
            triple.SmoothingGroup = smoothingGroup;
            int id;
            if (map.TryGetValue(triple, out id))
                return id;

            map.Add(triple, pos.Count);
            pos.Add(posInputBuffer[pIndex]);
            nor.Add(norInputBuffer[nIndex]);
            return pos.Count - 1;
        }
        private static int AddIfNotPresent(this Dictionary<IndexTuple, int> map, int pIndex, List<Vec3D> pos, List<Vec3D> posInputBuffer,
            int nIndex, List<Vec3D> nor, List<Vec3D> norInputBuffer, int tIndex, List<Vec2D> tex, List<Vec2D> texInputBuffer, int smoothingGroup = 0)
        {
            IndexTuple triple;
            triple.A = pIndex;
            triple.B = nIndex;
            triple.C = tIndex;
            triple.SmoothingGroup = smoothingGroup;
            int id;
            if (map.TryGetValue(triple, out id))
                return id;

            map.Add(triple, pos.Count);
            pos.Add(posInputBuffer[pIndex]);
            nor.Add(norInputBuffer[nIndex]);
            tex.Add(texInputBuffer[tIndex]);
            return pos.Count - 1;
        }

        private static void ReadFloatNumbers(string line, int startCharIndex, List<float> buffer)
        {
            //MatchCollection m = Regex.Matches(line, @"([\+\-])?\d+(\.\d*)?([Ee][+-]?\d+)?"/*@"(-)?\d+(\.\d+)?(E([+-])?\d+)?"*/, RegexOptions.IgnoreCase);
            //float[] values = new float[m.Count];
            //for (int i = 0; i < values.Length; ++i)
            //    values[i] = Convert.ToSingle(m[i].Value);
            //return values;

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
                        float f;
                        if (float.TryParse(n, out f))
                            buffer.Add(f);
                        //buffer.Add(NumberParser.ParseFloat(line, start));
                    }

                    start = i + 1;
                }
            }
        }

        private static void ReadIntNumbers(string line, int startCharIndex, List<int> buffer)
        {
            //MatchCollection m = Regex.Matches(line, @"(-)?\d+");
            //int[] values = new int[m.Count];
            //for (int i = 0; i < values.Length; ++i)
            //    values[i] = Convert.ToInt32(m[i].Value);
            //return values;

            buffer.Clear();
            line = line.Trim();
            line = line + " ";
            int l = line.Length;
            int start = startCharIndex;
            for (int i = start; i < l; ++i)
            {
                char c = line[i];
                if (c == ' ' || c == '/' || c == '\t')
                {
                    int count = i - start;
                    if (count > 0)
                    {
                        string n = line.Substring(start, count);
                        int f;
                        if (int.TryParse(n, out f))
                            buffer.Add(f);
                        //buffer.Add(NumberParser.ParseInt(line, start));
                    }

                    start = i + 1;
                }
            }
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
            List<float> values = new List<float>();
            List<int> indices = new List<int>();
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
                    PNTGeometry tg = new PNTGeometry(line.Substring(2).Trim(), triangles, pos, nor, tex);

                    current = tg;

                    geometry.Add(tg);
                }
                else if (line.StartsWith("s "))
                {
                    string s = line.Substring(2);
                    if (s == "off")
                        smoothingGroup = -1;
                    else
                        smoothingGroup = Convert.ToInt32(s);
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
                        PNTGeometry tg = new PNTGeometry(triangles, pos, nor, tex);

                        current = tg;

                        geometry.Add(tg);
                    }

                    ReadIntNumbers(line, 2, indices);
                    if (indices.Count == 3)
                    {
                        triangles.Add(new Tri(
                            map.AddIfNotPresent(indices[0] - 1, pos, vBuffer, smoothingGroup),
                            map.AddIfNotPresent(indices[2] - 1, pos, vBuffer, smoothingGroup),
                            map.AddIfNotPresent(indices[1] - 1, pos, vBuffer, smoothingGroup)));
                    }
                    else if (indices.Count == 9)
                    {
                        triangles.Add(new Tri(
                            map.AddIfNotPresent(indices[0] - 1, pos, vBuffer, indices[2] - 1, nor, vnBuffer, indices[1] - 1, tex, vtBuffer, smoothingGroup),
                            map.AddIfNotPresent(indices[6] - 1, pos, vBuffer, indices[8] - 1, nor, vnBuffer, indices[7] - 1, tex, vtBuffer, smoothingGroup),
                            map.AddIfNotPresent(indices[3] - 1, pos, vBuffer, indices[5] - 1, nor, vnBuffer, indices[4] - 1, tex, vtBuffer, smoothingGroup)));
                    }
                    else if (indices.Count == 6)
                    {
                        triangles.Add(new Tri(
                            map.AddIfNotPresent(indices[0] - 1, pos, vBuffer, indices[1] - 1, nor, vnBuffer, smoothingGroup),
                            map.AddIfNotPresent(indices[4] - 1, pos, vBuffer, indices[5] - 1, nor, vnBuffer, smoothingGroup),
                            map.AddIfNotPresent(indices[2] - 1, pos, vBuffer, indices[3] - 1, nor, vnBuffer, smoothingGroup)));
                    }
                    else if (indices.Count == 12)
                    {
                        //Quads
                        int a = map.AddIfNotPresent(indices[0] - 1, pos, vBuffer, indices[2] - 1, nor, vnBuffer, indices[1] - 1, tex, vtBuffer, smoothingGroup);
                        int b = map.AddIfNotPresent(indices[3] - 1, pos, vBuffer, indices[5] - 1, nor, vnBuffer, indices[4] - 1, tex, vtBuffer, smoothingGroup);
                        int c = map.AddIfNotPresent(indices[6] - 1, pos, vBuffer, indices[8] - 1, nor, vnBuffer, indices[7] - 1, tex, vtBuffer, smoothingGroup);
                        int d = map.AddIfNotPresent(indices[9] - 1, pos, vBuffer, indices[11] - 1, nor, vnBuffer, indices[10] - 1, tex, vtBuffer, smoothingGroup);

                        triangles.Add(new Tri(a, c, b));
                        triangles.Add(new Tri(a, d, c));
                    }
                    else
                    {
                        //throw new Exception();
                    }
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
                    }

                    current.MaterialName = line.Substring(7).Trim();
                }
            }
            sr.Close();

            return geometry;
        }
    }
}
