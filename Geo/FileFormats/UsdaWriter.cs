using System.Globalization;
using System.Text;
using GeoCore;

namespace Geo
{
    /// <summary>
    /// USD ASCII (.usda) triangle-mesh writer. Vertices are unique (position, normal, UV)
    /// tuples from <see cref="MeshNormalUV.Decompose"/>; surface patches become GeomSubsets.
    /// </summary>
    public static class UsdaWriter
    {
        private const string NumberFormat = "0.0################";
        private static readonly CultureInfo Ci = CultureInfo.InvariantCulture;

        public static void Write(
            string fileName,
            string meshName,
            List<Vec3D> points,
            List<Vec3D> normals,
            List<Vec2D> uv,
            List<Tri> triangles,
            List<int> groupPerTriangle,
            Dictionary<int, string> groupToName)
        {
            if (points == null)
                throw new ArgumentNullException(nameof(points));
            if (triangles == null)
                throw new ArgumentNullException(nameof(triangles));

            string primName = MakeIdentifier(string.IsNullOrEmpty(meshName) ? "Mesh" : meshName, "Mesh");
            bool hasNormals = normals != null && normals.Count == points.Count && points.Count > 0;
            bool hasUv = uv != null && uv.Count == points.Count && points.Count > 0;

            Vec3D extentMin = new Vec3D(double.MaxValue, double.MaxValue, double.MaxValue);
            Vec3D extentMax = new Vec3D(double.MinValue, double.MinValue, double.MinValue);
            for (int i = 0; i < points.Count; i++)
            {
                Vec3D p = points[i];
                if (p.X < extentMin.X) extentMin.X = p.X;
                if (p.Y < extentMin.Y) extentMin.Y = p.Y;
                if (p.Z < extentMin.Z) extentMin.Z = p.Z;
                if (p.X > extentMax.X) extentMax.X = p.X;
                if (p.Y > extentMax.Y) extentMax.Y = p.Y;
                if (p.Z > extentMax.Z) extentMax.Z = p.Z;
            }
            if (points.Count == 0)
            {
                extentMin = new Vec3D(0, 0, 0);
                extentMax = new Vec3D(0, 0, 0);
            }

            Dictionary<int, List<int>> facesPerGroup = GroupFaceIndices(triangles, groupPerTriangle);

            using (var w = new StreamWriter(fileName, false, new UTF8Encoding(false)))
            {
                w.WriteLine("#usda 1.0");
                w.WriteLine("(");
                w.WriteLine("    defaultPrim = \"" + primName + "\"");
                w.WriteLine("    metersPerUnit = 1");
                w.WriteLine("    upAxis = \"Z\"");
                w.WriteLine(")");
                w.WriteLine();
                w.WriteLine("def Mesh \"" + primName + "\"");
                w.WriteLine("{");
                w.WriteLine("    uniform token subdivisionScheme = \"none\"");
                if (facesPerGroup.Count > 0)
                    w.WriteLine("    uniform token subsetFamily:materialBind:familyType = \"nonOverlapping\"");
                w.Write("    float3[] extent = [");
                w.Write(Vec3(extentMin));
                w.Write(", ");
                w.Write(Vec3(extentMax));
                w.WriteLine("]");

                WriteIntArray(w, "int[] faceVertexCounts", triangles.Count, i => 3);
                WriteIntArray(w, "int[] faceVertexIndices", triangles.Count * 3, i =>
                {
                    Tri t = triangles[i / 3];
                    int k = i % 3;
                    return k == 0 ? t.A : (k == 1 ? t.B : t.C);
                });
                WriteVec3Array(w, "point3f[] points", points, null);

                if (hasNormals)
                    WriteVec3Array(w, "normal3f[] normals", normals, "interpolation = \"vertex\"");

                if (hasUv)
                    WriteVec2Array(w, "texCoord2f[] primvars:st", uv, "interpolation = \"vertex\"");

                var usedSubsetNames = new HashSet<string>(StringComparer.Ordinal) { primName };
                foreach (var kv in facesPerGroup.OrderBy(g => g.Key))
                {
                    string rawName;
                    if (groupToName != null && groupToName.TryGetValue(kv.Key, out rawName) && !string.IsNullOrEmpty(rawName))
                    { }
                    else
                        rawName = "group_" + kv.Key.ToString(Ci);

                    string subsetName = UniqueIdentifier(rawName, "subset", usedSubsetNames);
                    w.WriteLine();
                    w.WriteLine("    def GeomSubset \"" + subsetName + "\"");
                    w.WriteLine("    {");
                    w.WriteLine("        uniform token elementType = \"face\"");
                    w.WriteLine("        uniform token familyName = \"materialBind\"");
                    WriteIntArray(w, "int[] indices", kv.Value.Count, i => kv.Value[i], indent: 8);
                    w.WriteLine("    }");
                }

                w.WriteLine("}");
            }
        }

        private static Dictionary<int, List<int>> GroupFaceIndices(List<Tri> triangles, List<int> groupPerTriangle)
        {
            var facesPerGroup = new Dictionary<int, List<int>>();
            for (int i = 0; i < triangles.Count; i++)
            {
                int groupId = (groupPerTriangle != null && i < groupPerTriangle.Count) ? groupPerTriangle[i] : 0;
                List<int> faces;
                if (!facesPerGroup.TryGetValue(groupId, out faces))
                {
                    faces = new List<int>();
                    facesPerGroup[groupId] = faces;
                }
                faces.Add(i);
            }
            return facesPerGroup;
        }

        private static void WriteVec3Array(StreamWriter w, string decl, List<Vec3D> values, string metadata)
        {
            w.Write("    ");
            w.Write(decl);
            w.WriteLine(" = [");
            for (int i = 0; i < values.Count; i++)
            {
                w.Write("        ");
                w.Write(Vec3(values[i]));
                if (i + 1 < values.Count)
                    w.Write(',');
                w.WriteLine();
            }
            WriteArrayClose(w, "    ", metadata);
        }

        private static void WriteVec2Array(StreamWriter w, string decl, List<Vec2D> values, string metadata)
        {
            w.Write("    ");
            w.Write(decl);
            w.WriteLine(" = [");
            for (int i = 0; i < values.Count; i++)
            {
                Vec2D t = values[i];
                w.Write("        (");
                w.Write(F(t.X));
                w.Write(", ");
                w.Write(F(t.Y));
                w.Write(')');
                if (i + 1 < values.Count)
                    w.Write(',');
                w.WriteLine();
            }
            WriteArrayClose(w, "    ", metadata);
        }

        private static void WriteArrayClose(StreamWriter w, string pad, string metadata)
        {
            if (string.IsNullOrEmpty(metadata))
            {
                w.Write(pad);
                w.WriteLine("]");
                return;
            }
            w.Write(pad);
            w.WriteLine("] (");
            w.Write(pad);
            w.Write("    ");
            w.WriteLine(metadata);
            w.Write(pad);
            w.WriteLine(")");
        }

        private static void WriteIntArray(StreamWriter w, string decl, int count, Func<int, int> at, int indent = 4)
        {
            string pad = new string(' ', indent);
            w.Write(pad);
            w.Write(decl);
            if (count == 0)
            {
                w.WriteLine(" = []");
                return;
            }
            w.WriteLine(" = [");
            const int perLine = 12;
            for (int i = 0; i < count; i++)
            {
                if (i % perLine == 0)
                    w.Write(pad + "    ");
                w.Write(at(i).ToString(Ci));
                if (i + 1 < count)
                    w.Write(',');
                if (i + 1 == count || (i + 1) % perLine == 0)
                    w.WriteLine();
                else
                    w.Write(' ');
            }
            w.Write(pad);
            w.WriteLine("]");
        }

        private static string Vec3(Vec3D v)
        {
            return "(" + F(v.X) + ", " + F(v.Y) + ", " + F(v.Z) + ")";
        }

        private static string F(double v)
        {
            if (double.IsNaN(v) || double.IsInfinity(v))
                return "0.0";
            return v.ToString(NumberFormat, Ci);
        }

        internal static string MakeIdentifier(string name, string fallback)
        {
            if (string.IsNullOrEmpty(name))
                name = fallback;
            var sb = new StringBuilder(name.Length);
            for (int i = 0; i < name.Length; i++)
            {
                char c = name[i];
                if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_' || (c >= '0' && c <= '9'))
                    sb.Append(c);
                else if (sb.Length > 0 && sb[sb.Length - 1] != '_')
                    sb.Append('_');
            }
            if (sb.Length == 0)
                return fallback;
            if (sb[0] >= '0' && sb[0] <= '9')
                sb.Insert(0, '_');
            return sb.ToString();
        }

        static string UniqueIdentifier(string name, string fallback, HashSet<string> used)
        {
            string id = MakeIdentifier(name, fallback);
            if (used.Add(id))
                return id;
            int n = 2;
            while (!used.Add(id + "_" + n.ToString(Ci)))
                n++;
            return id + "_" + n.ToString(Ci);
        }
    }
}
