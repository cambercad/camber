using System.Globalization;
using GeoCore;

namespace Geo
{
    public static class ObjWriter
    {
        public static void Write(string fileName, List<Vec3D> points, List<Vec3D> normals, List<Vec2D> uv,
            List<Tri> triangles, List<int> groupPerTriangle, Dictionary<int, string> groupToName)
        {
            using var writer = new StreamWriter(fileName);
            WriteMesh(writer, points, normals, uv, triangles, groupPerTriangle, groupToName, 1);
        }

        public static void WriteAssembly(string fileName, IReadOnlyList<AssemblyLeaf> leaves)
        {
            using var writer = new StreamWriter(fileName);
            int offset = 1;
            foreach (var leaf in leaves)
            {
                var mesh = leaf.Snapshot();
                mesh.EnsureCoplanarPostProcessed();
                mesh.Mesh.Decompose(out var positions, out var normals, out var uvs,
                    out var triangles, out var groups);
                if (triangles.Count == 0) continue;
                writer.WriteLine("o " + ObjName(leaf.Path));
                WriteMesh(writer, positions, normals, uvs, triangles, groups,
                    mesh.groupIdToExtendedName, offset);
                offset += positions.Count;
            }
        }

        private static string ObjName(string name) => string.IsNullOrWhiteSpace(name)
            ? "unnamed"
            : string.Join("_", name.Split((char[])null, StringSplitOptions.RemoveEmptyEntries));

        private static void WriteMesh(StreamWriter writer, List<Vec3D> positions, List<Vec3D> normals,
            List<Vec2D> uvs, List<Tri> triangles, List<int> groups,
            Dictionary<int, string> groupNames, int offset)
        {
            if (normals != null && normals.Count != positions.Count)
                throw new ArgumentException("OBJ normals must match decomposed positions.");
            if (uvs != null && uvs.Count != positions.Count)
                throw new ArgumentException("OBJ UVs must match decomposed positions.");
            bool hasNormals = normals != null && normals.Count > 0;
            bool hasUvs = uvs != null && uvs.Count > 0;
            foreach (var p in positions)
                writer.WriteLine(string.Format(CultureInfo.InvariantCulture, "v {0:G17} {1:G17} {2:G17}", p.X, p.Y, p.Z));
            if (hasUvs)
                foreach (var t in uvs)
                    writer.WriteLine(string.Format(CultureInfo.InvariantCulture, "vt {0:G17} {1:G17}", t.X, t.Y));
            if (hasNormals)
                foreach (var n in normals)
                    writer.WriteLine(string.Format(CultureInfo.InvariantCulture, "vn {0:G17} {1:G17} {2:G17}", n.X, n.Y, n.Z));

            // A position may occur more than once: each decomposed index owns its
            // normal and UV, so hard edges and texture seams survive OBJ import.
            int previousGroup = int.MinValue;
            for (int i = 0; i < triangles.Count; i++)
            {
                int group = groups != null && i < groups.Count ? groups[i] : 0;
                if (group != previousGroup)
                {
                    writer.WriteLine("g " + ObjName(groupNames != null && groupNames.TryGetValue(group, out var name)
                        ? name : "group_" + group.ToString(CultureInfo.InvariantCulture)));
                    previousGroup = group;
                }
                var tri = triangles[i];
                writer.WriteLine("f " + Corner(tri.A) + " " + Corner(tri.B) + " " + Corner(tri.C));
            }

            string Corner(int index)
            {
                int obj = offset + index;
                string value = obj.ToString(CultureInfo.InvariantCulture);
                if (hasUvs && hasNormals) return value + "/" + value + "/" + value;
                if (hasUvs) return value + "/" + value;
                if (hasNormals) return value + "//" + value;
                return value;
            }
        }
    }
}
