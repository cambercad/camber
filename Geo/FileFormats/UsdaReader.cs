using System.Globalization;
using System.Text;
using GeoCore;

namespace Geo
{
    /// <summary>
    /// One UsdGeom Mesh from a USDA stage, triangulated. Corners are expanded when
    /// normals or UVs are face-varying so each vertex owns one normal and one UV.
    /// </summary>
    public sealed class UsdaMesh
    {
        public string Name;
        public List<Vec3D> Points = new List<Vec3D>();
        public List<Vec3D> Normals = new List<Vec3D>();
        public List<Vec2D> Uvs = new List<Vec2D>();
        public List<Tri> Triangles = new List<Tri>();
        /// <summary>Per-triangle index into <see cref="GroupNames"/>.</summary>
        public List<int> GroupPerTriangle = new List<int>();
        public List<string> GroupNames = new List<string>();
    }

    public sealed class UsdaDocument
    {
        public double MetersPerUnit = 1;
        /// <summary>"Y" or "Z". Geometry is still in the file's axis until the caller converts it.</summary>
        public string UpAxis = "Z";
        public List<UsdaMesh> Meshes = new List<UsdaMesh>();
    }

    /// <summary>
    /// USD ASCII reader for static triangle and polygon meshes (the subset camber writes,
    /// plus quads/n-gons, GeomSubset face groups, and vertex or face-varying normals/UVs).
    /// </summary>
    public static class UsdaReader
    {
        public static UsdaDocument Read(string fileName)
        {
            if (string.IsNullOrWhiteSpace(fileName))
                throw new ArgumentException("USDA path is required.", nameof(fileName));
            var tokens = Tokenize(File.ReadAllText(fileName));
            var doc = new UsdaDocument();
            int i = 0;
            while (i < tokens.Count)
            {
                Token t = tokens[i];
                if (t.Kind == Tok.LParen)
                {
                    i = ReadHeader(tokens, i + 1, doc);
                    continue;
                }
                if (t.Kind == Tok.Id && (t.Text == "def" || t.Text == "over"))
                {
                    i = ReadPrim(tokens, i, doc);
                    continue;
                }
                i++;
            }
            if (doc.Meshes.Count == 0)
                throw new InvalidDataException("USDA file contains no Mesh prim.");
            return doc;
        }

        /// <summary>Y-up to Z-up, right-handed: (x, y, z) → (x, −z, y).</summary>
        public static Vec3D YUpToZUp(Vec3D p) => new Vec3D(p.X, -p.Z, p.Y);

        static int ReadHeader(List<Token> tokens, int i, UsdaDocument doc)
        {
            while (i < tokens.Count && tokens[i].Kind != Tok.RParen)
            {
                if (tokens[i].Kind == Tok.Id && i + 2 < tokens.Count && tokens[i + 1].Kind == Tok.Eq)
                {
                    string key = tokens[i].Text;
                    Token value = tokens[i + 2];
                    if (key == "metersPerUnit" && value.Kind == Tok.Num && value.Number > 0)
                        doc.MetersPerUnit = value.Number;
                    else if (key == "upAxis" && value.Kind == Tok.Str && value.Text.Length > 0)
                        doc.UpAxis = value.Text;
                    i += 3;
                    continue;
                }
                i++;
            }
            return i < tokens.Count ? i + 1 : i;
        }

        static int ReadPrim(List<Token> tokens, int i, UsdaDocument doc)
        {
            i++; // def / over
            string type = "Xform";
            string name = "Prim";
            if (i < tokens.Count && tokens[i].Kind == Tok.Id)
            {
                type = tokens[i].Text;
                i++;
            }
            if (i < tokens.Count && tokens[i].Kind == Tok.Str)
            {
                name = tokens[i].Text;
                i++;
            }
            if (i < tokens.Count && tokens[i].Kind == Tok.LParen)
                i = SkipParens(tokens, i);
            if (i >= tokens.Count || tokens[i].Kind != Tok.LBrace)
                return i;
            i++;
            if (type == "Mesh")
                return ReadMesh(tokens, i, name, doc);
            return ReadBlock(tokens, i, doc);
        }

        static int ReadBlock(List<Token> tokens, int i, UsdaDocument doc)
        {
            while (i < tokens.Count && tokens[i].Kind != Tok.RBrace)
            {
                if (tokens[i].Kind == Tok.Id && (tokens[i].Text == "def" || tokens[i].Text == "over"))
                {
                    i = ReadPrim(tokens, i, doc);
                    continue;
                }
                if (tokens[i].Kind == Tok.LBrace)
                {
                    i = ReadBlock(tokens, i + 1, doc);
                    continue;
                }
                if (tokens[i].Kind == Tok.LParen)
                {
                    i = SkipParens(tokens, i);
                    continue;
                }
                i++;
            }
            return i < tokens.Count ? i + 1 : i;
        }

        static int ReadMesh(List<Token> tokens, int i, string name, UsdaDocument doc)
        {
            var points = new List<Vec3D>();
            var normals = new List<Vec3D>();
            var uvs = new List<Vec2D>();
            var counts = new List<int>();
            var indices = new List<int>();
            var normalIndices = new List<int>();
            var uvIndices = new List<int>();
            string normalInterp = null;
            string uvInterp = null;
            var subsets = new List<(string Name, List<int> Faces)>();

            while (i < tokens.Count && tokens[i].Kind != Tok.RBrace)
            {
                if (tokens[i].Kind == Tok.Id && (tokens[i].Text == "def" || tokens[i].Text == "over"))
                {
                    int look = i + 1;
                    string nestedType = look < tokens.Count && tokens[look].Kind == Tok.Id ? tokens[look].Text : "";
                    if (nestedType == "GeomSubset")
                        i = ReadSubset(tokens, i, subsets);
                    else
                        i = ReadPrim(tokens, i, doc);
                    continue;
                }
                if (!TryReadAttribute(tokens, ref i, out string attr, out List<double> numbers, out List<Vec3D> vec3, out List<Vec2D> vec2, out string metaInterp))
                {
                    i++;
                    continue;
                }
                if (attr == "points" && vec3 != null)
                    points = vec3;
                else if (attr == "normals" && vec3 != null)
                {
                    normals = vec3;
                    normalInterp = metaInterp ?? normalInterp;
                }
                else if ((attr == "primvars:st" || attr == "primvars:UV" || attr == "st") && vec2 != null)
                {
                    uvs = vec2;
                    uvInterp = metaInterp ?? uvInterp;
                }
                else if (attr == "faceVertexCounts")
                    counts = ToInts(numbers);
                else if (attr == "faceVertexIndices")
                    indices = ToInts(numbers);
                else if (attr == "normals:indices")
                    normalIndices = ToInts(numbers);
                else if (attr == "primvars:st:indices" || attr == "primvars:UV:indices")
                    uvIndices = ToInts(numbers);
            }
            if (i < tokens.Count && tokens[i].Kind == Tok.RBrace)
                i++;

            if (points.Count > 0 && counts.Count > 0 && indices.Count > 0)
                doc.Meshes.Add(BuildMesh(name, points, normals, normalInterp, normalIndices, uvs, uvInterp, uvIndices, counts, indices, subsets));
            return i;
        }

        static int ReadSubset(List<Token> tokens, int i, List<(string, List<int>)> subsets)
        {
            i++; // def
            if (i < tokens.Count && tokens[i].Kind == Tok.Id)
                i++;
            string name = "subset";
            if (i < tokens.Count && tokens[i].Kind == Tok.Str)
            {
                name = tokens[i].Text;
                i++;
            }
            if (i < tokens.Count && tokens[i].Kind == Tok.LParen)
                i = SkipParens(tokens, i);
            if (i >= tokens.Count || tokens[i].Kind != Tok.LBrace)
                return i;
            i++;
            var faces = new List<int>();
            while (i < tokens.Count && tokens[i].Kind != Tok.RBrace)
            {
                if (TryReadAttribute(tokens, ref i, out string attr, out List<double> numbers, out _, out _, out _))
                {
                    if (attr == "indices")
                        faces = ToInts(numbers);
                }
                else
                    i++;
            }
            if (i < tokens.Count && tokens[i].Kind == Tok.RBrace)
                i++;
            subsets.Add((name, faces));
            return i;
        }

        static bool TryReadAttribute(List<Token> tokens, ref int i, out string name, out List<double> numbers, out List<Vec3D> vec3, out List<Vec2D> vec2, out string interpolation)
        {
            name = null;
            numbers = null;
            vec3 = null;
            vec2 = null;
            interpolation = null;
            int start = i;
            while (i < tokens.Count && tokens[i].Kind == Tok.Id && IsQualifier(tokens[i].Text))
                i++;
            if (i >= tokens.Count || tokens[i].Kind != Tok.Id)
            {
                i = start;
                return false;
            }
            i++; // type
            if (i < tokens.Count && tokens[i].Kind == Tok.LBrack)
            {
                i++;
                if (i < tokens.Count && tokens[i].Kind == Tok.RBrack)
                    i++;
            }
            if (i >= tokens.Count || tokens[i].Kind != Tok.Id)
            {
                i = start;
                return false;
            }
            name = tokens[i].Text;
            i++;
            if (i >= tokens.Count || tokens[i].Kind != Tok.Eq)
            {
                i = start;
                return false;
            }
            i++;
            if (i >= tokens.Count)
                return false;
            if (tokens[i].Kind == Tok.LBrack)
            {
                i++;
                numbers = new List<double>();
                vec3 = new List<Vec3D>();
                vec2 = new List<Vec2D>();
                while (i < tokens.Count && tokens[i].Kind != Tok.RBrack)
                {
                    if (tokens[i].Kind == Tok.LParen)
                    {
                        i++;
                        var comps = new List<double>();
                        while (i < tokens.Count && tokens[i].Kind != Tok.RParen)
                        {
                            if (tokens[i].Kind == Tok.Num)
                                comps.Add(tokens[i].Number);
                            i++;
                        }
                        if (i < tokens.Count && tokens[i].Kind == Tok.RParen)
                            i++;
                        if (comps.Count >= 3)
                            vec3.Add(new Vec3D(comps[0], comps[1], comps[2]));
                        if (comps.Count >= 2)
                            vec2.Add(new Vec2D(comps[0], comps[1]));
                        foreach (double c in comps)
                            numbers.Add(c);
                    }
                    else if (tokens[i].Kind == Tok.Num)
                    {
                        numbers.Add(tokens[i].Number);
                        i++;
                    }
                    else
                        i++;
                }
                if (i < tokens.Count && tokens[i].Kind == Tok.RBrack)
                    i++;
                if (i < tokens.Count && tokens[i].Kind == Tok.LParen)
                {
                    int end = SkipParens(tokens, i);
                    interpolation = FindInterpolation(tokens, i, end);
                    i = end;
                }
                return true;
            }
            // Scalar, string, or asset path: consume it. Callers ignore unknown names.
            i++;
            return true;
        }

        static string FindInterpolation(List<Token> tokens, int from, int to)
        {
            for (int k = from; k + 2 < to; k++)
            {
                if (tokens[k].Kind == Tok.Id && tokens[k].Text == "interpolation" && tokens[k + 1].Kind == Tok.Eq && tokens[k + 2].Kind == Tok.Str)
                    return tokens[k + 2].Text;
            }
            return null;
        }

        static int SkipParens(List<Token> tokens, int i)
        {
            int depth = 0;
            do
            {
                if (tokens[i].Kind == Tok.LParen)
                    depth++;
                else if (tokens[i].Kind == Tok.RParen)
                    depth--;
                i++;
            }
            while (i < tokens.Count && depth > 0);
            return i;
        }

        static bool IsQualifier(string text) =>
            text == "uniform" || text == "varying" || text == "custom" || text == "prepend" || text == "append";

        static List<int> ToInts(List<double> numbers)
        {
            var list = new List<int>(numbers?.Count ?? 0);
            if (numbers == null)
                return list;
            foreach (double n in numbers)
                list.Add((int)Math.Round(n));
            return list;
        }

        static UsdaMesh BuildMesh(
            string name,
            List<Vec3D> points,
            List<Vec3D> normals,
            string normalInterp,
            List<int> normalIndices,
            List<Vec2D> uvs,
            string uvInterp,
            List<int> uvIndices,
            List<int> counts,
            List<int> indices,
            List<(string Name, List<int> Faces)> subsets)
        {
            int corners = 0;
            foreach (int c in counts)
                corners += Math.Max(0, c);
            bool normalFaceVarying = IsFaceVarying(normalInterp, normals.Count, points.Count, corners);
            bool uvFaceVarying = IsFaceVarying(uvInterp, uvs.Count, points.Count, corners);
            bool expand = normalFaceVarying || uvFaceVarying || normalIndices.Count == corners || uvIndices.Count == corners;

            var mesh = new UsdaMesh { Name = string.IsNullOrEmpty(name) ? "Mesh" : name };
            var faceGroups = FaceGroups(counts.Count, subsets, mesh);

            if (!expand)
            {
                mesh.Points = points;
                if (normals.Count == points.Count)
                    mesh.Normals = normals;
                if (uvs.Count == points.Count)
                    mesh.Uvs = uvs;
            }

            int corner = 0;
            for (int face = 0; face < counts.Count; face++)
            {
                int count = counts[face];
                if (count < 3 || corner + count > indices.Count)
                {
                    corner += Math.Max(0, count);
                    continue;
                }
                var poly = new List<Vec3D>(count);
                var cornerPoint = new List<int>(count);
                var cornerNormal = new List<Vec3D>(count);
                var cornerUv = new List<Vec2D>(count);
                bool ok = true;
                for (int k = 0; k < count; k++)
                {
                    int pi = indices[corner + k];
                    if (pi < 0 || pi >= points.Count)
                    {
                        ok = false;
                        break;
                    }
                    poly.Add(points[pi]);
                    cornerPoint.Add(pi);
                    cornerNormal.Add(NormalAt(normals, normalIndices, normalInterp, pi, corner + k, face, corners));
                    cornerUv.Add(UvAt(uvs, uvIndices, uvInterp, pi, corner + k, corners));
                }
                corner += count;
                if (!ok)
                    continue;

                List<int> localTris = Triangulate(poly);
                int group = face < faceGroups.Count ? faceGroups[face] : 0;
                for (int t = 0; t < localTris.Count; t += 3)
                {
                    int a = CornerVertex(mesh, expand, points, cornerPoint, cornerNormal, cornerUv, normals.Count > 0, uvs.Count > 0, localTris[t]);
                    int b = CornerVertex(mesh, expand, points, cornerPoint, cornerNormal, cornerUv, normals.Count > 0, uvs.Count > 0, localTris[t + 1]);
                    int c = CornerVertex(mesh, expand, points, cornerPoint, cornerNormal, cornerUv, normals.Count > 0, uvs.Count > 0, localTris[t + 2]);
                    if (a == b || b == c || a == c)
                        continue;
                    mesh.Triangles.Add(new Tri(a, b, c));
                    mesh.GroupPerTriangle.Add(group);
                }
            }
            if (mesh.GroupNames.Count == 0)
                mesh.GroupNames.Add(mesh.Name);
            return mesh;
        }

        static List<int> FaceGroups(int faceCount, List<(string Name, List<int> Faces)> subsets, UsdaMesh mesh)
        {
            var groups = new List<int>(faceCount);
            var assigned = new bool[faceCount];
            for (int i = 0; i < faceCount; i++)
                groups.Add(0);
            if (subsets == null || subsets.Count == 0)
            {
                mesh.GroupNames.Add(mesh.Name);
                return groups;
            }
            var known = new Dictionary<string, int>(StringComparer.Ordinal);
            foreach (var subset in subsets)
            {
                if (subset.Faces == null)
                    continue;
                if (!known.TryGetValue(subset.Name, out int group))
                {
                    group = mesh.GroupNames.Count;
                    mesh.GroupNames.Add(string.IsNullOrEmpty(subset.Name) ? "subset" : subset.Name);
                    known[subset.Name ?? ""] = group;
                }
                foreach (int face in subset.Faces)
                {
                    if (face >= 0 && face < faceCount)
                    {
                        groups[face] = group;
                        assigned[face] = true;
                    }
                }
            }
            if (mesh.GroupNames.Count == 0)
            {
                mesh.GroupNames.Add(mesh.Name);
                return groups;
            }
            int fallback = -1;
            for (int face = 0; face < faceCount; face++)
            {
                if (assigned[face])
                    continue;
                if (fallback < 0)
                {
                    fallback = mesh.GroupNames.Count;
                    mesh.GroupNames.Add(mesh.Name);
                }
                groups[face] = fallback;
            }
            return groups;
        }

        static bool IsFaceVarying(string interpolation, int valueCount, int pointCount, int cornerCount)
        {
            if (string.Equals(interpolation, "faceVarying", StringComparison.OrdinalIgnoreCase))
                return true;
            if (string.Equals(interpolation, "uniform", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(interpolation, "constant", StringComparison.OrdinalIgnoreCase))
                return true;
            if (string.Equals(interpolation, "vertex", StringComparison.OrdinalIgnoreCase))
                return false;
            return valueCount == cornerCount && valueCount != pointCount;
        }

        static Vec3D NormalAt(List<Vec3D> normals, List<int> indices, string interpolation, int pointIndex, int cornerIndex, int faceIndex, int cornerCount)
        {
            if (normals == null || normals.Count == 0)
                return new Vec3D(0, 0, 1);
            if (indices != null && cornerIndex < indices.Count)
            {
                int ni = indices[cornerIndex];
                if (ni >= 0 && ni < normals.Count)
                    return normals[ni];
            }
            if (string.Equals(interpolation, "constant", StringComparison.OrdinalIgnoreCase))
                return normals[0];
            if (string.Equals(interpolation, "uniform", StringComparison.OrdinalIgnoreCase))
                return faceIndex >= 0 && faceIndex < normals.Count ? normals[faceIndex] : normals[0];
            if (string.Equals(interpolation, "faceVarying", StringComparison.OrdinalIgnoreCase) || normals.Count == cornerCount)
                return cornerIndex >= 0 && cornerIndex < normals.Count ? normals[cornerIndex] : new Vec3D(0, 0, 1);
            return pointIndex >= 0 && pointIndex < normals.Count ? normals[pointIndex] : new Vec3D(0, 0, 1);
        }

        static Vec2D UvAt(List<Vec2D> uvs, List<int> indices, string interpolation, int pointIndex, int cornerIndex, int cornerCount)
        {
            if (uvs == null || uvs.Count == 0)
                return new Vec2D(0, 0);
            if (indices != null && cornerIndex < indices.Count)
            {
                int ui = indices[cornerIndex];
                if (ui >= 0 && ui < uvs.Count)
                    return uvs[ui];
            }
            if (string.Equals(interpolation, "faceVarying", StringComparison.OrdinalIgnoreCase) || uvs.Count == cornerCount)
                return cornerIndex >= 0 && cornerIndex < uvs.Count ? uvs[cornerIndex] : new Vec2D(0, 0);
            return pointIndex >= 0 && pointIndex < uvs.Count ? uvs[pointIndex] : new Vec2D(0, 0);
        }

        static int CornerVertex(UsdaMesh mesh, bool expand, List<Vec3D> authored, List<int> pointIndex, List<Vec3D> normals, List<Vec2D> uvs, bool hasNormals, bool hasUv, int local)
        {
            if (!expand)
                return pointIndex[local];
            mesh.Points.Add(authored[pointIndex[local]]);
            if (hasNormals)
                mesh.Normals.Add(normals[local]);
            if (hasUv)
                mesh.Uvs.Add(uvs[local]);
            return mesh.Points.Count - 1;
        }

        static List<int> Triangulate(List<Vec3D> poly)
        {
            int n = poly.Count;
            if (n < 3)
                return new List<int>();
            if (n == 3)
                return new List<int> { 0, 1, 2 };
            Vec3D normal = new Vec3D(0, 0, 0);
            for (int i = 0; i < n; i++)
            {
                Vec3D a = poly[i];
                Vec3D b = poly[(i + 1) % n];
                normal.X += (a.Y - b.Y) * (a.Z + b.Z);
                normal.Y += (a.Z - b.Z) * (a.X + b.X);
                normal.Z += (a.X - b.X) * (a.Y + b.Y);
            }
            double nx = Math.Abs(normal.X), ny = Math.Abs(normal.Y), nz = Math.Abs(normal.Z);
            int drop = nx >= ny && nx >= nz ? 0 : (ny >= nz ? 1 : 2);
            var flat = new List<Vec2D>(n);
            for (int i = 0; i < n; i++)
            {
                Vec3D p = poly[i];
                flat.Add(drop == 0 ? new Vec2D(p.Y, p.Z) : drop == 1 ? new Vec2D(p.X, p.Z) : new Vec2D(p.X, p.Y));
            }
            double area = 0;
            for (int i = 0; i < n; i++)
            {
                Vec2D a = flat[i];
                Vec2D b = flat[(i + 1) % n];
                area += a.X * b.Y - b.X * a.Y;
            }
            bool flip = area < 0;
            var ring = new List<int>(n);
            if (!flip)
            {
                for (int i = 0; i < n; i++)
                    ring.Add(i);
            }
            else
            {
                for (int i = n - 1; i >= 0; i--)
                    ring.Add(i);
            }
            var tris = new List<int>();
            int guard = ring.Count * ring.Count + 4;
            while (ring.Count > 3 && guard-- > 0)
            {
                bool clipped = false;
                for (int i = 0; i < ring.Count; i++)
                {
                    int i0 = ring[(i + ring.Count - 1) % ring.Count];
                    int i1 = ring[i];
                    int i2 = ring[(i + 1) % ring.Count];
                    if (!IsEar(flat, ring, i0, i1, i2))
                        continue;
                    tris.Add(i0);
                    tris.Add(i1);
                    tris.Add(i2);
                    ring.RemoveAt(i);
                    clipped = true;
                    break;
                }
                if (!clipped)
                    break;
            }
            if (ring.Count == 3)
            {
                tris.Add(ring[0]);
                tris.Add(ring[1]);
                tris.Add(ring[2]);
            }
            else if (ring.Count > 3)
            {
                for (int i = 1; i < ring.Count - 1; i++)
                {
                    tris.Add(ring[0]);
                    tris.Add(ring[i]);
                    tris.Add(ring[i + 1]);
                }
            }
            if (flip)
            {
                for (int i = 0; i < tris.Count; i += 3)
                {
                    int tmp = tris[i + 1];
                    tris[i + 1] = tris[i + 2];
                    tris[i + 2] = tmp;
                }
            }
            return tris;
        }

        static bool IsEar(List<Vec2D> flat, List<int> ring, int i0, int i1, int i2)
        {
            Vec2D a = flat[i0], b = flat[i1], c = flat[i2];
            double cross = (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
            if (cross <= 1e-14)
                return false;
            for (int i = 0; i < ring.Count; i++)
            {
                int k = ring[i];
                if (k == i0 || k == i1 || k == i2)
                    continue;
                if (PointInTri(flat[k], a, b, c))
                    return false;
            }
            return true;
        }

        static bool PointInTri(Vec2D p, Vec2D a, Vec2D b, Vec2D c)
        {
            double c0 = Cross(b - a, p - a);
            double c1 = Cross(c - b, p - b);
            double c2 = Cross(a - c, p - c);
            return c0 > 1e-14 && c1 > 1e-14 && c2 > 1e-14;
        }

        static double Cross(Vec2D a, Vec2D b) => a.X * b.Y - a.Y * b.X;

        enum Tok { Id, Str, Num, LBrack, RBrack, LParen, RParen, LBrace, RBrace, Eq, Comma }

        struct Token
        {
            public Tok Kind;
            public string Text;
            public double Number;
        }

        static List<Token> Tokenize(string text)
        {
            var tokens = new List<Token>();
            int i = 0;
            while (i < text.Length)
            {
                char c = text[i];
                if (c == '\\' && i + 1 < text.Length && (text[i + 1] == '\n' || text[i + 1] == '\r'))
                {
                    i++;
                    continue;
                }
                if (char.IsWhiteSpace(c) || c == ',')
                {
                    i++;
                    continue;
                }
                if (c == '#')
                {
                    while (i < text.Length && text[i] != '\n')
                        i++;
                    continue;
                }
                if (c == '/' && i + 1 < text.Length && text[i + 1] == '/')
                {
                    while (i < text.Length && text[i] != '\n')
                        i++;
                    continue;
                }
                if (c == '"')
                {
                    i++;
                    var sb = new StringBuilder();
                    while (i < text.Length && text[i] != '"')
                    {
                        if (text[i] == '\\' && i + 1 < text.Length)
                        {
                            i++;
                            sb.Append(text[i] == 'n' ? '\n' : text[i]);
                        }
                        else
                            sb.Append(text[i]);
                        i++;
                    }
                    if (i < text.Length)
                        i++;
                    tokens.Add(new Token { Kind = Tok.Str, Text = sb.ToString() });
                    continue;
                }
                if (c == '[' || c == ']' || c == '(' || c == ')' || c == '{' || c == '}' || c == '=')
                {
                    Tok kind = c == '[' ? Tok.LBrack : c == ']' ? Tok.RBrack : c == '(' ? Tok.LParen : c == ')' ? Tok.RParen : c == '{' ? Tok.LBrace : c == '}' ? Tok.RBrace : Tok.Eq;
                    tokens.Add(new Token { Kind = kind, Text = c.ToString() });
                    i++;
                    continue;
                }
                if (c == '+' || c == '-' || c == '.' || char.IsDigit(c))
                {
                    int start = i;
                    if (c == '+' || c == '-')
                        i++;
                    bool digit = false;
                    while (i < text.Length && char.IsDigit(text[i]))
                    {
                        digit = true;
                        i++;
                    }
                    if (i < text.Length && text[i] == '.')
                    {
                        i++;
                        while (i < text.Length && char.IsDigit(text[i]))
                        {
                            digit = true;
                            i++;
                        }
                    }
                    if (digit && i < text.Length && (text[i] == 'e' || text[i] == 'E'))
                    {
                        int e = i;
                        i++;
                        if (i < text.Length && (text[i] == '+' || text[i] == '-'))
                            i++;
                        int exp = i;
                        while (i < text.Length && char.IsDigit(text[i]))
                            i++;
                        if (i == exp)
                            i = e;
                    }
                    if (!digit)
                    {
                        i = start + 1;
                        continue;
                    }
                    string slice = text.Substring(start, i - start);
                    double.TryParse(slice, NumberStyles.Float, CultureInfo.InvariantCulture, out double number);
                    tokens.Add(new Token { Kind = Tok.Num, Text = slice, Number = number });
                    continue;
                }
                if (char.IsLetter(c) || c == '_' || c == ':')
                {
                    int start = i;
                    i++;
                    while (i < text.Length)
                    {
                        char d = text[i];
                        if (char.IsLetterOrDigit(d) || d == '_' || d == ':' || d == '.')
                            i++;
                        else
                            break;
                    }
                    tokens.Add(new Token { Kind = Tok.Id, Text = text.Substring(start, i - start) });
                    continue;
                }
                i++;
            }
            return tokens;
        }
    }
}
