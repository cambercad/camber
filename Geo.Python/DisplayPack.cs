using System.IO;
using System.Text;
using Geo;
using GeoCore;

namespace GeoPy;

/// <summary>
/// Packs named patches, curves, and anchor points for the Python viewer (little-endian blob).
/// </summary>
internal static class DisplayPack
{
    const uint Version = 3;
    static readonly byte[] Magic = { (byte)'C', (byte)'M', (byte)'B', (byte)'R' };

    public static string PackSolid(AnchorMesh mesh)
    {
        if (mesh == null)
            return Convert.ToBase64String(PackEmpty());
        mesh.EnsureCoplanarPostProcessed();
        using (var ms = new MemoryStream())
        using (var w = new BinaryWriter(ms, Encoding.UTF8, true))
        {
            w.Write(Magic);
            w.Write(Version);
            w.Write((uint)1);
            WriteMesh(w, mesh);
            return Convert.ToBase64String(ms.ToArray());
        }
    }

    public static string PackMeshes(System.Collections.Generic.IReadOnlyList<AnchorMesh> meshes)
    {
        int n = 0;
        if (meshes != null)
        {
            for (int i = 0; i < meshes.Count; i++)
            {
                if (meshes[i] != null)
                    n++;
            }
        }
        using (var ms = new MemoryStream())
        using (var w = new BinaryWriter(ms, Encoding.UTF8, true))
        {
            w.Write(Magic);
            w.Write(Version);
            w.Write((uint)n);
            if (meshes != null)
            {
                for (int i = 0; i < meshes.Count; i++)
                {
                    if (meshes[i] == null)
                        continue;
                    meshes[i].EnsureCoplanarPostProcessed();
                    WriteMesh(w, meshes[i]);
                }
            }
            return Convert.ToBase64String(ms.ToArray());
        }
    }

    /// <summary>
    /// Pack each assembly occurrence after applying its pose. Shared meshes must be
    /// snapshotted here: Update() mutates the same vertex buffer, so collecting
    /// references and writing later would show only the last instance.
    /// </summary>
    public static string PackAssembly(System.Collections.Generic.IReadOnlyList<AssemblyPart> parts)
    {
        int n = 0;
        if (parts != null)
        {
            for (int i = 0; i < parts.Count; i++)
            {
                if (parts[i] != null && parts[i].Mesh != null)
                    n++;
            }
        }
        using (var ms = new MemoryStream())
        using (var w = new BinaryWriter(ms, Encoding.UTF8, true))
        {
            w.Write(Magic);
            w.Write(Version);
            w.Write((uint)n);
            if (parts != null)
            {
                for (int i = 0; i < parts.Count; i++)
                {
                    AssemblyPart part = parts[i];
                    if (part == null || part.Mesh == null)
                        continue;
                    part.Mesh.Update(part.EvaluatePose());
                    part.Mesh.EnsureCoplanarPostProcessed();
                    WriteMesh(w, part.Mesh);
                }
            }
            return Convert.ToBase64String(ms.ToArray());
        }
    }

    static byte[] PackEmpty()
    {
        using (var ms = new MemoryStream())
        using (var w = new BinaryWriter(ms, Encoding.UTF8, true))
        {
            w.Write(Magic);
            w.Write(Version);
            w.Write((uint)0);
            return ms.ToArray();
        }
    }

    static void WriteMesh(BinaryWriter w, AnchorMesh mesh)
    {
        string prefix = mesh.Name ?? "mesh";
        mesh.Mesh.Decompose(out var positions, out var normals, out var uvs, out var triangles, out var perTriangleGroup);

        w.Write(positions.Count);
        for (int i = 0; i < positions.Count; i++)
        {
            w.Write(positions[i].X);
            w.Write(positions[i].Y);
            w.Write(positions[i].Z);
        }
        int nNorm = normals != null ? normals.Count : 0;
        w.Write(nNorm);
        for (int i = 0; i < nNorm; i++)
        {
            w.Write(normals[i].X);
            w.Write(normals[i].Y);
            w.Write(normals[i].Z);
        }
        int nUv = uvs != null ? uvs.Count : 0;
        w.Write(nUv);
        for (int i = 0; i < nUv; i++)
        {
            w.Write(uvs[i].X);
            w.Write(uvs[i].Y);
        }
        w.Write(triangles.Count);
        for (int i = 0; i < triangles.Count; i++)
        {
            w.Write(triangles[i].A);
            w.Write(triangles[i].B);
            w.Write(triangles[i].C);
            int gid = (perTriangleGroup != null && i < perTriangleGroup.Count) ? perTriangleGroup[i] : 0;
            w.Write(gid);
        }

        var usedGroups = new System.Collections.Generic.Dictionary<int, string>();
        if (perTriangleGroup != null)
        {
            for (int i = 0; i < perTriangleGroup.Count; i++)
            {
                int gid = perTriangleGroup[i];
                if (usedGroups.ContainsKey(gid))
                    continue;
                string local;
                if (mesh.groupIdToExtendedName == null || !mesh.groupIdToExtendedName.TryGetValue(gid, out local) || string.IsNullOrEmpty(local))
                    local = "group_" + gid.ToString();
                usedGroups[gid] = prefix + ":" + local;
            }
        }
        w.Write(usedGroups.Count);
        foreach (var kv in usedGroups)
        {
            w.Write(kv.Key);
            WriteUtf8(w, kv.Value);
            SurfaceType surfaceType = SurfaceType.Unknown;
            string localName = kv.Value;
            int colon = localName.IndexOf(':');
            if (colon >= 0 && colon + 1 < localName.Length)
                localName = localName.Substring(colon + 1);
            if (mesh.surfaceMetaData != null
                && mesh.surfaceMetaData.TryGetValue(localName, out var smd)
                && smd != null)
                surfaceType = smd.SurfaceType;
            w.Write((int)surfaceType);
        }

        int curveCount = 0;
        if (mesh.GroupEdges != null)
        {
            for (int i = 0; i < mesh.GroupEdges.Count; i++)
            {
                var g = mesh.GroupEdges[i];
                if (g.LineStrips3D == null)
                    continue;
                curveCount += g.LineStrips3D.Count;
            }
        }
        w.Write(curveCount);
        if (mesh.GroupEdges != null)
        {
            for (int i = 0; i < mesh.GroupEdges.Count; i++)
            {
                var g = mesh.GroupEdges[i];
                if (g.LineStrips3D == null)
                    continue;
                for (int j = 0; j < g.LineStrips3D.Count; j++)
                {
                    var strip = g.LineStrips3D[j];
                    string name = g.Name ?? "edge";
                    if (g.LineStrips3D.Count > 1)
                        name = name + "_" + j.ToString();
                    string pickName = prefix + ":" + name;
                    WriteUtf8(w, pickName);
                    EdgeCurveType edgeType = g.MetaData != null ? g.MetaData.CurveType : EdgeCurveType.Unknown;
                    w.Write((int)edgeType);
                    bool closed = strip.IsClosed();
                    w.Write(closed ? (byte)1 : (byte)0);
                    int np = strip.Points.Count;
                    w.Write(np);
                    for (int k = 0; k < np; k++)
                    {
                        w.Write(strip.Points[k].X);
                        w.Write(strip.Points[k].Y);
                        w.Write(strip.Points[k].Z);
                    }

                    int nAnchors = closed ? 5 : 3;
                    w.Write(nAnchors);
                    WriteAnchor(w, pickName, strip, 0.0);
                    WriteAnchor(w, pickName, strip, 0.5);
                    WriteAnchor(w, pickName, strip, 1.0);
                    if (closed)
                    {
                        WriteAnchor(w, pickName, strip, 0.25);
                        WriteAnchor(w, pickName, strip, 0.75);
                    }
                }
            }
        }
    }

    static void WriteAnchor(BinaryWriter w, string pickName, LineStrip3D strip, double u)
    {
        WriteUtf8(w, pickName + "@" + u.ToString("0.000", System.Globalization.CultureInfo.InvariantCulture));
        Vec3D p = strip.EvaluateUniform(u);
        w.Write(p.X);
        w.Write(p.Y);
        w.Write(p.Z);
    }

    static void WriteUtf8(BinaryWriter w, string s)
    {
        if (s == null)
            s = "";
        byte[] bytes = Encoding.UTF8.GetBytes(s);
        w.Write(bytes.Length);
        w.Write(bytes);
    }
}
