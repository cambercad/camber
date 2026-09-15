using GeoCore;

namespace Geo
{
    public static class ObjWriter
    {
        public static void Write(string fileName, List<Vec3D> points, List<Vec3D> normals, List<Vec2D> uv, List<Tri> triangles, 
            List<int> groupPerTriangle, Dictionary<int, string> groupToName)
        {
            // Group triangles by their group IDs
            Dictionary<int, List<Tri>> trianglesPerGroup = new Dictionary<int, List<Tri>>();
            Dictionary<int, List<int>> triangleIndicesPerGroup = new Dictionary<int, List<int>>();
            
            for (int i = 0; i < triangles.Count; i++)
            {
                int groupId = (groupPerTriangle != null && i < groupPerTriangle.Count) ? groupPerTriangle[i] : 0;
                
                if (!trianglesPerGroup.ContainsKey(groupId))
                {
                    trianglesPerGroup[groupId] = new List<Tri>();
                    triangleIndicesPerGroup[groupId] = new List<int>();
                }
                
                trianglesPerGroup[groupId].Add(triangles[i]);
                triangleIndicesPerGroup[groupId].Add(i);
            }

            using (StreamWriter sw = new StreamWriter(fileName))
            {
                // Write vertices
                for (int i = 0; i < points.Count; i++)
                {
                    Vec3D p = points[i];
                    sw.WriteLine($"v {p.X:F6} {p.Y:F6} {p.Z:F6}");
                }

                // Write normals if available
                if (normals != null && normals.Count > 0)
                {
                    for (int i = 0; i < normals.Count; i++)
                    {
                        Vec3D n = normals[i];
                        sw.WriteLine($"vn {n.X:F6} {n.Y:F6} {n.Z:F6}");
                    }
                }

                // Write texture coordinates if available
                if (uv != null && uv.Count > 0)
                {
                    for (int i = 0; i < uv.Count; i++)
                    {
                        Vec2D t = uv[i];
                        sw.WriteLine($"vt {t.X:F6} {t.Y:F6}");
                    }
                }

                // Write groups with their triangles
                foreach (var groupEntry in trianglesPerGroup.OrderBy(kvp => kvp.Key))
                {
                    int groupId = groupEntry.Key;
                    List<Tri> groupTriangles = groupEntry.Value;

                    // Generate group name
                    string groupName;
                    if (groupToName != null && groupToName.ContainsKey(groupId))
                    {
                        groupName = groupToName[groupId];
                    }
                    else
                    {
                        groupName = $"group_{groupId}";
                    }

                    // Write group header
                    sw.WriteLine($"g {groupName}");

                    // Write faces for this group
                    foreach (Tri tri in groupTriangles)
                    {
                        // OBJ indices are 1-based
                        int a = tri.A + 1;
                        int b = tri.B + 1;
                        int c = tri.C + 1;

                        // Determine face format based on available data
                        if (normals != null && normals.Count > 0 && uv != null && uv.Count > 0)
                        {
                            // Format: f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
                            sw.WriteLine($"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}");
                        }
                        else if (normals != null && normals.Count > 0)
                        {
                            // Format: f v1//vn1 v2//vn2 v3//vn3
                            sw.WriteLine($"f {a}//{a} {b}//{b} {c}//{c}");
                        }
                        else if (uv != null && uv.Count > 0)
                        {
                            // Format: f v1/vt1 v2/vt2 v3/vt3
                            sw.WriteLine($"f {a}/{a} {b}/{b} {c}/{c}");
                        }
                        else
                        {
                            // Format: f v1 v2 v3
                            sw.WriteLine($"f {a} {b} {c}");
                        }
                    }
                }
            }
        }
    }
}
