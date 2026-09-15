using CSG;
using GeoCore;
using GeoMeta;

namespace Geo
{
    public partial class GeoAPI
    {
        [APIDescription(@"CreateFromTriangles(positions: List[Vec3D], triangles: List[Tri], name: str = None) -> AnchorMesh
Build a solid from a triangle soup in world coordinates. Vertices are quantized onto the part lattice. isVolume follows watertightness.")]
        public AnchorMesh CreateFromTriangles(List<Vec3D> positions, List<Tri> triangles, string name = null)
        {
            if (positions == null || positions.Count == 0)
                throw new ArgumentException("positions is empty.", nameof(positions));
            if (triangles == null || triangles.Count == 0)
                throw new ArgumentException("triangles is empty.", nameof(triangles));

            name = name ?? GenerateName("Mesh");
            var normals = new List<Vec3D>(positions.Count);
            for (int i = 0; i < positions.Count; i++)
                normals.Add(default);
            for (int i = 0; i < triangles.Count; i++)
            {
                Tri t = triangles[i];
                if ((uint)t.A >= (uint)positions.Count || (uint)t.B >= (uint)positions.Count || (uint)t.C >= (uint)positions.Count)
                    throw new ArgumentOutOfRangeException(nameof(triangles), "Triangle index out of range.");
                Vec3D n = GeometricAlgorithms.ComputeTriangleNormal(positions[t.A], positions[t.B], positions[t.C], true);
                normals[t.A] = normals[t.A] + n;
                normals[t.B] = normals[t.B] + n;
                normals[t.C] = normals[t.C] + n;
            }
            for (int i = 0; i < normals.Count; i++)
            {
                double len = normals[i].Length();
                if (len > 1e-30)
                    normals[i] = normals[i] * (1.0 / len);
                else
                    normals[i] = new Vec3D(0, 0, 1);
            }

            var uvs = new List<Vec2D>(positions.Count);
            for (int i = 0; i < positions.Count; i++)
                uvs.Add(default);

            int gid = GetBaseGroupIndex();
            IncrementBaseGroupIndex(1);
            var groups = new List<int>(triangles.Count);
            for (int i = 0; i < triangles.Count; i++)
                groups.Add(gid);

            var mesh = new MeshNormalUV(converter, positions, normals, uvs, triangles, groups, skipWatertightCheck: true);
            bool volume = MeshAnalysis.IsWatertightMesh(mesh.Positions, mesh.Triangles);
            var names = new Dictionary<int, string> { { gid, name } };
            var result = new AnchorMesh(name, mesh, names, new Dictionary<string, SurfaceMetaData>(), volume);
            RegisterMesh(result);
            return result;
        }
    }
}
