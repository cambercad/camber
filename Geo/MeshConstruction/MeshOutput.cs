using GeoCore;

namespace Geo
{
    /// <summary>
    /// Bundles the output lists commonly produced by mesh generation methods (Extruder, Revolver, etc.).
    /// </summary>
    public class MeshOutput
    {
        internal Func<INurbsSurface> LoftSideSupportFactory { get; set; }
        internal Dictionary<string, ParametricRange> LoftSideDomains { get; set; }

        public List<Tri> Triangles { get; } = new List<Tri>();
        public List<Vec3D> Vertices { get; } = new List<Vec3D>();
        public List<Vec3D> Normals { get; } = new List<Vec3D>();
        public List<Vec2D> UVs { get; } = new List<Vec2D>();
        public List<int> TriangleGroups { get; } = new List<int>();
        public List<Rat3Hybrid> PrecisePositions { get; } = new List<Rat3Hybrid>();

        public void TransformToWorldSpace(CoordinateConverter converter, CoordinateSystem cs)
        {
            PreciseFrameTransform.Apply(converter, cs, Vertices, Normals, PrecisePositions);
        }

        public void RegeneratePrecisePositions(CoordinateConverter converter)
        {
            PrecisePositions.Clear();
            for (int i = 0; i < Vertices.Count; i++)
            {
                Int3 intPos = converter.Convert(Vertices[i]);
                PrecisePositions.Add(new Rat3Hybrid(intPos.X, intPos.Y, intPos.Z));
            }
        }

        public void AppendTo(List<Tri> triangles, List<Vec3D> vertices, List<Vec3D> normals,
            List<Vec2D> uv, List<int> triangleGroups, List<Rat3Hybrid> precisePositions)
        {
            triangles.AddRange(Triangles);
            vertices.AddRange(Vertices);
            normals.AddRange(Normals);
            uv.AddRange(UVs);
            triangleGroups.AddRange(TriangleGroups);
            precisePositions.AddRange(PrecisePositions);
        }
    }

    /// <summary>
    /// Bundles naming parameters for extrude/revolve operations.
    /// After calling the mesh generation method, TriangleGroupToName is populated with the mapping.
    /// </summary>
    public class MeshNaming
    {
        public List<List<string>> ContourNames { get; set; }
        public string OperationName { get; set; }
        public Dictionary<int, string> TriangleGroupToName { get; set; } = new Dictionary<int, string>();
    }
}
