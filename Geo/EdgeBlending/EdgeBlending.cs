#pragma warning disable CS8600 // Converting null literal or possible null value to non-nullable type

using GeoCore;

namespace Geo
{
    /// <summary>
    /// Provides CAD-style edge blending (filleting) for triangle meshes.
    /// Creates smooth cylindrical blend surfaces along edges and spherical patches at corners.
    /// Ensures G1 continuity (tangent plane continuity) at all junctions.
    /// </summary>
    public class EdgeBlending
    {
        private readonly EdgeBlendPipeline _pipeline = new();

        public EdgeGraph edgeGraph => _pipeline.edgeGraph;
        public List<GraphEdge> graphEdgesToBlend => _pipeline.graphEdgesToBlend;
        public List<BlendEdge> blendEdges => _pipeline.blendEdges;
        public Dictionary<int, UVSurface> originalSurfaces => _pipeline.originalSurfaces;

        /// <summary>
        /// Apply CAD-style edge blending to the specified edges of a mesh.
        /// </summary>
        public AnchorMesh BlendEdges(
            AnchorMesh mesh,
            List<string> edgeNamesToBlend,
            double blendRadius,
            CoordinateConverter cc,
            double maxDiscretizationDeviation,
            ref int groupIdOffset)
        {
            if (mesh == null)
                throw new ArgumentNullException(nameof(mesh));
            if (mesh.Mesh == null || mesh.Mesh.Positions == null || mesh.Mesh.Positions.Count == 0)
                throw new ArgumentException("Mesh must have valid geometry", nameof(mesh));
            if (mesh.Mesh.PrecisionPositions == null || mesh.Mesh.PrecisionPositions.Count == 0)
                throw new ArgumentException("Mesh must have PrecisionPositions for edge blending", nameof(mesh));
            if (edgeNamesToBlend == null || edgeNamesToBlend.Count == 0)
                throw new ArgumentException("No edges specified for blending", nameof(edgeNamesToBlend));
            if (blendRadius <= 0)
                throw new ArgumentException("Blend radius must be positive", nameof(blendRadius));
            if (cc.IntegerBoundingBox.Max.X == 0)
                throw new ArgumentException("Invalid coordinate converter", nameof(cc));

            return _pipeline.Run(
                mesh,
                edgeNamesToBlend,
                new FilletProfile(blendRadius),
                cc,
                maxDiscretizationDeviation,
                ref groupIdOffset);
        }
    }

    /// <summary>
    /// Union-Find (Disjoint Set Union) data structure for efficiently grouping elements into sets.
    /// </summary>
    public class UnionFind
    {
        private int[] parent;
        private int[] rank;
        private int count;

        public UnionFind(int size)
        {
            parent = new int[size];
            rank = new int[size];
            count = size;

            for (int i = 0; i < size; i++)
            {
                parent[i] = i;
                rank[i] = 0;
            }
        }

        public int Find(int x)
        {
            if (parent[x] != x)
                parent[x] = Find(parent[x]);
            return parent[x];
        }

        public void Union(int x, int y)
        {
            int rootX = Find(x);
            int rootY = Find(y);

            if (rootX == rootY)
                return;

            if (rank[rootX] < rank[rootY])
                parent[rootX] = rootY;
            else if (rank[rootX] > rank[rootY])
                parent[rootY] = rootX;
            else
            {
                parent[rootY] = rootX;
                rank[rootX]++;
            }

            count--;
        }

        public bool Connected(int x, int y) => Find(x) == Find(y);

        public int Count => count;

        public List<List<int>> GetComponents()
        {
            Dictionary<int, List<int>> componentMap = new Dictionary<int, List<int>>();

            for (int i = 0; i < parent.Length; i++)
            {
                int root = Find(i);
                if (!componentMap.ContainsKey(root))
                    componentMap[root] = new List<int>();
                componentMap[root].Add(i);
            }

            return new List<List<int>>(componentMap.Values);
        }
    }
}
