#pragma warning disable CS8600 // Converting null literal or possible null value to non-nullable type

using GeoCore;

namespace Geo
{
    /// <summary>
    /// Provides CAD-style symmetric edge chamfering for triangle meshes.
    /// </summary>
    public class ChamferBlending
    {
        private readonly EdgeBlendPipeline _pipeline = new();

        public EdgeGraph edgeGraph => _pipeline.edgeGraph;
        public List<GraphEdge> graphEdgesToBlend => _pipeline.graphEdgesToBlend;
        public List<BlendEdge> blendEdges => _pipeline.blendEdges;
        public Dictionary<int, UVSurface> originalSurfaces => _pipeline.originalSurfaces;

        public AnchorMesh ChamferEdges(
            AnchorMesh mesh,
            List<string> edgeNamesToChamfer,
            double chamferDistance,
            CoordinateConverter cc,
            double maxDiscretizationDeviation,
            ref int groupIdOffset)
        {
            if (mesh == null)
                throw new ArgumentNullException(nameof(mesh));
            if (mesh.Mesh == null || mesh.Mesh.Positions == null || mesh.Mesh.Positions.Count == 0)
                throw new ArgumentException("Mesh must have valid geometry", nameof(mesh));
            if (mesh.Mesh.PrecisionPositions == null || mesh.Mesh.PrecisionPositions.Count == 0)
                throw new ArgumentException("Mesh must have PrecisionPositions for edge chamfering", nameof(mesh));
            if (edgeNamesToChamfer == null || edgeNamesToChamfer.Count == 0)
                throw new ArgumentException("No edges specified for chamfering", nameof(edgeNamesToChamfer));
            if (chamferDistance <= 0)
                throw new ArgumentException("Chamfer distance must be positive", nameof(chamferDistance));
            if (cc.IntegerBoundingBox.Max.X == 0)
                throw new ArgumentException("Invalid coordinate converter", nameof(cc));

            return _pipeline.Run(
                mesh,
                edgeNamesToChamfer,
                new ChamferProfile(chamferDistance),
                cc,
                maxDiscretizationDeviation,
                ref groupIdOffset,
                resultNameSuffix: "_chamfered");
        }
    }
}
