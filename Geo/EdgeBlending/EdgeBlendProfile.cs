using Geo.NurbsConstruction;
using GeoCore;
using GeoMeta;

namespace Geo
{
    public interface IEdgeBlendProfile
    {
        double OffsetDistance { get; }

        void BuildStripSurface(BlendEdge edge, CoordinateConverter cc, double maxDeviation);

        UVSurface BuildCornerPatch(
            List<Vec3D> borderLoop,
            List<Rat3Hybrid> exactBoundary,
            List<int> cornerIndices,
            CoordinateConverter cc,
            EdgeBlendType blendType,
            double maxDeviation);

        void AttachPatchNurbs(
            Dictionary<string, SurfaceMetaData> meta,
            IReadOnlyList<BlendEdge> blendEdges,
            AnchorMesh probeMesh);

        string EdgePatchName(string sourceEdgeName);
        string CornerPatchName(int index);
    }

    public sealed class FilletProfile : IEdgeBlendProfile
    {
        public double OffsetDistance { get; }

        public FilletProfile(double filletRadius) => OffsetDistance = filletRadius;

        public void BuildStripSurface(BlendEdge edge, CoordinateConverter cc, double maxDeviation) =>
            edge.BuildFilletStripSurface(cc, maxDeviation);

        public UVSurface BuildCornerPatch(
            List<Vec3D> borderLoop,
            List<Rat3Hybrid> exactBoundary,
            List<int> cornerIndices,
            CoordinateConverter cc,
            EdgeBlendType blendType,
            double maxDeviation) =>
            BlendCorner.TessellateSphereCap(borderLoop, maxDeviation, exactBoundary, cornerIndices, cc, blendType);

        public void AttachPatchNurbs(
            Dictionary<string, SurfaceMetaData> meta,
            IReadOnlyList<BlendEdge> blendEdges,
            AnchorMesh probeMesh) =>
            BlendNurbsMetadataBuilder.AttachBlendPatchNurbs(meta, blendEdges, probeMesh);

        public string EdgePatchName(string sourceEdgeName) => EntityNaming.BlendEdge(sourceEdgeName);
        public string CornerPatchName(int index) => EntityNaming.BlendCorner(index);
    }

    public sealed class ChamferProfile : IEdgeBlendProfile
    {
        public double OffsetDistance { get; }

        public ChamferProfile(double chamferDistance) => OffsetDistance = chamferDistance;

        public void BuildStripSurface(BlendEdge edge, CoordinateConverter cc, double maxDeviation) =>
            edge.BuildChamferStripSurface(cc, maxDeviation);

        public UVSurface BuildCornerPatch(
            List<Vec3D> borderLoop,
            List<Rat3Hybrid> exactBoundary,
            List<int> cornerIndices,
            CoordinateConverter cc,
            EdgeBlendType blendType,
            double maxDeviation) =>
            BlendCorner.TessellatePlanarCap(borderLoop, exactBoundary, cornerIndices, cc, blendType);

        public void AttachPatchNurbs(
            Dictionary<string, SurfaceMetaData> meta,
            IReadOnlyList<BlendEdge> blendEdges,
            AnchorMesh probeMesh) =>
            BlendNurbsMetadataBuilder.AttachChamferPatchNurbs(meta, blendEdges, probeMesh);

        public string EdgePatchName(string sourceEdgeName) => EntityNaming.ChamferEdge(sourceEdgeName);
        public string CornerPatchName(int index) => EntityNaming.ChamferCorner(index);
    }
}
