using GeoCore;
using NURBS;

namespace Geo.NurbsConstruction
{
    /// <summary>
    /// Wraps a <see cref="BSplineSurface"/> so mesh-stored UV coordinates evaluate correctly
    /// (e.g. arc-length along profile, affine cap UV).
    /// </summary>
    public sealed class MeshUvMappedSurface : INurbsSurface
    {
        public BSplineSurface Inner { get; }
        public ArcLengthMapper UMapper { get; }
        public ArcLengthMapper VMapper { get; }
        public ParametricRange Range { get; }
        /// <summary>Mesh stores profile along u and rail/extrude along v; <see cref="BSplineLinearExtrudeSurface"/> uses the opposite NURBS parameterization.</summary>
        public bool SwapMeshUv { get; }

        public MeshUvMappedSurface(
            BSplineSurface inner,
            ArcLengthMapper uMapper = null,
            ParametricRange? range = null,
            ArcLengthMapper vMapper = null,
            bool swapMeshUv = false)
        {
            Inner = inner;
            UMapper = uMapper;
            VMapper = vMapper;
            Range = range ?? ParametricRange.UnitSquare;
            SwapMeshUv = swapMeshUv;
        }

        public Vec2D MapMeshUv(double u, double v)
        {
            var ranged = Range.MapMeshUvToNurbs(new Vec2D(u, v));
            double mappedU = UMapper != null ? UMapper.KnotFromNormalizedArcLength(ranged.X) : ranged.X;
            double mappedV = VMapper != null ? VMapper.KnotFromNormalizedArcLength(ranged.Y) : ranged.Y;
            var nurbsUv = new Vec2D(mappedU, mappedV);
            if (SwapMeshUv)
                return new Vec2D(nurbsUv.Y, nurbsUv.X);
            return nurbsUv;
        }

        public Vec3D Evaluate(double u, double v)
        {
            var nurbsUv = MapMeshUv(u, v);
            return Inner.Evaluate(nurbsUv.X, nurbsUv.Y);
        }

        public Vec3D EvaluateNormal(double u, double v)
        {
            var nurbsUv = MapMeshUv(u, v);
            return Inner.EvaluateNormal(nurbsUv.X, nurbsUv.Y);
        }

        public static BSplineSurface ResolveTessellationTarget(INurbsSurface surface)
        {
            if (surface is TransformedNurbsSurface transformed)
                return NurbsSurfaceFactory.Transform(
                    ResolveTessellationTarget(transformed.Inner), transformed.Transform);
            if (surface is MeshUvMappedSurface mapped)
                return mapped.Inner;
            if (surface is CapUvMappedSurface cap)
                return cap.Plane;
            if (surface is BSplineSurface direct)
                return direct;
            throw new InvalidOperationException($"Cannot tessellate NURBS surface type {surface.GetType().Name}");
        }
    }
}
