using GeoCore;

namespace Geo.NurbsConstruction
{
    /// <summary>
    /// Builds the shared rail arc-length mapper on first use (for sweep metadata factories).
    /// </summary>
    internal sealed class LazySharedRailMapper
    {
        private readonly IReadOnlyList<CoordinateSystem> _frames;
        private ArcLengthMapper _mapper;

        public LazySharedRailMapper(IReadOnlyList<CoordinateSystem> frames)
        {
            _frames = frames;
        }

        public ArcLengthMapper GetOrCreate()
        {
            if (_mapper != null)
                return _mapper;
            var rail = NurbsSurfaceFactory.BuildRailCurveFromFrames(_frames);
            _mapper = NurbsSurfaceFactory.CreateArcLengthMapperIfNeeded(rail);
            return _mapper;
        }
    }
}
