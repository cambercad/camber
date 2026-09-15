using Curves;
using GeoCore;

namespace Geo
{
    /// <summary>
    /// One projected sketch vertex: 2D source + mesh hit + surface frame.
    /// </summary>
    public class ProjectedVertex
    {
        public Vec2D SketchUv { get; set; }
        public Vec3D Hit { get; set; }
        /// <summary>Unit geometric triangle normal at the hit.</summary>
        public Vec3D SurfaceNormal { get; set; }
        public int TriangleIndex { get; set; }
        public int GroupId { get; set; }
    }

    /// <summary>
    /// Result of projecting a tessellated sketch onto a triangle mesh along the sketch-plane normal.
    /// </summary>
    public class ProjectedSketch
    {
        public string Name { get; set; }
        public PlotterSketcherCoordSys SourceSketch { get; set; }
        public AnchorMesh TargetMesh { get; set; }

        /// <summary>Named 3D polylines (one per sketch curve segment), registered on the GeoAPI.</summary>
        public List<PolylineCurve3D> Curves { get; set; } = new List<PolylineCurve3D>();

        /// <summary>2D tessellation strips (points), lockstep with <see cref="ContourNames"/>.</summary>
        public List<List<List<Vec2D>>> Contours2D { get; set; }

        public List<List<List<Vec2D>>> ContourNormals2D { get; set; }

        public List<List<string>> ContourNames { get; set; }

        /// <summary>
        /// Projected hits nested like Contours2D: strip → segment → vertex.
        /// </summary>
        public List<List<List<ProjectedVertex>>> Vertices { get; set; }
    }
}
