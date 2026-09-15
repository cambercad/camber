using GeoCore;
using NURBS;

namespace Geo.BRep
{
    public sealed class BRepVertex
    {
        public Vec3D Position;
        public int Id;
    }

    public sealed class BRepEdge
    {
        public BSplineCurve Curve;
        public int StartVertexId;
        public int EndVertexId;
        public int Id;
        public bool Reversed;
    }

    public sealed class BRepTrimLoop
    {
        public List<Vec2D> UvPoints = new();
        public List<Vec3D> WorldPoints = new();
        public List<int> EdgeIds = new();
        /// <summary>WorldPoints spanned by each EdgeIds entry, including both endpoints.</summary>
        public List<int> EdgePointCounts = new();
        public bool IsOuter = true;
    }

    public sealed class BRepFace
    {
        public string PatchName;
        public INurbsSurface Surface;
        public BSplineSurface TessellationSurface;
        public ParametricRange? ParamRange;
        public SurfaceType SurfaceType;
        public PlaneSurfaceParams PlaneParams;
        public CylinderSurfaceParams CylinderParams;
        public ConeSurfaceParams ConeParams;
        public SphereSurfaceParams SphereParams;
        public TorusSurfaceParams TorusParams;
        public List<BRepTrimLoop> Loops = new();
        public int Id;
        public bool Exportable => Surface != null || TessellationSurface != null;
        public bool UsesMeshFallback;
        public bool UsesFittedSurface;
    }

    public sealed class BRepShell
    {
        public List<BRepFace> Faces = new();
    }

    public sealed class BRepSolid
    {
        public string Name;
        public bool IsVolume = true;
        public BRepShell Shell = new();
        public List<BRepVertex> Vertices = new();
        public List<BRepEdge> Edges = new();
    }
}
