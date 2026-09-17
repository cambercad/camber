using CSG;
using Curves;
using Geo.NurbsConstruction;
using GeoCore;
using NURBS;
using GeoMeta;

namespace Geo;

public partial class GeoAPI
{
    /// <summary>
    /// Loft an uncapped NURBS sheet through connected sketch sections. Guides and end
    /// derivatives are optional. Sections use authored curve order, without automatic reversal.
    /// </summary>
    public AnchorMesh LoftSurface(IReadOnlyList<PlotterSketcherCoordSys> sections,
        IReadOnlyList<Curve3D> guides = null, Vec3D? startTangent = null, Vec3D? endTangent = null,
        string name = null, double maxDeviation = -1)
    {
        if (sections == null || sections.Count < 2)
            throw new ArgumentException("Surface loft requires at least two sketches.", nameof(sections));
        var profiles = new List<BSplineCurve>();
        foreach (var sketch in sections)
        {
            if (sketch == null) throw new ArgumentException("A section cannot be null.", nameof(sections));
            var strips = sketch.GetCurves().Select(s => s.Where(c => !c.IsHelperGeometry).ToList())
                .Where(s => s.Count > 0).ToList();
            if (strips.Count != 1)
                throw new ArgumentException("Each section must contain one connected curve strip.", nameof(sections));
            var curves = strips[0];
            for (int i = 0; i < curves.Count; i++)
            {
                if (curves[i] is not (Line2D or Arc2D or Circle2D or CubicHermiteSpline2D or Bezier2D or Ellipse2D or BSpline2D))
                    throw new ArgumentException("This section curve has no exact NURBS conversion.", nameof(sections));
                if (i > 0 && curves[i - 1].EndPosition != curves[i].StartPosition)
                    throw new ArgumentException("Section curves must connect in authored order.", nameof(sections));
            }
            profiles.Add(NurbsSurfaceFactory.ConcatenateCompatible(curves
                .Select(c => Curve2DToBSpline.ToBSplineCurve(c, sketch.CoordinateSystem)).ToList()));
        }
        var surface = SurfaceLoft.Build(profiles, guides, startTangent, endTangent);
        var tess = AdaptiveSurfaceSplitter.TriangulateAdaptive(surface,
            maxDeviation: ResolveMaxDeviation(maxDeviation));
        var precise = tess.Points.Select(p => converter.Convert(p)).Select(p => new Rat3Hybrid(p.X, p.Y, p.Z)).ToList();
        var positions = precise.Select(p => converter.Convert(p)).ToList();
        int gid = ReserveGroupIds(1);
        name ??= GenerateName("SurfaceLoft");
        string patch = EntityNaming.LoftSide(name);
        var mesh = new MeshNormalUV(converter, positions, tess.Normals, tess.UV, tess.Triangles,
            Enumerable.Repeat(gid, tess.Triangles.Count).ToList(), precise, skipWatertightCheck: true);
        var result = new AnchorMesh(name, mesh, new Dictionary<int, string> { [gid] = patch },
            new Dictionary<string, SurfaceMetaData> { [patch] = new(SurfaceType.Unknown, surface, ParametricRange.UnitSquare) }, false);
        RegisterMesh(result);
        return result;
    }
}
