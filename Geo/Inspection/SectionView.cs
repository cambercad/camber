using CSG;
using GeoCore;

namespace Geo;

/// <summary>A capped, unregistered snapshot retaining the negative Z side of a plane.</summary>
public sealed class SectionView
{
    public IReadOnlyList<AnchorMesh> Meshes { get; }
    public CoordinateSystem Plane { get; }
    readonly CoordinateConverter converter;

    public SectionView(IEnumerable<AnchorMesh> solids, CoordinateConverter converter, CoordinateSystem plane)
        : this(solids.Select(s => (s, new Transform(new Vec3D(0), TransformMath.IdentityOrientation), s.Name)), converter, plane) { }

    internal SectionView(IEnumerable<(AnchorMesh Mesh, Transform Pose, string Name)> occurrences,
        CoordinateConverter converter, CoordinateSystem plane)
    {
        foreach (var vector in new[] { plane.Origin, plane.X, plane.Y, plane.Z })
            if (!double.IsFinite(vector.X) || !double.IsFinite(vector.Y) || !double.IsFinite(vector.Z))
                throw new ArgumentException("Section frame coordinates must be finite.", nameof(plane));
        this.converter = converter;
        Plane = new CoordinateSystem(plane.Origin, plane.X, plane.Y, plane.Z);
        var result = new List<AnchorMesh>();
        foreach (var (source, pose, name) in occurrences)
        {
            if (!source.IsVolume) throw new ArgumentException($"Section requires a solid: '{name}' is a surface.");
            var snapshot = source.SnapshotForInspection(converter, pose, name);
            try
            {
                var cut = Clip(snapshot, converter, Plane);
                if (cut != null) result.Add(cut);
            }
            catch (Exception error) { throw new InvalidOperationException($"Section failed for '{name}'.", error); }
        }
        Meshes = result.AsReadOnly();
    }

    /// <summary>Nearest exact-mesh ray hit in the retained snapshot.</summary>
    public RayMeshHit Raycast(Vec3D origin, Vec3D direction)
    {
        RayMeshHit nearest = null;
        double distance = double.PositiveInfinity;
        foreach (var mesh in Meshes)
            if (RayMeshExact.TryCast(mesh, converter, origin, direction, out var hit) && hit != null)
            {
                double squared = (hit.Point-origin).LengthSquared();
                if (squared < distance) { distance=squared; nearest=hit; }
            }
        return nearest;
    }

    static BigRationalHybrid Exact(double value)
    {
        var rational = new BigRational(value);
        return new BigRationalHybrid(rational.Numerator, rational.Denominator);
    }

    static AnchorMesh Clip(AnchorMesh source, CoordinateConverter converter, CoordinateSystem plane)
    {
        var unit = Exact(converter.SmallestUnit());
        var min = converter.OperatingSpace.Min;
        Rat3Hybrid Vector(Vec3D p) => new(Exact(p.X)/unit, Exact(p.Y)/unit, Exact(p.Z)/unit);
        var origin = new Rat3Hybrid((Exact(plane.Origin.X)-Exact(min.X))/unit,
            (Exact(plane.Origin.Y)-Exact(min.Y))/unit, (Exact(plane.Origin.Z)-Exact(min.Z))/unit);
        var x = Vector(plane.X); var y = Vector(plane.Y); var z = Vector(plane.Z);
        var normal = Rat3Hybrid.Cross(x,y);
        var indices = source.Mesh.Triangles.SelectMany(t => new[] {t.A,t.B,t.C}).Distinct().ToArray();
        bool negative = false, positive = false;
        foreach (var i in indices)
        {
            int sign = Rat3Hybrid.Dot(source.Mesh.PrecisionPositions[i]-origin, normal).Sign();
            negative |= sign < 0; positive |= sign > 0;
        }
        if (!negative) return null;
        if (!positive) return source;
        // Outer faces lie beyond the solid. Exact affine cap vertices avoid
        // independently rounded corners warping an arbitrary section plane.
        var points = indices.Select(i => converter.Convert(source.Mesh.PrecisionPositions[i])-plane.Origin).ToList();
        double xmin=points.Min(p=>p.Dot(plane.X)), xmax=points.Max(p=>p.Dot(plane.X));
        double ymin=points.Min(p=>p.Dot(plane.Y)), ymax=points.Max(p=>p.Dot(plane.Y));
        double zmin=points.Min(p=>p.Dot(plane.Z));
        double margin=Math.Max(1, Math.Max(xmax-xmin,Math.Max(ymax-ymin,Math.Abs(zmin)))*.1);
        xmin-=margin; xmax+=margin; ymin-=margin; ymax+=margin; zmin-=margin;
        Rat3Hybrid Point(double a,double b,double c) => origin+x*Exact(a)+y*Exact(b)+z*Exact(c);
        var vertices = new List<Rat3Hybrid> {
            Point(xmin,ymin,zmin),Point(xmax,ymin,zmin),Point(xmax,ymax,zmin),Point(xmin,ymax,zmin),
            Point(xmin,ymin,0),Point(xmax,ymin,0),Point(xmax,ymax,0),Point(xmin,ymax,0) };
        var triangles = new List<Tri> { new(0,2,1),new(0,3,2),new(4,5,6),new(4,6,7),
            new(0,1,5),new(0,5,4),new(1,2,6),new(1,6,5),new(2,3,7),new(2,7,6),new(3,0,4),new(3,4,7) };
        // Deferred cap splitting allocates further IDs during display/export.
        int cap = GeoAPI.ReserveGroupIds(1);
        var corners = new List<MeshTriangle<TriangleVertexNormalUV>>();
        foreach (var t in triangles)
        {
            var a=converter.Convert(vertices[t.A]); var b=converter.Convert(vertices[t.B]); var c=converter.Convert(vertices[t.C]);
            var data=new TriangleVertexNormalUV { Normal=(b-a).Cross(c-a).Normalized(),UV=new Vec2D(0) };
            corners.Add(new() { V0=data,V1=data,V2=data,GroupId=cap });
        }
        var cutter=new MeshNormalUV(converter,vertices,triangles,corners,Enumerable.Repeat(cap,12).ToList());
        var clipped=MeshNormalUV.BooleanOperation(source.Mesh,cutter,BooleanOp.Intersect,converter);
        if (clipped.Triangles.Count==0) return null;
        var names=new Dictionary<int,string>(source.groupIdToExtendedName);
        string capName="section_cap";
        while (names.ContainsValue(capName)) capName="_"+capName;
        names[cap]=capName;
        var metadata=SurfaceMetaData.CloneDictionary(source.surfaceMetaData);
        metadata[capName]=new SurfaceMetaData(SurfaceType.Planar) {
            PlaneParams=new PlaneSurfaceParams { Origin=plane.Origin,Normal=plane.Z,RefDir=plane.X } };
        return new AnchorMesh(source.Name,clipped,names,metadata,deferCoplanarPostProcess:true);
    }
}
