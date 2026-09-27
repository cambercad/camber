using System.Collections.Generic;
using Curves;
using DotWrap;
using Geo;
using GeoCore;

namespace GeoPy;

/// <summary>World pose: origin + orthonormal right-handed axes (GeoAPI CoordinateSystem).</summary>
[DotWrapExpose]
public class NativeFrame
{
    internal CoordinateSystem Native;

    public NativeFrame(
        double ox, double oy, double oz,
        double xx, double xy, double xz,
        double yx, double yy, double yz,
        double zx, double zy, double zz)
    {
        Native = new CoordinateSystem(
            NativeUtil.V3(ox, oy, oz),
            NativeUtil.V3(xx, xy, xz),
            NativeUtil.V3(yx, yy, yz),
            NativeUtil.V3(zx, zy, zz));
    }

    internal NativeFrame(CoordinateSystem cs)
    {
        Native = cs;
    }

    public NativeFrame Offset(double dx, double dy, double dz)
    {
        return new NativeFrame(Native.GetOffsetCS(NativeUtil.V3(dx, dy, dz)));
    }

    public NativeFrame ToLocal(NativeFrame other)
    {
        return new NativeFrame(Native.ToLocal(other.Native));
    }

    public NativeFrame ToGlobal(NativeFrame local)
    {
        return new NativeFrame(Native.ToGlobal(local.Native));
    }

    public double Ox { get { return Native.Origin.X; } }
    public double Oy { get { return Native.Origin.Y; } }
    public double Oz { get { return Native.Origin.Z; } }
    public double Xx { get { return Native.X.X; } }
    public double Xy { get { return Native.X.Y; } }
    public double Xz { get { return Native.X.Z; } }
    public double Yx { get { return Native.Y.X; } }
    public double Yy { get { return Native.Y.Y; } }
    public double Yz { get { return Native.Y.Z; } }
    public double Zx { get { return Native.Z.X; } }
    public double Zy { get { return Native.Z.Y; } }
    public double Zz { get { return Native.Z.Z; } }
}

/// <summary>3D guide curve (Line3D, Helix3D, Circle3D, Arc3D, Spiral3D, …).</summary>
[DotWrapExpose]
public class NativeCurve
{
    internal readonly Curve3D Native;

    internal NativeCurve(Curve3D native)
    {
        Native = native;
    }

    public string Name { get { return Native.Name; } }

    public static NativeCurve Line(double x0, double y0, double z0, double x1, double y1, double z1, string name)
    {
        return new NativeCurve(new Line3D(NativeUtil.V3(x0, y0, z0), NativeUtil.V3(x1, y1, z1), NativeUtil.EmptyToNull(name)));
    }

    public static NativeCurve Linear(double x0, double y0, double z0,
        double x1, double y1, double z1, double ux, double uy, double uz, string name)
    {
        Vec3D start = NativeUtil.V3(x0, y0, z0);
        Vec3D end = NativeUtil.V3(x1, y1, z1);
        Vec3D direction = end - start;
        Vec3D up = NativeUtil.V3(ux, uy, uz);
        if (!double.IsFinite(direction.LengthSquared()) || direction.LengthSquared() <= 1e-24)
            throw new ArgumentException("A 3D line needs distinct finite endpoints.");
        if (!double.IsFinite(up.LengthSquared()) || up.LengthSquared() <= 1e-24)
            throw new ArgumentException("The line up direction must be finite and nonzero.");
        direction = direction.Normalized();
        up -= Vec3DOps.Dot(up, direction) * direction;
        if (up.LengthSquared() <= 1e-24)
            throw new ArgumentException("The line up direction cannot be parallel to the line.");
        return new NativeCurve(new LinearCurve3D(start, end, up.Normalized(), NativeUtil.EmptyToNull(name)));
    }

    public static NativeCurve Hermite(string packedPoints, string packedDirections, string name)
    {
        var points = NativePack.ReadPoints3(packedPoints);
        var directions = NativePack.ReadPoints3(packedDirections);
        if (points.Count < 2 || directions.Count != points.Count)
            throw new ArgumentException("Hermite needs at least two points and one tangent direction per point.");
        static bool Finite(Vec3D p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z);
        for (int i = 0; i < points.Count; i++)
        {
            if (!Finite(points[i]) || !Finite(directions[i]))
                throw new ArgumentException("Hermite points and tangent directions must be finite.");
            double magnitude = directions[i].Length();
            if (!double.IsFinite(magnitude) || magnitude < 1e-18)
                throw new ArgumentException("Hermite tangent directions must have finite nonzero length.");
            if (i > 0 && ((points[i] - points[i - 1]).LengthSquared() == 0 ||
                          !double.IsFinite((points[i] - points[i - 1]).Length())))
                throw new ArgumentException("Consecutive Hermite points must be distinct with finite separation.");
        }
        return new NativeCurve(new CubicHermiteSpline3D(points,
            directions.Select(v => (Vec3D?)v).ToArray(), NativeUtil.EmptyToNull(name)));
    }

    public static NativeCurve Sampled(string packedPoints, string name)
    {
        var points = NativePack.ReadPoints3(packedPoints);
        if (points.Count < 2)
            throw new ArgumentException("A sampled 3D curve needs at least two points.");
        static bool Finite(Vec3D p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z);
        var vertices = new List<CurveVertex3D>(points.Count);
        for (int i = 0; i < points.Count; i++)
        {
            if (!Finite(points[i]))
                throw new ArgumentException("Sampled 3D curve points must be finite.");
            Vec3D delta = i == 0 ? points[1] - points[0]
                : i == points.Count - 1 ? points[i] - points[i - 1]
                : points[i + 1] - points[i - 1];
            if (delta.LengthSquared() <= 1e-24 || !double.IsFinite(delta.LengthSquared()))
                throw new ArgumentException("Sampled 3D curve needs distinct points and nonzero tangents.");
            Vec3D tangent = delta.Normalized();
            Vec3D up = new Vec3D(0, 0, 1) - Vec3DOps.Dot(new Vec3D(0, 0, 1), tangent) * tangent;
            if (up.LengthSquared() <= 1e-24)
            {
                Vec3D axis = new Vec3D(1, 0, 0);
                up = axis - Vec3DOps.Dot(axis, tangent) * tangent;
            }
            vertices.Add(new CurveVertex3D(points[i], tangent, up.Normalized(), i / (double)(points.Count - 1)));
        }
        return new NativeCurve(new PolylineCurve3D(vertices, NativeUtil.EmptyToNull(name)));
    }

    public static NativeCurve TorusKnot(double p, double q, double radius, string name)
    {
        return new NativeCurve(new TorusKnot3D(p, q, radius, NativeUtil.EmptyToNull(name)));
    }

    private string EvaluateVector(double u, bool tangent)
    {
        if (!double.IsFinite(u) || u < 0 || u > 1)
            throw new ArgumentOutOfRangeException(nameof(u), "Curve parameter must lie in [0, 1].");
        var sample = Native.Evaluate(u);
        var value = tangent ? sample.Tangent : sample.Origin;
        return string.Create(System.Globalization.CultureInfo.InvariantCulture, $"{value.X:R} {value.Y:R} {value.Z:R}");
    }

    public string Point(double u) => EvaluateVector(u, false);
    public string Tangent(double u) => EvaluateVector(u, true);
    public string Tessellate(double maxDeviation)
    {
        var vertices = Native.Tessellate(maxDeviation);
        var points = new List<Vec3D>(vertices.Count);
        foreach (var vertex in vertices) points.Add(vertex.Origin);
        return NativePack.WritePoints3(points);
    }

    public static NativeCurve Helix(
        double ox, double oy, double oz,
        double xx, double xy, double xz,
        double yx, double yy, double yz,
        double radius, double pitch, double turns, int rightHanded, string name)
    {
        return new NativeCurve(new Helix3D(
            NativeUtil.V3(ox, oy, oz),
            NativeUtil.V3(xx, xy, xz),
            NativeUtil.V3(yx, yy, yz),
            radius,
            pitch,
            turns,
            rightHanded != 0,
            NativeUtil.EmptyToNull(name)));
    }

    public static NativeCurve Circle(
        double ox, double oy, double oz,
        double xx, double xy, double xz,
        double yx, double yy, double yz,
        double radius, string name)
    {
        return new NativeCurve(new Circle3D(
            NativeUtil.V3(ox, oy, oz),
            NativeUtil.V3(xx, xy, xz),
            NativeUtil.V3(yx, yy, yz),
            radius,
            NativeUtil.EmptyToNull(name)));
    }

    public static NativeCurve Arc(
        double ox, double oy, double oz,
        double xx, double xy, double xz,
        double yx, double yy, double yz,
        double radius, double startAngle, double sweepAngle, string name)
    {
        return new NativeCurve(new Arc3D(
            NativeUtil.V3(ox, oy, oz),
            NativeUtil.V3(xx, xy, xz),
            NativeUtil.V3(yx, yy, yz),
            radius,
            startAngle,
            sweepAngle,
            NativeUtil.EmptyToNull(name)));
    }

    public static NativeCurve Spiral(
        double ox, double oy, double oz,
        double xx, double xy, double xz,
        double yx, double yy, double yz,
        double startRadius, double endRadius, double zAdvancementPerRevolution, double turns, int rightHanded, string name)
    {
        return new NativeCurve(new Spiral3D(
            NativeUtil.V3(ox, oy, oz),
            NativeUtil.V3(xx, xy, xz),
            NativeUtil.V3(yx, yy, yz),
            startRadius,
            endRadius,
            zAdvancementPerRevolution,
            turns,
            rightHanded != 0,
            NativeUtil.EmptyToNull(name)));
    }
}

[DotWrapExpose]
public class NativeLoftOptions
{
    public int Style { get; set; }
    public int ProfileSamplesU { get; set; }
    public int VSubdivisionsPerSpan { get; set; }
    public int CapEnds { get; set; }
    public int AllowOpenContour { get; set; }
    public double LoftUmergeToleranceAbs { get; set; }
    public double LoftUmergeToleranceRel { get; set; }
    public int MaxMergedUAnchors { get; set; }
    public int CorrespondenceMode { get; set; }
    public int CreasePolicy { get; set; }
    public int AlignmentMode { get; set; }
    public int CapTriangulation { get; set; }
    public int ProfileSampling { get; set; }

    public NativeLoftOptions()
    {
        LoftOptions d = LoftOptions.Default;
        Style = (int)d.Style;
        ProfileSamplesU = d.ProfileSamplesU;
        VSubdivisionsPerSpan = d.VSubdivisionsPerSpan;
        CapEnds = d.CapEnds ? 1 : 0;
        AllowOpenContour = d.AllowOpenContour ? 1 : 0;
        LoftUmergeToleranceAbs = d.LoftUmergeToleranceAbs;
        LoftUmergeToleranceRel = d.LoftUmergeToleranceRel;
        MaxMergedUAnchors = d.MaxMergedUAnchors;
        CorrespondenceMode = (int)d.CorrespondenceMode;
        CreasePolicy = (int)d.CreasePolicy;
        AlignmentMode = (int)d.AlignmentMode;
        CapTriangulation = (int)d.CapTriangulation;
        ProfileSampling = (int)d.ProfileSampling;
    }

    internal LoftOptions ToLoftOptions()
    {
        return new LoftOptions
        {
            Style = (LoftStyle)Style,
            ProfileSamplesU = ProfileSamplesU,
            VSubdivisionsPerSpan = VSubdivisionsPerSpan,
            CapEnds = CapEnds != 0,
            AllowOpenContour = AllowOpenContour != 0,
            LoftUmergeToleranceAbs = LoftUmergeToleranceAbs,
            LoftUmergeToleranceRel = LoftUmergeToleranceRel,
            MaxMergedUAnchors = MaxMergedUAnchors,
            CorrespondenceMode = (LoftCorrespondenceMode)CorrespondenceMode,
            CreasePolicy = (LoftCreasePolicy)CreasePolicy,
            AlignmentMode = (LoftAlignmentMode)AlignmentMode,
            CapTriangulation = (LoftCapTriangulationMode)CapTriangulation,
            ProfileSampling = (LoftProfileSamplingSource)ProfileSampling
        };
    }
}

[DotWrapExpose]
public class NativeSketchList
{
    internal readonly List<PlotterSketcherCoordSys> Items = new List<PlotterSketcherCoordSys>();

    public void Add(NativeSketch sketch)
    {
        Items.Add(sketch.Native);
    }

    public int Count { get { return Items.Count; } }
}

[DotWrapExpose]
public class NativeCurveList
{
    internal readonly List<Curve3D> Items = new List<Curve3D>();

    public void Add(NativeCurve curve)
    {
        Items.Add(curve.Native);
    }

    public int Count { get { return Items.Count; } }
    public NativeCurve Get(int index) => new NativeCurve(Items[index]);
}

[DotWrapExpose]
public class NativeSolidList
{
    internal readonly List<AnchorMesh> Items = new List<AnchorMesh>();
    private readonly GeoAPI _ownerApi;

    public NativeSolidList() { }
    internal NativeSolidList(GeoAPI ownerApi = null) { _ownerApi = ownerApi; }

    public void Add(NativeSolid solid)
    {
        Items.Add(solid.Native);
    }

    public NativeSolid Get(int index) => new NativeSolid(Items[index], _ownerApi);

    public int Count { get { return Items.Count; } }
}

[DotWrapExpose]
public class NativeBooleanChain
{
    internal readonly List<BooleanOpChainNode> Items = new List<BooleanOpChainNode>();

    public void Add(NativeSolid meshB, int operation)
    {
        Items.Add(new BooleanOpChainNode
        {
            MeshB = meshB.Native,
            Operation = NativeUtil.ToBooleanOp(operation)
        });
    }

    public int Count { get { return Items.Count; } }
}
