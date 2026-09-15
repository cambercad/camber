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

    public static NativeLoftOptions PropellerBlade()
    {
        LoftOptions d = LoftOptions.PropellerBlade;
        NativeLoftOptions o = new NativeLoftOptions();
        o.Style = (int)d.Style;
        o.ProfileSamplesU = d.ProfileSamplesU;
        o.VSubdivisionsPerSpan = d.VSubdivisionsPerSpan;
        o.CapEnds = d.CapEnds ? 1 : 0;
        o.AllowOpenContour = d.AllowOpenContour ? 1 : 0;
        o.LoftUmergeToleranceAbs = d.LoftUmergeToleranceAbs;
        o.LoftUmergeToleranceRel = d.LoftUmergeToleranceRel;
        o.MaxMergedUAnchors = d.MaxMergedUAnchors;
        o.CorrespondenceMode = (int)d.CorrespondenceMode;
        o.CreasePolicy = (int)d.CreasePolicy;
        o.AlignmentMode = (int)d.AlignmentMode;
        o.CapTriangulation = (int)d.CapTriangulation;
        o.ProfileSampling = (int)d.ProfileSampling;
        return o;
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
}

[DotWrapExpose]
public class NativeSolidList
{
    internal readonly List<AnchorMesh> Items = new List<AnchorMesh>();

    public void Add(NativeSolid solid)
    {
        Items.Add(solid.Native);
    }

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
