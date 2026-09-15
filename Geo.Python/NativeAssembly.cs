using System.Collections.Generic;
using DotWrap;
using Geo;
using GeoCore;
using GeoSolver;

namespace GeoPy;

[DotWrapExpose]
public class NativeAssembly
{
    readonly Assembly _inner;

    internal NativeAssembly(Assembly inner)
    {
        _inner = inner;
    }

    internal Assembly Native { get { return _inner; } }

    public string Name { get { return _inner.Name; } }

    public int SolveAfterEveryConstraint
    {
        get { return _inner.SolveAfterEveryConstraint ? 1 : 0; }
        set { _inner.SolveAfterEveryConstraint = value != 0; }
    }

    public NativeAssemblyPart AddPart(
        NativeSolid solid,
        double px,
        double py,
        double pz,
        double qx,
        double qy,
        double qz,
        double qw)
    {
        Quaternion orientation = new Quaternion(qx, qy, qz, qw);
        return new NativeAssemblyPart(_inner.AddPart(solid.Native, new Vec3D(px, py, pz), orientation));
    }

    public NativeAssemblyOccurrence AddSubAssembly(
        NativeAssembly child,
        double px,
        double py,
        double pz,
        double qx,
        double qy,
        double qz,
        double qw)
    {
        Quaternion orientation = new Quaternion(qx, qy, qz, qw);
        return new NativeAssemblyOccurrence(_inner.AddSubAssembly(child.Native, new Vec3D(px, py, pz), orientation));
    }

    public void FixPart(NativeAssemblyPart part)
    {
        _inner.FixPart(part.Native);
    }

    public void FixSubAssembly(NativeAssemblyOccurrence occurrence)
    {
        _inner.FixSubAssembly(occurrence.Native);
    }

    public int PartCount { get { return _inner.GetParts().Count; } }

    public NativeAssemblyPart GetPart(int index)
    {
        return new NativeAssemblyPart(_inner.GetParts()[index]);
    }

    public int SubAssemblyCount { get { return _inner.GetSubAssemblies().Count; } }

    public NativeAssemblyOccurrence GetSubAssembly(int index)
    {
        return new NativeAssemblyOccurrence(_inner.GetSubAssemblies()[index]);
    }

    public double GetWorldPoseX(NativeAssemblyPart part)
    {
        return _inner.WorldPoseOf(part.Native).Position.X;
    }

    public double GetWorldPoseY(NativeAssemblyPart part)
    {
        return _inner.WorldPoseOf(part.Native).Position.Y;
    }

    public double GetWorldPoseZ(NativeAssemblyPart part)
    {
        return _inner.WorldPoseOf(part.Native).Position.Z;
    }

    public void SetCoincidentPoints(NativeAssemblyPointDatum a, NativeAssemblyPointDatum b)
    {
        _inner.SetCoincident(a.Native, b.Native);
    }

    public void SetCoincidentAxes(NativeAssemblyAxisDatum a, NativeAssemblyAxisDatum b)
    {
        _inner.SetCoincident(a.Native, b.Native);
    }

    public void SetCoincidentPlanes(NativeAssemblyPlaneDatum a, NativeAssemblyPlaneDatum b)
    {
        _inner.SetCoincident(a.Native, b.Native);
    }

    public void SetCoincidentPlanesOriented(
        NativeAssemblyPlaneDatum a,
        NativeAssemblyPlaneDatum b,
        int oppositeNormals)
    {
        _inner.SetCoincidentOriented(a.Native, b.Native, oppositeNormals != 0);
    }

    public void SetParallelAxes(NativeAssemblyAxisDatum a, NativeAssemblyAxisDatum b)
    {
        _inner.SetParallel(a.Native, b.Native);
    }

    public void SetConcentric(NativeAssemblyAxisDatum a, NativeAssemblyAxisDatum b)
    {
        _inner.SetConcentric(a.Native, b.Native);
    }

    public void SetPerpendicularAxes(NativeAssemblyAxisDatum a, NativeAssemblyAxisDatum b)
    {
        _inner.SetPerpendicular(a.Native, b.Native);
    }

    public void SetAngleAxes(NativeAssemblyAxisDatum a, NativeAssemblyAxisDatum b, double angleRadians)
    {
        _inner.SetAngle(a.Native, b.Native, angleRadians);
    }

    public void SetDistancePoints(NativeAssemblyPointDatum a, NativeAssemblyPointDatum b, double distance)
    {
        _inner.SetDistance(a.Native, b.Native, distance);
    }

    public void SetDistancePlanes(NativeAssemblyPlaneDatum a, NativeAssemblyPlaneDatum b, double distance)
    {
        _inner.SetDistance(a.Native, b.Native, distance);
    }

    public string SolveConstraints()
    {
        SolveResult result = _inner.SolveConstraintsDetailed();
        return string.Format(
            System.Globalization.CultureInfo.InvariantCulture,
            "converged={0} sse={1:G6} n={2} m={3} msg={4}",
            result.Converged,
            result.SumOfSquaredErrors,
            result.NumParameters,
            result.NumEquations,
            result.Message ?? "");
    }

    public string DumpDisplay()
    {
        return DisplayPack.PackAssembly(_inner);
    }

    public NativeFrame GetPlaneFrame(string reference)
    {
        if (string.IsNullOrWhiteSpace(reference))
            throw new System.ArgumentException("A planar surface reference is required.");

        var parts = new List<AssemblyPart>();
        var poses = new List<Transform>();
        _inner.CollectLeafWorldPoses(parts, poses);
        for (int i = 0; i < parts.Count; i++)
        {
            AssemblyPart part = parts[i];
            string prefix = (part.Mesh.Name ?? "") + ":";
            if (!reference.StartsWith(prefix, System.StringComparison.Ordinal))
                continue;

            string localReference = reference.Substring(prefix.Length);
            if (!part.Mesh.TryGetPlaneFromPatch(localReference, out PlaneSurfaceParams plane))
                throw new System.ArgumentException($"'{reference}' does not resolve to a planar surface.");

            Transform pose = poses[i];
            Vec3D localOrigin = plane.Origin;
            if (part.Mesh.TryGetSurface(localReference, out UVSurface surface) && surface.IsPlanar())
                localOrigin = surface.ApproximatePlanarSurfaceCenter(out _, out _, out _);
            Vec3D origin = TransformMath.TransformPoint(in pose, localOrigin);
            Vec3D z = TransformMath.TransformDirection(in pose, plane.Normal);
            Vec3D x = TransformMath.TransformDirection(in pose, plane.RefDir);
            z.Normalize();
            x -= Vec3DOps.Dot(x, z) * z;
            if (x.LengthSquared() < 1e-12)
                x = Vec3DOps.GetOrthoNormal(z);
            x.Normalize();
            Vec3D y = Vec3DOps.Cross(z, x);
            y.Normalize();
            x = Vec3DOps.Cross(y, z);
            x.Normalize();
            return new NativeFrame(new CoordinateSystem(origin, x, y, z));
        }
        throw new System.ArgumentException($"'{reference}' does not resolve to an assembly surface.");
    }

    public string DumpConstraints()
    {
        IReadOnlyList<AssemblyMateRecord> mates = _inner.GetMateRecords();
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < mates.Count; i++)
        {
            AssemblyMateRecord mate = mates[i];
            sb.Append("C\t");
            sb.Append(mate.Kind.ToString());
            sb.Append('\t');
            sb.Append(ToAsciiDump(mate.Label));
            sb.Append('\n');
            IReadOnlyList<string> entities = mate.Entities;
            for (int e = 0; e < entities.Count; e++)
            {
                string name = entities[e];
                if (string.IsNullOrEmpty(name))
                    continue;
                sb.Append("E\t");
                sb.Append(ToAsciiDump(name));
                sb.Append('\n');
            }
            AppendMateGlyphs(sb, _inner, mate);
        }
        return sb.ToString();
    }

    private static void AppendMateGlyphs(System.Text.StringBuilder sb, Assembly assembly, AssemblyMateRecord mate)
    {
        switch (mate.Kind)
        {
            case AssemblyMateKind.FixPart:
                AppendGlyph(sb, "point", MateWorldPoint(assembly, mate.PartA, new Vec3D(0)), new Vec3D(0, 0, 1));
                break;
            case AssemblyMateKind.CoincidentPoints:
            case AssemblyMateKind.DistancePoints:
                AppendGlyph(sb, "point", MateWorldPoint(assembly, mate.PartA, mate.LocalA), default);
                AppendGlyph(sb, "point", MateWorldPoint(assembly, mate.PartB, mate.LocalB), default);
                break;
            case AssemblyMateKind.CoincidentAxes:
            case AssemblyMateKind.ParallelAxes:
            case AssemblyMateKind.PerpendicularAxes:
            case AssemblyMateKind.Concentric:
            case AssemblyMateKind.AngleAxes:
                AppendGlyph(sb, "axis", MateWorldPoint(assembly, mate.PartA, mate.LocalA), MateWorldDir(assembly, mate.PartA, mate.DirA));
                AppendGlyph(sb, "axis", MateWorldPoint(assembly, mate.PartB, mate.LocalB), MateWorldDir(assembly, mate.PartB, mate.DirB));
                break;
            case AssemblyMateKind.CoincidentPlanes:
            case AssemblyMateKind.ParallelPlanes:
            case AssemblyMateKind.PerpendicularPlanes:
            case AssemblyMateKind.DistancePlanes:
                AppendGlyph(sb, "plane", MateWorldPoint(assembly, mate.PartA, mate.LocalA), MateWorldDir(assembly, mate.PartA, mate.DirA));
                AppendGlyph(sb, "plane", MateWorldPoint(assembly, mate.PartB, mate.LocalB), MateWorldDir(assembly, mate.PartB, mate.DirB));
                break;
            case AssemblyMateKind.PointOnPlane:
            case AssemblyMateKind.Contact:
                AppendGlyph(sb, "point", MateWorldPoint(assembly, mate.PartA, mate.LocalA), default);
                AppendGlyph(sb, "plane", MateWorldPoint(assembly, mate.PartB, mate.LocalB), MateWorldDir(assembly, mate.PartB, mate.DirB));
                break;
        }
    }

    private static Vec3D MateWorldPoint(Assembly assembly, AssemblyPart part, Vec3D local)
    {
        if (part == null)
            return local;
        Transform pose = assembly.WorldPoseOf(part);
        return TransformMath.TransformPoint(in pose, local);
    }

    private static Vec3D MateWorldDir(Assembly assembly, AssemblyPart part, Vec3D local)
    {
        Vec3D dir = local;
        if (dir.LengthSquared() < 1e-20)
            dir = new Vec3D(0, 0, 1);
        if (part == null)
            return dir;
        Transform pose = assembly.WorldPoseOf(part);
        Vec3D world = TransformMath.TransformDirection(in pose, dir);
        if (world.LengthSquared() < 1e-20)
            return new Vec3D(0, 0, 1);
        world.Normalize();
        return world;
    }

    private static void AppendGlyph(System.Text.StringBuilder sb, string kind, Vec3D origin, Vec3D direction)
    {
        sb.Append("G\t");
        sb.Append(kind);
        sb.Append('\t');
        AppendDumpVec(sb, origin);
        sb.Append('\t');
        AppendDumpVec(sb, direction);
        sb.Append('\n');
    }

    private static void AppendDumpVec(System.Text.StringBuilder sb, Vec3D v)
    {
        var inv = System.Globalization.CultureInfo.InvariantCulture;
        sb.Append(v.X.ToString("G9", inv));
        sb.Append('\t');
        sb.Append(v.Y.ToString("G9", inv));
        sb.Append('\t');
        sb.Append(v.Z.ToString("G9", inv));
    }

    // DotWrap marshals C# strings as the Windows ANSI code page. Python then
    // decodes them as UTF-8, so a degree sign (U+00B0 → 0xB0) crashes show().
    private static string ToAsciiDump(string s)
    {
        if (string.IsNullOrEmpty(s))
            return "";
        var outSb = new System.Text.StringBuilder(s.Length);
        for (int i = 0; i < s.Length; i++)
        {
            char c = s[i];
            if (c <= 0x7F)
                outSb.Append(c);
            else if (c == '°')
                outSb.Append(" deg");
            else
                outSb.Append('?');
        }
        return outSb.ToString();
    }
}

[DotWrapExpose]
public class NativeAssemblyOccurrence
{
    internal readonly AssemblyOccurrence Native;

    internal NativeAssemblyOccurrence(AssemblyOccurrence native)
    {
        Native = native;
    }

    public string Name { get { return Native.Name; } }

    public double PoseX { get { return Native.EvaluatePose().Position.X; } }
    public double PoseY { get { return Native.EvaluatePose().Position.Y; } }
    public double PoseZ { get { return Native.EvaluatePose().Position.Z; } }

    public NativeAssembly GetChild()
    {
        return new NativeAssembly(Native.Child);
    }

    public int PartCount { get { return Native.GetParts().Count; } }

    public NativeAssemblyPart GetPart(int index)
    {
        return new NativeAssemblyPart(Native.GetParts()[index]);
    }

    public int SubAssemblyCount { get { return Native.GetSubAssemblies().Count; } }

    public NativeAssemblyOccurrence GetSubAssembly(int index)
    {
        return new NativeAssemblyOccurrence(Native.GetSubAssemblies()[index]);
    }
}

[DotWrapExpose]
public class NativeAssemblyPart
{
    internal readonly AssemblyPart Native;

    internal NativeAssemblyPart(AssemblyPart native)
    {
        Native = native;
    }

    public string Name { get { return Native.Mesh.Name; } }

    public double PoseX { get { return Native.EvaluatePose().Position.X; } }
    public double PoseY { get { return Native.EvaluatePose().Position.Y; } }
    public double PoseZ { get { return Native.EvaluatePose().Position.Z; } }

    public NativeAssemblyAxisDatum AddAxisDatum(string reference)
    {
        return new NativeAssemblyAxisDatum(Native.AddAxisDatum(reference));
    }

    public NativeAssemblyPointDatum AddPointDatum(string reference)
    {
        return new NativeAssemblyPointDatum(Native.AddPointDatum(reference));
    }

    public NativeAssemblyPlaneDatum AddPlaneDatum(string reference)
    {
        return new NativeAssemblyPlaneDatum(Native.AddPlaneDatum(reference));
    }

    public NativeAssemblyAxisDatum AddAxisDatumAt(
        double px, double py, double pz,
        double dx, double dy, double dz)
    {
        return new NativeAssemblyAxisDatum(
            Native.AddAxisDatumAt(new Vec3D(px, py, pz), new Vec3D(dx, dy, dz)));
    }

    public NativeAssemblyPointDatum AddPointDatumAt(double px, double py, double pz)
    {
        return new NativeAssemblyPointDatum(Native.AddPointDatumAt(new Vec3D(px, py, pz)));
    }

    public NativeAssemblyPlaneDatum AddPlaneDatumAt(
        double px, double py, double pz,
        double nx, double ny, double nz)
    {
        return new NativeAssemblyPlaneDatum(
            Native.AddPlaneDatumAt(new Vec3D(px, py, pz), new Vec3D(nx, ny, nz)));
    }
}

[DotWrapExpose]
public class NativeAssemblyPointDatum
{
    internal readonly AssemblyPointDatum Native;

    internal NativeAssemblyPointDatum(AssemblyPointDatum native)
    {
        Native = native;
    }

    public string Entity { get { return Native.Entity ?? ""; } }
}

[DotWrapExpose]
public class NativeAssemblyAxisDatum
{
    internal readonly AssemblyAxisDatum Native;

    internal NativeAssemblyAxisDatum(AssemblyAxisDatum native)
    {
        Native = native;
    }

    public string Entity { get { return Native.Entity ?? ""; } }
}

[DotWrapExpose]
public class NativeAssemblyPlaneDatum
{
    internal readonly AssemblyPlaneDatum Native;

    internal NativeAssemblyPlaneDatum(AssemblyPlaneDatum native)
    {
        Native = native;
    }

    public string Entity { get { return Native.Entity ?? ""; } }
}
