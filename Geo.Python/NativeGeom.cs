using System.Globalization;
using System.Text;
using CSG;
using Curves;
using DotWrap;
using GeoCore;

namespace GeoPy;

/// <summary>Packed 2D geometry helpers for Python. Predicates use the part lattice (Rat2 from Rat3).</summary>
[DotWrapExpose]
public class NativeGeom
{
    public static string Triangulate(string packedLoops, string packedVolume)
    {
        var loops = NativePack.ReadLoops2(packedLoops);
        if (loops.Count == 0)
            return NativePack.WriteIndexed2(new List<Vec2D>(), new List<Tri>());

        var world = new List<Vec2D>();
        var polygons = new List<List<int>>();
        int index = 0;
        foreach (var loop in loops)
        {
            var indices = new List<int>(loop.Count);
            for (int k = 0; k < loop.Count; k++)
            {
                world.Add(loop[k]);
                indices.Add(index++);
            }
            polygons.Add(indices);
        }

        CoordinateConverter conv = NativeLattice.ConverterFor(world, packedVolume);
        var exact = NativeLattice.ToRat2(conv, world);
        var tris = TriangulationWithHoles<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>.Triangulate(exact, polygons);
        return NativePack.WriteIndexed2(world, tris);
    }

    public static double SignedArea(string packedLoop, string packedVolume)
    {
        var loop = NativePack.ReadFirstLoop2(packedLoop);
        if (loop == null || loop.Count < 3)
            return 0;
        CoordinateConverter conv = NativeLattice.ConverterFor(loop, packedVolume);
        var exact = NativeLattice.ToRat2(conv, loop);
        BigRationalHybrid twice = BigRationalHybrid.Zero;
        for (int i = 0; i < exact.Count; i++)
            twice += Rat2Hybrid.Cross(exact[i], exact[(i + 1) % exact.Count]);
        double unit = conv.SmallestUnit();
        return 0.5 * twice.ToDouble() * unit * unit;
    }

    public static int PointInPolygon(string packedLoop, double x, double y, string packedVolume)
    {
        var loop = NativePack.ReadFirstLoop2(packedLoop);
        if (loop.Count < 3)
            return (int)PointInPolygonResult.Outside;

        var pts = new List<Vec2D>(loop.Count + 1);
        pts.AddRange(loop);
        pts.Add(new Vec2D(x, y));
        CoordinateConverter conv = NativeLattice.ConverterFor(pts, packedVolume);

        var exact = NativeLattice.ToRat2(conv, loop);
        var indices = new List<int>(exact.Count);
        for (int i = 0; i < exact.Count; i++)
            indices.Add(i);
        Rat2Hybrid q;
        if (!NativeLattice.TryToRat2(conv, new Vec2D(x, y), out q))
            return (int)PointInPolygonResult.Outside;
        return (int)PolygonOps<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>.IsPointInPolygon(exact, indices, q);
    }

    public static string ConvexHull(string packedXy, string packedVolume)
    {
        var points = NativePack.ReadPoints2(packedXy);
        if (points.Count == 0)
            return NativePack.WriteLoops2(new List<List<Vec2D>>());
        CoordinateConverter conv = NativeLattice.ConverterFor(points, packedVolume);
        var exact = NativeLattice.ToRat2(conv, points);
        var hull = GiftWrapping<Rat2HybridArithmetic, Rat2Hybrid, BigRationalHybrid>.ConvexHull(exact);
        var loop = new List<Vec2D>(hull.Count);
        for (int i = 0; i < hull.Count; i++)
            loop.Add(points[hull[i]]);
        return NativePack.WriteLoops2(new List<List<Vec2D>> { loop });
    }

    public static string TessellateBezier(string packedControls, double maxDeviation)
    {
        var controls = NativePack.ReadPoints2(packedControls);
        if (controls.Count < 2)
            return NativePack.WriteLoops2(new List<List<Vec2D>> { controls });
        double tol = maxDeviation > 0 ? maxDeviation : 0.01;
        var pts = BezierTessellator.Tessellate(tol, controls);
        return NativePack.WriteLoops2(new List<List<Vec2D>> { pts });
    }

    public static string TextOutlines(string text, double originX, double originY, string family, double emSize, int styleFlags)
    {
        var font = new SketchFontSpec(
            string.IsNullOrWhiteSpace(family) ? "Arial" : family,
            emSize > 0 ? emSize : 100.0,
            (SketchFontStyleFlags)styleFlags);
        var contours = SketchTextProvider.Instance.GenerateAt(text ?? string.Empty, font, new Vec2D(originX, originY));
        double tol = Math.Max((emSize > 0 ? emSize : 100.0) * 0.01, 1e-6);
        var loops = new List<List<Vec2D>>();
        if (contours != null)
        {
            foreach (var contour in contours)
            {
                var loop = new List<Vec2D>();
                foreach (var curve in contour)
                {
                    var verts = curve.Tessellate(tol);
                    for (int i = 0; i < verts.Count; i++)
                    {
                        if (loop.Count > 0)
                        {
                            Vec2D last = loop[loop.Count - 1];
                            Vec2D p = verts[i].Position;
                            if ((p - last).LengthSquared() <= 1e-24)
                                continue;
                        }
                        loop.Add(verts[i].Position);
                    }
                }
                NativePack.DropClosingDuplicate(loop);
                if (loop.Count >= 3)
                    loops.Add(loop);
            }
        }
        return NativePack.WriteLoops2(loops);
    }
}

internal static class NativeLattice
{
    const int DefaultSlices = 1_000_000;

    public static string PackVolume(CoordinateConverter conv)
    {
        Box3D b = conv.OperatingSpace;
        int slices = conv.IntegerBoundingBox.Max.X - conv.IntegerBoundingBox.Min.X + 1;
        var inv = CultureInfo.InvariantCulture;
        return string.Format(inv, "{0}\n{1:G17} {2:G17} {3:G17}\n{4:G17} {5:G17} {6:G17}\n",
            slices, b.Min.X, b.Min.Y, b.Min.Z, b.Max.X, b.Max.Y, b.Max.Z);
    }

    public static CoordinateConverter ConverterFor(IList<Vec2D> points, string packedVolume)
    {
        int slices;
        Box3D box;
        bool auto;
        ParseVolume(packedVolume, out auto, out slices, out box);
        if (auto)
            box = AutoBox(points, slices);
        else
            EnsurePositiveExtent(ref box, slices);
        return new CoordinateConverter(box, slices);
    }

    public static List<Rat2Hybrid> ToRat2(CoordinateConverter conv, IList<Vec2D> points)
    {
        var exact = new List<Rat2Hybrid>(points.Count);
        for (int i = 0; i < points.Count; i++)
        {
            Rat2Hybrid r;
            if (!TryToRat2(conv, points[i], out r))
                throw new OverflowException("Point is outside the working volume lattice.");
            exact.Add(r);
        }
        return exact;
    }

    public static bool TryToRat2(CoordinateConverter conv, Vec2D p, out Rat2Hybrid r)
    {
        r = default;
        try
        {
            Int3 i = conv.Convert(new Vec3D(p.X, p.Y, conv.OperatingSpace.Min.Z));
            var r3 = new Rat3Hybrid(i.X, i.Y, i.Z);
            r = new Rat2Hybrid(r3.X, r3.Y);
            return true;
        }
        catch (OverflowException)
        {
            return false;
        }
    }

    static void ParseVolume(string packed, out bool auto, out int slices, out Box3D box)
    {
        auto = true;
        slices = DefaultSlices;
        box = default;
        if (string.IsNullOrWhiteSpace(packed))
            return;
        string[] lines = packed.Replace('\r', '\n').Split('\n');
        int i = 0;
        while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
            i++;
        if (i >= lines.Length)
            return;
        string first = lines[i].Trim();
        var inv = CultureInfo.InvariantCulture;
        if (first.StartsWith("auto", StringComparison.OrdinalIgnoreCase))
        {
            string[] tok = first.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            if (tok.Length > 1)
            {
                int n = int.Parse(tok[1], inv);
                if (n >= 2)
                    slices = n;
            }
            auto = true;
            return;
        }
        auto = false;
        slices = int.Parse(first, inv);
        if (slices < 2)
            slices = DefaultSlices;
        i++;
        while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
            i++;
        Vec3D min = ParseVec3(lines[i++], inv);
        while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
            i++;
        Vec3D max = ParseVec3(lines[i], inv);
        box = new Box3D(min, max, 0);
    }

    static Vec3D ParseVec3(string line, CultureInfo inv)
    {
        string[] p = line.Trim().Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
        double x = double.Parse(p[0], inv);
        double y = double.Parse(p[1], inv);
        double z = p.Length > 2 ? double.Parse(p[2], inv) : 0;
        return new Vec3D(x, y, z);
    }

    static Box3D AutoBox(IList<Vec2D> points, int slices)
    {
        Box3D box = Box3D.Empty;
        if (points != null)
        {
            for (int i = 0; i < points.Count; i++)
                box.IncludePoint(new Vec3D(points[i].X, points[i].Y, 0));
        }
        EnsurePositiveExtent(ref box, slices);
        return box;
    }

    static void EnsurePositiveExtent(ref Box3D box, int slices)
    {
        if (double.IsInfinity(box.Min.X) || box.Min.X > box.Max.X)
        {
            box = new Box3D(new Vec3D(-1), new Vec3D(1), 0);
            return;
        }
        double dx = box.Max.X - box.Min.X;
        double dy = box.Max.Y - box.Min.Y;
        double dz = box.Max.Z - box.Min.Z;
        double span = Math.Max(dx, Math.Max(dy, dz));
        if (span < 1e-30)
        {
            Vec3D c = new Vec3D(
                0.5 * (box.Min.X + box.Max.X),
                0.5 * (box.Min.Y + box.Max.Y),
                0.5 * (box.Min.Z + box.Max.Z));
            if (double.IsNaN(c.X) || double.IsInfinity(c.X))
                c = new Vec3D(0, 0, 0);
            box = new Box3D(c + new Vec3D(-1, -1, -1), c + new Vec3D(1, 1, 1), 0);
            return;
        }
        double minExtent = span / Math.Max(slices, 2);
        if (dx < minExtent)
        {
            double m = 0.5 * (box.Min.X + box.Max.X);
            box.Min.X = m - 0.5 * span;
            box.Max.X = m + 0.5 * span;
        }
        if (dy < minExtent)
        {
            double m = 0.5 * (box.Min.Y + box.Max.Y);
            box.Min.Y = m - 0.5 * span;
            box.Max.Y = m + 0.5 * span;
        }
        if (dz < minExtent)
        {
            double m = 0.5 * (box.Min.Z + box.Max.Z);
            box.Min.Z = m - 0.5 * span;
            box.Max.Z = m + 0.5 * span;
        }
    }
}

internal static class NativePack
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static List<List<Vec2D>> ReadLoops2(string packed)
    {
        var loops = new List<List<Vec2D>>();
        if (string.IsNullOrWhiteSpace(packed))
            return loops;
        string[] lines = packed.Replace('\r', '\n').Split('\n');
        int i = 0;
        while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
            i++;
        if (i >= lines.Length)
            return loops;
        int nLoops = int.Parse(lines[i++].Trim(), Inv);
        for (int l = 0; l < nLoops; l++)
        {
            while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
                i++;
            int n = int.Parse(lines[i++].Trim(), Inv);
            var loop = new List<Vec2D>(n);
            for (int k = 0; k < n; k++)
            {
                while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
                    i++;
                loop.Add(ParseVec2(lines[i++]));
            }
            DropClosingDuplicate(loop);
            loops.Add(loop);
        }
        return loops;
    }

    public static List<Vec2D> ReadFirstLoop2(string packed)
    {
        var loops = ReadLoops2(packed);
        if (loops.Count == 0)
            return ReadPoints2(packed);
        return loops[0];
    }

    public static List<Vec2D> ReadPoints2(string packed)
    {
        var pts = new List<Vec2D>();
        if (string.IsNullOrWhiteSpace(packed))
            return pts;
        string[] lines = packed.Replace('\r', '\n').Split('\n');
        int i = 0;
        while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
            i++;
        if (i >= lines.Length)
            return pts;
        int n = int.Parse(lines[i++].Trim(), Inv);
        for (int k = 0; k < n; k++)
        {
            while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
                i++;
            pts.Add(ParseVec2(lines[i++]));
        }
        return pts;
    }

    public static List<Vec3D> ReadPoints3(string packed)
    {
        var pts = new List<Vec3D>();
        if (string.IsNullOrWhiteSpace(packed))
            return pts;
        string[] lines = packed.Replace('\r', '\n').Split('\n');
        int i = 0;
        while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
            i++;
        int n = int.Parse(lines[i++].Trim(), Inv);
        for (int k = 0; k < n; k++)
        {
            while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
                i++;
            pts.Add(ParseVec3(lines[i++]));
        }
        return pts;
    }

    public static List<Tri> ReadTriangles(string packed)
    {
        var tris = new List<Tri>();
        if (string.IsNullOrWhiteSpace(packed))
            return tris;
        string[] lines = packed.Replace('\r', '\n').Split('\n');
        int i = 0;
        while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
            i++;
        int n = int.Parse(lines[i++].Trim(), Inv);
        for (int k = 0; k < n; k++)
        {
            while (i < lines.Length && string.IsNullOrWhiteSpace(lines[i]))
                i++;
            string[] p = lines[i++].Trim().Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            tris.Add(new Tri(int.Parse(p[0], Inv), int.Parse(p[1], Inv), int.Parse(p[2], Inv)));
        }
        return tris;
    }

    public static string WriteLoops2(List<List<Vec2D>> loops)
    {
        var sb = new StringBuilder();
        sb.Append(loops.Count.ToString(Inv));
        sb.Append('\n');
        for (int i = 0; i < loops.Count; i++)
        {
            var loop = loops[i];
            sb.Append(loop.Count.ToString(Inv));
            sb.Append('\n');
            for (int k = 0; k < loop.Count; k++)
            {
                Append2(sb, loop[k]);
                sb.Append('\n');
            }
        }
        return sb.ToString();
    }

    public static string WriteIndexed2(List<Vec2D> points, List<Tri> triangles)
    {
        var sb = new StringBuilder();
        sb.Append(points.Count.ToString(Inv));
        sb.Append('\n');
        for (int i = 0; i < points.Count; i++)
        {
            Append2(sb, points[i]);
            sb.Append('\n');
        }
        sb.Append(triangles.Count.ToString(Inv));
        sb.Append('\n');
        for (int i = 0; i < triangles.Count; i++)
        {
            Tri t = triangles[i];
            sb.Append(t.A.ToString(Inv));
            sb.Append(' ');
            sb.Append(t.B.ToString(Inv));
            sb.Append(' ');
            sb.Append(t.C.ToString(Inv));
            sb.Append('\n');
        }
        return sb.ToString();
    }

    public static string WriteIndexed3(List<Vec3D> points, List<Tri> triangles)
    {
        var sb = new StringBuilder();
        sb.Append(points.Count.ToString(Inv));
        sb.Append('\n');
        for (int i = 0; i < points.Count; i++)
        {
            Append3(sb, points[i]);
            sb.Append('\n');
        }
        sb.Append(triangles.Count.ToString(Inv));
        sb.Append('\n');
        for (int i = 0; i < triangles.Count; i++)
        {
            Tri t = triangles[i];
            sb.Append(t.A.ToString(Inv));
            sb.Append(' ');
            sb.Append(t.B.ToString(Inv));
            sb.Append(' ');
            sb.Append(t.C.ToString(Inv));
            sb.Append('\n');
        }
        return sb.ToString();
    }

    public static string WritePoints2(List<Vec2D> points)
    {
        var sb = new StringBuilder();
        sb.Append(points.Count.ToString(Inv));
        sb.Append('\n');
        for (int i = 0; i < points.Count; i++)
        {
            Append2(sb, points[i]);
            sb.Append('\n');
        }
        return sb.ToString();
    }

    public static double Shoelace(List<Vec2D> loop)
    {
        if (loop == null || loop.Count < 3)
            return 0;
        double sum = 0;
        for (int i = 0; i < loop.Count; i++)
        {
            Vec2D a = loop[i];
            Vec2D b = loop[(i + 1) % loop.Count];
            sum += a.X * b.Y - b.X * a.Y;
        }
        return 0.5 * sum;
    }

    public static void DropClosingDuplicate(List<Vec2D> loop)
    {
        if (loop == null || loop.Count < 2)
            return;
        Vec2D a = loop[0];
        Vec2D b = loop[loop.Count - 1];
        if ((a - b).LengthSquared() <= 1e-24)
            loop.RemoveAt(loop.Count - 1);
    }

    static Vec2D ParseVec2(string line)
    {
        string[] p = line.Trim().Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
        return new Vec2D(double.Parse(p[0], Inv), double.Parse(p[1], Inv));
    }

    static Vec3D ParseVec3(string line)
    {
        string[] p = line.Trim().Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
        return new Vec3D(double.Parse(p[0], Inv), double.Parse(p[1], Inv), double.Parse(p[2], Inv));
    }

    static void Append2(StringBuilder sb, Vec2D p)
    {
        sb.Append(p.X.ToString("G17", Inv));
        sb.Append(' ');
        sb.Append(p.Y.ToString("G17", Inv));
    }

    static void Append3(StringBuilder sb, Vec3D p)
    {
        sb.Append(p.X.ToString("G17", Inv));
        sb.Append(' ');
        sb.Append(p.Y.ToString("G17", Inv));
        sb.Append(' ');
        sb.Append(p.Z.ToString("G17", Inv));
    }
}
