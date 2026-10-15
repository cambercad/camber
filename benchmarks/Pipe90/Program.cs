using System.Diagnostics;
using System.Security.Cryptography;
using System.Text;
using CSG;
using Curves;
using Geo;
using GeoCore;

// Same dimensions, tolerance and feature sequence as examples/standalone/pipe.py.
// Run: dotnet run --project benchmarks/Pipe90 -c Release
int runs = args.Length == 0 ? 1 : int.Parse(args[0]);
if (Environment.GetEnvironmentVariable("PIPE90_ALLOC") == "1") AllocListener.Start();
for (int run = 0; run < runs; run++)
{
    GeoAPI.Clear();
    Console.WriteLine($"run={run + 1}");
    Run();
}
AllocListener.Report();

static void Run()
{
    var total = Stopwatch.StartNew();
    var stage = Stopwatch.StartNew();
    var api = new GeoAPI(new Box3D(new Vec3D(-1000), new Vec3D(1000)), .1);
    const string name = "pipe";
    var guide = api.GetPlotterSketcher(DefaultPlanes.OriginXY, "guide");
    guide.AddLine(new Vec2D(0, 0), new Vec2D(-150, 0)).Name = "h_line";
    double angle = 5 * Math.PI / 4;
    guide.AddArc(new Vec2D(-150, 0), new Vec2D(-150 + 260 * Math.Cos(angle), 260 + 260 * Math.Sin(angle)),
        new Vec2D(-410, 260)).Name = "bend";
    guide.AddLine(new Vec2D(-410, 260), new Vec2D(-410, 410)).Name = "v_line";
    var profile = api.GetPlotterSketcher(DefaultPlanes.OriginYZ, "annulus");
    profile.AddCircle(new Vec2D(0, 0), 145).Name = "outer";
    profile.AddCircle(new Vec2D(0, 0), 105).Name = "inner";
    var pipe = api.ExtrudeAlongSketch(profile, guide, name: name);
    Stage("sweep", stage, pipe);

    var bottom = api.GetPlotterSketcher("pipe-ExtrudeBottom", "bottom_profile");
    var top = api.GetPlotterSketcher("pipe-ExtrudeTop", "top_profile");
    AddFlange(bottom);
    AddFlange(top);
    var bottomSolid = api.ExtrudeTwoSides(bottom, 0, 50, name: "bottom_flange");
    var topSolid = api.ExtrudeTwoSides(top, 50, 0, name: "top_flange");
    Stage("flange_extrude", stage, bottomSolid);
    pipe = api.Boolean(pipe, bottomSolid, BooleanOp.Union, name);
    pipe = api.Boolean(pipe, topSolid, BooleanOp.Union, name);
    Stage("flange_union", stage, pipe);
    var edges = new List<string>();
    foreach (string flange in new[] { "bottom_flange", "top_flange" })
    {
        edges.Add($"[{flange}-south,{flange}-west]");
        edges.Add($"[{flange}-south,{flange}-east]");
        edges.Add($"[{flange}-east,{flange}-north]");
        edges.Add($"[{flange}-north,{flange}-west]");
    }
    pipe = api.Fillet(pipe, edges, 20, name: name);
    Stage("fillet", stage, pipe);
    Fingerprint("fillet", pipe);
    Stage("fillet_fingerprint", stage, pipe);

    CutBolts(bottom.CoordinateSystem, "bottom_counterbores", 30, 10, true);
    CutBolts(bottom.CoordinateSystem, "bottom_holes", 15, 50, true);
    CutBolts(top.CoordinateSystem, "top_counterbores", 30, 10, false);
    CutBolts(top.CoordinateSystem, "top_holes", 15, 50, false);
    Stage("bolt_cuts", stage, pipe);
    pipe = api.Chamfer(pipe, new List<string>
    {
        "[pipe-outer-h_line,bottom_flange-ExtrudeBottom]",
        "[pipe-outer-v_line,top_flange-ExtrudeTop]"
    }, 10, name: name);
    Stage("chamfer", stage, pipe);
    Fingerprint("chamfer", pipe);
    Stage("chamfer_fingerprint", stage, pipe);
    Console.WriteLine($"total_seconds={total.Elapsed.TotalSeconds:F3}");
    Console.WriteLine($"gc gen0={GC.CollectionCount(0)} gen1={GC.CollectionCount(1)} gen2={GC.CollectionCount(2)} pause_s={GC.GetTotalPauseDuration().TotalSeconds:F2} alloc_gb={GC.GetTotalAllocatedBytes() / 1e9:F2}");

    void CutBolts(CoordinateSystem frame, string sketchName, double radius, double depth, bool isBottom)
    {
        var sketch = api.GetPlotterSketcher(frame, sketchName);
        foreach (var p in new[] { (-130, -130), (130, -130), (130, 130), (-130, 130) })
            sketch.AddCircle(new Vec2D(p.Item1, p.Item2), radius);
        var cutter = api.ExtrudeTwoSides(sketch, isBottom ? 0 : depth, isBottom ? depth : 0,
            name: sketchName + "_cutter");
        pipe = api.Boolean(pipe, cutter, BooleanOp.Subtract, name);
    }
}

static void Stage(string name, Stopwatch stage, AnchorMesh body)
{
    Console.WriteLine($"{name}_seconds={stage.Elapsed.TotalSeconds:F3} triangles={body.Mesh.Triangles.Count}");
    stage.Restart();
}

static void AddFlange(PlotterSketcherCoordSys sketch)
{
    sketch.AddLine(new Vec2D(-180, -180), new Vec2D(180, -180)).Name = "south";
    sketch.AddLine(new Vec2D(180, -180), new Vec2D(180, 180)).Name = "east";
    sketch.AddLine(new Vec2D(180, 180), new Vec2D(-180, 180)).Name = "north";
    sketch.AddLine(new Vec2D(-180, 180), new Vec2D(-180, -180)).Name = "west";
    sketch.AddCircle(new Vec2D(0, 0), 125).Name = "clearance";
}

static void Fingerprint(string stage, AnchorMesh body)
{
    // Ignore triangle/vertex indexing and cyclic corner order, but retain winding,
    // exact positions, display positions, UV/normal bits and patch names.
    var mesh = body.Mesh;
    var rows = new string[mesh.Triangles.Count];
    string Bits(double v) => BitConverter.DoubleToInt64Bits(v).ToString("X16");
    string Vertex(int index, TriangleVertexNormalUV attr)
    {
        var p = mesh.PrecisionPositions[index];
        var d = mesh.Positions[index];
        return $"{p.X}|{p.Y}|{p.Z}|{Bits(d.X)}|{Bits(d.Y)}|{Bits(d.Z)}|" +
            $"{Bits(attr.Normal.X)}|{Bits(attr.Normal.Y)}|{Bits(attr.Normal.Z)}|{Bits(attr.UV.X)}|{Bits(attr.UV.Y)}";
    }
    for (int i = 0; i < rows.Length; i++)
    {
        var tri = mesh.Triangles[i];
        var attr = mesh.TrianglesEx[i];
        string a = Vertex(tri.A, attr.V0), b = Vertex(tri.B, attr.V1), c = Vertex(tri.C, attr.V2);
        string[] rotations = { a + ";" + b + ";" + c, b + ";" + c + ";" + a, c + ";" + a + ";" + b };
        Array.Sort(rotations, StringComparer.Ordinal);
        rows[i] = body.groupIdToExtendedName[attr.GroupId] + ":" + rotations[0];
    }
    Array.Sort(rows, StringComparer.Ordinal);
    var hash = SHA256.HashData(Encoding.UTF8.GetBytes(string.Join("\n", rows)));
    Console.WriteLine($"{stage}_triangles={rows.Length} sha256={Convert.ToHexString(hash)}");
}
