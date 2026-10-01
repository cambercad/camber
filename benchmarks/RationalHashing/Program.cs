using System.Diagnostics;
using System.Numerics;
using System.Text.Json;
using GeoCore;
using Geo;
using CSG;

var mode = args.Length > 0 ? args[0] : "all";
if (mode == "hex_bolt_once")
{
    long triangles = RunHexBolt();
    using var process = Process.GetCurrentProcess();
    Console.WriteLine(JsonSerializer.Serialize(new { name = "hex_bolt_once", triangles,
        peakWorkingSetBytes = process.PeakWorkingSet64,
        currentWorkingSetBytes = process.WorkingSet64,
        liveManagedBytes = GC.GetTotalMemory(false) }));
    return;
}
if (mode == "hex_bolt")
{
    Measure("hex_bolt", () => RunHexBolt());
    return;
}
if (mode == "hex_bolt_verify")
{
    for (int i = 0; i < 3; i++) RunHexBolt(verify: true);
    return;
}
var values = new BigRationalHybrid[1024];
var normalized = new BigRationalHybrid[1024];
var integers = new BigRationalHybrid[1024];
for (int i = 0; i < values.Length; i++)
{
    var n = (BigInteger.One << 128) + 2 * i + 1;
    var d = (BigInteger.One << 96) + 17;
    values[i] = new BigRationalHybrid(n * 21, d * 21);
    normalized[i] = new BigRationalHybrid(values[i]);
    normalized[i].Simplify();
    integers[i] = new BigRationalHybrid(i);
}
var points = new Rat3Hybrid[4096];
for (int i = 0; i < points.Length; i++)
{
    int j = i % 2048;
    points[i] = new Rat3Hybrid(new BigRationalHybrid((BigInteger)j * 21, 997 * 21),
        new BigRationalHybrid((BigInteger)j * 14, 991 * 14), new BigRationalHybrid(j % 7));
}
if (mode != "construction")
{
    Measure("integer_hash_2m", () => Hash(integers, 2000));
    Measure("normalized_hash_1m", () => Hash(normalized, 1000));
    Measure("unreduced_hash_100k", () => Hash(values, 100));
    Measure("dedup_4096_points_x30", () => { long n=0; for(int i=0;i<30;i++) n+=DuplicatePointRemover.DuplicateMap(points).Count; return n; });
    Measure("new_point_creator_4096_x10", () => { long n=0; for(int i=0;i<10;i++){var creator=new NewPointCreator();foreach(var p in points)n+=creator.GetIndex(new Rat3Hybrid(in p));}return n; });
}
if (mode != "micro") Measure("cylinder_boolean_x8", () =>
{
    long count = 0;
    for (int i = 0; i < 8; i++)
    {
        GeoAPI.Clear();
        var api = new GeoAPI(new Box3D(new Vec3D(-30),new Vec3D(30)),.1);
        var a=api.CreateCylinder(new CoordinateSystem(new Vec3D(0)),10,15,.1,"a");
        var b=api.CreateCylinder(new CoordinateSystem(new Vec3D(4,1,3)),8,15,.1,"b");
        var result=api.Boolean(a,b,BooleanOp.Union,"combined");
        if(!MeshAnalysis.IsWatertightMesh(result.Mesh.PrecisionPositions,result.Mesh.Triangles,true))throw new Exception("Not watertight");
        count+=result.Mesh.Triangles.Count;
    }
    return count;
});
static long Hash(BigRationalHybrid[] values,int repeats)
{
    long n=0;for(int r=0;r<repeats;r++)foreach(var v in values)n+=v.GetHashCode();return n;
}
static void Measure(string name,Func<long> action)
{
    action(); action();
    var times=new List<double>(); var allocations=new List<long>(); var checksums=new List<long>(); long checksum=0;
    for(int i=0;i<5;i++)
    {
        GC.Collect();GC.WaitForPendingFinalizers();GC.Collect();
        long bytes=GC.GetTotalAllocatedBytes(true);
        var timer=Stopwatch.StartNew();checksum=action();timer.Stop();
        times.Add(timer.Elapsed.TotalMilliseconds);allocations.Add(GC.GetTotalAllocatedBytes(true)-bytes);checksums.Add(checksum);
    }
    Console.WriteLine(JsonSerializer.Serialize(new{name,ms=times,bytes=allocations,checksum,checksums}));
}

// C# equivalent of Geo.Python/python/examples/standalone/hex_bolt.py (M10 x 35 mm).
static long RunHexBolt(bool verify = false)
{
    const double mm = 0.001, deviation = 0.000001;
    const double length = 0.035, diameter = 0.010, headHeight = 0.0064, flats = 0.016;
    GeoAPI.Clear();
    var part = new GeoAPI(new Box3D(new Vec3D(-0.5), new Vec3D(0.5)), deviation);
    double shankRadius = 0.5 * diameter + 0.5 * mm;
    double top = length + headHeight, circumRadius = flats / Math.Sqrt(3);
    double overlap = Math.Clamp(10 * deviation, 0.05 * mm, 0.2 * mm);
    overlap = Math.Min(overlap, 0.025 * Math.Min(length, headHeight));
    var shank = part.CreateCylinder(new CoordinateSystem(new Vec3D(0)), shankRadius, length + overlap, deviation, "bolt_shank");
    double washerHeight = Math.Max(0.22 * diameter, 0.55 * mm);
    var washer = part.CreateCylinder(new CoordinateSystem(new Vec3D(0, 0, length - washerHeight)),
        0.25 * flats, washerHeight + overlap, deviation, "bolt_washer");
    var hex = part.GetPlotterSketcher(new CoordinateSystem(new Vec3D(0, 0, length - overlap)), "bolt_head_profile");
    for (int i = 0; i < 6; i++)
    {
        double a = i * Math.PI / 3, b = ((i + 1) % 6) * Math.PI / 3;
        hex.AddLine(new Vec2D(circumRadius * Math.Cos(a), circumRadius * Math.Sin(a)),
            new Vec2D(circumRadius * Math.Cos(b), circumRadius * Math.Sin(b)));
    }
    var head = part.Extrude(hex, headHeight + overlap, deviation, "bolt_head");
    var body = part.Boolean(part.Boolean(shank, washer, BooleanOp.Union, "bolt_u1"), head, BooleanOp.Union, "bolt_u2");
    var meridian = new CoordinateSystem(new Vec3D(0), new Vec3D(0, 0, 1),
        new Vec3D(1, 0, 0), new Vec3D(0, 1, 0));
    double flatTop = 0.5 * flats * 0.96;
    double chamferDepth = Math.Clamp(0.32 * headHeight, 0.12 * headHeight, 0.85 * headHeight);
    double outer = circumRadius + Math.Max(0.6 * mm, 0.04 * flats);
    double cutterTop = top + Math.Max(0.35 * headHeight, 0.5 * mm);
    var topSketch = part.GetPlotterSketcher(meridian, "bolt_top_chamfer_cutter");
    AddPolygon(topSketch, new Vec2D(cutterTop, 0), new Vec2D(cutterTop, outer),
        new Vec2D(top - chamferDepth, outer), new Vec2D(top, flatTop), new Vec2D(top, 0));
    var withChamfer = part.Boolean(body,
        part.Revolve(topSketch, 2 * Math.PI, deviation, "bolt_topChamferCut"), BooleanOp.Subtract, "bolt_chamfer");
    double tipAxial = Math.Clamp(0.55 * diameter, 0.6 * mm, 2.5 * mm);
    if (tipAxial >= shankRadius) tipAxial = 0.45 * shankRadius;
    double tipClear = shankRadius + Math.Max(0.5 * mm, 0.05 * diameter);
    double tipLow = -Math.Max(0.15 * mm, 0.04 * tipAxial);
    var tipSketch = part.GetPlotterSketcher(meridian, "bolt_tip_chamfer_cutter");
    AddPolygon(tipSketch, new Vec2D(tipLow, 0), new Vec2D(tipLow, tipClear),
        new Vec2D(tipAxial, tipClear), new Vec2D(tipAxial, shankRadius),
        new Vec2D(0, shankRadius - tipAxial), new Vec2D(0, 0));
    var withTip = part.Boolean(withChamfer,
        part.Revolve(tipSketch, 2 * Math.PI, deviation, "bolt_tipChamferCut"), BooleanOp.Subtract, "bolt_tipChamfer");
    var threadAxis = new CoordinateSystem(new Vec3D(0, 0, length),
        new Vec3D(0, 1, 0), new Vec3D(1, 0, 0), new Vec3D(0, 0, -1));
    var thread = part.CreateMetricThreadForBoltNegative(threadAxis, diameter, 0.0015, length,
        deviation, true, "bolt_threadNeg", shankRadius + 0.002);
    var bolt = part.Boolean(withTip, thread, BooleanOp.Subtract, "bolt");
    if (verify)
    {
        var mesh = bolt.Mesh;
        var (triangleHashXor, triangleHashSum) = MeshSignature(mesh.Positions, mesh.Triangles);
        Console.WriteLine(JsonSerializer.Serialize(new { triangles = mesh.Triangles.Count,
            watertight = MeshAnalysis.IsWatertightMesh(mesh.PrecisionPositions, mesh.Triangles, true),
            volume = MeshAnalysis.ComputeSignedMeshVolume(mesh.Positions, mesh.Triangles),
            triangleHashXor, triangleHashSum }));
    }
    return bolt.Mesh.Triangles.Count;
}

static (ulong xor, ulong sum) MeshSignature(IList<Vec3D> points, IList<Tri> triangles)
{
    ulong xor = 0, sum = 0;
    foreach (var triangle in triangles)
    {
        Vec3D a = points[triangle.A], b = points[triangle.B], c = points[triangle.C];
        if (Compare(a, b) > 0) (a, b) = (b, a);
        if (Compare(b, c) > 0) (b, c) = (c, b);
        if (Compare(a, b) > 0) (a, b) = (b, a);
        ulong hash = 14695981039346656037UL;
        HashPoint(ref hash, a);
        HashPoint(ref hash, b);
        HashPoint(ref hash, c);
        xor ^= hash;
        sum = unchecked(sum + hash);
    }
    return (xor, sum);
}

static int Compare(Vec3D a, Vec3D b)
{
    int x = a.X.CompareTo(b.X);
    if (x != 0) return x;
    int y = a.Y.CompareTo(b.Y);
    return y != 0 ? y : a.Z.CompareTo(b.Z);
}

static void HashPoint(ref ulong hash, Vec3D point)
{
    hash = unchecked((hash ^ (ulong)BitConverter.DoubleToInt64Bits(point.X)) * 1099511628211UL);
    hash = unchecked((hash ^ (ulong)BitConverter.DoubleToInt64Bits(point.Y)) * 1099511628211UL);
    hash = unchecked((hash ^ (ulong)BitConverter.DoubleToInt64Bits(point.Z)) * 1099511628211UL);
}

static void AddPolygon(Curves.PlotterSketcherCoordSys sketch, params Vec2D[] vertices)
{
    for (int i = 0; i < vertices.Length; i++)
        sketch.AddLine(vertices[i], vertices[(i + 1) % vertices.Length]);
}
