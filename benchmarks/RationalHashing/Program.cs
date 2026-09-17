using System.Diagnostics;
using System.Numerics;
using System.Text.Json;
using GeoCore;
using Geo;
using CSG;

var mode = args.Length > 0 ? args[0] : "all";
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
    var times=new List<double>(); var allocations=new List<long>(); long checksum=0;
    for(int i=0;i<5;i++)
    {
        GC.Collect();GC.WaitForPendingFinalizers();GC.Collect();
        long bytes=GC.GetTotalAllocatedBytes(true);
        var timer=Stopwatch.StartNew();checksum=action();timer.Stop();
        times.Add(timer.Elapsed.TotalMilliseconds);allocations.Add(GC.GetTotalAllocatedBytes(true)-bytes);
    }
    Console.WriteLine(JsonSerializer.Serialize(new{name,ms=times,bytes=allocations,checksum}));
}
